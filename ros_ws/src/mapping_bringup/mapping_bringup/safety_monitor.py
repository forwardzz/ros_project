import math
import os
import re
import subprocess
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from robot_monitor_interfaces.msg import RobotSafetyStatus


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")

        self.declare_parameter("throttled_path", "/sys/devices/platform/soc/soc:firmware/get_throttled")
        self.declare_parameter("vcgencmd_path", "/usr/bin/vcgencmd")
        self.declare_parameter("rpi_volt_alarm_path", "")
        self.declare_parameter("fault_on_undervoltage_seen", False)
        self.declare_parameter("obstacle_fault_window_sec", 10.0)
        self.declare_parameter("obstacle_fault_count", 2)
        self.declare_parameter("progress_fault_window_sec", 20.0)
        self.declare_parameter("progress_fault_count", 2)
        self.declare_parameter("poll_period_sec", 1.0)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_pub = self.create_publisher(RobotSafetyStatus, "/robot_safety_status", latched_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.abort_client = self.create_client(Trigger, "/abort_mission")
        self.create_service(Trigger, "/reset_safety_monitor", self._handle_reset)

        self.create_subscription(Log, "/rosout", self._rosout_cb, 50)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_through_poses/_action/status",
            self._status_cb,
            10,
        )

        self.collision_events = deque()
        self.progress_events = deque()
        self.fault_active = False
        self.level = "SAFE"
        self.code = "READY"
        self.message = "Safety monitor online"
        self.mission_active = False
        self.abort_requested = False
        self.last_fault_time = 0.0
        self.voltage_available = False
        self.measured_voltage_v = math.nan
        self.undervoltage_now = False
        self.undervoltage_seen = False
        self.throttled_flags = 0
        self.power_monitor_available = False
        self._warned_power_monitor_unavailable = False

        poll_period = float(self.get_parameter("poll_period_sec").value)
        self.timer = self.create_timer(poll_period, self._poll)
        self._publish_status()
        self.get_logger().info("Safety monitor ready")

    def _status_cb(self, msg):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self.mission_active = any(status.status in active_states for status in msg.status_list)
        if not self.mission_active:
            self.abort_requested = False

    def _rosout_cb(self, msg):
        name = getattr(msg, "name", "")
        if "controller_server" not in name:
            return

        text = msg.msg.lower()
        now = time.monotonic()
        if "detected collision ahead" in text:
            self.collision_events.append(now)
            self._trim_events(self.collision_events, float(self.get_parameter("obstacle_fault_window_sec").value), now)
            if len(self.collision_events) >= int(self.get_parameter("obstacle_fault_count").value):
                self._trigger_fault(
                    "COLLISION_LOOP",
                    "Navigation detected repeated collisions ahead. Mission aborted and robot stopped.",
                )
        elif "failed to make progress" in text:
            self.progress_events.append(now)
            self._trim_events(self.progress_events, float(self.get_parameter("progress_fault_window_sec").value), now)
            if len(self.progress_events) >= int(self.get_parameter("progress_fault_count").value):
                self._trigger_fault(
                    "NO_PROGRESS",
                    "Navigation made no progress repeatedly. Mission aborted and robot stopped.",
                )

    def _poll(self):
        now = time.monotonic()
        self._trim_events(self.collision_events, float(self.get_parameter("obstacle_fault_window_sec").value), now)
        self._trim_events(self.progress_events, float(self.get_parameter("progress_fault_window_sec").value), now)
        self._update_power_status()

        fault_on_seen = bool(self.get_parameter("fault_on_undervoltage_seen").value)
        if self.undervoltage_now or (fault_on_seen and self.mission_active and self.undervoltage_seen):
            self._trigger_fault(
                "UNDERVOLTAGE",
                "Raspberry Pi undervoltage detected. Mission aborted to protect the board and power supply.",
            )

        if self.fault_active:
            self._publish_zero_cmd()
        elif self.level != "SAFE":
            self.level = "SAFE"
            self.code = "READY"
            self.message = "Safety monitor online"

        self._publish_status()

    def _update_power_status(self):
        throttled_path = str(self.get_parameter("throttled_path").value)
        value = self._read_vcgencmd_throttled(str(self.get_parameter("vcgencmd_path").value))
        if value is None:
            value = self._read_throttled_value(throttled_path)

        alarm = self._read_rpi_volt_alarm()
        self.power_monitor_available = value is not None or alarm is not None

        if value is not None:
            self.throttled_flags = int(value)
            self.undervoltage_now = bool(value & 0x1)
            self.undervoltage_seen = self.undervoltage_seen or bool(value & 0x10000)
        else:
            self.throttled_flags = 0
            self.undervoltage_now = False

        if alarm is not None and alarm:
            self.undervoltage_now = True
            self.undervoltage_seen = True

        # vcgencmd measure_volts is core voltage on Raspberry Pi, not the 5V input rail.
        # Keep this unavailable unless a real board-voltage ADC is added later.
        self.voltage_available = False
        self.measured_voltage_v = math.nan

        if not self.power_monitor_available and not self._warned_power_monitor_unavailable:
            self._warned_power_monitor_unavailable = True
            self.get_logger().warn(
                "No usable Raspberry Pi power monitor found. "
                "Check /dev/vcio permissions or rpi_volt hwmon support.")

    def _read_throttled_value(self, path):
        try:
            if not os.path.exists(path):
                return None
            with open(path, "r", encoding="utf-8") as handle:
                return int(handle.read().strip(), 0)
        except Exception:
            return None

    def _read_vcgencmd_throttled(self, command_path):
        try:
            output = subprocess.check_output([command_path, "get_throttled"], text=True, timeout=0.8).strip()
        except Exception:
            return None
        if "=" not in output:
            return None
        try:
            return int(output.split("=", 1)[1], 0)
        except ValueError:
            return None

    def _read_vcgencmd_voltage(self, command_path):
        try:
            output = subprocess.check_output([command_path, "measure_volts"], text=True, timeout=0.8).strip()
        except Exception:
            return None
        match = re.search(r"=([0-9.]+)V", output)
        if not match:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    def _read_rpi_volt_alarm(self):
        configured_path = str(self.get_parameter("rpi_volt_alarm_path").value).strip()
        paths = [configured_path] if configured_path else self._discover_rpi_volt_alarm_paths()
        for path in paths:
            try:
                with open(path, "r", encoding="utf-8") as handle:
                    return int(handle.read().strip(), 0) != 0
            except Exception:
                continue
        return None

    @staticmethod
    def _discover_rpi_volt_alarm_paths():
        paths = []
        base = "/sys/class/hwmon"
        try:
            entries = sorted(os.listdir(base))
        except Exception:
            return paths
        for entry in entries:
            hwmon_dir = os.path.join(base, entry)
            name_path = os.path.join(hwmon_dir, "name")
            alarm_path = os.path.join(hwmon_dir, "in0_lcrit_alarm")
            try:
                with open(name_path, "r", encoding="utf-8") as handle:
                    name = handle.read().strip()
            except Exception:
                continue
            if name == "rpi_volt" and os.path.exists(alarm_path):
                paths.append(alarm_path)
        return paths

    def _trigger_fault(self, code, message):
        if self.fault_active and self.code == code:
            return
        self.fault_active = True
        self.level = "FAULT"
        self.code = code
        self.message = message
        self.last_fault_time = time.time()
        self.get_logger().error(message)
        self._publish_zero_cmd()
        self._publish_status()
        if not self.abort_requested:
            self.abort_requested = True
            self._request_abort()

    def _request_abort(self):
        request = Trigger.Request()
        if not self.abort_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Abort mission service is unavailable during safety fault")
            return
        future = self.abort_client.call_async(request)
        future.add_done_callback(self._abort_done)

    def _abort_done(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().warn(result.message)
        except Exception as exc:
            self.get_logger().error(f"Abort mission service failed: {exc}")

    def _handle_reset(self, _request, response):
        self._update_power_status()
        if self.undervoltage_now:
            response.success = False
            response.message = (
                "Undervoltage is active. Fix the power supply before resetting safety.")
            self.get_logger().warn(response.message)
            return response

        self.collision_events.clear()
        self.progress_events.clear()
        self.undervoltage_seen = False
        self.fault_active = False
        self.abort_requested = False
        self.level = "SAFE"
        self.code = "RESET"
        self.message = "Safety monitor reset by operator"
        self._publish_status()
        response.success = True
        response.message = self.message
        self.get_logger().info(response.message)
        return response

    def _publish_zero_cmd(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    def _publish_status(self):
        msg = RobotSafetyStatus()
        msg.level = self.level
        msg.code = self.code
        msg.message = self.message
        msg.mission_active = self.mission_active
        msg.voltage_available = self.voltage_available
        msg.measured_voltage_v = self.measured_voltage_v
        msg.undervoltage_now = self.undervoltage_now
        msg.undervoltage_seen = self.undervoltage_seen
        msg.throttled_flags = self.throttled_flags
        self.status_pub.publish(msg)

    @staticmethod
    def _trim_events(events, window_sec, now):
        while events and now - events[0] > window_sec:
            events.popleft()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
