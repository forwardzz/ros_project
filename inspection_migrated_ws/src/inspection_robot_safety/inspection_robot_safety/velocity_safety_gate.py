import fcntl
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from robot_monitor_interfaces.msg import RobotSafetyStatus
from std_msgs.msg import Bool


DEFAULT_INPUT_TIMEOUT_SEC = 0.35
DEFAULT_GATE_PUBLISH_PERIOD_SEC = 0.02
MAX_GATE_PUBLISH_PERIOD_SEC = 0.05
DEFAULT_SAFETY_STATUS_TIMEOUT_SEC = 0.30
DEFAULT_SCAN_TIMEOUT_SEC = 0.40
MAX_FAILSAFE_STOP_BUDGET_SEC = 0.50


class VelocitySafetyGate(Node):
    """Single final command publisher for autonomous and teleop Twist inputs."""

    def __init__(self):
        super().__init__("velocity_safety_gate")
        self.declare_parameter("autonomy_topic", "/cmd_vel_auto")
        self.declare_parameter("teleop_topic", "/cmd_vel_teleop")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("stop_topic", "/safety_stop")
        self.declare_parameter("input_timeout_sec", DEFAULT_INPUT_TIMEOUT_SEC)
        self.declare_parameter(
            "publish_period_sec", DEFAULT_GATE_PUBLISH_PERIOD_SEC
        )
        self.declare_parameter("max_linear_speed", 0.18)
        self.declare_parameter("max_angular_speed", 0.55)
        self.declare_parameter(
            "safety_status_timeout_sec", DEFAULT_SAFETY_STATUS_TIMEOUT_SEC
        )
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("require_fresh_scan", True)
        self.declare_parameter("scan_timeout_sec", DEFAULT_SCAN_TIMEOUT_SEC)
        self.declare_parameter("teleop_priority", True)
        self.declare_parameter(
            "gate_lock_path", "/tmp/inspection_robot_cmd_vel_gate.lock"
        )

        self.autonomy = Twist()
        self.teleop = Twist()
        self.autonomy_time = None
        self.teleop_time = None
        self.fault = True
        self.external_stop = True
        # Bound every fail-safe interval here. The gate republishes the selected
        # command, so a relaxed input timeout would otherwise defeat the motor
        # driver's independent 0.35 s command watchdog.
        self.timeout = self._bounded_parameter(
            "input_timeout_sec",
            DEFAULT_INPUT_TIMEOUT_SEC,
            0.05,
            DEFAULT_INPUT_TIMEOUT_SEC,
        )
        self.publish_period = self._bounded_parameter(
            "publish_period_sec",
            DEFAULT_GATE_PUBLISH_PERIOD_SEC,
            0.005,
            MAX_GATE_PUBLISH_PERIOD_SEC,
        )
        self.safety_status_timeout = self._bounded_parameter(
            "safety_status_timeout_sec",
            DEFAULT_SAFETY_STATUS_TIMEOUT_SEC,
            0.10,
            DEFAULT_SAFETY_STATUS_TIMEOUT_SEC,
        )
        # These are hard upper bounds at the final command boundary.  Higher
        # launch parameters must not make a deployed vehicle faster.
        self.max_linear_speed = self._bounded_parameter(
            "max_linear_speed", 0.18, 0.0, 0.18
        )
        self.max_angular_speed = self._bounded_parameter(
            "max_angular_speed", 0.55, 0.0, 0.55
        )
        self.require_fresh_scan = bool(
            self.get_parameter("require_fresh_scan").value
        )
        self.scan_timeout = self._bounded_parameter(
            "scan_timeout_sec",
            DEFAULT_SCAN_TIMEOUT_SEC,
            0.05,
            DEFAULT_SCAN_TIMEOUT_SEC,
        )
        self.teleop_priority = bool(self.get_parameter("teleop_priority").value)
        self.safety_time = None
        self.scan_time = None
        self.clock = self.get_clock()
        self._gate_lock = None
        self.output = None
        self._last_stop_reason = None

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._acquire_gate_lock()
        if self._gate_lock is not None:
            self.output = self.create_publisher(
                Twist, str(self.get_parameter("output_topic").value), 10
            )
        self.autonomy_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("autonomy_topic").value),
            self._autonomy_cb,
            10,
        )
        self.teleop_sub = self.create_subscription(
            Twist, str(self.get_parameter("teleop_topic").value), self._teleop_cb, 10
        )
        self.safety_sub = self.create_subscription(
            RobotSafetyStatus, "/robot_safety_status", self._safety_cb, latched_qos
        )
        self.stop_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("stop_topic").value),
            self._stop_cb,
            latched_qos,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_cb,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(self.publish_period, self._tick)

    def _autonomy_cb(self, msg):
        self.autonomy = msg
        self.autonomy_time = self.clock.now()

    def _teleop_cb(self, msg):
        self.teleop = msg
        self.teleop_time = self.clock.now()

    def _safety_cb(self, msg):
        self.fault = str(msg.level).upper() == "FAULT"
        self.safety_time = self.clock.now()

    def _stop_cb(self, msg):
        self.external_stop = bool(msg.data)

    def _scan_cb(self, _msg):
        first_scan = self.scan_time is None
        self.scan_time = self.clock.now()
        if first_scan:
            self.get_logger().info("First laser scan received by velocity gate")

    def _acquire_gate_lock(self):
        path = str(self.get_parameter("gate_lock_path").value)
        try:
            lock = open(path, "w", encoding="utf-8")
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            lock.write("%d\\n" % os.getpid())
            lock.flush()
            self._gate_lock = lock
        except OSError as exc:
            self.get_logger().error(
                "Another velocity safety gate owns /cmd_vel (%s); "
                "this instance will not publish: %s"
                % (path, exc)
            )
            try:
                lock.close()
            except (UnboundLocalError, OSError):
                pass

    def _fresh(self, stamp):
        if stamp is None:
            return False
        return (self.clock.now() - stamp).nanoseconds / 1e9 <= self.timeout

    def _safety_fresh(self):
        if self.safety_time is None:
            return False
        return (
            (self.clock.now() - self.safety_time).nanoseconds / 1e9
            <= self.safety_status_timeout
        )

    def _scan_fresh(self):
        if self.scan_time is None:
            return False
        return (
            (self.clock.now() - self.scan_time).nanoseconds / 1e9
            <= self.scan_timeout
        )

    def _bounded_parameter(self, name, default, lower, upper):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            value = default
        if not math.isfinite(value):
            value = default
        return max(lower, min(upper, value))

    @staticmethod
    def _zero():
        return Twist()

    def _sanitize(self, msg):
        """Enforce the physical command envelope before the GPIO boundary."""
        output = Twist()
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            return output
        output.linear.x = max(
            -self.max_linear_speed, min(self.max_linear_speed, linear_x)
        )
        output.angular.z = max(
            -self.max_angular_speed, min(self.max_angular_speed, angular_z)
        )
        return output

    def _stop_reason(self):
        if not self._safety_fresh():
            return "safety status missing or stale"
        if self.fault:
            return "safety monitor fault"
        if self.external_stop:
            return "safety stop asserted"
        if self.require_fresh_scan and not self._scan_fresh():
            return "laser scan missing or stale"
        return ""

    def _report_stop_reason(self, reason):
        if reason == self._last_stop_reason:
            return
        self._last_stop_reason = reason
        if reason:
            self.get_logger().warn("Velocity output inhibited: %s" % reason)
        else:
            self.get_logger().info("Velocity output enabled: safety inputs are fresh")

    def _tick(self):
        if self.output is None:
            return
        stop_reason = self._stop_reason()
        self._report_stop_reason(stop_reason)
        if stop_reason:
            self.output.publish(self._zero())
            return
        # Manual input is intentionally highest priority.  The GUI cancels any
        # active mission when manual control starts, so autonomous motion does
        # not resume after a short teleop watchdog interval.
        if self.teleop_priority and self._fresh(self.teleop_time):
            self.output.publish(self._sanitize(self.teleop))
        elif self._fresh(self.autonomy_time):
            self.output.publish(self._sanitize(self.autonomy))
        elif self._fresh(self.teleop_time):
            self.output.publish(self._sanitize(self.teleop))
        else:
            self.output.publish(self._zero())

    def destroy_node(self):
        if rclpy.ok() and self.output is not None:
            self.output.publish(self._zero())
        if self._gate_lock is not None:
            try:
                fcntl.flock(self._gate_lock.fileno(), fcntl.LOCK_UN)
                self._gate_lock.close()
            except OSError:
                pass
            self._gate_lock = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
