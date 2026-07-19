import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool


class VelocitySafetyState:
    """Pure fail-closed arbitration state, kept separate for deterministic tests."""

    def __init__(self, input_timeout=0.35, heartbeat_timeout=0.30, scan_timeout=0.50):
        self.input_timeout = float(input_timeout)
        self.heartbeat_timeout = float(heartbeat_timeout)
        self.scan_timeout = float(scan_timeout)
        self.commands = {"teleop": None, "auto": None}
        self.command_times = {"teleop": None, "auto": None}
        self.heartbeat_time = None
        self.scan_time = None
        self.safety_stop = None
        self.software_estop = False

    def set_command(self, source, linear, angular, now):
        self.commands[source] = (float(linear), float(angular))
        self.command_times[source] = float(now)

    def output(self, now, duplicate_gate=False, max_linear=0.18, max_angular=0.55):
        now = float(now)
        healthy = (
            not duplicate_gate
            and self.safety_stop is False
            and not self.software_estop
            and self.heartbeat_time is not None
            and now - self.heartbeat_time <= self.heartbeat_timeout
            and self.scan_time is not None
            and now - self.scan_time <= self.scan_timeout
        )
        if not healthy:
            return 0.0, 0.0

        selected = None
        for source in ("teleop", "auto"):
            stamp = self.command_times[source]
            if stamp is not None and now - stamp <= self.input_timeout:
                selected = self.commands[source]
                break
        if selected is None or not all(math.isfinite(value) for value in selected):
            return 0.0, 0.0
        linear, angular = selected
        return (
            max(-max_linear, min(max_linear, linear)),
            max(-max_angular, min(max_angular, angular)),
        )


class VelocitySafetyGate(Node):
    def __init__(self):
        super().__init__("velocity_safety_gate")
        input_timeout = self.declare_parameter("input_timeout_sec", 0.35).value
        heartbeat_timeout = self.declare_parameter("safety_heartbeat_timeout_sec", 0.30).value
        scan_timeout = self.declare_parameter("scan_timeout_sec", 0.50).value
        self.max_linear = float(self.declare_parameter("max_linear_speed", 0.18).value)
        self.max_angular = float(self.declare_parameter("max_angular_speed", 0.55).value)
        self.state = VelocitySafetyState(input_timeout, heartbeat_timeout, scan_timeout)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.output_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_teleop", self._teleop_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_auto", self._auto_cb, 10)
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/safety_stop", self._safety_stop_cb, latched_qos)
        self.create_subscription(Empty, "/safety_heartbeat", self._heartbeat_cb, 10)
        self.create_service(SetBool, "/set_software_estop", self._software_estop_cb)
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info("Velocity safety gate ready (fail-closed)")

    @staticmethod
    def _now():
        return time.monotonic()

    def _teleop_cb(self, msg):
        self.state.set_command("teleop", msg.linear.x, msg.angular.z, self._now())

    def _auto_cb(self, msg):
        self.state.set_command("auto", msg.linear.x, msg.angular.z, self._now())

    def _scan_cb(self, _msg):
        self.state.scan_time = self._now()

    def _safety_stop_cb(self, msg):
        self.state.safety_stop = bool(msg.data)

    def _heartbeat_cb(self, _msg):
        self.state.heartbeat_time = self._now()

    def _software_estop_cb(self, request, response):
        self.state.software_estop = bool(request.data)
        response.success = True
        response.message = "Software emergency stop latched" if request.data else "Software emergency stop released"
        self._publish_zero()
        return response

    def _tick(self):
        publishers = self.get_publishers_info_by_topic("/cmd_vel")
        duplicate_gate = len(publishers) != 1
        linear, angular = self.state.output(
            self._now(), duplicate_gate, self.max_linear, self.max_angular
        )
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.output_pub.publish(msg)

    def _publish_zero(self):
        self.output_pub.publish(Twist())

    def destroy_node(self):
        self._publish_zero()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySafetyGate()
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
