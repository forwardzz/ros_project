import math
from copy import deepcopy

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ActualPathRecorder(Node):
    def __init__(self):
        super().__init__("actual_path_recorder")

        self.declare_parameter("min_step_m", 0.03)
        self.declare_parameter("max_poses", 5000)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.actual_path = Path()
        self.actual_path.header.frame_id = "map"
        self.last_xy = None
        self.navigation_active = False
        self.active_goal_ids = ()
        self.action_goal_ids = {
            "navigate_to_pose": (),
            "navigate_through_poses": (),
        }

        self.actual_path_pub = self.create_publisher(Path, "/actual_path", latched_qos)
        self.compat_path_pub = self.create_publisher(Path, "/mission_actual_path", latched_qos)

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 20)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            lambda msg: self._nav_status_cb("navigate_to_pose", msg),
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_through_poses/_action/status",
            lambda msg: self._nav_status_cb("navigate_through_poses", msg),
            10,
        )

        self._publish_path()
        self.get_logger().info("Actual path recorder ready")

    def _nav_status_cb(self, action_name, msg):
        active_ids = tuple(
            sorted(
                self._goal_id_key(status.goal_info.goal_id)
                for status in msg.status_list
                if status.status
                in (
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING,
                )
            )
        )
        self.action_goal_ids[action_name] = active_ids
        combined_ids = tuple(sorted(goal_id for ids in self.action_goal_ids.values() for goal_id in ids))

        if combined_ids and combined_ids != self.active_goal_ids:
            self._reset_path()
            self.get_logger().info("Navigation started; reset actual path")

        self.active_goal_ids = combined_ids
        self.navigation_active = bool(combined_ids)

    def _amcl_pose_cb(self, msg):
        if not self.navigation_active:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_xy is not None:
            last_x, last_y = self.last_xy
            if math.hypot(x - last_x, y - last_y) < float(self.get_parameter("min_step_m").value):
                return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = msg.header.stamp
        pose.pose = deepcopy(msg.pose.pose)

        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.actual_path.poses.append(pose)
        max_poses = int(self.get_parameter("max_poses").value)
        if max_poses > 0 and len(self.actual_path.poses) > max_poses:
            self.actual_path.poses = self.actual_path.poses[-max_poses:]
        self.last_xy = (x, y)
        self._publish_path()

    def _reset_path(self):
        self.actual_path = Path()
        self.actual_path.header.frame_id = "map"
        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.last_xy = None
        self._publish_path()

    def _publish_path(self):
        self.actual_path_pub.publish(self.actual_path)
        self.compat_path_pub.publish(self.actual_path)

    @staticmethod
    def _goal_id_key(goal_id):
        return tuple(goal_id.uuid)


def main(args=None):
    rclpy.init(args=args)
    node = ActualPathRecorder()
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
