import math
from copy import deepcopy

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class ActualPathRecorder(Node):
    def __init__(self):
        super().__init__("actual_path_recorder")

        self.declare_parameter("min_step_m", 0.03)
        self.declare_parameter("max_poses", 5000)
        self.declare_parameter("sample_period_s", 0.10)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("active_cmd_timeout_s", 1.0)
        self.declare_parameter("plan_start_timeout_s", 2.0)
        self.declare_parameter("new_plan_reset_gap_s", 5.0)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.actual_path = Path()
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.actual_path.header.frame_id = self.global_frame
        self.last_xy = None
        self.navigation_active = False
        self.active_goal_ids = ()
        self.action_goal_ids = {
            "navigate_to_pose": (),
            "navigate_through_poses": (),
        }
        self.last_amcl_pose = None
        self.last_nav_cmd_time = None
        self.last_plan_time = None
        self.last_active_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.actual_path_pub = self.create_publisher(Path, "/actual_path", latched_qos)
        self.compat_path_pub = self.create_publisher(Path, "/mission_actual_path", latched_qos)

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 20)
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._nav_cmd_cb, 10)
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

        self.create_timer(float(self.get_parameter("sample_period_s").value), self._sample_pose)
        self._publish_path()
        self.get_logger().info(
            f"Actual path recorder ready: publishing {self.global_frame}->{self.robot_frame} track on /actual_path"
        )

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
        if self.navigation_active:
            self.last_active_time = self.get_clock().now()

    def _plan_cb(self, msg):
        if not msg.poses:
            return

        now = self.get_clock().now()
        previous_plan_time = self.last_plan_time
        self.last_plan_time = now
        if self.navigation_active:
            return

        if not self.actual_path.poses:
            self._reset_path()
            self.get_logger().info("Nav2 plan received; reset actual path")
            self.last_active_time = now
            return

        if previous_plan_time is None:
            return

        plan_gap_s = self._age_s(previous_plan_time, now)
        reset_gap = float(self.get_parameter("new_plan_reset_gap_s").value)
        if plan_gap_s > reset_gap and not self._has_recent_nav_cmd(reset_gap, now):
            self._reset_path()
            self.get_logger().info("Nav2 plan received; reset actual path")
            self.last_active_time = now

    def _nav_cmd_cb(self, msg):
        linear = math.hypot(float(msg.linear.x), float(msg.linear.y))
        angular = abs(float(msg.angular.z))
        if linear > 0.001 or angular > 0.001:
            self.last_nav_cmd_time = self.get_clock().now()
            self.last_active_time = self.last_nav_cmd_time

    def _amcl_pose_cb(self, msg):
        self.last_amcl_pose = msg

    def _sample_pose(self):
        if not self._should_record():
            return

        pose = self._lookup_tf_pose()
        if pose is None:
            pose = self._latest_amcl_pose()
        if pose is None:
            return

        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        if self.last_xy is not None:
            last_x, last_y = self.last_xy
            if math.hypot(x - last_x, y - last_y) < float(self.get_parameter("min_step_m").value):
                return

        self.actual_path.header.stamp = self.get_clock().now().to_msg()
        self.actual_path.poses.append(pose)
        max_poses = int(self.get_parameter("max_poses").value)
        if max_poses > 0 and len(self.actual_path.poses) > max_poses:
            self.actual_path.poses = self.actual_path.poses[-max_poses:]
        self.last_xy = (x, y)
        self._publish_path()

    def _should_record(self):
        if self.navigation_active:
            return True

        now = self.get_clock().now()
        cmd_timeout = float(self.get_parameter("active_cmd_timeout_s").value)
        if self._has_recent_nav_cmd(cmd_timeout, now):
            return True

        plan_timeout = float(self.get_parameter("plan_start_timeout_s").value)
        if self.last_plan_time is not None:
            age_s = self._age_s(self.last_plan_time, now)
            if age_s <= plan_timeout:
                return True

        return False

    def _has_recent_nav_cmd(self, timeout_s, now=None):
        if self.last_nav_cmd_time is None:
            return False
        now = now if now is not None else self.get_clock().now()
        return self._age_s(self.last_nav_cmd_time, now) <= timeout_s

    @staticmethod
    def _age_s(previous_time, now):
        return (now - previous_time).nanoseconds / 1e9

    def _lookup_tf_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
            )
        except TransformException:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _latest_amcl_pose(self):
        if self.last_amcl_pose is None:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.last_amcl_pose.header.stamp
        pose.pose = deepcopy(self.last_amcl_pose.pose.pose)
        return pose

    def _reset_path(self):
        self.actual_path = Path()
        self.actual_path.header.frame_id = self.global_frame
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
