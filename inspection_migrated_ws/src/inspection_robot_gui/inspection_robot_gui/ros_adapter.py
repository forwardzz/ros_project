import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from robot_monitor_interfaces.msg import GasData, MissionStatus, RobotSafetyStatus
from robot_monitor_interfaces.srv import Localize, StartNavigation

from .config import (
    ACTION_NAVIGATE_TO_POSE,
    SERVICE_ABORT_MISSION,
    SERVICE_EMERGENCY_STOP,
    SERVICE_CLEAR_INSPECTION_REGIONS,
    SERVICE_CLEAR_RVIZ_POINTS,
    SERVICE_LOAD_INSPECTION_REGIONS,
    SERVICE_LOCALIZE_ROBOT,
    SERVICE_SAVE_INSPECTION_REGIONS,
    SERVICE_RESET_SAFETY_MONITOR,
    SERVICE_SET_REGION_MODE,
    SERVICE_START_NAVIGATION,
    SERVICE_UNDO_LAST_INSPECTION_REGION,
    SERVICE_UNDO_LAST_RVIZ_POINT,
    TOPIC_AMCL_POSE,
    TOPIC_CMD_VEL,
    TOPIC_GOAL_POSE,
    TOPIC_IMU_RAW,
    TOPIC_INITIAL_POSE,
    TOPIC_LASER_ODOM,
    TOPIC_MAP,
    TOPIC_MISSION_STATUS,
    TOPIC_MISSION_STATUS_TYPED,
    TOPIC_ODOM,
    TOPIC_SCAN,
    TOPIC_THERMAL,
    TOPIC_GAS,
    TOPIC_SAFETY,
    TOPIC_WHEEL_ODOM,
)


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RosAdapter:
    def __init__(
        self,
        status_callback=None,
        feedback_callback=None,
        result_callback=None,
        mission_status_callback=None,
    ):
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node("inspection_robot_gui")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.status_callback = status_callback
        self.feedback_callback = feedback_callback
        self.result_callback = result_callback
        self.mission_status_callback = mission_status_callback
        self._lock = threading.Lock()
        self._goal_handle = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_vel_pub = self.node.create_publisher(Twist, TOPIC_CMD_VEL, 10)
        self.initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, TOPIC_INITIAL_POSE, 10
        )
        self.goal_pose_pub = self.node.create_publisher(PoseStamped, TOPIC_GOAL_POSE, 10)

        self.localize_client = self.node.create_client(Localize, SERVICE_LOCALIZE_ROBOT)
        self.start_navigation_client = self.node.create_client(
            StartNavigation, SERVICE_START_NAVIGATION
        )
        self.clear_rviz_points_client = self.node.create_client(
            Trigger, SERVICE_CLEAR_RVIZ_POINTS
        )
        self.set_region_mode_client = self.node.create_client(
            SetBool, SERVICE_SET_REGION_MODE
        )
        self.clear_inspection_regions_client = self.node.create_client(
            Trigger, SERVICE_CLEAR_INSPECTION_REGIONS
        )
        self.save_inspection_regions_client = self.node.create_client(
            Trigger, SERVICE_SAVE_INSPECTION_REGIONS
        )
        self.load_inspection_regions_client = self.node.create_client(
            Trigger, SERVICE_LOAD_INSPECTION_REGIONS
        )
        self.abort_mission_client = self.node.create_client(
            Trigger, SERVICE_ABORT_MISSION
        )
        self.emergency_stop_client = self.node.create_client(
            SetBool, SERVICE_EMERGENCY_STOP
        )
        self.reset_safety_monitor_client = self.node.create_client(
            Trigger, SERVICE_RESET_SAFETY_MONITOR
        )
        self.undo_last_region_client = self.node.create_client(
            Trigger, SERVICE_UNDO_LAST_INSPECTION_REGION
        )
        self.undo_last_point_client = self.node.create_client(
            Trigger, SERVICE_UNDO_LAST_RVIZ_POINT
        )

        self.node.create_subscription(Odometry, TOPIC_ODOM, self._odom_cb, 10)
        self.node.create_subscription(Odometry, TOPIC_LASER_ODOM, self._laser_odom_cb, 10)
        self.node.create_subscription(Odometry, TOPIC_WHEEL_ODOM, self._wheel_odom_cb, 10)
        self.node.create_subscription(LaserScan, TOPIC_SCAN, self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(Imu, TOPIC_IMU_RAW, self._imu_cb, qos_profile_sensor_data)
        self.node.create_subscription(OccupancyGrid, TOPIC_MAP, self._map_cb, map_qos)
        self.node.create_subscription(
            PoseWithCovarianceStamped, TOPIC_AMCL_POSE, self._amcl_cb, 10
        )
        self.node.create_subscription(String, TOPIC_MISSION_STATUS, self._mission_status_cb, 10)
        self.node.create_subscription(MissionStatus, TOPIC_MISSION_STATUS_TYPED, self._typed_mission_status_cb, 10)
        self.node.create_subscription(Float32MultiArray, TOPIC_THERMAL, self._thermal_cb, 10)
        self.node.create_subscription(GasData, TOPIC_GAS, self._gas_cb, 10)
        self.node.create_subscription(RobotSafetyStatus, TOPIC_SAFETY, self._safety_cb, map_qos)

        self.nav_client = ActionClient(self.node, NavigateToPose, ACTION_NAVIGATE_TO_POSE)

        now = time.monotonic()
        self.data = {
            "last_scan": 0.0,
            "scan_count": 0,
            "scan_frame": "",
            "last_imu": 0.0,
            "imu_frame": "",
            "last_laser_odom": 0.0,
            "last_wheel_odom": 0.0,
            "last_odom": 0.0,
            "last_map": 0.0,
            "last_amcl": 0.0,
            "map_width": 0,
            "map_height": 0,
            "map_resolution": 0.0,
            "map_msg": None,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "wz": 0.0,
            "wheel_vx": 0.0,
            "wheel_wz": 0.0,
            "amcl_x": 0.0,
            "amcl_y": 0.0,
            "amcl_yaw": 0.0,
            "started": now,
            "nav_status": "idle",
            "nav_distance_remaining": None,
            "mission_status": "",
            "mission_active": False,
            "mission_mode": "",
            "last_thermal": 0.0,
            "thermal_frame": [],
            "thermal_width": 0,
            "thermal_height": 0,
            "thermal_min": 0.0,
            "thermal_max": 0.0,
            "thermal_avg": 0.0,
            "thermal_change_per_min": 0.0,
            "thermal_change_ready": False,
            "thermal_baseline_avg": None,
            "thermal_baseline_time": 0.0,
            "thermal_error": "",
            "gas": {"H2": 0.0, "CO": 0.0, "VOC": 0.0, "Smoke": 0.0},
            "last_gas": 0.0,
            "safety_level": "WAITING",
            "safety_code": "INIT",
            "safety_message": "No safety data",
        }

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)
        self._emit_status(f"[ROS] initial pose -> x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg")

    def publish_goal_pose(self, x, y, yaw):
        msg = self._make_pose_stamped(x, y, yaw)
        self.goal_pose_pub.publish(msg)
        self._emit_status(f"[ROS] /goal_pose -> x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg")

    def send_nav_goal(self, x, y, yaw):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._emit_result("NavigateToPose action server is unavailable.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(x, y, yaw)
        goal.behavior_tree = ""
        with self._lock:
            self.data["nav_status"] = "sending"
            self.data["nav_distance_remaining"] = None
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback_cb)
        send_future.add_done_callback(self._nav_goal_response_cb)
        self._emit_status(f"[NAV] goal -> x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg")
        return True

    def cancel_nav_goal(self):
        goal_handle = self._goal_handle
        if goal_handle is None:
            self._emit_status("[NAV] no active goal to cancel")
            return
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _future: self._emit_status("[NAV] cancel requested"))

    def call_service_async(self, client, request, done_callback, timeout_sec=6.0):
        def worker():
            if not client.wait_for_service(timeout_sec=timeout_sec):
                done_callback(None, f"Service {client.srv_name} is unavailable")
                return

            future = client.call_async(request)
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)

            if not future.done():
                done_callback(None, f"Service {client.srv_name} timed out")
                return

            try:
                done_callback(future.result(), None)
            except Exception as exc:
                done_callback(None, str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def snapshot(self):
        with self._lock:
            snapshot = dict(self.data)
            snapshot["thermal_frame"] = list(self.data.get("thermal_frame", []))
            snapshot["gas"] = dict(self.data.get("gas", {}))
            return snapshot

    def shutdown(self):
        try:
            self.publish_cmd_vel(0.0, 0.0)
        except Exception:
            pass
        self.executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _make_pose_stamped(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        with self._lock:
            self.data["x"] = float(msg.pose.pose.position.x)
            self.data["y"] = float(msg.pose.pose.position.y)
            self.data["yaw"] = quaternion_to_yaw(q)
            self.data["vx"] = float(msg.twist.twist.linear.x)
            self.data["wz"] = float(msg.twist.twist.angular.z)
            self.data["last_odom"] = time.monotonic()

    def _laser_odom_cb(self, _msg):
        with self._lock:
            self.data["last_laser_odom"] = time.monotonic()

    def _wheel_odom_cb(self, msg):
        with self._lock:
            self.data["last_wheel_odom"] = time.monotonic()
            self.data["wheel_vx"] = float(msg.twist.twist.linear.x)
            self.data["wheel_wz"] = float(msg.twist.twist.angular.z)

    def _scan_cb(self, msg):
        with self._lock:
            self.data["scan_count"] = len(msg.ranges)
            self.data["scan_frame"] = msg.header.frame_id
            self.data["last_scan"] = time.monotonic()

    def _imu_cb(self, msg):
        with self._lock:
            self.data["imu_frame"] = msg.header.frame_id
            self.data["last_imu"] = time.monotonic()

    def _map_cb(self, msg):
        with self._lock:
            self.data["map_width"] = int(msg.info.width)
            self.data["map_height"] = int(msg.info.height)
            self.data["map_resolution"] = float(msg.info.resolution)
            self.data["map_msg"] = msg
            self.data["last_map"] = time.monotonic()

    def _amcl_cb(self, msg):
        q = msg.pose.pose.orientation
        with self._lock:
            self.data["amcl_x"] = float(msg.pose.pose.position.x)
            self.data["amcl_y"] = float(msg.pose.pose.position.y)
            self.data["amcl_yaw"] = quaternion_to_yaw(q)
            self.data["last_amcl"] = time.monotonic()

    def _mission_status_cb(self, msg):
        text = str(msg.data)
        safety = text.startswith("[SAFETY]")
        message = text[len("[SAFETY]"):].strip() if safety else text
        with self._lock:
            self.data["mission_status"] = message
        if self.mission_status_callback:
            self.mission_status_callback.emit(message, safety)

    def _typed_mission_status_cb(self, msg):
        with self._lock:
            self.data["mission_status"] = str(msg.message)
            self.data["mission_active"] = bool(msg.active)
            self.data["mission_mode"] = str(msg.mode)
        if self.mission_status_callback:
            self.mission_status_callback.emit(str(msg.message), bool(msg.safety_warning))

    def _thermal_cb(self, msg):
        values = []
        for value in msg.data:
            try:
                value = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
        dims = list(msg.layout.dim)
        height = int(dims[0].size) if len(dims) > 0 else 0
        width = int(dims[1].size) if len(dims) > 1 else 0
        if not dims and len(values) == 32 * 24:
            width, height = 32, 24
        valid_layout = bool(values) and width > 0 and height > 0 and width * height == len(values)
        now = time.monotonic()
        with self._lock:
            self.data["last_thermal"] = now
            self.data["thermal_width"] = width
            self.data["thermal_height"] = height
            if not values:
                self.data["thermal_frame"] = []
                self.data["thermal_error"] = "热成像帧为空或包含无效数据"
                return
            if not valid_layout:
                self.data["thermal_frame"] = []
                self.data["thermal_error"] = (
                    f"热成像维度无效: {width}x{height}, data={len(values)}"
                )
                return

            average = sum(values) / len(values)
            self.data["thermal_frame"] = values
            self.data["thermal_min"] = min(values)
            self.data["thermal_max"] = max(values)
            self.data["thermal_avg"] = average
            self.data["thermal_error"] = ""
            baseline = self.data.get("thermal_baseline_avg")
            baseline_time = float(self.data.get("thermal_baseline_time", 0.0))
            if baseline is None:
                self.data["thermal_baseline_avg"] = average
                self.data["thermal_baseline_time"] = now
            elif now - baseline_time >= 60.0:
                self.data["thermal_change_per_min"] = average - float(baseline)
                self.data["thermal_change_ready"] = True
                self.data["thermal_baseline_avg"] = average
                self.data["thermal_baseline_time"] = now

    def _gas_cb(self, msg):
        with self._lock:
            self.data["gas"] = {
                "H2": float(msg.hydrogen_concentration),
                "CO": float(msg.co_concentration),
                "VOC": float(msg.voc_concentration),
                "Smoke": float(msg.smoke_concentration),
            }
            self.data["last_gas"] = time.monotonic()

    def _safety_cb(self, msg):
        with self._lock:
            self.data["safety_level"] = str(msg.level)
            self.data["safety_code"] = str(msg.code)
            self.data["safety_message"] = str(msg.message)

    def _nav_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        with self._lock:
            self.data["nav_status"] = "executing"
            self.data["nav_distance_remaining"] = float(feedback.distance_remaining)
        if self.feedback_callback:
            self.feedback_callback.emit(
                f"[NAV] distance remaining {feedback.distance_remaining:.2f} m"
            )

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            with self._lock:
                self.data["nav_status"] = "rejected"
            self._emit_result("NavigateToPose goal rejected.")
            return
        self._goal_handle = goal_handle
        with self._lock:
            self.data["nav_status"] = "accepted"
        self._emit_status("[NAV] goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        status_name = {
            GoalStatus.STATUS_UNKNOWN: "unknown",
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }.get(result.status, str(result.status))
        message = f"NavigateToPose {status_name}: {result.result.error_msg or 'no error'}"
        with self._lock:
            self.data["nav_status"] = status_name
            self.data["nav_distance_remaining"] = None
        self._goal_handle = None
        self._emit_result(message)

    def _emit_status(self, text):
        if self.status_callback:
            self.status_callback.emit(text)

    def _emit_result(self, text):
        if self.result_callback:
            self.result_callback.emit(text)
