"""ROS 2 adapter: subscriptions, publishers, services and state callbacks.

Kept separate from the Qt interface so ROS behavior can be tested directly.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, Trigger

from robot_monitor_interfaces.msg import GasData, MissionStatus
from robot_monitor_interfaces.srv import Localize, StartNavigation

from .logic.initial_pose import make_initial_pose_message
from .logic.topic_health import TopicHealthTracker


class RosUiAdapter:
    def __init__(self, args=None):
        rclpy.init(args=args)
        self.node = rclpy.create_node("robot_control_ui")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # created before any subscription so callbacks always find them
        self.topic_trackers = {
            "/scan": TopicHealthTracker("/scan"),
            "/odom": TopicHealthTracker("/odom"),
            "/map": TopicHealthTracker("/map"),
            "/amcl_pose": TopicHealthTracker("/amcl_pose"),
            "/mission_status_typed": TopicHealthTracker("/mission_status_typed"),
        }

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.odom_sub = self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos
        )
        self.amcl_sub = self.node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10
        )
        self.thermal_sub = self.node.create_subscription(
            Float32MultiArray, "/thermal_frame", self._thermal_cb, 10
        )
        self.gas_sub = self.node.create_subscription(
            GasData, "/gas_data", self._gas_cb, 10
        )
        self.mission_status_sub = self.node.create_subscription(
            MissionStatus, "/mission_status_typed", self._mission_status_cb, map_qos
        )
        self.localize_client = self.node.create_client(Localize, "/localize_robot")
        self.start_navigation_client = self.node.create_client(
            StartNavigation, "/start_navigation"
        )
        self.clear_rviz_points_client = self.node.create_client(
            Trigger, "/clear_rviz_points"
        )
        self.set_region_mode_client = self.node.create_client(
            SetBool, "/set_region_mode"
        )
        self.set_tsp_mode_client = self.node.create_client(SetBool, "/set_tsp_mode")
        self.clear_inspection_regions_client = self.node.create_client(
            Trigger, "/clear_inspection_regions"
        )
        self.save_inspection_regions_client = self.node.create_client(
            Trigger, "/save_inspection_regions"
        )
        self.load_inspection_regions_client = self.node.create_client(
            Trigger, "/load_inspection_regions"
        )
        self.abort_mission_client = self.node.create_client(Trigger, "/abort_mission")
        self.undo_rviz_point_client = self.node.create_client(
            Trigger, "/undo_last_rviz_point"
        )
        self.undo_region_client = self.node.create_client(
            Trigger, "/undo_last_inspection_region"
        )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_yaw_rate = 0.0
        self.scan_count = 0
        self.map_data = None
        self.map_revision = 0
        self.last_scan_stamp = 0.0
        self.last_odom_stamp = 0.0
        self.last_map_stamp = 0.0
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.last_amcl_stamp = 0.0
        self.last_thermal_stamp = 0.0
        self.last_gas_stamp = 0.0
        self.thermal_width = 32
        self.thermal_height = 24
        self.thermal_frame = []
        self.thermal_min = 0.0
        self.thermal_max = 0.0
        self.thermal_avg = 0.0
        self.thermal_change_per_min = 0.0
        self.thermal_change_ready = False
        self.thermal_baseline_avg = None
        self.thermal_baseline_time = 0.0
        self.gas_data = {
            "H2": 0.0,
            "CO": 0.0,
            "VOC": 0.0,
            "Smoke": 0.0,
        }
        self.mission_state = "IDLE"
        self.mission_mode = "waypoints"
        self.mission_message = "No mission"
        self.mission_active = False
        self.mission_current_index = 0
        self.mission_total_count = 0

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_yaw_rate = msg.twist.twist.angular.z
        now = time.time()
        self.last_odom_stamp = now
        self.topic_trackers["/odom"].track(now, self.odom_sub.get_publisher_count())

    def _scan_cb(self, msg):
        self.scan_count = len(msg.ranges)
        now = time.time()
        self.last_scan_stamp = now
        self.topic_trackers["/scan"].track(now, self.scan_sub.get_publisher_count())

    def _map_cb(self, msg):
        self.map_data = msg
        self.map_revision += 1
        now = time.time()
        self.last_map_stamp = now
        self.topic_trackers["/map"].track(now, self.map_sub.get_publisher_count())

    def _amcl_pose_cb(self, msg):
        self.amcl_x = float(msg.pose.pose.position.x)
        self.amcl_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.amcl_yaw = math.atan2(siny_cosp, cosy_cosp)
        now = time.time()
        self.last_amcl_stamp = now
        self.topic_trackers["/amcl_pose"].track(now, self.amcl_sub.get_publisher_count())

    def _gas_cb(self, msg):
        self.gas_data["H2"] = float(msg.hydrogen_concentration)
        self.gas_data["CO"] = float(msg.co_concentration)
        self.gas_data["VOC"] = float(msg.voc_concentration)
        self.gas_data["Smoke"] = float(msg.smoke_concentration)
        self.last_gas_stamp = time.time()

    def _thermal_cb(self, msg):
        dims = msg.layout.dim
        if len(dims) >= 2 and dims[0].size > 0 and dims[1].size > 0:
            self.thermal_height = int(dims[0].size)
            self.thermal_width = int(dims[1].size)

        if not msg.data:
            return

        self.thermal_frame = list(msg.data)
        self.thermal_min = min(self.thermal_frame)
        self.thermal_max = max(self.thermal_frame)
        self.thermal_avg = sum(self.thermal_frame) / len(self.thermal_frame)
        now = time.time()
        if self.thermal_baseline_avg is None:
            self.thermal_baseline_avg = self.thermal_avg
            self.thermal_baseline_time = now
        elif now - self.thermal_baseline_time >= 60.0:
            self.thermal_change_per_min = self.thermal_avg - self.thermal_baseline_avg
            self.thermal_change_ready = True
            self.thermal_baseline_avg = self.thermal_avg
            self.thermal_baseline_time = now
        self.last_thermal_stamp = time.time()

    def _mission_status_cb(self, msg):
        self.mission_state = msg.state or "IDLE"
        self.mission_mode = msg.mode or "waypoints"
        self.mission_message = msg.message or ""
        self.mission_active = bool(msg.active)
        self.mission_current_index = int(msg.current_index)
        self.mission_total_count = int(msg.total_count)
        now = time.time()
        self.topic_trackers["/mission_status_typed"].track(
            now, self.mission_status_sub.get_publisher_count()
        )

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def initial_pose_subscription_count(self):
        return self.initial_pose_pub.get_subscription_count()

    def publish_initial_pose(self, x, y, yaw):
        msg = make_initial_pose_message(
            x,
            y,
            yaw,
            self.node.get_clock().now().to_msg(),
        )
        self.initial_pose_pub.publish(msg)
        return msg

    def call_service_async(self, client, request, done_callback, timeout_sec=6.0):
        def worker():
            if not client.wait_for_service(timeout_sec=timeout_sec):
                done_callback(None, f"Service {client.srv_name} is unavailable")
                return

            future = client.call_async(request)
            deadline = time.time() + timeout_sec
            while rclpy.ok() and not future.done() and time.time() < deadline:
                time.sleep(0.05)

            if not future.done():
                done_callback(None, f"Service {client.srv_name} timed out")
                return

            try:
                result = future.result()
            except Exception as exc:
                done_callback(None, str(exc))
                return
            done_callback(result, None)

        threading.Thread(target=worker, daemon=True).start()

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
