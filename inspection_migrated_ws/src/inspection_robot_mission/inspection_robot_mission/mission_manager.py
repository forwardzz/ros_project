import math
import os
from concurrent.futures import ThreadPoolExecutor

import yaml

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener

from robot_mission_utils.inspection_planner import (
    RegionRouteOption,
    plan_mission_order,
    plan_region_mission_order,
    validate_mission_points,
)
from robot_mission_utils.grid_map import GridMap
from robot_mission_utils.map_inflation import inflate_map as inflate_grid_map
from robot_monitor_interfaces.msg import InspectionPoint, MissionStatus
from robot_monitor_interfaces.srv import ConfirmInspectionPoints, Localize, StartNavigation

from .mission_regions import (
    InspectionRegion,
    assign_path_headings,
    generate_chassis_path_for_region,
    generate_chassis_region_paths,
    generate_points_for_region,
    regions_from_yaml,
    regions_to_yaml_data,
    sweep_positions,
)
from .qos import latched_qos
from .robot_config import (
    ACTION_NAVIGATE_TO_POSE,
    FRAME_BASE_LINK,
    FRAME_MAP,
    INSPECTION_REGIONS_PATH,
    SERVICE_ABORT_MISSION,
    SERVICE_CLEAR_INSPECTION_REGIONS,
    SERVICE_CLEAR_RVIZ_POINTS,
    SERVICE_CONFIRM_INSPECTION_POINTS,
    SERVICE_LOAD_INSPECTION_REGIONS,
    SERVICE_LOCALIZE_ROBOT,
    SERVICE_SAVE_INSPECTION_REGIONS,
    SERVICE_SET_REGION_MODE,
    SERVICE_START_NAVIGATION,
    SERVICE_UNDO_LAST_INSPECTION_REGION,
    SERVICE_UNDO_LAST_RVIZ_POINT,
    TOPIC_AMCL_POSE,
    TOPIC_CLICKED_POINT,
    TOPIC_CMD_VEL_NAV,
    TOPIC_GOAL_POSE,
    TOPIC_MAP,
    TOPIC_MISSION_GOAL_POSE,
    TOPIC_MISSION_POINTS_MARKERS,
    TOPIC_MISSION_PREVIEW_PATH,
    TOPIC_MISSION_STATUS,
    TOPIC_ODOM,
    TOPIC_SCAN,
)
from .ros_utils import inspection_point_to_pose, make_inspection_point, quat_to_yaw


class MissionManager(Node):
    DEFAULT_WAYPOINT_PAUSE_SEC = 2.0
    MAX_WAYPOINT_PAUSE_SEC = 60.0
    RELAXED_GOAL_DISTANCE_M = 0.20

    def __init__(self):
        super().__init__("mission_manager")

        self.thermal_observation_distance = float(
            self.declare_parameter("thermal_observation_distance", 0.05).value
        )
        self.thermal_horizontal_fov_deg = float(
            self.declare_parameter("thermal_horizontal_fov_deg", 110.0).value
        )
        self.thermal_overlap_ratio = float(
            self.declare_parameter("thermal_overlap_ratio", 0.30).value
        )
        theoretical_spacing = (
            2.0
            * self.thermal_observation_distance
            * math.tan(math.radians(self.thermal_horizontal_fov_deg) * 0.5)
            * (1.0 - self.thermal_overlap_ratio)
        )
        self.sweep_spacing = float(
            self.declare_parameter("sweep_spacing", round(theoretical_spacing, 2)).value
        )
        self.region_margin = float(self.declare_parameter("region_margin", 0.23).value)
        self.straight_resolution = float(
            self.declare_parameter("straight_resolution", 0.05).value
        )
        self.arc_resolution = float(self.declare_parameter("arc_resolution", 0.02).value)
        self.chassis_linear_speed = float(
            self.declare_parameter("chassis_linear_speed", 0.06).value
        )
        self.chassis_angular_speed = float(
            self.declare_parameter("chassis_angular_speed", 0.35).value
        )
        self.chassis_position_tolerance = float(
            self.declare_parameter("chassis_position_tolerance", 0.03).value
        )
        self.chassis_angle_tolerance = float(
            self.declare_parameter("chassis_angle_tolerance", 0.05).value
        )
        self.chassis_obstacle_distance = float(
            self.declare_parameter("chassis_obstacle_distance", 0.28).value
        )
        self.region_obstacle_wait_sec = max(
            0.1,
            float(self.declare_parameter("region_obstacle_wait_sec", 3.0).value),
        )
        self.region_obstacle_clear_frames = max(
            1,
            int(self.declare_parameter("region_obstacle_clear_frames", 10).value),
        )
        self.region_obstacle_max_retries = max(
            0,
            int(self.declare_parameter("region_obstacle_max_retries", 2).value),
        )
        self.region_obstacle_inset = float(
            self.declare_parameter("region_obstacle_inset", 0.02).value
        )
        self.region_staging_distance = float(
            self.declare_parameter("region_staging_distance", 0.20).value
        )
        self.clearance_cluster_distance = float(
            self.declare_parameter("clearance_cluster_distance", 0.12).value
        )
        self.clearance_min_points = int(
            self.declare_parameter("clearance_min_points", 4).value
        )
        self.clearance_required_frames = int(
            self.declare_parameter("clearance_required_frames", 3).value
        )
        self.clearance_observation_sec = float(
            self.declare_parameter("clearance_observation_sec", 1.0).value
        )
        self.regions_path = str(
            self.declare_parameter(
                "inspection_regions_path",
                INSPECTION_REGIONS_PATH,
            ).value
        )
        self.mission_home_x = float(
            self.declare_parameter("mission_home_x", 0.0).value
        )
        self.mission_home_y = float(
            self.declare_parameter("mission_home_y", 0.0).value
        )
        self.mission_home_yaw = float(
            self.declare_parameter("mission_home_yaw", 0.0).value
        )

        self.current_odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.current_map_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.have_odom = False
        self.have_map_pose = False
        self.have_map = False
        self.map_msg = None
        self.confirmed_points = []
        self.rviz_points = []
        self.rviz_ordered_points = []
        self.rviz_preview_path = []
        self.rviz_solving_method = ""
        self.rviz_plan_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="rviz_tsp"
        )
        self.rviz_plan_generation = 0
        self.rviz_plan_future = None
        self.rviz_plan_running_request = None
        self.rviz_plan_queued_request = None
        self.pending_rviz_start = None
        self.region_plan_generation = 0
        self.region_plan_future = None
        self.region_plan_running_request = None
        self.region_plan_queued_request = None
        self.pending_region_start = None
        self.region_ordered_indices = []
        self.region_ordered_paths = []
        self.region_ordered_staging_points = []
        self.region_transition_paths = []
        self.region_return_path = []
        self.region_solving_method = ""
        self.region_preview_path = []
        self.rviz_recompute_throttle_sec = 0.5
        self.last_rviz_recompute_time = 0.0
        self.region_mode = False
        self.pending_region_corner = None
        self.inspection_regions = []
        self.region_preview_points = []
        self.region_generation_error = None
        self.goal_handle = None
        self.mission_active = False
        self.mission_points = []
        self.mission_index = 0
        self.mission_source = ""
        self.mission_wait_timer = None
        self.mission_run_id = 0
        self.mission_waypoint_pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        self.mission_return_to_start = False
        self.mission_returning_home = False
        self.last_mission_feedback_log_time = 0.0
        self.mission_early_transition_goal = None
        self.region_paths = []
        self.mission_regions = []
        self.region_path_index = 0
        self.region_phase = ""
        self.cached_region_plan = {}
        self.region_approach_goal_handle = None
        self.region_control_timer = None
        self.region_target_index = 0
        self.region_blocked_names = []
        self.region_staging_points = []
        self.clearance_started_time = None
        self.clearance_obstacle_frames = 0
        self.clearance_last_scan_stamp = None
        self.region_obstacle_wait_started = None
        self.region_obstacle_resume_phase = ""
        self.region_obstacle_clear_count = 0
        self.region_obstacle_retry_count = 0
        self.region_obstacle_reason = ""
        self.region_obstacle_last_scan_stamp = None
        self.latest_scan = None
        self.last_scan_time = None
        self.tf_consecutive_failures = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.direct_goal_handle = None
        self.direct_nav_active = False
        self.last_direct_feedback_log_time = 0.0

        self.create_subscription(Odometry, TOPIC_ODOM, self._odom_cb, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self._scan_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, TOPIC_AMCL_POSE, self._amcl_pose_cb, 10
        )
        self.create_subscription(OccupancyGrid, TOPIC_MAP, self._map_cb, latched_qos())
        self.create_subscription(PointStamped, TOPIC_CLICKED_POINT, self._clicked_point_cb, 10)
        self.create_subscription(
            PoseStamped, TOPIC_MISSION_GOAL_POSE, self._goal_pose_cb, 10
        )
        self.create_subscription(PoseStamped, TOPIC_GOAL_POSE, self._direct_goal_pose_cb, 10)

        self.preview_pub = self.create_publisher(Path, TOPIC_MISSION_PREVIEW_PATH, latched_qos())
        self.marker_pub = self.create_publisher(MarkerArray, TOPIC_MISSION_POINTS_MARKERS, latched_qos())
        self.cmd_vel_nav_pub = self.create_publisher(Twist, TOPIC_CMD_VEL_NAV, 10)
        # Keep the legacy String topic for existing RViz/UI tooling and publish a
        # typed status in parallel for the safety supervisor.
        self.mission_status_pub = self.create_publisher(String, TOPIC_MISSION_STATUS, 10)
        self.mission_status_typed_pub = self.create_publisher(
            MissionStatus, "/mission_status_typed", 10
        )

        self.create_service(Localize, SERVICE_LOCALIZE_ROBOT, self._handle_localize)
        self.create_service(
            ConfirmInspectionPoints,
            SERVICE_CONFIRM_INSPECTION_POINTS,
            self._handle_confirm_points,
        )
        self.create_service(StartNavigation, SERVICE_START_NAVIGATION, self._handle_start_navigation)
        self.create_service(Trigger, SERVICE_CLEAR_RVIZ_POINTS, self._handle_clear_rviz_points)
        self.create_service(SetBool, SERVICE_SET_REGION_MODE, self._handle_set_region_mode)
        self.create_service(
            Trigger,
            SERVICE_CLEAR_INSPECTION_REGIONS,
            self._handle_clear_inspection_regions,
        )
        self.create_service(
            Trigger,
            SERVICE_SAVE_INSPECTION_REGIONS,
            self._handle_save_inspection_regions,
        )
        self.create_service(
            Trigger,
            SERVICE_LOAD_INSPECTION_REGIONS,
            self._handle_load_inspection_regions,
        )
        self.create_service(Trigger, SERVICE_ABORT_MISSION, self._handle_abort_mission)
        self.create_service(
            Trigger,
            SERVICE_UNDO_LAST_INSPECTION_REGION,
            self._handle_undo_last_inspection_region,
        )
        self.create_service(
            Trigger,
            SERVICE_UNDO_LAST_RVIZ_POINT,
            self._handle_undo_last_rviz_point,
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, ACTION_NAVIGATE_TO_POSE)
        self.mission_position_timer = self.create_timer(
            0.25, self._check_mission_early_transition
        )
        self.rviz_plan_poll_timer = self.create_timer(0.05, self._poll_mission_plans)
        self.get_logger().info("Mission manager ready")

    def _odom_cb(self, msg):
        self.current_odom["x"] = msg.pose.pose.position.x
        self.current_odom["y"] = msg.pose.pose.position.y
        self.current_odom["theta"] = quat_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def _amcl_pose_cb(self, msg):
        self.current_map_pose["x"] = msg.pose.pose.position.x
        self.current_map_pose["y"] = msg.pose.pose.position.y
        self.current_map_pose["theta"] = quat_to_yaw(msg.pose.pose.orientation)
        self.have_map_pose = True
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_rviz_recompute_time >= self.rviz_recompute_throttle_sec:
            self.last_rviz_recompute_time = now_sec
            if self.inspection_regions and self.have_map and self.map_msg is not None and not self.mission_active:
                self._publish_rviz_plan_visuals()

    def _scan_cb(self, msg):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _map_cb(self, msg):
        self.have_map = True
        self.map_msg = msg
        if self.inspection_regions and not self.mission_active:
            self._cancel_pending_region_start(
                "Region mission planning cancelled because the map changed"
            )
            self._recompute_region_preview()
        if self.rviz_points:
            self._cancel_pending_rviz_start(
                "RViz mission planning cancelled because the map changed"
            )
            self._recompute_rviz_plan()
        elif self.inspection_regions and not self.mission_active:
            self._publish_rviz_plan_visuals()

    def _clicked_point_cb(self, msg):
        frame_id = msg.header.frame_id or FRAME_MAP
        if frame_id != FRAME_MAP:
            self.get_logger().warn(
                f"Ignoring RViz point in frame {frame_id}; use RViz Publish Point with Fixed Frame=map"
            )
            return

        if self.region_mode:
            self._add_region_corner(float(msg.point.x), float(msg.point.y))
            return
        if self.pending_rviz_start is not None:
            self.get_logger().warn(
                "Ignoring RViz point while mission planning is pending; cancel the task first"
            )
            return

        point = make_inspection_point(
            f"RVIZ_{len(self.rviz_points) + 1}",
            float(msg.point.x),
            float(msg.point.y),
            0.0,
        )
        valid, reason = self._validate_points_for_setting(
            [point],
            f"RViz mission point {point.point_name}",
        )
        if not valid:
            self._warn_safety(reason)
            return

        self.rviz_points.append(point)
        self.get_logger().info(
            f"Added RViz mission point {point.point_name} at ({point.x:.2f}, {point.y:.2f})"
        )
        self._recompute_rviz_plan()

    def _add_region_corner(self, x, y):
        if self.pending_region_corner is None:
            self.pending_region_corner = (x, y)
            self.get_logger().info(
                f"Stored first region corner at ({x:.2f}, {y:.2f}); click the opposite corner"
            )
            self._publish_rviz_plan_visuals()
            return

        first_x, first_y = self.pending_region_corner
        self.pending_region_corner = None
        region = InspectionRegion(
            name=f"REGION_{len(self.inspection_regions) + 1}",
            min_x=min(first_x, x),
            min_y=min(first_y, y),
            max_x=max(first_x, x),
            max_y=max(first_y, y),
        )
        self.inspection_regions.append(region)
        self._recompute_region_preview()
        if self.region_generation_error is not None:
            self.inspection_regions.pop()
            message = f"Inspection region {region.name} rejected: {self.region_generation_error}"
            self._warn_safety(message)
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()
            return
        if not self.region_preview_points:
            self.inspection_regions.pop()
            message = (
                f"Inspection region {region.name} rejected: no safe sweep waypoint can be generated. "
                f"Check region_margin={self.region_margin:.2f}m."
            )
            self._warn_safety(message)
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()
            return
        valid, reason = self._validate_region_entry(region)
        if not valid:
            self.inspection_regions.pop()
            self._warn_safety(reason)
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()
            return

        self.get_logger().info(
            f"Added inspection region {region.name}: "
            f"({region.min_x:.2f}, {region.min_y:.2f}) to ({region.max_x:.2f}, {region.max_y:.2f})"
        )
        self._publish_rviz_plan_visuals()

    def _goal_pose_cb(self, msg):
        frame_id = msg.header.frame_id or FRAME_MAP
        if frame_id != FRAME_MAP:
            self.get_logger().warn(
                f"Ignoring RViz mission heading pose in frame {frame_id}; use Fixed Frame=map"
            )
            return

        if not self.rviz_points:
            self.get_logger().warn(
                "Received a Mission Heading pose but there are no RViz mission points yet"
            )
            return

        target_x = float(msg.pose.position.x)
        target_y = float(msg.pose.position.y)
        closest_index = None
        closest_distance = None
        for index, point in enumerate(self.rviz_points):
            distance = math.hypot(point.x - target_x, point.y - target_y)
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
                closest_index = index

        if closest_index is None or closest_distance is None or closest_distance > 0.75:
            self.get_logger().warn(
                "Ignoring RViz mission heading pose because it is not close to any stored mission point"
            )
            return

        point = self.rviz_points[closest_index]
        point.theta = quat_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f"Updated heading for {point.point_name} to {math.degrees(point.theta):.1f} deg"
        )
        self._publish_rviz_plan_visuals()

    def _direct_goal_pose_cb(self, msg):
        frame_id = msg.header.frame_id or FRAME_MAP
        if frame_id != FRAME_MAP:
            self.get_logger().warn(
                f"Ignoring RViz navigation goal in frame {frame_id}; use Fixed Frame=map"
            )
            return

        if self.mission_active:
            self.get_logger().warn(
                "Ignoring RViz navigation goal because an inspection mission is already running"
            )
            return
        if self.pending_rviz_start is not None:
            self.get_logger().warn(
                "Ignoring RViz navigation goal while multi-point mission planning is pending"
            )
            return
        if self.pending_region_start is not None:
            self.get_logger().warn(
                "Ignoring RViz navigation goal while region mission planning is pending"
            )
            return

        if not self.have_map_pose:
            self.get_logger().warn(
                "Ignoring RViz navigation goal because AMCL pose is unavailable. Set the initial pose first."
            )
            return

        if not self.have_map or self.map_msg is None:
            self.get_logger().warn("Ignoring RViz navigation goal because no /map data has been received")
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Nav2 action server /navigate_to_pose is not ready")
            return

        goal_point = make_inspection_point(
            "RVIZ_NAV_GOAL",
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            quat_to_yaw(msg.pose.orientation),
        )
        validation = validate_mission_points(
            self.map_msg,
            (self.current_map_pose["x"], self.current_map_pose["y"]),
            [goal_point],
            min_start_distance_m=0.05,
        )
        if not validation.valid:
            self.get_logger().warn(f"RViz navigation goal rejected: {validation.message}")
            return

        if self.direct_goal_handle is not None:
            try:
                self.direct_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Failed to cancel previous RViz navigation goal: {exc}")

        goal = NavigateToPose.Goal()
        goal.pose = self._inspection_point_to_pose(goal_point)

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self._direct_feedback_cb,
        )
        send_goal_future.add_done_callback(self._direct_goal_response_cb)
        self.direct_nav_active = True
        self.get_logger().info(
            f"RViz navigation goal sent to ({goal_point.x:.2f}, {goal_point.y:.2f}, "
            f"{math.degrees(goal_point.theta):.1f} deg)"
        )

    def _recompute_rviz_plan(self):
        self.rviz_ordered_points = list(self.rviz_points)
        self.rviz_preview_path = []
        self.rviz_solving_method = ""
        if not self.rviz_points:
            self._invalidate_rviz_plans()
            self._publish_rviz_plan_visuals()
            return

        if self.map_msg is not None and self.have_map_pose:
            self._queue_rviz_plan()
        self._publish_rviz_plan_visuals()

    def _invalidate_rviz_plans(self):
        self.rviz_plan_generation += 1
        self.rviz_plan_queued_request = None

    def _cancel_pending_rviz_start(self, reason, publish=True):
        if self.pending_rviz_start is None:
            return False
        self.pending_rviz_start = None
        self._invalidate_rviz_plans()
        self._publish_zero_cmd()
        self.get_logger().warn(reason)
        if publish:
            self._publish_mission_status(reason, safety=True)
        return True

    def _queue_rviz_plan(self, start_request=None):
        self.rviz_plan_generation += 1
        generation = self.rviz_plan_generation
        request = {
            "generation": generation,
            "map_msg": self.map_msg,
            "start_xy": (
                self.current_map_pose["x"],
                self.current_map_pose["y"],
            ),
            "points": list(self.rviz_points),
            "start_request": start_request,
        }
        self.rviz_plan_queued_request = request
        if start_request is not None:
            self.pending_rviz_start = request
        if self.rviz_plan_future is None:
            self._start_queued_rviz_plan()
        return generation

    def _start_queued_rviz_plan(self):
        request = self.rviz_plan_queued_request
        if request is None or self.rviz_plan_future is not None:
            return
        self.rviz_plan_queued_request = None
        generation = request["generation"]
        self.rviz_plan_running_request = request
        self.rviz_plan_future = self.rviz_plan_executor.submit(
            plan_mission_order,
            request["map_msg"],
            request["start_xy"],
            request["points"],
            True,
            lambda: generation != self.rviz_plan_generation,
        )

    def _poll_rviz_plan(self):
        future = self.rviz_plan_future
        if future is None or not future.done():
            return

        request = self.rviz_plan_running_request
        self.rviz_plan_future = None
        self.rviz_plan_running_request = None
        plan = None
        error = None
        try:
            plan = future.result()
        except Exception as exc:
            error = exc

        if request is not None and request["generation"] == self.rviz_plan_generation:
            if error is not None:
                self.get_logger().error(f"RViz TSP planning failed: {error}")
            self._apply_completed_rviz_plan(request, plan)

        self._start_queued_rviz_plan()

    def _poll_mission_plans(self):
        self._poll_rviz_plan()
        self._poll_region_plan()

    def _apply_completed_rviz_plan(self, request, plan):
        points = request["points"]
        ordered_points = []
        if plan is not None:
            try:
                ordered_points = [points[index] for index in plan.ordered_indices]
            except (IndexError, TypeError):
                plan = None

        if plan is None:
            self.rviz_ordered_points = list(points)
            self.rviz_preview_path = []
            self.rviz_solving_method = ""
            message = "RViz mission has no complete collision-free TSP route"
            self.get_logger().warn(message)
        else:
            self.rviz_ordered_points = ordered_points
            self.rviz_preview_path = list(plan.preview_path)
            self.rviz_solving_method = plan.solving_method
            names = " -> ".join(point.point_name for point in ordered_points)
            self.get_logger().info(
                f"RViz mission preview optimized with {plan.solving_method}: {names}"
            )
        self._publish_rviz_plan_visuals()

        if request["start_request"] is not None:
            self._finish_pending_rviz_start(request, plan, ordered_points)

    def _finish_pending_rviz_start(self, request, plan, ordered_points):
        pending = self.pending_rviz_start
        if pending is None or pending["generation"] != request["generation"]:
            return
        self.pending_rviz_start = None

        if plan is None:
            self._publish_zero_cmd()
            self._warn_safety(
                "RViz mission rejected: no complete collision-free TSP route is "
                "available from the requested robot pose"
            )
            return

        validation = validate_mission_points(
            request["map_msg"],
            request["start_xy"],
            request["points"],
            optimize_order=True,
            mission_plan=plan,
        )
        if not validation.valid:
            self._publish_zero_cmd()
            self._warn_safety(f"RViz mission rejected: {validation.message}")
            return
        if self.mission_active or self.direct_nav_active:
            self._publish_zero_cmd()
            self._warn_safety(
                "RViz mission planning completed, but another navigation task is active"
            )
            return
        if not self.nav_to_pose_client.server_is_ready():
            self._publish_zero_cmd()
            self._warn_safety(
                "RViz mission planning completed, but /navigate_to_pose is no longer ready"
            )
            return

        pause_sec = request["start_request"]["pause_sec"]
        return_to_start = request["start_request"].get("return_to_start", False)
        if not self._start_sequential_mission(
            ordered_points,
            "rviz",
            pause_sec,
            return_to_start=return_to_start,
        ):
            self._publish_zero_cmd()
            self._warn_safety("Failed to start the planned RViz mission")
            return

        names = " -> ".join(point.point_name for point in ordered_points[:12])
        if len(ordered_points) > 12:
            names += f" -> ... ({len(ordered_points)} total)"
        message = (
            f"Sequential mission started with {len(ordered_points)} points from rviz, "
            f"pause={pause_sec:.1f}s, ordering={plan.solving_method}, "
            f"return_to_start={str(return_to_start).lower()}: {names}"
        )
        self.get_logger().info(message)
        self._publish_mission_status(message)

    def _recompute_region_preview(self):
        self._cancel_pending_region_start(
            "Pending region mission cancelled because its regions changed"
        )
        self._invalidate_region_plans()
        self.region_preview_points = self._generate_region_points()
        self.region_ordered_indices = list(range(len(self.inspection_regions)))
        self.region_ordered_paths = []
        self.region_ordered_staging_points = []
        self.region_transition_paths = []
        self.region_return_path = []
        self.region_solving_method = ""
        self.region_preview_path = []
        if (
            self.region_generation_error is None
            and self.inspection_regions
            and self.have_map
            and self.map_msg is not None
            and self.have_map_pose
            and not self.mission_active
        ):
            self._queue_region_plan()

    def _invalidate_region_plans(self):
        self.region_plan_generation += 1
        self.region_plan_queued_request = None

    def _cancel_pending_region_start(self, reason, publish=True):
        if self.pending_region_start is None:
            return False
        self.pending_region_start = None
        self._invalidate_region_plans()
        self._publish_zero_cmd()
        self.get_logger().warn(reason)
        if publish:
            self._publish_mission_status(reason, safety=True)
        return True

    def _clone_region_route(self, route, reverse=False):
        source = reversed(route) if reverse else route
        cloned = [
            make_inspection_point(point.point_name, point.x, point.y, point.theta)
            for point in source
        ]
        assign_path_headings(cloned)
        return cloned

    def _build_region_route_options(self):
        base_paths = generate_chassis_region_paths(
            self.inspection_regions, self.sweep_spacing, self.region_margin
        )
        if not base_paths or any(not path for path in base_paths):
            return None
        grid = GridMap.from_occupancy_grid(self.map_msg)
        option_groups = []
        route_groups = []
        staging_groups = []
        offset = self.region_margin + self.region_staging_distance
        for region, route in zip(self.inspection_regions, base_paths):
            options = []
            routes = []
            stagings = []
            for reverse in (False, True):
                ordered = self._clone_region_route(route, reverse=reverse)
                if len(ordered) < 2:
                    continue
                first, second = ordered[0], ordered[1]
                dx, dy = second.x - first.x, second.y - first.y
                length = math.hypot(dx, dy)
                if length < 1e-6:
                    continue
                ux, uy = dx / length, dy / length
                sx, sy = first.x - ux * offset, first.y - uy * offset
                gx, gy = grid.world_to_grid(sx, sy)
                if not grid.is_valid(gx, gy):
                    continue
                staging_heading = math.atan2(first.y - sy, first.x - sx)
                staging = make_inspection_point(
                    f"{region.name}_STAGING", sx, sy, staging_heading
                )
                last = ordered[-1]
                options.append(
                    RegionRouteOption(
                        entry_xy=(sx, sy),
                        exit_xy=(last.x, last.y),
                        entry_heading=staging_heading,
                        exit_heading=last.theta,
                    )
                )
                routes.append(ordered)
                stagings.append(staging)
            if not options:
                return None
            option_groups.append(options)
            route_groups.append(routes)
            staging_groups.append(stagings)
        return option_groups, route_groups, staging_groups

    def _queue_region_plan(self, start_request=None):
        built = self._build_region_route_options()
        if built is None:
            if start_request is not None:
                self._warn_safety(
                    "Region mission rejected: one or more regions have no valid staging direction"
                )
            return None
        option_groups, route_groups, staging_groups = built
        self.region_plan_generation += 1
        generation = self.region_plan_generation
        return_to_start = bool(
            start_request and start_request.get("return_to_start", False)
        )
        request = {
            "generation": generation,
            "map_msg": self.map_msg,
            "start_pose": (
                self.current_map_pose["x"],
                self.current_map_pose["y"],
                self.current_map_pose["theta"],
            ),
            "end_pose": (
                self.mission_home_x,
                self.mission_home_y,
                self.mission_home_yaw,
            )
            if return_to_start
            else None,
            "regions": list(self.inspection_regions),
            "option_groups": option_groups,
            "route_groups": route_groups,
            "staging_groups": staging_groups,
            "start_request": start_request,
        }
        self.region_plan_queued_request = request
        if start_request is not None:
            self.pending_region_start = request
        if self.region_plan_future is None:
            self._start_queued_region_plan()
        return generation

    def _start_queued_region_plan(self):
        request = self.region_plan_queued_request
        if request is None or self.region_plan_future is not None:
            return
        self.region_plan_queued_request = None
        generation = request["generation"]
        self.region_plan_running_request = request
        self.region_plan_future = self.rviz_plan_executor.submit(
            plan_region_mission_order,
            request["map_msg"],
            request["start_pose"],
            request["option_groups"],
            request["end_pose"],
            True,
            lambda: generation != self.region_plan_generation,
        )

    def _poll_region_plan(self):
        future = self.region_plan_future
        if future is None or not future.done():
            return
        request = self.region_plan_running_request
        self.region_plan_future = None
        self.region_plan_running_request = None
        plan = None
        error = None
        try:
            plan = future.result()
        except Exception as exc:
            error = exc
        if request is not None and request["generation"] == self.region_plan_generation:
            if error is not None:
                self.get_logger().error(f"Region TSP planning failed: {error}")
            self._apply_completed_region_plan(request, plan)
        self._start_queued_region_plan()

    def _compose_region_preview_path(self, paths, transitions, return_path):
        combined = []
        for index, route in enumerate(paths):
            if index < len(transitions):
                combined.extend(transitions[index])
            combined.extend((point.x, point.y) for point in route)
        combined.extend(return_path)
        return combined

    def _apply_completed_region_plan(self, request, plan):
        if plan is None:
            self.region_ordered_indices = list(range(len(request["regions"])))
            self.region_ordered_paths = []
            self.region_ordered_staging_points = []
            self.region_transition_paths = []
            self.region_return_path = []
            self.region_solving_method = ""
            self.region_preview_path = []
            self.get_logger().warn(
                "Region mission has no complete collision-free TSP route"
            )
        else:
            paths = []
            stagings = []
            for region_index, option_index in zip(
                plan.ordered_indices, plan.option_indices
            ):
                paths.append(request["route_groups"][region_index][option_index])
                stagings.append(request["staging_groups"][region_index][option_index])
            self.region_ordered_indices = list(plan.ordered_indices)
            self.region_ordered_paths = paths
            self.region_ordered_staging_points = stagings
            self.region_transition_paths = list(plan.transition_paths)
            self.region_return_path = list(plan.return_path)
            self.region_solving_method = plan.solving_method
            self.region_preview_path = self._compose_region_preview_path(
                paths, plan.transition_paths, plan.return_path
            )
            names = " -> ".join(
                request["regions"][index].name for index in plan.ordered_indices
            )
            self.get_logger().info(
                f"Region mission preview optimized with {plan.solving_method}: {names}"
            )
        self._publish_rviz_plan_visuals()
        if request["start_request"] is not None:
            self._finish_pending_region_start(request, plan)

    def _finish_pending_region_start(self, request, plan):
        pending = self.pending_region_start
        if pending is None or pending["generation"] != request["generation"]:
            return
        self.pending_region_start = None
        if plan is None:
            self._publish_zero_cmd()
            self._warn_safety(
                "Region mission rejected: no complete collision-free region TSP route is available"
            )
            return
        if self.mission_active or self.direct_nav_active:
            self._publish_zero_cmd()
            self._warn_safety(
                "Region mission planning completed, but another navigation task is active"
            )
            return
        if not self.nav_to_pose_client.server_is_ready():
            self._publish_zero_cmd()
            self._warn_safety(
                "Region mission planning completed, but /navigate_to_pose is no longer ready"
            )
            return
        regions = [request["regions"][index] for index in plan.ordered_indices]
        if not self._start_region_path_mission(
            regions,
            self.region_ordered_paths,
            self.region_ordered_staging_points,
            self.region_transition_paths,
            plan.solving_method,
            return_to_start=request["start_request"].get("return_to_start", False),
        ):
            self._publish_zero_cmd()
            self._warn_safety("Failed to start the planned region mission")
            return
        names = " -> ".join(region.name for region in regions)
        message = (
            f"Region mission started with {len(regions)} region(s), "
            f"ordering={plan.solving_method}, "
            f"return_to_start={str(self.mission_return_to_start).lower()}: {names}"
        )
        self.get_logger().info(message)
        self._publish_mission_status(message)

    def _generate_region_points(self):
        paths = generate_chassis_region_paths(
            self.inspection_regions, self.sweep_spacing, self.region_margin
        )
        self.region_generation_error = None
        if any(not path for path in paths):
            self.region_generation_error = (
                f"One or more regions are too small for spacing={self.sweep_spacing:.2f}m "
                f"and margin={self.region_margin:.2f}m"
            )
        return [point for path in paths for point in path]

    def _generate_points_for_region(self, region):
        return generate_chassis_path_for_region(
            region, self.sweep_spacing, self.region_margin
        )

    def _validate_points_for_setting(self, points, context):
        if not self.have_map_pose:
            return (
                False,
                f"{context} rejected: AMCL pose unavailable. Set the initial pose before adding mission points.",
            )
        if not self.have_map or self.map_msg is None:
            return False, f"{context} rejected: no /map data has been received."

        validation = validate_mission_points(
            self.map_msg,
            (self.current_map_pose["x"], self.current_map_pose["y"]),
            points,
            check_route=False,
        )
        if not validation.valid:
            return False, f"{context} rejected: {validation.message}"
        return True, validation.message

    def _validate_region_entry(self, region):
        width = region.max_x - region.min_x
        height = region.max_y - region.min_y
        min_dim = max(self.sweep_spacing + self.region_margin * 2.0, 0.10)
        if width < min_dim or height < min_dim:
            return (
                False,
                f"Region {region.name} rejected: dimensions ({width:.2f}x{height:.2f}m) "
                f"are too small. Minimum is {min_dim:.2f}m per side.",
            )
        if not self.have_map or self.map_msg is None:
            self.get_logger().info(
                f"Region {region.name} accepted without map validation (no /map yet)"
            )
            return True, "Region added (map not yet available)"
        grid_map = GridMap.from_occupancy_grid(self.map_msg)
        corners = [
            (region.min_x, region.min_y),
            (region.max_x, region.min_y),
            (region.max_x, region.max_y),
            (region.min_x, region.max_y),
        ]
        for cx, cy in corners:
            gx, gy = grid_map.world_to_grid(cx, cy)
            if not grid_map.in_bounds(gx, gy):
                return (
                    False,
                    f"Region {region.name} rejected: corner ({cx:.2f}, {cy:.2f}) "
                    f"is outside map bounds.",
                )
            if not grid_map.is_valid(gx, gy):
                return (
                    False,
                    f"Region {region.name} rejected: corner ({cx:.2f}, {cy:.2f}) "
                    f"is in an obstacle or unknown area.",
                )

        sweep_points = generate_points_for_region(
            region, self.sweep_spacing, self.region_margin
        )
        inflated_grid, _ = inflate_grid_map(grid_map, radius_m=self.region_margin)
        for sx, sy in sweep_points:
            gx, gy = grid_map.world_to_grid(sx, sy)
            if not grid_map.in_bounds(gx, gy):
                return (
                    False,
                    f"Region {region.name} rejected: sweep point ({sx:.2f}, {sy:.2f}) "
                    f"is outside map bounds.",
                )
            if not grid_map.is_valid(gx, gy):
                return (
                    False,
                    f"Region {region.name} rejected: sweep point ({sx:.2f}, {sy:.2f}) "
                    f"is in an obstacle or unknown area.",
                )
            if not inflated_grid.is_valid(gx, gy):
                return (
                    False,
                    f"Region {region.name} rejected: sweep point ({sx:.2f}, {sy:.2f}) "
                    f"is too close to an obstacle (clearance < {self.region_margin:.2f}m).",
                )
        return True, f"Region {region.name} validated"

    def _publish_mission_status(self, message, safety=False):
        msg = String()
        msg.data = f"[SAFETY] {message}" if safety else message
        self.mission_status_pub.publish(msg)
        typed = MissionStatus()
        typed.state = "RUNNING" if self.mission_active else ("FAULT" if safety else "IDLE")
        typed.mode = self.mission_source or "none"
        typed.message = str(message)
        typed.active = bool(self.mission_active)
        typed.safety_warning = bool(safety)
        typed.current_index = int(max(0, self.mission_index))
        typed.total_count = int(len(self.mission_points))
        self.mission_status_typed_pub.publish(typed)

    def _warn_safety(self, message):
        self.get_logger().warn(message)
        self._publish_mission_status(message, safety=True)

    @staticmethod
    def _sweep_positions(start, end, spacing):
        return sweep_positions(start, end, spacing)

    def _assign_path_headings(self, points):
        assign_path_headings(points)

    def _publish_rviz_plan_visuals(self):
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = FRAME_MAP
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        stamp = self.get_clock().now().to_msg()

        if self.mission_active and self.mission_source == "region":
            self._publish_mission_snapshot()
            return

        next_marker_id = 1

        for index, point in enumerate(self.rviz_ordered_points, start=1):
            marker_id = next_marker_id
            next_marker_id += 2

            sphere = Marker()
            sphere.header.frame_id = FRAME_MAP
            sphere.header.stamp = stamp
            sphere.ns = "mission_points"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = point.x
            sphere.pose.position.y = point.y
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.14
            sphere.scale.y = 0.14
            sphere.scale.z = 0.14
            sphere.color.r = 1.0
            sphere.color.g = 0.62
            sphere.color.b = 0.11
            sphere.color.a = 0.95
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = FRAME_MAP
            text.header.stamp = stamp
            text.ns = "mission_labels"
            text.id = marker_id + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = point.x
            text.pose.position.y = point.y + 0.10
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.color.r = 1.0
            text.color.g = 0.85
            text.color.b = 0.30
            text.color.a = 1.0
            text.text = str(index) if self.rviz_solving_method else "…"
            marker_array.markers.append(text)

        if self.pending_region_corner is not None:
            corner_id = next_marker_id
            next_marker_id += 3

            region_index = len(self.inspection_regions) + 1
            cx, cy = self.pending_region_corner
            corner_sphere = Marker()
            corner_sphere.header.frame_id = FRAME_MAP
            corner_sphere.header.stamp = stamp
            corner_sphere.ns = "inspection_region_pending"
            corner_sphere.id = corner_id
            corner_sphere.type = Marker.SPHERE
            corner_sphere.action = Marker.ADD
            corner_sphere.pose.position.x = cx
            corner_sphere.pose.position.y = cy
            corner_sphere.pose.position.z = 0.08
            corner_sphere.pose.orientation.w = 1.0
            corner_sphere.scale.x = 0.18
            corner_sphere.scale.y = 0.18
            corner_sphere.scale.z = 0.18
            corner_sphere.color.r = 0.47
            corner_sphere.color.g = 0.24
            corner_sphere.color.b = 0.72
            corner_sphere.color.a = 0.95
            marker_array.markers.append(corner_sphere)

            pending_label = Marker()
            pending_label.header.frame_id = FRAME_MAP
            pending_label.header.stamp = stamp
            pending_label.ns = "inspection_region_pending"
            pending_label.id = corner_id + 1
            pending_label.type = Marker.TEXT_VIEW_FACING
            pending_label.action = Marker.ADD
            pending_label.pose.position.x = cx
            pending_label.pose.position.y = cy + 0.20
            pending_label.pose.position.z = 0.40
            pending_label.pose.orientation.w = 1.0
            pending_label.scale.z = 0.16
            pending_label.color.r = 0.80
            pending_label.color.g = 0.70
            pending_label.color.b = 0.95
            pending_label.color.a = 1.0
            pending_label.text = f"\u533a\u57df {region_index} \u7b2c1\u89d2"
            marker_array.markers.append(pending_label)

            dot = Marker()
            dot.header.frame_id = FRAME_MAP
            dot.header.stamp = stamp
            dot.ns = "inspection_region_pending"
            dot.id = corner_id + 2
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = cx
            dot.pose.position.y = cy
            dot.pose.position.z = 0.02
            dot.pose.orientation.w = 1.0
            dot.scale.x = 0.06
            dot.scale.y = 0.06
            dot.scale.z = 0.01
            dot.color.r = 0.80
            dot.color.g = 0.70
            dot.color.b = 0.95
            dot.color.a = 0.90
            marker_array.markers.append(dot)

        region_base_id = next_marker_id + 100
        order_by_original_index = {
            original_index: order_index + 1
            for order_index, original_index in enumerate(self.region_ordered_indices)
        }
        for original_index, region in enumerate(self.inspection_regions):
            self._append_region_markers(
                marker_array,
                stamp,
                original_index + 1,
                region,
                region_base_id,
                order_by_original_index.get(original_index)
                if self.region_solving_method
                else None,
            )

        self.marker_pub.publish(marker_array)

        path_msg = Path()
        path_msg.header.frame_id = FRAME_MAP
        path_msg.header.stamp = stamp
        if self.inspection_regions and self.have_map_pose and self.have_map and self.map_msg is not None:
            preview_xy = self._compute_inter_region_preview_path()
        elif self.inspection_regions:
            preview_xy = []
        else:
            preview_xy = self.rviz_preview_path
        for i, (x, y) in enumerate(preview_xy):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            if i + 1 < len(preview_xy):
                nx, ny = preview_xy[i + 1]
                yaw = math.atan2(ny - y, nx - x)
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
            elif i > 0:
                px, py = preview_xy[i - 1]
                yaw = math.atan2(y - py, x - px)
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
            else:
                pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.preview_pub.publish(path_msg)

    def _compute_inter_region_preview_path(self):
        return list(self.region_preview_path)

    def _append_region_markers(
        self, marker_array, stamp, index, region, base_id, display_order=None
    ):
        width = region.max_x - region.min_x
        height = region.max_y - region.min_y
        region_marker_id = base_id + (index - 1) * 20
        blocked = region.name in self.region_blocked_names

        fill = Marker()
        fill.header.frame_id = FRAME_MAP
        fill.header.stamp = stamp
        fill.ns = "inspection_region_fill"
        fill.id = region_marker_id
        fill.type = Marker.CUBE
        fill.action = Marker.ADD
        fill.pose.position.x = (region.min_x + region.max_x) / 2.0
        fill.pose.position.y = (region.min_y + region.max_y) / 2.0
        fill.pose.position.z = 0.01
        fill.pose.orientation.w = 1.0
        fill.scale.x = max(width, 0.01)
        fill.scale.y = max(height, 0.01)
        fill.scale.z = 0.005
        fill.color.r = 0.85 if blocked else 0.47
        fill.color.g = 0.08 if blocked else 0.24
        fill.color.b = 0.06 if blocked else 0.72
        fill.color.a = 0.18
        marker_array.markers.append(fill)

        border = Marker()
        border.header.frame_id = FRAME_MAP
        border.header.stamp = stamp
        border.ns = "inspection_region_border"
        border.id = region_marker_id + 1
        border.type = Marker.LINE_STRIP
        border.action = Marker.ADD
        border.pose.orientation.w = 1.0
        border.scale.x = 0.045
        border.color.r = 0.95 if blocked else 0.65
        border.color.g = 0.12 if blocked else 0.35
        border.color.b = 0.08 if blocked else 0.85
        border.color.a = 0.95
        corners = [
            (region.min_x, region.min_y),
            (region.max_x, region.min_y),
            (region.max_x, region.max_y),
            (region.min_x, region.max_y),
            (region.min_x, region.min_y),
        ]
        for cx, cy in corners:
            border.points.append(self._marker_point(cx, cy, 0.06))
        marker_array.markers.append(border)

        for ci, (cx, cy) in enumerate(corners[:4]):
            corner_dot = Marker()
            corner_dot.header.frame_id = FRAME_MAP
            corner_dot.header.stamp = stamp
            corner_dot.ns = "inspection_region_corners"
            corner_dot.id = region_marker_id + 2 + ci
            corner_dot.type = Marker.SPHERE
            corner_dot.action = Marker.ADD
            corner_dot.pose.position.x = cx
            corner_dot.pose.position.y = cy
            corner_dot.pose.position.z = 0.07
            corner_dot.pose.orientation.w = 1.0
            corner_dot.scale.x = 0.10
            corner_dot.scale.y = 0.10
            corner_dot.scale.z = 0.10
            corner_dot.color.r = 0.65
            corner_dot.color.g = 0.35
            corner_dot.color.b = 0.85
            corner_dot.color.a = 0.95
            marker_array.markers.append(corner_dot)

        label = Marker()
        label.header.frame_id = FRAME_MAP
        label.header.stamp = stamp
        label.ns = "inspection_region_labels"
        label.id = region_marker_id + 6
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = (region.min_x + region.max_x) / 2.0
        label.pose.position.y = (region.min_y + region.max_y) / 2.0
        label.pose.position.z = 0.45
        label.pose.orientation.w = 1.0
        label.scale.z = 0.20
        label.color.r = 0.85
        label.color.g = 0.78
        label.color.b = 0.95
        label.color.a = 1.0
        order_text = str(display_order) if display_order is not None else "\u2026"
        label.text = (
            f"{order_text}. {region.name} \u53d7\u963b"
            if blocked
            else f"{order_text}. {region.name} ({width:.1f}\u00d7{height:.1f})"
        )
        marker_array.markers.append(label)

        region_points = self._generate_points_for_region(region)
        if region_points:
            coverage = Marker()
            coverage.header.frame_id = FRAME_MAP
            coverage.header.stamp = stamp
            coverage.ns = "inspection_thermal_coverage"
            coverage.id = region_marker_id + 7
            coverage.type = Marker.LINE_STRIP
            coverage.action = Marker.ADD
            coverage.pose.orientation.w = 1.0
            coverage.scale.x = max(
                0.02,
                2.0
                * self.thermal_observation_distance
                * math.tan(math.radians(self.thermal_horizontal_fov_deg) * 0.5),
            )
            coverage.color.r = 0.10
            coverage.color.g = 0.75
            coverage.color.b = 0.95
            coverage.color.a = 0.18
            for route_point in region_points:
                coverage.points.append(
                    self._marker_point(route_point.x, route_point.y, 0.065)
                )
            marker_array.markers.append(coverage)

            region_scan = Marker()
            region_scan.header.frame_id = FRAME_MAP
            region_scan.header.stamp = stamp
            region_scan.ns = "inspection_region_scan"
            region_scan.id = region_marker_id + 8
            region_scan.type = Marker.LINE_STRIP
            region_scan.action = Marker.ADD
            region_scan.pose.orientation.w = 1.0
            region_scan.scale.x = 0.04
            region_scan.color.r = 0.05
            region_scan.color.g = 0.72
            region_scan.color.b = 0.42
            region_scan.color.a = 0.95
            for route_point in region_points:
                region_scan.points.append(
                    self._marker_point(route_point.x, route_point.y, 0.08)
                )
            marker_array.markers.append(region_scan)

    @staticmethod
    def _marker_point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _handle_localize(self, _request, response):
        pose = self.current_map_pose if self.have_map_pose else self.current_odom
        response.current_x = pose["x"]
        response.current_y = pose["y"]
        response.current_theta = pose["theta"]
        response.success = self.have_map_pose
        if self.have_map_pose:
            response.message = (
                f"AMCL pose available at ({pose['x']:.2f}, {pose['y']:.2f}, "
                f"{math.degrees(pose['theta']):.1f} deg)"
            )
        elif self.have_odom:
            response.message = (
                "AMCL pose unavailable, returning odom pose. "
                "Set the initial pose in RViz before mission start."
            )
        else:
            response.message = "No odometry received yet."
        return response

    def _handle_confirm_points(self, request, response):
        self.confirmed_points = []
        for point in request.points:
            confirmed = InspectionPoint()
            confirmed.point_name = point.point_name
            confirmed.x = point.x
            confirmed.y = point.y
            confirmed.theta = point.theta
            confirmed.is_confirmed = True
            self.confirmed_points.append(confirmed)

        response.success = True
        response.message = f"Stored {len(self.confirmed_points)} mission points"
        self.get_logger().info(response.message)
        return response

    def _handle_clear_rviz_points(self, _request, response):
        count = len(self.rviz_points)
        self._cancel_pending_rviz_start(
            "Pending RViz mission cancelled because its points were cleared"
        )
        self._invalidate_rviz_plans()
        self.rviz_points = []
        self.rviz_ordered_points = []
        self.rviz_preview_path = []
        self.rviz_solving_method = ""
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Cleared {count} RViz mission points"
        self.get_logger().info(response.message)
        return response

    def _handle_set_region_mode(self, request, response):
        self.region_mode = bool(request.data)
        self.pending_region_corner = None
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = (
            "Region mode enabled. Use RViz Publish Point to click two opposite rectangle corners."
            if self.region_mode
            else "Region mode disabled. RViz Publish Point will add normal mission points."
        )
        self.get_logger().info(response.message)
        return response

    def _handle_clear_inspection_regions(self, _request, response):
        count = len(self.inspection_regions)
        self._cancel_pending_region_start(
            "Pending region mission planning aborted because regions were cleared",
            publish=False,
        )
        self._invalidate_region_plans()
        self.inspection_regions = []
        self.region_preview_points = []
        self.region_generation_error = None
        self.region_ordered_indices = []
        self.region_ordered_paths = []
        self.region_ordered_staging_points = []
        self.region_transition_paths = []
        self.region_return_path = []
        self.region_solving_method = ""
        self.region_preview_path = []
        self.pending_region_corner = None
        self.region_blocked_names = []
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Cleared {count} inspection region(s)"
        self.get_logger().info(response.message)
        return response

    def _handle_save_inspection_regions(self, _request, response):
        data = regions_to_yaml_data(
            self.inspection_regions,
            self.sweep_spacing,
            self.region_margin,
        )
        data.update(
            {
                "thermal_observation_distance": self.thermal_observation_distance,
                "thermal_horizontal_fov_deg": self.thermal_horizontal_fov_deg,
                "thermal_overlap_ratio": self.thermal_overlap_ratio,
                "straight_resolution": self.straight_resolution,
                "arc_resolution": self.arc_resolution,
                "execution_mode": "chassis_primitives",
                "turn_pattern": "rotate_drive_rotate",
                "chassis_linear_speed": self.chassis_linear_speed,
                "chassis_angular_speed": self.chassis_angular_speed,
                "chassis_position_tolerance": self.chassis_position_tolerance,
                "chassis_angle_tolerance": self.chassis_angle_tolerance,
                "chassis_obstacle_distance": self.chassis_obstacle_distance,
                "region_obstacle_wait_sec": self.region_obstacle_wait_sec,
                "region_obstacle_clear_frames": self.region_obstacle_clear_frames,
                "region_obstacle_max_retries": self.region_obstacle_max_retries,
                "region_obstacle_inset": self.region_obstacle_inset,
                "region_staging_distance": self.region_staging_distance,
                "clearance_cluster_distance": self.clearance_cluster_distance,
                "clearance_min_points": self.clearance_min_points,
                "clearance_required_frames": self.clearance_required_frames,
                "clearance_observation_sec": self.clearance_observation_sec,
            }
        )
        try:
            directory = os.path.dirname(self.regions_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.regions_path, "w", encoding="utf-8") as output:
                yaml.safe_dump(data, output, sort_keys=False)
        except Exception as exc:
            response.success = False
            response.message = f"Failed to save inspection regions: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f"Saved {len(self.inspection_regions)} inspection region(s) to {self.regions_path}"
        )
        self.get_logger().info(response.message)
        return response

    def _handle_load_inspection_regions(self, _request, response):
        try:
            with open(self.regions_path, "r", encoding="utf-8") as input_file:
                data = yaml.safe_load(input_file) or {}
            regions = self._regions_from_yaml(data)
        except Exception as exc:
            response.success = False
            response.message = f"Failed to load inspection regions: {exc}"
            self.get_logger().error(response.message)
            return response

        file_version = int(data.get("version", 1))
        self.inspection_regions = regions
        self.pending_region_corner = None
        if file_version >= 2:
            self.sweep_spacing = float(data.get("sweep_spacing", self.sweep_spacing))
            self.region_margin = float(data.get("region_margin", self.region_margin))
            self.thermal_observation_distance = float(
                data.get("thermal_observation_distance", self.thermal_observation_distance)
            )
            self.thermal_horizontal_fov_deg = float(
                data.get("thermal_horizontal_fov_deg", self.thermal_horizontal_fov_deg)
            )
            self.thermal_overlap_ratio = float(
                data.get("thermal_overlap_ratio", self.thermal_overlap_ratio)
            )
            self.straight_resolution = float(
                data.get("straight_resolution", self.straight_resolution)
            )
            self.arc_resolution = float(data.get("arc_resolution", self.arc_resolution))
            self.chassis_linear_speed = float(
                data.get("chassis_linear_speed", self.chassis_linear_speed)
            )
            self.chassis_angular_speed = float(
                data.get("chassis_angular_speed", self.chassis_angular_speed)
            )
            self.chassis_position_tolerance = float(
                data.get("chassis_position_tolerance", self.chassis_position_tolerance)
            )
            self.chassis_angle_tolerance = float(
                data.get("chassis_angle_tolerance", self.chassis_angle_tolerance)
            )
            self.chassis_obstacle_distance = float(
                data.get("chassis_obstacle_distance", self.chassis_obstacle_distance)
            )
            self.region_obstacle_wait_sec = max(
                0.1,
                float(data.get("region_obstacle_wait_sec", self.region_obstacle_wait_sec)),
            )
            self.region_obstacle_clear_frames = max(
                1,
                int(
                    data.get(
                        "region_obstacle_clear_frames",
                        self.region_obstacle_clear_frames,
                    )
                ),
            )
            self.region_obstacle_max_retries = max(
                0,
                int(
                    data.get(
                        "region_obstacle_max_retries",
                        self.region_obstacle_max_retries,
                    )
                ),
            )
            self.region_obstacle_inset = float(
                data.get("region_obstacle_inset", self.region_obstacle_inset)
            )
            self.region_staging_distance = float(
                data.get("region_staging_distance", self.region_staging_distance)
            )
            self.clearance_cluster_distance = float(
                data.get("clearance_cluster_distance", self.clearance_cluster_distance)
            )
            self.clearance_min_points = int(
                data.get("clearance_min_points", self.clearance_min_points)
            )
            self.clearance_required_frames = int(
                data.get("clearance_required_frames", self.clearance_required_frames)
            )
            self.clearance_observation_sec = float(
                data.get("clearance_observation_sec", self.clearance_observation_sec)
            )
        else:
            self.sweep_spacing = 0.10
            self.region_margin = 0.23
            self.thermal_observation_distance = 0.05
            self.thermal_horizontal_fov_deg = 110.0
            self.thermal_overlap_ratio = 0.30
            self.straight_resolution = 0.05
            self.arc_resolution = 0.02
            self.get_logger().warn(
                "Loaded version 1 inspection regions: preserved region geometry and migrated "
                "route settings to 50mm thermal defaults (spacing=0.10m, margin=0.23m). "
                "Save regions to persist version 2."
            )
        self._recompute_region_preview()
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = (
            f"Loaded {len(regions)} inspection region(s) from {self.regions_path} "
            f"with spacing={self.sweep_spacing:.2f}m"
        )
        self.get_logger().info(response.message)
        return response

    def _regions_from_yaml(self, data):
        return regions_from_yaml(data)

    def _publish_zero_cmd(self):
        stop = Twist()
        self.cmd_vel_nav_pub.publish(stop)

    def _handle_region_sensor_failure(self, reason):
        """Stop immediately and skip the region after repeated sensor failures."""
        self._publish_zero_cmd()
        self.tf_consecutive_failures += 1
        if self.tf_consecutive_failures >= 5:
            self._skip_current_region(reason)
            return True
        return False

    def _handle_abort_mission(self, _request, response):
        self._publish_zero_cmd()
        aborted = []
        errors = []
        mission_was_active = self.mission_active
        self.mission_run_id += 1

        if self._cancel_pending_rviz_start(
            "Pending RViz mission planning aborted", publish=False
        ):
            aborted.append("RViz mission planning")
        if self._cancel_pending_region_start(
            "Pending region mission planning aborted", publish=False
        ):
            aborted.append("region mission planning")

        if self.mission_wait_timer is not None:
            self._clear_mission_wait_timer()
            aborted.append("mission wait")

        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                errors.append(f"mission: {exc}")
            else:
                aborted.append("mission")
            self.goal_handle = None

        if mission_was_active and not aborted:
            aborted.append("mission")
        self.mission_active = False
        self._clear_mission_state()

        if self.direct_goal_handle is not None:
            try:
                self.direct_goal_handle.cancel_goal_async()
            except Exception as exc:
                errors.append(f"direct navigation: {exc}")
            else:
                aborted.append("direct navigation")
            self.direct_goal_handle = None
            self.direct_nav_active = False

        if errors:
            response.success = False
            response.message = "Failed to cancel " + "; ".join(errors)
            self.get_logger().error(response.message)
            return response

        if aborted:
            response.success = True
            response.message = f"Abort requested for {', '.join(aborted)} and stop command sent"
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = "No active mission; stop command sent"
            self.get_logger().info(response.message)
        self._publish_mission_status(response.message)
        return response

    def _handle_undo_last_inspection_region(self, _request, response):
        if self.pending_region_corner is not None:
            cx, cy = self.pending_region_corner
            self.pending_region_corner = None
            self._publish_rviz_plan_visuals()
            response.success = True
            response.message = f"Undone pending region corner at ({cx:.2f}, {cy:.2f})"
            self.get_logger().info(response.message)
            return response
        if not self.inspection_regions:
            response.success = False
            response.message = "No inspection regions to undo"
            return response
        removed = self.inspection_regions.pop()
        self._recompute_region_preview()
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Undone region {removed.name}. {len(self.inspection_regions)} region(s) remaining"
        self.get_logger().info(response.message)
        return response

    def _handle_undo_last_rviz_point(self, _request, response):
        if not self.rviz_points:
            response.success = False
            response.message = "No RViz points to undo"
            return response
        self._cancel_pending_rviz_start(
            "Pending RViz mission cancelled because its points changed"
        )
        removed = self.rviz_points.pop()
        self._recompute_rviz_plan()
        response.success = True
        response.message = f"Undone point {removed.point_name}. {len(self.rviz_points)} point(s) remaining"
        self.get_logger().info(response.message)
        return response

    def _handle_start_navigation(self, request, response):
        if self.mission_active:
            response.success = False
            response.message = "A mission is already running"
            return response
        if self.direct_nav_active:
            response.success = False
            response.message = "A direct RViz navigation goal is already running"
            return response
        if self.pending_rviz_start is not None:
            response.success = False
            response.message = "RViz mission planning is already in progress"
            return response
        if self.pending_region_start is not None:
            response.success = False
            response.message = "Region mission planning is already in progress"
            return response

        region_points = []
        if not request.waypoints and self.inspection_regions:
            self._recompute_region_preview()
            region_points = list(self.region_preview_points)
            if self.region_generation_error is not None:
                response.success = False
                response.message = self.region_generation_error
                return response
            if not region_points:
                response.success = False
                response.message = (
                    "Inspection regions are too small to generate sweep waypoints. "
                    f"Check region_margin={self.region_margin:.2f}m."
                )
                return response

        points = list(request.waypoints) if request.waypoints else list(self.confirmed_points)
        source = "request"
        if region_points:
            points = region_points
            source = "region"
        if not points and self.rviz_points:
            points = list(self.rviz_points)
            source = "rviz"
        if not points:
            response.success = False
            response.message = "No mission points available"
            return response

        if not self.have_map_pose:
            response.success = False
            response.message = "AMCL pose unavailable. Set the initial pose before starting a mission."
            return response

        if not self.have_map or self.map_msg is None:
            response.success = False
            response.message = "No /map data received"
            return response

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Nav2 action server /navigate_to_pose is not ready"
            return response

        pause_sec = self._clamp_waypoint_pause(
            getattr(request, "waypoint_pause_sec", self.DEFAULT_WAYPOINT_PAUSE_SEC)
        )
        return_to_start = bool(getattr(request, "return_to_start", False))
        if return_to_start:
            home_valid, home_message = self._validate_mission_home()
            if not home_valid:
                response.success = False
                response.message = home_message
                self.get_logger().warn(f"Mission request rejected: {response.message}")
                self._publish_mission_status(response.message, safety=True)
                return response

        if source == "region":
            location_validation = validate_mission_points(
                self.map_msg,
                (self.current_map_pose["x"], self.current_map_pose["y"]),
                points,
                check_route=False,
            )
            if not location_validation.valid:
                response.success = False
                response.message = (
                    f"Inspection region mission rejected: {location_validation.message}"
                )
                self.get_logger().warn(response.message)
                self._publish_mission_status(response.message, safety=True)
                return response
            self._publish_zero_cmd()
            generation = self._queue_region_plan(
                start_request={"return_to_start": return_to_start}
            )
            if generation is None:
                response.success = False
                response.message = (
                    "Inspection region mission rejected: one or more regions have no valid staging direction"
                )
                return response
            response.success = True
            response.message = (
                f"Region TSP planning accepted for {len(self.inspection_regions)} region(s); "
                "the robot will remain stopped until a current plan is ready; "
                f"return_to_start={str(return_to_start).lower()}"
            )
            self.get_logger().info(response.message)
            self._publish_mission_status(response.message)
            return response

        if source == "rviz":
            location_validation = validate_mission_points(
                self.map_msg,
                (self.current_map_pose["x"], self.current_map_pose["y"]),
                points,
                check_route=False,
            )
            if not location_validation.valid:
                response.success = False
                response.message = location_validation.message
                self.get_logger().warn(response.message)
                self._publish_mission_status(response.message, safety=True)
                return response

            self._publish_zero_cmd()
            self._queue_rviz_plan(
                start_request={
                    "pause_sec": pause_sec,
                    "return_to_start": return_to_start,
                }
            )
            response.success = True
            response.message = (
                f"RViz TSP planning accepted for {len(points)} points; "
                "the robot will remain stopped until a current plan is ready; "
                f"return_to_start={str(return_to_start).lower()}"
            )
            self.get_logger().info(response.message)
            self._publish_mission_status(response.message)
            return response

        validation = validate_mission_points(
            self.map_msg,
            (self.current_map_pose["x"], self.current_map_pose["y"]),
            points,
        )
        if not validation.valid:
            response.success = False
            if source == "region":
                response.message = f"Inspection region mission rejected: {validation.message}"
            else:
                response.message = validation.message
            self.get_logger().warn(f"Mission request rejected: {response.message}")
            self._publish_mission_status(response.message, safety=True)
            return response

        ordered_points = list(points)
        if not self._start_sequential_mission(
            ordered_points, source, pause_sec, return_to_start=return_to_start
        ):
            response.success = False
            response.message = "Failed to start sequential mission"
            return response

        names = " -> ".join(point.point_name for point in ordered_points[:12])
        if len(ordered_points) > 12:
            names += f" -> ... ({len(ordered_points)} total)"
        response.success = True
        response.message = (
            f"Sequential mission started with {len(ordered_points)} points from {source}, "
            f"pause={pause_sec:.1f}s, return_to_start={str(return_to_start).lower()}: {names}"
        )
        self.get_logger().info(response.message)
        self._publish_mission_status(response.message)
        return response

    def _inspection_point_to_pose(self, point):
        return inspection_point_to_pose(point, self.get_clock().now().to_msg())

    @staticmethod
    def _make_inspection_point(name, x, y, theta):
        return make_inspection_point(name, x, y, theta)

    def _clamp_waypoint_pause(self, value):
        try:
            pause_sec = float(value)
        except (TypeError, ValueError):
            pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        if not math.isfinite(pause_sec):
            pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        return max(0.0, min(self.MAX_WAYPOINT_PAUSE_SEC, pause_sec))

    def _validate_mission_home(self):
        values = (self.mission_home_x, self.mission_home_y, self.mission_home_yaw)
        if not all(math.isfinite(value) for value in values):
            return False, "Return-to-start is unavailable: fixed initial pose is invalid"
        grid_map = GridMap.from_occupancy_grid(self.map_msg)
        gx, gy = grid_map.world_to_grid(self.mission_home_x, self.mission_home_y)
        if not grid_map.in_bounds(gx, gy):
            return (
                False,
                "Return-to-start is unavailable: fixed initial pose "
                f"({self.mission_home_x:.2f}, {self.mission_home_y:.2f}) is outside map bounds",
            )
        if not grid_map.is_valid(gx, gy):
            return (
                False,
                "Return-to-start is unavailable: fixed initial pose "
                f"({self.mission_home_x:.2f}, {self.mission_home_y:.2f}) is occupied or unknown",
            )
        return True, "Fixed mission start pose is valid"

    def _start_region_path_mission(
        self,
        regions,
        region_paths,
        staging_points,
        transition_paths,
        solving_method,
        return_to_start=False,
    ):
        self._clear_mission_wait_timer()
        self.region_blocked_names = []
        if (
            not regions
            or len(regions) != len(region_paths)
            or len(regions) != len(staging_points)
            or len(regions) != len(transition_paths)
        ):
            return False
        self.mission_active = True
        self.mission_source = "region"
        self.mission_return_to_start = bool(return_to_start)
        self.mission_returning_home = False
        self.mission_run_id += 1
        self.mission_regions = list(regions)
        self.region_paths = [list(path) for path in region_paths]
        self.region_staging_points = list(staging_points)
        self.region_path_index = 0
        self.region_phase = "approach"
        self._reset_region_obstacle_recovery()
        self.region_solving_method = solving_method

        entries = []
        for region, route, staging, nav2_path in zip(
            self.mission_regions,
            self.region_paths,
            self.region_staging_points,
            transition_paths,
        ):
            entries.append(
                {
                    "region_name": region.name,
                    "staging_point": staging,
                    "chassis_route": route,
                    "nav2_approach_path": list(nav2_path),
                }
            )

        self.cached_region_plan = {"entries": entries}
        self._publish_mission_snapshot()
        return self._send_region_approach_goal()

    def _send_region_approach_goal(self):
        if not self.mission_active or self.region_path_index >= len(self.region_paths):
            return False
        point = self.region_staging_points[self.region_path_index]
        if point is None:
            self._skip_current_region("no reachable staging point outside region")
            return True
        goal = NavigateToPose.Goal()
        goal.pose = self._inspection_point_to_pose(point)
        run_id = self.mission_run_id
        self.region_phase = "approach"
        self.get_logger().info(
            f"Navigating to region staging point {point.point_name} at ({point.x:.2f}, {point.y:.2f})"
        )
        future = self.nav_to_pose_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, rid=run_id: self._region_approach_response_cb(done, rid)
        )
        return True

    def _region_approach_response_cb(self, future, run_id):
        if not self.mission_active or run_id != self.mission_run_id:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self._finish_mission_failed(f"Region approach request failed: {exc}")
            return
        if not handle.accepted:
            self._finish_mission_failed("Region approach goal was rejected by Nav2")
            return
        self.region_approach_goal_handle = handle
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done, rid=run_id, expected=handle: (
                self._region_approach_result_cb(done, rid, expected)
            )
        )

    def _region_approach_result_cb(self, future, run_id, expected_handle):
        if not self.mission_active or run_id != self.mission_run_id:
            return
        if expected_handle is not self.region_approach_goal_handle:
            return
        self.region_approach_goal_handle = None
        self.goal_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self._finish_mission_failed(f"Region approach result failed: {exc}")
            return
        if result.error_code != NavigateToPose.Result.NONE:
            self._skip_current_region(
                f"Nav2 approach failed with code {result.error_code}: "
                f"{result.error_msg or 'no error details'}"
            )
            return
        static_obstacle = self._region_static_obstacle()
        if static_obstacle is not None:
            self._skip_current_region(
                f"static occupied cluster inside region near ({static_obstacle[0]:.2f}, {static_obstacle[1]:.2f})"
            )
            return
        self.region_target_index = 0
        self.region_phase = "clearance"
        self.clearance_started_time = self.get_clock().now()
        self.clearance_obstacle_frames = 0
        self.clearance_last_scan_stamp = None
        self.tf_consecutive_failures = 0
        self._reset_region_obstacle_recovery()
        self._publish_mission_snapshot()
        self._start_region_control_timer()
        self.get_logger().info(
            f"Region {self.region_path_index + 1}: Nav2 released at staging point; "
            "collecting region clearance scans"
        )

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _start_region_control_timer(self):
        self._stop_region_control_timer()
        self.region_control_timer = self.create_timer(0.05, self._region_control_tick)

    def _stop_region_control_timer(self):
        timer = self.region_control_timer
        if timer is None:
            return
        self.region_control_timer = None
        timer.cancel()
        self.destroy_timer(timer)

    def _laser_fresh(self):
        if self.last_scan_time is None:
            return False
        return (self.get_clock().now() - self.last_scan_time).nanoseconds < 500_000_000

    def _update_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                FRAME_MAP,
                FRAME_BASE_LINK,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return False
        t = transform.transform.translation
        r = transform.transform.rotation
        self.current_map_pose["x"] = t.x
        self.current_map_pose["y"] = t.y
        self.current_map_pose["theta"] = quat_to_yaw(r)
        self.have_map_pose = True
        return True

    def _scan_points_in_map(self):
        if self.latest_scan is None:
            return None
        source_frame = self.latest_scan.header.frame_id or "laser"
        try:
            transform = self.tf_buffer.lookup_transform(
                FRAME_MAP,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.10),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Cannot transform laser scan into map: {exc}")
            return None
        points = []
        angle = self.latest_scan.angle_min
        translation = transform.transform.translation
        yaw = quat_to_yaw(transform.transform.rotation)
        for distance in self.latest_scan.ranges:
            if math.isfinite(distance) and self.latest_scan.range_min < distance < self.latest_scan.range_max:
                points.append(
                    (
                        translation.x + distance * math.cos(yaw + angle),
                        translation.y + distance * math.sin(yaw + angle),
                    )
                )
            angle += self.latest_scan.angle_increment
        return points

    def _point_in_current_region(self, x, y):
        region = self.mission_regions[self.region_path_index]
        inset = max(0.0, self.region_obstacle_inset)
        return (
            region.min_x + inset < x < region.max_x - inset
            and region.min_y + inset < y < region.max_y - inset
        )

    def _scan_points_in_current_region(self):
        points = self._scan_points_in_map()
        if points is None:
            return None
        return [(x, y) for x, y in points if self._point_in_current_region(x, y)]

    def _region_static_obstacle(self):
        if self.map_msg is None:
            return None
        grid = GridMap.from_occupancy_grid(self.map_msg)
        region = self.mission_regions[self.region_path_index]
        inset = max(0.0, self.region_obstacle_inset)
        min_gx, min_gy = grid.world_to_grid(region.min_x + inset, region.min_y + inset)
        max_gx, max_gy = grid.world_to_grid(region.max_x - inset, region.max_y - inset)
        occupied = set()
        for gy in range(min_gy, max_gy + 1):
            for gx in range(min_gx, max_gx + 1):
                if not grid.in_bounds(gx, gy):
                    continue
                value = grid.value(gx, gy)
                if value >= grid.occupied_threshold:
                    occupied.add((gx, gy))
        while occupied:
            seed = occupied.pop()
            component = {seed}
            pending = [seed]
            while pending:
                cx, cy = pending.pop()
                for ox in (-1, 0, 1):
                    for oy in (-1, 0, 1):
                        neighbor = (cx + ox, cy + oy)
                        if neighbor in occupied:
                            occupied.remove(neighbor)
                            component.add(neighbor)
                            pending.append(neighbor)
            if len(component) >= 3:
                return grid.grid_to_world(*seed)
        return None

    def _largest_scan_cluster(self, points):
        remaining = list(points)
        largest = []
        threshold = max(0.02, self.clearance_cluster_distance)
        while remaining:
            component = [remaining.pop()]
            pending = list(component)
            while pending:
                px, py = pending.pop()
                connected = [
                    point
                    for point in remaining
                    if math.hypot(point[0] - px, point[1] - py) <= threshold
                ]
                for point in connected:
                    remaining.remove(point)
                    component.append(point)
                    pending.append(point)
            if len(component) > len(largest):
                largest = component
        return largest

    def _clearance_tick(self):
        if not self._laser_fresh():
            self._handle_region_sensor_failure("laser timeout")
            return
        if not self._update_pose_from_tf():
            self._handle_region_sensor_failure(
                "map to base_link transform unavailable"
            )
            return
        self.tf_consecutive_failures = 0
        stamp = self.latest_scan.header.stamp
        stamp_key = (stamp.sec, stamp.nanosec)
        if stamp_key != self.clearance_last_scan_stamp:
            self.clearance_last_scan_stamp = stamp_key
            points = self._scan_points_in_current_region()
            if points is None:
                self.get_logger().warn("clearance scan transform unavailable")
                return
            cluster = self._largest_scan_cluster(points)
            if len(cluster) >= self.clearance_min_points:
                self.clearance_obstacle_frames += 1
                if self.clearance_obstacle_frames >= self.clearance_required_frames:
                    cx = sum(point[0] for point in cluster) / len(cluster)
                    cy = sum(point[1] for point in cluster) / len(cluster)
                    self._begin_region_obstacle_recovery(
                        f"laser obstacle cluster inside region: points={len(cluster)}, "
                        f"frames={self.clearance_obstacle_frames}, center=({cx:.2f}, {cy:.2f})"
                    )
                    return
            else:
                self.clearance_obstacle_frames = 0
        elapsed = (self.get_clock().now() - self.clearance_started_time).nanoseconds / 1e9
        if elapsed >= self.clearance_observation_sec:
            self._reset_region_obstacle_recovery()
            self.region_phase = "rotate"
            self._publish_mission_snapshot()
            self.get_logger().info(
                f"Region {self.region_path_index + 1} clear; chassis inspection program started"
            )

    def _motion_corridor_blocked(self, target):
        px = self.current_map_pose["x"]
        py = self.current_map_pose["y"]
        dx = target.x - px
        dy = target.y - py
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return False
        ux, uy = dx / length, dy / length
        points = self._scan_points_in_current_region()
        if points is None:
            return True
        candidates = []
        for ox, oy in points:
            rx, ry = ox - px, oy - py
            forward = rx * ux + ry * uy
            lateral = abs(rx * uy - ry * ux)
            if 0.0 < forward < min(length + 0.10, self.chassis_obstacle_distance) and lateral < 0.14:
                candidates.append((ox, oy))
        return len(self._largest_scan_cluster(candidates)) >= 2

    def _nearby_obstacle(self, radius=0.20):
        px = self.current_map_pose["x"]
        py = self.current_map_pose["y"]
        points = self._scan_points_in_current_region()
        if points is None:
            return True
        nearby = [(x, y) for x, y in points if math.hypot(x - px, y - py) < radius]
        return len(self._largest_scan_cluster(nearby)) >= 2

    def _reset_region_obstacle_recovery(self, reset_retries=True):
        self.region_obstacle_wait_started = None
        self.region_obstacle_resume_phase = ""
        self.region_obstacle_clear_count = 0
        self.region_obstacle_reason = ""
        self.region_obstacle_last_scan_stamp = None
        if reset_retries:
            self.region_obstacle_retry_count = 0

    def _begin_region_obstacle_recovery(self, reason):
        self._publish_zero_cmd()
        if self.region_phase == "obstacle_wait":
            return True
        if self.region_obstacle_retry_count >= self.region_obstacle_max_retries:
            self._skip_current_region(
                f"obstacle recovery exhausted after {self.region_obstacle_retry_count} "
                f"attempt(s): {reason}"
            )
            return False
        self.region_obstacle_retry_count += 1
        self.region_obstacle_resume_phase = self.region_phase
        self.region_obstacle_wait_started = self.get_clock().now()
        self.region_obstacle_clear_count = 0
        self.region_obstacle_reason = reason
        self.region_phase = "obstacle_wait"
        self._warn_safety(
            f"Region {self.mission_regions[self.region_path_index].name} obstacle detected; "
            f"stopped for clearance observation "
            f"({self.region_obstacle_retry_count}/{self.region_obstacle_max_retries})"
        )
        self._publish_mission_snapshot()
        return True

    def _region_obstacle_still_blocked(self):
        resume_phase = self.region_obstacle_resume_phase
        if resume_phase == "clearance":
            points = self._scan_points_in_current_region()
            if points is None:
                return True
            return len(self._largest_scan_cluster(points)) >= self.clearance_min_points
        if resume_phase == "rotate":
            return self._nearby_obstacle()
        if resume_phase == "drive":
            route = self.region_paths[self.region_path_index]
            if self.region_target_index >= len(route):
                return False
            return self._motion_corridor_blocked(route[self.region_target_index])
        return True

    def _region_obstacle_recovery_tick(self):
        self._publish_zero_cmd()
        if self.region_obstacle_wait_started is None:
            self._skip_current_region("obstacle recovery state is invalid")
            return

        elapsed = (
            self.get_clock().now() - self.region_obstacle_wait_started
        ).nanoseconds / 1e9
        stamp = self.latest_scan.header.stamp
        stamp_key = (stamp.sec, stamp.nanosec)
        if self._region_obstacle_still_blocked():
            self.region_obstacle_clear_count = 0
            self.region_obstacle_last_scan_stamp = stamp_key
            if elapsed >= self.region_obstacle_wait_sec:
                self._skip_current_region(
                    f"obstacle remained for {self.region_obstacle_wait_sec:.1f}s: "
                    f"{self.region_obstacle_reason}"
                )
            return

        if stamp_key == self.region_obstacle_last_scan_stamp:
            return
        self.region_obstacle_last_scan_stamp = stamp_key
        self.region_obstacle_clear_count += 1
        if elapsed >= self.region_obstacle_wait_sec:
            self._skip_current_region(
                f"obstacle clearance was not stable for "
                f"{self.region_obstacle_clear_frames} consecutive frame(s)"
            )
            return
        if self.region_obstacle_clear_count < self.region_obstacle_clear_frames:
            return

        resume_phase = self.region_obstacle_resume_phase
        region_name = self.mission_regions[self.region_path_index].name
        self._reset_region_obstacle_recovery(reset_retries=False)
        self.region_phase = resume_phase
        if resume_phase == "clearance":
            self.clearance_started_time = self.get_clock().now()
            self.clearance_obstacle_frames = 0
            self.clearance_last_scan_stamp = None
        self.get_logger().info(
            f"Region {region_name} obstacle cleared; resuming {resume_phase} phase"
        )
        self._publish_mission_status(
            f"Region {region_name} obstacle cleared; resuming inspection"
        )
        self._publish_mission_snapshot()

    def _region_control_tick(self):
        if not self.mission_active or self.mission_source != "region":
            self._stop_region_control_timer()
            return
        if not self._laser_fresh():
            self._handle_region_sensor_failure("laser timeout")
            return
        if not self._update_pose_from_tf():
            self._handle_region_sensor_failure(
                "map to base_link transform unavailable"
            )
            return
        self.tf_consecutive_failures = 0
        if self.region_phase == "obstacle_wait":
            self._region_obstacle_recovery_tick()
            return
        if self.region_phase == "clearance":
            self._clearance_tick()
            return
        route = self.region_paths[self.region_path_index]
        if self.region_target_index >= len(route):
            self._complete_current_region()
            return
        target = route[self.region_target_index]
        dx = target.x - self.current_map_pose["x"]
        dy = target.y - self.current_map_pose["y"]
        distance = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        error = self._normalize_angle(desired - self.current_map_pose["theta"])
        command = Twist()
        if self.region_phase == "rotate":
            if self._nearby_obstacle():
                self._begin_region_obstacle_recovery(
                    "obstacle detected while turning"
                )
                return
            if abs(error) <= self.chassis_angle_tolerance:
                self.region_phase = "drive"
            else:
                command.angular.z = max(
                    -self.chassis_angular_speed,
                    min(self.chassis_angular_speed, 1.2 * error),
                )
                self.cmd_vel_nav_pub.publish(command)
                return
        if distance <= self.chassis_position_tolerance:
            self._publish_zero_cmd()
            self.region_target_index += 1
            self.region_phase = "rotate"
            self._reset_region_obstacle_recovery()
            self._publish_mission_snapshot()
            return
        if self._motion_corridor_blocked(target):
            self._begin_region_obstacle_recovery(
                "obstacle detected in chassis motion corridor"
            )
            return
        command.linear.x = min(self.chassis_linear_speed, max(0.02, 0.8 * distance))
        command.angular.z = max(-0.20, min(0.20, 1.5 * error))
        self.cmd_vel_nav_pub.publish(command)

    def _skip_current_region(self, reason):
        self._stop_region_control_timer()
        self._publish_zero_cmd()
        region_name = self.mission_regions[self.region_path_index].name
        self.region_blocked_names.append(region_name)
        self._reset_region_obstacle_recovery()
        self._publish_mission_snapshot()
        self._warn_safety(f"Region {region_name} blocked: {reason}; navigating to next region")
        self._publish_rviz_plan_visuals()
        self._advance_region()

    def _complete_current_region(self):
        self._stop_region_control_timer()
        self._publish_zero_cmd()
        self.get_logger().info(
            f"Region {self.region_path_index + 1}/{len(self.region_paths)} chassis inspection completed"
        )
        self._publish_mission_snapshot()
        self._advance_region()

    def _advance_region(self):
        self.region_path_index += 1
        if self.region_path_index >= len(self.region_paths):
            if self.region_blocked_names:
                self._finish_mission_failed(
                    "Region inspection partially completed; blocked: "
                    + ", ".join(self.region_blocked_names)
                )
            else:
                self._finish_mission_success()
            return
        self._send_region_approach_goal()

    def _start_sequential_mission(
        self, ordered_points, source, pause_sec, return_to_start=False
    ):
        self._clear_mission_wait_timer()
        self.goal_handle = None
        self.mission_points = list(ordered_points)
        self.mission_index = 0
        self.mission_source = source
        self.mission_run_id += 1
        self.mission_waypoint_pause_sec = pause_sec
        self.mission_return_to_start = bool(return_to_start)
        self.mission_returning_home = False
        self.last_mission_feedback_log_time = 0.0
        self.mission_early_transition_goal = None
        self.mission_active = True
        return self._send_current_mission_goal()

    def _send_current_mission_goal(self):
        if not self.mission_active:
            return False
        if self.mission_index >= len(self.mission_points):
            self._finish_mission_success()
            return True

        point = self.mission_points[self.mission_index]
        waypoint_number = self.mission_index + 1
        total = len(self.mission_points)
        run_id = self.mission_run_id

        if self.mission_source == "region":
            dx = point.x - self.current_map_pose["x"]
            dy = point.y - self.current_map_pose["y"]
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                theta = self.current_map_pose["theta"]
            else:
                theta = math.atan2(dy, dx)
            goal_point = self._make_inspection_point(point.point_name, point.x, point.y, theta)
        else:
            goal_point = point

        goal = NavigateToPose.Goal()
        goal.pose = self._inspection_point_to_pose(goal_point)
        goal.behavior_tree = ""

        try:
            send_goal_future = self.nav_to_pose_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback_msg, run_id=run_id, index=self.mission_index: (
                    self._mission_feedback_cb(feedback_msg, run_id, index)
                ),
            )
        except Exception as exc:
            self._finish_mission_failed(f"Failed to send mission waypoint {waypoint_number}/{total}: {exc}")
            return False

        send_goal_future.add_done_callback(
            lambda future, run_id=run_id, index=self.mission_index: self._mission_goal_response_cb(
                future, run_id, index
            )
        )
        self.get_logger().info(
            f"Mission waypoint {waypoint_number}/{total} sent to {point.point_name} "
            f"({point.x:.2f}, {point.y:.2f}, {math.degrees(point.theta):.1f} deg)"
        )
        return True

    def _mission_goal_response_cb(self, future, run_id, waypoint_index):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self._is_current_mission_goal(run_id, waypoint_index):
                self._finish_mission_failed(
                    f"Failed to send mission waypoint {waypoint_index + 1}/{len(self.mission_points)}: {exc}"
                )
            return

        if not self._is_current_mission_goal(run_id, waypoint_index):
            if goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warn(f"Failed to cancel stale mission waypoint goal: {exc}")
            return

        if not goal_handle.accepted:
            self._finish_mission_failed(
                f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} was rejected by Nav2"
            )
            return

        self.goal_handle = goal_handle
        self.get_logger().info(
            f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} accepted by Nav2"
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, handle=goal_handle, run_id=run_id, index=waypoint_index: self._mission_result_cb(
                future, handle, run_id, index
            )
        )

    def _mission_feedback_cb(self, feedback_msg, run_id, waypoint_index):
        if not self._is_current_mission_goal(run_id, waypoint_index):
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_mission_feedback_log_time < 2.0:
            return
        self.last_mission_feedback_log_time = now_sec
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} feedback: "
            f"{feedback.distance_remaining:.2f} m left"
        )

    def _mission_result_cb(self, future, goal_handle, run_id, waypoint_index):
        if run_id != self.mission_run_id:
            return
        if goal_handle is not self.goal_handle:
            return
        if not self._is_current_mission_goal(run_id, waypoint_index):
            return

        early_transition_goal = (run_id, waypoint_index)
        is_early_transition_cancel = self.mission_early_transition_goal == early_transition_goal
        if is_early_transition_cancel:
            self.mission_early_transition_goal = None

        self.goal_handle = None
        try:
            result_msg = future.result()
            result = result_msg.result
        except Exception as exc:
            self._finish_mission_failed(f"Mission result retrieval failed: {exc}")
            return

        if result.error_code != NavigateToPose.Result.NONE:
            if not is_early_transition_cancel:
                point_name = self.mission_points[waypoint_index].point_name
                self._finish_mission_failed(
                    f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} "
                    f"({point_name}) failed with code {result.error_code}: {result.error_msg}"
                )
                return

        self._publish_zero_cmd()
        point_name = self.mission_points[waypoint_index].point_name
        self.get_logger().info(
            f"Mission waypoint {waypoint_index + 1}/{len(self.mission_points)} "
            f"({point_name}) reached"
        )

        next_index = waypoint_index + 1
        if next_index >= len(self.mission_points):
            self._finish_mission_success()
            return

        self.mission_index = next_index
        next_point = self.mission_points[self.mission_index]
        pause_sec = self.mission_waypoint_pause_sec
        self.get_logger().info(
            f"Waiting {pause_sec:.1f} s before planning to "
            f"mission waypoint {self.mission_index + 1}/{len(self.mission_points)} "
            f"({next_point.point_name})"
        )
        if pause_sec <= 0.0:
            self._send_current_mission_goal()
            return

        self._clear_mission_wait_timer()
        self.mission_wait_timer = self.create_timer(
            pause_sec,
            self._mission_wait_complete_cb,
        )

    def _mission_wait_complete_cb(self):
        self._clear_mission_wait_timer()
        if not self.mission_active:
            return
        self._send_current_mission_goal()

    def _check_mission_early_transition(self):
        if not self.mission_active or self.mission_source != "region":
            return
        if self.goal_handle is None:
            return
        if self.mission_index >= len(self.mission_points):
            return
        if self.mission_index == len(self.mission_points) - 1:
            return
        if not self.have_map_pose:
            return

        waypoint = self.mission_points[self.mission_index]
        dx = self.current_map_pose["x"] - waypoint.x
        dy = self.current_map_pose["y"] - waypoint.y
        distance = math.hypot(dx, dy)
        if distance >= self.RELAXED_GOAL_DISTANCE_M:
            return

        early_transition_goal = (self.mission_run_id, self.mission_index)
        if self.mission_early_transition_goal == early_transition_goal:
            return

        self.get_logger().info(
            f"Early transition on waypoint {self.mission_index + 1}/{len(self.mission_points)} "
            f"({waypoint.point_name}) at {distance:.3f}m (tolerance {self.RELAXED_GOAL_DISTANCE_M:.2f}m)"
        )
        try:
            self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(f"Failed to request early waypoint transition: {exc}")
            return
        self.mission_early_transition_goal = early_transition_goal

    def _is_current_mission_goal(self, run_id, waypoint_index):
        return (
            self.mission_active
            and run_id == self.mission_run_id
            and waypoint_index == self.mission_index
        )

    def _clear_mission_wait_timer(self):
        timer = self.mission_wait_timer
        if timer is None:
            return
        self.mission_wait_timer = None
        timer.cancel()
        self.destroy_timer(timer)

    def _clear_mission_state(self):
        self._stop_region_control_timer()
        self.mission_points = []
        self.mission_index = 0
        self.mission_source = ""
        self.mission_waypoint_pause_sec = self.DEFAULT_WAYPOINT_PAUSE_SEC
        self.mission_return_to_start = False
        self.mission_returning_home = False
        self.last_mission_feedback_log_time = 0.0
        self.mission_early_transition_goal = None
        self.region_paths = []
        self.mission_regions = []
        self.region_path_index = 0
        self.region_phase = ""
        self.region_approach_goal_handle = None
        self.region_target_index = 0
        self.region_staging_points = []
        self.clearance_started_time = None
        self.clearance_obstacle_frames = 0
        self.clearance_last_scan_stamp = None
        self._reset_region_obstacle_recovery()
        self.tf_consecutive_failures = 0
        self.cached_region_plan = {}
        self._last_mission_snapshot_time = None

    def _publish_region_mission_markers(self, target_array=None):
        if not self.mission_active or self.mission_source != "region":
            return
        if not self.cached_region_plan:
            return
        entries = self.cached_region_plan.get("entries", [])
        if not entries:
            return

        if target_array is None:
            target_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, entry in enumerate(entries):
            chassis_route = entry["chassis_route"]
            staging = entry["staging_point"]
            nav2_path = entry["nav2_approach_path"]

            blocked = entry["region_name"] in self.region_blocked_names
            is_current = i == self.region_path_index
            is_past = i < self.region_path_index
            is_approaching = is_current and self.region_phase == "approach"

            nav2_color_r = 0.1
            nav2_color_g = 0.85
            nav2_color_b = 0.85
            chassis_color_r = 0.05
            chassis_color_g = 0.72
            chassis_color_b = 0.42
            chassis_trim_start = 0

            if blocked:
                nav2_color_r = 0.95
                nav2_color_g = 0.15
                nav2_color_b = 0.15
                chassis_color_r = 0.95
                chassis_color_g = 0.15
                chassis_color_b = 0.15
            elif is_past:
                nav2_color_r = 0.5
                nav2_color_g = 0.5
                nav2_color_b = 0.5
                chassis_color_r = 0.5
                chassis_color_g = 0.5
                chassis_color_b = 0.5
            elif is_current and not is_approaching:
                chassis_color_r = 1.0
                chassis_color_g = 0.75
                chassis_color_b = 0.15
                chassis_trim_start = self.region_target_index

            if nav2_path:
                nav2_line = Marker()
                nav2_line.header.frame_id = FRAME_MAP
                nav2_line.header.stamp = stamp
                nav2_line.ns = "mission_nav2_segments"
                nav2_line.id = 5000 + i
                nav2_line.type = Marker.LINE_STRIP
                nav2_line.action = Marker.ADD
                nav2_line.pose.orientation.w = 1.0
                nav2_line.scale.x = 0.04
                nav2_line.color.r = nav2_color_r
                nav2_line.color.g = nav2_color_g
                nav2_line.color.b = nav2_color_b
                nav2_line.color.a = 0.95
                for x, y in nav2_path:
                    nav2_line.points.append(self._marker_point(x, y, 0.06))
                target_array.markers.append(nav2_line)

            if chassis_route:
                chassis_line = Marker()
                chassis_line.header.frame_id = FRAME_MAP
                chassis_line.header.stamp = stamp
                chassis_line.ns = "mission_chassis_routes"
                chassis_line.id = 6000 + i
                chassis_line.type = Marker.LINE_STRIP
                chassis_line.action = Marker.ADD
                chassis_line.pose.orientation.w = 1.0
                chassis_line.scale.x = 0.05
                chassis_line.color.r = chassis_color_r
                chassis_line.color.g = chassis_color_g
                chassis_line.color.b = chassis_color_b
                chassis_line.color.a = 0.95
                for pi, pt in enumerate(chassis_route):
                    if pi >= chassis_trim_start:
                        chassis_line.points.append(
                            self._marker_point(pt.x, pt.y, 0.08)
                        )
                target_array.markers.append(chassis_line)

            if staging is not None:
                staging_color_r = 0.05
                staging_color_g = 0.72
                staging_color_b = 0.42
                if blocked:
                    staging_color_r = 0.95
                    staging_color_g = 0.15
                    staging_color_b = 0.15
                elif is_past:
                    staging_color_r = 0.5
                    staging_color_g = 0.5
                    staging_color_b = 0.5
                elif is_current:
                    staging_color_r = 1.0
                    staging_color_g = 0.75
                    staging_color_b = 0.15

                staging_sphere = Marker()
                staging_sphere.header.frame_id = FRAME_MAP
                staging_sphere.header.stamp = stamp
                staging_sphere.ns = "mission_staging_points"
                staging_sphere.id = 7000 + i
                staging_sphere.type = Marker.SPHERE
                staging_sphere.action = Marker.ADD
                staging_sphere.pose.position.x = staging.x
                staging_sphere.pose.position.y = staging.y
                staging_sphere.pose.position.z = 0.10
                staging_sphere.pose.orientation.w = 1.0
                staging_sphere.scale.x = 0.12
                staging_sphere.scale.y = 0.12
                staging_sphere.scale.z = 0.12
                staging_sphere.color.r = staging_color_r
                staging_sphere.color.g = staging_color_g
                staging_sphere.color.b = staging_color_b
                staging_sphere.color.a = 0.95
                target_array.markers.append(staging_sphere)

                staging_text = Marker()
                staging_text.header.frame_id = FRAME_MAP
                staging_text.header.stamp = stamp
                staging_text.ns = "mission_staging_points"
                staging_text.id = 7000 + i + 1000
                staging_text.type = Marker.TEXT_VIEW_FACING
                staging_text.action = Marker.ADD
                staging_text.pose.position.x = staging.x
                staging_text.pose.position.y = staging.y + 0.15
                staging_text.pose.position.z = 0.25
                staging_text.pose.orientation.w = 1.0
                staging_text.scale.z = 0.16
                staging_text.color.r = staging_color_r
                staging_text.color.g = staging_color_g
                staging_text.color.b = staging_color_b
                staging_text.color.a = 1.0
                staging_text.text = str(i + 1)
                target_array.markers.append(staging_text)

        self.marker_pub.publish(target_array)

    def _publish_mission_snapshot(self):
        now = self.get_clock().now()
        if hasattr(self, '_last_mission_snapshot_time') and self._last_mission_snapshot_time is not None:
            if (now - self._last_mission_snapshot_time).nanoseconds < 50_000_000:
                return
        self._last_mission_snapshot_time = now
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = FRAME_MAP
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        path_msg = Path()
        path_msg.header.frame_id = FRAME_MAP
        path_msg.header.stamp = stamp
        self.preview_pub.publish(path_msg)

        self._publish_region_mission_markers(target_array=marker_array)

        for index, region in enumerate(self.mission_regions, start=1):
            width = region.max_x - region.min_x
            height = region.max_y - region.min_y
            blocked = region.name in self.region_blocked_names
            is_current = (index - 1) == self.region_path_index
            is_past = (index - 1) < self.region_path_index

            fill_r, fill_g, fill_b = 0.47, 0.24, 0.72
            border_r, border_g, border_b = 0.65, 0.35, 0.85
            if blocked:
                fill_r, fill_g, fill_b = 0.85, 0.08, 0.06
                border_r, border_g, border_b = 0.95, 0.12, 0.08
            elif is_current:
                fill_r, fill_g, fill_b = 0.55, 0.55, 0.20
                border_r, border_g, border_b = 1.0, 0.75, 0.15
            elif is_past:
                border_r, border_g, border_b = 0.5, 0.5, 0.5

            fill = Marker()
            fill.header.frame_id = FRAME_MAP
            fill.header.stamp = stamp
            fill.ns = "inspection_region_fill"
            fill.id = 100 + (index - 1) * 20
            fill.type = Marker.CUBE
            fill.action = Marker.ADD
            fill.pose.position.x = (region.min_x + region.max_x) / 2.0
            fill.pose.position.y = (region.min_y + region.max_y) / 2.0
            fill.pose.position.z = 0.01
            fill.pose.orientation.w = 1.0
            fill.scale.x = max(width, 0.01)
            fill.scale.y = max(height, 0.01)
            fill.scale.z = 0.005
            fill.color.r = fill_r
            fill.color.g = fill_g
            fill.color.b = fill_b
            fill.color.a = 0.18
            marker_array.markers.append(fill)

            border = Marker()
            border.header.frame_id = FRAME_MAP
            border.header.stamp = stamp
            border.ns = "inspection_region_border"
            border.id = 100 + (index - 1) * 20 + 1
            border.type = Marker.LINE_STRIP
            border.action = Marker.ADD
            border.pose.orientation.w = 1.0
            border.scale.x = 0.045
            border.color.r = border_r
            border.color.g = border_g
            border.color.b = border_b
            border.color.a = 0.95
            corners = [
                (region.min_x, region.min_y),
                (region.max_x, region.min_y),
                (region.max_x, region.max_y),
                (region.min_x, region.max_y),
                (region.min_x, region.min_y),
            ]
            for cx, cy in corners:
                border.points.append(self._marker_point(cx, cy, 0.06))
            marker_array.markers.append(border)

            for ci, (cx, cy) in enumerate(corners[:4]):
                corner_dot = Marker()
                corner_dot.header.frame_id = FRAME_MAP
                corner_dot.header.stamp = stamp
                corner_dot.ns = "inspection_region_corners"
                corner_dot.id = 100 + (index - 1) * 20 + 2 + ci
                corner_dot.type = Marker.SPHERE
                corner_dot.action = Marker.ADD
                corner_dot.pose.position.x = cx
                corner_dot.pose.position.y = cy
                corner_dot.pose.position.z = 0.07
                corner_dot.pose.orientation.w = 1.0
                corner_dot.scale.x = 0.10
                corner_dot.scale.y = 0.10
                corner_dot.scale.z = 0.10
                corner_dot.color.r = border_r
                corner_dot.color.g = border_g
                corner_dot.color.b = border_b
                corner_dot.color.a = 0.95
                marker_array.markers.append(corner_dot)

            label = Marker()
            label.header.frame_id = FRAME_MAP
            label.header.stamp = stamp
            label.ns = "inspection_region_labels"
            label.id = 100 + (index - 1) * 20 + 6
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = (region.min_x + region.max_x) / 2.0
            label.pose.position.y = (region.min_y + region.max_y) / 2.0
            label.pose.position.z = 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.r = 0.85
            label.color.g = 0.78
            label.color.b = 0.95
            label.color.a = 1.0
            label.text = f"{index}. {region.name} ({width:.1f}\u00d7{height:.1f})"
            marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)

    def _finish_mission_success(self):
        if self.mission_return_to_start and not self.mission_returning_home:
            self._start_return_to_start()
            return
        self._finalize_mission_success("Mission completed successfully")

    def _start_return_to_start(self):
        self.goal_handle = None
        self._clear_mission_wait_timer()
        self._stop_region_control_timer()
        self._publish_zero_cmd()
        self.mission_returning_home = True
        run_id = self.mission_run_id
        home = self._make_inspection_point(
            "MISSION_HOME",
            self.mission_home_x,
            self.mission_home_y,
            self.mission_home_yaw,
        )
        goal = NavigateToPose.Goal()
        goal.pose = self._inspection_point_to_pose(home)
        goal.behavior_tree = ""
        message = (
            "Inspection completed successfully; returning to fixed start pose "
            f"({self.mission_home_x:.2f}, {self.mission_home_y:.2f}, "
            f"{math.degrees(self.mission_home_yaw):.1f} deg)"
        )
        self.get_logger().info(message)
        self._publish_mission_status(message)
        try:
            future = self.nav_to_pose_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback_msg, rid=run_id: (
                    self._return_home_feedback_cb(feedback_msg, rid)
                ),
            )
        except Exception as exc:
            self._finish_mission_failed(
                f"Inspection completed, but return-to-start request failed: {exc}"
            )
            return
        future.add_done_callback(
            lambda done, rid=run_id: self._return_home_goal_response_cb(done, rid)
        )

    def _return_home_goal_response_cb(self, future, run_id):
        try:
            handle = future.result()
        except Exception as exc:
            if self._is_current_return_home(run_id):
                self._finish_mission_failed(
                    f"Inspection completed, but return-to-start request failed: {exc}"
                )
            return

        if not self._is_current_return_home(run_id):
            if handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warn(
                        f"Failed to cancel stale return-to-start goal: {exc}"
                    )
            return
        if not handle.accepted:
            self._finish_mission_failed(
                "Inspection completed, but return-to-start goal was rejected by Nav2"
            )
            return

        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done, rid=run_id, expected=handle: self._return_home_result_cb(
                done, rid, expected
            )
        )
        self.get_logger().info("Return-to-start goal accepted by Nav2")

    def _return_home_feedback_cb(self, feedback_msg, run_id):
        if not self._is_current_return_home(run_id):
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_mission_feedback_log_time < 2.0:
            return
        self.last_mission_feedback_log_time = now_sec
        self.get_logger().info(
            "Return-to-start feedback: "
            f"{feedback_msg.feedback.distance_remaining:.2f} m left"
        )

    def _return_home_result_cb(self, future, run_id, expected_handle):
        if not self._is_current_return_home(run_id):
            return
        if expected_handle is not self.goal_handle:
            return
        self.goal_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self._finish_mission_failed(
                f"Inspection completed, but return-to-start result failed: {exc}"
            )
            return
        if result.error_code != NavigateToPose.Result.NONE:
            detail = getattr(result, "error_msg", "")
            self._finish_mission_failed(
                "Inspection completed, but return-to-start navigation failed with "
                f"code {result.error_code}: {detail}"
            )
            return
        self._publish_zero_cmd()
        self._finalize_mission_success(
            "Mission completed successfully and robot returned to start"
        )

    def _is_current_return_home(self, run_id):
        return (
            self.mission_active
            and self.mission_returning_home
            and run_id == self.mission_run_id
        )

    def _finalize_mission_success(self, message):
        self.goal_handle = None
        self.mission_active = False
        self._clear_mission_wait_timer()
        self._publish_zero_cmd()
        self.get_logger().info(message)
        self._publish_mission_status(message)
        self._clear_mission_state()

    def _finish_mission_failed(self, message):
        self.goal_handle = None
        self.mission_active = False
        self._clear_mission_wait_timer()
        self._publish_zero_cmd()
        self.get_logger().error(message)
        self._publish_mission_status(message, safety=True)
        self._clear_mission_state()

    def _direct_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.direct_nav_active = False
            self.get_logger().error(f"Failed to send RViz navigation goal: {exc}")
            return

        if not goal_handle.accepted:
            self.direct_nav_active = False
            self.get_logger().error("RViz navigation goal was rejected by Nav2")
            return

        self.direct_goal_handle = goal_handle
        self.get_logger().info("RViz navigation goal accepted by Nav2")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, handle=goal_handle: self._direct_result_cb(future, handle)
        )

    def _direct_feedback_cb(self, feedback_msg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_direct_feedback_log_time < 2.0:
            return
        self.last_direct_feedback_log_time = now_sec
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"RViz navigation feedback: {feedback.distance_remaining:.2f} m left"
        )

    def _direct_result_cb(self, future, goal_handle):
        if goal_handle is not self.direct_goal_handle:
            return
        self.direct_goal_handle = None
        self.direct_nav_active = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"RViz navigation result retrieval failed: {exc}")
            self._publish_zero_cmd()
            return

        if result.error_code == NavigateToPose.Result.NONE:
            self.get_logger().info("RViz navigation goal completed successfully")
        else:
            self.get_logger().error(
                f"RViz navigation goal failed with code {result.error_code}: {result.error_msg}"
            )
            self._publish_zero_cmd()

    def destroy_node(self):
        self.pending_rviz_start = None
        self.pending_region_start = None
        self._invalidate_rviz_plans()
        self._invalidate_region_plans()
        self.rviz_plan_poll_timer.cancel()
        self.rviz_plan_executor.shutdown(wait=False, cancel_futures=True)
        # A launcher shutdown or an unhandled exception must not leave the
        # last navigation command active while the downstream gate is still up.
        context_ok = rclpy.ok()
        if context_ok:
            self._publish_zero_cmd()
        self._stop_region_control_timer()
        if context_ok and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if context_ok and self.direct_goal_handle is not None:
            self.direct_goal_handle.cancel_goal_async()
        self._clear_mission_wait_timer()
        super().destroy_node()

    @staticmethod
    def _quat_to_yaw(orientation):
        return quat_to_yaw(orientation)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
