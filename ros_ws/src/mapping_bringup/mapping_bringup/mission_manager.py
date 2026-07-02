from dataclasses import dataclass
import math
import os

import yaml

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from robot_mission_utils.inspection_planner import (
    preview_current_order,
    validate_mission_points,
)
from robot_monitor_interfaces.msg import InspectionPoint
from robot_monitor_interfaces.srv import ConfirmInspectionPoints, Localize, StartNavigation


@dataclass(frozen=True)
class InspectionRegion:
    name: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.sweep_spacing = float(self.declare_parameter("sweep_spacing", 0.30).value)
        self.region_margin = float(self.declare_parameter("region_margin", 0.15).value)
        self.regions_path = str(
            self.declare_parameter(
                "inspection_regions_path",
                "/home/yy/ros2_ws/config/inspection_regions.yaml",
            ).value
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        self.rviz_recompute_throttle_sec = 0.5
        self.last_rviz_recompute_time = 0.0
        self.region_mode = False
        self.pending_region_corner = None
        self.inspection_regions = []
        self.region_preview_points = []
        self.region_generation_error = None
        self.goal_handle = None
        self.mission_active = False
        self.direct_goal_handle = None
        self.direct_nav_active = False
        self.last_direct_feedback_log_time = 0.0

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched_qos)
        self.create_subscription(PointStamped, "/clicked_point", self._clicked_point_cb, 10)
        self.create_subscription(
            PoseStamped, "/mission_goal_pose", self._goal_pose_cb, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self._direct_goal_pose_cb, 10)

        self.preview_pub = self.create_publisher(Path, "/mission_preview_path", latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/mission_points_markers", latched_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        self.create_service(Localize, "/localize_robot", self._handle_localize)
        self.create_service(
            ConfirmInspectionPoints,
            "/confirm_inspection_points",
            self._handle_confirm_points,
        )
        self.create_service(StartNavigation, "/start_navigation", self._handle_start_navigation)
        self.create_service(Trigger, "/clear_rviz_points", self._handle_clear_rviz_points)
        self.create_service(SetBool, "/set_region_mode", self._handle_set_region_mode)
        self.create_service(
            Trigger,
            "/clear_inspection_regions",
            self._handle_clear_inspection_regions,
        )
        self.create_service(
            Trigger,
            "/save_inspection_regions",
            self._handle_save_inspection_regions,
        )
        self.create_service(
            Trigger,
            "/load_inspection_regions",
            self._handle_load_inspection_regions,
        )
        self.create_service(Trigger, "/abort_mission", self._handle_abort_mission)

        self.nav_client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Mission manager ready")

    def _odom_cb(self, msg):
        self.current_odom["x"] = msg.pose.pose.position.x
        self.current_odom["y"] = msg.pose.pose.position.y
        self.current_odom["theta"] = self._quat_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def _amcl_pose_cb(self, msg):
        self.current_map_pose["x"] = msg.pose.pose.position.x
        self.current_map_pose["y"] = msg.pose.pose.position.y
        self.current_map_pose["theta"] = self._quat_to_yaw(msg.pose.pose.orientation)
        self.have_map_pose = True
        if self.rviz_points and not self.mission_active:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_rviz_recompute_time >= self.rviz_recompute_throttle_sec:
                self.last_rviz_recompute_time = now_sec
                self._recompute_rviz_plan()

    def _map_cb(self, msg):
        self.have_map = True
        self.map_msg = msg
        if self.rviz_points:
            self._recompute_rviz_plan()
        elif self.inspection_regions:
            self._recompute_region_preview()
            self._publish_rviz_plan_visuals()

    def _clicked_point_cb(self, msg):
        frame_id = msg.header.frame_id or "map"
        if frame_id != "map":
            self.get_logger().warn(
                f"Ignoring RViz point in frame {frame_id}; use RViz Publish Point with Fixed Frame=map"
            )
            return

        if self.region_mode:
            self._add_region_corner(float(msg.point.x), float(msg.point.y))
            return

        point = self._make_inspection_point(
            f"RVIZ_{len(self.rviz_points) + 1}",
            float(msg.point.x),
            float(msg.point.y),
            0.0,
        )
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
        self.get_logger().info(
            f"Added inspection region {region.name}: "
            f"({region.min_x:.2f}, {region.min_y:.2f}) to ({region.max_x:.2f}, {region.max_y:.2f})"
        )
        self._recompute_region_preview()
        self._publish_rviz_plan_visuals()

    def _goal_pose_cb(self, msg):
        frame_id = msg.header.frame_id or "map"
        if frame_id != "map":
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
        point.theta = self._quat_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f"Updated heading for {point.point_name} to {math.degrees(point.theta):.1f} deg"
        )
        self._publish_rviz_plan_visuals()

    def _direct_goal_pose_cb(self, msg):
        frame_id = msg.header.frame_id or "map"
        if frame_id != "map":
            self.get_logger().warn(
                f"Ignoring RViz navigation goal in frame {frame_id}; use Fixed Frame=map"
            )
            return

        if self.mission_active:
            self.get_logger().warn(
                "Ignoring RViz navigation goal because an inspection mission is already running"
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

        goal_point = self._make_inspection_point(
            "RVIZ_NAV_GOAL",
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            self._quat_to_yaw(msg.pose.orientation),
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
        if not self.rviz_points:
            self._publish_rviz_plan_visuals()
            return

        if self.map_msg is not None and self.have_map_pose:
            preview = preview_current_order(
                self.map_msg,
                (self.current_map_pose["x"], self.current_map_pose["y"]),
                self.rviz_points,
            )
            if preview:
                self.rviz_preview_path = list(preview.preview_path)
        self._publish_rviz_plan_visuals()

    def _recompute_region_preview(self):
        self.region_preview_points = self._generate_region_points()

    def _generate_region_points(self):
        generated = []
        self.region_generation_error = None
        for region in self.inspection_regions:
            region_points = self._generate_points_for_region(region)
            for point_index, (x, y) in enumerate(region_points, start=1):
                generated.append(
                    self._make_inspection_point(
                        f"{region.name}_P{point_index}",
                        x,
                        y,
                        0.0,
                    )
                )
            if not region_points:
                message = (
                    f"{region.name} is too small for spacing={self.sweep_spacing:.2f}m "
                    f"and margin={self.region_margin:.2f}m"
                )
                if self.region_generation_error is None:
                    self.region_generation_error = message
                self.get_logger().warn(message)

        self._assign_path_headings(generated)
        return generated

    def _generate_points_for_region(self, region):
        min_x = region.min_x + self.region_margin
        min_y = region.min_y + self.region_margin
        max_x = region.max_x - self.region_margin
        max_y = region.max_y - self.region_margin
        if min_x > max_x or min_y > max_y:
            return []

        width = max_x - min_x
        height = max_y - min_y
        spacing = max(self.sweep_spacing, 0.05)

        points = []
        if width >= height:
            rows = self._sweep_positions(min_y, max_y, spacing)
            for row_index, y in enumerate(rows):
                if row_index % 2 == 0:
                    points.append((min_x, y))
                    if width > 0.02:
                        points.append((max_x, y))
                else:
                    points.append((max_x, y))
                    if width > 0.02:
                        points.append((min_x, y))
        else:
            columns = self._sweep_positions(min_x, max_x, spacing)
            for col_index, x in enumerate(columns):
                if col_index % 2 == 0:
                    points.append((x, min_y))
                    if height > 0.02:
                        points.append((x, max_y))
                else:
                    points.append((x, max_y))
                    if height > 0.02:
                        points.append((x, min_y))
        return points

    @staticmethod
    def _sweep_positions(start, end, spacing):
        if start > end:
            return []
        positions = []
        value = start
        while value <= end + 1e-9:
            positions.append(value)
            value += spacing
        if not positions or end - positions[-1] > min(spacing * 0.5, 0.10):
            positions.append(end)
        return positions

    def _assign_path_headings(self, points):
        for index, point in enumerate(points):
            target = None
            if index + 1 < len(points):
                target = points[index + 1]
            elif index > 0:
                target = point
                point = points[index - 1]
            if target is None:
                continue
            dx = target.x - point.x
            dy = target.y - point.y
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                points[index].theta = math.atan2(dy, dx)

    def _publish_rviz_plan_visuals(self):
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = "map"
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(self.rviz_ordered_points, start=1):
            marker_id = index * 3
            sphere = Marker()
            sphere.header.frame_id = "map"
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
            text.header.frame_id = "map"
            text.header.stamp = stamp
            text.ns = "mission_labels"
            text.id = marker_id + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = point.x
            text.pose.position.y = point.y
            text.pose.position.z = 0.28
            text.pose.orientation.w = 1.0
            text.scale.z = 0.14
            text.color.r = 0.16
            text.color.g = 0.21
            text.color.b = 0.25
            text.color.a = 1.0
            text.text = f"{index}:{point.point_name} ({math.degrees(point.theta):.0f}deg)"
            marker_array.markers.append(text)

            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = stamp
            arrow.ns = "mission_heading"
            arrow.id = marker_id + 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = point.x
            arrow.pose.position.y = point.y
            arrow.pose.position.z = 0.08
            arrow.pose.orientation.z = math.sin(point.theta / 2.0)
            arrow.pose.orientation.w = math.cos(point.theta / 2.0)
            arrow.scale.x = 0.24
            arrow.scale.y = 0.05
            arrow.scale.z = 0.07
            arrow.color.r = 0.16
            arrow.color.g = 0.53
            arrow.color.b = 0.90
            arrow.color.a = 0.95
            marker_array.markers.append(arrow)

        if self.pending_region_corner is not None:
            corner = Marker()
            corner.header.frame_id = "map"
            corner.header.stamp = stamp
            corner.ns = "inspection_region_pending"
            corner.id = 9000
            corner.type = Marker.SPHERE
            corner.action = Marker.ADD
            corner.pose.position.x = self.pending_region_corner[0]
            corner.pose.position.y = self.pending_region_corner[1]
            corner.pose.position.z = 0.08
            corner.pose.orientation.w = 1.0
            corner.scale.x = 0.18
            corner.scale.y = 0.18
            corner.scale.z = 0.18
            corner.color.r = 0.47
            corner.color.g = 0.24
            corner.color.b = 0.72
            corner.color.a = 0.95
            marker_array.markers.append(corner)

        for index, region in enumerate(self.inspection_regions, start=1):
            self._append_region_markers(marker_array, stamp, index, region)

        if self.region_preview_points:
            preview_marker = Marker()
            preview_marker.header.frame_id = "map"
            preview_marker.header.stamp = stamp
            preview_marker.ns = "inspection_region_preview"
            preview_marker.id = 9500
            preview_marker.type = Marker.LINE_STRIP
            preview_marker.action = Marker.ADD
            preview_marker.pose.orientation.w = 1.0
            preview_marker.scale.x = 0.035
            preview_marker.color.r = 0.05
            preview_marker.color.g = 0.62
            preview_marker.color.b = 0.38
            preview_marker.color.a = 0.95
            for path_point in self.region_preview_points:
                preview_marker.points.append(self._marker_point(path_point.x, path_point.y, 0.07))
            marker_array.markers.append(preview_marker)

        self.marker_pub.publish(marker_array)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = stamp
        if self.inspection_regions:
            preview_xy = [(point.x, point.y) for point in self.region_preview_points]
        else:
            preview_xy = self.rviz_preview_path
        for x, y in preview_xy:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.preview_pub.publish(path_msg)

    def _append_region_markers(self, marker_array, stamp, index, region):
        base_id = 10000 + index * 10

        border = Marker()
        border.header.frame_id = "map"
        border.header.stamp = stamp
        border.ns = "inspection_regions"
        border.id = base_id
        border.type = Marker.LINE_STRIP
        border.action = Marker.ADD
        border.pose.orientation.w = 1.0
        border.scale.x = 0.045
        border.color.r = 0.47
        border.color.g = 0.24
        border.color.b = 0.72
        border.color.a = 0.95
        corners = [
            (region.min_x, region.min_y),
            (region.max_x, region.min_y),
            (region.max_x, region.max_y),
            (region.min_x, region.max_y),
            (region.min_x, region.min_y),
        ]
        for x, y in corners:
            border.points.append(self._marker_point(x, y, 0.05))
        marker_array.markers.append(border)

        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = stamp
        label.ns = "inspection_region_labels"
        label.id = base_id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = (region.min_x + region.max_x) / 2.0
        label.pose.position.y = (region.min_y + region.max_y) / 2.0
        label.pose.position.z = 0.35
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color.r = 0.16
        label.color.g = 0.21
        label.color.b = 0.25
        label.color.a = 1.0
        label.text = f"{index}:{region.name}"
        marker_array.markers.append(label)

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
        self.rviz_points = []
        self.rviz_ordered_points = []
        self.rviz_preview_path = []
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
        self.inspection_regions = []
        self.region_preview_points = []
        self.region_generation_error = None
        self.pending_region_corner = None
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Cleared {count} inspection region(s)"
        self.get_logger().info(response.message)
        return response

    def _handle_save_inspection_regions(self, _request, response):
        data = {
            "version": 1,
            "map_frame": "map",
            "sweep_spacing": self.sweep_spacing,
            "region_margin": self.region_margin,
            "regions": [
                {
                    "name": region.name,
                    "min_x": region.min_x,
                    "min_y": region.min_y,
                    "max_x": region.max_x,
                    "max_y": region.max_y,
                }
                for region in self.inspection_regions
            ],
        }
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

        self.inspection_regions = regions
        self.pending_region_corner = None
        self.sweep_spacing = float(data.get("sweep_spacing", self.sweep_spacing))
        self.region_margin = float(data.get("region_margin", self.region_margin))
        self._recompute_region_preview()
        self._publish_rviz_plan_visuals()
        response.success = True
        response.message = f"Loaded {len(regions)} inspection region(s) from {self.regions_path}"
        self.get_logger().info(response.message)
        return response

    def _regions_from_yaml(self, data):
        if int(data.get("version", 1)) != 1:
            raise ValueError("unsupported inspection region file version")
        if data.get("map_frame", "map") != "map":
            raise ValueError("inspection region file must use map_frame=map")

        regions = []
        for index, item in enumerate(data.get("regions", []), start=1):
            min_x = float(item["min_x"])
            min_y = float(item["min_y"])
            max_x = float(item["max_x"])
            max_y = float(item["max_y"])
            regions.append(
                InspectionRegion(
                    name=str(item.get("name") or f"REGION_{index}"),
                    min_x=min(min_x, max_x),
                    min_y=min(min_y, max_y),
                    max_x=max(min_x, max_x),
                    max_y=max(min_y, max_y),
                )
            )
        return regions

    def _publish_zero_cmd(self):
        stop = Twist()
        self.cmd_vel_nav_pub.publish(stop)
        self.cmd_vel_pub.publish(stop)

    def _handle_abort_mission(self, _request, response):
        self._publish_zero_cmd()
        aborted = []
        errors = []
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                errors.append(f"mission: {exc}")
            else:
                aborted.append("mission")
            self.goal_handle = None
            self.mission_active = False

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
            return response

        self.mission_active = False
        self.direct_nav_active = False
        response.success = True
        response.message = "No active mission. Stop command sent"
        self.get_logger().warn(response.message)
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

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Nav2 action server /navigate_through_poses is not ready"
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
            return response

        ordered_points = list(points)

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._inspection_point_to_pose(point) for point in ordered_points]

        send_goal_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)
        self.mission_active = True

        names = " -> ".join(point.point_name for point in ordered_points[:12])
        if len(ordered_points) > 12:
            names += f" -> ... ({len(ordered_points)} total)"
        response.success = True
        if source == "region":
            response.message = (
                f"Region inspection mission sent with {len(ordered_points)} sweep waypoint(s) "
                f"from {len(self.inspection_regions)} region(s): {names}"
            )
        else:
            response.message = f"Mission goal sent with {len(ordered_points)} points from {source}: {names}"
        self.get_logger().info(response.message)
        return response

    def _inspection_point_to_pose(self, point):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(point.theta / 2.0)
        pose.pose.orientation.w = math.cos(point.theta / 2.0)
        return pose

    @staticmethod
    def _make_inspection_point(name, x, y, theta):
        point = InspectionPoint()
        point.point_name = name
        point.x = float(x)
        point.y = float(y)
        point.theta = float(theta)
        point.is_confirmed = True
        return point

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.mission_active = False
            self.get_logger().error(f"Failed to send mission goal: {exc}")
            return

        if not goal_handle.accepted:
            self.mission_active = False
            self.get_logger().error("Mission goal was rejected by Nav2")
            return

        self.goal_handle = goal_handle
        self.get_logger().info("Mission accepted by Nav2")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Mission feedback: {feedback.number_of_poses_remaining} poses remaining, "
            f"{feedback.distance_remaining:.2f} m left"
        )

    def _result_cb(self, future):
        self.goal_handle = None
        self.mission_active = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"Mission result retrieval failed: {exc}")
            return

        if result.error_code == NavigateThroughPoses.Result.NONE:
            self.get_logger().info("Mission completed successfully")
        else:
            self.get_logger().error(
                f"Mission failed with code {result.error_code}: {result.error_msg}"
            )

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
        result_future.add_done_callback(self._direct_result_cb)

    def _direct_feedback_cb(self, feedback_msg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_direct_feedback_log_time < 2.0:
            return
        self.last_direct_feedback_log_time = now_sec
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"RViz navigation feedback: {feedback.distance_remaining:.2f} m left"
        )

    def _direct_result_cb(self, future):
        self.direct_goal_handle = None
        self.direct_nav_active = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"RViz navigation result retrieval failed: {exc}")
            return

        if result.error_code == NavigateToPose.Result.NONE:
            self.get_logger().info("RViz navigation goal completed successfully")
        else:
            self.get_logger().error(
                f"RViz navigation goal failed with code {result.error_code}: {result.error_msg}"
            )

    def destroy_node(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.direct_goal_handle is not None:
            self.direct_goal_handle.cancel_goal_async()
        super().destroy_node()

    @staticmethod
    def _quat_to_yaw(orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
