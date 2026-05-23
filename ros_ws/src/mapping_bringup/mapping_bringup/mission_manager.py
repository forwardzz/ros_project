import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from robot_mission_utils.inspection_planner import plan_mission_order, preview_current_order
from robot_mission_utils.tsp_planner import TSPPlanner
from robot_monitor_interfaces.msg import InspectionPoint
from robot_monitor_interfaces.srv import ConfirmInspectionPoints, Localize, StartNavigation


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

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
        self.goal_handle = None
        self.mission_active = False
        self.tsp_planner = TSPPlanner()

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched_qos)
        self.create_subscription(PointStamped, "/clicked_point", self._clicked_point_cb, 10)
        self.create_subscription(
            PoseStamped, "/mission_goal_pose", self._goal_pose_cb, 10
        )

        self.preview_pub = self.create_publisher(Path, "/mission_preview_path", latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/mission_points_markers", latched_qos)

        self.create_service(Localize, "/localize_robot", self._handle_localize)
        self.create_service(
            ConfirmInspectionPoints,
            "/confirm_inspection_points",
            self._handle_confirm_points,
        )
        self.create_service(StartNavigation, "/start_navigation", self._handle_start_navigation)
        self.create_service(Trigger, "/clear_rviz_points", self._handle_clear_rviz_points)

        self.nav_client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
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
        if self.rviz_points:
            self._recompute_rviz_plan()

    def _map_cb(self, msg):
        self.have_map = True
        self.map_msg = msg
        if self.rviz_points:
            self._recompute_rviz_plan()

    def _clicked_point_cb(self, msg):
        frame_id = msg.header.frame_id or "map"
        if frame_id != "map":
            self.get_logger().warn(
                f"Ignoring RViz point in frame {frame_id}; use RViz Publish Point with Fixed Frame=map"
            )
            return

        point = InspectionPoint()
        point.point_name = f"RVIZ_{len(self.rviz_points) + 1}"
        point.x = float(msg.point.x)
        point.y = float(msg.point.y)
        point.theta = 0.0
        point.is_confirmed = True
        self.rviz_points.append(point)
        self.get_logger().info(
            f"Added RViz mission point {point.point_name} at ({point.x:.2f}, {point.y:.2f})"
        )
        self._recompute_rviz_plan()

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

    def _recompute_rviz_plan(self):
        self.rviz_ordered_points = list(self.rviz_points)
        self.rviz_preview_path = []
        if not self.rviz_points:
            self._publish_rviz_plan_visuals()
            return

        if self.map_msg is not None and self.have_map_pose:
            advanced_plan = plan_mission_order(
                self.map_msg,
                (self.current_map_pose["x"], self.current_map_pose["y"]),
                self.rviz_points,
            )
            if advanced_plan:
                self.rviz_ordered_points = [
                    self.rviz_points[index] for index in advanced_plan.ordered_indices
                ]
                self.rviz_preview_path = list(advanced_plan.preview_path)
            else:
                preview = preview_current_order(
                    self.map_msg,
                    (self.current_map_pose["x"], self.current_map_pose["y"]),
                    self.rviz_points,
                )
                if preview:
                    self.rviz_preview_path = list(preview.preview_path)
        self._publish_rviz_plan_visuals()

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

        self.marker_pub.publish(marker_array)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = stamp
        for x, y in self.rviz_preview_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.preview_pub.publish(path_msg)

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

    def _handle_start_navigation(self, request, response):
        if self.mission_active:
            response.success = False
            response.message = "A mission is already running"
            return response

        points = list(request.waypoints) if request.waypoints else list(self.confirmed_points)
        source = "request"
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

        ordered_points = self._order_points(points)
        if source == "rviz" and self.rviz_ordered_points:
            ordered_points = list(self.rviz_ordered_points)

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._inspection_point_to_pose(point) for point in ordered_points]

        send_goal_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)
        self.mission_active = True

        names = " -> ".join(point.point_name for point in ordered_points)
        response.success = True
        response.message = f"Mission goal sent with {len(ordered_points)} points from {source}: {names}"
        self.get_logger().info(response.message)
        return response

    def _order_points(self, points):
        if len(points) < 2:
            return points

        advanced_plan = None
        if self.map_msg is not None and self.have_map_pose:
            advanced_plan = plan_mission_order(
                self.map_msg,
                (self.current_map_pose["x"], self.current_map_pose["y"]),
                points,
            )
        if advanced_plan:
            ordered = [points[index] for index in advanced_plan.ordered_indices]
            self.get_logger().info(
                "Advanced mission planner selected order: %s"
                % " -> ".join(point.point_name for point in ordered)
            )
            return ordered

        anchor = InspectionPoint()
        anchor.point_name = "ROBOT"
        anchor.x = self.current_map_pose["x"]
        anchor.y = self.current_map_pose["y"]
        anchor.theta = self.current_map_pose["theta"]
        anchor.is_confirmed = True

        candidates = [anchor, *points]
        self.tsp_planner.calculate_distance_matrix(candidates)
        order = self.tsp_planner.solve_tsp_dynamic_programming(candidates)
        if not order:
            self.get_logger().warn("Mission planner fallback kept the original point order")
            return points
        ordered = [candidates[index] for index in order if index != 0]
        self.get_logger().warn(
            "Advanced mission planner unavailable, falling back to Euclidean TSP order"
        )
        return ordered

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

    def destroy_node(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
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
