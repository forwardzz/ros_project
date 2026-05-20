import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

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
        self.confirmed_points = []
        self.goal_handle = None
        self.mission_active = False
        self.tsp_planner = TSPPlanner()

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched_qos)

        self.create_service(Localize, "/localize_robot", self._handle_localize)
        self.create_service(
            ConfirmInspectionPoints,
            "/confirm_inspection_points",
            self._handle_confirm_points,
        )
        self.create_service(
            StartNavigation, "/start_navigation", self._handle_start_navigation
        )

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

    def _map_cb(self, _msg):
        self.have_map = True

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

    def _handle_start_navigation(self, request, response):
        if self.mission_active:
            response.success = False
            response.message = "A mission is already running"
            return response

        points = list(request.waypoints) if request.waypoints else list(self.confirmed_points)
        if not points:
            response.success = False
            response.message = "No mission points available"
            return response

        if not self.have_map_pose:
            response.success = False
            response.message = "AMCL pose unavailable. Set the initial pose before starting a mission."
            return response

        if not self.have_map:
            response.success = False
            response.message = "No /map data received"
            return response

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Nav2 action server /navigate_through_poses is not ready"
            return response

        ordered_points = self._order_points(points)
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._inspection_point_to_pose(point) for point in ordered_points]

        send_goal_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)
        self.mission_active = True

        names = " -> ".join(point.point_name for point in ordered_points)
        response.success = True
        response.message = f"Mission goal sent with {len(ordered_points)} points: {names}"
        self.get_logger().info(response.message)
        return response

    def _order_points(self, points):
        if len(points) < 2:
            return points

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
            return points
        return [candidates[index] for index in order if index != 0]

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
