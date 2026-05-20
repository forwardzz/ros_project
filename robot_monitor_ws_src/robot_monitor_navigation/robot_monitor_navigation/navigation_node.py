import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
import time

from robot_monitor_interfaces.srv import Localize, StartNavigation, ConfirmInspectionPoints
from robot_monitor_interfaces.msg import InspectionPoint
from .astar_planner import AStarPlanner
from .tsp_planner import TSPPlanner


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')

        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_latched
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.manual_control_sub = self.create_subscription(
            Bool,
            '/manual_control_active',
            self.manual_control_callback,
            10
        )

        self.localize_srv = self.create_service(
            Localize,
            '/localize_robot',
            self.handle_localize
        )

        self.navigation_srv = self.create_service(
            StartNavigation,
            '/start_navigation',
            self.handle_start_navigation
        )

        self.confirm_points_srv = self.create_service(
            ConfirmInspectionPoints,
            '/confirm_inspection_points',
            self.handle_confirm_points
        )

        self.astar_planner = AStarPlanner()
        self.tsp_planner = TSPPlanner()

        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.is_localized = False
        self.is_navigating = False
        self.manual_override = False
        self.map_received = False

        self.inspection_points = []
        self.current_nav_path = []
        self.current_path_index = 0

        self._localize_client = self.create_client(Localize, '/localize_robot')
        while not self._localize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for localize service...')

        self.get_logger().info('Navigation node started successfully')

    def map_callback(self, msg):
        self.map_received = True
        self.astar_planner.set_map(msg)
        self.get_logger().info('Map received and planner updated')

    def odom_callback(self, msg):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_pose['theta'] = self._quaternion_to_euler(q.w, q.x, q.y, q.z)

    def manual_control_callback(self, msg):
        self.manual_override = msg.data
        if self.manual_override:
            self.get_logger().warn('Manual control activated - pausing auto navigation')
            self.cmd_vel_pub.publish(Twist())
        else:
            self.get_logger().info('Manual control released - resuming auto navigation')

    def _quaternion_to_euler(self, w, x, y, z):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return yaw

    def handle_localize(self, request, response):
        self.get_logger().info('Localization requested')

        self.is_localized = False

        self._publish_initial_pose_estimate()

        time.sleep(2.0)

        self._wait_for_localization_convergence()

        self.is_localized = True
        response.success = True
        response.message = f"Localization successful at ({self.current_pose['x']:.2f}, {self.current_pose['y']:.2f})"
        response.current_x = self.current_pose['x']
        response.current_y = self.current_pose['y']
        response.current_theta = self.current_pose['theta']

        self.get_logger().info(f"Localization complete: {response.message}")
        return response

    def _publish_initial_pose_estimate(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.pose.pose.position.x = self.current_pose['x']
        initial_pose_msg.pose.pose.position.y = self.current_pose['y']
        initial_pose_msg.pose.pose.position.z = 0.0
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0

        for i in range(36):
            initial_pose_msg.pose.covariance[i] = 0.0
        initial_pose_msg.pose.covariance[0] = 0.5 * 0.5
        initial_pose_msg.pose.covariance[7] = 0.5 * 0.5
        initial_pose_msg.pose.covariance[35] = (math.pi / 12.0) ** 2

    def _wait_for_localization_convergence(self):
        start_time = time.time()
        timeout = 30.0

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if abs(self.current_pose['x']) < 0.01 and abs(self.current_pose['y']) < 0.01:
                self.get_logger().info('Localization converged')
                return True

        self.get_logger().warn('Localization timeout, proceeding anyway')
        return False

    def handle_confirm_points(self, request, response):
        self.get_logger().info(f'Received {len(request.points)} inspection points for confirmation')

        confirmed_points = []
        for point in request.points:
            confirmed_point = InspectionPoint()
            confirmed_point.point_name = point.point_name
            confirmed_point.x = point.x
            confirmed_point.y = point.y
            confirmed_point.theta = point.theta
            confirmed_point.is_confirmed = True
            confirmed_points.append(confirmed_point)
            self.get_logger().info(f"Confirmed point: {point.point_name} at ({point.x:.2f}, {point.y:.2f})")

        self.inspection_points = confirmed_points

        response.success = True
        response.message = f"Successfully confirmed {len(confirmed_points)} inspection points"

        return response

    def handle_start_navigation(self, request, response):
        if not self.is_localized:
            response.success = False
            response.message = "Robot not localized. Please localize first."
            self.get_logger().error(response.message)
            return response

        if not self.map_received:
            response.success = False
            response.message = "Map not received. Cannot plan path."
            self.get_logger().error(response.message)
            return response

        waypoints = request.waypoints
        if not waypoints:
            waypoints = self.inspection_points

        if not waypoints:
            response.success = False
            response.message = "No waypoints provided for navigation"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(f'Starting navigation with {len(waypoints)} waypoints')

        self.tsp_planner.calculate_distance_matrix(waypoints)

        if len(waypoints) <= 10:
            tsp_order = self.tsp_planner.solve_tsp_brute_force(waypoints)
        else:
            tsp_order = self.tsp_planner.solve_tsp_dynamic_programming(waypoints)

        ordered_waypoints = [waypoints[i] for i in tsp_order]

        self.get_logger().info('TSP planning complete. Starting navigation...')

        response.success = True
        response.message = f"Navigation started with {len(ordered_waypoints)} waypoints in optimal order"

        self.is_navigating = True
        self._execute_navigation(ordered_waypoints)

        return response

    def _execute_navigation(self, waypoints):
        try:
            for i, waypoint in enumerate(waypoints):
                if not self.is_navigating:
                    self.get_logger().info('Navigation cancelled')
                    return

                self.get_logger().info(f'Navigating to waypoint {i+1}/{len(waypoints)}: {waypoint.point_name}')

                path = self.astar_planner.plan(
                    (self.current_pose['x'], self.current_pose['y']),
                    (waypoint.x, waypoint.y)
                )

                if not path:
                    self.get_logger().warn(f'No path to {waypoint.point_name}, skipping')
                    continue

                self.current_nav_path = path
                self.current_path_index = 0

                for j, target in enumerate(path):
                    if self.manual_override:
                        self.get_logger().info('Manual override active, waiting...')
                        while self.manual_override:
                            rclpy.spin_once(self, timeout_sec=0.1)

                    if not self.is_navigating:
                        return

                    self._navigate_to_point(target)

                self.get_logger().info(f'Reached waypoint: {waypoint.point_name}')
                time.sleep(1.0)

            self.get_logger().info('All waypoints completed!')
            self.is_navigating = False

        except Exception as e:
            self.get_logger().error(f'Navigation error: {str(e)}')
            self.is_navigating = False
            self.cmd_vel_pub.publish(Twist())

    def _navigate_to_point(self, target):
        rate = self.create_rate(10)

        while rclpy.ok():
            if self.manual_override:
                return

            dx = target[0] - self.current_pose['x']
            dy = target[1] - self.current_pose['y']
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.goal_tolerance:
                self.cmd_vel_pub.publish(Twist())
                break

            target_angle = math.atan2(dy, dx)
            angle_diff = self._normalize_angle(target_angle - self.current_pose['theta'])

            twist = Twist()

            if abs(angle_diff) > 0.1:
                twist.linear.x = 0.0
                twist.angular.z = self.max_angular_speed * (angle_diff / abs(angle_diff))
            else:
                twist.linear.x = min(self.max_speed, distance * 0.5)
                twist.angular.z = angle_diff * 0.5

            self.cmd_vel_pub.publish(twist)

            rate.sleep()

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_navigation(self):
        self.is_navigating = False
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Navigation stopped')

    def destroy_node(self):
        self.stop_navigation()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
