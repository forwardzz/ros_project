import math

from geometry_msgs.msg import PoseStamped

from robot_monitor_interfaces.msg import InspectionPoint

from .robot_config import FRAME_MAP


def quat_to_yaw(orientation):
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def make_inspection_point(name, x, y, theta):
    point = InspectionPoint()
    point.point_name = name
    point.x = float(x)
    point.y = float(y)
    point.theta = float(theta)
    point.is_confirmed = True
    return point


def inspection_point_to_pose(point, stamp):
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = FRAME_MAP
    pose.pose.position.x = point.x
    pose.pose.position.y = point.y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(point.theta / 2.0)
    pose.pose.orientation.w = math.cos(point.theta / 2.0)
    return pose
