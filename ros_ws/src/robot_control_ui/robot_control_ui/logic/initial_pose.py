import math

from geometry_msgs.msg import PoseWithCovarianceStamped


INITIAL_POSE_COVARIANCE_XY = 0.25
INITIAL_POSE_COVARIANCE_YAW = 0.0685


def make_initial_pose_message(x, y, yaw, stamp):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = stamp
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
    msg.pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
    msg.pose.covariance[0] = INITIAL_POSE_COVARIANCE_XY
    msg.pose.covariance[7] = INITIAL_POSE_COVARIANCE_XY
    msg.pose.covariance[35] = INITIAL_POSE_COVARIANCE_YAW
    return msg


class InitialPoseRetryState:
    RETRY = "retry"
    CONFIRMED = "confirmed"
    TIMED_OUT = "timed_out"
    INACTIVE = "inactive"

    def __init__(self, max_attempts=8):
        self.max_attempts = int(max_attempts)
        self.attempts = 0
        self.request_started_at = 0.0
        self.active = False

    def begin(self, now):
        self.attempts = 0
        self.request_started_at = float(now)
        self.active = True

    def record_publish(self):
        if not self.active:
            return False
        self.attempts += 1
        return True

    def evaluate(self, last_amcl_at):
        if not self.active:
            return self.INACTIVE
        if float(last_amcl_at) > self.request_started_at:
            self.active = False
            return self.CONFIRMED
        if self.attempts >= self.max_attempts:
            self.active = False
            return self.TIMED_OUT
        return self.RETRY

    def cancel(self):
        self.active = False
