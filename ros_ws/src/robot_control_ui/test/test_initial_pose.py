import math

from builtin_interfaces.msg import Time

from robot_control_ui.logic.initial_pose import (
    INITIAL_POSE_COVARIANCE_XY,
    INITIAL_POSE_COVARIANCE_YAW,
    InitialPoseRetryState,
    make_initial_pose_message,
)


def test_initial_pose_message_matches_amcl_contract():
    stamp = Time(sec=12, nanosec=34)
    message = make_initial_pose_message(1.25, -0.75, math.pi / 2.0, stamp)

    assert message.header.frame_id == "map"
    assert message.header.stamp == stamp
    assert message.pose.pose.position.x == 1.25
    assert message.pose.pose.position.y == -0.75
    assert math.isclose(message.pose.pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(message.pose.pose.orientation.w, math.sqrt(0.5))
    assert message.pose.covariance[0] == INITIAL_POSE_COVARIANCE_XY
    assert message.pose.covariance[7] == INITIAL_POSE_COVARIANCE_XY
    assert message.pose.covariance[35] == INITIAL_POSE_COVARIANCE_YAW
    assert sum(value != 0.0 for value in message.pose.covariance) == 3


def test_retry_requires_amcl_update_newer_than_request():
    state = InitialPoseRetryState(max_attempts=3)
    state.begin(100.0)
    assert state.record_publish()
    assert state.evaluate(99.0) == state.RETRY
    assert state.evaluate(100.0) == state.RETRY
    assert state.evaluate(100.1) == state.CONFIRMED
    assert not state.active


def test_retry_times_out_after_configured_publish_count():
    state = InitialPoseRetryState(max_attempts=2)
    state.begin(100.0)
    assert state.record_publish()
    assert state.evaluate(0.0) == state.RETRY
    assert state.record_publish()
    assert state.evaluate(0.0) == state.TIMED_OUT
    assert state.evaluate(101.0) == state.INACTIVE


def test_cancel_blocks_further_publish_attempts():
    state = InitialPoseRetryState()
    state.begin(10.0)
    state.cancel()
    assert not state.record_publish()
    assert state.evaluate(11.0) == state.INACTIVE
