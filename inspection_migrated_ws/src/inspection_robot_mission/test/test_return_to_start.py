from concurrent.futures import Future
from types import SimpleNamespace

from nav2_msgs.action import NavigateToPose

from inspection_robot_mission.mission_manager import MissionManager


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        return Future()


class FakeNavClient:
    def __init__(self, response_future=None):
        self.response_future = response_future or Future()
        self.goals = []

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append((goal, feedback_callback))
        return self.response_future


class FakeReturnManager:
    _finish_mission_success = MissionManager._finish_mission_success
    _start_return_to_start = MissionManager._start_return_to_start
    _return_home_goal_response_cb = MissionManager._return_home_goal_response_cb
    _return_home_feedback_cb = MissionManager._return_home_feedback_cb
    _return_home_result_cb = MissionManager._return_home_result_cb
    _is_current_return_home = MissionManager._is_current_return_home
    _finalize_mission_success = MissionManager._finalize_mission_success

    def __init__(self, return_to_start):
        self.mission_active = True
        self.mission_return_to_start = return_to_start
        self.mission_returning_home = False
        self.mission_run_id = 7
        self.mission_home_x = 1.25
        self.mission_home_y = -0.75
        self.mission_home_yaw = 0.4
        self.goal_handle = None
        self.last_mission_feedback_log_time = 0.0
        self.nav_to_pose_client = FakeNavClient()
        self.stop_count = 0
        self.status_messages = []
        self.final_message = None
        self.failure_message = None

    def get_logger(self):
        return FakeLogger()

    def _make_inspection_point(self, name, x, y, theta):
        return SimpleNamespace(point_name=name, x=x, y=y, theta=theta)

    def _inspection_point_to_pose(self, point):
        return point

    def _clear_mission_wait_timer(self):
        pass

    def _stop_region_control_timer(self):
        pass

    def _publish_zero_cmd(self):
        self.stop_count += 1

    def _publish_mission_status(self, message, safety=False):
        self.status_messages.append((message, safety))

    def _clear_mission_state(self):
        self.mission_return_to_start = False
        self.mission_returning_home = False

    def _finish_mission_failed(self, message):
        self.failure_message = message
        self.mission_active = False
        self._publish_zero_cmd()
        self._clear_mission_state()


def test_success_without_return_finalizes_immediately():
    manager = FakeReturnManager(return_to_start=False)

    manager._finish_mission_success()

    assert manager.mission_active is False
    assert manager.nav_to_pose_client.goals == []
    assert manager.status_messages[-1] == ("Mission completed successfully", False)


def test_success_with_return_sends_fixed_home_and_waits_for_result():
    manager = FakeReturnManager(return_to_start=True)
    handle = FakeGoalHandle()

    manager._finish_mission_success()

    assert manager.mission_active is True
    assert manager.mission_returning_home is True
    assert len(manager.nav_to_pose_client.goals) == 1
    goal = manager.nav_to_pose_client.goals[0][0]
    assert goal.pose.point_name == "MISSION_HOME"
    assert (goal.pose.x, goal.pose.y, goal.pose.theta) == (1.25, -0.75, 0.4)

    manager.nav_to_pose_client.response_future.set_result(handle)
    assert manager.goal_handle is handle
    assert manager.mission_active is True

    result = SimpleNamespace(
        error_code=NavigateToPose.Result.NONE,
        error_msg="",
    )
    handle.result_future.set_result(SimpleNamespace(result=result))

    assert manager.mission_active is False
    assert manager.stop_count >= 2
    assert manager.status_messages[-1] == (
        "Mission completed successfully and robot returned to start",
        False,
    )


def test_stale_return_goal_is_cancelled_after_abort_generation_change():
    manager = FakeReturnManager(return_to_start=True)
    handle = FakeGoalHandle()

    manager._finish_mission_success()
    manager.mission_run_id += 1
    manager.mission_active = False
    manager.nav_to_pose_client.response_future.set_result(handle)

    assert handle.cancel_count == 1
    assert manager.goal_handle is None


def test_return_failure_marks_requested_workflow_failed():
    manager = FakeReturnManager(return_to_start=True)
    handle = FakeGoalHandle()

    manager._finish_mission_success()
    manager.nav_to_pose_client.response_future.set_result(handle)
    result = SimpleNamespace(error_code=42, error_msg="blocked")
    handle.result_future.set_result(SimpleNamespace(result=result))

    assert manager.mission_active is False
    assert "return-to-start navigation failed" in manager.failure_message
