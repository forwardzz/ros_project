from concurrent.futures import Future
from types import SimpleNamespace

from nav2_msgs.action import NavigateToPose

from inspection_robot_mission.mission_manager import MissionManager


class FakeDuration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return FakeDuration(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return FakeTime(self.seconds)


class FakeLogger:
    def info(self, _message):
        pass


class FakeObstacleManager:
    _reset_region_obstacle_recovery = MissionManager._reset_region_obstacle_recovery
    _begin_region_obstacle_recovery = MissionManager._begin_region_obstacle_recovery
    _region_obstacle_recovery_tick = MissionManager._region_obstacle_recovery_tick

    def __init__(self):
        self.clock = FakeClock()
        self.region_phase = "drive"
        self.region_path_index = 0
        self.mission_regions = [SimpleNamespace(name="REGION_1")]
        self.region_obstacle_wait_sec = 3.0
        self.region_obstacle_clear_frames = 3
        self.region_obstacle_max_retries = 2
        self.region_obstacle_wait_started = None
        self.region_obstacle_resume_phase = ""
        self.region_obstacle_clear_count = 0
        self.region_obstacle_retry_count = 0
        self.region_obstacle_reason = ""
        self.region_obstacle_last_scan_stamp = None
        self.clearance_started_time = None
        self.clearance_obstacle_frames = 0
        self.clearance_last_scan_stamp = None
        self.blocked = True
        self.latest_scan = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
        )
        self.stop_count = 0
        self.skipped_reasons = []
        self.status_messages = []
        self.snapshot_count = 0

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return FakeLogger()

    def _publish_zero_cmd(self):
        self.stop_count += 1

    def _warn_safety(self, message):
        self.status_messages.append((message, True))

    def _publish_mission_status(self, message, safety=False):
        self.status_messages.append((message, safety))

    def _publish_mission_snapshot(self):
        self.snapshot_count += 1

    def _skip_current_region(self, reason):
        self.skipped_reasons.append(reason)

    def _region_obstacle_still_blocked(self):
        return self.blocked


def clear_one_recovery(manager):
    manager.blocked = False
    for frame in range(1, manager.region_obstacle_clear_frames + 1):
        manager.latest_scan.header.stamp.nanosec = frame
        manager._region_obstacle_recovery_tick()


def test_obstacle_stops_immediately_and_enters_wait_state():
    manager = FakeObstacleManager()

    started = manager._begin_region_obstacle_recovery("corridor blocked")

    assert started is True
    assert manager.stop_count == 1
    assert manager.region_phase == "obstacle_wait"
    assert manager.region_obstacle_resume_phase == "drive"
    assert manager.region_obstacle_retry_count == 1
    assert manager.skipped_reasons == []


def test_stable_clearance_resumes_same_phase_and_target_attempt():
    manager = FakeObstacleManager()
    manager._begin_region_obstacle_recovery("corridor blocked")

    clear_one_recovery(manager)

    assert manager.region_phase == "drive"
    assert manager.region_obstacle_retry_count == 1
    assert manager.region_obstacle_wait_started is None
    assert manager.skipped_reasons == []
    assert manager.stop_count == 1 + manager.region_obstacle_clear_frames


def test_persistent_obstacle_times_out_and_skips_region():
    manager = FakeObstacleManager()
    manager._begin_region_obstacle_recovery("corridor blocked")
    manager.clock.seconds = 3.1

    manager._region_obstacle_recovery_tick()

    assert len(manager.skipped_reasons) == 1
    assert "obstacle remained for 3.0s" in manager.skipped_reasons[0]


def test_third_obstacle_skips_after_two_recoveries():
    manager = FakeObstacleManager()

    manager._begin_region_obstacle_recovery("first")
    clear_one_recovery(manager)
    manager.blocked = True
    manager._begin_region_obstacle_recovery("second")
    clear_one_recovery(manager)
    manager.blocked = True

    started = manager._begin_region_obstacle_recovery("third")

    assert started is False
    assert manager.region_obstacle_retry_count == 2
    assert len(manager.skipped_reasons) == 1
    assert "recovery exhausted" in manager.skipped_reasons[0]


def test_target_completion_reset_clears_retry_budget():
    manager = FakeObstacleManager()
    manager.region_obstacle_retry_count = 2
    manager.region_obstacle_wait_started = FakeTime(1.0)

    manager._reset_region_obstacle_recovery()

    assert manager.region_obstacle_retry_count == 0
    assert manager.region_obstacle_wait_started is None


class FakeApproachManager:
    _region_approach_result_cb = MissionManager._region_approach_result_cb

    def __init__(self):
        self.mission_active = True
        self.mission_run_id = 4
        self.region_path_index = 0
        self.region_approach_goal_handle = object()
        self.goal_handle = self.region_approach_goal_handle
        self.skipped_reasons = []
        self.failed_reasons = []

    def _skip_current_region(self, reason):
        self.skipped_reasons.append(reason)
        self.region_path_index += 1

    def _finish_mission_failed(self, reason):
        self.failed_reasons.append(reason)


def test_nav2_terminal_approach_failure_skips_and_continues():
    manager = FakeApproachManager()
    expected_handle = manager.region_approach_goal_handle
    future = Future()
    future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(error_code=42, error_msg="blocked")
        )
    )

    manager._region_approach_result_cb(future, 4, expected_handle)

    assert manager.region_path_index == 1
    assert manager.failed_reasons == []
    assert manager.skipped_reasons == [
        "Nav2 approach failed with code 42: blocked"
    ]


def test_nav2_approach_result_transport_error_still_fails_mission():
    manager = FakeApproachManager()
    expected_handle = manager.region_approach_goal_handle
    future = Future()
    future.set_exception(RuntimeError("action transport lost"))

    manager._region_approach_result_cb(future, 4, expected_handle)

    assert manager.skipped_reasons == []
    assert manager.failed_reasons == [
        "Region approach result failed: action transport lost"
    ]
