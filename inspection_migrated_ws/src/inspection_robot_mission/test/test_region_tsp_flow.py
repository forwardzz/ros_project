from concurrent.futures import Future
from types import SimpleNamespace

from inspection_robot_mission.mission_manager import MissionManager


def make_map(width=30, height=20):
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=[0] * (width * height),
    )


def make_point(name, x, y):
    return SimpleNamespace(point_name=name, x=x, y=y, theta=0.0)


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeNavClient:
    def wait_for_server(self, timeout_sec):
        return timeout_sec == 2.0

    def server_is_ready(self):
        return True


class FakeRegionStartManager:
    DEFAULT_WAYPOINT_PAUSE_SEC = MissionManager.DEFAULT_WAYPOINT_PAUSE_SEC
    MAX_WAYPOINT_PAUSE_SEC = MissionManager.MAX_WAYPOINT_PAUSE_SEC
    _clamp_waypoint_pause = MissionManager._clamp_waypoint_pause
    _validate_mission_home = MissionManager._validate_mission_home

    def __init__(self):
        self.mission_active = False
        self.direct_nav_active = False
        self.pending_rviz_start = None
        self.pending_region_start = None
        self.inspection_regions = [
            SimpleNamespace(name="REGION_1"),
            SimpleNamespace(name="REGION_2"),
            SimpleNamespace(name="REGION_3"),
        ]
        self.region_preview_points = []
        self.region_generation_error = None
        self.confirmed_points = []
        self.rviz_points = []
        self.have_map_pose = True
        self.current_map_pose = {"x": 1.5, "y": 1.5, "theta": 0.0}
        self.have_map = True
        self.map_msg = make_map()
        self.nav_to_pose_client = FakeNavClient()
        self.mission_home_x = 1.5
        self.mission_home_y = 1.5
        self.mission_home_yaw = 0.0
        self.stop_count = 0
        self.queue_calls = []
        self.status_messages = []

    def get_logger(self):
        return FakeLogger()

    def _recompute_region_preview(self):
        self.region_preview_points = [
            make_point("REGION_1_P1", 18.5, 2.5),
            make_point("REGION_2_P1", 4.5, 2.5),
            make_point("REGION_3_P1", 11.5, 2.5),
        ]

    def _queue_region_plan(self, start_request=None):
        self.queue_calls.append(start_request)
        self.pending_region_start = {"generation": 4}
        return 4

    def _publish_zero_cmd(self):
        self.stop_count += 1

    def _publish_mission_status(self, message, safety=False):
        self.status_messages.append((message, safety))


def test_region_start_service_accepts_background_tsp_without_starting_robot():
    manager = FakeRegionStartManager()
    request = SimpleNamespace(
        waypoints=[], waypoint_pause_sec=0.0, return_to_start=True
    )
    response = SimpleNamespace(success=False, message="")

    result = MissionManager._handle_start_navigation(manager, request, response)

    assert result.success is True
    assert "planning accepted" in result.message
    assert "remain stopped" in result.message
    assert manager.queue_calls == [{"return_to_start": True}]
    assert manager.stop_count == 1


class FakeApplyManager:
    def __init__(self):
        self.pending_region_start = {"generation": 9}
        self.mission_active = False
        self.direct_nav_active = False
        self.nav_to_pose_client = FakeNavClient()
        self.region_ordered_paths = [[make_point("R2", 2.0, 2.0)], [make_point("R1", 8.0, 2.0)]]
        self.region_ordered_staging_points = [
            make_point("R2_STAGING", 1.5, 2.0),
            make_point("R1_STAGING", 7.5, 2.0),
        ]
        self.region_transition_paths = [[(0.0, 0.0), (1.5, 2.0)], [(2.0, 2.0), (7.5, 2.0)]]
        self.mission_return_to_start = False
        self.start_calls = []
        self.status_messages = []
        self.stop_count = 0

    def get_logger(self):
        return FakeLogger()

    def _start_region_path_mission(self, *args, **kwargs):
        self.start_calls.append((args, kwargs))
        self.mission_return_to_start = kwargs["return_to_start"]
        return True

    def _publish_zero_cmd(self):
        self.stop_count += 1

    def _warn_safety(self, message):
        self.status_messages.append((message, True))

    def _publish_mission_status(self, message, safety=False):
        self.status_messages.append((message, safety))


def test_completed_region_plan_starts_in_optimized_order_without_reordering_source():
    regions = [SimpleNamespace(name="REGION_1"), SimpleNamespace(name="REGION_2")]
    request = {
        "generation": 9,
        "regions": regions,
        "start_request": {"return_to_start": True},
    }
    plan = SimpleNamespace(
        ordered_indices=[1, 0],
        solving_method="exact",
    )
    manager = FakeApplyManager()

    MissionManager._finish_pending_region_start(manager, request, plan)

    assert [region.name for region in regions] == ["REGION_1", "REGION_2"]
    assert len(manager.start_calls) == 1
    args, kwargs = manager.start_calls[0]
    assert [region.name for region in args[0]] == ["REGION_2", "REGION_1"]
    assert args[4] == "exact"
    assert kwargs == {"return_to_start": True}
    assert "REGION_2 -> REGION_1" in manager.status_messages[-1][0]


def test_stale_region_plan_cannot_start_or_replace_preview():
    future = Future()
    future.set_result(SimpleNamespace(ordered_indices=[1, 0]))
    manager = SimpleNamespace(
        region_plan_future=future,
        region_plan_running_request={"generation": 3},
        region_plan_generation=4,
        apply_count=0,
        start_queued_count=0,
        get_logger=lambda: FakeLogger(),
    )
    manager._apply_completed_region_plan = lambda _request, _plan: setattr(
        manager, "apply_count", manager.apply_count + 1
    )
    manager._start_queued_region_plan = lambda: setattr(
        manager, "start_queued_count", manager.start_queued_count + 1
    )

    MissionManager._poll_region_plan(manager)

    assert manager.apply_count == 0
    assert manager.start_queued_count == 1
    assert manager.region_plan_future is None
