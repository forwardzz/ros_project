from concurrent.futures import Future
from types import SimpleNamespace

from inspection_robot_mission.mission_manager import MissionManager
from robot_mission_utils.inspection_planner import plan_mission_order


def make_map(width=15, height=15):
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=[0] * (width * height),
    )


def make_point(name, x, y, theta):
    return SimpleNamespace(point_name=name, x=x, y=y, theta=theta)


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class FakeClickedPointManager:
    _clicked_point_cb = MissionManager._clicked_point_cb
    _validate_points_for_setting = MissionManager._validate_points_for_setting

    def __init__(self):
        self.region_mode = False
        self.pending_rviz_start = None
        self.rviz_points = []
        self.have_map_pose = True
        self.current_map_pose = {"x": 1.5, "y": 1.5, "theta": 0.0}
        self.have_map = True
        self.map_msg = make_map()
        self.safety_messages = []
        self.recompute_count = 0

    def get_logger(self):
        return FakeLogger()

    def _warn_safety(self, message):
        self.safety_messages.append(message)

    def _recompute_rviz_plan(self):
        self.recompute_count += 1


def clicked_point(x, y):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        point=SimpleNamespace(x=x, y=y),
    )


def test_rviz_point_near_robot_can_be_recorded_before_tsp_orders_mission():
    manager = FakeClickedPointManager()

    manager._clicked_point_cb(clicked_point(1.6, 1.5))

    assert len(manager.rviz_points) == 1
    assert manager.rviz_points[0].point_name == "RVIZ_1"
    assert manager.safety_messages == []
    assert manager.recompute_count == 1


def test_rviz_point_in_occupied_cell_is_still_rejected_on_insertion():
    manager = FakeClickedPointManager()
    manager.map_msg.data[1 * manager.map_msg.info.width + 3] = 100

    manager._clicked_point_cb(clicked_point(3.5, 1.5))

    assert manager.rviz_points == []
    assert len(manager.safety_messages) == 1
    assert "obstacle or unknown area" in manager.safety_messages[0]
    assert manager.recompute_count == 0


class DeferredExecutor:
    def __init__(self):
        self.future = None
        self.call = None

    def submit(self, function, *args):
        self.future = Future()
        self.call = (function, args)
        return self.future

    def complete(self):
        function, args = self.call
        self.future.set_result(function(*args))


class FakePreviewManager:
    _queue_rviz_plan = MissionManager._queue_rviz_plan
    _start_queued_rviz_plan = MissionManager._start_queued_rviz_plan
    _poll_rviz_plan = MissionManager._poll_rviz_plan
    _apply_completed_rviz_plan = MissionManager._apply_completed_rviz_plan
    _recompute_rviz_plan = MissionManager._recompute_rviz_plan
    _invalidate_rviz_plans = MissionManager._invalidate_rviz_plans

    def __init__(self, points):
        self.map_msg = make_map()
        self.have_map_pose = True
        self.current_map_pose = {"x": 1.5, "y": 1.5, "theta": 0.0}
        self.rviz_points = points
        self.rviz_ordered_points = []
        self.rviz_preview_path = []
        self.rviz_solving_method = ""
        self.rviz_plan_executor = DeferredExecutor()
        self.rviz_plan_generation = 0
        self.rviz_plan_future = None
        self.rviz_plan_running_request = None
        self.rviz_plan_queued_request = None
        self.pending_rviz_start = None
        self.pending_region_start = None
        self.publish_count = 0
        self.started_count = 0

    def get_logger(self):
        return FakeLogger()

    def _publish_rviz_plan_visuals(self):
        self.publish_count += 1

    def _finish_pending_rviz_start(self, _request, _plan, _ordered_points):
        self.started_count += 1


def test_rviz_preview_planning_is_deferred_and_applies_optimized_order():
    points = [
        make_point("RVIZ_1", 8.5, 1.5, 0.15),
        make_point("RVIZ_2", 2.5, 1.5, 0.25),
        make_point("RVIZ_3", 7.5, 1.5, 0.35),
    ]
    manager = FakePreviewManager(points)

    manager._recompute_rviz_plan()

    # The ROS callback returns with planning still pending and no mission started.
    assert manager.rviz_plan_future is not None
    assert not manager.rviz_plan_future.done()
    assert manager.rviz_solving_method == ""
    assert manager.started_count == 0

    manager.rviz_plan_executor.complete()
    manager._poll_rviz_plan()

    assert [point.point_name for point in manager.rviz_ordered_points] == [
        "RVIZ_2",
        "RVIZ_3",
        "RVIZ_1",
    ]
    assert [point.theta for point in manager.rviz_ordered_points] == [0.25, 0.35, 0.15]
    assert manager.rviz_solving_method == "exact"
    assert manager.rviz_preview_path
    assert manager.publish_count == 2


def test_stale_completed_plan_cannot_start_a_mission():
    points = [make_point("RVIZ_1", 5.5, 5.5, 0.0)]
    manager = FakePreviewManager(points)
    plan = plan_mission_order(manager.map_msg, (1.5, 1.5), points)
    future = Future()
    future.set_result(plan)
    manager.rviz_plan_future = future
    manager.rviz_plan_running_request = {
        "generation": 1,
        "map_msg": manager.map_msg,
        "start_xy": (1.5, 1.5),
        "points": points,
        "start_request": {"pause_sec": 0.0},
    }
    manager.rviz_plan_generation = 2
    manager.pending_rviz_start = None

    manager._poll_rviz_plan()

    assert manager.started_count == 0
    assert manager.rviz_plan_future is None


class FakeNavClient:
    def wait_for_server(self, timeout_sec):
        return timeout_sec == 2.0


class FakeStartManager:
    DEFAULT_WAYPOINT_PAUSE_SEC = MissionManager.DEFAULT_WAYPOINT_PAUSE_SEC
    MAX_WAYPOINT_PAUSE_SEC = MissionManager.MAX_WAYPOINT_PAUSE_SEC
    _clamp_waypoint_pause = MissionManager._clamp_waypoint_pause

    def __init__(self, rviz_points):
        self.mission_active = False
        self.direct_nav_active = False
        self.pending_rviz_start = None
        self.pending_region_start = None
        self.inspection_regions = []
        self.region_preview_points = []
        self.region_generation_error = None
        self.confirmed_points = []
        self.rviz_points = rviz_points
        self.have_map_pose = True
        self.current_map_pose = {"x": 1.5, "y": 1.5, "theta": 0.0}
        self.have_map = True
        self.map_msg = make_map()
        self.nav_to_pose_client = FakeNavClient()
        self.queue_calls = []
        self.start_calls = []
        self.stop_count = 0
        self.status_messages = []

    def get_logger(self):
        return FakeLogger()

    def _publish_zero_cmd(self):
        self.stop_count += 1

    def _publish_mission_status(self, message, safety=False):
        self.status_messages.append((message, safety))

    def _queue_rviz_plan(self, start_request=None):
        self.queue_calls.append(start_request)
        self.pending_rviz_start = {"generation": 1}

    def _start_sequential_mission(self, *args, **kwargs):
        self.start_calls.append((args, kwargs))
        return True


def test_start_service_accepts_planning_without_running_tsp_or_starting_robot():
    points = [
        make_point("RVIZ_1", 8.5, 1.5, 0.15),
        make_point("RVIZ_2", 2.5, 1.5, 0.25),
    ]
    manager = FakeStartManager(points)
    request = SimpleNamespace(waypoints=[], waypoint_pause_sec=0.0)
    response = SimpleNamespace(success=False, message="")

    result = MissionManager._handle_start_navigation(manager, request, response)

    assert result.success is True
    assert "planning accepted" in result.message
    assert "remain stopped" in result.message
    assert manager.queue_calls == [
        {"pause_sec": 0.0, "return_to_start": False}
    ]
    assert manager.stop_count == 1
    assert manager.start_calls == []


def test_external_and_confirmed_points_keep_supplied_order():
    points = [
        make_point("EXTERNAL_1", 8.5, 1.5, 0.15),
        make_point("EXTERNAL_2", 2.5, 1.5, 0.25),
    ]

    for use_request_points in (True, False):
        manager = FakeStartManager([])
        manager.confirmed_points = [] if use_request_points else points
        request = SimpleNamespace(
            waypoints=points if use_request_points else [],
            waypoint_pause_sec=0.0,
        )
        response = SimpleNamespace(success=False, message="")

        result = MissionManager._handle_start_navigation(manager, request, response)

        assert result.success is True
        assert manager.queue_calls == []
        assert len(manager.start_calls) == 1
        args, kwargs = manager.start_calls[0]
        ordered_points, source, pause_sec = args
        assert ordered_points == points
        assert source == "request"
        assert pause_sec == 0.0
        assert kwargs == {"return_to_start": False}


def test_rviz_start_carries_return_to_start_through_async_request():
    manager = FakeStartManager([make_point("RVIZ_1", 5.5, 5.5, 0.0)])
    manager.mission_home_x = 1.5
    manager.mission_home_y = 1.5
    manager.mission_home_yaw = 0.0
    manager._validate_mission_home = MissionManager._validate_mission_home.__get__(manager)
    request = SimpleNamespace(
        waypoints=[], waypoint_pause_sec=1.0, return_to_start=True
    )
    response = SimpleNamespace(success=False, message="")

    result = MissionManager._handle_start_navigation(manager, request, response)

    assert result.success is True
    assert manager.queue_calls == [
        {"pause_sec": 1.0, "return_to_start": True}
    ]
