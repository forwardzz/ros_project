from concurrent.futures import Future
from types import SimpleNamespace

from mapping_bringup.mission_manager import MissionManager


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


def point(name, x, y, theta=0.0):
    return SimpleNamespace(point_name=name, x=x, y=y, theta=theta)


class Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


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


class PreviewManager:
    _queue_rviz_plan = MissionManager._queue_rviz_plan
    _start_queued_rviz_plan = MissionManager._start_queued_rviz_plan
    _poll_rviz_plan = MissionManager._poll_rviz_plan
    _apply_completed_rviz_plan = MissionManager._apply_completed_rviz_plan
    _recompute_rviz_plan = MissionManager._recompute_rviz_plan
    _invalidate_rviz_plans = MissionManager._invalidate_rviz_plans

    def __init__(self, points, use_tsp=True):
        self.map_msg = make_map()
        self.have_map_pose = True
        self.current_map_pose = {"x": 1.5, "y": 1.5, "theta": 0.0}
        self.rviz_points = points
        self.tsp_enabled = use_tsp
        self.rviz_ordered_points = []
        self.rviz_preview_path = []
        self.rviz_solving_method = ""
        self.rviz_plan_executor = DeferredExecutor()
        self.rviz_plan_generation = 0
        self.rviz_plan_future = None
        self.rviz_plan_running_request = None
        self.rviz_plan_queued_request = None
        self.pending_rviz_start = None
        self.publish_count = 0
        self.started_count = 0

    def get_logger(self):
        return Logger()

    def _publish_rviz_plan_visuals(self):
        self.publish_count += 1

    def _finish_pending_rviz_start(self, _request, _plan, _ordered_points):
        self.started_count += 1


def test_tsp_preview_is_deferred_and_applies_optimized_order():
    points = [point("P1", 8.5, 1.5), point("P2", 2.5, 1.5), point("P3", 7.5, 1.5)]
    manager = PreviewManager(points, use_tsp=True)

    manager._recompute_rviz_plan()

    assert manager.rviz_plan_future is not None
    assert not manager.rviz_plan_future.done()
    assert manager.rviz_solving_method == "planning"
    manager.rviz_plan_executor.complete()
    manager._poll_rviz_plan()
    assert [p.point_name for p in manager.rviz_ordered_points] == ["P2", "P3", "P1"]
    assert manager.rviz_solving_method == "exact"


def test_tsp_off_preserves_selected_point_order_in_background_planner():
    points = [point("P1", 8.5, 1.5), point("P2", 2.5, 1.5)]
    manager = PreviewManager(points, use_tsp=False)

    manager._recompute_rviz_plan()
    manager.rviz_plan_executor.complete()
    manager._poll_rviz_plan()

    assert [p.point_name for p in manager.rviz_ordered_points] == ["P1", "P2"]
    assert manager.rviz_solving_method == "selected_order"


def test_stale_completed_plan_cannot_replace_preview_or_start_task():
    points = [point("P1", 5.5, 5.5)]
    manager = PreviewManager(points)
    manager._recompute_rviz_plan()
    manager.rviz_plan_generation += 1
    manager.rviz_plan_executor.complete()

    manager._poll_rviz_plan()

    assert manager.started_count == 0
    assert manager.rviz_ordered_points == points
    assert manager.rviz_preview_path == []


class StartStateManager:
    _start_sequential_mission = MissionManager._start_sequential_mission

    def __init__(self):
        self.mission_wait_timer = None
        self.goal_handle = object()
        self.mission_points = []
        self.mission_index = 0
        self.mission_source = ""
        self.mission_run_id = 4
        self.mission_waypoint_pause_sec = 2.0
        self.mission_return_to_start = False
        self.mission_returning_home = False
        self.last_mission_feedback_log_time = 0.0
        self.mission_early_transition_goal = None
        self.mission_active = False
        self.mission_home_x = 0.0
        self.mission_home_y = 0.0
        self.mission_home_yaw = 0.0

    def _clear_mission_wait_timer(self):
        pass

    def _send_current_mission_goal(self):
        return True


def test_return_home_uses_pose_snapshot_from_task_start():
    manager = StartStateManager()

    assert manager._start_sequential_mission(
        [point("P1", 3.0, 3.0)],
        "request",
        1.0,
        return_to_start=True,
        home_pose=(1.25, -0.75, 0.4),
    )

    assert manager.mission_return_to_start is True
    assert (manager.mission_home_x, manager.mission_home_y, manager.mission_home_yaw) == (
        1.25,
        -0.75,
        0.4,
    )
