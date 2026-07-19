import math

from mapping_bringup.mission_logic import (
    clamp_waypoint_pause,
    failure_limit_reached,
    mission_callback_is_current,
    should_run_regions,
)


def test_pause_is_clamped_and_non_finite_uses_default():
    assert clamp_waypoint_pause(-1.0) == 0.0
    assert clamp_waypoint_pause(1.5) == 1.5
    assert clamp_waypoint_pause(90.0) == 60.0
    assert clamp_waypoint_pause(math.nan) == 2.0
    assert clamp_waypoint_pause("bad") == 2.0


def test_regions_require_explicit_region_mode_and_empty_request():
    regions = [object()]
    assert should_run_regions(True, [], regions)
    assert not should_run_regions(False, [], regions)
    assert not should_run_regions(True, [object()], regions)
    assert not should_run_regions(True, [], [])


def test_run_id_and_index_reject_stale_callbacks_after_cancel():
    assert mission_callback_is_current(True, 4, 4, 1, 1)
    assert not mission_callback_is_current(True, 3, 4, 1, 1)
    assert not mission_callback_is_current(True, 4, 4, 0, 1)
    assert not mission_callback_is_current(False, 4, 4, 1, 1)


def test_sensor_and_tf_failure_threshold_is_fail_closed():
    assert not failure_limit_reached(4)
    assert failure_limit_reached(5)
    assert failure_limit_reached(6)
