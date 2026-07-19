import math


DEFAULT_WAYPOINT_PAUSE_SEC = 2.0
MAX_WAYPOINT_PAUSE_SEC = 60.0


def clamp_waypoint_pause(value, default=DEFAULT_WAYPOINT_PAUSE_SEC):
    try:
        pause_sec = float(value)
    except (TypeError, ValueError):
        pause_sec = float(default)
    if not math.isfinite(pause_sec):
        pause_sec = float(default)
    return max(0.0, min(MAX_WAYPOINT_PAUSE_SEC, pause_sec))


def should_run_regions(region_mode, request_waypoints, regions):
    return bool(region_mode and not request_waypoints and regions)


def mission_callback_is_current(active, callback_run_id, run_id, waypoint_index, current_index):
    return bool(
        active
        and callback_run_id == run_id
        and waypoint_index == current_index
    )


def failure_limit_reached(consecutive_failures, limit=5):
    return int(consecutive_failures) >= int(limit)
