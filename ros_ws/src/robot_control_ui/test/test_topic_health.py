"""Unit tests for robot_control_ui.topic_health."""

import sys
import time
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from robot_control_ui.logic.topic_health import EXPECTED, TopicHealthTracker, classify


def _tracker_at(hz, seconds, start=None):
    """Tracker with timestamps at the given rate for `seconds`."""
    start = time.time() if start is None else start
    tr = TopicHealthTracker("/test")
    n = int(hz * seconds)
    for i in range(n):
        tr.track(start + i / hz)
    return tr, start


def test_rate_estimation_10hz():
    tr, start = _tracker_at(10.0, 1.0)
    assert tr.estimate_rate(start + 1.0) == 10.0


def test_rate_estimation_no_data():
    tr = TopicHealthTracker("/test")
    assert tr.estimate_rate() is None
    assert tr.age() is None
    assert tr.received is False


def test_idle_mode_marks_unstarted():
    snap = classify("/scan", "idle", None, now=time.time())
    assert snap.state == "unstarted"
    assert snap.summary == "not started"


def test_unknown_topic_in_mode_is_na():
    snap = classify("/gas_data", "navigation", None, now=time.time())
    assert snap.state == "na"


def test_continuous_topic_online_stale_offline():
    tr, start = _tracker_at(10.0, 1.0)
    # age 0.1s -> online
    snap = classify("/scan", "mapping", tr, now=start + 1.0, publishers=1)
    assert snap.state == "online"
    assert snap.rate_hz == 10.0
    assert snap.publishers == 1
    # age 6s (>3 periods of 5Hz) -> stale
    snap2 = classify("/scan", "mapping", tr, now=start + 1.0 + 6.0, publishers=1)
    assert snap2.state == "stale"
    # age 30s -> offline
    snap3 = classify("/scan", "mapping", tr, now=start + 1.0 + 30.0, publishers=1)
    assert snap3.state == "offline"


def test_static_map_semantics():
    tr, start = _tracker_at(10.0, 0.5)
    # map received once + publisher present -> available regardless of age
    snap = classify("/map", "mapping", tr, now=start + 100.0, publishers=1)
    assert snap.state == "available"
    # publisher gone -> stale (cached map)
    snap2 = classify("/map", "mapping", tr, now=start + 100.0, publishers=0)
    assert snap2.state == "stale"


def test_missing_topic_offline_in_running_mode():
    snap = classify("/odom", "navigation", None, now=time.time())
    assert snap.state == "offline"
    assert snap.summary == "no data"


def test_expected_spec_contains_required_topics():
    assert "/scan" in EXPECTED["mapping"]
    assert "/map" in EXPECTED["mapping"]
    assert "/amcl_pose" in EXPECTED["navigation"]
    assert "/robot_safety_status" not in EXPECTED["mapping"]
