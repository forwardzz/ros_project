"""ROS topic health statistics for the robot control UI.

Pure logic (no rclpy import) so frequency estimation, map semantics and
mode-dependent status can be unit tested.  The UI feeds timestamps via
``track()`` and reads snapshots via ``snapshot()``.
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional


# Topic expectations per mode.  ``mode`` values: idle / mapping / navigation.
EXPECTED = {
    "mapping": {
        "/scan": 5.0,
        "/odom": 10.0,
        "/map": None,          # static map: special semantics
        "/robot_safety_status": 1.0,
    },
    "navigation": {
        "/scan": 5.0,
        "/odom": 10.0,
        "/map": None,
        "/amcl_pose": 1.0,
        "/robot_safety_status": 1.0,
        "/mission_status_typed": 1.0,
    },
}

# states: online / stale / offline / available / unstarted / na
STATE_COLORS = {
    "online": "#2dc653",
    "stale": "#f77f00",
    "offline": "#d00000",
    "available": "#2dc653",
    "unstarted": "#8f8f8f",
    "na": "#8f8f8f",
}


@dataclass
class TopicSnapshot:
    name: str = ""
    publishers: int = 0
    rate_hz: float = None
    age_s: float = None
    summary: str = ""
    state: str = "unstarted"


@dataclass
class TopicHealthTracker:
    """Tracks per-topic timestamps to estimate rate and age."""

    name: str
    window_size: int = 50
    timestamps: List[float] = field(default_factory=list)
    publishers: int = 0
    last_stamp: Optional[float] = None
    received: bool = False

    def track(self, now: Optional[float] = None, publishers: Optional[int] = None):
        now = time.time() if now is None else now
        self.timestamps.append(now)
        if len(self.timestamps) > self.window_size:
            self.timestamps = self.timestamps[-self.window_size:]
        self.last_stamp = now
        self.received = True
        if publishers is not None:
            self.publishers = publishers

    def estimate_rate(self, now: Optional[float] = None) -> Optional[float]:
        if len(self.timestamps) < 2:
            return None
        now = time.time() if now is None else now
        recent = [t for t in self.timestamps if now - t <= 5.0]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        if span <= 0:
            return None
        return round((len(recent) - 1) / span, 2)

    def age(self, now: Optional[float] = None) -> Optional[float]:
        if self.last_stamp is None:
            return None
        now = time.time() if now is None else now
        return round(now - self.last_stamp, 2)


def classify(
    name: str,
    mode: str,
    tracker: Optional[TopicHealthTracker],
    now: Optional[float] = None,
    age_s: Optional[float] = None,
    publishers: Optional[int] = None,
    expected_hz: Optional[float] = None,
) -> TopicSnapshot:
    """Classify one topic into a state + summary for the status table."""
    now = time.time() if now is None else now
    snap = TopicSnapshot(name=name)
    if publishers is not None:
        snap.publishers = publishers
    elif tracker is not None:
        snap.publishers = tracker.publishers

    if mode == "idle" or mode not in ("mapping", "navigation"):
        snap.state = "unstarted"
        snap.summary = "not started"
        return snap

    if name not in EXPECTED.get(mode, {}):
        snap.state = "na"
        snap.summary = "not required in %s mode" % mode
        return snap

    expected = expected_hz
    if expected is None:
        expected = EXPECTED[mode].get(name)

    if tracker is None or not tracker.received:
        snap.state = "offline" if _mode_requires(mode, name) else "unstarted"
        snap.summary = "no data"
        return snap

    snap.age_s = age_s if age_s is not None else tracker.age(now)
    snap.rate_hz = tracker.estimate_rate(now)

    if expected is None:
        # static map: presence + publisher is enough
        if snap.publishers > 0:
            snap.state = "available"
            snap.summary = "map available"
        else:
            snap.state = "stale"
            snap.summary = "map cached, no publisher"
        return snap

    period = 1.0 / expected if expected > 0 else 1.0
    if snap.age_s is None:
        snap.state = "offline"
        snap.summary = "no data"
    elif snap.age_s <= period * 3.0:
        snap.state = "online"
        snap.summary = "%s Hz / %.1f s" % (_fmt_rate(snap.rate_hz), snap.age_s)
    elif snap.age_s < 15.0:
        snap.state = "stale"
        snap.summary = "stale %.1f s" % snap.age_s
    else:
        snap.state = "offline"
        snap.summary = "offline %.1f s" % snap.age_s
    return snap


def _mode_requires(mode: str, name: str) -> bool:
    # scan/odom/safety are hard requirements; map too after data expected
    return name in ("/scan", "/odom", "/robot_safety_status", "/map")


def _fmt_rate(rate) -> str:
    if rate is None:
        return "--"
    return "%.1f" % rate
