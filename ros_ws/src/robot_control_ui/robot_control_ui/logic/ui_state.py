"""Thread-safe UI state snapshot and small formatting helpers.

ROS callbacks write through ``UiStateStore.update``; the UI thread reads a
consistent copy via ``UiStateStore.snapshot``.  The occupancy grid is stored
by reference with a version number so the UI never copies a whole map per
refresh tick.
"""

import copy
import dataclasses
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class UiStateSnapshot:
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_yaw: float = 0.0
    robot_yaw_rate: float = 0.0
    scan_count: int = 0
    amcl_x: float = 0.0
    amcl_y: float = 0.0
    amcl_yaw: float = 0.0
    mission_state: str = "IDLE"
    mission_mode: str = "waypoints"
    mission_message: str = "No mission"
    mission_active: bool = False
    mission_current_index: int = 0
    mission_total_count: int = 0
    thermal_width: int = 32
    thermal_height: int = 24
    thermal_frame: list = field(default_factory=list)
    thermal_min: float = 0.0
    thermal_max: float = 0.0
    thermal_avg: float = 0.0
    thermal_change_per_min: float = 0.0
    thermal_change_ready: bool = False
    gas_data: dict = field(default_factory=lambda: {"H2": 0.0, "CO": 0.0, "VOC": 0.0, "Smoke": 0.0})
    map_msg: Any = None  # OccupancyGrid reference, not copied
    map_version: int = 0
    last_scan_stamp: float = 0.0
    last_odom_stamp: float = 0.0
    last_map_stamp: float = 0.0
    last_amcl_stamp: float = 0.0
    last_thermal_stamp: float = 0.0
    last_gas_stamp: float = 0.0


class UiStateStore:
    """Lock-protected store: ROS callbacks update(), UI reads snapshot()."""

    def __init__(self):
        self._lock = threading.RLock()
        self._state = UiStateSnapshot()

    def update(self, **kwargs):
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self._state, key):
                    setattr(self._state, key, value)

    def snapshot(self) -> UiStateSnapshot:
        with self._lock:
            state = self._state
            # shallow copy: map_msg by reference, gas_data copied
            snap = dataclasses.replace(state)
            snap.gas_data = dict(state.gas_data) if state.gas_data else None
            return snap

    def bump_map(self, msg):
        with self._lock:
            self._state.map_msg = msg
            self._state.map_version += 1


def _fmt_uptime(seconds):
    seconds = int(seconds)
    days, rem = divmod(seconds, 86400)
    hours, rem = divmod(rem, 3600)
    mins, secs = divmod(rem, 60)
    if days:
        return f"{days}d {hours}h"
    if hours:
        return f"{hours}h {mins}m"
    if mins:
        return f"{mins}m {secs}s"
    return f"{secs}s"


def _fmt_rate(rate):
    return "--" if rate is None else f"{rate:.1f}"


def _fmt_age(age):
    return "--" if age is None else f"{age:.1f}s"
