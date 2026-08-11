"""Unit tests for robot_control_ui.ui_state (phase-1 snapshot store)."""

import time


def test_snapshot_store_update_and_copy():
    from robot_control_ui.logic.ui_state import UiStateStore

    store = UiStateStore()
    store.update(robot_x=1.5, scan_count=720, bogus_field=42)
    snap = store.snapshot()
    assert snap.robot_x == 1.5
    assert snap.scan_count == 720
    assert not hasattr(snap, "bogus_field")
    # mutating the snapshot must not affect the store
    snap.robot_x = 99.0
    assert store.snapshot().robot_x == 1.5


def test_map_bump_keeps_reference_and_version():
    from robot_control_ui.logic.ui_state import UiStateStore

    class FakeMap:
        info = None
        data = []

    store = UiStateStore()
    m1 = FakeMap()
    m2 = FakeMap()
    store.bump_map(m1)
    store.bump_map(m2)
    snap = store.snapshot()
    assert snap.map_msg is m2  # reference, not a copy
    assert snap.map_version == 2


def test_gas_data_copied_in_snapshot():
    from robot_control_ui.logic.ui_state import UiStateStore

    store = UiStateStore()
    store.update(gas_data={"H2": 1.0})
    snap = store.snapshot()
    snap.gas_data["H2"] = 999.0
    assert store.snapshot().gas_data["H2"] == 1.0


def test_fmt_helpers():
    from robot_control_ui.logic.ui_state import _fmt_age, _fmt_rate, _fmt_uptime

    assert _fmt_uptime(95) == "1m 35s"
    assert _fmt_uptime(3700) == "1h 1m"
    assert _fmt_uptime(90061) == "1d 1h"
    assert _fmt_rate(9.8) == "9.8"
    assert _fmt_rate(None) == "--"
    assert _fmt_age(0.12) == "0.1s"
