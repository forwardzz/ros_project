"""Safety and correctness regressions for the default Qt backend."""

import os
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtGui, QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _LaunchManager:
    def __init__(self):
        self.remote_user = "robot"
        self.remote_host = "10.0.0.8"
        self.active_name = "idle"
        self.started = []
        self.running = False
        self.thermal_running = False
        self.stop_calls = 0
        self.stop_thermal_calls = 0

    def start(self, name, command):
        self.started.append((name, command))
        return True

    def stop(self):
        self.stop_calls += 1

    def stop_thermal(self):
        self.stop_thermal_calls += 1

    def is_running(self):
        return self.running

    def is_thermal_running(self):
        return self.thermal_running


class _Result:
    def __init__(self, success=True, message="ok"):
        self.success = success
        self.message = message


class _ManualAdapter:
    abort_mission_client = "abort"

    def __init__(self):
        self.published = []
        self.calls = []

    def publish_cmd_vel(self, vx, wz):
        self.published.append((vx, wz))

    def call_service_async(self, client, request, done, timeout_sec=6.0):
        self.calls.append((client, request, done, timeout_sec))


class _TickAdapter:
    def __init__(self):
        self.topic_trackers = {}
        self.gas_data = {"H2": 0.0, "CO": 0.0, "VOC": 0.0, "Smoke": 0.0}
        self.last_gas_stamp = 0.0
        self.thermal_frame = []
        self.last_thermal_stamp = 0.0
        self.thermal_width = 32
        self.thermal_height = 24
        self.map_data = None
        self.map_revision = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.published = []
        self.shutdown_calls = 0

    def publish_cmd_vel(self, vx, wz):
        self.published.append((vx, wz))

    def shutdown(self):
        self.shutdown_calls += 1


class _Origin:
    class Position:
        x = 0.0
        y = 0.0

    position = Position()


class _MapInfo:
    width = 3
    height = 1
    resolution = 1.0
    origin = _Origin()


class _Map:
    info = _MapInfo()
    data = [0, 100, -1]


def test_navigation_uses_seeded_and_shell_quoted_map_path():
    from robot_control_ui.robot_control_ui import MissionControlPanel

    manager = _LaunchManager()
    panel = MissionControlPanel(
        launch_manager=manager, map_path="/maps/default.yaml"
    )
    assert panel.user_edit.text() == "robot"
    assert panel.host_edit.text() == "10.0.0.8"
    assert panel.map_edit.text() == "/maps/default.yaml"

    panel.map_edit.setText("/maps/my map.yaml")
    panel._start_navigation()
    assert manager.started == [(
        "navigation",
        "ros2 launch mapping_bringup navigation.launch.py "
        "map:='/maps/my map.yaml'",
    )]
    panel.map_edit.clear()
    panel._start_navigation()
    assert len(manager.started) == 1
    panel.close()


def test_ip_apply_rejects_invalid_and_running_targets():
    from robot_control_ui.robot_control_ui import MissionControlPanel

    manager = _LaunchManager()
    panel = MissionControlPanel(launch_manager=manager)
    emitted = []
    panel.ip_applied.connect(emitted.append)

    panel.host_edit.setText("999.1.1.1")
    panel._apply_ip()
    assert manager.remote_host == "10.0.0.8"

    manager.running = True
    panel.host_edit.setText("10.0.0.9")
    panel._apply_ip()
    assert manager.remote_host == "10.0.0.8"

    manager.running = False
    manager.thermal_running = True
    panel._apply_ip()
    assert manager.remote_host == "10.0.0.8"

    manager.thermal_running = False
    panel.user_edit.setText("yy")
    panel._apply_ip()
    assert (manager.remote_user, manager.remote_host) == ("yy", "10.0.0.9")
    assert emitted == ["10.0.0.9"]
    panel.close()


def test_manual_drive_waits_for_abort_and_ignores_late_result():
    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = _ManualAdapter()
    panel = ManualDrivePanel(adapter=adapter)
    panel._begin(0.12, 0.0)
    assert adapter.published == [(0.0, 0.0)]
    assert adapter.calls[0][0] == "abort"
    adapter.calls[0][2](_Result(), None)
    assert adapter.published[-1] == (0.12, 0.0)

    panel.stop_robot()
    panel._begin(0.0, 0.55)
    late_done = adapter.calls[-1][2]
    panel.stop_robot()
    count = len(adapter.published)
    late_done(_Result(), None)
    assert len(adapter.published) == count
    assert adapter.published[-1] == (0.0, 0.0)
    panel.close()


def test_manual_drive_mapping_mode_publishes_without_abort_and_stops():
    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = _ManualAdapter()
    manager = _LaunchManager()
    manager.active_name = "mapping"
    panel = ManualDrivePanel(adapter=adapter, launch_manager=manager)

    panel._begin(0.12, 0.0)
    assert adapter.calls == []
    assert adapter.published[-1] == (0.12, 0.0)
    assert panel._timer.isActive()

    panel._publish()
    assert adapter.published[-1] == (0.12, 0.0)
    panel.stop_robot()
    assert adapter.published[-1] == (0.0, 0.0)
    assert not panel._timer.isActive()
    panel.close()


def test_manual_drive_navigation_mode_still_requires_abort():
    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = _ManualAdapter()
    manager = _LaunchManager()
    manager.active_name = "navigation"
    panel = ManualDrivePanel(adapter=adapter, launch_manager=manager)

    panel._begin(0.0, 0.55)
    assert adapter.published == [(0.0, 0.0)]
    assert len(adapter.calls) == 1
    adapter.calls[0][2](_Result(), None)
    assert adapter.published[-1] == (0.0, 0.55)
    panel.close()


def test_manual_drive_navigation_ignores_abort_after_key_release():
    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = _ManualAdapter()
    manager = _LaunchManager()
    manager.active_name = "navigation"
    panel = ManualDrivePanel(adapter=adapter, launch_manager=manager)

    panel._begin(0.12, 0.0)
    callback = adapter.calls[0][2]
    panel.stop_robot()
    count = len(adapter.published)
    callback(_Result(), None)
    assert len(adapter.published) == count
    assert adapter.published[-1] == (0.0, 0.0)
    panel.close()


def test_manual_drive_blocks_when_abort_fails():
    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = _ManualAdapter()
    panel = ManualDrivePanel(adapter=adapter)
    panel._begin(0.12, 0.0)
    adapter.calls[0][2](_Result(False, "still active"), None)
    assert all(command == (0.0, 0.0) for command in adapter.published)
    assert panel._active is False
    panel.close()


def test_sensor_lamps_use_freshness():
    from robot_control_ui.robot_control_ui import QtBridge, QtMainWindow, RobotStatusPanel

    panel = RobotStatusPanel()
    assert "safety" not in panel.lamps
    panel.close()

    adapter = _TickAdapter()
    window = QtMainWindow(
        bridge=QtBridge(), adapter=adapter, launch_manager=_LaunchManager()
    )
    window._on_tick()
    assert "等待数据" in window.status_panel.lamps["gas"][1].text()
    assert "等待数据" in window.status_panel.lamps["thermal"][1].text()

    adapter.last_gas_stamp = time.time() - 3.0
    adapter.last_thermal_stamp = time.time() - 3.0
    adapter.thermal_frame = [20.0]
    window._on_tick()
    assert "数据超时" in window.status_panel.lamps["gas"][1].text()
    assert "数据超时" in window.status_panel.lamps["thermal"][1].text()
    window.close()


def test_health_values_are_retained_and_marked_expired_once():
    from robot_control_ui.logic.remote_health import SystemHealth
    from robot_control_ui.robot_control_ui import RobotStatusPanel

    panel = RobotStatusPanel()
    online = SystemHealth(
        online=True, latency_ms=5.0, temp_c=42.0, cpu_percent=10.0,
        mem_percent=20.0, uptime_s=30.0, throttled_flags=0,
        core_voltage_v=0.8,
    )
    panel.update_health(online)
    offline = SystemHealth(online=False, error_code="timeout")
    panel.update_health(offline)
    panel.update_health(offline)
    for key in ("power", "temp", "cpu", "mem", "uptime"):
        assert panel.health_labels[key].text().count("（数据过期）") == 1
    assert "42.0" in panel.health_labels["temp"].text()
    panel.close()


def test_grayscale_map_pixels_cover_full_occupancy_range():
    from robot_control_ui.robot_control_ui import build_map_image

    image = build_map_image(_Map())
    assert QtGui.qGray(image.pixel(0, 0)) == 255
    assert QtGui.qGray(image.pixel(1, 0)) == 0
    assert QtGui.qGray(image.pixel(2, 0)) == 128


def test_map_base_rebuilds_only_for_new_revision():
    from robot_control_ui.robot_control_ui import QtBridge, QtMainWindow

    adapter = _TickAdapter()
    adapter.map_data = _Map()
    adapter.map_revision = 1
    manager = _LaunchManager()
    window = QtMainWindow(
        bridge=QtBridge(), adapter=adapter, launch_manager=manager
    )
    map_calls = []
    pose_calls = []
    window.live_map.update_map = (
        lambda msg, pose=None: map_calls.append((msg, pose))
    )
    window.live_map.update_pose = pose_calls.append

    window._on_tick()
    window._on_tick()
    assert len(map_calls) == 1
    adapter.robot_x = 1.1
    window._on_tick()
    assert pose_calls == [(1, 0)]
    adapter.map_revision = 2
    window._on_tick()
    assert len(map_calls) == 2
    window.close()


def test_window_shutdown_stops_every_runtime_once():
    from robot_control_ui.robot_control_ui import QtBridge, QtMainWindow

    class Probe:
        stops = 0

        def stop(self):
            self.stops += 1

    adapter = _TickAdapter()
    manager = _LaunchManager()
    probe = Probe()
    window = QtMainWindow(
        bridge=QtBridge(), adapter=adapter, launch_manager=manager,
        health_probe=probe,
    )
    window.shutdown()
    window.shutdown()
    assert adapter.published[-1] == (0.0, 0.0)
    assert adapter.shutdown_calls == 1
    assert manager.stop_thermal_calls == 1
    assert manager.stop_calls == 1
    assert probe.stops == 1
    window.close()


def test_package_declares_qt_runtime_and_test_dependency():
    package_xml = (
        Path(__file__).resolve().parents[1] / "package.xml"
    ).read_text(encoding="utf-8")
    assert "<exec_depend>python_qt_binding</exec_depend>" in package_xml
    assert "<exec_depend>rqt_graph</exec_depend>" in package_xml
    assert "<test_depend>python_qt_binding</test_depend>" in package_xml
