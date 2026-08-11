"""Alignment tests: health probe + thermal start/stop in Qt UI."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _FakeLM:
    active_name = "idle"
    remote_host = "192.168.43.30"
    thermal_calls = []

    def start_thermal(self):
        self.thermal_calls.append("start")

    def stop_thermal(self):
        self.thermal_calls.append("stop")

    def start(self, name, command):
        return True

    def stop(self):
        pass


class _FakeAdapter:
    topic_trackers = {}

    def shutdown(self):
        pass


class _FakeProbe:
    def __init__(self):
        self.stops = 0
        self.starts = 0

    def stop(self):
        self.stops += 1

    def start(self):
        self.starts += 1


def test_thermal_panel_has_start_stop_buttons():
    from robot_control_ui.robot_control_ui import ThermalPanel

    lm = _FakeLM()
    panel = ThermalPanel(launch_manager=lm)
    assert panel.start_btn.text() == "启动热成像"
    assert panel.stop_btn.text() == "停止热成像"
    panel.start_thermal()
    panel.stop_thermal()
    assert lm.thermal_calls == ["start", "stop"]
    panel.close()


def test_main_window_applies_ip_and_restarts_probe():
    from robot_control_ui.robot_control_ui import QtBridge, QtMainWindow

    probe = _FakeProbe()
    lm = _FakeLM()
    win = QtMainWindow(
        bridge=QtBridge(), adapter=_FakeAdapter(),
        launch_manager=lm, health_probe=probe,
    )
    win.mission_control.host_edit.setText("192.168.43.77")
    win.mission_control._apply_ip()
    # old probe stopped, new probe started (tracked via restart handler)
    assert probe.stops >= 1
    assert win.health_probe is not None
    assert lm.remote_host == "192.168.43.77"
    win.close()


def test_health_signal_updates_status_panel():
    from robot_control_ui.logic.remote_health import SystemHealth
    from robot_control_ui.robot_control_ui import QtBridge, QtMainWindow

    bridge = QtBridge()
    win = QtMainWindow(bridge=bridge, adapter=_FakeAdapter(), launch_manager=_FakeLM())
    health = SystemHealth()
    health.online = True
    health.latency_ms = 8.0
    health.temp_c = 40.5
    bridge.put_health(health)
    win._on_tick()  # drains the queue into the signal
    assert "在线" in win.status_panel.health_labels["ssh"].text()
    assert "40.5" in win.status_panel.health_labels["temp"].text()
    win.close()
