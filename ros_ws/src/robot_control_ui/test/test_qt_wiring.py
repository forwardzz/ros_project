"""Qt main-window wiring tests: adapter + launch_manager injection."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakeLaunchManager:
    def __init__(self):
        self.started = []
        self.remote_host = "192.168.43.30"

    def start(self, name, command):
        self.started.append((name, command))
        return True

    def stop(self):
        pass

    @property
    def active_name(self):
        return "idle"


class FakeAdapter:
    topic_trackers = {}

    def shutdown(self):
        pass


def test_main_window_wires_launch_manager_to_mission_control():
    from robot_control_ui.robot_control_ui import QtBridge
    from robot_control_ui.robot_control_ui import QtMainWindow

    lm = FakeLaunchManager()
    bridge = QtBridge()
    win = QtMainWindow(bridge=bridge, adapter=FakeAdapter(), launch_manager=lm)
    assert win.mission_control.launch_manager is lm
    win.mission_control._start("mapping", "ros2 launch mapping_bringup mapping.launch.py")
    assert lm.started == [("mapping", "ros2 launch mapping_bringup mapping.launch.py")]
    win.close()  # closeEvent must not raise


def test_mission_control_start_does_not_report_not_connected_when_wired():
    from robot_control_ui.robot_control_ui import QtBridge
    from robot_control_ui.robot_control_ui import QtMainWindow

    lm = FakeLaunchManager()
    win = QtMainWindow(bridge=QtBridge(), adapter=FakeAdapter(), launch_manager=lm)
    logs = []
    win.bridge.log_received.connect(logs.append)
    win.mission_control._start("navigation", "ros2 launch mapping_bringup navigation.launch.py")
    assert "launch manager not connected" not in logs
    assert lm.started
    win.close()
