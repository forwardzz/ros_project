"""Mission Control panel tests (offscreen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_mission_control_defaults():
    from robot_control_ui.robot_control_ui import QtBridge
    from robot_control_ui.robot_control_ui import MissionControlPanel

    bridge = QtBridge()
    panel = MissionControlPanel(bridge=bridge)
    assert panel.host_edit.text() == "192.168.43.31"
    assert panel.user_edit.text() == "yy"
    assert panel.start_mapping_btn.text() == "启动建图"
    assert panel.stop_btn.text() == "停止当前模式"
    panel.close()


def test_mission_control_apply_ip_updates_host():
    from robot_control_ui.robot_control_ui import QtBridge
    from robot_control_ui.robot_control_ui import MissionControlPanel

    class FakeLM:
        remote_host = None
        started = []

        def start(self, name, command):
            self.started.append((name, command))
            return True

        def stop(self):
            pass

    lm = FakeLM()
    panel = MissionControlPanel(launch_manager=lm, bridge=QtBridge())
    panel.host_edit.setText("192.168.43.77")
    panel._apply_ip()
    assert lm.remote_host == "192.168.43.77"
    assert panel.remote_host == "192.168.43.77"
    panel.close()


def test_mission_control_start_mapping_uses_launch_manager():
    from robot_control_ui.robot_control_ui import QtBridge
    from robot_control_ui.robot_control_ui import MissionControlPanel

    class FakeLM:
        started = []

        def start(self, name, command):
            self.started.append((name, command))
            return True

        def stop(self):
            pass

    lm = FakeLM()
    panel = MissionControlPanel(launch_manager=lm, bridge=QtBridge())
    panel._start("mapping", "ros2 launch mapping_bringup mapping.launch.py")
    assert lm.started == [("mapping", "ros2 launch mapping_bringup mapping.launch.py")]
    panel.close()


def test_runtime_log_appends_lines():
    from robot_control_ui.robot_control_ui import RuntimeLogPanel

    panel = RuntimeLogPanel()
    panel.append_line("hello")
    panel.append_line("world")
    text = panel.log_view.toPlainText()
    assert "hello" in text and "world" in text
    panel.close()


def test_runtime_log_expanded_window_mirrors_content():
    from robot_control_ui.robot_control_ui import RuntimeLogPanel

    panel = RuntimeLogPanel()
    panel.append_line("before-open")
    panel.show_expanded_log()
    assert panel._expanded_dialog is not None
    assert panel._expanded_dialog.isModal() is False
    assert panel._expanded_view.toPlainText() == panel.log_view.toPlainText()
    panel.append_line("after-open")
    assert "after-open" in panel._expanded_view.toPlainText()
    dialog = panel._expanded_dialog
    panel.show_expanded_log()
    assert panel._expanded_dialog is dialog
    dialog.close()
    _APP.processEvents()
    panel.close()
