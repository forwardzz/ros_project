"""Review-fix regression tests."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _FakeLM:
    active_name = "idle"
    run_once_calls = []

    def run_once(self, name, command):
        self.run_once_calls.append(command)

    def start(self, name, command):
        return True

    def stop(self):
        pass


class _FakeAdapter:
    topic_trackers = {}
    start_navigation_client = "start_navigation"
    calls = []

    def call_service_async(self, client, request, done, timeout_sec=6.0):
        self.calls.append((client, request))

    def shutdown(self):
        pass


def test_scan_button_opens_dialog_without_missing_module():
    # Regression: _on_scan must not import a deleted lan_scan_dialog module.
    import robot_control_ui.robot_control_ui as qt

    source = open(
        os.path.join(os.path.dirname(qt.__file__), "robot_control_ui.py"), encoding="utf-8"
    ).read()
    assert "from .lan_scan_dialog import" not in source
    assert "class LanScanDialog" in source


def test_save_map_quotes_prefix():
    from robot_control_ui.robot_control_ui import MissionControlPanel

    lm = _FakeLM()
    panel = MissionControlPanel(launch_manager=lm)
    panel.map_edit.setText("/tmp/my map.yaml")
    panel._save_map()
    assert lm.run_once_calls and "'/tmp/my map'" in lm.run_once_calls[0]
    panel.close()


def test_start_mission_builds_request():
    from robot_control_ui.robot_control_ui import MissionPanel

    adapter = _FakeAdapter()
    panel = MissionPanel(adapter=adapter)
    panel.tsp_check.setChecked(True)
    panel.return_check.setChecked(False)
    panel.pause_edit.setText("3.0")
    panel._start_mission()
    assert adapter.calls and adapter.calls[0][0] == "start_navigation"
    request = adapter.calls[0][1]
    assert request.use_tsp is True
    assert request.return_to_start is False
    assert request.waypoint_pause_sec == 3.0
    panel.close()


def test_scan_dialog_status_emits_signal():
    from robot_control_ui.robot_control_ui import LanScanDialog

    dialog = LanScanDialog()
    texts = []
    dialog.status_changed.connect(texts.append)
    dialog.status_changed.emit("scan error: boom")
    assert texts == ["scan error: boom"]
    dialog.close()


def test_stop_tasks_stops_launch_manager():
    from robot_control_ui.robot_control_ui import MissionPanel

    class _LM:
        stopped = 0

        def stop(self):
            self.stopped += 1

    lm = _LM()
    panel = MissionPanel(launch_manager=lm)
    panel._stop_tasks()
    assert lm.stopped == 1
    panel.close()
