"""Contracts for remote map discovery and managed, on-demand RViz."""

import os
import subprocess
import threading
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtCore, QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _manager():
    from robot_control_ui.launch_manager import LaunchManager

    return LaunchManager("/remote ws", "yy", "10.0.0.8", "/opt/ros/setup.bash", lambda _line: None)


def _query(monkeypatch, result=None, exception=None, directory="/remote ws"):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append((argv, kwargs))
        if exception:
            raise exception
        return result

    monkeypatch.setattr(subprocess, "run", fake_run)
    done = threading.Event()
    answer = []
    _manager().query_map_files(
        directory, lambda paths, error: (answer.append((paths, error)), done.set())
    )
    assert done.wait(1)
    return calls, answer[0]


def test_map_query_returns_sorted_yaml_and_shell_quotes_directory(monkeypatch):
    result = SimpleNamespace(
        returncode=0,
        stdout="/remote ws/z.yaml\n/remote ws/a.yaml\n",
        stderr="",
    )
    calls, answer = _query(monkeypatch, result=result)
    assert answer == (["/remote ws/a.yaml", "/remote ws/z.yaml"], "")
    remote_command = calls[0][0][-1]
    assert "'/remote ws'" in remote_command
    assert '-maxdepth 1' in remote_command
    assert '-name "*.yaml"' in remote_command


def test_map_query_reports_empty_timeout_and_ssh_error(monkeypatch):
    _, answer = _query(
        monkeypatch,
        result=SimpleNamespace(returncode=0, stdout="", stderr=""),
    )
    assert answer == ([], "")

    _, answer = _query(
        monkeypatch,
        exception=subprocess.TimeoutExpired("ssh", 5),
    )
    assert answer[0] == [] and "超时" in answer[1]

    _, answer = _query(
        monkeypatch,
        result=SimpleNamespace(returncode=255, stdout="", stderr="offline"),
    )
    assert answer[0] == [] and "SSH 查询失败" in answer[1]


def test_editable_map_combo_preserves_manual_input_and_uses_full_selection():
    from robot_control_ui.robot_control_ui import MissionControlPanel

    class Manager:
        workspace_path = "/maps"
        remote_user = "yy"
        remote_host = "10.0.0.8"

        def query_map_files(self, directory, callback, timeout=5):
            self.directory = directory
            callback(["/maps/alpha.yaml", "/maps/beta map.yaml"], "")

    manager = Manager()
    panel = MissionControlPanel(launch_manager=manager, map_path="/maps/new.yaml")
    panel.refresh_maps()
    assert manager.directory == "/maps"
    assert [panel.map_combo.itemText(i) for i in range(2)] == ["alpha.yaml", "beta map.yaml"]
    panel._on_map_selected(1)
    assert panel.map_edit.text() == "/maps/beta map.yaml"
    panel.map_edit.setText("/maps/manual.yaml")
    panel.refresh_maps()
    assert panel.map_edit.text() == "/maps/manual.yaml"
    panel.close()


class _Signal:
    def __init__(self):
        self.callbacks = []

    def connect(self, callback):
        self.callbacks.append(callback)

    def emit(self, *args):
        for callback in self.callbacks:
            callback(*args)


class _FakeProcess:
    def __init__(self):
        self.started = _Signal()
        self.finished = _Signal()
        self.errorOccurred = _Signal()
        self.readyReadStandardOutput = _Signal()
        self._state = QtCore.QProcess.NotRunning
        self.start_calls = 0
        self.terminate_calls = 0
        self.arguments = []

    def setProcessChannelMode(self, _mode): pass
    def state(self): return self._state
    def setProgram(self, program): self.program = program
    def setArguments(self, arguments): self.arguments = arguments
    def readAllStandardOutput(self): return b""
    def errorString(self): return ""
    def waitForFinished(self, _timeout): return True
    def kill(self): self._state = QtCore.QProcess.NotRunning

    def start(self):
        self.start_calls += 1
        self._state = QtCore.QProcess.Running
        self.started.emit()

    def terminate(self):
        self.terminate_calls += 1
        self._state = QtCore.QProcess.NotRunning


def test_rviz_is_single_instance_and_closed_with_window(monkeypatch, tmp_path):
    import robot_control_ui.robot_control_ui as ui

    config_dir = tmp_path / "rviz"
    config_dir.mkdir()
    (config_dir / "sllidar_ros2.rviz").write_text("Panels: []", encoding="utf-8")
    monkeypatch.setattr(ui, "get_package_share_directory", lambda _name: str(tmp_path))
    monkeypatch.setattr(
        ui.QtCore.QStandardPaths, "findExecutable", lambda name: f"/usr/bin/{name}"
    )
    process = _FakeProcess()
    win = ui.QtMainWindow(bridge=ui.QtBridge(), rviz_process=process)
    win.open_rviz()
    win.open_rviz()
    assert process.start_calls == 1
    assert process.arguments == ["-d", str(config_dir / "sllidar_ros2.rviz")]
    assert "已打开" in win.rviz_btn.text()
    win.close()
    assert process.terminate_calls == 1
