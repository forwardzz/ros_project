"""Qt main-window skeleton tests (offscreen platform)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

# single QApplication for the whole test session (a second instance aborts Qt)
_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_main_window_creates_offscreen():
    from robot_control_ui.robot_control_ui import QtBridge
    from robot_control_ui.robot_control_ui import QtMainWindow

    bridge = QtBridge()
    win = QtMainWindow(bridge=bridge)
    assert "履带机器人" in win.windowTitle()
    assert win.mission_control is not None
    assert win.status_panel is not None
    assert win.live_map is not None
    assert win.manual_drive is not None
    assert win.rqt_graph_btn.text() == "查看 RQT Graph"
    assert win.left_scroll.widgetResizable() is True
    assert win.right_scroll.widgetResizable() is True
    win.close()


def test_bridge_drains_queues_to_signals():
    from robot_control_ui.robot_control_ui import QtBridge

    bridge = QtBridge()
    received = []
    bridge.log_received.connect(received.append)
    bridge.put_log("line-1")
    bridge.put_log("line-2")
    bridge.drain()
    assert received == ["line-1", "line-2"]
    # drain again must be a no-op
    bridge.drain()
    assert received == ["line-1", "line-2"]


def test_main_window_close_is_idempotent():
    from robot_control_ui.robot_control_ui import QtBridge
    from robot_control_ui.robot_control_ui import QtMainWindow

    win = QtMainWindow(bridge=QtBridge())
    win.close()
    win.close()  # second closeEvent must not raise


def test_rqt_graph_process_is_single_and_managed():
    from python_qt_binding import QtCore

    from robot_control_ui.robot_control_ui import QtBridge, QtMainWindow

    class Signal:
        def __init__(self):
            self.callbacks = []

        def connect(self, callback):
            self.callbacks.append(callback)

        def emit(self, *args):
            for callback in self.callbacks:
                callback(*args)

    class FakeProcess:
        def __init__(self):
            self.started = Signal()
            self.finished = Signal()
            self.errorOccurred = Signal()
            self.readyReadStandardOutput = Signal()
            self.start_calls = 0
            self.terminate_calls = 0
            self._state = QtCore.QProcess.NotRunning
            self.program = ""

        def setProcessChannelMode(self, _mode):
            pass

        def state(self):
            return self._state

        def setProgram(self, program):
            self.program = program

        def setArguments(self, _arguments):
            pass

        def start(self):
            self.start_calls += 1
            self._state = QtCore.QProcess.Running
            self.started.emit()

        def terminate(self):
            self.terminate_calls += 1
            self._state = QtCore.QProcess.NotRunning

        def waitForFinished(self, _timeout):
            return True

        def kill(self):
            self._state = QtCore.QProcess.NotRunning

        def readAllStandardOutput(self):
            return b""

        def errorString(self):
            return ""

    process = FakeProcess()
    win = QtMainWindow(bridge=QtBridge(), rqt_process=process)
    win._find_rqt_graph = lambda: "/opt/ros/jazzy/bin/rqt_graph"
    win.open_rqt_graph()
    win.open_rqt_graph()
    assert process.start_calls == 1
    assert process.program.endswith("rqt_graph")
    assert "已打开" in win.rqt_graph_btn.text()
    win.close()
    assert process.terminate_calls == 1
