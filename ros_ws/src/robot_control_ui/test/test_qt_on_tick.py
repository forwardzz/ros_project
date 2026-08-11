"""Regression: _on_tick must classify topics without NameError."""

import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _FakeLaunchManager:
    active_name = "mapping"


class _FakeAdapter:
    def __init__(self):
        from robot_control_ui.logic.topic_health import TopicHealthTracker

        tracker = TopicHealthTracker("/scan")
        now = time.time()
        for i in range(5):
            tracker.track(now + i * 0.1)
        self.topic_trackers = {"/scan": tracker}

    def shutdown(self):
        pass


def test_on_tick_classifies_topics_with_real_trackers():
    from robot_control_ui.robot_control_ui import QtBridge, QtMainWindow

    win = QtMainWindow(
        bridge=QtBridge(),
        adapter=_FakeAdapter(),
        launch_manager=_FakeLaunchManager(),
    )
    win._on_tick()  # must not raise NameError
    assert win.status_panel.topic_table.rowCount() == 1
    assert win.status_panel.topic_table.item(0, 0).text() == "/scan"
    win.close()
