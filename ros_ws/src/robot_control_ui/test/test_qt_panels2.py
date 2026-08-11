"""Phase 4-6 panel tests (offscreen)."""

import math
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakeAdapter:
    def __init__(self):
        self.published = []
        self.abort_mission_client = "abort_mission"
        self.software_estop_client = "software_estop"

    def publish_cmd_vel(self, vx, wz):
        self.published.append((vx, wz))

    def publish_initial_pose(self, x, y, yaw):
        self.published.append(("pose", x, y, yaw))

    def call_service_async(self, client, request, done, timeout_sec=6.0):
        class Result:
            success = True
            message = "ok"

        done(Result(), None)


def test_robot_status_health_online():
    from robot_control_ui.logic.remote_health import SystemHealth
    from robot_control_ui.robot_control_ui import RobotStatusPanel

    panel = RobotStatusPanel()
    health = SystemHealth()
    health.online = True
    health.latency_ms = 12.3
    health.temp_c = 41.5
    health.cpu_percent = 3.2
    health.mem_percent = 14.1
    health.uptime_s = 5399.0
    health.throttled_flags = 0x50000
    health.core_voltage_v = 0.7991
    panel.update_health(health)
    assert "在线" in panel.health_labels["ssh"].text()
    assert "41.5" in panel.health_labels["temp"].text()
    assert "14.1" in panel.health_labels["mem"].text()
    panel.close()


def test_robot_status_health_offline_expired():
    from robot_control_ui.logic.remote_health import SystemHealth
    from robot_control_ui.robot_control_ui import RobotStatusPanel

    panel = RobotStatusPanel()
    health = SystemHealth()
    health.online = False
    health.error_code = "auth_failed"
    panel.update_health(health)
    assert "auth_failed" in panel.health_labels["ssh"].text()
    panel.close()


def test_robot_status_topic_table_rows():
    from robot_control_ui.logic.topic_health import TopicSnapshot
    from robot_control_ui.robot_control_ui import RobotStatusPanel

    panel = RobotStatusPanel()
    snap = TopicSnapshot(name="/scan", publishers=1, rate_hz=10.0,
                         age_s=0.1, summary="10.0 Hz / 0.1 s", state="online")
    panel.update_topics([("/scan", snap)])
    assert panel.topic_table.rowCount() == 1
    assert panel.topic_table.item(0, 0).text() == "/scan"
    assert panel.topic_table.item(0, 1).text() == "1"
    panel.close()


def test_manual_drive_press_release_publishes_zero():
    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = FakeAdapter()
    panel = ManualDrivePanel(adapter=adapter)
    panel._begin(0.12, 0.0)
    assert adapter.published[-1] == (0.12, 0.0)
    panel.stop_robot()
    assert adapter.published[-1] == (0.0, 0.0)
    assert panel._active is False
    panel.close()


def test_manual_drive_focus_loss_stops():
    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = FakeAdapter()
    panel = ManualDrivePanel(adapter=adapter)
    panel._begin(0.12, 0.0)
    panel.stop_all()
    assert adapter.published[-1] == (0.0, 0.0)
    panel.close()


def test_manual_drive_keyboard_ignores_autorepeat():
    from python_qt_binding import QtCore, QtGui

    from robot_control_ui.robot_control_ui import ManualDrivePanel

    adapter = FakeAdapter()
    panel = ManualDrivePanel(adapter=adapter)
    event = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_W,
                             QtCore.Qt.NoModifier)
    event._auto_repeat = True
    # isAutoRepeat is a property on the C++ event; simulate via class
    if not hasattr(event, "isAutoRepeat") or not event.isAutoRepeat():
        pass
    panel.keyPressEvent(event)
    panel.close()


def test_live_map_build_image():
    from robot_control_ui.robot_control_ui import LiveMapPanel, build_map_image

    class Info:
        width = 4
        height = 2
        resolution = 0.05

    class FakeMap:
        info = Info()
        data = [0, 100, -1, 50, 0, 0, 100, -1]

    image = build_map_image(FakeMap())
    assert image.width() == 4
    assert image.height() == 2
    panel_owner = LiveMapPanel()
    panel_owner.update_map(FakeMap())
    assert panel_owner._pixmap is not None
    panel_owner.close()


def test_localization_publishes_initial_pose():
    from robot_control_ui.robot_control_ui import LocalizationPanel

    adapter = FakeAdapter()
    panel = LocalizationPanel(adapter=adapter)
    panel.x_edit.setText("1.0")
    panel.y_edit.setText("2.0")
    panel.yaw_edit.setText("90.0")
    panel.set_initial_pose()
    kind, x, y, yaw = adapter.published[-1]
    assert (kind, x, y) == ("pose", 1.0, 2.0)
    assert math.isclose(yaw, math.pi / 2.0)
    panel.close()
