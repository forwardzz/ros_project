"""LAN scan dialog tests (offscreen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_dialog_populates_table():
    from robot_control_ui.logic.network_discovery import DeviceInfo
    from robot_control_ui.robot_control_ui import LanScanDialog

    dialog = LanScanDialog()
    devices = [
        DeviceInfo(ip="192.168.43.30", hostname="yy-desktop",
                   mac="88:a2:9e:62:02:e0", reachable=True, ssh_open=True, current=True),
        DeviceInfo(ip="192.168.43.1", mac="--", ssh_open=False),
    ]
    dialog._on_finished(devices)
    assert dialog.table.rowCount() == 2
    assert dialog.table.item(0, 0).text() == "192.168.43.30"
    assert dialog.table.item(0, 3).text() == "是"
    assert dialog.table.item(0, 4).text() == "开放"
    dialog.close()


def test_dialog_apply_selected_emits_ip():
    from robot_control_ui.logic.network_discovery import DeviceInfo
    from robot_control_ui.robot_control_ui import LanScanDialog

    dialog = LanScanDialog()
    got = []
    dialog.ip_applied.connect(got.append)
    dialog._on_finished([DeviceInfo(ip="192.168.43.30", ssh_open=True)])
    dialog.table.selectRow(0)
    dialog._apply_selected()
    assert got == ["192.168.43.30"]
    dialog.close()
