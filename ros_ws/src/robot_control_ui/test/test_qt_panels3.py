"""Mission + Thermal panel tests (offscreen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_mission_panel_defaults():
    from robot_control_ui.robot_control_ui import MissionPanel

    panel = MissionPanel()
    assert panel.tsp_check.isChecked() is True
    assert panel.return_check.isChecked() is False
    assert panel.pause_edit.text() == "2.0"
    panel.close()


def test_thermal_panel_no_data_state():
    from robot_control_ui.robot_control_ui import ThermalPanel

    panel = ThermalPanel()
    assert panel._empty is True
    assert "暂无数据" in panel.status.text()
    panel.close()


def test_thermal_panel_updates_frame():
    from robot_control_ui.robot_control_ui import ThermalPanel

    panel = ThermalPanel()
    data = [20.0 + i for i in range(32 * 24)]
    panel.update_frame(32, 24, data)
    assert panel._empty is False
    assert "平均" in panel.status.text()
    assert panel._im is not None
    panel.close()
