"""Alignment tests part 2: save map / lamps / mission tools."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding import QtWidgets

_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _FakeLM:
    active_name = "idle"
    remote_host = "192.168.43.30"
    run_once_calls = []

    def run_once(self, name, command):
        self.run_once_calls.append((name, command))

    def start(self, name, command):
        return True

    def stop(self):
        pass

    def start_thermal(self):
        pass

    def stop_thermal(self):
        pass


class _FakeAdapter:
    topic_trackers = {}
    gas_data = {"H2": 0.1, "CO": 0.2, "VOC": 0.3, "Smoke": 0.4}
    reset_calls = []

    class _Client:
        def __init__(self, owner):
            self.owner = owner

    clear_rviz_points_client = None
    set_region_mode_client = None
    save_inspection_regions_client = None
    load_inspection_regions_client = None
    clear_inspection_regions_client = None
    undo_region_client = None
    undo_rviz_point_client = None

    def call_service_async(self, client, request, done):
        self.reset_calls.append(request)

    def shutdown(self):
        pass


def test_mission_control_has_save_map():
    from robot_control_ui.robot_control_ui import MissionControlPanel

    lm = _FakeLM()
    panel = MissionControlPanel(launch_manager=lm, bridge=None)
    assert panel.save_map_btn.text() == "保存地图"
    panel.map_edit.setText("/home/yy/ros2_ws/map.yaml")
    panel._save_map()
    assert lm.run_once_calls and "map_saver_cli -f /home/yy/ros2_ws/map" in lm.run_once_calls[0][1]
    panel.close()


def test_status_panel_has_lamps_and_updates():
    from robot_control_ui.logic.topic_health import TopicSnapshot
    from robot_control_ui.robot_control_ui import RobotStatusPanel

    panel = RobotStatusPanel()
    assert set(panel.lamps) == {"ssh", "laser", "odom", "map", "thermal", "gas"}
    snap = TopicSnapshot(name="/scan", publishers=1, state="online")
    panel.update_topics([("/scan", snap)])
    label = panel.lamps["laser"][1].text()
    assert "scan" in label and "在线" in label
    panel.update_gas({"H2": 1.0, "CO": 2.0, "VOC": 3.0, "Smoke": 4.0})
    assert "H2：1.0" in panel.gas_label.text()
    panel.close()


def test_mission_panel_has_rviz_and_region_tools():
    from robot_control_ui.robot_control_ui import MissionPanel

    panel = MissionPanel()
    assert panel.clear_points_btn.text() == "清除 RViz 点位"
    assert panel.region_mode_check.text() == "区域巡检模式"
    assert panel.save_regions_btn.text() == "保存区域"
    assert panel.undo_region_btn.text() == "撤销区域"
    assert panel.undo_point_btn.text() == "撤销点位"
    panel.close()
