"""Qt control UI - single-file interface implementation.

Merged from qt_bridge / qt_main_window / widgets / renderers so all Qt
interface code lives in one module.  Non-interface logic stays in
ros_adapter / launch_manager / ui_state / network_discovery / remote_health /
topic_health.
"""

import math
import os
import queue
import shlex
import sys
import threading
import time

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from ament_index_python.packages import get_package_share_directory
from python_qt_binding import QtCore, QtGui, QtWidgets

MAPPING_CMD = "ros2 launch mapping_bringup mapping.launch.py"
NAVIGATION_CMD = "ros2 launch mapping_bringup navigation.launch.py"

STYLE_QSS = """
QWidget {
    color: #243746;
    font-family: "Noto Sans CJK SC", "Microsoft YaHei", "WenQuanYi Micro Hei", sans-serif;
    font-size: 13px;
}
QMainWindow, QWidget#appRoot, QScrollArea, QScrollArea > QWidget > QWidget {
    background-color: #eef3f6;
}
QFrame#appHeader {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #123b4a, stop:1 #176579);
    border: 0;
    border-radius: 10px;
}
QLabel#headerTitle {
    color: white;
    font-size: 22px;
    font-weight: 700;
}
QLabel#headerSubtitle {
    color: #cfe7ed;
    font-size: 12px;
}
QWidget#panelCard {
    background-color: #ffffff;
    border: 1px solid #d7e1e7;
    border-radius: 10px;
}
QLabel#panelTitle {
    font-size: 15px;
    font-weight: 700;
    color: #153f50;
    padding: 2px 0 5px 0;
}
QLabel#metricCard {
    background-color: #f4f8fa;
    border: 1px solid #dce7ec;
    border-radius: 6px;
    padding: 7px 9px;
}
QPushButton {
    background-color: #e7eff3;
    border: 1px solid #bdcdd5;
    border-radius: 6px;
    min-height: 28px;
    padding: 3px 12px;
}
QPushButton:hover { background-color: #d7e6ec; border-color: #86a9b7; }
QPushButton:pressed { background-color: #c8dce4; }
QPushButton:disabled { color: #94a4ab; background-color: #edf1f3; border-color: #d9e0e4; }
QPushButton#primary { background-color: #126477; border-color: #126477; color: white; font-weight: 600; }
QPushButton#primary:hover { background-color: #16788d; }
QPushButton#danger { background-color: #a3264c; border-color: #a3264c; color: white; font-weight: 600; }
QPushButton#danger:hover { background-color: #bd315b; }
QPushButton#headerTool {
    color: white;
    background-color: rgba(255, 255, 255, 35);
    border: 1px solid rgba(255, 255, 255, 95);
    min-height: 30px;
    padding: 3px 16px;
}
QPushButton#headerTool:hover { background-color: rgba(255, 255, 255, 60); }
QPushButton#compactTool { min-height: 24px; padding: 1px 10px; }
QLineEdit, QComboBox {
    background-color: white;
    border: 1px solid #b9cad2;
    border-radius: 5px;
    min-height: 27px;
    padding: 1px 7px;
    selection-background-color: #176579;
}
QLineEdit:focus, QComboBox:focus { border: 1px solid #16758a; }
QCheckBox { spacing: 7px; }
QTableWidget {
    background-color: white;
    alternate-background-color: #f2f7f9;
    border: 1px solid #d5e0e5;
    border-radius: 6px;
    gridline-color: #e1e8ec;
}
QHeaderView::section {
    background-color: #194b5d;
    color: white;
    padding: 6px;
    border: none;
    font-weight: 600;
}
QSplitter::handle { background-color: #c5d6de; width: 4px; }
QPlainTextEdit {
    background-color: #10252e;
    color: #d8eef4;
    border: 1px solid #294b58;
    border-radius: 6px;
    padding: 6px;
    font-family: "DejaVu Sans Mono", "Noto Sans Mono CJK SC", monospace;
    font-size: 12px;
    selection-background-color: #267d92;
}
QDialog { background-color: #eef3f6; }
QScrollBar:vertical { background: #edf2f4; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #b7cbd4; min-height: 28px; border-radius: 5px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""

STATE_COLORS = {
    "online": "#2dc653", "stale": "#f77f00", "offline": "#d00000",
    "available": "#2dc653", "unstarted": "#8f8f8f", "na": "#8f8f8f",
}

from .launch_manager import LaunchManager
from .logic.network_discovery import default_subnet, is_valid_ipv4, scan_subnet
from .logic.topic_health import classify as classify_topic
from .ros_adapter import RosUiAdapter


def style_as_card(widget, minimum_height=None):
    """Apply the shared dashboard-card identity to a panel widget."""
    widget.setObjectName("panelCard")
    widget.setAttribute(QtCore.Qt.WA_StyledBackground, True)
    if minimum_height is not None:
        widget.setMinimumHeight(minimum_height)


# ---------------------------------------------------------------------
# Rendering helpers (pure)
# ---------------------------------------------------------------------
def grid_to_world(map_msg, x, y):
    """Grid cell (x, y) -> world coordinates."""
    info = map_msg.info
    return (
        info.origin.position.x + (x + 0.5) * info.resolution,
        info.origin.position.y + (y + 0.5) * info.resolution,
    )



def world_to_grid(map_msg, wx, wy):
    """World coordinates -> grid cell (x, y), clamped to the map."""
    info = map_msg.info
    x = int((wx - info.origin.position.x) / info.resolution)
    y = int((wy - info.origin.position.y) / info.resolution)
    x = max(0, min(x, info.width - 1))
    y = max(0, min(y, info.height - 1))
    return x, y



def occupancy_to_rows(map_msg):
    """Return the map data reshaped into rows: list of rows, each a list of ints."""
    width = map_msg.info.width
    data = list(map_msg.data)
    return [data[i * width:(i + 1) * width] for i in range(map_msg.info.height)]

def analyze_frame(data):
    """Return (min, max, avg) of a flat thermal frame; zeros when empty."""
    if not data:
        return 0.0, 0.0, 0.0
    return min(data), max(data), sum(data) / len(data)



def update_baseline(baseline_avg, baseline_time, avg, now, window=60.0):
    """Return (change_per_min, change_ready, new_baseline, new_time)."""
    if baseline_avg is None:
        return 0.0, False, avg, now
    if now - baseline_time >= window:
        change = avg - baseline_avg
        return change, True, avg, now
    return 0.0, False, baseline_avg, baseline_time

# ---------------------------------------------------------------------
# Qt event bridge
# ---------------------------------------------------------------------
class QtBridge(QtCore.QObject):
    """Signal hub with per-channel queues drained from the GUI thread."""

    log_received = QtCore.Signal(str)
    health_received = QtCore.Signal(object)
    scan_status_received = QtCore.Signal(str)
    scan_devices_received = QtCore.Signal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._queues = {
            "log": queue.Queue(),
            "health": queue.Queue(),
            "scan_status": queue.Queue(),
            "scan_devices": queue.Queue(),
        }

    # --- producers (callable from any thread) ---
    def put(self, channel, payload):
        q = self._queues.get(channel)
        if q is not None:
            q.put(payload)

    def put_log(self, line):
        self.put("log", line)

    def put_health(self, health):
        self.put("health", health)

    def put_scan_status(self, text):
        self.put("scan_status", text)

    def put_scan_devices(self, devices):
        self.put("scan_devices", devices)

    # --- consumer (GUI thread, called by QTimer) ---
    def drain(self):
        self._drain("log", self.log_received)
        self._drain("health", self.health_received)
        self._drain("scan_status", self.scan_status_received)
        self._drain("scan_devices", self.scan_devices_received)

    def _drain(self, channel, signal):
        q = self._queues[channel]
        while not q.empty():
            try:
                payload = q.get_nowait()
            except queue.Empty:
                break
            signal.emit(payload)


# ---------------------------------------------------------------------
# Panels
# ---------------------------------------------------------------------
def build_map_image(map_msg):
    """Render an OccupancyGrid into a QImage (Grayscale8)."""
    width = map_msg.info.width
    height = map_msg.info.height
    pixels = bytearray(width * height)
    for index, value in enumerate(map_msg.data):
        if value < 0:
            gray = 128  # unknown
        else:
            occupancy = max(0, min(100, int(value)))
            gray = 255 - round(occupancy * 255 / 100)
        pixels[index] = gray
    # copy() detaches the QImage from the temporary Python byte buffer.
    return QtGui.QImage(
        bytes(pixels), width, height, width, QtGui.QImage.Format_Grayscale8
    ).copy()



class MissionControlPanel(QtWidgets.QWidget):
    ip_applied = QtCore.Signal(str)
    map_query_finished = QtCore.Signal(list, str)

    def __init__(self, launch_manager=None, bridge=None, adapter=None,
                 map_path=None, parent=None):
        super().__init__(parent)
        style_as_card(self, 215)
        self.launch_manager = launch_manager
        self.bridge = bridge
        self.adapter = adapter
        self.remote_user = getattr(launch_manager, "remote_user", None) or "yy"
        self.remote_host = (
            getattr(launch_manager, "remote_host", None) or "192.168.43.31"
        )
        self.map_path = map_path or "/home/yy/ros2_ws/map_name.yaml"

        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("任务与运行控制")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        # SSH address row
        ssh_row = QtWidgets.QHBoxLayout()
        ssh_row.addWidget(QtWidgets.QLabel("SSH"))
        self.user_edit = QtWidgets.QLineEdit(self.remote_user)
        self.user_edit.setMaximumWidth(90)
        ssh_row.addWidget(self.user_edit)
        ssh_row.addWidget(QtWidgets.QLabel("@"))
        self.host_edit = QtWidgets.QLineEdit(self.remote_host)
        ssh_row.addWidget(self.host_edit, 1)
        layout.addLayout(ssh_row)

        # scan row
        scan_row = QtWidgets.QHBoxLayout()
        self.scan_btn = QtWidgets.QPushButton("扫描局域网")
        self.refresh_btn = QtWidgets.QPushButton("刷新")
        self.apply_btn = QtWidgets.QPushButton("应用地址")
        self.scan_status = QtWidgets.QLabel("")
        for btn in (self.scan_btn, self.refresh_btn, self.apply_btn):
            scan_row.addWidget(btn)
        scan_row.addWidget(self.scan_status, 1)
        layout.addLayout(scan_row)
        self.scan_btn.clicked.connect(self._on_scan)
        self.refresh_btn.clicked.connect(self._on_scan)
        self.apply_btn.clicked.connect(self._apply_ip)

        # map yaml row
        map_row = QtWidgets.QHBoxLayout()
        map_row.addWidget(QtWidgets.QLabel("地图文件"))
        self.map_combo = QtWidgets.QComboBox()
        self.map_combo.setEditable(True)
        self.map_combo.setInsertPolicy(QtWidgets.QComboBox.NoInsert)
        self.map_combo.setCurrentText(self.map_path)
        # Compatibility for callers/tests that used the old QLineEdit.
        self.map_edit = self.map_combo.lineEdit()
        self.map_refresh_btn = QtWidgets.QPushButton("刷新地图")
        self.map_refresh_status = QtWidgets.QLabel("")
        map_row.addWidget(self.map_combo, 1)
        map_row.addWidget(self.map_refresh_btn)
        layout.addLayout(map_row)
        self.map_refresh_status.setWordWrap(True)
        self.map_refresh_status.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
        )
        layout.addWidget(self.map_refresh_status)
        self.map_combo.activated.connect(self._on_map_selected)
        self.map_refresh_btn.clicked.connect(self.refresh_maps)
        self.map_query_finished.connect(self._on_maps_refreshed)

        # action buttons
        actions = QtWidgets.QHBoxLayout()
        self.start_mapping_btn = QtWidgets.QPushButton("启动建图")
        self.start_navigation_btn = QtWidgets.QPushButton("启动导航")
        self.save_map_btn = QtWidgets.QPushButton("保存地图")
        self.stop_btn = QtWidgets.QPushButton("停止当前模式")
        self.start_mapping_btn.setObjectName("primary")
        self.start_navigation_btn.setObjectName("primary")
        self.stop_btn.setObjectName("danger")
        for btn in (self.start_mapping_btn, self.start_navigation_btn,
                    self.save_map_btn):
            actions.addWidget(btn)
        actions.addStretch(1)
        actions.addWidget(self.stop_btn)
        layout.addLayout(actions)

        self.start_mapping_btn.clicked.connect(lambda: self._start("mapping", MAPPING_CMD))
        self.start_navigation_btn.clicked.connect(self._start_navigation)
        self.save_map_btn.clicked.connect(self._save_map)
        self.stop_btn.clicked.connect(self._stop)
        QtCore.QTimer.singleShot(0, self.refresh_maps)

    # ------------------------------------------------------------------
    def _start(self, name, command):
        if self.launch_manager is None:
            self._log("[运行] 启动管理器未连接")
            return
        if self.launch_manager.start(name, command):
            self._log(f"[RUN] {name}: {command}")

    def _start_navigation(self):
        map_path = self.map_edit.text().strip()
        if not map_path:
            self._log("[导航] 请先选择地图 YAML 文件")
            return
        command = f"{NAVIGATION_CMD} map:={shlex.quote(map_path)}"
        self._start("navigation", command)

    def _stop(self):
        if self.launch_manager is not None:
            self.launch_manager.stop()
            self._log("[STOP] remote tasks")

    def _save_map(self):
        if self.launch_manager is None:
            self._log("[地图] 启动管理器未连接")
            return
        prefix = self.map_edit.text().strip()
        if not prefix:
            self._log("[地图] 请先填写地图保存路径")
            return
        prefix = os.path.splitext(prefix)[0]
        self.launch_manager.run_once(
            "save_map",
            f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(prefix)}",
        )
        self._log(f"[MAP] save map -> {prefix}")

    def _apply_ip(self):
        text = self.host_edit.text().strip()
        if not is_valid_ipv4(text):
            message = f"IPv4 地址无效：{text or '空'}"
            self.scan_status.setText(message)
            self._log(f"[NET] {message}")
            return
        if self.launch_manager is not None:
            running = getattr(self.launch_manager, "is_running", None)
            thermal_running = getattr(self.launch_manager, "is_thermal_running", None)
            if ((callable(running) and running()) or
                    (callable(thermal_running) and thermal_running())):
                message = "切换机器人地址前请先停止当前任务"
                self.scan_status.setText(message)
                self._log(f"[NET] {message}")
                return
        user = self.user_edit.text().strip() or self.remote_user
        if self.launch_manager is not None:
            self.launch_manager.remote_host = text
            self.launch_manager.remote_user = user
        self.remote_user = user
        self.remote_host = text
        self.user_edit.setText(user)
        self.host_edit.setText(text)
        self.scan_status.setText(f"已应用 {user}@{text}")
        self._log(f"[网络] 已应用机器人地址 {text}")
        self.ip_applied.emit(text)
        self.refresh_maps()

    def _map_directory(self):
        path = self.map_edit.text().strip() or self.map_path
        return os.path.dirname(path) or getattr(
            self.launch_manager, "workspace_path", "/home/yy/ros2_ws"
        )

    def refresh_maps(self):
        query = getattr(self.launch_manager, "query_map_files", None)
        if not callable(query):
            self.map_refresh_status.setText("地图查询不可用")
            return
        self.map_refresh_btn.setEnabled(False)
        self.map_refresh_status.setText("正在刷新…")
        directory = self._map_directory()
        query(
            directory,
            lambda paths, error: self.map_query_finished.emit(paths, error),
        )

    def _on_maps_refreshed(self, paths, error):
        self.map_refresh_btn.setEnabled(True)
        if error:
            self.map_refresh_status.setText(error)
            self._log(f"[地图] {error}")
            return
        current = self.map_edit.text().strip()
        self.map_combo.blockSignals(True)
        self.map_combo.clear()
        for path in paths:
            self.map_combo.addItem(os.path.basename(path), path)
        self.map_combo.setEditText(current)
        self.map_combo.blockSignals(False)
        if paths:
            self.map_refresh_status.setText(f"已找到 {len(paths)} 个")
        else:
            self.map_refresh_status.setText("目录中没有地图 YAML")

    def _on_map_selected(self, index):
        path = self.map_combo.itemData(index)
        if path:
            self.map_combo.setEditText(path)

    def _on_scan(self):
        dialog = LanScanDialog(self)
        dialog.ip_applied.connect(self._apply_ip_text)
        dialog.exec_()

    def _apply_ip_text(self, text):
        self.host_edit.setText(text)
        self._apply_ip()

    def _log(self, line):
        if self.bridge is not None:
            self.bridge.put_log(line)


class LocalizationPanel(QtWidgets.QWidget):
    def __init__(self, adapter=None, parent=None):
        super().__init__(parent)
        style_as_card(self, 125)
        self.adapter = adapter
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("定位与初始位姿")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("X"))
        self.x_edit = QtWidgets.QLineEdit("0.0")
        row.addWidget(self.x_edit)
        row.addWidget(QtWidgets.QLabel("Y"))
        self.y_edit = QtWidgets.QLineEdit("0.0")
        row.addWidget(self.y_edit)
        row.addWidget(QtWidgets.QLabel("航向角 (°)"))
        self.yaw_edit = QtWidgets.QLineEdit("0.0")
        row.addWidget(self.yaw_edit)
        self.set_btn = QtWidgets.QPushButton("设置初始位姿")
        row.addWidget(self.set_btn)
        layout.addLayout(row)
        self.status = QtWidgets.QLabel("AMCL：请先启动导航，再设置初始位姿")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        self.set_btn.clicked.connect(self.set_initial_pose)

    def set_initial_pose(self):
        try:
            x = float(self.x_edit.text())
            y = float(self.y_edit.text())
            yaw_degrees = float(self.yaw_edit.text())
        except ValueError:
            self.status.setText("输入无效：X、Y 和航向角必须为数字")
            return
        if self.adapter is not None:
            self.adapter.publish_initial_pose(x, y, math.radians(yaw_degrees))
            self.status.setText(
                f"AMCL: x={x:.2f}, y={y:.2f}, yaw={yaw_degrees:.1f}deg"
            )


class MissionPanel(QtWidgets.QWidget):
    def __init__(self, bridge=None, adapter=None, launch_manager=None, parent=None):
        super().__init__(parent)
        style_as_card(self, 285)
        self.bridge = bridge
        self.adapter = adapter
        self.launch_manager = launch_manager
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("巡检任务")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        self.tsp_check = QtWidgets.QCheckBox("TSP 路径顺序优化")
        self.tsp_check.setChecked(True)
        self.return_check = QtWidgets.QCheckBox("任务完成后返回起点")
        self.return_check.setChecked(False)
        layout.addWidget(self.tsp_check)
        layout.addWidget(self.return_check)

        pause_row = QtWidgets.QHBoxLayout()
        pause_row.addWidget(QtWidgets.QLabel("点位停留时间（秒）"))
        self.pause_edit = QtWidgets.QLineEdit("2.0")
        pause_row.addWidget(self.pause_edit)
        layout.addLayout(pause_row)

        actions = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("开始任务")
        self.start_btn.setObjectName("primary")
        self.stop_btn = QtWidgets.QPushButton("停止全部任务")
        self.stop_btn.setObjectName("danger")
        actions.addWidget(self.start_btn)
        actions.addWidget(self.stop_btn)
        layout.addLayout(actions)

        tools = QtWidgets.QHBoxLayout()
        self.clear_points_btn = QtWidgets.QPushButton("清除 RViz 点位")
        self.region_mode_check = QtWidgets.QCheckBox("区域巡检模式")
        tools.addWidget(self.clear_points_btn)
        tools.addWidget(self.region_mode_check)
        layout.addLayout(tools)

        regions = QtWidgets.QHBoxLayout()
        self.save_regions_btn = QtWidgets.QPushButton("保存区域")
        self.load_regions_btn = QtWidgets.QPushButton("加载区域")
        self.clear_regions_btn = QtWidgets.QPushButton("清除区域")
        for btn in (self.save_regions_btn, self.load_regions_btn, self.clear_regions_btn):
            regions.addWidget(btn)
        layout.addLayout(regions)

        undo_row = QtWidgets.QHBoxLayout()
        self.undo_region_btn = QtWidgets.QPushButton("撤销区域")
        self.undo_point_btn = QtWidgets.QPushButton("撤销点位")
        undo_row.addWidget(self.undo_region_btn)
        undo_row.addWidget(self.undo_point_btn)
        undo_row.addStretch(1)
        layout.addLayout(undo_row)

        self.start_btn.clicked.connect(self._start_mission)
        self.stop_btn.clicked.connect(self._stop_tasks)
        self.clear_points_btn.clicked.connect(self._clear_points)
        self.region_mode_check.toggled.connect(self._set_region_mode)
        self.save_regions_btn.clicked.connect(lambda: self._service_call("save_regions"))
        self.load_regions_btn.clicked.connect(lambda: self._service_call("load_regions"))
        self.clear_regions_btn.clicked.connect(lambda: self._service_call("clear_regions"))
        self.undo_region_btn.clicked.connect(lambda: self._service_call("undo_region"))
        self.undo_point_btn.clicked.connect(lambda: self._service_call("undo_point"))
    def _start_mission(self):
        if self.adapter is None:
            self._log("[任务] ROS 适配器未连接")
            return
        from robot_monitor_interfaces.srv import StartNavigation

        request = StartNavigation.Request()
        try:
            request.waypoint_pause_sec = max(
                0.0, min(60.0, float(self.pause_edit.text()))
            )
        except (TypeError, ValueError):
            request.waypoint_pause_sec = 2.0
            self.pause_edit.setText("2.0")
        request.use_tsp = bool(self.tsp_check.isChecked())
        request.return_to_start = bool(self.return_check.isChecked())
        self._log(
            f"[任务] 正在发送（TSP={request.use_tsp}，"
            f"返航={request.return_to_start}，停留={request.waypoint_pause_sec}s）"
        )
        self.adapter.call_service_async(
            self.adapter.start_navigation_client, request,
            lambda result, error: self._log(
                f"[MISSION] result: {result.message if result else error}"
            ),
            timeout_sec=10.0,
        )

    def _stop_tasks(self):
        self._log("[任务] 停止全部任务")
        if self.launch_manager is not None:
            self.launch_manager.stop()

    def _clear_points(self):
        if self.adapter is None:
            self._log("[任务] ROS 适配器未连接")
            return
        from std_srvs.srv import Trigger

        self.adapter.call_service_async(
            self.adapter.clear_rviz_points_client, Trigger.Request(),
            lambda result, error: self._log(
                f"[RViz] clear points: {result.message if result else error}"
            ),
        )

    def _set_region_mode(self, enabled):
        if self.adapter is None:
            return
        from std_srvs.srv import SetBool

        request = SetBool.Request()
        request.data = bool(enabled)
        self.adapter.call_service_async(
            self.adapter.set_region_mode_client, request,
            lambda result, error: self._log(
                f"[Region] mode={'on' if enabled else 'off'}: "
                f"{result.message if result else error}"
            ),
        )

    def _service_call(self, name):
        if self.adapter is None:
            self._log("[任务] ROS 适配器未连接")
            return
        from std_srvs.srv import Trigger

        client = {
            "save_regions": self.adapter.save_inspection_regions_client,
            "load_regions": self.adapter.load_inspection_regions_client,
            "clear_regions": self.adapter.clear_inspection_regions_client,
            "undo_region": self.adapter.undo_region_client,
            "undo_point": self.adapter.undo_rviz_point_client,
        }.get(name)
        if client is None:
            return
        self.adapter.call_service_async(
            client, Trigger.Request(),
            lambda result, error: self._log(
                f"[MISSION] {name}: {result.message if result else error}"
            ),
        )

    def _log(self, line):
        if self.bridge is not None:
            self.bridge.put_log(line)


class RuntimeLogPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        style_as_card(self, 190)
        self._expanded_dialog = None
        self._expanded_view = None
        layout = QtWidgets.QVBoxLayout(self)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("运行日志")
        title.setObjectName("panelTitle")
        self.expand_btn = QtWidgets.QPushButton("放大查看")
        self.expand_btn.setObjectName("compactTool")
        self.expand_btn.setToolTip("在独立窗口中查看实时运行日志")
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.expand_btn)
        layout.addLayout(header)

        self.log_view = QtWidgets.QPlainTextEdit(self)
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        layout.addWidget(self.log_view, 1)
        self.expand_btn.clicked.connect(self.show_expanded_log)

    def append_line(self, line):
        self.log_view.appendPlainText(line)
        if self._expanded_view is not None:
            self._expanded_view.appendPlainText(line)
        for view in (self.log_view, self._expanded_view):
            if view is None:
                continue
            cursor = view.textCursor()
            cursor.movePosition(QtGui.QTextCursor.End)
            view.setTextCursor(cursor)
            view.ensureCursorVisible()

    def show_expanded_log(self):
        """Show one modeless, live view of the shared runtime-log document."""
        if self._expanded_dialog is not None:
            self._expanded_dialog.show()
            self._expanded_dialog.raise_()
            self._expanded_dialog.activateWindow()
            return

        dialog = QtWidgets.QDialog(self.window())
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.setWindowTitle("运行日志 - 放大查看")
        dialog.resize(1000, 650)
        dialog.setMinimumSize(700, 420)

        layout = QtWidgets.QVBoxLayout(dialog)
        hint = QtWidgets.QLabel("实时显示控制台与机器人端任务输出（最多保留 2000 行）")
        hint.setObjectName("headerSubtitle")
        hint.setStyleSheet("color: #46616d;")
        layout.addWidget(hint)

        view = QtWidgets.QPlainTextEdit(dialog)
        view.setReadOnly(True)
        view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        view.setMaximumBlockCount(2000)
        view.setPlainText(self.log_view.toPlainText())
        layout.addWidget(view, 1)

        self._expanded_dialog = dialog
        self._expanded_view = view
        dialog.destroyed.connect(self._on_expanded_destroyed)
        dialog.show()

        cursor = view.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        view.setTextCursor(cursor)

    def _on_expanded_destroyed(self, _object=None):
        self._expanded_dialog = None
        self._expanded_view = None


class RobotStatusPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        style_as_card(self, 330)
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("机器人状态")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        self.health_labels = {}

        # indicator lamps (SSH/Laser/Odom/Map/Thermal/Gas)
        lamps = QtWidgets.QHBoxLayout()
        self.lamps = {}
        for key, label in (
            ("ssh", "SSH"), ("laser", "雷达"), ("odom", "里程计"),
            ("map", "地图"), ("thermal", "热成像"),
            ("gas", "气体"),
        ):
            dot = QtWidgets.QLabel("\u25cf")
            dot.setStyleSheet("color: #8f8f8f; font-size: 12px;")
            text = QtWidgets.QLabel(f"{label}：等待数据")
            text.setStyleSheet("font-size: 9px;")
            cell = QtWidgets.QWidget()
            cell_layout = QtWidgets.QHBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.addWidget(dot)
            cell_layout.addWidget(text)
            lamps.addWidget(cell)
            self.lamps[key] = (dot, text)
        lamps.addStretch(1)
        layout.addLayout(lamps)

        grid = QtWidgets.QGridLayout()
        cards = [
            ("SSH", "ssh"), ("电源", "power"), ("CPU 温度", "temp"),
            ("CPU", "cpu"), ("内存", "mem"), ("运行时间", "uptime"),
        ]
        for idx, (label, key) in enumerate(cards):
            cell = QtWidgets.QLabel(f"{label}：--")
            cell.setObjectName("metricCard")
            cell.setWordWrap(True)
            self.health_labels[key] = cell
            grid.addWidget(cell, idx // 3, idx % 3)
        layout.addLayout(grid)

        gas_row = QtWidgets.QHBoxLayout()
        self.gas_label = QtWidgets.QLabel("H2：--  CO：--  VOC：--  烟雾：--")
        gas_row.addWidget(self.gas_label)
        gas_row.addStretch(1)
        layout.addLayout(gas_row)

        self.topic_table = QtWidgets.QTableWidget(0, 5)
        self.topic_table.setHorizontalHeaderLabels(
            ["话题", "发布者", "频率 (Hz)", "延迟", "状态摘要"]
        )
        self.topic_table.setAlternatingRowColors(True)
        self.topic_table.horizontalHeader().setStretchLastSection(True)
        self.topic_table.verticalHeader().setVisible(False)
        layout.addWidget(self.topic_table, 1)
    # ------------------------------------------------------------------
    def _set_lamp(self, key, color, text):
        dot, label = self.lamps[key]
        dot.setStyleSheet(f"color: {color}; font-size: 12px;")
        label.setText(text)

    def update_gas(self, gas_data, state="online"):
        if state == "waiting" or not gas_data:
            self._set_lamp("gas", "#8f8f8f", "气体：等待数据")
            self.gas_label.setText("H2：--  CO：--  VOC：--  烟雾：--")
            return
        online = state == "online"
        self._set_lamp(
            "gas", "#2dc653" if online else "#d00000",
            "气体：在线" if online else "气体：数据超时",
        )
        suffix = "" if online else "（数据过期）"
        self.gas_label.setText(
            f"H2：{gas_data['H2']:.1f}  CO：{gas_data['CO']:.1f}  "
            f"VOC：{gas_data['VOC']:.1f}  烟雾：{gas_data['Smoke']:.1f}{suffix}"
        )

    def update_thermal_lamp(self, state):
        if state == "waiting":
            self._set_lamp("thermal", "#8f8f8f", "热成像：等待数据")
            return
        online = state is True or state == "online"
        self._set_lamp(
            "thermal", "#2dc653" if online else "#d00000",
            "热成像：在线" if online else "热成像：数据超时",
        )

    def update_health(self, health):
        if health is None:
            return
        if health.online:
            self._set_lamp("ssh", "#2dc653", f"SSH: {health.latency_ms}ms")
            self.health_labels["ssh"].setText(
                f"SSH：在线 {health.latency_ms}ms"
            )
            self.health_labels["temp"].setText(
                f"CPU 温度：{health.temp_c} °C" if health.temp_c is not None else "CPU 温度：N/A"
            )
            cpu = "N/A" if health.cpu_percent is None else f"{health.cpu_percent}%"
            self.health_labels["cpu"].setText(f"CPU：{cpu}")
            mem = "N/A" if health.mem_percent is None else f"{health.mem_percent}%"
            self.health_labels["mem"].setText(f"内存：{mem}")
            self.health_labels["uptime"].setText(
                f"运行时间：{health.uptime_s:.0f}s" if health.uptime_s is not None else "运行时间：N/A"
            )
            power = "5V 输入：N/A（无 ADC）"
            if health.throttled_flags is not None:
                uv = "欠压" if health.throttled_flags & 0x1 else "正常"
                power += f"  供电：{uv}"
            if health.core_voltage_v is not None:
                power += f"  核心：{health.core_voltage_v}V"
            self.health_labels["power"].setText(f"电源：{power}")
        else:
            self._set_lamp(
                "ssh", "#d00000" if health.error_code != "unprobed" else "#8f8f8f",
                f"SSH：{health.error_code}",
            )
            self.health_labels["ssh"].setText(f"SSH：{health.error_code}")
            for key in ("power", "temp", "cpu", "mem", "uptime"):
                current = self.health_labels[key].text()
                if "（数据过期）" in current:
                    continue
                if "--" in current:
                    current = current.replace("--", "N/A")
                self.health_labels[key].setText(f"{current}（数据过期）")

    TOPIC_LAMP = {
        "/scan": "laser", "/odom": "odom", "/map": "map",
    }

    def update_topics(self, rows):
        """rows: list of (name, TopicSnapshot)."""
        self.topic_table.setRowCount(len(rows))
        for row, (name, snap) in enumerate(rows):
            state_color = STATE_COLORS.get(snap.state, "#8f8f8f")
            state_text = {
                "online": "在线", "stale": "数据过期", "offline": "离线",
                "available": "可用", "unstarted": "未启动", "na": "不适用",
            }.get(snap.state, snap.state)
            lamp_key = self.TOPIC_LAMP.get(name)
            if lamp_key is not None:
                self._set_lamp(
                    lamp_key, state_color,
                    f"{name.split('/')[-1]}：{state_text}",
                )
            cells = [
                name,
                str(snap.publishers),
                "--" if snap.rate_hz is None else f"{snap.rate_hz:.1f}",
                "--" if snap.age_s is None else f"{snap.age_s:.1f}s",
                snap.summary,
            ]
            for col, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                if col == 0:
                    item.setForeground(QtCore.Qt.blue)
                elif col == 4:
                    item.setForeground(
                        QtGui.QColor(state_color)
                    )
                self.topic_table.setItem(row, col, item)


class LiveMapPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        style_as_card(self, 320)
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("实时地图")
        title.setObjectName("panelTitle")
        layout.addWidget(title)
        self.canvas = QtWidgets.QLabel("暂无地图数据")
        self.canvas.setAlignment(QtCore.Qt.AlignCenter)
        self.canvas.setMinimumHeight(200)
        layout.addWidget(self.canvas, 1)
        self._pixmap = None
        self._trail = []
        self._last_pose = None

    def set_trail(self, points):
        self._trail = list(points)

    def update_map(self, map_msg, pose=None):
        image = build_map_image(map_msg)
        pixmap = QtGui.QPixmap.fromImage(image)
        self._pixmap = pixmap
        self._last_pose = pose
        self._redraw(pose)

    def update_pose(self, pose):
        if pose == self._last_pose:
            return
        self._last_pose = pose
        self._redraw(pose)

    def _redraw(self, pose=None):
        if self._pixmap is None:
            return
        overlay = QtGui.QPixmap(self._pixmap)
        painter = QtGui.QPainter(overlay)
        painter.setPen(QtGui.QColor("#d00000"))
        for x, y in self._trail:
            painter.drawPoint(x, y)
        if pose is not None:
            x, y = pose
            painter.setPen(QtGui.QPen(QtGui.QColor("#0f4c5c"), 2))
            painter.drawEllipse(int(x) - 3, int(y) - 3, 7, 7)
        painter.end()
        self.canvas.setPixmap(
            overlay.scaled(
                self.canvas.size(), QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )
        )


class ThermalPanel(QtWidgets.QWidget):
    def __init__(self, launch_manager=None, parent=None):
        super().__init__(parent)
        style_as_card(self, 385)
        self.launch_manager = launch_manager
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("热成像")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        btn_row = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("启动热成像")
        self.stop_btn = QtWidgets.QPushButton("停止热成像")
        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(self.stop_btn)
        layout.addLayout(btn_row)
        self.start_btn.clicked.connect(self.start_thermal)
        self.stop_btn.clicked.connect(self.stop_thermal)

        self.figure = Figure(figsize=(4, 3), dpi=80)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111)
        self._im = None
        self._cbar = None
        self._empty = True
        layout.addWidget(self.canvas, 1)
        self.status = QtWidgets.QLabel("热成像：暂无数据")
        layout.addWidget(self.status)

    def start_thermal(self):
        if self.launch_manager is not None:
            self.launch_manager.start_thermal()

    def stop_thermal(self):
        if self.launch_manager is not None:
            self.launch_manager.stop_thermal()

    def update_frame(self, width, height, data):
        if not data:
            self.status.setText("热成像：暂无数据")
            return
        frame_min, frame_max, frame_avg = analyze_frame(data)
        import numpy as np

        matrix = np.array(data, dtype=float).reshape(height, width)
        if self._im is None:
            self._im = self.ax.imshow(matrix, cmap="inferno", aspect="auto")
            self.figure.colorbar(self._im, ax=self.ax)
        else:
            self._im.set_data(matrix)
            self._im.set_clim(frame_min, frame_max)
        self.canvas.draw_idle()
        self._empty = False
        self.status.setText(
            f"热成像：最低 {frame_min:.1f}°C  最高 {frame_max:.1f}°C  平均 {frame_avg:.1f}°C"
        )

    def set_freshness(self, state):
        if state == "waiting":
            self.status.setText("热成像：等待数据")
        elif state == "timeout" and "（数据过期）" not in self.status.text():
            self.status.setText(f"{self.status.text()}（数据过期）")


class ManualDrivePanel(QtWidgets.QWidget):
    abort_finished = QtCore.Signal(int, object, object)

    def __init__(self, adapter=None, bridge=None, launch_manager=None,
                 parent=None):
        super().__init__(parent)
        style_as_card(self, 205)
        self.adapter = adapter
        self.bridge = bridge
        self.launch_manager = launch_manager
        self.manual_linear = 0.12
        self.manual_angular = 0.55
        self._cmd = (0.0, 0.0)
        self._pending_cmd = None
        self._active = False
        self._request_id = 0

        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("手动驾驶")
        title.setObjectName("panelTitle")
        layout.addWidget(title)
        layout.addWidget(QtWidgets.QLabel("使用 W/A/S/D 或方向键控制，空格键立即停止"))

        pad = QtWidgets.QGridLayout()
        self._add_drive_button(pad, "←", 0, 0, 0.0, self.manual_angular)
        self._add_drive_button(pad, "↑", 0, 1, self.manual_linear, 0.0)
        self._add_drive_button(pad, "↓", 1, 0, -self.manual_linear, 0.0)
        self._add_drive_button(pad, "→", 1, 1, 0.0, -self.manual_angular)
        stop_btn = QtWidgets.QPushButton("立即停止")
        stop_btn.setObjectName("danger")
        stop_btn.clicked.connect(self.stop_robot)
        pad.addWidget(stop_btn, 0, 2, 2, 1)
        layout.addLayout(pad)

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._publish)
        self.abort_finished.connect(self._on_abort_finished)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

    def _add_drive_button(self, grid, text, row, col, vx, wz):
        btn = QtWidgets.QPushButton(text)
        btn.pressed.connect(lambda: self._begin(vx, wz))
        btn.released.connect(self.stop_robot)
        grid.addWidget(btn, row, col)

    # ------------------------------------------------------------------
    def _begin(self, vx, wz):
        requested = (float(vx), float(wz))
        if requested == (0.0, 0.0):
            self.stop_robot()
            return
        if self._active and self._cmd == requested:
            return

        self.stop_robot()
        mode = getattr(self.launch_manager, "active_name", None)
        if mode in ("idle", "mapping"):
            self._log(f"[MANUAL] direct manual control in {mode} mode")
            self._activate_command(requested)
            return
        if (self.adapter is None or
                not hasattr(self.adapter, "call_service_async") or
                not hasattr(self.adapter, "abort_mission_client")):
            self._log("[MANUAL] autonomous abort unavailable; command blocked")
            return

        from std_srvs.srv import Trigger

        self._pending_cmd = requested
        request_id = self._request_id
        self._log("[MANUAL] aborting autonomy before manual drive")
        self.adapter.call_service_async(
            self.adapter.abort_mission_client,
            Trigger.Request(),
            lambda result, error: self.abort_finished.emit(
                request_id, result, error
            ),
            timeout_sec=2.0,
        )

    def _on_abort_finished(self, request_id, result, error):
        if request_id != self._request_id or self._pending_cmd is None:
            return
        if error is not None or result is None or not result.success:
            detail = error or (
                result.message if result is not None else "unknown error"
            )
            self._pending_cmd = None
            self._log(f"[MANUAL] autonomous abort failed; command blocked: {detail}")
            return
        command = self._pending_cmd
        self._pending_cmd = None
        self._activate_command(command)

    def _activate_command(self, command):
        self._cmd = command
        self._active = True
        self._publish()
        self._timer.start()

    def stop_robot(self):
        self._request_id += 1
        self._pending_cmd = None
        self._cmd = (0.0, 0.0)
        self._active = False
        self._timer.stop()
        if self.adapter is not None and hasattr(self.adapter, "publish_cmd_vel"):
            self.adapter.publish_cmd_vel(0.0, 0.0)

    def _publish(self):
        if (self._active and self.adapter is not None and
                hasattr(self.adapter, "publish_cmd_vel")):
            self.adapter.publish_cmd_vel(self._cmd[0], self._cmd[1])

    def stop_all(self):
        self.stop_robot()

    def _log(self, line):
        if self.bridge is not None:
            self.bridge.put_log(line)

    # ------------------------------------------------------------ keyboard
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_Space:
            self.stop_robot()
            event.accept()
            return
        mapping = {
            QtCore.Qt.Key_W: (self.manual_linear, 0.0),
            QtCore.Qt.Key_Up: (self.manual_linear, 0.0),
            QtCore.Qt.Key_S: (-self.manual_linear, 0.0),
            QtCore.Qt.Key_Down: (-self.manual_linear, 0.0),
            QtCore.Qt.Key_A: (0.0, self.manual_angular),
            QtCore.Qt.Key_Left: (0.0, self.manual_angular),
            QtCore.Qt.Key_D: (0.0, -self.manual_angular),
            QtCore.Qt.Key_Right: (0.0, -self.manual_angular),
        }
        if event.key() in mapping:
            self._begin(*mapping[event.key()])
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        # X11/WSLg keyboard auto-repeat can emit synthetic release/press
        # pairs while the physical key is still held.  Treat only the final,
        # non-repeat release as a stop command.
        if event.isAutoRepeat():
            event.accept()
            return
        if event.key() in (
            QtCore.Qt.Key_W, QtCore.Qt.Key_Up, QtCore.Qt.Key_S,
            QtCore.Qt.Key_Down, QtCore.Qt.Key_A, QtCore.Qt.Key_Left,
            QtCore.Qt.Key_D, QtCore.Qt.Key_Right, QtCore.Qt.Key_Space,
        ):
            self.stop_robot()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        self.stop_robot()
        super().focusOutEvent(event)


class LanScanDialog(QtWidgets.QDialog):
    scan_finished = QtCore.Signal(list)
    status_changed = QtCore.Signal(str)
    ip_applied = QtCore.Signal(str)

    COLUMNS = ("IP", "主机名", "MAC", "可达", "SSH", "当前地址")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("局域网设备扫描")
        self.setMinimumSize(780, 440)
        self._cancel = None
        self._scanning = False

        layout = QtWidgets.QVBoxLayout(self)
        self.table = QtWidgets.QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        layout.addWidget(self.table, 1)

        bar = QtWidgets.QHBoxLayout()
        self.status = QtWidgets.QLabel("就绪")
        bar.addWidget(self.status, 1)
        self.scan_btn = QtWidgets.QPushButton("开始扫描")
        self.cancel_btn = QtWidgets.QPushButton("取消")
        self.apply_btn = QtWidgets.QPushButton("应用所选地址")
        bar.addWidget(self.scan_btn)
        bar.addWidget(self.cancel_btn)
        bar.addWidget(self.apply_btn)
        layout.addLayout(bar)

        self.scan_btn.clicked.connect(self.start_scan)
        self.cancel_btn.clicked.connect(self.cancel_scan)
        self.apply_btn.clicked.connect(self._apply_selected)
        self.scan_finished.connect(self._on_finished)
        self.status_changed.connect(self.status.setText)

    # ------------------------------------------------------------------
    def start_scan(self):
        if self._scanning:
            return
        self._scanning = True
        self._cancel = threading.Event()
        self.status.setText("正在扫描…")
        self.scan_btn.setEnabled(False)
        threading.Thread(target=self._worker, daemon=True).start()

    def cancel_scan(self):
        if self._cancel is not None:
            self._cancel.set()
            self.status.setText("正在取消…")

    def _worker(self):
        subnet = default_subnet()
        if not subnet:
            self.status_changed.emit("无法确定当前活动子网")
            self.scan_finished.emit([])
            return
        try:
            devices = scan_subnet(subnet, timeout=0.8, cancel_event=self._cancel)
        except Exception as exc:  # noqa: BLE001
            self.status_changed.emit(f"扫描错误：{exc}")
            devices = []
        self.scan_finished.emit(devices)

    def _on_finished(self, devices):
        self._scanning = False
        self.scan_btn.setEnabled(True)
        self.table.setRowCount(len(devices))
        for row, dev in enumerate(devices):
            values = (
                dev.ip,
                dev.hostname,
                dev.mac,
                "是" if dev.reachable else "否",
                "开放" if dev.ssh_open else "关闭",
                "*" if dev.current else "",
            )
            for col, text in enumerate(values):
                item = QtWidgets.QTableWidgetItem(text)
                self.table.setItem(row, col, item)
        self.status.setText(f"扫描完成：发现 {len(devices)} 台设备")

    def _apply_selected(self):
        row = self.table.currentRow()
        if row < 0:
            self.status.setText("请先选择一行设备")
            return
        ip = self.table.item(row, 0).text()
        self.ip_applied.emit(ip)
        self.accept()


# ---------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------
class QtMainWindow(QtWidgets.QMainWindow):
    """Main window for the Qt control UI."""

    def __init__(self, bridge=None, adapter=None, launch_manager=None,
                 health_probe=None, map_path=None, parent=None,
                 rqt_process=None, rviz_process=None):
        super().__init__(parent)
        self.bridge = bridge if bridge is not None else QtBridge()
        self.adapter = adapter  # RosUiAdapter (may be None in tests/skeleton)
        self.launch_manager = launch_manager
        self.health_probe = health_probe
        self.map_path = map_path
        self._last_map_revision = None
        self._last_robot_grid_pose = None
        self._last_thermal_render_stamp = 0.0
        self._rqt_process = rqt_process or QtCore.QProcess(self)
        self._rqt_process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self._rqt_process.started.connect(self._on_rqt_started)
        self._rqt_process.finished.connect(self._on_rqt_finished)
        self._rqt_process.errorOccurred.connect(self._on_rqt_error)
        self._rqt_process.readyReadStandardOutput.connect(self._on_rqt_output)
        self._rviz_process = rviz_process or QtCore.QProcess(self)
        self._rviz_process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self._rviz_process.started.connect(self._on_rviz_started)
        self._rviz_process.finished.connect(self._on_rviz_finished)
        self._rviz_process.errorOccurred.connect(self._on_rviz_error)
        self._rviz_process.readyReadStandardOutput.connect(self._on_rviz_output)

        self.setWindowTitle("履带机器人控制中心")
        self.resize(1440, 900)
        self.setMinimumSize(1120, 720)

        self._build_ui()
        self.bridge.log_received.connect(self.log_panel.append_line)
        self.bridge.health_received.connect(self.status_panel.update_health)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(200)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QtWidgets.QWidget(self)
        central.setObjectName("appRoot")
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # header
        header = QtWidgets.QFrame(central)
        header.setObjectName("appHeader")
        header.setMinimumHeight(76)
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(16, 10, 16, 10)

        logo = QtWidgets.QLabel(header)
        logo.setFixedSize(50, 50)
        logo.setAlignment(QtCore.Qt.AlignCenter)
        try:
            logo_path = os.path.join(
                get_package_share_directory("robot_control_ui"),
                "assets", "wut_logo.png",
            )
            pixmap = QtGui.QPixmap(logo_path)
            if not pixmap.isNull():
                logo.setPixmap(
                    pixmap.scaled(
                        logo.size(), QtCore.Qt.KeepAspectRatio,
                        QtCore.Qt.SmoothTransformation,
                    )
                )
            else:
                logo.hide()
        except Exception:
            logo.hide()
        header_layout.addWidget(logo)

        heading = QtWidgets.QVBoxLayout()
        heading.setSpacing(1)
        title = QtWidgets.QLabel("履带机器人控制中心", header)
        title.setObjectName("headerTitle")
        subtitle = QtWidgets.QLabel("ROS 2 建图 · 导航 · 巡检与设备监控", header)
        subtitle.setObjectName("headerSubtitle")
        heading.addWidget(title)
        heading.addWidget(subtitle)
        header_layout.addLayout(heading)
        header_layout.addStretch(1)

        self.rqt_graph_btn = QtWidgets.QPushButton("查看 RQT Graph", header)
        self.rqt_graph_btn.setObjectName("headerTool")
        self.rqt_graph_btn.setToolTip("打开当前 ROS 2 网络的节点与话题关系图")
        self.rqt_graph_btn.clicked.connect(self.open_rqt_graph)
        header_layout.addWidget(self.rqt_graph_btn)
        self.rviz_btn = QtWidgets.QPushButton("启动 RViz", header)
        self.rviz_btn.setObjectName("headerTool")
        self.rviz_btn.setToolTip("按需打开用于建图、导航和任务点设置的 RViz")
        self.rviz_btn.clicked.connect(self.open_rviz)
        header_layout.addWidget(self.rviz_btn)
        root.addWidget(header)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal, central)
        splitter.setChildrenCollapsible(False)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(2, 2, 6, 2)
        left_layout.setSpacing(10)
        self.mission_control = MissionControlPanel(
            launch_manager=self.launch_manager, bridge=self.bridge,
            adapter=self.adapter, map_path=self.map_path, parent=left,
        )
        self.mission_control.ip_applied.connect(self._on_ip_applied)
        self.localization = LocalizationPanel(adapter=self.adapter, parent=left)
        self.mission = MissionPanel(
            bridge=self.bridge, adapter=self.adapter,
            launch_manager=self.launch_manager, parent=left,
        )
        self.log_panel = RuntimeLogPanel(left)
        self.status_panel = RobotStatusPanel(left)
        for panel in (self.mission_control, self.localization, self.mission,
                      self.log_panel, self.status_panel):
            left_layout.addWidget(panel)
        left_layout.addStretch(1)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(6, 2, 2, 2)
        right_layout.setSpacing(10)
        self.live_map = LiveMapPanel(right)
        self.thermal = ThermalPanel(launch_manager=self.launch_manager, parent=right)
        self.manual_drive = ManualDrivePanel(
            adapter=self.adapter, bridge=self.bridge,
            launch_manager=self.launch_manager, parent=right,
        )
        for panel in (self.live_map, self.thermal, self.manual_drive):
            right_layout.addWidget(panel)
        right_layout.addStretch(1)

        self.left_scroll = self._make_scroll_area(left)
        self.right_scroll = self._make_scroll_area(right)
        splitter.addWidget(self.left_scroll)
        splitter.addWidget(self.right_scroll)
        splitter.setStretchFactor(0, 58)
        splitter.setStretchFactor(1, 42)
        splitter.setSizes([820, 590])
        root.addWidget(splitter, 1)

    @staticmethod
    def _make_scroll_area(content):
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll.setWidget(content)
        return scroll

    # ------------------------------------------------------------- tools
    def open_rqt_graph(self):
        """Start one managed rqt_graph instance with the UI's ROS environment."""
        if self._rqt_process.state() != QtCore.QProcess.NotRunning:
            self.bridge.put_log("[RQT] RQT Graph 已在运行")
            return

        executable = self._find_rqt_graph()
        if not executable:
            message = "未找到 rqt_graph，请安装 ROS 2 rqt_graph 软件包。"
            self.bridge.put_log(f"[RQT] {message}")
            QtWidgets.QMessageBox.warning(self, "无法启动 RQT Graph", message)
            return

        self.rqt_graph_btn.setText("RQT Graph 启动中…")
        self.rqt_graph_btn.setEnabled(False)
        self._rqt_process.setProgram(executable)
        self._rqt_process.setArguments([])
        self._rqt_process.start()

    @staticmethod
    def _find_rqt_graph():
        return QtCore.QStandardPaths.findExecutable("rqt_graph")

    def _on_rqt_started(self):
        self.rqt_graph_btn.setText("RQT Graph 已打开")
        self.rqt_graph_btn.setEnabled(True)
        self.bridge.put_log("[RQT] RQT Graph 已启动")

    def _on_rqt_finished(self, exit_code, _exit_status):
        self.rqt_graph_btn.setText("查看 RQT Graph")
        self.rqt_graph_btn.setEnabled(True)
        self.bridge.put_log(f"[RQT] RQT Graph 已退出（代码 {exit_code}）")

    def _on_rqt_error(self, _error):
        message = self._rqt_process.errorString() or "未知错误"
        self.rqt_graph_btn.setText("查看 RQT Graph")
        self.rqt_graph_btn.setEnabled(True)
        self.bridge.put_log(f"[RQT] 启动失败：{message}")
        QtWidgets.QMessageBox.warning(
            self, "RQT Graph 启动失败", f"无法启动 RQT Graph：{message}"
        )

    def _on_rqt_output(self):
        output = bytes(self._rqt_process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        for line in output.splitlines():
            if line.strip():
                self.bridge.put_log(f"[RQT] {line}")

    def _stop_rqt_graph(self):
        if self._rqt_process.state() == QtCore.QProcess.NotRunning:
            return
        self._rqt_process.terminate()
        if not self._rqt_process.waitForFinished(1500):
            self._rqt_process.kill()
            self._rqt_process.waitForFinished(500)

    def open_rviz(self):
        if self._rviz_process.state() != QtCore.QProcess.NotRunning:
            self.bridge.put_log("[RViz] RViz 已在运行")
            return
        executable = QtCore.QStandardPaths.findExecutable("rviz2")
        if not executable:
            self._rviz_launch_error("未找到 rviz2，请安装 ROS 2 RViz 软件包。")
            return
        try:
            config = os.path.join(
                get_package_share_directory("sllidar_ros2"),
                "rviz", "sllidar_ros2.rviz",
            )
        except Exception as exc:
            self._rviz_launch_error(f"无法解析 sllidar_ros2 配置：{exc}")
            return
        if not os.path.isfile(config):
            self._rviz_launch_error(f"RViz 配置不存在：{config}")
            return
        self.rviz_btn.setText("RViz 启动中…")
        self.rviz_btn.setEnabled(False)
        self._rviz_process.setProgram(executable)
        self._rviz_process.setArguments(["-d", config])
        self._rviz_process.start()

    def _rviz_launch_error(self, message):
        self.rviz_btn.setText("启动 RViz")
        self.rviz_btn.setEnabled(True)
        self.bridge.put_log(f"[RViz] {message}")
        QtWidgets.QMessageBox.warning(self, "无法启动 RViz", message)

    def _on_rviz_started(self):
        self.rviz_btn.setText("RViz 已打开")
        self.rviz_btn.setEnabled(True)
        self.bridge.put_log("[RViz] RViz 已启动")

    def _on_rviz_finished(self, exit_code, _exit_status):
        self.rviz_btn.setText("启动 RViz")
        self.rviz_btn.setEnabled(True)
        self.bridge.put_log(f"[RViz] RViz 已退出（代码 {exit_code}）")

    def _on_rviz_error(self, _error):
        self._rviz_launch_error(self._rviz_process.errorString() or "未知错误")

    def _on_rviz_output(self):
        output = bytes(self._rviz_process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        for line in output.splitlines():
            if line.strip():
                self.bridge.put_log(f"[RViz] {line}")

    def _stop_rviz(self):
        if self._rviz_process.state() == QtCore.QProcess.NotRunning:
            return
        self._rviz_process.terminate()
        if not self._rviz_process.waitForFinished(1500):
            self._rviz_process.kill()
            self._rviz_process.waitForFinished(500)

    # ---------------------------------------------------------------- tick
    def _on_ip_applied(self, text):
        # Restart the health probe against the newly selected address.
        if self.health_probe is not None:
            self.health_probe.stop()
        from .logic.remote_health import RemoteHealthProbe

        self.health_probe = RemoteHealthProbe(
            self.remote_user() if callable(getattr(self, "remote_user", None)) else "yy",
            text,
            interval=2.0,
            callback=self.bridge.put_health,
        )
        self.health_probe.start()

    def remote_user(self):
        return self.mission_control.user_edit.text().strip() or "yy"

    def _on_tick(self):
        self.bridge.drain()
        if self.adapter is not None and hasattr(self, "status_panel"):
            now = time.time()
            mode = "idle"
            if self.launch_manager is not None:
                active = self.launch_manager.active_name
                if active in ("mapping", "navigation"):
                    mode = active
            rows = []
            for name, tracker in self.adapter.topic_trackers.items():
                snap = classify_topic(name, mode, tracker)
                rows.append((name, snap))
            self.status_panel.update_topics(rows)

            gas_stamp = float(getattr(self.adapter, "last_gas_stamp", 0.0) or 0.0)
            gas_state = (
                "waiting" if gas_stamp <= 0.0 else
                "online" if now - gas_stamp < 2.0 else "timeout"
            )
            self.status_panel.update_gas(
                getattr(self.adapter, "gas_data", None), gas_state
            )
            # live map
            map_msg = getattr(self.adapter, "map_data", None)
            if map_msg is not None:
                pose = self._robot_grid_pose()
                revision = getattr(self.adapter, "map_revision", id(map_msg))
                if revision != self._last_map_revision:
                    self.live_map.update_map(map_msg, pose=pose)
                    self._last_map_revision = revision
                    self._last_robot_grid_pose = pose
                elif pose != self._last_robot_grid_pose:
                    self.live_map.update_pose(pose)
                    self._last_robot_grid_pose = pose
            # thermal
            thermal_frame = getattr(self.adapter, "thermal_frame", None)
            thermal_stamp = float(
                getattr(self.adapter, "last_thermal_stamp", 0.0) or 0.0
            )
            thermal_state = (
                "waiting" if thermal_stamp <= 0.0 else
                "online" if now - thermal_stamp < 2.0 else "timeout"
            )
            if (thermal_state == "online" and thermal_frame and
                    thermal_stamp != self._last_thermal_render_stamp):
                self.thermal.update_frame(
                    getattr(self.adapter, "thermal_width", 32),
                    getattr(self.adapter, "thermal_height", 24),
                    thermal_frame,
                )
                self._last_thermal_render_stamp = thermal_stamp
            self.thermal.set_freshness(thermal_state)
            self.status_panel.update_thermal_lamp(thermal_state)

    def _robot_grid_pose(self):
        if self.adapter.map_data is None:
            return None
        try:
            x, y = world_to_grid(
                self.adapter.map_data, self.adapter.robot_x, self.adapter.robot_y
            )
        except Exception:
            return None
        return x, y

    # ------------------------------------------------------------- closing
    def shutdown(self):
        """Best-effort, idempotent cleanup of local and robot-side runtimes."""
        if getattr(self, "_closed", False):
            return
        self._closed = True

        cleanup_steps = []
        if hasattr(self, "_timer"):
            cleanup_steps.append(self._timer.stop)
        if hasattr(self, "_rqt_process"):
            cleanup_steps.append(self._stop_rqt_graph)
        if hasattr(self, "_rviz_process"):
            cleanup_steps.append(self._stop_rviz)
        if hasattr(self, "manual_drive"):
            cleanup_steps.append(self.manual_drive.stop_all)
        if self.health_probe is not None:
            cleanup_steps.append(self.health_probe.stop)
        if self.launch_manager is not None:
            stop_thermal = getattr(self.launch_manager, "stop_thermal", None)
            stop_runtime = getattr(self.launch_manager, "stop", None)
            if callable(stop_thermal):
                cleanup_steps.append(stop_thermal)
            if callable(stop_runtime):
                cleanup_steps.append(stop_runtime)
        if self.adapter is not None:
            cleanup_steps.append(self.adapter.shutdown)

        for step in cleanup_steps:
            try:
                step()
            except Exception:
                pass

    def closeEvent(self, event):
        self.shutdown()
        event.accept()


# ---------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------
def main(args=None):
    if args is None:
        args = sys.argv
    app = QtWidgets.QApplication(args)
    app.setStyleSheet(STYLE_QSS)

    adapter = RosUiAdapter(args=args)
    node = adapter.node
    workspace_path = node.declare_parameter(
        "workspace_path", "/home/yy/ros2_ws"
    ).value
    remote_user = node.declare_parameter("remote_user", "yy").value
    remote_host = node.declare_parameter(
        "remote_host", "192.168.43.31"
    ).value
    ros_setup_path = node.declare_parameter(
        "ros_setup_path", "/opt/ros/jazzy/setup.bash"
    ).value
    map_path = node.declare_parameter(
        "map_path", "/home/yy/ros2_ws/map_name.yaml"
    ).value

    bridge = QtBridge()
    launch_manager = LaunchManager(
        workspace_path,
        remote_user,
        remote_host,
        ros_setup_path,
        bridge.put_log,
    )
    from .logic.remote_health import RemoteHealthProbe

    health_probe = RemoteHealthProbe(
        remote_user,
        remote_host,
        interval=2.0,
        callback=bridge.put_health,
    )
    health_probe.start()
    window = QtMainWindow(
        bridge=bridge,
        adapter=adapter,
        launch_manager=launch_manager,
        health_probe=health_probe,
        map_path=map_path,
    )
    window.show()
    try:
        return app.exec_()
    finally:
        window.shutdown()


if __name__ == "__main__":
    sys.exit(main())
