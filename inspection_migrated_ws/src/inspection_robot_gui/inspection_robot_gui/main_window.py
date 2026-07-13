from glob import glob
import math
import os
import re
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QObject, QPointF, QRect, QSettings, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSlider,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtCore import Qt
from std_srvs.srv import SetBool, Trigger

from robot_monitor_interfaces.srv import Localize, StartNavigation

from .config import (
    DEFAULT_ANGULAR_SPEED_RADPS,
    DEFAULT_LINEAR_SPEED_MPS,
    DEFAULT_MAP_PATH,
    DEFAULT_ROS_SETUP_PATH,
    DEFAULT_WORKSPACE_PATH,
    MAX_ANGULAR_SPEED_RADPS,
    MAX_LINEAR_SPEED_MPS,
)
from .launch_manager import LaunchManager

try:
    import psutil
except ImportError:
    psutil = None


class GuiSignals(QObject):
    log = pyqtSignal(str)
    nav_feedback = pyqtSignal(str)
    nav_result = pyqtSignal(str)
    service_result = pyqtSignal(str, bool, str, str)
    mission_status = pyqtSignal(str, bool)


class MapView(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(230)
        self.map_msg = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.trail = []

    def set_state(self, map_msg, robot_x, robot_y, robot_yaw, trail):
        self.map_msg = map_msg
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.robot_yaw = robot_yaw
        self.trail = list(trail)
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#fcfbf7"))
        bounds = self.rect().adjusted(10, 10, -10, -10)
        painter.setPen(QPen(QColor("#c8d0d2"), 1))
        painter.drawRect(bounds)

        if self.map_msg is None or self.map_msg.info.width == 0 or self.map_msg.info.height == 0:
            painter.setPen(QColor("#6f797c"))
            painter.drawText(bounds, Qt.AlignCenter, "等待 /map")
            painter.end()
            return

        info = self.map_msg.info
        grid_w = int(info.width)
        grid_h = int(info.height)
        resolution = float(info.resolution)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        data = self.map_msg.data
        step = max(1, max(grid_w, grid_h) // 120)
        cell_w = bounds.width() / max(1.0, grid_w / step)
        cell_h = bounds.height() / max(1.0, grid_h / step)

        painter.setPen(Qt.NoPen)
        for gy in range(0, grid_h, step):
            for gx in range(0, grid_w, step):
                value = data[gy * grid_w + gx]
                if value < 0:
                    color = QColor("#d8d8d8")
                elif value > 50:
                    color = QColor("#222222")
                else:
                    continue
                x0 = bounds.left() + (gx / step) * cell_w
                y0 = bounds.bottom() - ((gy / step) + 1) * cell_h
                painter.fillRect(int(x0), int(y0), int(cell_w) + 1, int(cell_h) + 1, color)

        trail_points = [
            self._world_to_view(x, y, origin_x, origin_y, resolution, grid_w, grid_h, bounds)
            for x, y in self.trail
        ]
        if len(trail_points) >= 2:
            painter.setPen(QPen(QColor("#1f77d0"), 2))
            for start, end in zip(trail_points, trail_points[1:]):
                painter.drawLine(start, end)

        robot = self._world_to_view(
            self.robot_x,
            self.robot_y,
            origin_x,
            origin_y,
            resolution,
            grid_w,
            grid_h,
            bounds,
        )
        heading = QPointF(math.cos(self.robot_yaw) * 13.0, -math.sin(self.robot_yaw) * 13.0)
        left = QPointF(math.cos(self.robot_yaw + 2.45) * 7.0, -math.sin(self.robot_yaw + 2.45) * 7.0)
        right = QPointF(math.cos(self.robot_yaw - 2.45) * 7.0, -math.sin(self.robot_yaw - 2.45) * 7.0)
        painter.setBrush(QBrush(QColor("#d94141")))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(QPolygonF([robot + heading, robot + left, robot + right]))
        painter.setBrush(QBrush(QColor("#ffffff")))
        painter.drawEllipse(robot, 3.0, 3.0)
        painter.end()

    @staticmethod
    def _world_to_view(x, y, origin_x, origin_y, resolution, grid_w, grid_h, bounds):
        map_w = max(resolution * grid_w, 1e-9)
        map_h = max(resolution * grid_h, 1e-9)
        rel_x = max(0.0, min(1.0, (x - origin_x) / map_w))
        rel_y = max(0.0, min(1.0, (y - origin_y) / map_h))
        return QPointF(bounds.left() + rel_x * bounds.width(), bounds.bottom() - rel_y * bounds.height())


class ThermalView(QWidget):
    """Pure Qt heatmap for the real MLX90640 Float32MultiArray stream."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(190)
        self.frame = []
        self.width = 0
        self.height = 0
        self.minimum = 0.0
        self.maximum = 0.0
        self.average = 0.0
        self.error = ""

    def set_state(self, frame, width, height, minimum, maximum, average, error=""):
        self.frame = list(frame or [])
        self.width = int(width or 0)
        self.height = int(height or 0)
        self.minimum = float(minimum or 0.0)
        self.maximum = float(maximum or 0.0)
        self.average = float(average or 0.0)
        self.error = str(error or "")
        self.update()

    def save_snapshot(self, path):
        thermal_image = self._render_image()
        if thermal_image is None:
            return False
        # Export a readable report-sized image instead of the raw 32x24 pixel
        # buffer.  This keeps the color scale and summary values available
        # when the snapshot is reviewed away from the live GUI.
        canvas = QImage(960, 620, QImage.Format_RGB32)
        canvas.fill(QColor("#202729"))
        painter = QPainter(canvas)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        painter.setPen(QColor("#e7eeee"))
        painter.drawText(40, 30, "MLX90640 thermal frame")

        image_bounds = QRect(40, 50, 760, 520)
        scaled_size = thermal_image.size().scaled(image_bounds.size(), Qt.KeepAspectRatio)
        image_rect = QRect(0, 0, scaled_size.width(), scaled_size.height())
        image_rect.moveCenter(image_bounds.center())
        painter.drawImage(image_rect, thermal_image)
        painter.setPen(QPen(QColor("#aebabc"), 1))
        painter.drawRect(image_rect)

        bar_left = 840
        for index in range(image_bounds.height()):
            ratio = 1.0 - index / max(1, image_bounds.height() - 1)
            painter.fillRect(bar_left, image_bounds.top() + index, 24, 1, self._temperature_color(ratio))
        painter.setPen(QColor("#e7eeee"))
        painter.drawText(bar_left + 31, image_bounds.top() + 8, f"{self.maximum:.1f} C")
        painter.drawText(bar_left + 31, image_bounds.bottom(), f"{self.minimum:.1f} C")
        painter.drawText(
            40,
            600,
            f"{self.width}x{self.height}    min={self.minimum:.1f} C    "
            f"max={self.maximum:.1f} C    avg={self.average:.1f} C",
        )
        painter.end()
        return bool(canvas.save(path, "PNG"))

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#202729"))
        bounds = self.rect().adjusted(8, 8, -8, -8)
        image = self._render_image()
        if image is None:
            painter.setPen(QColor("#d9e1e2"))
            message = self.error or "等待 /thermal_frame"
            painter.drawText(bounds, Qt.AlignCenter, message)
            painter.end()
            return

        colorbar_width = 24
        image_bounds = bounds.adjusted(0, 0, -(colorbar_width + 34), 0)
        scaled_size = image.size().scaled(image_bounds.size(), Qt.KeepAspectRatio)
        image_rect = QRect(0, 0, scaled_size.width(), scaled_size.height())
        image_rect.moveCenter(image_bounds.center())
        painter.drawImage(image_rect, image)
        painter.setPen(QPen(QColor("#aebabc"), 1))
        painter.drawRect(image_rect)

        bar_left = image_bounds.right() + 14
        bar_top = bounds.top()
        bar_height = bounds.height()
        for index in range(bar_height):
            ratio = 1.0 - index / max(1, bar_height - 1)
            color = self._temperature_color(ratio)
            painter.fillRect(bar_left, bar_top + index, colorbar_width, 1, color)
        painter.setPen(QColor("#e7eeee"))
        painter.drawText(bar_left + colorbar_width + 5, bar_top + 8, f"{self.maximum:.1f}")
        painter.drawText(bar_left + colorbar_width + 5, bar_top + bar_height, f"{self.minimum:.1f}")
        painter.end()

    def _render_image(self):
        if (
            not self.frame
            or self.width <= 0
            or self.height <= 0
            or self.width * self.height != len(self.frame)
        ):
            return None
        image = QImage(self.width, self.height, QImage.Format_RGB32)
        span = max(self.maximum - self.minimum, 1e-6)
        for row in range(self.height):
            for column in range(self.width):
                value = self.frame[row * self.width + column]
                ratio = max(0.0, min(1.0, (value - self.minimum) / span))
                image.setPixelColor(column, row, self._temperature_color(ratio))
        return image

    @staticmethod
    def _temperature_color(ratio):
        # High-contrast blue → cyan → yellow → red scale without Matplotlib.
        hue = int(max(0.0, min(1.0, ratio)) * 240.0)
        return QColor.fromHsv(240 - hue, 235, 255)


class ThermalDetailDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("红外热成像详情")
        self.resize(900, 620)
        layout = QVBoxLayout(self)
        self.view = ThermalView(self)
        self.view.setMinimumSize(720, 480)
        layout.addWidget(self.view, 1)
        self.stats_label = QLabel("等待热成像数据")
        self.stats_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.stats_label)

    def set_state(self, snap):
        self.view.set_state(
            snap.get("thermal_frame", []),
            snap.get("thermal_width", 0),
            snap.get("thermal_height", 0),
            snap.get("thermal_min", 0.0),
            snap.get("thermal_max", 0.0),
            snap.get("thermal_avg", 0.0),
            snap.get("thermal_error", ""),
        )
        if snap.get("thermal_error"):
            self.stats_label.setText(snap["thermal_error"])
        else:
            self.stats_label.setText(
                f"尺寸 {snap.get('thermal_width', 0)}x{snap.get('thermal_height', 0)}  "
                f"最小 {snap.get('thermal_min', 0.0):.1f}°C  "
                f"最大 {snap.get('thermal_max', 0.0):.1f}°C  "
                f"平均 {snap.get('thermal_avg', 0.0):.1f}°C"
            )


class MainWindow(QMainWindow):
    def __init__(self, ros_adapter, signals):
        super().__init__()
        self.ros = ros_adapter
        self.signals = signals
        self.settings = QSettings("inspection_robot", "inspection_robot_gui")

        node = self.ros.node
        workspace_default = node.declare_parameter("workspace_path", DEFAULT_WORKSPACE_PATH).value
        map_default = node.declare_parameter("map_path", DEFAULT_MAP_PATH).value
        ros_setup_default = node.declare_parameter("ros_setup_path", DEFAULT_ROS_SETUP_PATH).value
        remote_user = node.declare_parameter("remote_user", "yy").value
        remote_host = node.declare_parameter("remote_host", "192.168.43.21").value

        self.launch_manager = LaunchManager(
            workspace_default, ros_setup_default, remote_user, remote_host
        )
        self.launch_manager.log_line.connect(self.append_log)
        self.launch_manager.state_changed.connect(self._launch_state_changed)
        self.signals.log.connect(self.append_log)
        self.signals.nav_feedback.connect(self.append_log)
        self.signals.nav_result.connect(self._nav_result)
        self.signals.service_result.connect(self._service_result)
        self.signals.mission_status.connect(self._mission_status)
        self._pending_initial_pose = None
        self._initial_pose_retries_remaining = 0
        self._last_manual_takeover_request = 0.0
        self.pose_history = []
        self._last_system_status_update = 0.0
        self._system_status_cache = {
            "host": "主机: 等待数据",
            "gpu": "GPU: 等待数据",
            "temp": "温度: 等待数据",
        }
        self.thermal_detail_dialog = None
        self.thermal_view = ThermalView()

        self.setWindowTitle("巡检小车实车巡检控制台")
        self.resize(1260, 780)

        self.workspace_edit = QLineEdit(
            self.settings.value("workspace_path", workspace_default)
        )
        self.ros_setup_edit = QLineEdit(
            self.settings.value("ros_setup_path", ros_setup_default)
        )
        self.remote_user_edit = QLineEdit(
            self.settings.value("remote_user", remote_user)
        )
        self.remote_host_edit = QLineEdit(
            self.settings.value("remote_host", remote_host)
        )
        self.map_combo = QComboBox()
        self.map_combo.setEditable(True)
        self.map_combo.addItem(self.settings.value("map_path", map_default))
        # Keep a long absolute map path from making the whole left column
        # wider than the thermal/status column on a normal laptop display.
        self.map_combo.setMinimumContentsLength(14)
        self.map_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.use_rviz_check = QCheckBox("启动 RViz")
        self.use_rviz_check.setChecked(self.settings.value("use_rviz", "true") == "true")
        self.headless_check = QCheckBox("远程无界面")
        self.headless_check.setChecked(self.settings.value("headless", "false") == "true")

        self.linear_spin = self._double_spin(0.0, MAX_LINEAR_SPEED_MPS, DEFAULT_LINEAR_SPEED_MPS, 0.01)
        self.angular_spin = self._double_spin(0.0, MAX_ANGULAR_SPEED_RADPS, DEFAULT_ANGULAR_SPEED_RADPS, 0.01)
        self.linear_slider = self._speed_slider(self.linear_spin, MAX_LINEAR_SPEED_MPS)
        self.angular_slider = self._speed_slider(self.angular_spin, MAX_ANGULAR_SPEED_RADPS)

        self.init_x_spin = self._double_spin(-20.0, 20.0, 0.0, 0.05)
        self.init_y_spin = self._double_spin(-20.0, 20.0, 0.0, 0.05)
        self.init_yaw_spin = self._double_spin(-180.0, 180.0, 0.0, 1.0)
        try:
            pause_default = float(self.settings.value("waypoint_pause_sec", 2.0))
        except (TypeError, ValueError):
            pause_default = 2.0
        self.waypoint_pause_spin = self._double_spin(0.0, 60.0, pause_default, 0.5)

        self.pose_label = QLabel("里程计: 等待数据")
        self.velocity_label = QLabel("速度: 等待数据")
        self.wheel_odom_label = QLabel("轮速里程计: 实车未提供")
        self.amcl_label = QLabel("AMCL: 等待数据")
        self.map_label = QLabel("地图: 等待数据")
        self.nav_label = QLabel("导航: 空闲")
        self.launch_label = QLabel("启动状态: 空闲")
        self.host_label = QLabel("主机: 等待数据")
        self.gpu_label = QLabel("GPU: 等待数据")
        self.temp_label = QLabel("温度: 等待数据")
        self.gas_label = QLabel("气体: 等待数据")
        self.thermal_label = QLabel("热成像: 等待数据")
        self.safety_label = QLabel("安全: 等待数据")
        self.gas_status_label = QLabel("状态: 等待气体数据")
        self.gas_h2_label = QLabel("H2: --")
        self.gas_co_label = QLabel("CO: --")
        self.gas_voc_label = QLabel("VOC: --")
        self.gas_smoke_label = QLabel("Smoke: --")
        self.thermal_change_label = QLabel("温度变化: 未达到 60 s 基线窗口")
        self.mission_label = QLabel("任务: 等待服务")
        self.mission_label.setWordWrap(True)
        self.region_mode_check = QCheckBox("区域模式")
        self.map_view = MapView()
        self.topic_labels = {
            "scan": QLabel("/scan"),
            "imu": QLabel("/imu"),
            "laser_odom": QLabel("/laser_odom"),
            "wheel_odom": QLabel("/wheel_odom"),
            "odom": QLabel("/odom"),
            "map": QLabel("/map"),
            "amcl": QLabel("/amcl_pose"),
        }
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1200)

        self._build_ui()
        self._apply_style()
        self._connect()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(300)

        self.initial_pose_retry_timer = QTimer(self)
        self.initial_pose_retry_timer.timeout.connect(self._retry_initial_pose)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        header = QHBoxLayout()
        logo = QLabel()
        logo_pix = QPixmap(self._asset_path("wut_logo.png"))
        if not logo_pix.isNull():
            logo.setPixmap(logo_pix.scaledToHeight(46, Qt.SmoothTransformation))
            logo.setFixedSize(58, 50)
            logo.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        title = QLabel("巡检小车实车巡检控制台")
        title.setObjectName("Title")
        header.addWidget(logo)
        header.addWidget(title, 1)
        root.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter, 1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(10)
        left_layout.addWidget(self._build_launch_group())
        left_layout.addWidget(self._build_pose_group())
        left_layout.addWidget(self._build_log_group(), 1)
        splitter.addWidget(left)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        right_layout.addWidget(self._build_mission_group())
        right_layout.addWidget(self._build_map_group())
        right_layout.addWidget(self._build_thermal_group())
        right_layout.addWidget(self._build_drive_group())
        right_layout.addWidget(self._build_status_group(), 1)
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QScrollArea.NoFrame)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        right_scroll.setWidget(right)
        splitter.addWidget(right_scroll)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setChildrenCollapsible(False)
        splitter.setSizes([int(1260 * 0.6), int(1260 * 0.4)])

    def _build_launch_group(self):
        group = QGroupBox("启动控制")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("工作空间"), 0, 0)
        layout.addWidget(self.workspace_edit, 0, 1, 1, 3)
        browse_workspace = QPushButton("浏览")
        browse_workspace.clicked.connect(self._browse_workspace)
        layout.addWidget(browse_workspace, 0, 4)

        layout.addWidget(QLabel("ROS 环境"), 1, 0)
        layout.addWidget(self.ros_setup_edit, 1, 1, 1, 3)
        browse_setup = QPushButton("浏览")
        browse_setup.clicked.connect(self._browse_ros_setup)
        layout.addWidget(browse_setup, 1, 4)

        layout.addWidget(QLabel("远程用户"), 2, 0)
        layout.addWidget(self.remote_user_edit, 2, 1, 1, 1)
        layout.addWidget(QLabel("远程主机/IP"), 2, 2)
        layout.addWidget(self.remote_host_edit, 2, 3, 1, 2)

        layout.addWidget(QLabel("地图 YAML"), 3, 0)
        layout.addWidget(self.map_combo, 3, 1, 1, 2)
        refresh_maps = QPushButton("刷新地图")
        refresh_maps.clicked.connect(self.refresh_maps)
        layout.addWidget(refresh_maps, 3, 3)
        browse_map = QPushButton("浏览")
        browse_map.clicked.connect(self._browse_map)
        layout.addWidget(browse_map, 3, 4)

        layout.addWidget(self.use_rviz_check, 4, 1)
        layout.addWidget(self.headless_check, 4, 2)

        buttons = QHBoxLayout()
        for text, handler in [
            ("开始建图", self.start_mapping),
            ("开始导航", self.start_navigation),
            ("启动传感器", self.start_thermal),
            ("保存地图", self.save_map),
            ("停止启动项", self.stop_launch),
        ]:
            button = QPushButton(text)
            button.clicked.connect(handler)
            buttons.addWidget(button)
        layout.addLayout(buttons, 5, 0, 1, 5)
        return group

    def _build_pose_group(self):
        group = QGroupBox("定位与目标点")
        layout = QGridLayout(group)

        init_form = QFormLayout()
        init_form.addRow("初始 X", self.init_x_spin)
        init_form.addRow("初始 Y", self.init_y_spin)
        init_form.addRow("初始航向 deg", self.init_yaw_spin)
        layout.addLayout(init_form, 0, 0)

        init_buttons = QVBoxLayout()
        set_initial = QPushButton("设置初始位姿")
        set_initial.clicked.connect(self.set_initial_pose)
        init_buttons.addWidget(set_initial)
        init_buttons.addStretch(1)
        layout.addLayout(init_buttons, 0, 1)
        layout.setColumnStretch(2, 1)
        return group

    def _build_mission_group(self):
        group = QGroupBox("任务与区域")
        layout = QVBoxLayout(group)

        top = QHBoxLayout()
        check_localization = QPushButton("检查定位")
        check_localization.clicked.connect(self.check_localization)
        start_mission = QPushButton("开始任务")
        start_mission.clicked.connect(self.start_mission)
        stop_all = QPushButton("停止所有任务")
        stop_all.clicked.connect(self.stop_all_tasks)
        emergency_stop = QPushButton("紧急停止（锁存）")
        emergency_stop.setStyleSheet("background:#a00000;color:white;font-weight:bold;")
        emergency_stop.clicked.connect(self.emergency_stop)
        reset_emergency_stop = QPushButton("复位软件急停")
        reset_emergency_stop.clicked.connect(self.reset_emergency_stop)
        clear_points = QPushButton("清空点位")
        clear_points.clicked.connect(self.clear_rviz_points)
        top.addWidget(check_localization)
        top.addWidget(start_mission)
        top.addWidget(stop_all)
        top.addWidget(emergency_stop)
        top.addWidget(reset_emergency_stop)
        top.addWidget(clear_points)
        layout.addLayout(top)

        params = QFormLayout()
        params.addRow("点位停留时间 s", self.waypoint_pause_spin)
        layout.addLayout(params)

        region = QHBoxLayout()
        self.region_mode_check.clicked.connect(self.set_region_mode)
        save_regions = QPushButton("保存区域")
        save_regions.clicked.connect(self.save_regions)
        load_regions = QPushButton("加载区域")
        load_regions.clicked.connect(self.load_regions)
        clear_regions = QPushButton("清空区域")
        clear_regions.clicked.connect(self.clear_regions)
        region.addWidget(self.region_mode_check)
        region.addWidget(save_regions)
        region.addWidget(load_regions)
        region.addWidget(clear_regions)
        layout.addLayout(region)

        undo_row = QHBoxLayout()
        undo_region = QPushButton("撤销区域")
        undo_region.clicked.connect(self.undo_last_region)
        undo_point_btn = QPushButton("撤销点位")
        undo_point_btn.clicked.connect(self.undo_last_rviz_point)
        undo_row.addWidget(undo_region)
        undo_row.addWidget(undo_point_btn)
        undo_row.addStretch(1)
        layout.addLayout(undo_row)

        self.mission_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.mission_label)
        return group

    def _build_map_group(self):
        group = QGroupBox("实时地图")
        layout = QVBoxLayout(group)
        layout.addWidget(self.map_view)
        return group

    def _build_thermal_group(self):
        group = QGroupBox("红外热成像 / MLX90640")
        layout = QVBoxLayout(group)
        layout.addWidget(self.thermal_view)
        self.thermal_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.thermal_change_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.thermal_label)
        layout.addWidget(self.thermal_change_label)

        buttons = QHBoxLayout()
        open_detail = QPushButton("打开详情")
        open_detail.clicked.connect(self.open_thermal_detail)
        save_snapshot = QPushButton("保存截图")
        save_snapshot.clicked.connect(self.save_thermal_snapshot)
        stop_sensor = QPushButton("停止传感器")
        stop_sensor.clicked.connect(self.stop_thermal)
        buttons.addWidget(open_detail)
        buttons.addWidget(save_snapshot)
        buttons.addWidget(stop_sensor)
        layout.addLayout(buttons)
        return group

    def _build_drive_group(self):
        group = QGroupBox("手动行进")
        layout = QVBoxLayout(group)

        speed_form = QFormLayout()
        speed_form.addRow("线速度 m/s", self.linear_spin)
        speed_form.addRow("", self.linear_slider)
        speed_form.addRow("角速度 rad/s", self.angular_spin)
        speed_form.addRow("", self.angular_slider)
        layout.addLayout(speed_form)

        pad = QGridLayout()
        buttons = {
            (0, 1): ("W", self.drive_forward),
            (1, 0): ("A", self.turn_left),
            (1, 1): ("Space 空格", self.stop_robot),
            (1, 2): ("D", self.turn_right),
            (2, 1): ("S", self.drive_backward),
        }
        for (row, col), (text, handler) in buttons.items():
            button = QPushButton(text)
            button.setMinimumHeight(44)
            button.clicked.connect(handler)
            pad.addWidget(button, row, col)
        layout.addLayout(pad)

        arc = QHBoxLayout()
        left = QPushButton("前进左转")
        right = QPushButton("前进右转")
        left.clicked.connect(self.forward_left)
        right.clicked.connect(self.forward_right)
        arc.addWidget(left)
        arc.addWidget(right)
        layout.addLayout(arc)

        return group

    def _build_status_group(self):
        group = QGroupBox("实时状态")
        layout = QVBoxLayout(group)

        topics = QGridLayout()
        for index, (name, label) in enumerate(self.topic_labels.items()):
            row = index // 3
            col = index % 3
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(30)
            topics.addWidget(label, row, col)
        layout.addLayout(topics)

        for label in [
            self.pose_label,
            self.velocity_label,
            self.wheel_odom_label,
            self.amcl_label,
            self.map_label,
            self.nav_label,
            self.launch_label,
            self.host_label,
            self.gpu_label,
            self.temp_label,
            self.safety_label,
        ]:
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            layout.addWidget(label)
        layout.addWidget(self._build_gas_group())
        layout.addStretch(1)
        return group

    def _build_gas_group(self):
        group = QGroupBox("气体传感器")
        layout = QVBoxLayout(group)
        self.gas_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.gas_status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.gas_label)
        layout.addWidget(self.gas_status_label)

        cards = QGridLayout()
        for index, label in enumerate(
            [self.gas_h2_label, self.gas_co_label, self.gas_voc_label, self.gas_smoke_label]
        ):
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(28)
            label.setStyleSheet(
                "background: #eef1f1; color: #263238; border-radius: 4px; padding: 4px;"
            )
            cards.addWidget(label, index // 2, index % 2)
        layout.addLayout(cards)
        threshold = QLabel("浓度阈值: 未配置（当前仅显示数据，不触发停机）")
        threshold.setStyleSheet("color: #667174;")
        layout.addWidget(threshold)
        return group

    def _build_log_group(self):
        group = QGroupBox("运行日志")
        layout = QVBoxLayout(group)
        layout.addWidget(self.log_view)
        return group

    def _connect(self):
        self.linear_spin.valueChanged.connect(
            lambda value: self.linear_slider.setValue(int(value / MAX_LINEAR_SPEED_MPS * 100))
        )
        self.angular_spin.valueChanged.connect(
            lambda value: self.angular_slider.setValue(int(value / MAX_ANGULAR_SPEED_RADPS * 100))
        )
        self.linear_slider.valueChanged.connect(
            lambda value: self.linear_spin.setValue(MAX_LINEAR_SPEED_MPS * value / 100.0)
        )
        self.angular_slider.valueChanged.connect(
            lambda value: self.angular_spin.setValue(MAX_ANGULAR_SPEED_RADPS * value / 100.0)
        )

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f0f2f2; color: #202426; }
            QLabel#Title { font-size: 22px; font-weight: 700; padding: 6px 0; }
            QLabel#Hint { color: #667174; }
            QGroupBox {
                font-weight: 700;
                border: 1px solid #c5cccc;
                border-radius: 6px;
                margin-top: 12px;
                padding: 12px;
                background: #ffffff;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QLineEdit, QDoubleSpinBox, QPlainTextEdit {
                background: #ffffff;
                border: 1px solid #b8c0c2;
                border-radius: 4px;
                padding: 5px;
            }
            QPushButton {
                background: #2f5f73;
                color: white;
                border: 0;
                border-radius: 5px;
                padding: 8px 10px;
                font-weight: 600;
            }
            QPushButton:hover { background: #39758d; }
            QPushButton:pressed { background: #234758; }
            """
        )

    def _asset_path(self, filename):
        try:
            return os.path.join(
                get_package_share_directory("inspection_robot_gui"),
                "assets",
                filename,
            )
        except Exception:
            return os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "assets", filename)
            )

    def _double_spin(self, minimum, maximum, value, step):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(3 if step < 0.1 else 2)
        spin.setButtonSymbols(QAbstractSpinBox.PlusMinus)
        return spin

    def _speed_slider(self, spin, max_value):
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(int(spin.value() / max_value * 100))
        return slider

    def _apply_launch_paths(self):
        workspace = self.workspace_edit.text().strip() or DEFAULT_WORKSPACE_PATH
        ros_setup = self.ros_setup_edit.text().strip() or DEFAULT_ROS_SETUP_PATH
        remote_user = self.remote_user_edit.text().strip() or "yy"
        remote_host = self.remote_host_edit.text().strip() or "192.168.43.21"
        self.remote_user_edit.setText(remote_user)
        self.remote_host_edit.setText(remote_host)
        self.launch_manager.update_paths(
            workspace, ros_setup, remote_user=remote_user, remote_host=remote_host
        )
        self.settings.setValue("workspace_path", workspace)
        self.settings.setValue("ros_setup_path", ros_setup)
        self.settings.setValue("remote_user", remote_user)
        self.settings.setValue("remote_host", remote_host)
        map_path = self._normalize_map_path(self._map_text())
        self._set_map_text(map_path)
        self.settings.setValue("map_path", map_path)
        self.settings.setValue("use_rviz", "true" if self.use_rviz_check.isChecked() else "false")
        self.settings.setValue("headless", "true" if self.headless_check.isChecked() else "false")

    def start_mapping(self):
        self._apply_launch_paths()
        self.launch_manager.start_mapping()

    def start_thermal(self):
        self._apply_launch_paths()
        self.launch_manager.start_thermal()

    def stop_thermal(self):
        self.launch_manager.stop_thermal()
        self.append_log("[SENSOR] 已请求停止热成像/气体节点")

    def open_thermal_detail(self):
        if self.thermal_detail_dialog is None:
            self.thermal_detail_dialog = ThermalDetailDialog(self)
        self.thermal_detail_dialog.set_state(self.ros.snapshot())
        self.thermal_detail_dialog.show()
        self.thermal_detail_dialog.raise_()
        self.thermal_detail_dialog.activateWindow()

    def save_thermal_snapshot(self):
        snap = self.ros.snapshot()
        if not snap.get("thermal_frame"):
            QMessageBox.information(self, "红外截图", "当前没有有效的热成像帧。")
            return
        output_dir = os.path.join(os.path.expanduser("~"), "Pictures", "thermal_snapshots")
        try:
            os.makedirs(output_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(output_dir, f"thermal_{timestamp}.png")
            if not self.thermal_view.save_snapshot(output_path):
                raise RuntimeError("热成像图像生成失败")
        except Exception as exc:
            self.append_log(f"[THERMAL] 截图失败: {exc}")
            QMessageBox.warning(self, "红外截图", str(exc))
            return
        self.append_log(f"[THERMAL] 截图已保存: {output_path}")
        QMessageBox.information(self, "红外截图", f"已保存到:\n{output_path}")

    def start_navigation(self):
        self._apply_launch_paths()
        map_path = self._normalize_map_path(self._map_text())
        if not map_path:
            QMessageBox.warning(self, "缺少地图路径", "启动导航前需要设置地图 YAML。")
            return
        self._set_map_text(map_path)
        self.launch_manager.start_navigation(map_path, self.use_rviz_check.isChecked())

    def save_map(self):
        self._apply_launch_paths()
        map_path = self._normalize_map_path(self._map_text())
        if not map_path:
            QMessageBox.warning(self, "缺少地图路径", "请先设置输出地图 YAML 路径。")
            return
        self._set_map_text(map_path)
        self.launch_manager.save_map(map_path)

    def stop_launch(self):
        self.launch_manager.stop_thermal()
        self.launch_manager.stop()

    def check_localization(self):
        self.append_log("[MISSION] check localization")
        self.ros.call_service_async(
            self.ros.localize_client,
            Localize.Request(),
            lambda result, error: self._emit_service_result("检查定位", result, error),
        )

    def start_mission(self):
        request = StartNavigation.Request()
        request.waypoint_pause_sec = float(self.waypoint_pause_spin.value())
        self.settings.setValue("waypoint_pause_sec", request.waypoint_pause_sec)
        self.append_log(f"[MISSION] start mission pause={request.waypoint_pause_sec:.1f}s")
        self.ros.call_service_async(
            self.ros.start_navigation_client,
            request,
            lambda result, error: self._emit_service_result("开始任务", result, error),
            timeout_sec=10.0,
        )

    def stop_all_tasks(self):
        self.append_log("[MISSION] stop all tasks")
        self.ros.publish_cmd_vel(0.0, 0.0)
        self.ros.cancel_nav_goal()
        self.ros.call_service_async(
            self.ros.abort_mission_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("停止所有任务", result, error),
            timeout_sec=6.0,
        )

    def emergency_stop(self):
        self.append_log("[SAFETY] software emergency stop requested")
        self.ros.publish_cmd_vel(0.0, 0.0)
        self.ros.cancel_nav_goal()
        request = SetBool.Request()
        request.data = True
        self.ros.call_service_async(
            self.ros.emergency_stop_client,
            request,
            lambda result, error: self._emit_service_result("软件急停", result, error),
            timeout_sec=1.0,
        )

    def reset_emergency_stop(self):
        reply = QMessageBox.question(
            self,
            "复位软件急停",
            "仅在已排除故障、机器人处于安全状态且物理急停已释放时复位。是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        self.ros.call_service_async(
            self.ros.reset_safety_monitor_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("复位软件急停", result, error),
            timeout_sec=3.0,
        )

    def clear_rviz_points(self):
        self.append_log("[MISSION] clear RViz points")
        self.ros.call_service_async(
            self.ros.clear_rviz_points_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("清空点位", result, error),
        )

    def set_region_mode(self):
        request = SetBool.Request()
        request.data = bool(self.region_mode_check.isChecked())
        self.append_log(f"[MISSION] region mode {request.data}")
        self.ros.call_service_async(
            self.ros.set_region_mode_client,
            request,
            lambda result, error: self._emit_service_result("区域模式", result, error),
        )

    def save_regions(self):
        self._call_region_trigger(
            self.ros.save_inspection_regions_client,
            "保存区域",
            "[MISSION] save regions",
        )

    def load_regions(self):
        self._call_region_trigger(
            self.ros.load_inspection_regions_client,
            "加载区域",
            "[MISSION] load regions",
        )

    def clear_regions(self):
        self._call_region_trigger(
            self.ros.clear_inspection_regions_client,
            "清空区域",
            "[MISSION] clear regions",
        )

    def undo_last_region(self):
        self.append_log("[MISSION] undo last region")
        self.ros.call_service_async(
            self.ros.undo_last_region_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("撤销区域", result, error),
        )

    def undo_last_rviz_point(self):
        self.append_log("[MISSION] undo last point")
        self.ros.call_service_async(
            self.ros.undo_last_point_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("撤销点位", result, error),
        )

    def _call_region_trigger(self, client, title, log_line):
        self.append_log(log_line)
        self.ros.call_service_async(
            client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result(title, result, error),
        )

    def _emit_service_result(self, title, result, error):
        if error is not None:
            self.signals.service_result.emit(title, False, "", error)
            return
        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", ""))
        self.signals.service_result.emit(title, success, message, "")

    def _service_result(self, title, success, message, error):
        if error:
            self.append_log(f"[ERROR] {title}: {error}")
            self.mission_label.setText(f"任务: {title} 失败")
            if title == "区域模式":
                self.region_mode_check.blockSignals(True)
                self.region_mode_check.setChecked(not self.region_mode_check.isChecked())
                self.region_mode_check.blockSignals(False)
            QMessageBox.warning(self, title, error)
            return

        prefix = "[MISSION]" if success else "[WARN]"
        self.append_log(f"{prefix} {title}: {message}")
        self.mission_label.setText(f"任务: {message or title}")
        if title == "区域模式" and not success:
            self.region_mode_check.blockSignals(True)
            self.region_mode_check.setChecked(not self.region_mode_check.isChecked())
            self.region_mode_check.blockSignals(False)
        if success:
            QMessageBox.information(self, title, message or "完成")
        else:
            QMessageBox.warning(self, title, message or "未完成")

    def _mission_status(self, message, safety):
        if not message:
            return
        if safety:
            self.append_log(f"[SAFETY] {message}")
            self.mission_label.setText(f"安全提醒: {message}")
            QMessageBox.warning(self, "安全提醒", message)
        else:
            self.append_log(f"[MISSION] {message}")
            self.mission_label.setText(f"任务: {message}")

    def set_initial_pose(self):
        self._pending_initial_pose = (
            self.init_x_spin.value(),
            self.init_y_spin.value(),
            math.radians(self.init_yaw_spin.value()),
        )
        self._initial_pose_retries_remaining = 8
        self._publish_pending_initial_pose(force=True)
        self.initial_pose_retry_timer.start(1000)

    def _publish_pending_initial_pose(self, force=False):
        if self._pending_initial_pose is None:
            return
        if not force:
            snap = self.ros.snapshot()
            if time.monotonic() - snap["last_amcl"] < 2.0:
                self.initial_pose_retry_timer.stop()
                self._pending_initial_pose = None
                self.append_log("[ROS] AMCL pose is active; initial pose retry stopped")
                return
        self.ros.publish_initial_pose(*self._pending_initial_pose)
        self._initial_pose_retries_remaining -= 1
        if self._initial_pose_retries_remaining <= 0:
            self.initial_pose_retry_timer.stop()
            self._pending_initial_pose = None
            self.append_log("[ROS] initial pose retry window ended")

    def _retry_initial_pose(self):
        self._publish_pending_initial_pose()

    def copy_odom_to_initial(self):
        snap = self.ros.snapshot()
        self.init_x_spin.setValue(snap["x"])
        self.init_y_spin.setValue(snap["y"])
        self.init_yaw_spin.setValue(math.degrees(snap["yaw"]))

    def drive_forward(self):
        self._begin_manual_takeover()
        self.ros.publish_cmd_vel(self.linear_spin.value(), 0.0)

    def drive_backward(self):
        self._begin_manual_takeover()
        self.ros.publish_cmd_vel(-self.linear_spin.value(), 0.0)

    def turn_left(self):
        self._begin_manual_takeover()
        self.ros.publish_cmd_vel(0.0, self.angular_spin.value())

    def turn_right(self):
        self._begin_manual_takeover()
        self.ros.publish_cmd_vel(0.0, -self.angular_spin.value())

    def forward_left(self):
        self._begin_manual_takeover()
        self.ros.publish_cmd_vel(self.linear_spin.value(), self.angular_spin.value() * 0.5)

    def forward_right(self):
        self._begin_manual_takeover()
        self.ros.publish_cmd_vel(self.linear_spin.value(), -self.angular_spin.value() * 0.5)

    def stop_robot(self):
        self._begin_manual_takeover()
        self.ros.publish_cmd_vel(0.0, 0.0)

    def _begin_manual_takeover(self):
        """Make a GUI command preempt autonomous motion before publishing it."""
        self.ros.cancel_nav_goal()
        now = time.monotonic()
        # Key-repeat and slider use must not create unbounded abort service calls.
        if now - self._last_manual_takeover_request < 0.5:
            return
        self._last_manual_takeover_request = now
        self.ros.call_service_async(
            self.ros.abort_mission_client,
            Trigger.Request(),
            lambda result, error: self._emit_service_result("手动接管", result, error),
            timeout_sec=1.0,
        )

    def keyPressEvent(self, event):
        key = event.key()
        if key in (Qt.Key_W, Qt.Key_Up):
            self.drive_forward()
        elif key in (Qt.Key_S, Qt.Key_Down):
            self.drive_backward()
        elif key in (Qt.Key_A, Qt.Key_Left):
            self.turn_left()
        elif key in (Qt.Key_D, Qt.Key_Right):
            self.turn_right()
        elif key == Qt.Key_Space:
            self.stop_robot()
        else:
            super().keyPressEvent(event)

    def append_log(self, line):
        if not line:
            return
        self.log_view.appendPlainText(line)
        bar = self.log_view.verticalScrollBar()
        bar.setValue(bar.maximum())

    def _nav_result(self, line):
        self.append_log(line)
        self.nav_label.setText("导航: " + line)

    def _launch_state_changed(self, state):
        self.launch_label.setText(f"启动状态: {state}")

    def _refresh_status(self):
        snap = self.ros.snapshot()
        now = time.monotonic()
        self._record_pose(snap["x"], snap["y"])
        self._set_topic("scan", now - snap["last_scan"], f"scan {snap['scan_count']} {snap['scan_frame']}")
        self._set_topic("imu", now - snap["last_imu"], f"imu {snap['imu_frame']}")
        self._set_topic("laser_odom", now - snap["last_laser_odom"], "laser odom")
        self._set_topic("wheel_odom", now - snap["last_wheel_odom"], "wheel odom")
        self._set_topic("odom", now - snap["last_odom"], "odom")
        self._set_topic("map", now - snap["last_map"], "map")
        self._set_topic("amcl", now - snap["last_amcl"], "amcl")

        self.pose_label.setText(
            f"里程计: x={snap['x']:.3f}  y={snap['y']:.3f}  yaw={math.degrees(snap['yaw']):.1f} deg"
        )
        self.velocity_label.setText(
            f"速度: 线={snap['vx']:.3f} m/s  角={snap['wz']:.3f} rad/s"
        )
        self.wheel_odom_label.setText(
            f"编码器里程计: 线={snap['wheel_vx']:.3f} m/s  角={snap['wheel_wz']:.3f} rad/s"
        )
        gas_age = now - snap["last_gas"]
        gas = snap["gas"]
        gas_online = gas_age < 3.0
        if gas_online:
            self.gas_label.setText(
                "汇总: H2={H2:.2f} CO={CO:.2f} VOC={VOC:.2f} Smoke={Smoke:.2f}".format(**gas)
            )
            self.gas_status_label.setText("状态: 在线")
            self.gas_status_label.setStyleSheet("color: #237a57;")
            self.gas_h2_label.setText(f"H2: {gas['H2']:.2f}")
            self.gas_co_label.setText(f"CO: {gas['CO']:.2f}")
            self.gas_voc_label.setText(f"VOC: {gas['VOC']:.2f}")
            self.gas_smoke_label.setText(f"Smoke: {gas['Smoke']:.2f}")
            for label in [
                self.gas_h2_label,
                self.gas_co_label,
                self.gas_voc_label,
                self.gas_smoke_label,
            ]:
                label.setStyleSheet(
                    "background: #d9f0e4; color: #1b5e20; border-radius: 4px; padding: 4px;"
                )
        else:
            self.gas_label.setText("汇总: 等待 /gas_data")
            self.gas_status_label.setText("状态: 超时或未启动")
            self.gas_status_label.setStyleSheet("color: #b88728;")
            self.gas_h2_label.setText("H2: --")
            self.gas_co_label.setText("CO: --")
            self.gas_voc_label.setText("VOC: --")
            self.gas_smoke_label.setText("Smoke: --")
            for label in [
                self.gas_h2_label,
                self.gas_co_label,
                self.gas_voc_label,
                self.gas_smoke_label,
            ]:
                label.setStyleSheet(
                    "background: #f3ead4; color: #7c5c17; border-radius: 4px; padding: 4px;"
                )
        thermal_age = now - snap["last_thermal"]
        thermal_online = thermal_age < 3.0 and bool(snap.get("thermal_frame"))
        self.thermal_view.set_state(
            snap.get("thermal_frame", []),
            snap.get("thermal_width", 0),
            snap.get("thermal_height", 0),
            snap.get("thermal_min", 0.0),
            snap.get("thermal_max", 0.0),
            snap.get("thermal_avg", 0.0),
            snap.get("thermal_error", "") if thermal_age < 3.0 else "热成像超时或未启动",
        )
        if thermal_online:
            self.thermal_label.setText(
                f"热成像: {snap['thermal_width']}x{snap['thermal_height']} "
                f"min={snap['thermal_min']:.1f} max={snap['thermal_max']:.1f} "
                f"avg={snap['thermal_avg']:.1f}"
            )
            self.thermal_label.setStyleSheet("color: #237a57;")
        else:
            self.thermal_label.setText(
                f"热成像: {snap.get('thermal_error') or '等待 /thermal_frame'}"
            )
            self.thermal_label.setStyleSheet("color: #b88728;")
        if snap.get("thermal_change_ready"):
            self.thermal_change_label.setText(
                f"温度变化: {snap['thermal_change_per_min']:+.1f}°C/min"
            )
        else:
            self.thermal_change_label.setText("温度变化: 未达到 60 s 基线窗口")
        if self.thermal_detail_dialog is not None and self.thermal_detail_dialog.isVisible():
            self.thermal_detail_dialog.set_state(snap)
        self.safety_label.setText(
            f"安全: {snap['safety_level']} [{snap['safety_code']}] {snap['safety_message']}"
        )
        self.amcl_label.setText(
            f"AMCL: x={snap['amcl_x']:.3f}  y={snap['amcl_y']:.3f}  yaw={math.degrees(snap['amcl_yaw']):.1f} deg"
        )
        if snap["map_width"] > 0:
            self.map_label.setText(
                f"地图: {snap['map_width']} x {snap['map_height']} @ {snap['map_resolution']:.3f} m/cell"
            )
        else:
            self.map_label.setText("地图: 等待数据")
        self.map_view.set_state(
            snap.get("map_msg"),
            snap["x"],
            snap["y"],
            snap["yaw"],
            self.pose_history,
        )
        distance = snap["nav_distance_remaining"]
        if distance is None:
            self.nav_label.setText(f"导航: {snap['nav_status']}")
        else:
            self.nav_label.setText(f"导航: {snap['nav_status']}  剩余={distance:.2f} m")
        self._refresh_system_status(now)

    def _refresh_system_status(self, now):
        if now - self._last_system_status_update < 1.5:
            self.host_label.setText(self._system_status_cache["host"])
            self.gpu_label.setText(self._system_status_cache["gpu"])
            self.temp_label.setText(self._system_status_cache["temp"])
            return

        self._last_system_status_update = now
        self._system_status_cache["host"] = self._read_host_status()
        self._system_status_cache["gpu"] = self._read_gpu_status()
        self._system_status_cache["temp"] = self._read_temperature_status()
        self.host_label.setText(self._system_status_cache["host"])
        self.gpu_label.setText(self._system_status_cache["gpu"])
        self.temp_label.setText(self._system_status_cache["temp"])

    def _read_host_status(self):
        if psutil is None:
            return "主机: psutil 不可用"
        cpu = psutil.cpu_percent(interval=None)
        memory = psutil.virtual_memory()
        used_gb = memory.used / (1024 ** 3)
        total_gb = memory.total / (1024 ** 3)
        return (
            f"主机: CPU={cpu:.0f}%  内存={memory.percent:.0f}% "
            f"({used_gb:.1f}/{total_gb:.1f} GB)"
        )

    def _read_gpu_status(self):
        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu",
                    "--format=csv,noheader,nounits",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=0.8,
            )
        except (OSError, subprocess.TimeoutExpired):
            return "GPU: 不可用"
        if result.returncode != 0 or not result.stdout.strip():
            return "GPU: 不可用"
        fields = [field.strip() for field in result.stdout.splitlines()[0].split(",")]
        if len(fields) < 4:
            return "GPU: 不可用"
        util, mem_used, mem_total, temp = fields[:4]
        return f"GPU: {util}%  显存={mem_used}/{mem_total} MB  温度={temp} C"

    def _read_temperature_status(self):
        temperatures = []
        if psutil is not None and hasattr(psutil, "sensors_temperatures"):
            try:
                for entries in psutil.sensors_temperatures().values():
                    for entry in entries:
                        if entry.current is not None and entry.current > 0:
                            temperatures.append(float(entry.current))
            except Exception:
                temperatures = []
        if not temperatures:
            temperatures = self._read_temperatures_from_sensors()
        if not temperatures:
            return "温度: 不可用"
        return f"温度: 最高 {max(temperatures):.1f} C"

    def _read_temperatures_from_sensors(self):
        try:
            result = subprocess.run(
                ["sensors"],
                check=False,
                capture_output=True,
                text=True,
                timeout=0.8,
            )
        except (OSError, subprocess.TimeoutExpired):
            return []
        if result.returncode != 0:
            return []
        return [
            float(match.group(1))
            for match in re.finditer(r"\+([0-9]+(?:\.[0-9]+)?)\s*°?C", result.stdout)
        ]

    def _record_pose(self, x, y):
        point = (round(float(x), 2), round(float(y), 2))
        if self.pose_history and self.pose_history[-1] == point:
            return
        self.pose_history.append(point)
        if len(self.pose_history) > 160:
            self.pose_history.pop(0)

    def _set_topic(self, name, age, text):
        label = self.topic_labels[name]
        label.setText(text)
        if age < 1.5:
            label.setStyleSheet("background: #237a57; color: white; border-radius: 4px;")
        elif age < 5.0:
            label.setStyleSheet("background: #b88728; color: white; border-radius: 4px;")
        else:
            label.setStyleSheet("background: #7c8588; color: white; border-radius: 4px;")

    def _browse_workspace(self):
        path = QFileDialog.getExistingDirectory(self, "工作空间", self.workspace_edit.text())
        if path:
            self.workspace_edit.setText(path)

    def _browse_ros_setup(self):
        path, _ = QFileDialog.getOpenFileName(self, "ROS 环境脚本", self.ros_setup_edit.text())
        if path:
            self.ros_setup_edit.setText(path)

    def _browse_map(self):
        start = os.path.dirname(self._map_text()) or os.path.join(self.workspace_edit.text(), "maps")
        path, _ = QFileDialog.getOpenFileName(self, "地图 YAML", start, "YAML files (*.yaml *.yml)")
        if path:
            self._set_map_text(path)

    def refresh_maps(self):
        self._apply_launch_paths()
        current = self._map_text()
        maps_dir = os.path.join(self.workspace_edit.text().strip() or DEFAULT_WORKSPACE_PATH, "maps")
        maps = sorted(glob(os.path.join(maps_dir, "*.yaml")))
        self.map_combo.clear()
        for path in maps or [current or self._normalize_map_path("")]:
            self.map_combo.addItem(path)
        if current and current in maps:
            self._set_map_text(current)
        elif maps:
            self._set_map_text(maps[0])
        self.append_log(f"[MAP] {maps_dir} found {len(maps)} map yaml file(s)")

    def _map_text(self):
        return self.map_combo.currentText().strip()

    def _set_map_text(self, path):
        index = self.map_combo.findText(path)
        if index < 0:
            self.map_combo.addItem(path)
            index = self.map_combo.findText(path)
        self.map_combo.setCurrentIndex(index)

    def _normalize_map_path(self, path):
        maps_dir = os.path.join(self.workspace_edit.text().strip() or DEFAULT_WORKSPACE_PATH, "maps")
        if not path:
            return os.path.join(maps_dir, "inspection_map.yaml")
        if os.path.basename(path) == path:
            return os.path.join(maps_dir, path)
        return path

    def closeEvent(self, event):
        self.settings.sync()
        self.launch_manager.stop_thermal()
        if self.launch_manager.is_running():
            reply = QMessageBox.question(
                self,
                "停止启动项",
                "仍有 launch 进程在运行。是否停止后关闭？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes,
            )
            if reply != QMessageBox.Yes:
                event.ignore()
                return
        self.launch_manager.stop()
        self.ros.shutdown()
        event.accept()


def run_app(ros_adapter, signals):
    app = QApplication.instance() or QApplication([])
    window = MainWindow(ros_adapter, signals)
    window.show()
    return app.exec_()
