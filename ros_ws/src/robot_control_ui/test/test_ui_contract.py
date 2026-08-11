from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
UI_SOURCE = (PACKAGE_ROOT / "robot_control_ui" / "robot_control_ui.py").read_text(
    encoding="utf-8"
)
ROS_ADAPTER_SOURCE = (
    PACKAGE_ROOT / "robot_control_ui" / "ros_adapter.py"
).read_text(encoding="utf-8")
LAUNCH_MANAGER_SOURCE = (
    PACKAGE_ROOT / "robot_control_ui" / "launch_manager.py"
).read_text(encoding="utf-8")
SETUP_SOURCE = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")


def _method_source(name, next_name):
    start = UI_SOURCE.index(f"    def {name}(")
    end = UI_SOURCE.index(f"    def {next_name}(", start)
    return UI_SOURCE[start:end]


def _class_source(name, next_name):
    start = UI_SOURCE.index(f"class {name}(")
    end = UI_SOURCE.index(f"class {next_name}(", start)
    return UI_SOURCE[start:end]


def test_qt_is_the_only_ui_module_and_entrypoint():
    assert "tkinter" not in UI_SOURCE.lower()
    assert not (PACKAGE_ROOT / "robot_control_ui" / "qt_ui.py").exists()
    assert '"robot_control_ui = robot_control_ui.robot_control_ui:main"' in SETUP_SOURCE
    assert "robot_control_ui_qt" not in SETUP_SOURCE
    launch = (PACKAGE_ROOT / "launch" / "ui.launch.py").read_text(encoding="utf-8")
    assert 'executable="robot_control_ui"' in launch
    assert '"ui_executable"' not in launch


def test_ui_publishes_directly_to_cmd_vel():
    assert 'create_publisher(Twist, "/cmd_vel", 10)' in ROS_ADAPTER_SOURCE
    assert "/cmd_vel_teleop" not in ROS_ADAPTER_SOURCE
    manual = _class_source("ManualDrivePanel", "LanScanDialog")
    assert "self._timer.timeout.connect(self._publish)" in manual
    assert "self.stop_robot()" in _method_source("_begin", "_on_abort_finished")


def test_ui_has_amcl_initial_pose_round_trip():
    assert 'PoseWithCovarianceStamped, "/initialpose", 10' in ROS_ADAPTER_SOURCE
    assert 'PoseWithCovarianceStamped, "/amcl_pose"' in ROS_ADAPTER_SOURCE
    assert 'QPushButton("设置初始位姿")' in UI_SOURCE
    assert "self.adapter.publish_initial_pose" in UI_SOURCE
    assert "def publish_initial_pose" in ROS_ADAPTER_SOURCE


def test_rviz_mission_controls_follow_qt_task_order():
    mission = _class_source("MissionPanel", "RuntimeLogPanel")
    ordered_labels = [
        "TSP 路径顺序优化",
        "任务完成后返回起点",
        "点位停留时间（秒）",
        "开始任务",
        "停止全部任务",
        "清除 RViz 点位",
        "区域巡检模式",
        "保存区域",
        "加载区域",
        "清除区域",
        "撤销区域",
        "撤销点位",
    ]
    offsets = [mission.index(label) for label in ordered_labels]
    assert offsets == sorted(offsets)


def test_ui_exposes_tsp_and_task_start_return_controls():
    mission = _class_source("MissionPanel", "RuntimeLogPanel")
    start = _method_source("_start_mission", "_stop_tasks")
    assert 'QCheckBox("TSP 路径顺序优化")' in mission
    assert 'QCheckBox("任务完成后返回起点")' in mission
    assert "self.tsp_check.setChecked(True)" in mission
    assert "self.return_check.setChecked(False)" in mission
    assert "request.use_tsp = bool(self.tsp_check.isChecked())" in start
    assert "request.return_to_start = bool(self.return_check.isChecked())" in start
    assert 'SetBool, "/set_tsp_mode"' in ROS_ADAPTER_SOURCE


def test_remote_cleanup_has_no_process_wide_ros2_pattern():
    assert '"ros2",' not in LAUNCH_MANAGER_SOURCE
    assert "mapping.launch.py" in LAUNCH_MANAGER_SOURCE
    assert "navigation.launch.py" in LAUNCH_MANAGER_SOURCE


# ---------------------------------------------------------------------------
# Network / status-monitoring contract (network & status rework)
# ---------------------------------------------------------------------------

def test_default_robot_address_is_192_168_43_31():
    assert '"remote_host", "192.168.43.31"' in UI_SOURCE
    launch = (PACKAGE_ROOT / "launch" / "ui.launch.py").read_text(encoding="utf-8")
    assert 'default_value="192.168.43.31"' in launch
    start_script = (PACKAGE_ROOT.parents[2] / "start.sh").read_text(encoding="utf-8")
    assert 'REMOTE_HOST="${REMOTE_HOST:-192.168.43.31}"' in start_script


def test_status_panel_has_scan_and_apply_controls():
    panel = _class_source("MissionControlPanel", "LocalizationPanel")
    for label in ("扫描局域网", "刷新", "应用地址"):
        assert 'QPushButton("%s")' % label in panel


def test_network_discovery_module_present():
    module = (PACKAGE_ROOT / "robot_control_ui" / "logic" / "network_discovery.py").read_text(
        encoding="utf-8"
    )
    assert "def is_valid_ipv4" in module
    assert "def scan_subnet" in module
    assert "MAX_HOSTS" in module


def test_remote_health_module_present():
    module = (PACKAGE_ROOT / "robot_control_ui" / "logic" / "remote_health.py").read_text(
        encoding="utf-8"
    )
    assert "class RemoteHealthProbe" in module
    assert "def parse_throttled" in module
    assert "BatchMode=yes" in module


def test_topic_health_module_present():
    module = (PACKAGE_ROOT / "robot_control_ui" / "logic" / "topic_health.py").read_text(
        encoding="utf-8"
    )
    assert "class TopicHealthTracker" in module
    assert "def classify" in module
    assert "available" in module


def test_ssh_indicator_does_not_trust_unprobed_state():
    refresh = _method_source("update_health", "update_topics")
    # The old bug trusted last_exit_code None as healthy; that path is gone.
    assert "last_exit_code in (None, 0)" not in refresh
    assert "health.online" in refresh
    assert "error_code" in refresh


# ---------------------------------------------------------------------------
# DDS environment & topic-tracker contract (WSL-PI communication fix)
# ---------------------------------------------------------------------------

def test_remote_command_sets_dds_environment():
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in LAUNCH_MANAGER_SOURCE
    assert "CYCLONEDDS_URI=" in LAUNCH_MANAGER_SOURCE
    assert "ROS_DOMAIN_ID=0" in LAUNCH_MANAGER_SOURCE
    assert "ROS_LOCALHOST_ONLY=0" in LAUNCH_MANAGER_SOURCE
    assert "shlex.quote(self.ros_setup_path)" in LAUNCH_MANAGER_SOURCE
    assert "shlex.quote(self.workspace_path)" in LAUNCH_MANAGER_SOURCE


def test_launch_files_do_not_override_dds_uri():
    for name in ("mapping", "navigation", "sensor_monitor"):
        path = (
            PACKAGE_ROOT.parents[0]
            / "mapping_bringup"
            / "launch"
            / (name + ".launch.py")
        )
        launch_text = path.read_text(encoding="utf-8")
        assert "SetEnvironmentVariable" not in launch_text
        assert "CYCLONEDDS_URI" not in launch_text


def test_ui_reuses_adapter_topic_trackers():
    assert "self.adapter.topic_trackers.items()" in UI_SOURCE
    # RosUiAdapter init creates trackers before any subscription
    assert "self.topic_trackers = {" in ROS_ADAPTER_SOURCE
    assert "TopicHealthTracker" in ROS_ADAPTER_SOURCE
