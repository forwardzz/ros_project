from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
UI_SOURCE = (PACKAGE_ROOT / "robot_control_ui" / "robot_control_ui.py").read_text(
    encoding="utf-8"
)


def _method_source(name, next_name):
    start = UI_SOURCE.index(f"    def {name}(")
    end = UI_SOURCE.index(f"    def {next_name}(", start)
    return UI_SOURCE[start:end]


def test_ui_uses_project_velocity_safety_chain():
    assert 'create_publisher(Twist, "/cmd_vel_teleop", 10)' in UI_SOURCE
    assert 'create_publisher(Twist, "/cmd_vel",' not in UI_SOURCE
    assert "self.root.after(100, self._publish_manual_cmd)" in UI_SOURCE
    assert "self.stop_robot()" in _method_source("_begin_manual_cmd", "_publish_manual_cmd")


def test_ui_has_amcl_initial_pose_round_trip():
    assert 'PoseWithCovarianceStamped, "/initialpose", 10' in UI_SOURCE
    assert 'PoseWithCovarianceStamped, "/amcl_pose"' in UI_SOURCE
    assert 'text="Set Initial Pose"' in UI_SOURCE
    assert "InitialPoseRetryState(max_attempts=8)" in UI_SOURCE


def test_rviz_mission_controls_follow_qt_task_order():
    mission = _method_source("_build_mission_panel", "_build_map_panel")
    ordered_labels = [
        "Check Localization",
        "Start Mission",
        "Stop All Tasks",
        "Clear RViz Points",
        "Point pause (s)",
        "Region Mode",
        "Save Regions",
        "Load Regions",
        "Clear Regions",
        "Undo Region",
        "Undo Point",
    ]
    offsets = [mission.index(label) for label in ordered_labels]
    assert offsets == sorted(offsets)


def test_ui_exposes_tsp_and_task_start_return_controls():
    mission = _method_source("_build_mission_panel", "_build_map_panel")
    start = _method_source("start_mission", "reset_safety")
    assert 'text="TSP"' in mission
    assert 'text="Return to Start"' in mission
    assert "self.tsp_mode_var = tk.BooleanVar(value=True)" in UI_SOURCE
    assert "self.return_to_start_var = tk.BooleanVar(value=False)" in UI_SOURCE
    assert "request.use_tsp = bool(self.tsp_mode_var.get())" in start
    assert "request.return_to_start = bool(self.return_to_start_var.get())" in start
    assert 'SetBool, "/set_tsp_mode"' in UI_SOURCE


def test_remote_cleanup_has_no_process_wide_ros2_pattern():
    cleanup = _method_source("_stop_patterns_for", "_build_remote_command")
    assert '"ros2",' not in cleanup
    assert "mapping.launch.py" in cleanup
    assert "navigation.launch.py" in cleanup


# ---------------------------------------------------------------------------
# Network / status-monitoring contract (network & status rework)
# ---------------------------------------------------------------------------

def test_default_robot_address_is_192_168_43_30():
    assert '"remote_host", "192.168.43.30"' in UI_SOURCE
    launch = (PACKAGE_ROOT / "launch" / "ui.launch.py").read_text(encoding="utf-8")
    assert 'default_value="192.168.43.30"' in launch
    start_script = (PACKAGE_ROOT.parents[2] / "start.sh").read_text(encoding="utf-8")
    assert 'REMOTE_HOST="${REMOTE_HOST:-__AUTO__}"' in start_script


def test_status_panel_has_scan_and_apply_controls():
    panel = _method_source("_build_launch_panel", "_build_pose_panel")
    for label in ("Scan LAN", "Refresh", "Apply IP"):
        assert 'text="%s"' % label in panel


def test_network_discovery_module_present():
    module = (PACKAGE_ROOT / "robot_control_ui" / "network_discovery.py").read_text(
        encoding="utf-8"
    )
    assert "def is_valid_ipv4" in module
    assert "def scan_subnet" in module
    assert "MAX_HOSTS" in module


def test_remote_health_module_present():
    module = (PACKAGE_ROOT / "robot_control_ui" / "remote_health.py").read_text(
        encoding="utf-8"
    )
    assert "class RemoteHealthProbe" in module
    assert "def parse_throttled" in module
    assert "BatchMode=yes" in module


def test_topic_health_module_present():
    module = (PACKAGE_ROOT / "robot_control_ui" / "topic_health.py").read_text(
        encoding="utf-8"
    )
    assert "class TopicHealthTracker" in module
    assert "def classify" in module
    assert "available" in module


def test_ssh_indicator_does_not_trust_unprobed_state():
    refresh = _method_source("_refresh_status", "_set_indicator")
    # The old bug trusted last_exit_code None as healthy; that path is gone.
    assert "last_exit_code in (None, 0)" not in refresh
    assert "current_health" in refresh
    assert "error_code" in refresh


# ---------------------------------------------------------------------------
# DDS environment & topic-tracker contract (WSL-PI communication fix)
# ---------------------------------------------------------------------------

def test_remote_command_sets_dds_environment():
    method = _method_source("_build_remote_command", "_build_ssh_invocation")
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in method
    assert "CYCLONEDDS_URI=" in method
    assert "ROS_DOMAIN_ID=0" in method
    assert "ROS_LOCALHOST_ONLY=0" in method
    assert "shlex.quote(self.ros_setup_path)" in method
    assert "shlex.quote(self.workspace_path)" in method


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
    assert "self.topic_trackers = self.ros.topic_trackers" in UI_SOURCE
    # RosUiAdapter init (first class) creates trackers before any subscription
    ros_init = UI_SOURCE[UI_SOURCE.index("class RosUiAdapter:"):UI_SOURCE.index("class LaunchManager:")]
    assert "self.topic_trackers = {" in ros_init
