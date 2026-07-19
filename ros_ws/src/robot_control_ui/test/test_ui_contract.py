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
