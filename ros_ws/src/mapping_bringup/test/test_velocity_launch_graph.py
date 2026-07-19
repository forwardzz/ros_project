from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]


def test_navigation_velocity_chain_and_single_final_publisher():
    launch = (WORKSPACE_SRC / "mapping_bringup/launch/navigation.launch.py").read_text()
    gate = (WORKSPACE_SRC / "mapping_bringup/mapping_bringup/velocity_safety_gate.py").read_text()
    mission = (WORKSPACE_SRC / "mapping_bringup/mapping_bringup/mission_manager.py").read_text()
    assert launch.count('(\"/cmd_vel\", \"/cmd_vel_nav\")') == 3
    assert '(\"/cmd_vel_smoothed\", \"/cmd_vel_auto\")' in launch
    assert 'create_publisher(Twist, "/cmd_vel", 10)' in gate
    assert 'create_publisher(Twist, TOPIC_CMD_VEL_NAV, 10)' in mission


def test_mapping_starts_monitor_and_gate():
    launch = (WORKSPACE_SRC / "mapping_bringup/launch/mapping.launch.py").read_text()
    assert 'executable="safety_monitor"' in launch
    assert 'executable="velocity_safety_gate"' in launch


def test_gui_only_publishes_manual_velocity_to_teleop_topic():
    gui = (WORKSPACE_SRC / "robot_control_ui/robot_control_ui/robot_control_ui.py").read_text()
    assert 'create_publisher(Twist, "/cmd_vel_teleop", 10)' in gui
    assert 'create_publisher(Twist, "/cmd_vel", 10)' not in gui
