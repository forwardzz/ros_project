from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]


def test_navigation_velocity_smoother_publishes_directly_to_cmd_vel():
    launch = (WORKSPACE_SRC / "mapping_bringup/launch/navigation.launch.py").read_text()
    mission = (WORKSPACE_SRC / "mapping_bringup/mapping_bringup/mission_manager.py").read_text()
    assert launch.count('(\"/cmd_vel\", \"/cmd_vel_nav\")') == 3
    assert '(\"/cmd_vel_smoothed\", \"/cmd_vel\")' in launch
    assert 'create_publisher(Twist, TOPIC_CMD_VEL_NAV, 10)' in mission


def test_launches_do_not_start_removed_safety_nodes():
    mapping = (WORKSPACE_SRC / "mapping_bringup/launch/mapping.launch.py").read_text()
    navigation = (WORKSPACE_SRC / "mapping_bringup/launch/navigation.launch.py").read_text()
    for launch in (mapping, navigation):
        assert 'executable="safety_monitor"' not in launch
        assert 'executable="velocity_safety_gate"' not in launch


def test_gui_publishes_manual_velocity_directly_to_cmd_vel():
    adapter = (WORKSPACE_SRC / "robot_control_ui/robot_control_ui/ros_adapter.py").read_text()
    assert 'create_publisher(Twist, "/cmd_vel", 10)' in adapter
    assert "/cmd_vel_teleop" not in adapter
