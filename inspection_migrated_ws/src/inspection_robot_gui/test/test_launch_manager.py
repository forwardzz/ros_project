import re
import shlex

from inspection_robot_gui.launch_manager import LaunchManager, ROBOT_PROCESS_PATTERNS


def _capture_start(manager):
    calls = []

    def fake_start(name, command):
        calls.append((name, command))
        return True

    manager.start = fake_start
    return calls


def test_mapping_defaults_keep_motors_disabled_and_use_calibrated_polarity():
    manager = LaunchManager()
    calls = _capture_start(manager)

    assert manager.start_mapping()

    assert calls == [(
        "mapping",
        "ros2 launch inspection_robot_bringup mapping.launch.py "
        "motor_pair:=disabled left_inverted:=false right_inverted:=false "
        "actuation_enabled:=false",
    )]


def test_navigation_forwards_explicit_cd_motor_options():
    manager = LaunchManager()
    calls = _capture_start(manager)

    assert manager.start_navigation(
        "/home/yy/inspection_migrated_ws/maps/inspection map.yaml",
        motor_pair="cd",
        left_inverted=False,
        right_inverted=False,
        actuation_enabled=True,
    )

    assert calls == [(
        "navigation",
        "ros2 launch inspection_robot_bringup navigation.launch.py "
        "map:='/home/yy/inspection_migrated_ws/maps/inspection map.yaml' "
        "regions:=/home/yy/inspection_migrated_ws/maps/inspection_regions.yaml "
        "mission_home_x:=0 mission_home_y:=0 mission_home_yaw:=0 "
        "motor_pair:=cd left_inverted:=false right_inverted:=false "
        "actuation_enabled:=true",
    )]


def test_navigation_forwards_regions_and_fixed_mission_home():
    manager = LaunchManager()
    calls = _capture_start(manager)

    assert manager.start_navigation(
        "/tmp/map.yaml",
        regions_path="/tmp/inspection regions.yaml",
        mission_home_x=1.25,
        mission_home_y=-0.5,
        mission_home_yaw=1.57079632679,
    )

    assert calls == [(
        "navigation",
        "ros2 launch inspection_robot_bringup navigation.launch.py "
        "map:=/tmp/map.yaml regions:='/tmp/inspection regions.yaml' "
        "mission_home_x:=1.25 mission_home_y:=-0.5 mission_home_yaw:=1.57079633 "
        "motor_pair:=disabled left_inverted:=false right_inverted:=false "
        "actuation_enabled:=false",
    )]


def test_stop_cleans_every_remote_bringup_child():
    manager = LaunchManager()
    cleanup_calls = []
    manager._remote_cleanup = lambda patterns: cleanup_calls.append(patterns)

    manager.stop()

    assert cleanup_calls == [ROBOT_PROCESS_PATTERNS]
    for required_pattern in (
        "static_transform_publisher",
        "ybimu_driver",
        "ekf_node",
        "mission_manager",
        "lifecycle_manager",
    ):
        assert required_pattern in cleanup_calls[0]


def test_ssh_wraps_the_complete_login_shell_command():
    manager = LaunchManager(
        workspace_path="/home/robot/ws with spaces",
        ros_setup_path="/opt/ros/jazzy/setup.bash",
        remote_user="robot",
        remote_host="192.0.2.10",
    )
    command = "ros2 node list"

    args = manager._ssh_args(command)

    expected_remote_shell = "bash -lc %s" % shlex.quote(
        manager._remote_command(command)
    )
    assert args == ["ssh", "robot@192.0.2.10", expected_remote_shell]


def test_remote_uses_same_rmw_as_gui_environment(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    manager = LaunchManager()

    command = manager._remote_command("ros2 node list")

    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in command
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" in command
    assert "unset ROS_LOCALHOST_ONLY" in command


def test_cleanup_pattern_does_not_match_its_own_command_line():
    pattern = LaunchManager._self_safe_pkill_pattern("mapping.launch.py")

    assert pattern == r"[m]apping\.launch\.py"
    assert re.search(pattern, "ros2 launch mapping.launch.py")
    assert not re.search(pattern, "pkill -f '[m]apping\\.launch\\.py'")


def test_list_maps_queries_the_remote_workspace(monkeypatch):
    manager = LaunchManager(
        workspace_path="/home/robot/ws with spaces",
        remote_user="robot",
        remote_host="192.0.2.10",
    )
    calls = []

    class Result:
        returncode = 0
        stdout = (
            "/home/robot/ws with spaces/maps/z.yml\n"
            "/home/robot/ws with spaces/maps/a.yaml\n"
        )
        stderr = ""

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        return Result()

    monkeypatch.setattr(
        "inspection_robot_gui.launch_manager.subprocess.run", fake_run
    )

    assert manager.list_maps() == [
        "/home/robot/ws with spaces/maps/a.yaml",
        "/home/robot/ws with spaces/maps/z.yml",
    ]
    assert len(calls) == 1
    expected_command = (
        "find '/home/robot/ws with spaces/maps' -maxdepth 1 -type f "
        "\\( -name '*.yaml' -o -name '*.yml' \\) -print"
    )
    assert calls[0][0] == manager._ssh_args(expected_command)
