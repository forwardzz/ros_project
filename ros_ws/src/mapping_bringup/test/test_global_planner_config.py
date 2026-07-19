from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


def load_nav2_params():
    return yaml.safe_load((CONFIG_DIR / "nav2_params.yaml").read_text(encoding="utf-8"))


def test_global_costmap_uses_live_laser_obstacles_and_real_robot_geometry():
    params = load_nav2_params()["global_costmap"]["global_costmap"]["ros__parameters"]

    assert params["resolution"] == 0.03
    assert params["update_frequency"] == 5.0
    assert params["footprint"] == (
        "[[0.135, 0.11], [0.135, -0.11], [-0.135, -0.11], [-0.135, 0.11]]"
    )
    assert params["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]
    laser = params["obstacle_layer"]["laser"]
    assert laser["topic"] == "/scan"
    assert laser["marking"] is True
    assert laser["clearing"] is True
    assert laser["obstacle_max_range"] == 3.0
    assert laser["raytrace_max_range"] == 3.0
    inflation = params["inflation_layer"]
    assert inflation["inflation_radius"] == 0.18
    assert inflation["cost_scaling_factor"] == 4.0


def test_smac_planner_matches_selected_hardware_profile():
    nav2 = load_nav2_params()
    planner = nav2["planner_server"]["ros__parameters"]
    grid = planner["GridBased"]

    assert planner["expected_planner_frequency"] == 5.0
    assert grid["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert grid["allow_unknown"] is False
    assert grid["downsample_costmap"] is False
    assert grid["max_planning_time"] == 2.0
    assert grid["cost_travel_multiplier"] == 1.5
    assert grid["GridBased.smoother.do_refinement"] is True
    assert "NavfnPlanner" not in (CONFIG_DIR / "nav2_params.yaml").read_text()

    controller = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    assert controller["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert nav2["behavior_server"]["ros__parameters"]["behavior_plugins"] == [
        "backup",
        "drive_on_heading",
        "assisted_teleop",
        "wait",
    ]


def test_navigation_trees_replan_only_for_changed_or_invalid_paths_without_spin():
    for filename, compute_tag in (
        ("navigate_to_pose_no_spin.xml", "ComputePathToPose"),
        ("navigate_through_poses_no_spin.xml", "ComputePathThroughPoses"),
    ):
        path = CONFIG_DIR / filename
        root = ET.parse(path).getroot()
        tags = [element.tag for element in root.iter()]

        assert tags.count(compute_tag) == 1
        assert "GlobalUpdatedGoal" in tags
        assert "IsPathValid" in tags
        assert "GoalCheckerSelector" in tags
        assert "ProgressCheckerSelector" in tags
        assert "PlannerSelector" in tags
        assert "Spin" not in tags

        rate = next(element for element in root.iter("RateController"))
        follow = next(element for element in root.iter("FollowPath"))
        wait = next(element for element in root.iter("Wait"))
        backup = next(element for element in root.iter("BackUp"))
        assert rate.attrib["hz"] == "1.0"
        assert follow.attrib["goal_checker_id"] == "{selected_goal_checker}"
        assert follow.attrib["progress_checker_id"] == "{selected_progress_checker}"
        assert wait.attrib["wait_duration"] == "3.0"
        assert backup.attrib["backup_dist"] == "0.20"
        assert backup.attrib["backup_speed"] == "0.08"


def test_smac_dependency_and_behavior_trees_are_installed_by_package():
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    setup_py = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    launch = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "<depend>nav2_smac_planner</depend>" in package_xml
    assert 'glob("config/*.xml")' in setup_py
    assert '"config", "navigate_to_pose_no_spin.xml"' in launch
    assert '"config", "navigate_through_poses_no_spin.xml"' in launch
