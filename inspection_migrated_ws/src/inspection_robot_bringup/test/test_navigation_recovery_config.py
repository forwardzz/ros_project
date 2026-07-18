from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_progress_checker_is_explicitly_wired_in_active_behavior_trees():
    for filename in (
        "navigate_to_pose_no_spin.xml",
        "navigate_through_poses_no_spin.xml",
    ):
        root = ElementTree.parse(PACKAGE_ROOT / "config" / filename).getroot()
        selector = root.find(".//ProgressCheckerSelector")
        follow_path = root.find(".//FollowPath")

        assert selector is not None
        assert selector.attrib["default_progress_checker"] == "progress_checker"
        assert selector.attrib["selected_progress_checker"] == (
            "{selected_progress_checker}"
        )
        assert follow_path is not None
        assert follow_path.attrib["progress_checker_id"] == (
            "{selected_progress_checker}"
        )


def test_real_robot_progress_timeout_and_position_only_goal_policy():
    params_path = PACKAGE_ROOT / "config" / "nav2_params.yaml"
    with params_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    controller = config["controller_server"]["ros__parameters"]
    assert controller["progress_checker"]["movement_time_allowance"] == 30.0
    assert controller["FollowPath"]["rotate_to_goal_heading"] is False
    assert controller["general_goal_checker"]["plugin"] == (
        "nav2_controller::PositionGoalChecker"
    )


def test_real_robot_recovery_trees_do_not_spin_tracked_chassis():
    for filename in (
        "navigate_to_pose_no_spin.xml",
        "navigate_through_poses_no_spin.xml",
    ):
        root = ElementTree.parse(PACKAGE_ROOT / "config" / filename).getroot()
        assert root.find(".//Spin") is None
