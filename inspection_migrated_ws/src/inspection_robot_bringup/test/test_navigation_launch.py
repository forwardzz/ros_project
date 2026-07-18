import importlib.util
import math
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


NAV_COMMAND_REMAP = ("/cmd_vel", "/cmd_vel_nav")
SMOOTHER_OUTPUT_REMAP = ("/cmd_vel_smoothed", "/cmd_vel_auto")


def _load_navigation_launch():
    launch_path = Path(__file__).parents[1] / "launch" / "navigation.launch.py"
    spec = importlib.util.spec_from_file_location("navigation_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _load_navigation_params():
    params_path = Path(__file__).parents[1] / "config" / "nav2_params.yaml"
    with params_path.open(encoding="utf-8") as params_file:
        return yaml.safe_load(params_file)


def _node_remappings(node, context):
    return {
        (
            perform_substitutions(context, source),
            perform_substitutions(context, target),
        )
        for source, target in node._Node__remappings
    }


def _nodes_by_name(launch_description):
    return {
        node._Node__node_name: node
        for node in launch_description.entities
        if isinstance(node, Node)
    }


def test_navigation_commands_are_routed_through_safety_chain():
    nodes = _nodes_by_name(_load_navigation_launch())
    context = LaunchContext()

    assert NAV_COMMAND_REMAP in _node_remappings(
        nodes["controller_server"], context
    )
    assert NAV_COMMAND_REMAP in _node_remappings(
        nodes["behavior_server"], context
    )
    assert _node_remappings(nodes["velocity_smoother"], context) == {
        NAV_COMMAND_REMAP,
        SMOOTHER_OUTPUT_REMAP,
    }


def test_costmap_clearance_encloses_robot_footprint():
    params = _load_navigation_params()
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
    local_footprint = yaml.safe_load(local["footprint"])
    circumscribed_radius = max(math.hypot(x, y) for x, y in local_footprint)

    assert global_costmap["robot_radius"] >= circumscribed_radius
    assert local["inflation_layer"]["inflation_radius"] >= circumscribed_radius
    assert (
        global_costmap["inflation_layer"]["inflation_radius"]
        >= global_costmap["robot_radius"]
    )
