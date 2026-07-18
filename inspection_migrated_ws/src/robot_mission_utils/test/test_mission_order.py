from itertools import permutations
import math
from types import SimpleNamespace
import time

from robot_mission_utils.grid_map import GridMap
import robot_mission_utils.inspection_planner as inspection_planner
from robot_mission_utils.inspection_planner import (
    MissionPlan,
    RegionRouteOption,
    compute_pairwise_paths,
    plan_mission_order,
    plan_region_mission_order,
    route_objective,
    validate_mission_points,
)


def make_map(width, height, occupied=()):
    data = [0] * (width * height)
    for x, y in occupied:
        data[y * width + x] = 100
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        ),
        data=data,
    )


def make_point(name, x, y, theta=0.0):
    return SimpleNamespace(point_name=name, x=x, y=y, theta=theta)


def make_region_options(start_x, end_x, y=2.5):
    return [
        RegionRouteOption(
            entry_xy=(start_x, y),
            exit_xy=(end_x, y),
            entry_heading=0.0,
            exit_heading=0.0,
        ),
        RegionRouteOption(
            entry_xy=(end_x, y),
            exit_xy=(start_x, y),
            entry_heading=math.pi,
            exit_heading=math.pi,
        ),
    ]


def test_mission_plan_solving_method_has_compatible_default():
    plan = MissionPlan([], [], 0.0, 0.0)

    assert plan.solving_method == "exact"


def test_pairwise_grid_paths_search_each_undirected_pair_once(monkeypatch):
    calls = []

    def fake_find_path(start, end, _grid_map, weight=1.0):
        calls.append((start, end, weight))
        return [start, end]

    monkeypatch.setattr(inspection_planner, "_find_segment_path", fake_find_path)
    points = [(0, 0), (1, 0), (2, 0), (3, 0)]

    pairwise = compute_pairwise_paths(
        points, GridMap.from_occupancy_grid(make_map(5, 5))
    )

    assert len(calls) == 6
    assert pairwise.path[3][0] == [(3, 0), (0, 0)]
    assert pairwise.cost[0][3] == pairwise.cost[3][0]


def test_cancelled_mission_order_stops_before_path_search(monkeypatch):
    def unexpected_search(*_args, **_kwargs):
        raise AssertionError("stale planning request performed a path search")

    monkeypatch.setattr(inspection_planner, "_find_segment_path", unexpected_search)
    map_msg = make_map(10, 10)
    points = [make_point("RVIZ_1", 5.5, 5.5)]

    plan = plan_mission_order(
        map_msg,
        (1.5, 1.5),
        points,
        cancel_check=lambda: True,
    )

    assert plan is None


def test_exact_open_tsp_changes_click_order_and_is_globally_minimal():
    map_msg = make_map(15, 15)
    points = [
        make_point("RVIZ_1", 8.5, 1.5, 0.1),
        make_point("RVIZ_2", 2.5, 1.5, 0.2),
        make_point("RVIZ_3", 7.5, 1.5, 0.3),
    ]

    plan = plan_mission_order(map_msg, (1.5, 1.5), points, smooth=False)

    assert plan is not None
    assert plan.solving_method == "exact"
    assert plan.ordered_indices == [1, 2, 0]

    grid_map = GridMap.from_occupancy_grid(map_msg)
    grid_points = [(1, 1)] + [
        grid_map.world_to_grid(point.x, point.y) for point in points
    ]
    pairwise = compute_pairwise_paths(grid_points, grid_map)
    selected = [0] + [index + 1 for index in plan.ordered_indices]
    selected_cost = route_objective(selected, pairwise.cost, grid_points)
    possible_costs = [
        route_objective([0] + list(order), pairwise.cost, grid_points)
        for order in permutations(range(1, len(grid_points)))
    ]
    assert selected_cost == min(possible_costs)


def test_obstacle_path_cost_wins_over_euclidean_distance():
    # RVIZ_1 is geometrically close but separated by a wall whose opening is far away.
    wall = [(2, y) for y in range(11)]
    map_msg = make_map(15, 15, wall)
    points = [
        make_point("RVIZ_1", 3.5, 1.5),
        make_point("RVIZ_2", 1.5, 6.5),
    ]

    plan = plan_mission_order(map_msg, (1.5, 1.5), points, smooth=False)

    assert plan is not None
    assert plan.ordered_indices == [1, 0]


def test_unreachable_target_rejects_route_instead_of_using_click_order():
    wall = [(2, y) for y in range(15)]
    map_msg = make_map(15, 15, wall)
    points = [
        make_point("RVIZ_1", 3.5, 1.5),
        make_point("RVIZ_2", 1.5, 6.5),
    ]

    assert plan_mission_order(map_msg, (1.5, 1.5), points, smooth=False) is None


def test_more_than_ten_targets_uses_nearest_neighbor_2opt_once_each():
    map_msg = make_map(30, 10)
    points = [
        make_point(f"RVIZ_{index + 1}", float(22 - index) + 0.5, 2.5)
        for index in range(11)
    ]

    started = time.monotonic()
    plan = plan_mission_order(map_msg, (1.5, 2.5), points, smooth=False)
    elapsed = time.monotonic() - started

    assert plan is not None
    assert plan.solving_method == "nearest_neighbor_2opt"
    assert sorted(plan.ordered_indices) == list(range(11))
    assert len(set(plan.ordered_indices)) == 11
    assert elapsed < 5.0


def test_single_target_preserves_behavior_and_reports_single():
    map_msg = make_map(10, 10)
    point = make_point("RVIZ_1", 5.5, 5.5, theta=1.25)

    plan = plan_mission_order(map_msg, (1.5, 1.5), [point], smooth=False)

    assert plan is not None
    assert plan.ordered_indices == [0]
    assert plan.solving_method == "single"


def test_start_distance_uses_optimized_first_point_not_click_order():
    map_msg = make_map(10, 10)
    points = [
        make_point("RVIZ_NEAR", 1.6, 1.5),
        make_point("RVIZ_FIRST", 5.5, 1.5),
    ]
    far_first = MissionPlan(
        ordered_indices=[1, 0],
        preview_path=[],
        raw_cost=1.0,
        final_cost=1.0,
    )
    near_first = MissionPlan(
        ordered_indices=[0, 1],
        preview_path=[],
        raw_cost=1.0,
        final_cost=1.0,
    )

    accepted = validate_mission_points(
        map_msg,
        (1.5, 1.5),
        points,
        optimize_order=True,
        mission_plan=far_first,
    )
    rejected = validate_mission_points(
        map_msg,
        (1.5, 1.5),
        points,
        optimize_order=True,
        mission_plan=near_first,
    )

    assert accepted.valid is True
    assert rejected.valid is False
    assert "RVIZ_NEAR is too close" in rejected.message


def test_region_tsp_jointly_optimizes_order_and_coverage_direction():
    map_msg = make_map(30, 8)
    option_groups = [
        make_region_options(20.5, 22.5),
        make_region_options(4.5, 6.5),
        make_region_options(11.5, 13.5),
    ]

    plan = plan_region_mission_order(
        map_msg, (1.5, 2.5, 0.0), option_groups, smooth=False
    )

    assert plan is not None
    assert plan.solving_method == "exact"
    assert plan.ordered_indices == [1, 2, 0]
    assert plan.option_indices == [0, 0, 0]
    assert len(plan.transition_paths) == 3


def test_region_tsp_includes_fixed_home_and_can_reverse_final_region():
    map_msg = make_map(30, 8)
    option_groups = [
        make_region_options(20.5, 22.5),
        make_region_options(4.5, 6.5),
        make_region_options(11.5, 13.5),
    ]

    open_plan = plan_region_mission_order(
        map_msg, (1.5, 2.5, 0.0), option_groups, smooth=False
    )
    return_plan = plan_region_mission_order(
        map_msg,
        (1.5, 2.5, 0.0),
        option_groups,
        end_pose=(1.5, 2.5, 0.0),
        smooth=False,
    )

    assert open_plan is not None and return_plan is not None
    assert open_plan.option_indices[-1] == 0
    assert return_plan.option_indices[-1] == 1
    assert return_plan.return_path
    assert return_plan.raw_cost > open_plan.raw_cost


def test_more_than_ten_regions_uses_joint_nearest_neighbor_2opt():
    map_msg = make_map(50, 8)
    option_groups = [
        make_region_options(float(3 * index + 2) + 0.5, float(3 * index + 3) + 0.5)
        for index in reversed(range(11))
    ]

    plan = plan_region_mission_order(
        map_msg, (1.5, 2.5, 0.0), option_groups, smooth=False
    )

    assert plan is not None
    assert plan.solving_method == "nearest_neighbor_2opt"
    assert sorted(plan.ordered_indices) == list(range(11))
    assert len(set(plan.ordered_indices)) == 11
    assert len(plan.option_indices) == 11


def test_unreachable_region_rejects_entire_region_tsp():
    wall = [(10, y) for y in range(8)]
    map_msg = make_map(30, 8, wall)
    option_groups = [
        make_region_options(4.5, 6.5),
        make_region_options(20.5, 22.5),
    ]

    assert (
        plan_region_mission_order(
            map_msg, (1.5, 2.5, 0.0), option_groups, smooth=False
        )
        is None
    )
