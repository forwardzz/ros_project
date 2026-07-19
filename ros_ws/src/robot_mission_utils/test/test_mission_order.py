from itertools import permutations
import math
from types import SimpleNamespace

from robot_mission_utils.grid_map import GridMap
import robot_mission_utils.inspection_planner as planner
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


def point(name, x, y, theta=0.0):
    return SimpleNamespace(point_name=name, x=x, y=y, theta=theta)


def region_options(start_x, end_x, y=2.5):
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


def test_exact_open_tsp_reorders_points_and_is_globally_minimal():
    map_msg = make_map(15, 15)
    points = [
        point("P1", 8.5, 1.5),
        point("P2", 2.5, 1.5),
        point("P3", 7.5, 1.5),
    ]

    result = plan_mission_order(map_msg, (1.5, 1.5), points, smooth=False)

    assert result is not None
    assert result.solving_method == "exact"
    assert result.ordered_indices == [1, 2, 0]
    grid = GridMap.from_occupancy_grid(map_msg)
    grid_points = [(1, 1)] + [grid.world_to_grid(p.x, p.y) for p in points]
    pairwise = compute_pairwise_paths(grid_points, grid)
    selected = [0] + [index + 1 for index in result.ordered_indices]
    selected_cost = route_objective(selected, pairwise.cost, grid_points)
    all_costs = [
        route_objective([0] + list(order), pairwise.cost, grid_points)
        for order in permutations(range(1, len(grid_points)))
    ]
    assert selected_cost == min(all_costs)


def test_tsp_uses_map_path_cost_instead_of_euclidean_distance():
    wall = [(2, y) for y in range(11)]
    result = plan_mission_order(
        make_map(15, 15, wall),
        (1.5, 1.5),
        [point("NEAR_BEHIND_WALL", 3.5, 1.5), point("OPEN", 1.5, 6.5)],
        smooth=False,
    )

    assert result is not None
    assert result.ordered_indices == [1, 0]


def test_more_than_ten_points_uses_nearest_neighbor_2opt_once_each():
    points = [point(f"P{i}", float(22 - i) + 0.5, 2.5) for i in range(11)]

    result = plan_mission_order(make_map(30, 10), (1.5, 2.5), points, smooth=False)

    assert result is not None
    assert result.solving_method == "nearest_neighbor_2opt"
    assert sorted(result.ordered_indices) == list(range(11))


def test_stale_planning_generation_can_cancel_before_search(monkeypatch):
    def unexpected_search(*_args, **_kwargs):
        raise AssertionError("stale planning request performed a path search")

    monkeypatch.setattr(planner, "_find_segment_path", unexpected_search)
    result = plan_mission_order(
        make_map(10, 10),
        (1.5, 1.5),
        [point("P1", 5.5, 5.5)],
        cancel_check=lambda: True,
    )

    assert result is None


def test_strict_real_robot_obstacle_margin_is_preserved():
    map_msg = make_map(10, 10, occupied=[(4, 5)])
    too_close = point("P1", 5.5, 5.5)

    validation = validate_mission_points(
        map_msg,
        (1.5, 1.5),
        [too_close],
        obstacle_margin_m=1.0,
        check_route=False,
    )

    assert validation.valid is False
    assert "too close to an obstacle" in validation.message


def test_start_distance_uses_tsp_first_point():
    map_msg = make_map(10, 10)
    points = [point("NEAR", 1.6, 1.5), point("FAR", 5.5, 1.5)]
    plan = MissionPlan([1, 0], [], 1.0, 1.0)

    validation = validate_mission_points(
        map_msg,
        (1.5, 1.5),
        points,
        optimize_order=True,
        mission_plan=plan,
    )

    assert validation.valid is True


def test_region_tsp_jointly_optimizes_order_and_sweep_direction():
    groups = [
        region_options(20.5, 22.5),
        region_options(4.5, 6.5),
        region_options(11.5, 13.5),
    ]

    result = plan_region_mission_order(
        make_map(30, 8), (1.5, 2.5, 0.0), groups, smooth=False
    )

    assert result is not None
    assert result.solving_method == "exact"
    assert result.ordered_indices == [1, 2, 0]
    assert result.option_indices == [0, 0, 0]


def test_region_tsp_return_home_can_reverse_final_sweep():
    map_msg = make_map(30, 8)
    groups = [
        region_options(20.5, 22.5),
        region_options(4.5, 6.5),
        region_options(11.5, 13.5),
    ]

    open_result = plan_region_mission_order(
        map_msg, (1.5, 2.5, 0.0), groups, smooth=False
    )
    return_result = plan_region_mission_order(
        map_msg,
        (1.5, 2.5, 0.0),
        groups,
        end_pose=(1.5, 2.5, 0.0),
        smooth=False,
    )

    assert open_result is not None and return_result is not None
    assert open_result.option_indices[-1] == 0
    assert return_result.option_indices[-1] == 1
    assert return_result.return_path


def test_more_than_ten_regions_uses_joint_nearest_neighbor_2opt_once_each():
    groups = [
        region_options(float(3 * i + 2) + 0.5, float(3 * i + 3) + 0.5)
        for i in reversed(range(11))
    ]

    result = plan_region_mission_order(
        make_map(50, 8), (1.5, 2.5, 0.0), groups, smooth=False
    )

    assert result is not None
    assert result.solving_method == "nearest_neighbor_2opt"
    assert sorted(result.ordered_indices) == list(range(11))
    assert len(result.option_indices) == 11


def test_unreachable_region_rejects_entire_tsp_route():
    wall = [(10, y) for y in range(8)]
    groups = [region_options(4.5, 6.5), region_options(20.5, 22.5)]

    result = plan_region_mission_order(
        make_map(30, 8, wall), (1.5, 2.5, 0.0), groups, smooth=False
    )

    assert result is None
