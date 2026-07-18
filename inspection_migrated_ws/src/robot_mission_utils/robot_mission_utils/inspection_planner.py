from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .bidirectional_astar import bidirectional_astar
from .grid_map import GridMap
from .lazy_theta_star import lazy_theta_star
from .map_inflation import resolve_planning_map
from .path_collision import bresenham_cells, validate_path_segments
from .path_smoothing import smooth_path

Point = Tuple[int, int]
STRAIGHT_COST = 1.0
DIAGONAL_COST = 1.7


class PlanningCancelled(Exception):
    """Internal signal used to stop a stale background planning request."""


@dataclass(frozen=True)
class PairwisePaths:
    cost: List[List[float]]
    path: List[List[Optional[List[Point]]]]


@dataclass(frozen=True)
class InspectionPlanResult:
    visiting_order: List[Point]
    raw_path: List[Point]
    final_path: List[Point]
    raw_cost: float
    final_cost: float
    inflation_cells: int = 0


@dataclass(frozen=True)
class MissionPlan:
    ordered_indices: List[int]
    preview_path: List[Tuple[float, float]]
    raw_cost: float
    final_cost: float
    inflation_cells: int = 0
    solving_method: str = "exact"


@dataclass(frozen=True)
class RegionRouteOption:
    entry_xy: Tuple[float, float]
    exit_xy: Tuple[float, float]
    entry_heading: float
    exit_heading: float


@dataclass(frozen=True)
class RegionMissionPlan:
    ordered_indices: List[int]
    option_indices: List[int]
    transition_paths: List[List[Tuple[float, float]]]
    return_path: List[Tuple[float, float]]
    raw_cost: float
    inflation_cells: int = 0
    solving_method: str = "exact"


@dataclass(frozen=True)
class MissionValidation:
    valid: bool
    message: str


def path_cost(path: Sequence[Point]) -> float:
    if len(path) <= 1:
        return 0.0
    total = 0.0
    for start, end in zip(path, path[1:]):
        cells = bresenham_cells(start, end)
        for cell_a, cell_b in zip(cells, cells[1:]):
            dx = abs(cell_b[0] - cell_a[0])
            dy = abs(cell_b[1] - cell_a[1])
            total += DIAGONAL_COST if dx == 1 and dy == 1 else STRAIGHT_COST
    return total


def _find_segment_path(a: Point, b: Point, grid_map, weight: float = 1.0) -> Optional[List[Point]]:
    segment = lazy_theta_star(a, b, grid_map, weight=weight)
    if segment:
        return segment
    return bidirectional_astar(a, b, grid_map, weight=weight)


def _direction_safe_reverse(path: List[Point], grid_map) -> List[Point]:
    reversed_path = list(reversed(path))
    if validate_path_segments(reversed_path, grid_map):
        return reversed_path

    # Bresenham rasterization can select different cells by direction for long
    # sparse segments. Expand the known-valid forward raster before reversing.
    dense_path: List[Point] = []
    for start, end in zip(path, path[1:]):
        cells = bresenham_cells(start, end)
        if not dense_path:
            dense_path.extend(cells)
        else:
            dense_path.extend(cells[1:])
    return list(reversed(dense_path)) if dense_path else reversed_path


def compute_pairwise_paths(
    points: List[Point],
    grid_map,
    weight: float = 1.0,
    cancel_check: Optional[Callable[[], bool]] = None,
) -> PairwisePaths:
    count = len(points)
    cost = [[float('inf')] * count for _ in range(count)]
    path = [[None] * count for _ in range(count)]

    for i in range(count):
        cost[i][i] = 0.0
        path[i][i] = [points[i]]

    # Grid traversal is undirected, so one search can populate both directions.
    for i in range(count):
        for j in range(i + 1, count):
            if cancel_check is not None and cancel_check():
                raise PlanningCancelled()
            segment = _find_segment_path(points[i], points[j], grid_map, weight=weight)
            if segment:
                path[i][j] = segment
                path[j][i] = _direction_safe_reverse(segment, grid_map)
                cost[i][j] = path_cost(segment)
                cost[j][i] = cost[i][j]
    return PairwisePaths(cost=cost, path=path)


def _pairwise_has_hamiltonian_edges(pairwise: PairwisePaths, count: int) -> bool:
    if count <= 2:
        return pairwise.cost[0][count - 1] != float('inf')
    for index in range(1, count - 1):
        if pairwise.cost[0][index] != float('inf'):
            return True
    return False


def turn_penalty(prev_point: Point, cur_point: Point, next_point: Point, turn_weight: float) -> float:
    v1x = cur_point[0] - prev_point[0]
    v1y = cur_point[1] - prev_point[1]
    v2x = next_point[0] - cur_point[0]
    v2y = next_point[1] - cur_point[1]
    n1 = math.hypot(v1x, v1y)
    n2 = math.hypot(v2x, v2y)
    if n1 == 0 or n2 == 0:
        return 0.0
    cos_theta = (v1x * v2x + v1y * v2y) / (n1 * n2)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return turn_weight * (1.0 - cos_theta)


def tsp_fixed_start_end(cost: List[List[float]], points: List[Point], start_idx: int, end_idx: int, turn_weight: float = 0.525) -> Optional[List[int]]:
    count = len(cost)
    if count == 0:
        return []
    if start_idx == end_idx:
        return [start_idx] if count == 1 else None

    intermediates = [index for index in range(count) if index not in (start_idx, end_idx)]
    middle_count = len(intermediates)
    if middle_count == 0:
        return [start_idx, end_idx] if cost[start_idx][end_idx] != float('inf') else None

    middle_index = {node: bit for bit, node in enumerate(intermediates)}
    mask_size = 1 << middle_count
    dp = [[[float('inf')] * count for _ in range(count)] for _ in range(mask_size)]
    parent: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {}

    for first in intermediates:
        edge = cost[start_idx][first]
        if edge == float('inf'):
            continue
        bit = 1 << middle_index[first]
        dp[bit][start_idx][first] = edge
        parent[(bit, start_idx, first)] = None

    for mask in range(mask_size):
        for prev in range(count):
            for last in range(count):
                current_cost = dp[mask][prev][last]
                if current_cost == float('inf'):
                    continue
                for nxt in intermediates:
                    bit = 1 << middle_index[nxt]
                    if mask & bit:
                        continue
                    edge = cost[last][nxt]
                    if edge == float('inf'):
                        continue
                    new_mask = mask | bit
                    penalty = turn_penalty(points[prev], points[last], points[nxt], turn_weight)
                    new_cost = current_cost + edge + penalty
                    if new_cost < dp[new_mask][last][nxt]:
                        dp[new_mask][last][nxt] = new_cost
                        parent[(new_mask, last, nxt)] = (mask, prev, last)

    full_mask = mask_size - 1
    best_state = None
    best_total = float('inf')
    for prev in range(count):
        for last in intermediates:
            current_cost = dp[full_mask][prev][last]
            if current_cost == float('inf'):
                continue
            edge = cost[last][end_idx]
            if edge == float('inf'):
                continue
            total = current_cost + edge + turn_penalty(points[prev], points[last], points[end_idx], turn_weight)
            if total < best_total:
                best_total = total
                best_state = (full_mask, prev, last)

    if best_state is None:
        return None

    reversed_middle = []
    state = best_state
    while state is not None:
        _, _, last = state
        reversed_middle.append(last)
        state = parent.get(state)
    reversed_middle.reverse()
    order = [start_idx] + reversed_middle + [end_idx]
    return order if len(order) == count else None


def route_objective(
    order_idx: Sequence[int],
    cost: List[List[float]],
    points: Sequence[Point],
    turn_weight: float = 0.525,
) -> float:
    """Return path length plus the turn penalty for an open route."""
    if len(order_idx) <= 1:
        return 0.0

    total = 0.0
    for start_idx, end_idx in zip(order_idx, order_idx[1:]):
        edge = cost[start_idx][end_idx]
        if edge == float("inf"):
            return float("inf")
        total += edge
    for prev_idx, current_idx, next_idx in zip(
        order_idx, order_idx[1:], order_idx[2:]
    ):
        total += turn_penalty(
            points[prev_idx], points[current_idx], points[next_idx], turn_weight
        )
    return total


def tsp_open_exact(
    cost: List[List[float]],
    points: List[Point],
    start_idx: int = 0,
    turn_weight: float = 0.525,
) -> Optional[List[int]]:
    """Solve an open TSP exactly while keeping only the start fixed."""
    count = len(cost)
    if count == 0:
        return []

    targets = [index for index in range(count) if index != start_idx]
    if not targets:
        return [start_idx]

    target_bits = {node: bit for bit, node in enumerate(targets)}
    mask_size = 1 << len(targets)
    dp = [[[float("inf")] * count for _ in range(count)] for _ in range(mask_size)]
    parent: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {}

    for first in targets:
        edge = cost[start_idx][first]
        if edge == float("inf"):
            continue
        mask = 1 << target_bits[first]
        dp[mask][start_idx][first] = edge
        parent[(mask, start_idx, first)] = None

    for mask in range(mask_size):
        for prev in range(count):
            for last in targets:
                current_cost = dp[mask][prev][last]
                if current_cost == float("inf"):
                    continue
                for nxt in targets:
                    bit = 1 << target_bits[nxt]
                    if mask & bit:
                        continue
                    edge = cost[last][nxt]
                    if edge == float("inf"):
                        continue
                    new_mask = mask | bit
                    candidate = current_cost + edge + turn_penalty(
                        points[prev], points[last], points[nxt], turn_weight
                    )
                    if candidate < dp[new_mask][last][nxt]:
                        dp[new_mask][last][nxt] = candidate
                        parent[(new_mask, last, nxt)] = (mask, prev, last)

    full_mask = mask_size - 1
    best_state = None
    best_cost = float("inf")
    for prev in range(count):
        for last in targets:
            candidate = dp[full_mask][prev][last]
            if candidate < best_cost:
                best_cost = candidate
                best_state = (full_mask, prev, last)

    if best_state is None:
        return None

    reversed_targets = []
    state = best_state
    while state is not None:
        reversed_targets.append(state[2])
        state = parent.get(state)
    reversed_targets.reverse()
    order = [start_idx] + reversed_targets
    return order if len(order) == count else None


def tsp_open_nearest_neighbor_2opt(
    cost: List[List[float]],
    points: List[Point],
    start_idx: int = 0,
    turn_weight: float = 0.525,
) -> Optional[List[int]]:
    """Build a map-cost nearest-neighbor route and improve it with open 2-opt."""
    count = len(cost)
    if count == 0:
        return []

    unvisited = set(range(count))
    unvisited.discard(start_idx)
    order = [start_idx]
    while unvisited:
        last = order[-1]
        nxt = min(unvisited, key=lambda index: (cost[last][index], index))
        if cost[last][nxt] == float("inf"):
            return None
        order.append(nxt)
        unvisited.remove(nxt)

    best_cost = route_objective(order, cost, points, turn_weight)
    if best_cost == float("inf"):
        return None

    # Keep the robot start fixed. Reversals may include the free final target.
    for _ in range(50):
        improved_order = None
        improved_cost = best_cost
        for begin in range(1, len(order) - 1):
            for end in range(begin + 1, len(order)):
                candidate = order[:begin] + list(reversed(order[begin : end + 1])) + order[end + 1 :]
                candidate_cost = route_objective(candidate, cost, points, turn_weight)
                if candidate_cost + 1e-9 < improved_cost:
                    improved_order = candidate
                    improved_cost = candidate_cost
        if improved_order is None:
            break
        order = improved_order
        best_cost = improved_cost
    return order


def _concat_segments(order_idx: Sequence[int], pairwise: PairwisePaths) -> Optional[List[Point]]:
    full_path: List[Point] = []
    for start_idx, end_idx in zip(order_idx, order_idx[1:]):
        segment = pairwise.path[start_idx][end_idx]
        if not segment:
            return None
        if not full_path:
            full_path.extend(segment)
        else:
            full_path.extend(segment[1:])
    return full_path


def _pick_final_path(raw_path: List[Point], ordered_points: List[Point], planning_map, base_map, smooth: bool) -> Optional[List[Point]]:
    required = set(ordered_points)
    if not validate_path_segments(raw_path, planning_map):
        if not validate_path_segments(raw_path, base_map):
            return None
    if not smooth:
        return raw_path if required.issubset(set(raw_path)) else None

    smoothed = smooth_path(raw_path, planning_map, required_points=ordered_points)
    if validate_path_segments(smoothed, planning_map) and required.issubset(set(smoothed)):
        return smoothed
    if validate_path_segments(raw_path, planning_map) and required.issubset(set(raw_path)):
        return raw_path
    return raw_path if required.issubset(set(raw_path)) else None


def _run_pipeline(points: List[Point], planning_map, base_map, weight: float, turn_weight: float, smooth: bool, inflation_cells: int) -> Optional[InspectionPlanResult]:
    count = len(points)
    pairwise = compute_pairwise_paths(points, planning_map, weight=weight)
    if not _pairwise_has_hamiltonian_edges(pairwise, count):
        return None

    order_idx = tsp_fixed_start_end(pairwise.cost, points=points, start_idx=0, end_idx=count - 1, turn_weight=turn_weight)
    if not order_idx:
        return None

    ordered_points = [points[index] for index in order_idx]
    raw_path = _concat_segments(order_idx, pairwise)
    if not raw_path:
        return None

    final_path = _pick_final_path(raw_path, ordered_points, planning_map, base_map, smooth)
    if not final_path:
        return None

    return InspectionPlanResult(
        visiting_order=ordered_points,
        raw_path=raw_path,
        final_path=final_path,
        raw_cost=path_cost(raw_path),
        final_cost=path_cost(final_path),
        inflation_cells=inflation_cells,
    )


def build_inspection_path(start: Point, end: Point, waypoints: List[Point], grid_map, weight: float = 1.0, turn_weight: float = 0.525, smooth: bool = True, inflate_obstacles: bool = True) -> Optional[InspectionPlanResult]:
    base_map = grid_map
    points: List[Point] = [start] + list(waypoints) + [end]
    if not all(base_map.is_valid(point[0], point[1]) for point in points):
        return None

    if inflate_obstacles:
        planning_map, inflation_cells = resolve_planning_map(base_map, points)
    else:
        planning_map, inflation_cells = base_map, 0

    result = _run_pipeline(points, planning_map, base_map, weight, turn_weight, smooth, inflation_cells)
    if result:
        return result
    if inflate_obstacles and inflation_cells > 0:
        return _run_pipeline(points, base_map, base_map, weight, turn_weight, smooth, 0)
    return None


def _map_order_to_indices(visited_points: Sequence[Point], original_points: Sequence[Point]) -> Optional[List[int]]:
    buckets: Dict[Point, List[int]] = {}
    for index, point in enumerate(original_points):
        buckets.setdefault(point, []).append(index)

    ordered_indices: List[int] = []
    for point in visited_points:
        candidates = buckets.get(point)
        if not candidates:
            return None
        ordered_indices.append(candidates.pop(0))
    return ordered_indices


def _world_path(grid_map: GridMap, path: Sequence[Point], start_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
    world_path = [tuple(start_xy)]
    for index, point in enumerate(path):
        world_point = grid_map.grid_to_world(point[0], point[1])
        if index == 0:
            world_path[0] = world_point
        else:
            world_path.append(world_point)
    return world_path


def _extract_xy(point) -> Tuple[float, float]:
    return float(point.x), float(point.y)


def _heading_penalty(source_heading, target_heading, weight=0.525):
    return weight * (1.0 - math.cos(target_heading - source_heading))


def _transition_cost(path, source_heading, target_heading, turn_weight=0.525):
    if not path:
        return float("inf")
    total = path_cost(path)
    if len(path) < 2:
        return total + _heading_penalty(source_heading, target_heading, turn_weight)
    first = path[0]
    second = path[1]
    before_target = path[-2]
    target = path[-1]
    first_heading = math.atan2(second[1] - first[1], second[0] - first[0])
    final_heading = math.atan2(
        target[1] - before_target[1], target[0] - before_target[0]
    )
    return (
        total
        + _heading_penalty(source_heading, first_heading, turn_weight)
        + _heading_penalty(final_heading, target_heading, turn_weight)
    )


def _best_region_options_for_order(order, initial_cost, transition_cost, terminal_cost):
    if not order:
        return 0.0, []
    states = {}
    parents = []
    first_region = order[0]
    for option_index, cost in enumerate(initial_cost[first_region]):
        if cost != float("inf"):
            states[option_index] = cost
    if not states:
        return float("inf"), []
    parents.append({option_index: None for option_index in states})

    for previous_region, region_index in zip(order, order[1:]):
        next_states = {}
        next_parents = {}
        for option_index in range(len(initial_cost[region_index])):
            best_cost = float("inf")
            best_previous = None
            for previous_option, previous_cost in states.items():
                edge = transition_cost[previous_region][previous_option][
                    region_index
                ][option_index]
                candidate = previous_cost + edge
                if candidate < best_cost:
                    best_cost = candidate
                    best_previous = previous_option
            if best_previous is not None and best_cost != float("inf"):
                next_states[option_index] = best_cost
                next_parents[option_index] = best_previous
        if not next_states:
            return float("inf"), []
        states = next_states
        parents.append(next_parents)

    final_region = order[-1]
    best_option = None
    best_total = float("inf")
    for option_index, cost in states.items():
        candidate = cost + terminal_cost[final_region][option_index]
        if candidate < best_total:
            best_total = candidate
            best_option = option_index
    if best_option is None:
        return float("inf"), []

    options = [best_option]
    for parent_by_option in reversed(parents[1:]):
        best_option = parent_by_option[best_option]
        options.append(best_option)
    options.reverse()
    return best_total, options


def _solve_region_exact(initial_cost, transition_cost, terminal_cost):
    region_count = len(initial_cost)
    states = {}
    parent = {}
    for region_index, option_costs in enumerate(initial_cost):
        for option_index, cost in enumerate(option_costs):
            if cost == float("inf"):
                continue
            key = (1 << region_index, region_index, option_index)
            states[key] = cost
            parent[key] = None

    for mask in range(1, 1 << region_count):
        current_states = [item for item in states.items() if item[0][0] == mask]
        for (state_mask, previous_region, previous_option), current_cost in current_states:
            for region_index in range(region_count):
                if state_mask & (1 << region_index):
                    continue
                for option_index in range(len(initial_cost[region_index])):
                    edge = transition_cost[previous_region][previous_option][
                        region_index
                    ][option_index]
                    if edge == float("inf"):
                        continue
                    next_key = (
                        state_mask | (1 << region_index),
                        region_index,
                        option_index,
                    )
                    candidate = current_cost + edge
                    if candidate < states.get(next_key, float("inf")):
                        states[next_key] = candidate
                        parent[next_key] = (
                            state_mask,
                            previous_region,
                            previous_option,
                        )

    full_mask = (1 << region_count) - 1
    best_key = None
    best_total = float("inf")
    for key, cost in states.items():
        mask, region_index, option_index = key
        if mask != full_mask:
            continue
        candidate = cost + terminal_cost[region_index][option_index]
        if candidate < best_total:
            best_total = candidate
            best_key = key
    if best_key is None:
        return None

    reversed_order = []
    reversed_options = []
    key = best_key
    while key is not None:
        _, region_index, option_index = key
        reversed_order.append(region_index)
        reversed_options.append(option_index)
        key = parent[key]
    return best_total, list(reversed(reversed_order)), list(reversed(reversed_options))


def _solve_region_heuristic(initial_cost, transition_cost, terminal_cost):
    region_count = len(initial_cost)
    unvisited = set(range(region_count))
    order = []
    previous_region = None
    previous_option = None
    while unvisited:
        best = None
        for region_index in sorted(unvisited):
            for option_index in range(len(initial_cost[region_index])):
                if previous_region is None:
                    edge = initial_cost[region_index][option_index]
                else:
                    edge = transition_cost[previous_region][previous_option][region_index][option_index]
                candidate = (edge, region_index, option_index)
                if edge != float("inf") and (best is None or candidate < best):
                    best = candidate
        if best is None:
            return None
        _, previous_region, previous_option = best
        order.append(previous_region)
        unvisited.remove(previous_region)

    best_cost, best_options = _best_region_options_for_order(
        order, initial_cost, transition_cost, terminal_cost
    )
    if best_cost == float("inf"):
        return None
    improved = True
    while improved:
        improved = False
        for start in range(region_count - 1):
            for end in range(start + 1, region_count):
                candidate_order = (
                    order[:start]
                    + list(reversed(order[start : end + 1]))
                    + order[end + 1 :]
                )
                candidate_cost, candidate_options = _best_region_options_for_order(
                    candidate_order, initial_cost, transition_cost, terminal_cost
                )
                if candidate_cost + 1e-9 < best_cost:
                    order = candidate_order
                    best_cost = candidate_cost
                    best_options = candidate_options
                    improved = True
                    break
            if improved:
                break
    return best_cost, order, best_options


def plan_region_mission_order(
    map_msg,
    start_pose: Tuple[float, float, float],
    option_groups: Sequence[Sequence[RegionRouteOption]],
    end_pose: Optional[Tuple[float, float, float]] = None,
    smooth: bool = True,
    cancel_check: Optional[Callable[[], bool]] = None,
) -> Optional[RegionMissionPlan]:
    """Jointly optimize region order and forward/reverse coverage direction."""
    if not option_groups or any(not group for group in option_groups):
        return None
    grid_map = GridMap.from_occupancy_grid(map_msg)
    start = grid_map.world_to_grid(start_pose[0], start_pose[1])
    end = None if end_pose is None else grid_map.world_to_grid(end_pose[0], end_pose[1])
    option_cells = [
        [
            (
                grid_map.world_to_grid(*option.entry_xy),
                grid_map.world_to_grid(*option.exit_xy),
            )
            for option in group
        ]
        for group in option_groups
    ]
    critical_points = [start]
    for group in option_cells:
        for entry, exit_point in group:
            critical_points.extend((entry, exit_point))
    if end is not None:
        critical_points.append(end)
    if not all(grid_map.is_valid(point[0], point[1]) for point in critical_points):
        return None

    planning_map, inflation_cells = resolve_planning_map(grid_map, critical_points)
    candidates = [(planning_map, inflation_cells)]
    if planning_map is not grid_map and inflation_cells > 0:
        candidates.append((grid_map, 0))

    region_count = len(option_groups)
    solving_method = (
        "single"
        if region_count == 1
        else "exact" if region_count <= 10 else "nearest_neighbor_2opt"
    )
    for candidate_map, candidate_inflation in candidates:
        path_cache = {}

        def segment(source, target):
            if cancel_check is not None and cancel_check():
                raise PlanningCancelled()
            key = (source, target)
            if key not in path_cache:
                path_cache[key] = _find_segment_path(source, target, candidate_map)
            return path_cache[key]

        try:
            initial_cost = []
            initial_paths = []
            for region_index, group in enumerate(option_groups):
                costs = []
                paths = []
                for option_index, option in enumerate(group):
                    path = segment(start, option_cells[region_index][option_index][0])
                    paths.append(path)
                    costs.append(
                        _transition_cost(
                            path, start_pose[2], option.entry_heading
                        )
                    )
                initial_cost.append(costs)
                initial_paths.append(paths)

            transition_cost = []
            transition_paths = []
            for source_region, source_group in enumerate(option_groups):
                source_cost_groups = []
                source_path_groups = []
                for source_option, source in enumerate(source_group):
                    target_regions = []
                    target_path_regions = []
                    source_exit = option_cells[source_region][source_option][1]
                    for target_region, target_group in enumerate(option_groups):
                        target_costs = []
                        target_paths = []
                        for target_option, target in enumerate(target_group):
                            if source_region == target_region:
                                path = None
                            else:
                                path = segment(
                                    source_exit,
                                    option_cells[target_region][target_option][0],
                                )
                            target_paths.append(path)
                            target_costs.append(
                                _transition_cost(
                                    path,
                                    source.exit_heading,
                                    target.entry_heading,
                                )
                            )
                        target_regions.append(target_costs)
                        target_path_regions.append(target_paths)
                    source_cost_groups.append(target_regions)
                    source_path_groups.append(target_path_regions)
                transition_cost.append(source_cost_groups)
                transition_paths.append(source_path_groups)

            terminal_cost = []
            terminal_paths = []
            for region_index, group in enumerate(option_groups):
                costs = []
                paths = []
                for option_index, option in enumerate(group):
                    if end is None:
                        path = None
                        cost = 0.0
                    else:
                        path = segment(option_cells[region_index][option_index][1], end)
                        cost = _transition_cost(
                            path, option.exit_heading, end_pose[2]
                        )
                    paths.append(path)
                    costs.append(cost)
                terminal_cost.append(costs)
                terminal_paths.append(paths)
        except PlanningCancelled:
            return None

        if solving_method == "exact" or solving_method == "single":
            solved = _solve_region_exact(
                initial_cost, transition_cost, terminal_cost
            )
        else:
            solved = _solve_region_heuristic(
                initial_cost, transition_cost, terminal_cost
            )
        if solved is None:
            continue
        raw_cost, order, options = solved

        selected_paths = []
        first_region = order[0]
        first_option = options[0]
        selected_paths.append(initial_paths[first_region][first_option])
        for source_region, source_option, target_region, target_option in zip(
            order, options, order[1:], options[1:]
        ):
            selected_paths.append(
                transition_paths[source_region][source_option][target_region][target_option]
            )
        final_raw_path = terminal_paths[order[-1]][options[-1]]

        world_transitions = []
        for raw_path in selected_paths:
            if not raw_path:
                world_transitions = []
                break
            final_path = _pick_final_path(
                raw_path,
                [raw_path[0], raw_path[-1]],
                candidate_map,
                grid_map,
                smooth,
            )
            if not final_path:
                world_transitions = []
                break
            world_transitions.append(
                [grid_map.grid_to_world(point[0], point[1]) for point in final_path]
            )
        if len(world_transitions) != len(selected_paths):
            continue

        world_return = []
        if final_raw_path:
            final_path = _pick_final_path(
                final_raw_path,
                [final_raw_path[0], final_raw_path[-1]],
                candidate_map,
                grid_map,
                smooth,
            )
            if not final_path:
                continue
            world_return = [
                grid_map.grid_to_world(point[0], point[1]) for point in final_path
            ]
        return RegionMissionPlan(
            ordered_indices=order,
            option_indices=options,
            transition_paths=world_transitions,
            return_path=world_return,
            raw_cost=raw_cost,
            inflation_cells=candidate_inflation,
            solving_method=solving_method,
        )
    return None


def plan_mission_order(
    map_msg,
    start_xy: Tuple[float, float],
    points: Sequence[object],
    smooth: bool = True,
    cancel_check: Optional[Callable[[], bool]] = None,
) -> Optional[MissionPlan]:
    if not points:
        return None
    if cancel_check is not None and cancel_check():
        return None

    grid_map = GridMap.from_occupancy_grid(map_msg)
    start = grid_map.world_to_grid(start_xy[0], start_xy[1])
    if not grid_map.is_valid(start[0], start[1]):
        return None

    target_points = [grid_map.world_to_grid(*_extract_xy(point)) for point in points]
    if not all(grid_map.is_valid(point[0], point[1]) for point in target_points):
        return None

    if len(target_points) == 1:
        if cancel_check is not None and cancel_check():
            return None
        preview = preview_current_order(map_msg, start_xy, points, smooth=smooth)
        if not preview:
            return None
        return MissionPlan(
            ordered_indices=[0],
            preview_path=preview.preview_path,
            raw_cost=preview.raw_cost,
            final_cost=preview.final_cost,
            inflation_cells=preview.inflation_cells,
            solving_method="single",
        )

    critical_points = [start] + target_points
    planning_map, inflation_cells = resolve_planning_map(grid_map, critical_points)
    candidates = [(planning_map, inflation_cells)]
    if planning_map is not grid_map and inflation_cells > 0:
        candidates.append((grid_map, 0))

    solving_method = (
        "exact" if len(target_points) <= 10 else "nearest_neighbor_2opt"
    )
    for candidate_map, candidate_inflation in candidates:
        try:
            pairwise = compute_pairwise_paths(
                critical_points,
                candidate_map,
                cancel_check=cancel_check,
            )
        except PlanningCancelled:
            return None
        if solving_method == "exact":
            order_idx = tsp_open_exact(pairwise.cost, critical_points)
        else:
            order_idx = tsp_open_nearest_neighbor_2opt(
                pairwise.cost, critical_points
            )
        if not order_idx:
            continue

        raw_path = _concat_segments(order_idx, pairwise)
        if not raw_path:
            continue
        ordered_grid_points = [critical_points[index] for index in order_idx]
        final_path = _pick_final_path(
            raw_path,
            ordered_grid_points,
            candidate_map,
            grid_map,
            smooth,
        )
        if not final_path:
            continue

        return MissionPlan(
            ordered_indices=[index - 1 for index in order_idx[1:]],
            preview_path=_world_path(grid_map, final_path, start_xy),
            raw_cost=path_cost(raw_path),
            final_cost=path_cost(final_path),
            inflation_cells=candidate_inflation,
            solving_method=solving_method,
        )
    return None


def preview_current_order(map_msg, start_xy: Tuple[float, float], points: Sequence[object], smooth: bool = True) -> Optional[MissionPlan]:
    if not points:
        return None

    grid_map = GridMap.from_occupancy_grid(map_msg)
    start = grid_map.world_to_grid(start_xy[0], start_xy[1])
    ordered_points = [grid_map.world_to_grid(*_extract_xy(point)) for point in points]
    critical_points = [start] + ordered_points
    if not all(grid_map.is_valid(point[0], point[1]) for point in critical_points):
        return None

    planning_map, inflation_cells = resolve_planning_map(grid_map, critical_points)
    raw_path: List[Point] = []
    for start_point, end_point in zip(critical_points, critical_points[1:]):
        segment = _find_segment_path(start_point, end_point, planning_map)
        if segment is None and planning_map is not grid_map:
            segment = _find_segment_path(start_point, end_point, grid_map)
        if not segment:
            return None
        if not raw_path:
            raw_path.extend(segment)
        else:
            raw_path.extend(segment[1:])

    final_path = _pick_final_path(raw_path, critical_points, planning_map, grid_map, smooth)
    if not final_path:
        return None

    return MissionPlan(
        ordered_indices=list(range(len(points))),
        preview_path=_world_path(grid_map, final_path, start_xy),
        raw_cost=path_cost(raw_path),
        final_cost=path_cost(final_path),
        inflation_cells=inflation_cells,
    )



def validate_mission_points(
    map_msg,
    start_xy: Tuple[float, float],
    points: Sequence[object],
    obstacle_margin_m: float = 0.12,
    min_start_distance_m: float = 0.18,
    optimize_order: bool = False,
    mission_plan: Optional[MissionPlan] = None,
    check_route: bool = True,
) -> MissionValidation:
    if not points:
        return MissionValidation(False, 'No mission points available')

    grid_map = GridMap.from_occupancy_grid(map_msg)
    start = grid_map.world_to_grid(start_xy[0], start_xy[1])
    if not grid_map.is_valid(start[0], start[1]):
        return MissionValidation(False, 'Robot pose is invalid on the current map. Re-localize before starting a mission.')

    for point in points:
        gx, gy = grid_map.world_to_grid(*_extract_xy(point))
        name = getattr(point, 'point_name', '') or 'Unnamed'
        if not grid_map.in_bounds(gx, gy):
            return MissionValidation(False, f'Mission point {name} is outside the current map bounds')
        if not grid_map.is_valid(gx, gy):
            return MissionValidation(False, f'Mission point {name} is in an obstacle or unknown area')

    if not check_route:
        return MissionValidation(
            True,
            f'Mission point locations validated: {len(points)} traversable waypoint(s)',
        )

    preview = mission_plan
    if preview is None:
        preview = (
            plan_mission_order(map_msg, start_xy, points, smooth=False)
            if optimize_order
            else preview_current_order(map_msg, start_xy, points, smooth=False)
        )
    if preview is None:
        if optimize_order:
            return MissionValidation(
                False,
                'The selected mission points do not have a complete collision-free TSP route. Adjust the points before starting navigation.',
            )
        return MissionValidation(False, 'The selected mission points do not have a collision-free path in the current order. Adjust the points before starting navigation.')

    first_index = preview.ordered_indices[0] if optimize_order else 0
    first_point = points[first_index]
    first_x, first_y = _extract_xy(first_point)
    if math.hypot(first_x - start_xy[0], first_y - start_xy[1]) < min_start_distance_m:
        name = getattr(first_point, 'point_name', '') or 'Unnamed'
        return MissionValidation(False, f'Mission point {name} is too close to the robot start pose. Move it at least {min_start_distance_m:.2f} m away before starting navigation.')

    if optimize_order:
        return MissionValidation(
            True,
            f'Mission points validated: {len(points)} safe waypoint(s), complete TSP route available',
        )
    return MissionValidation(True, f'Mission points validated: {len(points)} safe waypoint(s) ready')
