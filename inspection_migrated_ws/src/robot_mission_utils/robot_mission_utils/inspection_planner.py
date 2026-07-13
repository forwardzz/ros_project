from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple

from .bidirectional_astar import bidirectional_astar
from .grid_map import GridMap
from .lazy_theta_star import lazy_theta_star
from .map_inflation import inflate_map, resolve_planning_map
from .path_collision import bresenham_cells, validate_path_segments
from .path_smoothing import smooth_path

Point = Tuple[int, int]
STRAIGHT_COST = 1.0
DIAGONAL_COST = 1.7


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


def compute_pairwise_paths(points: List[Point], grid_map, weight: float = 1.0) -> PairwisePaths:
    count = len(points)
    cost = [[float('inf')] * count for _ in range(count)]
    path = [[None] * count for _ in range(count)]

    for i in range(count):
        cost[i][i] = 0.0
        path[i][i] = [points[i]]

    for i in range(count):
        for j in range(count):
            if i == j:
                continue
            segment = _find_segment_path(points[i], points[j], grid_map, weight=weight)
            if segment:
                path[i][j] = segment
                cost[i][j] = path_cost(segment)
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


def plan_mission_order(map_msg, start_xy: Tuple[float, float], points: Sequence[object], smooth: bool = True) -> Optional[MissionPlan]:
    if not points:
        return None

    grid_map = GridMap.from_occupancy_grid(map_msg)
    start = grid_map.world_to_grid(start_xy[0], start_xy[1])
    if not grid_map.is_valid(start[0], start[1]):
        return None

    target_points = [grid_map.world_to_grid(*_extract_xy(point)) for point in points]
    if not all(grid_map.is_valid(point[0], point[1]) for point in target_points):
        return None

    if len(target_points) == 1:
        preview = preview_current_order(map_msg, start_xy, points, smooth=smooth)
        if not preview:
            return None
        return MissionPlan(ordered_indices=[0], preview_path=preview.preview_path, raw_cost=preview.raw_cost, final_cost=preview.final_cost, inflation_cells=preview.inflation_cells)

    best_plan: Optional[MissionPlan] = None
    for end_index, end_point in enumerate(target_points):
        middle_points = [target_points[index] for index in range(len(target_points)) if index != end_index]
        result = build_inspection_path(start, end_point, middle_points, grid_map, smooth=smooth)
        if not result:
            continue
        ordered_indices = _map_order_to_indices(result.visiting_order[1:], target_points)
        if not ordered_indices or len(ordered_indices) != len(target_points):
            continue
        plan = MissionPlan(
            ordered_indices=ordered_indices,
            preview_path=_world_path(grid_map, result.final_path, start_xy),
            raw_cost=result.raw_cost,
            final_cost=result.final_cost,
            inflation_cells=result.inflation_cells,
        )
        if best_plan is None or plan.final_cost < best_plan.final_cost:
            best_plan = plan
    return best_plan


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
) -> MissionValidation:
    if not points:
        return MissionValidation(False, 'No mission points available')

    grid_map = GridMap.from_occupancy_grid(map_msg)
    start = grid_map.world_to_grid(start_xy[0], start_xy[1])
    if not grid_map.is_valid(start[0], start[1]):
        return MissionValidation(False, 'Robot pose is invalid on the current map. Re-localize before starting a mission.')

    first_x, first_y = _extract_xy(points[0])
    if math.hypot(first_x - start_xy[0], first_y - start_xy[1]) < min_start_distance_m:
        name = getattr(points[0], 'point_name', '') or 'Unnamed'
        return MissionValidation(False, f'Mission point {name} is too close to the robot start pose. Move it at least {min_start_distance_m:.2f} m away before starting navigation.')

    for point in points:
        gx, gy = grid_map.world_to_grid(*_extract_xy(point))
        name = getattr(point, 'point_name', '') or 'Unnamed'
        if not grid_map.in_bounds(gx, gy):
            return MissionValidation(False, f'Mission point {name} is outside the current map bounds')
        if not grid_map.is_valid(gx, gy):
            return MissionValidation(False, f'Mission point {name} is in an obstacle or unknown area')

    preview = preview_current_order(map_msg, start_xy, points, smooth=False)
    if preview is None:
        return MissionValidation(False, 'The selected mission points do not have a collision-free path in the current order. Adjust the points before starting navigation.')

    return MissionValidation(True, f'Mission points validated: {len(points)} safe waypoint(s) ready')
