from __future__ import annotations

from typing import Iterable, Tuple

Point = Tuple[int, int]


def _inflate(grid_map, radius_cells: int):
    if radius_cells <= 0:
        return grid_map.clone_with_data(grid_map.data), 0

    data = list(grid_map.data)
    inflated = 0
    occupied = grid_map.occupied_cells()
    for ox, oy in occupied:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx = ox + dx
                ny = oy + dy
                if not grid_map.in_bounds(nx, ny):
                    continue
                idx = grid_map.index(nx, ny)
                if data[idx] >= grid_map.occupied_threshold:
                    continue
                data[idx] = 100
                inflated += 1
    return grid_map.clone_with_data(data), inflated


def inflate_map(base_map, radius_m: float = 0.08):
    radius_cells = max(1, int(round(radius_m / max(base_map.resolution, 1e-6))))
    return _inflate(base_map, radius_cells)


def resolve_planning_map(base_map, critical_points):
    planning_map, inflated = inflate_map(base_map, radius_m=0.08)
    if inflated == 0:
        return base_map, 0
    if all(planning_map.is_valid(point[0], point[1]) for point in critical_points):
        return planning_map, inflated
    return base_map, 0
