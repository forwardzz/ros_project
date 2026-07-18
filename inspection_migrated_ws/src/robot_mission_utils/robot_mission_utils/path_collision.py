from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

Point = Tuple[int, int]


def bresenham_cells(start: Point, end: Point) -> List[Point]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    cells: List[Point] = []

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return cells


def line_of_sight(a: Point, b: Point, grid_map) -> bool:
    return all(grid_map.is_valid(x, y) for x, y in bresenham_cells(a, b))


def validate_path_segments(path: Sequence[Point], grid_map) -> bool:
    if not path:
        return False
    for point in path:
        if not grid_map.is_valid(point[0], point[1]):
            return False
    for start, end in zip(path, path[1:]):
        if not line_of_sight(start, end, grid_map):
            return False
    return True
