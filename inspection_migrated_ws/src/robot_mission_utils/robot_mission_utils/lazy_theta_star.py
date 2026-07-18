from __future__ import annotations

from typing import List, Optional, Tuple

from .bidirectional_astar import bidirectional_astar
from .path_collision import line_of_sight

Point = Tuple[int, int]


def lazy_theta_star(start: Point, goal: Point, grid_map, weight: float = 1.0) -> Optional[List[Point]]:
    path = bidirectional_astar(start, goal, grid_map, weight=weight)
    if not path:
        return None
    if len(path) <= 2:
        return path

    smoothed: List[Point] = [path[0]]
    anchor = path[0]
    index = 1
    while index < len(path):
        furthest = index
        for probe in range(index + 1, len(path)):
            if line_of_sight(anchor, path[probe], grid_map):
                furthest = probe
            else:
                break
        smoothed.append(path[furthest])
        anchor = path[furthest]
        index = furthest + 1
    return smoothed
