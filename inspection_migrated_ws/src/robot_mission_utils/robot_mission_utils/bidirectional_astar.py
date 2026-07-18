from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

Point = Tuple[int, int]


def _heuristic(a: Point, b: Point) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx * dx + dy * dy) ** 0.5


def _reconstruct(came_from: Dict[Point, Point], current: Point) -> List[Point]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def bidirectional_astar(start: Point, goal: Point, grid_map, weight: float = 1.0) -> Optional[List[Point]]:
    if start == goal:
        return [start]
    if not grid_map.is_valid(start[0], start[1]) or not grid_map.is_valid(goal[0], goal[1]):
        return None

    open_set: List[Tuple[float, Point]] = []
    heapq.heappush(open_set, (0.0, start))
    came_from: Dict[Point, Point] = {}
    g_score: Dict[Point, float] = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return _reconstruct(came_from, current)

        for neighbor, step_cost in grid_map.neighbors8(current[0], current[1]):
            tentative_g = g_score[current] + step_cost
            if tentative_g >= g_score.get(neighbor, float('inf')):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score = tentative_g + weight * _heuristic(neighbor, goal)
            heapq.heappush(open_set, (f_score, neighbor))
    return None
