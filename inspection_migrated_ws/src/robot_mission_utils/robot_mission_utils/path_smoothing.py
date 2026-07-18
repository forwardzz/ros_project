from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

from .path_collision import line_of_sight

Point = Tuple[int, int]


def _smooth_segment(points: Sequence[Point], grid_map) -> List[Point]:
    if len(points) <= 2:
        return list(points)
    result: List[Point] = [points[0]]
    anchor_index = 0
    while anchor_index < len(points) - 1:
        next_index = anchor_index + 1
        for probe in range(anchor_index + 2, len(points)):
            if line_of_sight(points[anchor_index], points[probe], grid_map):
                next_index = probe
            else:
                break
        result.append(points[next_index])
        anchor_index = next_index
    return result


def smooth_path(path: Sequence[Point], grid_map, required_points: Sequence[Point]):
    if not path:
        return []

    required = set(required_points)
    anchor_indices = [0]
    for index, point in enumerate(path[1:-1], start=1):
        if point in required:
            anchor_indices.append(index)
    anchor_indices.append(len(path) - 1)

    smoothed: List[Point] = []
    for start_index, end_index in zip(anchor_indices, anchor_indices[1:]):
        segment = _smooth_segment(path[start_index:end_index + 1], grid_map)
        if not smoothed:
            smoothed.extend(segment)
        else:
            smoothed.extend(segment[1:])
    return smoothed
