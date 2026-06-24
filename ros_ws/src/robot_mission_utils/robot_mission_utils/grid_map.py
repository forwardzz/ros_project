from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[int, int]


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Tuple[int, ...]
    occupied_threshold: int = 50
    unknown_is_obstacle: bool = True

    @classmethod
    def from_occupancy_grid(cls, map_msg):
        info = map_msg.info
        return cls(
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            data=tuple(int(value) for value in map_msg.data),
        )

    def clone_with_data(self, data: Sequence[int]) -> "GridMap":
        return GridMap(
            width=self.width,
            height=self.height,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            data=tuple(int(value) for value in data),
            occupied_threshold=self.occupied_threshold,
            unknown_is_obstacle=self.unknown_is_obstacle,
        )

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def index(self, x: int, y: int) -> int:
        return y * self.width + x

    def value(self, x: int, y: int) -> int:
        return self.data[self.index(x, y)]

    def is_valid(self, x: int, y: int) -> bool:
        if not self.in_bounds(x, y):
            return False
        value = self.value(x, y)
        if value < 0:
            return not self.unknown_is_obstacle
        return value < self.occupied_threshold

    def world_to_grid(self, x: float, y: float) -> Point:
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, x: int, y: int) -> Tuple[float, float]:
        wx = (x + 0.5) * self.resolution + self.origin_x
        wy = (y + 0.5) * self.resolution + self.origin_y
        return wx, wy

    def neighbors8(self, x: int, y: int) -> Iterable[Tuple[Point, float]]:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if self.is_valid(nx, ny):
                    cost = 1.41421356237 if dx != 0 and dy != 0 else 1.0
                    yield (nx, ny), cost

    def occupied_cells(self) -> List[Point]:
        cells = []
        for y in range(self.height):
            for x in range(self.width):
                if not self.is_valid(x, y):
                    cells.append((x, y))
        return cells
