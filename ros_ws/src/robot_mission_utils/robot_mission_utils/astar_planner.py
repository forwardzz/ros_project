import heapq

import numpy as np


class AStarPlanner:
    def __init__(self, resolution=0.05, inflation_radius=0.3):
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = resolution

    def set_map(self, map_msg):
        self.map_data = map_msg.data
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y
        self.map_resolution = map_msg.info.resolution

    def world_to_grid(self, x, y):
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        x = grid_x * self.map_resolution + self.map_origin_x
        y = grid_y * self.map_resolution + self.map_origin_y
        return x, y

    def is_valid(self, x, y):
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return False
        idx = y * self.map_width + x
        return self.map_data[idx] != 100 and self.map_data[idx] != -1

    def heuristic(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def get_neighbors(self, pos):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_x = pos[0] + dx
                new_y = pos[1] + dy
                if self.is_valid(new_x, new_y):
                    cost = 1.0 if dx == 0 or dy == 0 else np.sqrt(2)
                    neighbors.append(((new_x, new_y), cost))
        return neighbors

    def plan(self, start, goal):
        if self.map_data is None:
            return []

        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])

        if not self.is_valid(start_grid[0], start_grid[1]):
            return []
        if not self.is_valid(goal_grid[0], goal_grid[1]):
            return []

        open_set = []
        heapq.heappush(open_set, (0, start_grid))

        came_from = {}
        g_score = {start_grid: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(current[0], current[1]))
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))

        return []
