from itertools import permutations

import numpy as np


class TSPPlanner:
    def __init__(self):
        self.distance_matrix = None

    def calculate_distance_matrix(self, points):
        n = len(points)
        self.distance_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                dist = np.sqrt((points[i].x - points[j].x) ** 2 + (points[i].y - points[j].y) ** 2)
                self.distance_matrix[i][j] = dist
        return self.distance_matrix

    def solve_tsp_brute_force(self, points):
        n = len(points)
        if n <= 2:
            return list(range(n))

        if n > 10:
            return self._solve_tsp_nearest_neighbor(points)

        min_distance = float("inf")
        best_order = None
        cities = list(range(1, n))
        for perm in permutations(cities):
            order = [0] + list(perm) + [0]
            distance = 0
            for i in range(len(order) - 1):
                distance += self.distance_matrix[order[i]][order[i + 1]]
            if distance < min_distance:
                min_distance = distance
                best_order = order[:-1]
        return best_order

    def _solve_tsp_nearest_neighbor(self, points):
        n = len(points)
        visited = [False] * n
        path = [0]
        visited[0] = True

        for _ in range(n - 1):
            last = path[-1]
            nearest = -1
            min_dist = float("inf")
            for j in range(n):
                if not visited[j] and self.distance_matrix[last][j] < min_dist:
                    nearest = j
                    min_dist = self.distance_matrix[last][j]
            path.append(nearest)
            visited[nearest] = True

        return path

    def solve_tsp_dynamic_programming(self, points):
        n = len(points)
        if n <= 2:
            return list(range(n))

        if n > 15:
            return self._solve_tsp_nearest_neighbor(points)

        inf = float("inf")
        dp = [[inf] * (1 << n) for _ in range(n)]
        dp[0][1] = 0

        for mask in range(1, 1 << n):
            for u in range(n):
                if (mask >> u) & 1 and dp[u][mask] < inf:
                    for v in range(n):
                        if not ((mask >> v) & 1):
                            new_mask = mask | (1 << v)
                            new_cost = dp[u][mask] + self.distance_matrix[u][v]
                            if new_cost < dp[v][new_mask]:
                                dp[v][new_mask] = new_cost

        path = [0]
        mask = (1 << n) - 1
        current = 0
        remaining = set(range(1, n))

        while remaining:
            best_next = -1
            best_cost = inf
            for v in remaining:
                cost = dp[v][mask ^ (1 << current)] + self.distance_matrix[current][v]
                if cost < best_cost:
                    best_cost = cost
                    best_next = v
            if best_next == -1:
                break
            path.append(best_next)
            mask ^= 1 << current
            current = best_next
            remaining.remove(current)

        return path
