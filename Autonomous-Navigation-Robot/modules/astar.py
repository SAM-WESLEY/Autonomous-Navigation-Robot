import heapq
import numpy as np
from typing import List, Tuple, Optional


class Node:
    def __init__(self, pos, g=0, h=0, parent=None):
        self.pos    = pos
        self.g      = g       # cost from start
        self.h      = h       # heuristic to goal
        self.f      = g + h   # total cost
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.pos == other.pos


class AStarPlanner:
    def __init__(self, grid_size=(50, 50)):
        self.grid_size = grid_size
        self.grid      = np.zeros(grid_size, dtype=np.int8)

    def heuristic(self, a, b):
        """Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbours(self, pos):
        """Get valid 8-directional neighbours."""
        r, c = pos
        moves = [(-1,0),(1,0),(0,-1),(0,1),
                 (-1,-1),(-1,1),(1,-1),(1,1)]
        neighbours = []
        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            if (0 <= nr < self.grid_size[0] and
                0 <= nc < self.grid_size[1] and
                self.grid[nr, nc] == 0):
                # Diagonal moves cost more
                cost = 1.414 if abs(dr) + abs(dc) == 2 else 1.0
                neighbours.append(((nr, nc), cost))
        return neighbours

    def plan(self, start: Tuple, goal: Tuple) -> Optional[List[Tuple]]:
        """
        A* path planning from start to goal.
        Returns list of (row, col) waypoints or None if no path found.
        """
        if self.grid[goal[0], goal[1]] == 1:
            print("[A*] Goal is blocked!")
            return None

        open_list  = []
        closed_set = set()

        start_node = Node(start, g=0, h=self.heuristic(start, goal))
        heapq.heappush(open_list, start_node)
        open_dict  = {start: start_node}

        while open_list:
            current = heapq.heappop(open_list)

            if current.pos == goal:
                return self._reconstruct(current)

            closed_set.add(current.pos)

            for (npos, cost) in self.get_neighbours(current.pos):
                if npos in closed_set:
                    continue
                g_new = current.g + cost
                h_new = self.heuristic(npos, goal)
                neighbour = Node(npos, g=g_new, h=h_new, parent=current)

                if npos in open_dict and open_dict[npos].g <= g_new:
                    continue

                heapq.heappush(open_list, neighbour)
                open_dict[npos] = neighbour

        print("[A*] No path found!")
        return None

    def _reconstruct(self, node: Node) -> List[Tuple]:
        """Reconstruct path from goal to start."""
        path = []
        while node:
            path.append(node.pos)
            node = node.parent
        return list(reversed(path))

    def update_obstacle(self, pos, value=1):
        """Add or remove obstacle from grid."""
        r, c = pos
        if 0 <= r < self.grid_size[0] and 0 <= c < self.grid_size[1]:
            self.grid[r, c] = value

    def add_obstacle_circle(self, center, radius=2):
        """Add circular obstacle region."""
        cr, cc = center
        for r in range(max(0, cr-radius), min(self.grid_size[0], cr+radius+1)):
            for c in range(max(0, cc-radius), min(self.grid_size[1], cc+radius+1)):
                if (r-cr)**2 + (c-cc)**2 <= radius**2:
                    self.grid[r, c] = 1

    def clear_grid(self):
        self.grid[:] = 0

    def get_grid_json(self):
        return self.grid.tolist()


if __name__ == "__main__":
    planner = AStarPlanner(grid_size=(20, 20))
    planner.add_obstacle_circle((8, 8), radius=3)
    planner.add_obstacle_circle((5, 12), radius=2)

    path = planner.plan((0, 0), (18, 18))
    if path:
        print(f"[A*] Path found! {len(path)} waypoints.")
        # Visualise in terminal
        grid_vis = [['.' for _ in range(20)] for _ in range(20)]
        for r in range(20):
            for c in range(20):
                if planner.grid[r, c] == 1:
                    grid_vis[r][c] = '#'
        for (r, c) in path:
            grid_vis[r][c] = '*'
        grid_vis[0][0]   = 'S'
        grid_vis[18][18] = 'T'
        for row in grid_vis:
            print(' '.join(row))
