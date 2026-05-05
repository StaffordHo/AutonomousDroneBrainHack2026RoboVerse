import heapq
import math
import numpy as np

class AStarPlanner:
    def __init__(self, grid, origin_offset, resolution=0.5):
        """
        grid: 2D numpy array (1=obstacle, 0=free)
        origin_offset: (min_n, min_e) in meters
        resolution: meters per cell
        """
        self.grid = grid
        self.min_n, self.min_e = origin_offset
        self.res = resolution
        self.height, self.width = grid.shape

    def ned_to_grid(self, n, e):
        row = int((n - self.min_n) / self.res)
        col = int((e - self.min_e) / self.res)
        return (row, col)

    def grid_to_ned(self, row, col):
        n = row * self.res + self.min_n
        e = col * self.res + self.min_e
        return (n, e)

    def is_valid(self, row, col):
        if 0 <= row < self.height and 0 <= col < self.width:
            return self.grid[row, col] == 0
        return False

    def find_path(self, start_ned, goal_ned):
        start = self.ned_to_grid(start_ned[0], start_ned[1])
        goal = self.ned_to_grid(goal_ned[0], goal_ned[1])

        if not self.is_valid(start[0], start[1]):
            # If start is blocked (e.g. drone is very close to wall), find nearest free cell
            print("A* Warning: Start cell is blocked. Finding nearest free cell...")
            start = self.find_nearest_free(start)
        
        if not self.is_valid(goal[0], goal[1]):
            print("A* Warning: Goal cell is blocked. Finding nearest free cell...")
            goal = self.find_nearest_free(goal)

        # Priority queue: (f_score, row, col)
        pq = [(0, start[0], start[1])]
        came_from = {}
        g_score = {start: 0}
        
        # 8-connectivity
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), 
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while pq:
            _, r, c = heapq.heappop(pq)
            current = (r, c)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dr, dc in neighbors:
                neighbor = (r + dr, c + dc)
                
                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue
                
                # Diagonal distance check
                step_cost = 1.414 if dr != 0 and dc != 0 else 1.0
                tentative_g = g_score[current] + step_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    came_from[neighbor] = current
                    heapq.heappush(pq, (f, neighbor[0], neighbor[1]))

        return None # No path found

    def heuristic(self, a, b):
        # Euclidean distance
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(self.grid_to_ned(current[0], current[1]))
            current = came_from[current]
        path.reverse()
        
        # Path smoothing: return only points where direction changes significantly
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            p1 = path[i-1]
            p2 = path[i]
            p3 = path[i+1]
            
            # Cross product to check collinearity
            # (y2-y1)*(x3-x2) - (x2-x1)*(y3-y2)
            cross = (p2[1]-p1[1])*(p3[0]-p2[0]) - (p2[0]-p1[0])*(p3[1]-p2[1])
            if abs(cross) > 0.01:
                smoothed.append(p2)
        
        smoothed.append(path[-1])
        return smoothed

    def find_nearest_free(self, pos):
        # BFS to find nearest 0 in grid
        queue = [pos]
        visited = {pos}
        while queue:
            r, c = queue.pop(0)
            if self.is_valid(r, c):
                return (r, c)
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                n = (r+dr, c+dc)
                if 0 <= n[0] < self.height and 0 <= n[1] < self.width and n not in visited:
                    visited.add(n)
                    queue.append(n)
        return pos
