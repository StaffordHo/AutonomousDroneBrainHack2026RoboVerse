import math
import json
import os
import numpy as np


def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def angle_diff_deg(a, b):
    return abs(normalize_angle_deg(a - b))


class LocalMapper:
    """
    Lightweight 2D occupancy grid around the start position.

    Grid values:
      0 = unknown
      1 = free / visited
      2 = blocked

    This version separates:
    - free ray updates
    - actual obstacle endpoint updates

    This prevents clear far-depth readings from being marked as phantom obstacles.
    """

    UNKNOWN = 0
    FREE = 1
    BLOCKED = 2

    def __init__(self, size_m=50.0, resolution=0.5):
        self.resolution = resolution
        self.grid_size = int(size_m / resolution)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        self.start_n = None
        self.start_e = None

        self.blocked_heading_memory = []

    def initialize_start(self, n, e):
        self.start_n = float(n)
        self.start_e = float(e)

    def _ned_to_grid(self, n, e):
        if self.start_n is None or self.start_e is None:
            return None, None

        offset_n = n - self.start_n
        offset_e = e - self.start_e

        row = int(self.grid_size / 2 + offset_n / self.resolution)
        col = int(self.grid_size / 2 + offset_e / self.resolution)

        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            return row, col

        return None, None

    def update_visited(self, n, e):
        row, col = self._ned_to_grid(n, e)

        if row is not None and col is not None:
            if self.grid[row, col] != self.BLOCKED:
                self.grid[row, col] = self.FREE

    def remember_blocked_heading(self, yaw_deg, ttl=8):
        self.blocked_heading_memory.append(
            {
                "yaw": normalize_angle_deg(yaw_deg),
                "ttl": ttl,
            }
        )

        self.blocked_heading_memory = self.blocked_heading_memory[-20:]

    def decay_heading_memory(self):
        updated = []

        for item in self.blocked_heading_memory:
            item["ttl"] -= 1
            if item["ttl"] > 0:
                updated.append(item)

        self.blocked_heading_memory = updated

    def update_ray(self, n, e, yaw_deg, distance_m, mark_obstacle=False):
        """
        Mark cells along a ray as free.

        If mark_obstacle=True, mark the endpoint as blocked.

        Use mark_obstacle=True only when the obstacle is actually close.
        Do not mark the endpoint as blocked for far/open readings.
        """
        if distance_m <= 0 or self.start_n is None:
            return

        yaw_rad = math.radians(yaw_deg)

        free_distance = max(0.0, distance_m - self.resolution)
        free_steps = int(free_distance / self.resolution)

        for i in range(1, free_steps + 1):
            inter_n = n + (i * self.resolution) * math.cos(yaw_rad)
            inter_e = e + (i * self.resolution) * math.sin(yaw_rad)

            row, col = self._ned_to_grid(inter_n, inter_e)

            if row is not None and col is not None:
                if self.grid[row, col] != self.BLOCKED:
                    self.grid[row, col] = self.FREE

        if mark_obstacle:
            obs_n = n + distance_m * math.cos(yaw_rad)
            obs_e = e + distance_m * math.sin(yaw_rad)

            row, col = self._ned_to_grid(obs_n, obs_e)

            if row is not None and col is not None:
                self.grid[row, col] = self.BLOCKED

    def mark_obstacle(self, n, e, yaw_deg, distance_m):
        self.update_ray(n, e, yaw_deg, distance_m, mark_obstacle=True)
        self.remember_blocked_heading(yaw_deg)

    def score_heading(self, current_n, current_e, heading_deg, current_yaw_deg, max_ray_m=5.0):
        yaw_rad = math.radians(heading_deg)
        steps = int(max_ray_m / self.resolution)

        score = 0.0

        # Mild turn penalty to reduce jitter.
        score -= 0.05 * angle_diff_deg(heading_deg, current_yaw_deg)

        # Avoid recently blocked headings.
        for item in self.blocked_heading_memory:
            if angle_diff_deg(heading_deg, item["yaw"]) < 25:
                score -= 40.0 * (item["ttl"] / 8.0)

        for i in range(1, steps + 1):
            check_n = current_n + (i * self.resolution) * math.cos(yaw_rad)
            check_e = current_e + (i * self.resolution) * math.sin(yaw_rad)

            row, col = self._ned_to_grid(check_n, check_e)

            if row is None or col is None:
                score -= 100.0
                break

            value = self.grid[row, col]

            if value == self.BLOCKED:
                score -= 120.0
                break

            if value == self.UNKNOWN:
                score += 8.0

            elif value == self.FREE:
                score += 0.5

        return score

    def suggest_heading(self, current_n, current_e, current_yaw_deg, clearances=None):
        """
        Choose a heading using map memory and live clearances.

        Positive relative heading roughly means turning right.
        Negative relative heading roughly means turning left.
        """
        options = [0, 30, -30, 60, -60, 90, -90, 135, -135, 180]

        best_heading = current_yaw_deg
        best_score = -1e9

        for rel_heading in options:
            heading = normalize_angle_deg(current_yaw_deg + rel_heading)
            score = self.score_heading(current_n, current_e, heading, current_yaw_deg)

            if clearances is not None:
                if rel_heading == 0:
                    score += min(clearances.get("center", 0.0), 6.0) * 4.0
                elif rel_heading > 0:
                    score += min(clearances.get("right", 0.0), 6.0) * 3.0
                else:
                    score += min(clearances.get("left", 0.0), 6.0) * 3.0

            if score > best_score:
                best_score = score
                best_heading = heading

        self.decay_heading_memory()
        return best_heading

    def save(self, log_dir="logs", start_time_str=""):
        os.makedirs(log_dir, exist_ok=True)

        grid_path = os.path.join(log_dir, f"occupancy_grid_{start_time_str}.npy")
        meta_path = os.path.join(log_dir, f"occupancy_meta_{start_time_str}.json")

        np.save(grid_path, self.grid)

        with open(meta_path, "w") as f:
            json.dump(
                {
                    "resolution": self.resolution,
                    "grid_size": self.grid_size,
                    "start_n": self.start_n,
                    "start_e": self.start_e,
                },
                f,
                indent=4,
            )

        print(f"Occupancy grid saved to {grid_path}")
        print(f"Occupancy metadata saved to {meta_path}")
