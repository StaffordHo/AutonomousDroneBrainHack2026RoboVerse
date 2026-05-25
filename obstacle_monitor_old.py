import time
import numpy as np
import cv2

class ObstacleMonitor:
    def __init__(self, obstacle_distance_m=2.0):
        self.obstacle_distance_m = obstacle_distance_m
        self.latest_depth = None
        self.latest_timestamp = 0

    def update_depth(self, depth_map):
        self.latest_depth = depth_map
        self.latest_timestamp = time.time()

    def get_directional_clearance(self):
        """Returns {left, center, right} distances."""
        if self.latest_depth is None:
            return {"left": 0.0, "center": 0.0, "right": 0.0}

        # 1. Noise Filter
        depth = cv2.medianBlur(self.latest_depth.astype(np.float32), 5)
        h, w = depth.shape

        # 2. Vertical Crop (Horizon only: 20% to 50%)
        v1, v2 = int(h * 0.20), int(h * 0.50)
        horizon = depth[v1:v2, :]

        # 3. Horizontal Slices
        w_third = w // 3
        slices = {
            "left": horizon[:, :w_third],
            "center": horizon[:, w_third:2*w_third],
            "right": horizon[:, 2*w_third:]
        }

        results = {}
        for name, area in slices.items():
            valid = area[np.isfinite(area)]
            if valid.size < (area.size * 0.2):
                results[name] = 0.0 # Blind = Blocked
            else:
                # Use 20th percentile (robust middle-ground)
                results[name] = float(np.percentile(valid, 20))
        
        return results

    def obstacle_too_close(self):
        clearances = self.get_directional_clearance()
        dist = clearances["center"]
        return dist < self.obstacle_distance_m, dist
