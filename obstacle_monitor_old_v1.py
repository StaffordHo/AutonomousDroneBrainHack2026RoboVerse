import time
import cv2
import numpy as np


class ObstacleMonitor:
    """
    Depth-camera utility for RoboVerse.

    Assumption:
    - /depth_camera publishes gz.msgs.Image with R_FLOAT32 depth values in metres.
    - The depth camera is forward-facing.
    """

    def __init__(
        self,
        obstacle_distance_m=1.6,
        warning_distance_m=2.5,
        max_valid_depth_m=12.0,
    ):
        self.obstacle_distance_m = obstacle_distance_m
        self.warning_distance_m = warning_distance_m
        self.max_valid_depth_m = max_valid_depth_m

        self.latest_depth = None
        self.latest_timestamp = 0.0

    def update_depth(self, depth_map):
        self.latest_depth = depth_map.astype(np.float32, copy=False)
        self.latest_timestamp = time.time()

    def has_recent_depth(self, timeout_s=1.0):
        return self.latest_depth is not None and (time.time() - self.latest_timestamp) < timeout_s

    def _clean_depth(self):
        if self.latest_depth is None:
            return None

        depth = self.latest_depth.copy()
        depth[~np.isfinite(depth)] = 0.0
        depth[(depth < 0.05) | (depth > self.max_valid_depth_m)] = 0.0

        # Median filtering reduces Gazebo speckle noise.
        return cv2.medianBlur(depth, 5)

    @staticmethod
    def _robust_distance(region, min_valid_ratio=0.12, percentile=20):
        if region is None or region.size == 0:
            return 0.0

        valid = region[np.isfinite(region)]
        valid = valid[(valid > 0.05) & (valid < 12.0)]

        if valid.size < region.size * min_valid_ratio:
            # Treat mostly-blind regions as unsafe.
            return 0.0

        return float(np.percentile(valid, percentile))

    def get_directional_clearance(self):
        """
        Returns approximate clearances for regions of the forward depth image.

        front/left/right use a horizon crop.
        lower_front is useful to catch low obstacles that the horizon crop may miss.
        """
        depth = self._clean_depth()
        if depth is None:
            return {
                "left": 0.0,
                "center": 0.0,
                "right": 0.0,
                "lower_center": 0.0,
            }

        h, w = depth.shape

        # Horizon crop: avoids ceiling/floor dominance.
        y1, y2 = int(h * 0.20), int(h * 0.55)
        horizon = depth[y1:y2, :]

        third = w // 3
        left = horizon[:, :third]
        center = horizon[:, third:2 * third]
        right = horizon[:, 2 * third:]

        # Lower-front crop catches low barrels/walls but may see floor, so it is secondary.
        ly1, ly2 = int(h * 0.50), int(h * 0.75)
        lx1, lx2 = int(w * 0.35), int(w * 0.65)
        lower_center = depth[ly1:ly2, lx1:lx2]

        return {
            "left": self._robust_distance(left),
            "center": self._robust_distance(center),
            "right": self._robust_distance(right),
            "lower_center": self._robust_distance(lower_center, min_valid_ratio=0.08, percentile=15),
        }

    def obstacle_too_close(self):
        c = self.get_directional_clearance()
        front = c["center"]

        # If a low obstacle is much closer than the horizon crop, respect it.
        lower = c["lower_center"]
        if lower > 0:
            front = min(front if front > 0 else lower, lower)

        return front < self.obstacle_distance_m, front

    def sample_depth_for_rgb_bbox(self, bbox, rgb_shape):
        """
        Estimate depth for an RGB detection bbox by scaling RGB coordinates to depth resolution.
        Returns median depth in the centre part of the bbox.
        """
        if self.latest_depth is None or rgb_shape is None:
            return None

        rgb_h, rgb_w = rgb_shape[:2]
        depth_h, depth_w = self.latest_depth.shape[:2]

        x, y, w, h = bbox

        # Use centre 60% of bbox to avoid border/background.
        cx1 = x + int(0.20 * w)
        cy1 = y + int(0.20 * h)
        cx2 = x + int(0.80 * w)
        cy2 = y + int(0.80 * h)

        dx1 = int(cx1 * depth_w / max(rgb_w, 1))
        dx2 = int(cx2 * depth_w / max(rgb_w, 1))
        dy1 = int(cy1 * depth_h / max(rgb_h, 1))
        dy2 = int(cy2 * depth_h / max(rgb_h, 1))

        dx1 = max(0, min(depth_w - 1, dx1))
        dx2 = max(0, min(depth_w, dx2))
        dy1 = max(0, min(depth_h - 1, dy1))
        dy2 = max(0, min(depth_h, dy2))

        if dx2 <= dx1 or dy2 <= dy1:
            return None

        region = self.latest_depth[dy1:dy2, dx1:dx2]
        valid = region[np.isfinite(region)]
        valid = valid[(valid > 0.05) & (valid < self.max_valid_depth_m)]

        if valid.size < 5:
            return None

        return float(np.median(valid))
