import time
import cv2
import numpy as np


class ObstacleMonitor:
    """
    Depth-camera obstacle monitor for RoboVerse x500_vision.

    Uses /depth_camera as gz.msgs.Image with float32 depth values in metres.

    Important design choices:
    - Uses horizon-centre depth for forward obstacle checks.
    - Does not treat the floor as a forward obstacle.
    - Treats far/inf/no-hit depth as max range, not as a 0 m wall.
    """

    def __init__(
        self,
        obstacle_distance_m=1.55,
        warning_distance_m=2.35,
        max_valid_depth_m=12.0,
        critical_lower_distance_m=0.45,
    ):
        self.obstacle_distance_m = obstacle_distance_m
        self.warning_distance_m = warning_distance_m
        self.max_valid_depth_m = max_valid_depth_m
        self.critical_lower_distance_m = critical_lower_distance_m

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

        depth = self.latest_depth.astype(np.float32, copy=True)

        # Far / no-hit / inf should not become a near obstacle.
        depth[(~np.isfinite(depth)) | (depth > self.max_valid_depth_m)] = self.max_valid_depth_m

        # Invalid near-zero should also not become a fake wall.
        depth[depth < 0.05] = self.max_valid_depth_m

        return cv2.medianBlur(depth, 5)

    def _robust_distance(self, region, min_valid_ratio=0.08, percentile=20):
        if region is None or region.size == 0:
            return self.max_valid_depth_m

        values = region[np.isfinite(region)]
        values = values[(values >= 0.05) & (values <= self.max_valid_depth_m)]

        if values.size < region.size * min_valid_ratio:
            return self.max_valid_depth_m

        return float(np.percentile(values, percentile))

    def get_directional_clearance(self):
        """
        Returns:
            left, center, right, lower_center

        center is the main forward obstacle distance.
        lower_center is diagnostic/emergency only because it often sees the floor.
        """
        depth = self._clean_depth()

        if depth is None:
            return {
                "left": self.max_valid_depth_m,
                "center": self.max_valid_depth_m,
                "right": self.max_valid_depth_m,
                "lower_center": self.max_valid_depth_m,
            }

        h, w = depth.shape

        # Horizon crop for obstacle detection.
        y1 = int(h * 0.22)
        y2 = int(h * 0.55)
        horizon = depth[y1:y2, :]

        third = w // 3
        left = horizon[:, :third]
        center = horizon[:, third:2 * third]
        right = horizon[:, 2 * third:]

        # Narrow centre catches direct frontal obstacles.
        nx1 = int(w * 0.42)
        nx2 = int(w * 0.58)
        narrow_center = horizon[:, nx1:nx2]

        # Lower crop is mostly floor; only used for diagnostics / emergency.
        ly1 = int(h * 0.50)
        ly2 = int(h * 0.72)
        lx1 = int(w * 0.35)
        lx2 = int(w * 0.65)
        lower_center = depth[ly1:ly2, lx1:lx2]

        center_distance = min(
            self._robust_distance(center, percentile=20),
            self._robust_distance(narrow_center, min_valid_ratio=0.06, percentile=15),
        )

        return {
            "left": self._robust_distance(left, percentile=20),
            "center": center_distance,
            "right": self._robust_distance(right, percentile=20),
            "lower_center": self._robust_distance(lower_center, min_valid_ratio=0.06, percentile=15),
        }

    def obstacle_too_close(self):
        c = self.get_directional_clearance()
        front = c["center"]
        lower = c["lower_center"]

        if front < self.obstacle_distance_m:
            return True, front

        # Emergency-only lower crop rule.
        if 0.05 < lower < self.critical_lower_distance_m:
            return True, lower

        return False, front

    def obstacle_warning(self):
        c = self.get_directional_clearance()
        front = c["center"]
        return front < self.warning_distance_m, front

    def sample_depth_for_rgb_bbox(self, bbox, rgb_shape):
        """
        Estimate depth for an RGB detection bbox by scaling RGB bbox to depth image coordinates.
        Used for rough duplicate suppression.
        """
        if self.latest_depth is None or rgb_shape is None:
            return None

        rgb_h, rgb_w = rgb_shape[:2]
        depth_h, depth_w = self.latest_depth.shape[:2]

        x, y, w, h = bbox

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
