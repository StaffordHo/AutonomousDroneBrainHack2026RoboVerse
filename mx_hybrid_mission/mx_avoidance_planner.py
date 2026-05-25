import math

import cv2
import numpy as np


class MXAvoidancePlanner:
    """
    Depth polar-histogram planner adapted from the MX mapper.

    The original MX stack uses this planner as its main motion controller. In
    this hybrid mission it is only a heading/step advisor; competition_mission
    still performs the final attitude, depth, corridor, and recovery checks.
    """

    def __init__(
        self,
        K,
        width,
        height,
        max_speed=2.9,
        safe_distance=3.2,
        critical_distance=1.4,
        num_bins=48,
        smoothing_alpha=0.65,
        max_valid_depth_m=12.0,
    ):
        self.fx = float(K[0, 0])
        self.cx = float(K[0, 2])
        self.width = int(width)
        self.height = int(height)
        self.max_speed = float(max_speed)
        self.safe_distance = float(safe_distance)
        self.critical_distance = float(critical_distance)
        self.num_bins = int(num_bins)
        self.alpha = float(smoothing_alpha)
        self.max_valid_depth_m = float(max_valid_depth_m)

        self.prev_north = None
        self.prev_east = None
        self.prev_down = None

    def _clean_depth(self, depth_map):
        depth = depth_map.astype(np.float32, copy=True)
        depth[(~np.isfinite(depth)) | (depth > self.max_valid_depth_m)] = self.max_valid_depth_m
        depth[depth < 0.05] = self.max_valid_depth_m
        return cv2.medianBlur(depth, 5)

    def _robust_distance(self, region, percentile=20, min_valid_ratio=0.05):
        if region is None or region.size == 0:
            return self.max_valid_depth_m

        valid = region[np.isfinite(region)]
        valid = valid[(valid >= 0.05) & (valid <= self.max_valid_depth_m)]
        if valid.size < region.size * min_valid_ratio:
            return self.max_valid_depth_m

        return float(np.percentile(valid, percentile))

    def pixel_to_angle(self, u):
        return math.atan((float(u) - self.cx) / max(self.fx, 1e-6))

    def compute_histogram(self, depth_map):
        """
        Return obstacle cost, camera-relative bin angles, and robust distances.

        Each bin covers a vertical strip of the depth image. The 20th percentile
        makes close structure matter while avoiding single-pixel noise.
        """
        depth = self._clean_depth(depth_map)
        h, w = depth.shape

        histogram = np.zeros(self.num_bins, dtype=np.float32)
        angles = np.zeros(self.num_bins, dtype=np.float32)
        distances = np.zeros(self.num_bins, dtype=np.float32)

        # Use the same horizon-ish band as the mission obstacle monitor, not the
        # full image, so floor returns do not dominate tight-passage selection.
        y1 = int(h * 0.22)
        y2 = int(h * 0.58)
        band = depth[y1:y2, :]

        for i in range(self.num_bins):
            x_start = int(i * w / self.num_bins)
            x_end = int((i + 1) * w / self.num_bins)
            region = band[:, x_start:x_end]

            distance = self._robust_distance(region, percentile=20)
            distances[i] = distance

            if distance <= self.critical_distance:
                cost = 1.0
            else:
                cost = np.clip(1.0 / (distance + 1e-3), 0.0, 1.0)

            histogram[i] = cost
            angles[i] = self.pixel_to_angle((x_start + x_end) / 2.0)

        return histogram, angles, distances

    def compute_clearance(self, depth_map):
        depth = self._clean_depth(depth_map)
        h, w = depth.shape
        y1 = int(h * 0.22)
        y2 = int(h * 0.58)
        band = depth[y1:y2, :]

        left = self._robust_distance(band[:, : w // 3], percentile=20)
        center = self._robust_distance(band[:, w // 3 : 2 * w // 3], percentile=20)
        right = self._robust_distance(band[:, 2 * w // 3 :], percentile=20)

        return left, center, right

    def detect_blocked(self, left, center, right):
        return (
            center < self.critical_distance
            and left < self.safe_distance
            and right < self.safe_distance
        )

    def detect_environment(self, left, center, right):
        if center > self.safe_distance and left > self.safe_distance and right > self.safe_distance:
            return "OPEN"
        if center > self.safe_distance:
            return "FORWARD_CLEAR"
        if left > right:
            return "LEFT_OPEN"
        return "RIGHT_OPEN"

    def choose_histogram_heading(
        self,
        depth_map,
        current_yaw_deg,
        score_fn,
        preferred_heading_deg=None,
        max_deviation_deg=90.0,
        min_bin_distance_m=2.2,
        preferred_penalty=0.08,
        distance_bonus=0.65,
        cost_penalty=20.0,
    ):
        """
        Pick a camera-forward heading using MX's full image polar histogram.

        score_fn(candidate_heading_deg) should return a mission-memory score.
        The method then adds depth distance reward and obstacle cost penalty.
        """
        histogram, angles, distances = self.compute_histogram(depth_map)
        current_yaw_deg = normalize_angle_deg(current_yaw_deg)

        best = None

        for cost, angle_rad, distance in zip(histogram, angles, distances):
            if not np.isfinite(distance) or distance < min_bin_distance_m:
                continue

            relative_deg = math.degrees(float(angle_rad))
            if abs(relative_deg) > max_deviation_deg:
                continue

            candidate_heading = normalize_angle_deg(current_yaw_deg + relative_deg)
            score = float(score_fn(candidate_heading))
            score += min(float(distance), self.max_valid_depth_m) * distance_bonus
            score -= float(cost) * cost_penalty

            if preferred_heading_deg is not None:
                score -= angle_diff_deg(candidate_heading, preferred_heading_deg) * preferred_penalty

            item = {
                "heading": candidate_heading,
                "score": score,
                "distance": float(distance),
                "cost": float(cost),
                "relative_deg": relative_deg,
            }

            if best is None or item["score"] > best["score"]:
                best = item

        return best

    def smooth_position(self, north, east, down):
        if self.prev_north is None:
            self.prev_north = north
            self.prev_east = east
            self.prev_down = down
            return north, east, down

        north_s = self.alpha * self.prev_north + (1.0 - self.alpha) * north
        east_s = self.alpha * self.prev_east + (1.0 - self.alpha) * east
        down_s = self.alpha * self.prev_down + (1.0 - self.alpha) * down

        self.prev_north = north_s
        self.prev_east = east_s
        self.prev_down = down_s

        return north_s, east_s, down_s


def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def angle_diff_deg(a, b):
    return abs(normalize_angle_deg(a - b))
