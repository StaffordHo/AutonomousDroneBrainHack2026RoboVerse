import math
import time


def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def angle_diff_deg(a, b):
    return abs(normalize_angle_deg(a - b))


def circular_mean_deg(values):
    if not values:
        return 0.0

    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)

    return normalize_angle_deg(math.degrees(math.atan2(s, c)))


class TargetMemory:
    """
    Tracks possible fuel-barrel detections over time.

    Important:
    - A colour blob is NOT immediately counted as score.
    - A target is only confirmed after it is seen repeatedly, from enough
      yaw/viewpoint variation, with sensible depth and consistent location.

    This helps reject:
    - large decorative barrels
    - ladder rails
    - wall accents
    - one-frame colour flashes
    """

    def __init__(
        self,
        min_confidence=0.60,
        min_depth_m=0.35,
        max_depth_m=9.0,
        min_confirm_count=4,
        min_confirm_age_s=0.8,
        min_yaw_span_deg=8.0,
        duplicate_distance_m=0.9,
        duplicate_bearing_deg=5.5,
        stale_candidate_s=8.0,
    ):
        self.min_confidence = min_confidence
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.min_confirm_count = min_confirm_count
        self.min_confirm_age_s = min_confirm_age_s
        self.min_yaw_span_deg = min_yaw_span_deg
        self.duplicate_distance_m = duplicate_distance_m
        self.duplicate_bearing_deg = duplicate_bearing_deg
        self.stale_candidate_s = stale_candidate_s

        self.candidates = []
        self.confirmed = []

    def _valid_detection(self, det):
        if det.get("confidence", 0.0) < self.min_confidence:
            return False

        depth_m = det.get("depth_m")

        if depth_m is None:
            return False

        if not (self.min_depth_m <= depth_m <= self.max_depth_m):
            return False

        if "bearing_deg" not in det:
            return False

        return True

    @staticmethod
    def _avg_xy(values_n, values_e):
        return (
            sum(values_n) / max(len(values_n), 1),
            sum(values_e) / max(len(values_e), 1),
        )

    def _same_target(self, entry, det):
        if entry["colour"] != det["colour"]:
            return False

        tn = det.get("target_n")
        te = det.get("target_e")

        if entry["target_n_list"] and tn is not None and te is not None:
            an, ae = self._avg_xy(entry["target_n_list"], entry["target_e_list"])
            if math.hypot(an - tn, ae - te) <= self.duplicate_distance_m:
                return True

        bearing = det.get("bearing_deg")
        if bearing is None:
            return False

        mean_bearing = circular_mean_deg(entry["bearings"])
        if angle_diff_deg(mean_bearing, bearing) > self.duplicate_bearing_deg:
            return False

        depth = det.get("depth_m")
        depths = entry.get("depths", [])

        if depth is None or not depths:
            return True

        avg_depth = sum(depths) / len(depths)
        return abs(avg_depth - depth) <= 2.0

    def _yaw_span(self, candidate):
        yaws = candidate["observer_yaws"]

        if len(yaws) < 2:
            return 0.0

        mean_yaw = circular_mean_deg(yaws)
        return max(angle_diff_deg(y, mean_yaw) for y in yaws) * 2.0

    def _depth_stable(self, candidate):
        depths = candidate["depths"]

        if len(depths) < 2:
            return True

        avg_depth = sum(depths) / len(depths)
        max_err = max(abs(d - avg_depth) for d in depths)

        # Loose threshold because depth bbox sampling is approximate.
        return max_err <= 1.5

    def _already_confirmed(self, det):
        for confirmed in self.confirmed:
            if self._same_target(confirmed, det):
                return True

        return False

    def _candidate_ready(self, candidate):
        age_s = candidate["last_seen"] - candidate["first_seen"]

        if candidate["count"] < self.min_confirm_count:
            return False

        if age_s < self.min_confirm_age_s:
            return False

        if self._yaw_span(candidate) < self.min_yaw_span_deg:
            return False

        if not self._depth_stable(candidate):
            return False

        return True

    def _make_confirmed(self, candidate):
        confirmed = candidate.copy()
        confirmed["confirmed_time"] = time.time()
        confirmed["mean_bearing_deg"] = circular_mean_deg(candidate["bearings"])

        if candidate["target_n_list"] and candidate["target_e_list"]:
            tn, te = self._avg_xy(candidate["target_n_list"], candidate["target_e_list"])
            confirmed["mean_target_n"] = tn
            confirmed["mean_target_e"] = te

        return confirmed

    def prune_stale_candidates(self):
        now = time.time()

        self.candidates = [
            candidate
            for candidate in self.candidates
            if now - candidate["last_seen"] <= self.stale_candidate_s
        ]

    def add_detection(self, det, observer_yaw_deg):
        """
        Add a possible target detection.

        Returns:
            confirmed target dict if newly confirmed, otherwise None.
        """
        self.prune_stale_candidates()

        if not self._valid_detection(det):
            return None

        if self._already_confirmed(det):
            return None

        for candidate in self.candidates:
            if self._same_target(candidate, det):
                candidate["count"] += 1
                candidate["last_seen"] = time.time()
                candidate["bearings"].append(det["bearing_deg"])
                candidate["depths"].append(det["depth_m"])
                candidate["observer_yaws"].append(normalize_angle_deg(observer_yaw_deg))

                if det.get("target_n") is not None and det.get("target_e") is not None:
                    candidate["target_n_list"].append(det["target_n"])
                    candidate["target_e_list"].append(det["target_e"])

                if det.get("confidence", 0.0) >= candidate.get("confidence", 0.0):
                    candidate["bbox"] = det.get("bbox", candidate.get("bbox"))
                    candidate["center"] = det.get("center", candidate.get("center"))
                    candidate["area"] = det.get("area", candidate.get("area"))
                    candidate["confidence"] = det.get("confidence", candidate.get("confidence", 0.0))
                    candidate["best_detection"] = det.copy()

                if self._candidate_ready(candidate):
                    confirmed = self._make_confirmed(candidate)
                    self.confirmed.append(confirmed)
                    self.candidates.remove(candidate)
                    return confirmed

                return None

        now = time.time()

        new_candidate = {
            "colour": det["colour"],
            "bbox": det.get("bbox"),
            "center": det.get("center"),
            "area": det.get("area", 0.0),
            "confidence": det.get("confidence", 0.0),
            "bearings": [det["bearing_deg"]],
            "depths": [det["depth_m"]],
            "observer_yaws": [normalize_angle_deg(observer_yaw_deg)],
            "target_n_list": [det["target_n"]] if det.get("target_n") is not None else [],
            "target_e_list": [det["target_e"]] if det.get("target_e") is not None else [],
            "count": 1,
            "first_seen": now,
            "last_seen": now,
            "best_detection": det.copy(),
        }

        self.candidates.append(new_candidate)
        return None

    def get_best_unconfirmed_candidate(self, allowed_colours=None):
        """
        Returns the most promising unconfirmed candidate for active investigation.
        """
        self.prune_stale_candidates()

        candidates = self.candidates

        if allowed_colours is not None:
            candidates = [c for c in candidates if c["colour"] in allowed_colours]

        if not candidates:
            return None

        def score(candidate):
            yaw_span = self._yaw_span(candidate)
            return (
                candidate["count"] * 2.0
                + candidate.get("confidence", 0.0) * 3.0
                + min(yaw_span / 10.0, 2.0)
            )

        return max(candidates, key=score)

    def summary(self):
        red = sum(1 for item in self.confirmed if item["colour"] == "red")
        yellow = sum(1 for item in self.confirmed if item["colour"] == "yellow")

        return {
            "red": red,
            "yellow": yellow,
            "total": red + yellow,
            "confirmed": self.confirmed,
            "candidates": self.candidates,
        }
