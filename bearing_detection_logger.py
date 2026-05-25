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


class BearingDetectionLogger:
    """
    Confirms detections across multiple frames and suppresses duplicates.
    Uses local position + bearing + optional depth.
    """

    def __init__(
        self,
        red_confirmation_frames=4,
        yellow_confirmation_frames=4,
        dist_threshold_m=0.9,
        bearing_threshold_deg=5.5,
        min_confidence=0.60,
    ):
        self.red_confirmation_frames = red_confirmation_frames
        self.yellow_confirmation_frames = yellow_confirmation_frames
        self.dist_threshold_m = dist_threshold_m
        self.bearing_threshold_deg = bearing_threshold_deg
        self.min_confidence = min_confidence

        self.candidates = []
        self.confirmed = []

    def _required_frames(self, colour):
        if colour == "red":
            return self.red_confirmation_frames
        if colour == "yellow":
            return self.yellow_confirmation_frames
        return 4

    @staticmethod
    def _avg_xy(values_n, values_e):
        return (
            sum(values_n) / max(len(values_n), 1),
            sum(values_e) / max(len(values_e), 1),
        )

    def _same(self, entry, det):
        if entry["colour"] != det["colour"]:
            return False

        tn = det.get("target_n")
        te = det.get("target_e")

        if entry["target_n_list"] and tn is not None and te is not None:
            an, ae = self._avg_xy(entry["target_n_list"], entry["target_e_list"])
            return math.hypot(an - tn, ae - te) < self.dist_threshold_m

        if "bearing_deg" not in det:
            return False

        return angle_diff_deg(
            circular_mean_deg(entry["bearings"]),
            det["bearing_deg"],
        ) < self.bearing_threshold_deg

    def add_detection(self, det):
        if det.get("confidence", 0.0) < self.min_confidence:
            return None

        if "bearing_deg" not in det and det.get("target_n") is None:
            return None

        for confirmed in self.confirmed:
            if self._same(confirmed, det):
                return None

        for candidate in self.candidates:
            if self._same(candidate, det):
                candidate["count"] += 1
                candidate["last_seen"] = time.time()

                if det.get("bearing_deg") is not None:
                    candidate["bearings"].append(det["bearing_deg"])

                if det.get("target_n") is not None and det.get("target_e") is not None:
                    candidate["target_n_list"].append(det["target_n"])
                    candidate["target_e_list"].append(det["target_e"])

                if det.get("confidence", 0.0) >= candidate.get("confidence", 0.0):
                    candidate["bbox"] = det.get("bbox", candidate.get("bbox"))
                    candidate["center"] = det.get("center", candidate.get("center"))
                    candidate["area"] = det.get("area", candidate.get("area"))
                    candidate["confidence"] = det.get("confidence", candidate.get("confidence", 0.0))

                required = self._required_frames(candidate["colour"])

                if candidate["count"] >= required and not candidate["confirmed"]:
                    candidate["confirmed"] = True
                    confirmed = candidate.copy()
                    confirmed["confirmed_time"] = time.time()

                    self.confirmed.append(confirmed)
                    self.candidates.remove(candidate)

                    return confirmed

                return None

        self.candidates.append(
            {
                "colour": det["colour"],
                "bbox": det.get("bbox"),
                "center": det.get("center"),
                "area": det.get("area", 0.0),
                "confidence": det.get("confidence", 0.0),
                "bearings": [det.get("bearing_deg", 0.0)],
                "target_n_list": [det["target_n"]] if det.get("target_n") is not None else [],
                "target_e_list": [det["target_e"]] if det.get("target_e") is not None else [],
                "count": 1,
                "confirmed": False,
                "first_seen": time.time(),
                "last_seen": time.time(),
            }
        )

        return None

    def summary(self):
        red = sum(1 for item in self.confirmed if item["colour"] == "red")
        yellow = sum(1 for item in self.confirmed if item["colour"] == "yellow")

        return {
            "red": red,
            "yellow": yellow,
            "total": red + yellow,
            "confirmed": self.confirmed,
        }
