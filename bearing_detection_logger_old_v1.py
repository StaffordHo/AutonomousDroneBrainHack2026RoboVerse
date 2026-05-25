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
    Deduplicates detections and only returns a new confirmation once.

    Preferred matching:
    1. target_n / target_e distance, if target localisation is available.
    2. bearing angle fallback.
    """

    def __init__(
        self,
        confirmation_frames=4,
        dist_threshold_m=1.4,
        bearing_threshold_deg=8.0,
        min_confidence=0.0,
    ):
        self.confirmation_frames = confirmation_frames
        self.dist_threshold_m = dist_threshold_m
        self.bearing_threshold_deg = bearing_threshold_deg
        self.min_confidence = min_confidence

        self.candidates = []
        self.confirmed = []

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
            dist = math.hypot(an - tn, ae - te)
            return dist < self.dist_threshold_m

        if "bearing_deg" not in det:
            return False

        avg_bearing = circular_mean_deg(entry["bearings"])
        return angle_diff_deg(avg_bearing, det["bearing_deg"]) < self.bearing_threshold_deg

    def add_detection(self, det):
        """
        Returns confirmed detection dict only on first-time confirmation.
        Otherwise returns None.
        """
        if det.get("confidence", 1.0) < self.min_confidence:
            return None

        if "bearing_deg" not in det and det.get("target_n") is None:
            return None

        # Ignore if it matches an already-confirmed object.
        for c in self.confirmed:
            if self._same(c, det):
                return None

        for cand in self.candidates:
            if self._same(cand, det):
                cand["count"] += 1
                cand["last_seen"] = time.time()
                cand["bearings"].append(det.get("bearing_deg", circular_mean_deg(cand["bearings"])))

                if det.get("target_n") is not None and det.get("target_e") is not None:
                    cand["target_n_list"].append(det["target_n"])
                    cand["target_e_list"].append(det["target_e"])

                # Keep best visual bbox for evidence.
                if det.get("confidence", 0) >= cand.get("confidence", 0):
                    cand["bbox"] = det.get("bbox", cand.get("bbox"))
                    cand["center"] = det.get("center", cand.get("center"))
                    cand["area"] = det.get("area", cand.get("area"))
                    cand["confidence"] = det.get("confidence", cand.get("confidence", 0))

                if cand["count"] >= self.confirmation_frames and not cand["confirmed"]:
                    cand["confirmed"] = True
                    confirmed = cand.copy()
                    confirmed["confirmed_time"] = time.time()
                    self.confirmed.append(confirmed)
                    self.candidates.remove(cand)
                    return confirmed

                return None

        self.candidates.append({
            "colour": det["colour"],
            "bbox": det.get("bbox"),
            "center": det.get("center"),
            "area": det.get("area", 0),
            "confidence": det.get("confidence", 0),
            "bearings": [det.get("bearing_deg", 0.0)],
            "target_n_list": [det["target_n"]] if det.get("target_n") is not None else [],
            "target_e_list": [det["target_e"]] if det.get("target_e") is not None else [],
            "count": 1,
            "confirmed": False,
            "first_seen": time.time(),
            "last_seen": time.time(),
        })
        return None

    def summary(self):
        red = sum(1 for d in self.confirmed if d["colour"] == "red")
        yellow = sum(1 for d in self.confirmed if d["colour"] == "yellow")
        return {
            "red": red,
            "yellow": yellow,
            "total": red + yellow,
            "confirmed": self.confirmed,
        }
