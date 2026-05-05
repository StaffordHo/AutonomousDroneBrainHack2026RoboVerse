import time
import math


def normalize_angle_deg(angle):
    """Normalize angle to [-180, 180)."""
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def angle_diff_deg(a, b):
    """Smallest signed angular difference."""
    return abs(normalize_angle_deg(a - b))


class BearingDetectionLogger:
    def __init__(self, bearing_threshold_deg=8.0, confirmation_frames=3):
        self.bearing_threshold_deg = bearing_threshold_deg
        self.confirmation_frames = confirmation_frames

        self.candidates = []
        self.confirmed = []

    def _is_same_detection(self, det_a, det_b):
        if det_a["colour"] != det_b["colour"]:
            return False

        return angle_diff_deg(det_a["bearing_deg"], det_b["bearing_deg"]) < self.bearing_threshold_deg

    def update(self, detections):
        new_confirmations = []

        detections = sorted(detections, key=lambda d: d.get("area", 0), reverse=True)

        for det in detections:
            if "bearing_deg" not in det:
                continue

            matched_confirmed = any(
                self._is_same_detection(det, confirmed_det)
                for confirmed_det in self.confirmed
            )

            if matched_confirmed:
                continue

            matched_candidate = None

            for candidate in self.candidates:
                if self._is_same_detection(det, candidate):
                    matched_candidate = candidate
                    break

            if matched_candidate is None:
                candidate = {
                    "colour": det["colour"],
                    "center": det["center"],
                    "bbox": det["bbox"],
                    "area": det["area"],
                    "bearing_deg": det["bearing_deg"],
                    "count": 1,
                    "first_seen": time.time(),
                    "last_seen": time.time(),
                }
                self.candidates.append(candidate)
            else:
                matched_candidate["count"] += 1

                # Keep a running average bearing for stability
                matched_candidate["bearing_deg"] = normalize_angle_deg(
                    0.7 * matched_candidate["bearing_deg"] + 0.3 * det["bearing_deg"]
                )

                # Keep the larger detection as representative
                if det["area"] > matched_candidate["area"]:
                    matched_candidate["center"] = det["center"]
                    matched_candidate["bbox"] = det["bbox"]
                    matched_candidate["area"] = det["area"]

                matched_candidate["last_seen"] = time.time()

                if matched_candidate["count"] >= self.confirmation_frames:
                    confirmed = matched_candidate.copy()
                    confirmed["confirmed_time"] = time.time()

                    self.confirmed.append(confirmed)
                    new_confirmations.append(confirmed)
                    self.candidates.remove(matched_candidate)

        return new_confirmations

    def summary(self):
        red_count = sum(1 for det in self.confirmed if det["colour"] == "red")
        yellow_count = sum(1 for det in self.confirmed if det["colour"] == "yellow")

        return {
            "red": red_count,
            "yellow": yellow_count,
            "total": len(self.confirmed),
            "confirmed": self.confirmed,
        }
