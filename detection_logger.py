import time
import math


class DetectionLogger:
    def __init__(
        self,
        center_distance_threshold=100,
        confirmation_frames=3,
        same_x_threshold=180,
        vertical_duplicate_threshold=260,
    ):
        self.center_distance_threshold = center_distance_threshold
        self.confirmation_frames = confirmation_frames
        self.same_x_threshold = same_x_threshold
        self.vertical_duplicate_threshold = vertical_duplicate_threshold

        self.candidates = []
        self.confirmed = []

    def _distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _bbox_bottom(self, det):
        x, y, w, h = det["bbox"]
        return y + h

    def _bbox_top(self, det):
        x, y, w, h = det["bbox"]
        return y

    def _is_same_detection(self, det_a, det_b):
        if det_a["colour"] != det_b["colour"]:
            return False

        # Normal case: centres are close.
        if self._distance(det_a["center"], det_b["center"]) < self.center_distance_threshold:
            return True

        ax, ay = det_a["center"]
        bx, by = det_b["center"]

        same_x_column = abs(ax - bx) < self.same_x_threshold
        vertical_near = abs(ay - by) < self.vertical_duplicate_threshold

        # Important for bottom-clipped barrels:
        # If two same-colour detections are in almost the same x-column,
        # treat them as the same barrel even if one is lower in the image.
        if same_x_column and vertical_near:
            return True

        return False

    def update(self, detections):
        new_confirmations = []

        # Sort larger detections first so main barrel boxes are registered before fragments.
        detections = sorted(detections, key=lambda d: d.get("area", 0), reverse=True)

        for det in detections:
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
                    "count": 1,
                    "first_seen": time.time(),
                    "last_seen": time.time(),
                }
                self.candidates.append(candidate)
            else:
                matched_candidate["count"] += 1

                # Keep the larger bbox/area as the representative detection.
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
