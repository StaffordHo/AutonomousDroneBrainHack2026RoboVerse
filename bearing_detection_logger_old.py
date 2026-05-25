import math
import numpy as np

def normalize_angle_deg(angle):
    """Normalize angle to [-180, 180]"""
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

class BearingDetectionLogger:
    def __init__(self, confirmation_frames=5, dist_threshold_m=1.5, bearing_threshold_deg=8.0):
        self.confirmation_frames = confirmation_frames
        self.dist_threshold_m = dist_threshold_m
        self.bearing_threshold_deg = bearing_threshold_deg
        self.detections = [] # List of dicts: {colour, bearings[], n_list[], e_list[], confirmed}

    def _is_same_detection(self, d1, d2_colour, d2_bearing, d2_n=None, d2_e=None):
        if d1["colour"] != d2_colour:
            return False
        
        # Priority 1: NED Distance (if both have coordinates)
        if d1["n_list"] and d2_n is not None:
            avg_n = sum(d1["n_list"]) / len(d1["n_list"])
            avg_e = sum(d1["e_list"]) / len(d1["e_list"])
            dist = math.sqrt((avg_n - d2_n)**2 + (avg_e - d2_e)**2)
            return dist < self.dist_threshold_m
        
        # Priority 2: Bearing Difference
        avg_bearing = self._get_circular_average(d1["bearings"])
        diff = abs(normalize_angle_deg(avg_bearing - d2_bearing))
        return diff < self.bearing_threshold_deg

    def _get_circular_average(self, angles_deg):
        """Computes the correct average for angles (e.g., avg of 359 and 1 is 0)."""
        if not angles_deg: return 0.0
        sin_sum = sum(math.sin(math.radians(a)) for a in angles_deg)
        cos_sum = sum(math.cos(math.radians(a)) for a in angles_deg)
        return math.degrees(math.atan2(sin_sum, cos_sum))

    def add_detection(self, det):
        """det: {colour, bearing_deg, n, e (optional)}"""
        colour = det["colour"]
        bearing = det["bearing_deg"]
        n = det.get("n")
        e = det.get("e")

        for d in self.detections:
            if self._is_same_detection(d, colour, bearing, n, e):
                d["bearings"].append(bearing)
                if n is not None:
                    d["n_list"].append(n)
                    d["e_list"].append(e)
                
                if len(d["bearings"]) >= self.confirmation_frames:
                    if not d["confirmed"]:
                        d["confirmed"] = True
                        return True # First-time confirmation
                return False

        # New detection
        self.detections.append({
            "colour": colour,
            "bearings": [bearing],
            "n_list": [n] if n is not None else [],
            "e_list": [e] if e is not None else [],
            "confirmed": False
        })
        return False

    def is_confirmed(self, colour):
        for d in self.detections:
            if d["colour"] == colour and d["confirmed"]:
                return True
        return False

    def clear_confirmed(self):
        """Keep unconfirmed, remove confirmed to move to next target."""
        self.detections = [d for d in self.detections if not d["confirmed"]]
