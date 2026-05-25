import csv
import json
import os
from datetime import datetime


class TrajectoryLogger:
    """
    Logs drone trajectory, depth clearances, and decision/action labels.

    Saves:
    - logs/trajectory_<timestamp>.csv
    - logs/trajectory_<timestamp>.json
    """

    def __init__(self, log_dir="logs"):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)

        self.start_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"trajectory_{self.start_time_str}.csv")
        self.json_path = os.path.join(self.log_dir, f"trajectory_{self.start_time_str}.json")

        self.records = []

    def log_state(
        self,
        north_m,
        east_m,
        down_m,
        yaw_deg,
        clearances=None,
        action_taken="",
        pitch_deg=None,
        roll_deg=None,
        distance_from_start_m=None,
    ):
        if clearances is None:
            clearances = {}

        record = {
            "timestamp": datetime.now().isoformat(),
            "north_m": float(north_m) if north_m is not None else 0.0,
            "east_m": float(east_m) if east_m is not None else 0.0,
            "down_m": float(down_m) if down_m is not None else 0.0,
            "yaw_deg": float(yaw_deg) if yaw_deg is not None else 0.0,
            "pitch_deg": None if pitch_deg is None else float(pitch_deg),
            "roll_deg": None if roll_deg is None else float(roll_deg),
            "distance_from_start_m": None if distance_from_start_m is None else float(distance_from_start_m),
            "clearance_left_m": float(clearances.get("left", -1.0)),
            "clearance_center_m": float(clearances.get("center", -1.0)),
            "clearance_right_m": float(clearances.get("right", -1.0)),
            "clearance_lower_center_m": float(clearances.get("lower_center", -1.0)),
            "action_taken": str(action_taken),
        }

        self.records.append(record)

    def save(self):
        os.makedirs(self.log_dir, exist_ok=True)

        with open(self.json_path, "w") as f:
            json.dump(self.records, f, indent=2)

        if self.records:
            with open(self.csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.records[0].keys()))
                writer.writeheader()
                writer.writerows(self.records)
        else:
            with open(self.csv_path, "w", newline="") as f:
                f.write("")

        print(f"Trajectory JSON saved to {self.json_path}")
        print(f"Trajectory CSV saved to {self.csv_path}")

        return self.csv_path, self.json_path
