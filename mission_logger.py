import csv
import json
import os
from datetime import datetime


class MissionLogger:
    """
    Logs state-action data for future behaviour cloning.

    This does not control the drone.
    It only records:
    - current local NED state
    - attitude
    - depth clearances
    - selected action
    - selected/preferred heading
    - range from start
    - visited/blocked memory counts
    - target memory status

    Outputs:
    - bc_logs/mission_log_<timestamp>.csv
    - bc_logs/mission_log_<timestamp>.jsonl
    """

    def __init__(self, log_dir="bc_logs"):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)

        self.start_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"mission_log_{self.start_time_str}.csv")
        self.jsonl_path = os.path.join(self.log_dir, f"mission_log_{self.start_time_str}.jsonl")

        self.records = []

    def log(
        self,
        action_type,
        label="",
        north_m=None,
        east_m=None,
        down_m=None,
        yaw_deg=None,
        pitch_deg=None,
        roll_deg=None,
        target_down_m=None,
        range_from_start_m=None,
        preferred_heading_deg=None,
        selected_heading_deg=None,
        heading_score=None,
        clearances=None,
        visited_cells=None,
        blocked_cells=None,
        red_confirmed=0,
        yellow_confirmed=0,
        candidate_count=0,
        blocked_streak=0,
        return_fail_streak=0,
        extra=None,
    ):
        if clearances is None:
            clearances = {}

        if extra is None:
            extra = {}

        record = {
            "timestamp": datetime.now().isoformat(),
            "action_type": action_type,
            "label": label,

            "north_m": north_m,
            "east_m": east_m,
            "down_m": down_m,
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
            "roll_deg": roll_deg,

            "target_down_m": target_down_m,
            "range_from_start_m": range_from_start_m,

            "preferred_heading_deg": preferred_heading_deg,
            "selected_heading_deg": selected_heading_deg,
            "heading_score": heading_score,

            "clearance_left_m": clearances.get("left"),
            "clearance_center_m": clearances.get("center"),
            "clearance_right_m": clearances.get("right"),
            "clearance_lower_center_m": clearances.get("lower_center"),

            "visited_cells": visited_cells,
            "blocked_cells": blocked_cells,

            "red_confirmed": red_confirmed,
            "yellow_confirmed": yellow_confirmed,
            "candidate_count": candidate_count,

            "blocked_streak": blocked_streak,
            "return_fail_streak": return_fail_streak,

            "extra": json.dumps(extra),
        }

        self.records.append(record)

        with open(self.jsonl_path, "a") as f:
            f.write(json.dumps(record) + "\n")

    def save(self):
        if not self.records:
            print("MissionLogger: no records to save.")
            return None, None

        fieldnames = list(self.records[0].keys())

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        print(f"Mission log CSV saved to: {self.csv_path}")
        print(f"Mission log JSONL saved to: {self.jsonl_path}")

        return self.csv_path, self.jsonl_path
