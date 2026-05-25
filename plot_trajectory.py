import os
import glob
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


def get_latest_log_prefix(log_dir="logs"):
    if not os.path.exists(log_dir):
        return None

    json_files = glob.glob(os.path.join(log_dir, "trajectory_*.json"))

    if not json_files:
        return None

    latest_file = max(json_files, key=os.path.getmtime)
    basename = os.path.basename(latest_file)

    return basename.replace("trajectory_", "").replace(".json", "")


def get_value(record, *keys, default=0.0):
    for key in keys:
        if key in record:
            return record[key]

    return default


def plot_latest_run():
    log_dir = "logs"
    prefix = get_latest_log_prefix(log_dir)

    if not prefix:
        print("No logs found.")
        return

    print(f"Loading data for run: {prefix}")

    trajectory_path = os.path.join(log_dir, f"trajectory_{prefix}.json")
    grid_path = os.path.join(log_dir, f"occupancy_grid_{prefix}.npy")
    metadata_path = os.path.join(log_dir, f"occupancy_meta_{prefix}.json")

    with open(trajectory_path, "r") as f:
        trajectory = json.load(f)

    grid = None
    metadata = None

    if os.path.exists(grid_path) and os.path.exists(metadata_path):
        grid = np.load(grid_path)

        with open(metadata_path, "r") as f:
            metadata = json.load(f)

    fig, ax = plt.subplots(figsize=(10, 10))

    if grid is not None and metadata is not None:
        resolution = metadata["resolution"]
        size_m = metadata["grid_size"] * resolution
        start_n = metadata["start_n"]
        start_e = metadata["start_e"]

        half_size = size_m / 2.0

        extent = [
            start_e - half_size,
            start_e + half_size,
            start_n - half_size,
            start_n + half_size,
        ]

        # 0 unknown, 1 free, 2 blocked.
        cmap = ListedColormap(["#d3d3d3", "#ffffff", "#000000"])

        ax.imshow(
            np.flipud(grid),
            extent=extent,
            cmap=cmap,
            alpha=0.75,
        )

    if not trajectory:
        print("Trajectory log is empty.")
        return

    norths = [get_value(p, "north_m") for p in trajectory]
    easts = [get_value(p, "east_m") for p in trajectory]

    ax.plot(easts, norths, "b-", label="Trajectory", linewidth=2)

    decision_n = []
    decision_e = []
    blocked_n = []
    blocked_e = []

    for point in trajectory:
        action = point.get("action_taken", "")

        if action:
            decision_n.append(get_value(point, "north_m"))
            decision_e.append(get_value(point, "east_m"))

        if (
            "Blocked" in action
            or "blocked" in action
            or "abort" in action
            or "Abort" in action
        ):
            blocked_n.append(get_value(point, "north_m"))
            blocked_e.append(get_value(point, "east_m"))

    ax.scatter(
        decision_e,
        decision_n,
        s=10,
        label="Decisions/scans",
        zorder=3,
    )

    ax.scatter(
        blocked_e,
        blocked_n,
        s=35,
        marker="x",
        label="Blocked/abort",
        zorder=4,
    )

    ax.plot(easts[0], norths[0], "go", markersize=10, label="Start")
    ax.plot(easts[-1], norths[-1], "ro", markersize=10, label="End")

    ax.set_title(f"Autonomous Exploration Trajectory - {prefix}")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()
    ax.set_aspect("equal")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_latest_run()
