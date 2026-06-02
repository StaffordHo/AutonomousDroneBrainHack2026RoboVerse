#!/usr/bin/env python3
import argparse
import csv
import math
import os
import re
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ARENA_M = 40.0


POSE_IN_NAME_RE = re.compile(
    r"_n(?P<n>[+-]\d+(?:\.\d+)?)"
    r"_e(?P<e>[+-]\d+(?:\.\d+)?)"
    r"_d(?P<d>[+-]\d+(?:\.\d+)?)"
    r"_yaw(?P<yaw>[+-]\d+(?:\.\d+)?)"
    r"_(?P<idx>\d+)\.jpg$"
)
LOG_TIME_RE = re.compile(r"\[(?P<t>\d+(?:\.\d+)?)\]")
DEPTH_RE = re.compile(r"Depth velocity (?P<state>\S+) L/C/R/min=")
DANGER_RE = re.compile(r"Depth hazard repeated near N=(?P<n>[+-]?\d+(?:\.\d+)?) E=(?P<e>[+-]?\d+(?:\.\d+)?)")
OUTSIDE_RE = re.compile(r"outside hard arena bounds N=(?P<n>[+-]?\d+(?:\.\d+)?) E=(?P<e>[+-]?\d+(?:\.\d+)?)")
DANGER_STATE_RE = re.compile(r"N=(?P<n>[+-]?\d+(?:\.\d+)?) E=(?P<e>[+-]?\d+(?:\.\d+)?)")


def parse_pose_from_name(path: Path):
    match = POSE_IN_NAME_RE.search(path.name)
    if not match:
        return None
    return {
        "t": None,
        "n": float(match.group("n")),
        "e": float(match.group("e")),
        "d": float(match.group("d")),
        "yaw": float(match.group("yaw")),
        "state": "candidate",
        "source": str(path),
        "idx": int(match.group("idx")),
    }


def parse_dataset_dir(dataset_dir: Path, pattern: str):
    samples = []
    for path in dataset_dir.glob(pattern):
        sample = parse_pose_from_name(path)
        if sample is not None:
            samples.append(sample)
    return sorted(samples, key=lambda item: item["idx"])


def parse_depth_csv(path: Path):
    samples = []
    danger_zones = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            sample = {
                "t": float(row.get("monotonic_s") or row.get("wall_time_s") or 0.0),
                "n": float(row["north_m"]),
                "e": float(row["east_m"]),
                "d": float(row["down_m"]),
                "yaw": float(row["yaw_deg"]),
                "state": row.get("state", ""),
                "left": to_float(row.get("left_m")),
                "center": to_float(row.get("center_m")),
                "right": to_float(row.get("right_m")),
                "min": to_float(row.get("min_m")),
                "source": str(path),
            }
            samples.append(sample)
            danger_state = row.get("danger_state", "")
            match = DANGER_STATE_RE.search(danger_state)
            if match:
                danger_zones.append((float(match.group("n")), float(match.group("e"))))
    return samples, unique_points(danger_zones, radius_m=0.5), []


def parse_terminal_log(path: Path):
    samples = []
    depth_events = []
    danger_zones = []
    crash_markers = []

    with path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            t_match = LOG_TIME_RE.search(line)
            t = float(t_match.group("t")) if t_match else None

            if "Saved candidate frame" in line and "path=" in line:
                img_path = Path(line.rsplit("path=", 1)[1].strip())
                sample = parse_pose_from_name(img_path)
                if sample is not None:
                    sample["t"] = t
                    sample["state"] = "candidate"
                    samples.append(sample)
                continue

            depth_match = DEPTH_RE.search(line)
            if depth_match:
                depth_events.append({"t": t, "state": depth_match.group("state")})
                continue

            danger_match = DANGER_RE.search(line)
            if danger_match:
                danger_zones.append((float(danger_match.group("n")), float(danger_match.group("e"))))
                continue

            outside_match = OUTSIDE_RE.search(line)
            if outside_match:
                crash_markers.append(
                    {
                        "t": t,
                        "n": float(outside_match.group("n")),
                        "e": float(outside_match.group("e")),
                        "state": "pose_outside",
                    }
                )
                continue

            if "local position jumped" in line:
                crash_markers.append({"t": t, "n": None, "e": None, "state": "pose_jump"})

    samples.sort(key=lambda item: item["t"] if item["t"] is not None else item.get("idx", 0))
    attach_depth_events(samples, depth_events)
    attach_crash_markers(samples, crash_markers)
    return samples, unique_points(danger_zones, radius_m=0.5), crash_markers


def attach_depth_events(samples, events):
    timed_samples = [sample for sample in samples if sample["t"] is not None]
    if not timed_samples:
        return
    for event in events:
        if event["t"] is None:
            continue
        nearest = min(timed_samples, key=lambda sample: abs(sample["t"] - event["t"]))
        if abs(nearest["t"] - event["t"]) <= 2.5:
            if nearest["state"] == "candidate":
                nearest["state"] = event["state"]


def attach_crash_markers(samples, markers):
    timed_samples = [sample for sample in samples if sample["t"] is not None]
    if not timed_samples:
        return
    for marker in markers:
        if marker["n"] is not None or marker["t"] is None:
            continue
        prior = [sample for sample in timed_samples if sample["t"] <= marker["t"]]
        if prior:
            nearest = prior[-1]
            marker["n"] = nearest["n"]
            marker["e"] = nearest["e"]


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def unique_points(points, radius_m):
    unique = []
    for n, e in points:
        if not any(math.hypot(n - old_n, e - old_e) <= radius_m for old_n, old_e in unique):
            unique.append((n, e))
    return unique


def latest_depth_csv():
    paths = sorted(Path("/tmp/ros_logs").glob("depth_velocity_path_*.csv"), key=lambda p: p.stat().st_mtime)
    return paths[-1] if paths else None


def extract_map_from_pdf(pdf_path: Path, page: int, dpi: int):
    with tempfile.TemporaryDirectory(prefix="roboverse_map_") as tmp_dir:
        prefix = Path(tmp_dir) / "page"
        subprocess.run(
            [
                "pdftoppm",
                "-png",
                "-singlefile",
                "-f",
                str(page),
                "-l",
                str(page),
                "-r",
                str(dpi),
                str(pdf_path),
                str(prefix),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        image = Image.open(prefix.with_suffix(".png")).convert("RGB")
        return crop_largest_nonwhite_component(image)


def crop_largest_nonwhite_component(image: Image.Image):
    arr = np.asarray(image)
    mask = np.any(arr < 235, axis=2).astype(np.uint8)
    try:
        import cv2

        count, _labels, stats, _centers = cv2.connectedComponentsWithStats(mask, 8)
        if count <= 1:
            return image
        largest = max(range(1, count), key=lambda idx: stats[idx, cv2.CC_STAT_AREA])
        x = int(stats[largest, cv2.CC_STAT_LEFT])
        y = int(stats[largest, cv2.CC_STAT_TOP])
        w = int(stats[largest, cv2.CC_STAT_WIDTH])
        h = int(stats[largest, cv2.CC_STAT_HEIGHT])
        pad = 4
        return image.crop((max(0, x - pad), max(0, y - pad), min(image.width, x + w + pad), min(image.height, y + h + pad)))
    except Exception:
        ys, xs = np.where(mask)
        if xs.size == 0:
            return image
        return image.crop((int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1))


def estimate_walkable_cells(map_image: Image.Image, cell_m: float):
    grid = int(round(ARENA_M / cell_m))
    if grid <= 0:
        return set()
    gray = np.asarray(map_image.convert("L"))
    h, w = gray.shape
    walkable = set()
    for e_idx in range(grid):
        for n_idx in range(grid):
            x1 = int(e_idx * w / grid)
            x2 = int((e_idx + 1) * w / grid)
            y1 = int((grid - n_idx - 1) * h / grid)
            y2 = int((grid - n_idx) * h / grid)
            cell = gray[y1:y2, x1:x2]
            if cell.size and np.mean(cell > 115) > 0.28:
                walkable.add((e_idx, n_idx))
    return walkable


def state_color(state: str):
    if "pose" in state:
        return (214, 39, 40)
    if "critical" in state:
        return (214, 39, 40)
    if "blocked" in state:
        return (255, 127, 14)
    if "danger" in state:
        return (148, 103, 189)
    if "clear" in state:
        return (44, 160, 44)
    return (31, 119, 180)


def distance_m(samples):
    total = 0.0
    last = None
    for sample in samples:
        if last is not None:
            step = math.hypot(sample["n"] - last["n"], sample["e"] - last["e"])
            if step < 8.0:
                total += step
        last = sample
    return total


def visited_cells(samples, cell_m):
    cells = set()
    grid = int(round(ARENA_M / cell_m))
    for sample in samples:
        n = sample["n"]
        e = sample["e"]
        if 0.0 <= n < ARENA_M and 0.0 <= e < ARENA_M:
            cells.add((min(grid - 1, int(e // cell_m)), min(grid - 1, int(n // cell_m))))
    return cells


def plot(samples, danger_zones, crash_markers, map_image, args):
    if not samples:
        raise SystemExit("No path samples found.")

    canvas_px = 1200
    margin_px = 80
    plot_px = canvas_px - 2 * margin_px
    canvas = Image.new("RGBA", (canvas_px, canvas_px), (255, 255, 255, 255))
    draw = ImageDraw.Draw(canvas, "RGBA")

    walkable = set()
    if map_image is not None:
        resample = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
        map_layer = map_image.resize((plot_px, plot_px), resample).convert("RGBA")
        map_layer.putalpha(145)
        canvas.alpha_composite(map_layer, (margin_px, margin_px))
        walkable = estimate_walkable_cells(map_image, args.coverage_cell_m)

    cells = visited_cells(samples, args.coverage_cell_m)
    for e_idx, n_idx in cells:
        x1, y1 = to_px(e_idx * args.coverage_cell_m, (n_idx + 1) * args.coverage_cell_m, margin_px, plot_px)
        x2, y2 = to_px((e_idx + 1) * args.coverage_cell_m, n_idx * args.coverage_cell_m, margin_px, plot_px)
        draw.rectangle((x1, y1, x2, y2), fill=(102, 194, 102, 50))

    path_points = [to_px(sample["e"], sample["n"], margin_px, plot_px) for sample in samples]
    for idx in range(1, len(samples)):
        prev = samples[idx - 1]
        cur = samples[idx]
        if math.hypot(cur["n"] - prev["n"], cur["e"] - prev["e"]) >= 8.0:
            continue
        draw.line((path_points[idx - 1], path_points[idx]), fill=(11, 79, 156, 255), width=4)

    draw_marker(draw, path_points[0], radius=9, fill=(31, 157, 85, 255), outline=(255, 255, 255, 255))
    draw_marker(draw, path_points[-1], radius=9, fill=(17, 17, 17, 255), outline=(255, 255, 255, 255))

    for sample in samples:
        state = sample.get("state", "")
        if state and state != "candidate":
            point = to_px(sample["e"], sample["n"], margin_px, plot_px)
            draw_marker(draw, point, radius=5, fill=(*state_color(state), 220))

    for n, e in danger_zones:
        center = to_px(e, n, margin_px, plot_px)
        radius_px = int(args.danger_radius_m / ARENA_M * plot_px)
        draw.ellipse(
            (
                center[0] - radius_px,
                center[1] - radius_px,
                center[0] + radius_px,
                center[1] + radius_px,
            ),
            fill=(148, 103, 189, 45),
            outline=(74, 35, 90, 210),
            width=3,
        )
        draw_cross(draw, center, size=10, fill=(74, 35, 90, 255), width=3)

    for marker in crash_markers:
        n = marker.get("n")
        e = marker.get("e")
        if n is None or e is None:
            continue
        draw_cross(draw, to_px(e, n, margin_px, plot_px), size=13, fill=(214, 39, 40, 255), width=5)

    for coord in np.arange(0, ARENA_M + 0.001, args.coverage_cell_m):
        x1, y = to_px(0.0, coord, margin_px, plot_px)
        x2, _ = to_px(ARENA_M, coord, margin_px, plot_px)
        draw.line((x1, y, x2, y), fill=(85, 85, 85, 95), width=1)
        x, y1 = to_px(coord, 0.0, margin_px, plot_px)
        _, y2 = to_px(coord, ARENA_M, margin_px, plot_px)
        draw.line((x, y1, x, y2), fill=(85, 85, 85, 95), width=1)

    draw.rectangle((margin_px, margin_px, margin_px + plot_px, margin_px + plot_px), outline=(30, 30, 30, 255), width=2)

    summary = summarize(samples, cells, walkable, args.coverage_cell_m)
    font = load_font(18)
    title_font = load_font(28)
    draw.text((margin_px, 24), "RoboVerse Depth-Velocity Path", fill=(10, 10, 10, 255), font=title_font)
    draw.text((margin_px, canvas_px - margin_px + 12), "East (m) ->", fill=(10, 10, 10, 255), font=font)
    draw.text((8, margin_px), "North (m)", fill=(10, 10, 10, 255), font=font)
    draw_summary_box(draw, summary, font, margin_px + 12, canvas_px - margin_px - 92)
    draw_legend(draw, font, canvas_px - margin_px - 220, margin_px + 12)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.convert("RGB").save(output)
    print(summary)
    print(f"Wrote {output}")


def to_px(e_m: float, n_m: float, margin_px: int, plot_px: int):
    x = margin_px + int(round((e_m / ARENA_M) * plot_px))
    y = margin_px + int(round(((ARENA_M - n_m) / ARENA_M) * plot_px))
    return x, y


def draw_marker(draw, point, radius, fill, outline=None):
    x, y = point
    bbox = (x - radius, y - radius, x + radius, y + radius)
    draw.ellipse(bbox, fill=fill, outline=outline, width=2 if outline else 1)


def draw_cross(draw, point, size, fill, width):
    x, y = point
    draw.line((x - size, y - size, x + size, y + size), fill=fill, width=width)
    draw.line((x - size, y + size, x + size, y - size), fill=fill, width=width)


def load_font(size):
    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def draw_summary_box(draw, text, font, x, y):
    lines = text.splitlines()
    width = max(draw.textlength(line, font=font) for line in lines) + 20
    height = len(lines) * 24 + 18
    draw.rectangle((x, y, x + width, y + height), fill=(255, 255, 255, 220), outline=(180, 180, 180, 255))
    cursor_y = y + 9
    for line in lines:
        draw.text((x + 10, cursor_y), line, fill=(10, 10, 10, 255), font=font)
        cursor_y += 24


def draw_legend(draw, font, x, y):
    items = [
        ("path", (11, 79, 156)),
        ("clear", state_color("clear")),
        ("blocked", state_color("blocked")),
        ("critical/pose", state_color("critical")),
        ("danger zone", state_color("danger")),
    ]
    draw.rectangle((x - 10, y - 10, x + 220, y + len(items) * 28 + 10), fill=(255, 255, 255, 220), outline=(180, 180, 180, 255))
    for idx, (label, color) in enumerate(items):
        cy = y + idx * 28 + 10
        draw_marker(draw, (x + 10, cy), radius=6, fill=(*color, 230))
        draw.text((x + 26, cy - 10), label, fill=(10, 10, 10, 255), font=font)


def summarize(samples, cells, walkable, cell_m):
    total_cells = int(round(ARENA_M / cell_m)) ** 2
    if walkable:
        denominator = len(walkable)
        coverage = len(cells & walkable) / max(1, denominator) * 100.0
        coverage_text = f"{len(cells & walkable)}/{denominator} walkable {cell_m:.0f}m cells"
    else:
        coverage = len(cells) / max(1, total_cells) * 100.0
        coverage_text = f"{len(cells)}/{total_cells} arena {cell_m:.0f}m cells"
    return (
        f"samples: {len(samples)}\n"
        f"path length: {distance_m(samples):.1f} m\n"
        f"coverage: {coverage_text} ({coverage:.1f}%)"
    )


def load_inputs(args):
    if args.csv:
        print(f"Using depth CSV: {args.csv}")
        return parse_depth_csv(Path(args.csv))
    if args.log:
        print(f"Using saved terminal log: {args.log}")
        return parse_terminal_log(Path(args.log))
    if args.dataset_dir:
        print(
            "Using dataset image filenames only. This may combine multiple runs "
            "unless --dataset-glob is limited to one timestamp."
        )
        return parse_dataset_dir(Path(args.dataset_dir), args.dataset_glob), [], []

    latest_csv = latest_depth_csv()
    if latest_csv:
        print(f"Using latest depth CSV: {latest_csv}")
        return parse_depth_csv(latest_csv)

    default_dataset = Path("datasets/fuel_barrels_v1/images/train")
    print(
        "No depth CSV found in /tmp/ros_logs; falling back to dataset image filenames. "
        "This is approximate and may combine multiple runs."
    )
    return parse_dataset_dir(default_dataset, args.dataset_glob), [], []


def load_map(args):
    if args.map_image:
        return Image.open(args.map_image).convert("RGB")
    if args.map_pdf and Path(args.map_pdf).exists():
        return extract_map_from_pdf(Path(args.map_pdf), args.map_page, args.pdf_dpi)
    return None


def main():
    parser = argparse.ArgumentParser(description="Plot a RoboVerse depth-velocity run on the top-down map.")
    parser.add_argument("--csv", help="CSV from depth_path_log_enabled.")
    parser.add_argument("--log", help="Saved terminal log from ros2 launch output.")
    parser.add_argument("--dataset-dir", help="Dataset image directory containing candidate filenames with n/e/yaw.")
    parser.add_argument("--dataset-glob", default="candidate_*.jpg")
    default_map_pdf = Path(__file__).resolve().parents[2] / "documents" / "RoboVerse 2026 Qualifier.pdf"
    parser.add_argument("--map-pdf", default=str(default_map_pdf) if default_map_pdf.exists() else "")
    parser.add_argument("--map-page", type=int, default=2)
    parser.add_argument("--map-image", help="Optional cropped top-down map image.")
    parser.add_argument("--pdf-dpi", type=int, default=160)
    parser.add_argument("--coverage-cell-m", type=float, default=4.0)
    parser.add_argument("--danger-radius-m", type=float, default=3.0)
    parser.add_argument("--output", default="/tmp/ros_logs/depth_velocity_path_plot.png")
    args = parser.parse_args()

    samples, danger_zones, crash_markers = load_inputs(args)
    map_image = load_map(args)
    plot(samples, danger_zones, crash_markers, map_image, args)


if __name__ == "__main__":
    main()
