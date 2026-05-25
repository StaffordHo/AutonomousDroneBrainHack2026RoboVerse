import os
import time
from datetime import datetime

import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


DEPTH_TOPIC = "/depth_camera"
OUT_DIR = "depth_test_output"


latest_depth = None
latest_stamp = None


def depth_callback(msg: Image):
    global latest_depth, latest_stamp

    width = msg.width
    height = msg.height

    depth = np.frombuffer(msg.data, dtype=np.float32)

    expected_size = width * height
    if depth.size != expected_size:
        print(f"Size mismatch: got {depth.size}, expected {expected_size}")
        return

    latest_depth = depth.reshape((height, width))
    latest_stamp = time.time()


def valid_depth_values(depth):
    valid = depth[np.isfinite(depth)]
    valid = valid[(valid > 0.05) & (valid < 50.0)]
    return valid


def region_stats(name, region):
    valid = valid_depth_values(region)

    if valid.size == 0:
        return {
            "name": name,
            "valid_count": 0,
            "valid_ratio": 0.0,
            "min": None,
            "p10": None,
            "p20": None,
            "median": None,
            "mean": None,
            "max": None,
        }

    return {
        "name": name,
        "valid_count": int(valid.size),
        "valid_ratio": float(valid.size / max(region.size, 1)),
        "min": float(np.min(valid)),
        "p10": float(np.percentile(valid, 10)),
        "p20": float(np.percentile(valid, 20)),
        "median": float(np.median(valid)),
        "mean": float(np.mean(valid)),
        "max": float(np.max(valid)),
    }


def print_stats(depth):
    h, w = depth.shape

    full = depth

    # Same horizon-style crop used by obstacle monitor.
    horizon_y1 = int(h * 0.22)
    horizon_y2 = int(h * 0.55)
    horizon = depth[horizon_y1:horizon_y2, :]

    third = w // 3
    left = horizon[:, :third]
    center = horizon[:, third:2 * third]
    right = horizon[:, 2 * third:]

    # Extra regions to check whether floor/ceiling is confusing the result.
    top = depth[:int(h * 0.20), :]
    bottom = depth[int(h * 0.70):, :]
    lower_center = depth[int(h * 0.50):int(h * 0.75), int(w * 0.35):int(w * 0.65)]

    regions = [
        region_stats("full", full),
        region_stats("top", top),
        region_stats("horizon_left", left),
        region_stats("horizon_center", center),
        region_stats("horizon_right", right),
        region_stats("lower_center", lower_center),
        region_stats("bottom", bottom),
    ]

    print("\n--- Depth statistics ---")
    print(f"shape: {w}x{h}")

    for s in regions:
        if s["valid_count"] == 0:
            print(f"{s['name']:>15}: no valid depth")
        else:
            print(
                f"{s['name']:>15}: "
                f"valid={s['valid_ratio']:.2f}, "
                f"min={s['min']:.2f}, "
                f"p10={s['p10']:.2f}, "
                f"p20={s['p20']:.2f}, "
                f"median={s['median']:.2f}, "
                f"max={s['max']:.2f}"
            )


def save_depth_visual(depth, label="depth"):
    os.makedirs(OUT_DIR, exist_ok=True)

    h, w = depth.shape

    clean = depth.copy()
    clean[~np.isfinite(clean)] = 0.0
    clean[(clean < 0.05) | (clean > 20.0)] = 0.0

    display = np.clip(clean, 0, 10)
    display = (display / 10.0 * 255).astype(np.uint8)
    color = cv2.applyColorMap(255 - display, cv2.COLORMAP_JET)

    # Draw horizon crop.
    y1 = int(h * 0.22)
    y2 = int(h * 0.55)
    cv2.rectangle(color, (0, y1), (w - 1, y2), (255, 255, 255), 2)

    third = w // 3
    cv2.line(color, (third, y1), (third, y2), (255, 255, 255), 2)
    cv2.line(color, (2 * third, y1), (2 * third, y2), (255, 255, 255), 2)

    # Draw lower center crop.
    lx1 = int(w * 0.35)
    lx2 = int(w * 0.65)
    ly1 = int(h * 0.50)
    ly2 = int(h * 0.75)
    cv2.rectangle(color, (lx1, ly1), (lx2, ly2), (0, 255, 255), 2)

    center_depth = depth[h // 2, w // 2]
    cv2.putText(
        color,
        f"center raw depth: {center_depth:.2f} m",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(OUT_DIR, f"{label}_{timestamp}.png")
    cv2.imwrite(path, color)

    print(f"Saved depth visual: {path}")


def main():
    node = Node()
    node.subscribe(Image, DEPTH_TOPIC, depth_callback)

    print(f"Listening to {DEPTH_TOPIC}")
    print("Press:")
    print("  s = save depth visual")
    print("  q = quit")
    print("Statistics will print every 1 second.")

    last_print = 0.0

    while True:
        if latest_depth is None:
            print("Waiting for depth frame...")
            time.sleep(0.5)
            continue

        depth = latest_depth.copy()

        now = time.time()
        if now - last_print > 1.0:
            print_stats(depth)
            last_print = now

        # Basic visualisation window.
        clean = depth.copy()
        clean[~np.isfinite(clean)] = 0.0
        clean[(clean < 0.05) | (clean > 20.0)] = 0.0

        display = np.clip(clean, 0, 10)
        display = (display / 10.0 * 255).astype(np.uint8)
        color = cv2.applyColorMap(255 - display, cv2.COLORMAP_JET)

        h, w = depth.shape
        y1 = int(h * 0.22)
        y2 = int(h * 0.55)
        third = w // 3

        cv2.rectangle(color, (0, y1), (w - 1, y2), (255, 255, 255), 2)
        cv2.line(color, (third, y1), (third, y2), (255, 255, 255), 2)
        cv2.line(color, (2 * third, y1), (2 * third, y2), (255, 255, 255), 2)

        cv2.imshow("Depth Camera Test", color)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            save_depth_visual(depth, "manual")

        elif key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
