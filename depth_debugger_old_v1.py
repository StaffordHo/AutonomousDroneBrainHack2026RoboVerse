import os
from datetime import datetime
import cv2
import numpy as np


def save_depth_debug(depth_map, label="depth"):
    if depth_map is None:
        return None

    os.makedirs("competition_evidence", exist_ok=True)

    depth = depth_map.copy()
    depth[~np.isfinite(depth)] = 0.0

    display = np.clip(depth, 0, 10)
    display = (display / 10.0 * 255).astype(np.uint8)
    colour = cv2.applyColorMap(255 - display, cv2.COLORMAP_JET)

    h, w = depth.shape
    centre = depth[h // 2, w // 2]

    cv2.putText(
        colour,
        f"{label} centre={centre:.2f}m",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
    )

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = f"competition_evidence/depth_{label}_{ts}.png"
    cv2.imwrite(path, colour)
    return path
