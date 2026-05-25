import cv2
import numpy as np


def _morph(mask):
    kernel3 = np.ones((3, 3), np.uint8)
    kernel5 = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)
    return mask


def _large_decorative_bboxes(mask):
    """
    Large decorative barrels are huge same-colour components.
    We reject small candidates embedded in or very near these regions.
    """
    bboxes = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)

        if area > 12000 or (w > 120 and h > 80):
            bboxes.append((x, y, w, h, area))

    return bboxes


def _inside_padded_bbox(candidate_bbox, large_bboxes, pad=15):
    x, y, w, h = candidate_bbox
    cx = x + w // 2
    cy = y + h // 2

    for bx, by, bw, bh, _ in large_bboxes:
        if (bx - pad) <= cx <= (bx + bw + pad) and (by - pad) <= cy <= (by + bh + pad):
            return True

    return False


def _standalone_candidate(candidate_bbox, contour_area, mask):
    """
    Reject tiny patches attached to a much larger same-colour region.
    """
    x, y, w, h = candidate_bbox
    img_h, img_w = mask.shape[:2]

    pad = 35
    x1 = max(0, x - pad)
    y1 = max(0, y - pad)
    x2 = min(img_w, x + w + pad)
    y2 = min(img_h, y + h + pad)

    local = mask[y1:y2, x1:x2]
    contours, _ = cv2.findContours(local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False

    largest = max(cv2.contourArea(c) for c in contours)

    # If local colour mass is much bigger than the candidate, this is probably a scratch/patch.
    if largest > contour_area * 7.0 and largest > 900:
        return False

    return True


def _candidate_score(area, w, h, y, img_h):
    # Lightweight ranking only; this is not a real ML confidence.
    aspect_h_over_w = h / max(w, 1)
    size_score = min(area / 1000.0, 1.0)
    aspect_score = max(0.0, 1.0 - abs(aspect_h_over_w - 2.0) / 3.0)
    centre_score = 1.0 if 0.05 * img_h < y < 0.95 * img_h else 0.5
    return float(0.45 * size_score + 0.40 * aspect_score + 0.15 * centre_score)


def detect_small_fuel_barrels(frame):
    """
    HSV-based stopgap detector for small red/yellow fuel canisters.

    This is intentionally conservative. It should prefer missing a target over
    scoring false positives on the huge decorative barrels.
    Long-term replacement: YOLO detector trained on RoboVerse screenshots.
    """
    if frame is None:
        return [], None, None, []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img_h, img_w = frame.shape[:2]

    # Red wrap-around mask.
    red1 = cv2.inRange(hsv, np.array([0, 140, 55]), np.array([10, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([170, 140, 55]), np.array([180, 255, 255]))
    red_mask = _morph(cv2.bitwise_or(red1, red2))

    # Yellow mask.
    yellow_mask = _morph(cv2.inRange(hsv, np.array([18, 90, 80]), np.array([38, 255, 255])))

    # Large decorative regions in either colour can host false positives.
    decorative_bboxes = _large_decorative_bboxes(red_mask) + _large_decorative_bboxes(yellow_mask)

    detections = []
    raw = []

    for colour, mask in [("red", red_mask), ("yellow", yellow_mask)]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            bbox = (x, y, w, h)

            raw.append({
                "colour": colour,
                "bbox": bbox,
                "center": (x + w // 2, y + h // 2),
                "area": float(area),
            })

            # Reject noise and large decorative objects.
            if not (90 <= area <= 4500):
                continue
            if w < 8 or h < 14:
                continue
            if w > 110 or h > 150:
                continue

            aspect_h_over_w = h / max(w, 1)

            # Real canisters are usually compact/vertical; ladder rungs are too flat.
            if not (0.75 <= aspect_h_over_w <= 5.5):
                continue

            # Context rules from challenge: red elevated, yellow ground-level.
            # Keep this soft because viewpoint changes can move objects in image.
            if colour == "red" and y > img_h * 0.78:
                continue

            # Avoid rust/scratches at the very bottom of huge yellow barrels.
            if colour == "yellow" and (y + h) > img_h * 0.94 and h < 45:
                continue

            # Reject if candidate sits inside a huge decorative barrel region.
            if _inside_padded_bbox(bbox, decorative_bboxes, pad=12):
                continue

            if not _standalone_candidate(bbox, area, mask):
                continue

            detections.append({
                "colour": colour,
                "bbox": bbox,
                "center": (x + w // 2, y + h // 2),
                "area": float(area),
                "confidence": _candidate_score(area, w, h, y, img_h),
            })

    detections.sort(key=lambda d: d.get("confidence", 0), reverse=True)
    return detections, yellow_mask, red_mask, raw
