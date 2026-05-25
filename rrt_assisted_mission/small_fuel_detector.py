import cv2
import numpy as np


BARREL_LABEL_BY_COLOUR = {
    "red": "red_barrel",
    "yellow": "yellow_barrel",
}


def barrel_label_for_colour(colour):
    return BARREL_LABEL_BY_COLOUR.get(colour, f"{colour}_barrel")


def _morph(mask, open_size=3, close_size=5):
    open_kernel = np.ones((open_size, open_size), np.uint8)
    close_kernel = np.ones((close_size, close_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    return mask


def _make_red_mask(hsv):
    red1 = cv2.inRange(hsv, np.array([0, 80, 60]), np.array([14, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([168, 80, 60]), np.array([180, 255, 255]))
    orange_band = cv2.inRange(hsv, np.array([8, 45, 80]), np.array([26, 255, 255]))
    return _morph(cv2.bitwise_or(cv2.bitwise_or(red1, red2), orange_band), open_size=3, close_size=5)


def _make_yellow_mask(hsv):
    return _morph(
        cv2.inRange(hsv, np.array([20, 70, 80]), np.array([42, 255, 255])),
        open_size=3,
        close_size=5,
    )


def _make_broad_red_mask(hsv):
    red1 = cv2.inRange(hsv, np.array([0, 35, 60]), np.array([28, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([165, 55, 70]), np.array([180, 255, 255]))
    return _morph(cv2.bitwise_or(red1, red2), open_size=5, close_size=11)


def _make_broad_yellow_mask(hsv):
    return _morph(
        cv2.inRange(hsv, np.array([12, 45, 45]), np.array([45, 255, 255])),
        open_size=5,
        close_size=11,
    )


def _bbox_center(bbox):
    x, y, w, h = bbox
    return (x + w // 2, y + h // 2)


def _find_large_regions(mask):
    """
    Detect large same-colour objects such as the decorative barrels.
    """
    dilated = cv2.dilate(mask, np.ones((19, 19), np.uint8), iterations=1)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    regions = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)

        if area > 9000 or (w > 90 and h > 80) or (w > 160 and h > 40):
            regions.append((x, y, w, h, area))

    return regions


def _find_ladder_regions(red_broad_mask):
    """
    Detect ladder / red rail structures so they do not become false red targets.
    """
    contours, _ = cv2.findContours(red_broad_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rung_boxes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 250:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        aspect = w / max(h, 1)

        # Thin horizontal red rungs
        if w > 50 and h < 40 and aspect > 2.0:
            rung_boxes.append((x, y, w, h))

    regions = [(x, y, w, h, 0.0) for (x, y, w, h) in rung_boxes]

    # If multiple rungs exist, also reject the whole ladder stack region
    if len(rung_boxes) >= 3:
        xs = [b[0] for b in rung_boxes]
        ys = [b[1] for b in rung_boxes]
        x2s = [b[0] + b[2] for b in rung_boxes]
        y2s = [b[1] + b[3] for b in rung_boxes]

        x1 = min(xs)
        y1 = min(ys)
        x2 = max(x2s)
        y2 = max(y2s)

        regions.append((x1, y1, x2 - x1, y2 - y1, 0.0))

    return regions


def _is_near_region(candidate_bbox, regions, pad):
    cx, cy = _bbox_center(candidate_bbox)

    for rx, ry, rw, rh, _ in regions:
        if (rx - pad) <= cx <= (rx + rw + pad) and (ry - pad) <= cy <= (ry + rh + pad):
            return True

    return False


def _standalone_candidate(candidate_bbox, contour_area, same_colour_mask):
    """
    Reject tiny candidate patches that are actually attached to a much bigger same-colour object.
    """
    x, y, w, h = candidate_bbox
    img_h, img_w = same_colour_mask.shape[:2]

    pad = 45
    x1 = max(0, x - pad)
    y1 = max(0, y - pad)
    x2 = min(img_w, x + w + pad)
    y2 = min(img_h, y + h + pad)

    local = same_colour_mask[y1:y2, x1:x2]
    local = cv2.dilate(local, np.ones((9, 9), np.uint8), iterations=1)

    contours, _ = cv2.findContours(local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    largest = max(cv2.contourArea(c) for c in contours)

    if largest > contour_area * 5.0 and largest > 700:
        return False

    return True


def _candidate_stats(contour):
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)
    bbox_area = max(w * h, 1)

    hull = cv2.convexHull(contour)
    hull_area = max(cv2.contourArea(hull), 1.0)

    fill_ratio = area / bbox_area
    solidity = area / hull_area

    return {
        "area": float(area),
        "bbox": (x, y, w, h),
        "fill_ratio": float(fill_ratio),
        "solidity": float(solidity),
    }


def _white_ratio_in_bbox(hsv, bbox):
    x, y, w, h = bbox
    roi = hsv[y:y + h, x:x + w]
    if roi.size == 0:
        return 0.0

    white = cv2.inRange(roi, np.array([0, 0, 110]), np.array([180, 90, 255]))
    return float(np.count_nonzero(white) / max(w * h, 1))


def _passes_shape_filter(colour, stats, img_w, img_h):
    area = stats["area"]
    x, y, w, h = stats["bbox"]
    fill_ratio = stats["fill_ratio"]
    solidity = stats["solidity"]

    if not (70 <= area <= 2400):
        return False

    if w < 8 or h < 16:
        return False

    if w > 62 or h > 125:
        return False

    aspect_h_over_w = h / max(w, 1)

    min_aspect = 0.85 if colour == "red" else 1.2
    if not (min_aspect <= aspect_h_over_w <= 5.8):
        return False

    if not (0.18 <= fill_ratio <= 0.92):
        return False

    if solidity < 0.30:
        return False

    cx, cy = _bbox_center(stats["bbox"])

    # Strong context priors based on your qualifier scene:
    # red = elevated, yellow = floor-level
    if colour == "red":
        if cy > img_h * 0.76:
            return False
    else:
        if cy < img_h * 0.34:
            return False

    # Very edge-hugging tiny detections are usually junk
    edge_margin = 0.005 if colour == "red" else 0.02
    if cx < img_w * edge_margin or cx > img_w * (1.0 - edge_margin):
        return False

    return True


def _candidate_confidence(colour, stats, white_ratio, img_h):
    x, y, w, h = stats["bbox"]
    area = stats["area"]

    aspect_h_over_w = h / max(w, 1)

    # size score
    size_score = min(area / 1200.0, 1.0)

    # aspect score
    ideal_aspect = 2.2
    aspect_score = max(0.0, 1.0 - abs(aspect_h_over_w - ideal_aspect) / 3.0)

    # context score
    _, cy = _bbox_center(stats["bbox"])
    if colour == "red":
        context_score = 1.0 if cy < img_h * 0.70 else 0.70
    else:
        context_score = 1.0 if cy > img_h * 0.45 else 0.60

    # white stripe / label cue
    white_score = min(white_ratio / 0.10, 1.0)

    score = float(
        0.30 * size_score
        + 0.30 * aspect_score
        + 0.25 * context_score
        + 0.15 * white_score
    )

    if colour == "red":
        score += 0.08

    return min(score, 1.0)


def detect_small_fuel_barrels(frame):
    """
    Detect only the SMALL target fuel canisters, not the large decorative barrels.

    Returns:
        detections, yellow_mask, red_mask, raw_detections
    """
    if frame is None:
        return [], None, None, []

    img_h, img_w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red_mask = _make_red_mask(hsv)
    yellow_mask = _make_yellow_mask(hsv)

    broad_red_mask = _make_broad_red_mask(hsv)
    broad_yellow_mask = _make_broad_yellow_mask(hsv)

    large_red_regions = _find_large_regions(broad_red_mask)
    large_yellow_regions = _find_large_regions(broad_yellow_mask)
    ladder_regions = _find_ladder_regions(broad_red_mask)

    all_large_regions = large_red_regions + large_yellow_regions

    detections = []
    raw_detections = []

    for colour, mask in [("red", red_mask), ("yellow", yellow_mask)]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        same_large_regions = large_red_regions if colour == "red" else large_yellow_regions

        for contour in contours:
            stats = _candidate_stats(contour)
            bbox = stats["bbox"]

            raw_detections.append(
                {
                    "colour": colour,
                    "label": barrel_label_for_colour(colour),
                    "source": "hsv",
                    "bbox": bbox,
                    "center": _bbox_center(bbox),
                    "area": stats["area"],
                    "fill_ratio": stats["fill_ratio"],
                    "solidity": stats["solidity"],
                }
            )

            if not _passes_shape_filter(colour, stats, img_w, img_h):
                continue

            if _is_near_region(bbox, same_large_regions, pad=30):
                continue

            if _is_near_region(bbox, all_large_regions, pad=6):
                continue

            if colour == "red" and _is_near_region(bbox, ladder_regions, pad=36):
                continue

            if not _standalone_candidate(bbox, stats["area"], mask):
                continue

            white_ratio = _white_ratio_in_bbox(hsv, bbox)
            confidence = _candidate_confidence(colour, stats, white_ratio, img_h)

            detections.append(
                {
                    "colour": colour,
                    "label": barrel_label_for_colour(colour),
                    "source": "hsv",
                    "bbox": bbox,
                    "center": _bbox_center(bbox),
                    "area": stats["area"],
                    "fill_ratio": stats["fill_ratio"],
                    "solidity": stats["solidity"],
                    "white_ratio": white_ratio,
                    "confidence": confidence,
                }
            )

    detections.sort(key=lambda d: d.get("confidence", 0.0), reverse=True)

    return detections, yellow_mask, red_mask, raw_detections
