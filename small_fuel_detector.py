import cv2
import numpy as np
import math


IMAGE_PATH = "actual_targets_test.png"
OUTPUT_PATH = "small_fuel_detection_result.png"


def clean_mask(mask, kernel_size=3):
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def reject_large_decorative_barrel(x, y, w, h, area, image_area):
    bbox_area = w * h

    if bbox_area > image_area * 0.08:
        return True

    if area > image_area * 0.04:
        return True

    return False


def compute_solidity(contour):
    """Solidity = contour area / convex hull area. Solid objects ~0.8+."""
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area < 1:
        return 0.0
    return cv2.contourArea(contour) / hull_area


def is_reasonable_small_target(x, y, w, h, area, image_area, contour=None):
    bbox_area = w * h

    # Reject tiny noise artifacts.
    if w < 8:
        return False

    if h < 10:
        return False

    if area < 50:
        return False

    # Reject large decorative barrels / large coloured structures.
    if w > 85:
        return False

    if h > 95:
        return False

    if bbox_area > 7000:
        return False

    if area > 4500:
        return False

    aspect_ratio = h / max(w, 1)

    # Allow slightly squarish for distant barrels, but reject very flat.
    if aspect_ratio < 0.65 or aspect_ratio > 4.5:
        return False

    # Reject horizontal stripes (ladder rungs are wide + thin).
    if w > 2.5 * h:
        return False

    fill_ratio = area / max(bbox_area, 1)

    if fill_ratio < 0.08:
        return False

    # Solidity check: actual fuel barrels are solid cylinders.
    # Ladder rungs, thin edges, and irregular shapes have low solidity.
    if contour is not None:
        solidity = compute_solidity(contour)
        if solidity < 0.35:
            return False

    return True

def center_distance(det_a, det_b):
    ax, ay = det_a["center"]
    bx, by = det_b["center"]
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def merge_nearby_detections(detections, distance_threshold=75):
    merged = []

    for det in detections:
        matched = None

        for existing in merged:
            if existing["colour"] != det["colour"]:
                continue

            if center_distance(existing, det) <= distance_threshold:
                matched = existing
                break

        if matched is None:
            merged.append(det.copy())
        else:
            x1, y1, w1, h1 = matched["bbox"]
            x2, y2, w2, h2 = det["bbox"]

            left = min(x1, x2)
            top = min(y1, y2)
            right = max(x1 + w1, x2 + w2)
            bottom = max(y1 + h1, y2 + h2)

            new_w = right - left
            new_h = bottom - top

            matched["bbox"] = (left, top, new_w, new_h)
            matched["center"] = (left + new_w // 2, top + new_h // 2)
            matched["area"] += det["area"]
            matched["fill_ratio"] = matched["area"] / max(new_w * new_h, 1)
            matched["merged_parts"] = matched.get("merged_parts", 1) + 1

    return merged


def detect_small_fuel_barrels(frame):
    image_h, image_w = frame.shape[:2]
    image_area = image_h * image_w

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow_lower = np.array([18, 70, 80])
    yellow_upper = np.array([42, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    red_lower_1 = np.array([0, 60, 60])
    red_upper_1 = np.array([12, 255, 255])

    red_lower_2 = np.array([170, 60, 60])
    red_upper_2 = np.array([180, 255, 255])

    # Tightened to avoid pink/magenta ladder paint.
    orange_red_lower = np.array([8, 80, 100])
    orange_red_upper = np.array([18, 255, 255])

    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    red_orange_mask = cv2.inRange(hsv, orange_red_lower, orange_red_upper)

    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
    red_mask = cv2.bitwise_or(red_mask, red_orange_mask)

    yellow_mask = clean_mask(yellow_mask, kernel_size=3)
    red_mask = clean_mask(red_mask, kernel_size=3)

    raw_detections = []

    for colour_name, mask in [
        ("yellow", yellow_mask),
        ("red", red_mask),
    ]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)

            if not is_reasonable_small_target(x, y, w, h, area, image_area, contour=contour):
                continue

            cx = x + w // 2
            cy = y + h // 2

            if colour_name == "yellow":
                # Reject tiny yellow lights/reflections.
                if cy < image_h * 0.30:
                    continue

            if colour_name == "red":
                # Red fuel barrels are expected to be elevated / inside compartments.
                # Reject very low red detections from big barrels/floor artifacts.
                if cy > image_h * 0.88:
                    continue

            raw_detections.append({
                "colour": colour_name,
                "area": float(area),
                "bbox": (x, y, w, h),
                "center": (cx, cy),
                "fill_ratio": float(area / max(w * h, 1)),
                "merged_parts": 1,
            })

    merged_detections = merge_nearby_detections(raw_detections, distance_threshold=75)

    merged_detections.sort(key=lambda d: (d["colour"], -d["area"]))

    return merged_detections, yellow_mask, red_mask, raw_detections


def draw_detections(frame, detections, raw_detections=None):
    output = frame.copy()

    # Draw raw detections lightly as small circles for debugging.
    if raw_detections is not None:
        for raw in raw_detections:
            cx, cy = raw["center"]
            colour = raw["colour"]
            dot_colour = (0, 0, 180) if colour == "red" else (0, 180, 180)
            cv2.circle(output, (cx, cy), 3, dot_colour, -1)

    for det in detections:
        x, y, w, h = det["bbox"]
        cx, cy = det["center"]
        colour = det["colour"]

        if colour == "red":
            box_colour = (0, 0, 255)
        else:
            box_colour = (0, 255, 255)

        cv2.rectangle(output, (x, y), (x + w, y + h), box_colour, 2)
        cv2.circle(output, (cx, cy), 5, box_colour, -1)

        label = f"{colour} fuel"
        if det.get("merged_parts", 1) > 1:
            label += f" x{det['merged_parts']}"

        cv2.putText(
            output,
            label,
            (x, max(y - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            box_colour,
            2,
        )

    cv2.putText(
        output,
        f"small fuel detections: {len(detections)}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
    )

    return output


def main():
    frame = cv2.imread(IMAGE_PATH)

    if frame is None:
        raise FileNotFoundError(
            f"Could not read {IMAGE_PATH}. "
            "Make sure actual_targets_test.png is in ~/roboverse_qualifier."
        )

    detections, yellow_mask, red_mask, raw_detections = detect_small_fuel_barrels(frame)
    output = draw_detections(frame, detections, raw_detections)

    cv2.imwrite(OUTPUT_PATH, output)
    cv2.imwrite("small_fuel_yellow_mask.png", yellow_mask)
    cv2.imwrite("small_fuel_red_mask.png", red_mask)

    print(f"Saved detection result to {OUTPUT_PATH}")
    print("Saved masks:")
    print("  small_fuel_yellow_mask.png")
    print("  small_fuel_red_mask.png")
    print()
    print(f"Raw detections before merge: {len(raw_detections)}")
    for det in raw_detections:
        print("RAW:", det)

    print()
    print(f"Merged detections after merge: {len(detections)}")
    for det in detections:
        print("MERGED:", det)


if __name__ == "__main__":
    main()
