import cv2
import numpy as np


IMAGE_PATH = "actual_targets_test.png"
OUTPUT_PATH = "small_fuel_detection_result.png"


def clean_mask(mask, kernel_size=3):
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def reject_large_decorative_barrel(x, y, w, h, area, image_area):
    """
    Rejects the large decorative barrels we previously detected.
    The actual qualifier fuel barrels are visually much smaller.
    """
    bbox_area = w * h

    if bbox_area > image_area * 0.08:
        return True

    if area > image_area * 0.04:
        return True

    return False


def is_reasonable_small_target(x, y, w, h, area, image_area):
    """
    General compact-object filter for small fuel barrels.
    Allows small distant barrels and medium closer barrels.
    """
    bbox_area = w * h

    if bbox_area < 30:
        return False

    if area < 15:
        return False

    if reject_large_decorative_barrel(x, y, w, h, area, image_area):
        return False

    aspect_ratio = h / max(w, 1)

    # Small barrels may be viewed from different angles, so keep this loose.
    if aspect_ratio < 0.35 or aspect_ratio > 4.5:
        return False

    # Avoid very long thin strips from wall lights / markings.
    fill_ratio = area / max(bbox_area, 1)
    if fill_ratio < 0.08:
        return False

    return True

def merge_nearby_detections(detections, distance_threshold=60):
    merged = []

    for det in detections:
        matched = None

        for existing in merged:
            if existing["colour"] != det["colour"]:
                continue

            ex, ey = existing["center"]
            dx, dy = det["center"]

            distance = ((ex - dx) ** 2 + (ey - dy) ** 2) ** 0.5

            if distance < distance_threshold:
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

    return merged

def detect_small_fuel_barrels(frame):
    image_h, image_w = frame.shape[:2]
    image_area = image_h * image_w

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Yellow fuel barrel.
    # Tuned to include bright yellow barrels under strong Gazebo lighting.
    yellow_lower = np.array([18, 70, 80])
    yellow_upper = np.array([42, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    # Red / orange-red fuel barrel.
    # In your screenshot the red barrel looks orange-ish due to lighting,
    # so include both red hue wrap-around and orange-red.
    red_lower_1 = np.array([0, 60, 60])
    red_upper_1 = np.array([12, 255, 255])

    red_lower_2 = np.array([170, 60, 60])
    red_upper_2 = np.array([180, 255, 255])

    orange_red_lower = np.array([8, 50, 80])
    orange_red_upper = np.array([24, 255, 255])

    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    red_orange_mask = cv2.inRange(hsv, orange_red_lower, orange_red_upper)

    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
    red_mask = cv2.bitwise_or(red_mask, red_orange_mask)

    yellow_mask = clean_mask(yellow_mask, kernel_size=3)
    red_mask = clean_mask(red_mask, kernel_size=3)

    detections = []

    for colour_name, mask in [
        ("yellow", yellow_mask),
        ("red", red_mask),
    ]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)

            if not is_reasonable_small_target(x, y, w, h, area, image_area):
                continue

            cx = x + w // 2
            cy = y + h // 2

            # Context rule based on qualifier:
            # yellow barrels are ground-level, red barrels are not ground-level.
            #
            # Keep these rules loose for now because camera pitch/altitude changes
            # the apparent y-position.
            if colour_name == "yellow":
                # Reject tiny yellow wall lights near the top half.
                if cy < image_h * 0.35 and area < 800:
                    continue

            if colour_name == "red":
                # Reject tiny red/orange noise near floor if any appears.
                # Red targets are expected to be elevated.
                if cy > image_h * 0.92 and area < 1000:
                    continue

            detections.append({
                "colour": colour_name,
                "area": float(area),
                "bbox": (x, y, w, h),
                "center": (cx, cy),
                "fill_ratio": float(area / max(w * h, 1)),
            })

    # Sort by colour then area descending for easier reading
    detections.sort(key=lambda d: (d["colour"], -d["area"]))

    return detections, yellow_mask, red_mask


def draw_detections(frame, detections):
    output = frame.copy()

    for det in detections:
        x, y, w, h = det["bbox"]
        cx, cy = det["center"]
        colour = det["colour"]

        if colour == "red":
            box_colour = (0, 0, 255)
        else:
            box_colour = (0, 255, 255)

        cv2.rectangle(output, (x, y), (x + w, y + h), box_colour, 2)
        cv2.circle(output, (cx, cy), 4, box_colour, -1)

        label = f"{colour} fuel"
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

    detections, yellow_mask, red_mask = detect_small_fuel_barrels(frame)
    output = draw_detections(frame, detections)

    cv2.imwrite(OUTPUT_PATH, output)
    cv2.imwrite("small_fuel_yellow_mask.png", yellow_mask)
    cv2.imwrite("small_fuel_red_mask.png", red_mask)

    print(f"Saved detection result to {OUTPUT_PATH}")
    print("Saved masks:")
    print("  small_fuel_yellow_mask.png")
    print("  small_fuel_red_mask.png")
    print()
    print("Detections:")

    for det in detections:
        print(det)


if __name__ == "__main__":
    main()
