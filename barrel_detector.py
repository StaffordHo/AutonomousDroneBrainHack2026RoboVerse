import cv2
import numpy as np


IMAGE_PATH = "camera_frame.png"
OUTPUT_PATH = "barrel_detection_result.png"


def point_inside_box(point, box):
    px, py = point
    x, y, w, h = box
    return x <= px <= x + w and y <= py <= y + h


def find_colour_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def clean_mask(mask):
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_barrels(image_path: str):
    frame = cv2.imread(image_path)

    if frame is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Yellow threshold
    yellow_lower = np.array([18, 80, 80])
    yellow_upper = np.array([40, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    # Red wraps around HSV hue, so use two ranges
    red_lower_1 = np.array([0, 80, 60])
    red_upper_1 = np.array([10, 255, 255])
    red_lower_2 = np.array([170, 80, 60])
    red_upper_2 = np.array([180, 255, 255])

    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    yellow_mask = clean_mask(yellow_mask)
    red_mask = clean_mask(red_mask)

    detections = []

    # ---------------------------------------------------------
    # 1. Detect red barrels first
    # ---------------------------------------------------------
    red_boxes = []

    for contour in find_colour_contours(red_mask):
        area = cv2.contourArea(contour)

        if area < 10000:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = h / max(w, 1)

        if aspect_ratio < 1.0:
            continue

        cx = x + w // 2
        cy = y + h // 2

        red_boxes.append((x, y, w, h))

        detections.append({
            "colour": "red",
            "area": area,
            "bbox": (x, y, w, h),
            "center": (cx, cy),
        })

    # ---------------------------------------------------------
    # 2. Detect yellow barrels, while rejecting yellow stickers
    # ---------------------------------------------------------
    for contour in find_colour_contours(yellow_mask):
        area = cv2.contourArea(contour)

        # Reject small yellow hazard stickers
        if area < 15000:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = h / max(w, 1)
        cx = x + w // 2
        cy = y + h // 2

        # Real barrels are more vertical than square stickers
        if aspect_ratio < 1.3:
            continue

        # Reject yellow blobs whose centres are inside red barrel boxes
        if any(point_inside_box((cx, cy), red_box) for red_box in red_boxes):
            continue

        detections.append({
            "colour": "yellow",
            "area": area,
            "bbox": (x, y, w, h),
            "center": (cx, cy),
        })

    # ---------------------------------------------------------
    # 3. Draw final detections
    # ---------------------------------------------------------
    for det in detections:
        x, y, w, h = det["bbox"]
        cx, cy = det["center"]
        colour = det["colour"]

        if colour == "red":
            box_colour = (0, 0, 255)
        else:
            box_colour = (0, 255, 255)

        cv2.rectangle(frame, (x, y), (x + w, y + h), box_colour, 3)
        cv2.circle(frame, (cx, cy), 6, box_colour, -1)
        cv2.putText(
            frame,
            f"{colour} barrel",
            (x, max(y - 10, 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            box_colour,
            2,
        )

    cv2.imwrite(OUTPUT_PATH, frame)

    print(f"Saved detection result to {OUTPUT_PATH}")
    print("Detections:")

    for det in detections:
        print(det)


if __name__ == "__main__":
    detect_barrels(IMAGE_PATH)
