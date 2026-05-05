import time
import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from detection_logger import DetectionLogger


IMAGE_TOPIC = "/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image"

latest_frame = None


def point_inside_box(point, box):
    px, py = point
    x, y, w, h = box
    return x <= px <= x + w and y <= py <= y + h


def clean_mask(mask):
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_barrels(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    yellow_lower = np.array([18, 80, 80])
    yellow_upper = np.array([40, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

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
    red_boxes = []

    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in red_contours:
        area = cv2.contourArea(contour)

        if area < 3000:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = h / max(w, 1)

        if aspect_ratio < 0.8:
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

    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in yellow_contours:
        area = cv2.contourArea(contour)

        if area < 3000:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = h / max(w, 1)
        cx = x + w // 2
        cy = y + h // 2

        if aspect_ratio < 0.8:
            continue

        if any(point_inside_box((cx, cy), red_box) for red_box in red_boxes):
            continue

        detections.append({
            "colour": "yellow",
            "area": area,
            "bbox": (x, y, w, h),
            "center": (cx, cy),
        })

    return detections


def draw_detections(frame, detections, summary):
    output = frame.copy()

    for det in detections:
        x, y, w, h = det["bbox"]
        cx, cy = det["center"]
        colour = det["colour"]

        if colour == "red":
            box_colour = (0, 0, 255)
        else:
            box_colour = (0, 255, 255)

        cv2.rectangle(output, (x, y), (x + w, y + h), box_colour, 3)
        cv2.circle(output, (cx, cy), 5, box_colour, -1)
        cv2.putText(
            output,
            f"{colour} barrel",
            (x, max(y - 10, 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            box_colour,
            2,
        )

    cv2.putText(
        output,
        f"Confirmed: red={summary['red']} yellow={summary['yellow']} total={summary['total']}",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        3,
    )

    return output


def image_callback(msg: Image):
    global latest_frame

    width = msg.width
    height = msg.height

    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((height, width, 3))

    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def main():
    global latest_frame

    node = Node()
    node.subscribe(Image, IMAGE_TOPIC, image_callback)

    logger = DetectionLogger(center_distance_threshold=100, confirmation_frames=3)

    print("Live barrel detector with duplicate filtering running.")
    print("Press Ctrl+C in terminal to stop.")

    last_print_time = 0

    try:
        while True:
            if latest_frame is None:
                time.sleep(0.05)
                continue

            frame = latest_frame.copy()
            detections = detect_barrels(frame)

            new_confirmations = logger.update(detections)

            for confirmed in new_confirmations:
                print(
                    f"NEW confirmed detection: {confirmed['colour']} barrel "
                    f"at {confirmed['center']}"
                )

            summary = logger.summary()
            output = draw_detections(frame, detections, summary)

            cv2.imshow("RoboVerse Live Barrel Detector", output)

            now = time.time()
            if now - last_print_time > 1.0:
                print(
                    f"Confirmed barrels: red={summary['red']}, "
                    f"yellow={summary['yellow']}, total={summary['total']}"
                )
                last_print_time = now

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping detector.")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
