import asyncio
import os
import time
from datetime import datetime

import cv2
import numpy as np

from mavsdk import System
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from detection_logger import DetectionLogger


IMAGE_TOPIC = "/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image"

latest_frame = None
latest_detections = []

EVIDENCE_DIR = "evidence"


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

    # Yellow barrel / yellow hazard colour
    yellow_lower = np.array([18, 80, 80])
    yellow_upper = np.array([40, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    # Red wraps around HSV hue
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

    # Detect red barrels first
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in red_contours:
        area = cv2.contourArea(contour)

        x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = h / max(w, 1)

    touches_bottom = (y + h) >= frame.shape[0] - 5
    touches_side = x <= 5 or (x + w) >= frame.shape[1] - 5
    is_edge_clipped = touches_bottom or touches_side

    # Allow smaller/less vertical detections if object is clipped by image edge
    min_area = 1500 if is_edge_clipped else 3000
    min_aspect_ratio = 0.45 if is_edge_clipped else 0.8

    if area < min_area:
        continue

    if aspect_ratio < min_aspect_ratio:
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

    # Detect yellow barrels, reject yellow stickers inside red barrels
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in yellow_contours:
        area = cv2.contourArea(contour)

        x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = h / max(w, 1)
    cx = x + w // 2
    cy = y + h // 2

    touches_bottom = (y + h) >= frame.shape[0] - 5
    touches_side = x <= 5 or (x + w) >= frame.shape[1] - 5
    is_edge_clipped = touches_bottom or touches_side

    # Yellow stickers are usually small and square, so still reject tiny blobs.
    # But allow partial yellow barrels if clipped by image edge.
    min_area = 1500 if is_edge_clipped else 3000
    min_aspect_ratio = 0.45 if is_edge_clipped else 0.8

    if area < min_area:
        continue

    if aspect_ratio < min_aspect_ratio:
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


def save_evidence(frame, detections, summary, label):
    os.makedirs(EVIDENCE_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = draw_detections(frame, detections, summary)

    image_path = os.path.join(EVIDENCE_DIR, f"{timestamp}_{label}.png")
    cv2.imwrite(image_path, output)

    print(f"Saved evidence image: {image_path}")


def image_callback(msg: Image):
    global latest_frame

    width = msg.width
    height = msg.height

    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((height, width, 3))

    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


async def wait_connected(drone):
    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected.")
            return


async def wait_local_position(drone):
    print("Waiting for local position estimate...")
    async for health in drone.telemetry.health():
        print(
            f"global={health.is_global_position_ok}, "
            f"home={health.is_home_position_ok}, "
            f"local={health.is_local_position_ok}"
        )

        if health.is_local_position_ok:
            print("Local position OK.")
            return


async def perception_loop(duration_s=15):
    global latest_frame, latest_detections

    logger = DetectionLogger(center_distance_threshold=100, confirmation_frames=3)

    print(f"Running perception loop for {duration_s} seconds...")

    start_time = time.time()
    last_print_time = 0
    saved_once = False

    while time.time() - start_time < duration_s:
        if latest_frame is None:
            await asyncio.sleep(0.05)
            continue

        frame = latest_frame.copy()
        detections = detect_barrels(frame)
        latest_detections = detections

        new_confirmations = logger.update(detections)

        for confirmed in new_confirmations:
            print(
                f"NEW confirmed detection: {confirmed['colour']} barrel "
                f"at {confirmed['center']}"
            )

        summary = logger.summary()

        if new_confirmations and not saved_once:
            save_evidence(frame, detections, summary, "first_confirmations")
            saved_once = True

        now = time.time()
        if now - last_print_time > 1.0:
            print(
                f"Confirmed barrels: red={summary['red']}, "
                f"yellow={summary['yellow']}, total={summary['total']}"
            )
            last_print_time = now

        # Optional live display
        output = draw_detections(frame, detections, summary)
        cv2.imshow("Integrated Mission Detection", output)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        await asyncio.sleep(0.03)

    final_summary = logger.summary()

    if latest_frame is not None:
        save_evidence(latest_frame.copy(), latest_detections, final_summary, "final_summary")

    cv2.destroyAllWindows()

    return final_summary


async def main():
    print("Starting Gazebo camera subscriber...")
    node = Node()
    node.subscribe(Image, IMAGE_TOPIC, image_callback)

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)
    await wait_local_position(drone)

    print("Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(2.5)

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    print("Waiting for takeoff to stabilise...")
    await asyncio.sleep(8)

    summary = await perception_loop(duration_s=15)

    print("Perception summary:")
    print(summary)

    print("Landing...")
    await drone.action.land()

    print("Integrated stationary mission complete.")


if __name__ == "__main__":
    asyncio.run(main())
