import json
import math
import os
from typing import List, Optional

os.environ.setdefault("CUDA_VISIBLE_DEVICES", "-1")

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import image_msg_to_bgr, image_msg_to_depth, pose_to_ne_down_yaw

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - optional runtime dependency
    YOLO = None


class FuelDetectorNode(Node):
    """
    Publishes red/yellow fuel barrel detections as JSON.

    Preferred mode is YOLO. If no model is available, it falls back to a simple
    HSV detector so the rest of the ROS2 navigation stack can still be tested.
    """

    def __init__(self):
        super().__init__("fuel_detector_node")

        self.declare_parameter("image_topic", "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image")
        self.declare_parameter("depth_topic", "/depth_camera")
        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("detections_topic", "/roboverse/fuel_detections")
        self.declare_parameter("model_path", "Codes/yolov8s_roboverse.pt")
        self.declare_parameter("confidence", 0.52)
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("camera_hfov_deg", 69.0)
        self.declare_parameter("standoff_m", 2.0)
        self.declare_parameter("max_depth_m", 10.0)

        self.latest_depth: Optional[np.ndarray] = None
        self.pose: Optional[PoseStamped] = None
        self.model = None

        model_path = str(self.get_parameter("model_path").value)
        if YOLO is not None and model_path:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f"Loaded YOLO model: {model_path}")
            except Exception as exc:
                self.get_logger().warn(f"Could not load YOLO model '{model_path}': {exc}. Falling back to HSV.")
        else:
            self.get_logger().warn("Ultralytics is unavailable or model_path is empty; using HSV fallback.")

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            5,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self.depth_callback,
            5,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )
        self.det_pub = self.create_publisher(
            String,
            str(self.get_parameter("detections_topic").value),
            10,
        )

    def pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def depth_callback(self, msg: Image):
        try:
            depth = image_msg_to_depth(msg)
            depth[(~np.isfinite(depth)) | (depth <= 0.05)] = np.nan
            self.latest_depth = depth
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")

    def image_callback(self, msg: Image):
        try:
            frame = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        detections = self.detect_yolo(frame) if self.model is not None else self.detect_hsv(frame)
        if not detections:
            return

        h, w = frame.shape[:2]
        pose_info = pose_to_ne_down_yaw(self.pose)
        for det in detections:
            enriched = self.enrich_detection(det, w, h, pose_info)
            self.det_pub.publish(String(data=json.dumps(enriched)))

    def detect_yolo(self, frame) -> List[dict]:
        conf = float(self.get_parameter("confidence").value)
        imgsz = int(self.get_parameter("imgsz").value)
        device = str(self.get_parameter("device").value)
        results = self.model.predict(frame, conf=conf, imgsz=imgsz, device=device, verbose=False)
        detections = []

        for result in results:
            names = result.names or getattr(self.model, "names", {})
            for box in result.boxes:
                cls_id = int(box.cls[0])
                name = str(names.get(cls_id, cls_id)).lower()
                label = self.normalize_label(name)
                if label is None:
                    continue
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                detections.append(
                    {
                        "source": "yolo",
                        "label": label,
                        "confidence": float(box.conf[0]),
                        "bbox_xyxy": [x1, y1, x2, y2],
                    }
                )
        return detections

    def detect_hsv(self, frame) -> List[dict]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = {
            "yellow_fuel_barrel": cv2.inRange(hsv, np.array([18, 70, 70]), np.array([38, 255, 255])),
            "red_fuel_barrel": cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0, 80, 60]), np.array([12, 255, 255])),
                cv2.inRange(hsv, np.array([165, 70, 60]), np.array([180, 255, 255])),
            ),
        }
        detections = []
        image_area = frame.shape[0] * frame.shape[1]

        for label, mask in masks.items():
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 45 or area > image_area * 0.12:
                    continue
                x, y, bw, bh = cv2.boundingRect(contour)
                if bh <= 4 or bw <= 4:
                    continue
                aspect = bh / max(bw, 1)
                if aspect < 0.55 or aspect > 5.5:
                    continue
                detections.append(
                    {
                        "source": "hsv_fallback",
                        "label": label,
                        "confidence": min(0.70, 0.35 + area / 5000.0),
                        "bbox_xyxy": [float(x), float(y), float(x + bw), float(y + bh)],
                    }
                )
        return detections

    def enrich_detection(self, det: dict, image_w: int, image_h: int, pose_info):
        north, east, down, yaw_rad = pose_info
        x1, y1, x2, y2 = det["bbox_xyxy"]
        cx = 0.5 * (x1 + x2)
        norm_x = (cx - image_w * 0.5) / max(image_w * 0.5, 1.0)
        bearing_deg = norm_x * float(self.get_parameter("camera_hfov_deg").value) * 0.5

        depth_m = self.sample_depth(det["bbox_xyxy"], image_w, image_h)
        target_n = None
        target_e = None
        visit_n = None
        visit_e = None
        if depth_m is not None and self.pose is not None:
            heading = yaw_rad + math.radians(bearing_deg)
            distance = min(depth_m, float(self.get_parameter("max_depth_m").value))
            target_n = north + distance * math.cos(heading)
            target_e = east + distance * math.sin(heading)
            standoff = float(self.get_parameter("standoff_m").value)
            visit_distance = max(0.8, distance - standoff)
            visit_n = north + visit_distance * math.cos(heading)
            visit_e = east + visit_distance * math.sin(heading)

        det.update(
            {
                "bearing_deg": bearing_deg,
                "depth_m": depth_m,
                "vehicle_north_m": north,
                "vehicle_east_m": east,
                "vehicle_down_m": down,
                "vehicle_yaw_deg": math.degrees(yaw_rad),
                "target_north_m": target_n,
                "target_east_m": target_e,
                "visit_north_m": visit_n,
                "visit_east_m": visit_e,
            }
        )
        return det

    def sample_depth(self, bbox_xyxy, image_w: int, image_h: int):
        if self.latest_depth is None:
            return None
        depth_h, depth_w = self.latest_depth.shape[:2]
        x1, y1, x2, y2 = bbox_xyxy
        sx1 = int((x1 + 0.20 * (x2 - x1)) * depth_w / max(image_w, 1))
        sx2 = int((x1 + 0.80 * (x2 - x1)) * depth_w / max(image_w, 1))
        sy1 = int((y1 + 0.20 * (y2 - y1)) * depth_h / max(image_h, 1))
        sy2 = int((y1 + 0.80 * (y2 - y1)) * depth_h / max(image_h, 1))
        sx1, sx2 = max(0, sx1), min(depth_w, sx2)
        sy1, sy2 = max(0, sy1), min(depth_h, sy2)
        if sx2 <= sx1 or sy2 <= sy1:
            return None
        crop = self.latest_depth[sy1:sy2, sx1:sx2]
        valid = crop[np.isfinite(crop)]
        valid = valid[(valid > 0.2) & (valid < float(self.get_parameter("max_depth_m").value))]
        if valid.size < 5:
            return None
        return float(np.median(valid))

    @staticmethod
    def normalize_label(name: str):
        if "yellow" in name:
            return "yellow_fuel_barrel"
        if "red" in name:
            return "red_fuel_barrel"
        return None


def main(args=None):
    rclpy.init(args=args)
    node = FuelDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
