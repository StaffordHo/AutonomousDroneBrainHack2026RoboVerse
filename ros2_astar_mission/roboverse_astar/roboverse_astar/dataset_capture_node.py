import os
import time
from datetime import datetime
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from .common import image_msg_to_bgr, pose_to_ne_down_yaw


CLASS_NAMES = ["red_fuel_barrel", "yellow_fuel_barrel"]
CLASS_IDS = {name: idx for idx, name in enumerate(CLASS_NAMES)}


class DatasetCaptureNode(Node):
    """
    Low-load image collector for RoboVerse fuel-barrel training data.

    It saves candidate frames with permissive HSV labels. Treat these as weak
    labels for fast review; inspect/correct them before training a final model.
    """

    def __init__(self):
        super().__init__("dataset_capture_node")
        self.declare_parameter("image_topic", "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image")
        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("output_dir", "datasets/fuel_barrels_v1")
        self.declare_parameter("capture_period_s", 2.0)
        self.declare_parameter("candidate_capture_period_s", 1.0)
        self.declare_parameter("process_hz", 3.0)
        self.declare_parameter("max_image_width", 640)
        self.declare_parameter("save_raw_periodic", False)
        self.declare_parameter("save_annotated", True)
        self.declare_parameter("save_crops", True)
        self.declare_parameter("max_saved_frames", 900)
        self.declare_parameter("min_blob_area_px", 80)

        self.pose: Optional[PoseStamped] = None
        self.last_process_time = 0.0
        self.last_candidate_save_time = 0.0
        self.last_raw_save_time = 0.0
        self.saved_frames = 0
        self.saved_candidates = 0

        self.output_dir = str(self.get_parameter("output_dir").value)
        self.images_dir = os.path.join(self.output_dir, "images", "train")
        self.labels_dir = os.path.join(self.output_dir, "labels", "train")
        self.annotated_dir = os.path.join(self.output_dir, "annotated")
        self.crops_dir = os.path.join(self.output_dir, "crops")
        self.raw_dir = os.path.join(self.output_dir, "raw")
        for path in (self.images_dir, self.labels_dir, self.annotated_dir, self.crops_dir, self.raw_dir):
            os.makedirs(path, exist_ok=True)
        self.write_data_yaml()
        self.ensure_metadata_header()

        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self.image_callback, 1)
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self.pose_callback, 10)

    def pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def image_callback(self, msg: Image):
        now = time.monotonic()
        period = 1.0 / max(0.2, float(self.get_parameter("process_hz").value))
        if now - self.last_process_time < period:
            return
        self.last_process_time = now

        if self.saved_frames >= int(self.get_parameter("max_saved_frames").value):
            return

        try:
            frame = self.resize_for_capture(image_msg_to_bgr(msg))
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")
            return

        detections = self.detect_hsv_candidates(frame)
        if detections:
            min_period = max(0.1, float(self.get_parameter("candidate_capture_period_s").value))
            if now - self.last_candidate_save_time >= min_period:
                self.last_candidate_save_time = now
                self.save_candidate_frame(frame, detections)
            return

        if bool(self.get_parameter("save_raw_periodic").value):
            min_period = max(0.2, float(self.get_parameter("capture_period_s").value))
            if now - self.last_raw_save_time >= min_period:
                self.last_raw_save_time = now
                self.save_raw_frame(frame)

    def resize_for_capture(self, frame):
        max_width = int(self.get_parameter("max_image_width").value)
        if max_width <= 0 or frame.shape[1] <= max_width:
            return frame
        scale = max_width / float(frame.shape[1])
        new_size = (max_width, max(1, int(frame.shape[0] * scale)))
        return cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)

    def detect_hsv_candidates(self, frame) -> List[dict]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, np.array([18, 65, 70]), np.array([40, 255, 255]))
        warm_red_or_orange = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 55, 55]), np.array([17, 255, 255])),
            cv2.inRange(hsv, np.array([165, 55, 55]), np.array([180, 255, 255])),
        )

        detections = []
        detections.extend(self.contours_to_detections(frame, yellow, "yellow_fuel_barrel", expand=(0.18, 0.18)))
        detections.extend(self.contours_to_detections(frame, warm_red_or_orange, "red_fuel_barrel", expand=(0.75, 1.30)))
        return self.nms(detections, iou_threshold=0.35)

    def contours_to_detections(self, frame, mask, label: str, expand: Tuple[float, float]) -> List[dict]:
        h, w = frame.shape[:2]
        image_area = h * w
        min_area = max(12.0, float(self.get_parameter("min_blob_area_px").value))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)

        detections = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > image_area * 0.18:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 5 or bh < 5:
                continue

            aspect = bh / max(bw, 1)
            if label == "yellow_fuel_barrel" and (aspect < 0.35 or aspect > 5.5):
                continue
            if label == "red_fuel_barrel" and (aspect < 0.12 or aspect > 6.5):
                continue

            x1, y1, x2, y2 = self.expand_box(x, y, x + bw, y + bh, w, h, expand)
            detections.append(
                {
                    "label": label,
                    "class_id": CLASS_IDS[label],
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "area": float(area),
                    "score": min(0.99, 0.35 + area / max(image_area * 0.02, 1.0)),
                }
            )
        return detections

    @staticmethod
    def expand_box(x1: int, y1: int, x2: int, y2: int, width: int, height: int, expand: Tuple[float, float]):
        bw = x2 - x1
        bh = y2 - y1
        pad_x = int(round(bw * expand[0]))
        pad_y = int(round(bh * expand[1]))
        return (
            max(0, x1 - pad_x),
            max(0, y1 - pad_y),
            min(width - 1, x2 + pad_x),
            min(height - 1, y2 + pad_y),
        )

    def save_candidate_frame(self, frame, detections: List[dict]):
        n, e, d, yaw = pose_to_ne_down_yaw(self.pose)
        stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
        stem = f"candidate_{stamp}_n{n:+05.1f}_e{e:+05.1f}_d{d:+04.1f}_yaw{yaw:+.2f}_{self.saved_frames:05d}"

        image_path = os.path.join(self.images_dir, f"{stem}.jpg")
        label_path = os.path.join(self.labels_dir, f"{stem}.txt")
        cv2.imwrite(image_path, frame)
        self.write_yolo_labels(label_path, frame.shape[1], frame.shape[0], detections)

        if bool(self.get_parameter("save_annotated").value):
            annotated = self.annotate(frame, detections)
            cv2.imwrite(os.path.join(self.annotated_dir, f"{stem}.jpg"), annotated)

        if bool(self.get_parameter("save_crops").value):
            self.save_crops(stem, frame, detections)

        self.append_metadata(stem, detections, n, e, d, yaw)
        self.saved_frames += 1
        self.saved_candidates += len(detections)
        labels = ",".join(sorted({det["label"] for det in detections}))
        self.get_logger().info(
            f"Saved candidate frame {self.saved_frames}: labels={labels} boxes={len(detections)} path={image_path}"
        )

    def save_raw_frame(self, frame):
        n, e, d, yaw = pose_to_ne_down_yaw(self.pose)
        stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
        stem = f"raw_{stamp}_n{n:+05.1f}_e{e:+05.1f}_d{d:+04.1f}_yaw{yaw:+.2f}_{self.saved_frames:05d}"
        path = os.path.join(self.raw_dir, f"{stem}.jpg")
        cv2.imwrite(path, frame)
        self.saved_frames += 1
        self.get_logger().info(f"Saved raw frame {self.saved_frames}: {path}")

    @staticmethod
    def write_yolo_labels(path: str, image_w: int, image_h: int, detections: List[dict]):
        lines = []
        for det in detections:
            x1, y1, x2, y2 = det["bbox_xyxy"]
            cx = ((x1 + x2) * 0.5) / max(image_w, 1)
            cy = ((y1 + y2) * 0.5) / max(image_h, 1)
            bw = (x2 - x1) / max(image_w, 1)
            bh = (y2 - y1) / max(image_h, 1)
            lines.append(f"{det['class_id']} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
        with open(path, "w", encoding="utf-8") as handle:
            handle.writelines(lines)

    def annotate(self, frame, detections: List[dict]):
        annotated = frame.copy()
        colours = {
            "red_fuel_barrel": (40, 80, 255),
            "yellow_fuel_barrel": (0, 220, 255),
        }
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox_xyxy"]]
            colour = colours.get(det["label"], (255, 255, 255))
            cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(
                annotated,
                det["label"].replace("_fuel_barrel", ""),
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                colour,
                1,
                cv2.LINE_AA,
            )
        return annotated

    def save_crops(self, stem: str, frame, detections: List[dict]):
        for idx, det in enumerate(detections):
            x1, y1, x2, y2 = [int(v) for v in det["bbox_xyxy"]]
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            label_dir = os.path.join(self.crops_dir, det["label"])
            os.makedirs(label_dir, exist_ok=True)
            cv2.imwrite(os.path.join(label_dir, f"{stem}_{idx:02d}.jpg"), crop)

    def write_data_yaml(self):
        path = os.path.join(self.output_dir, "data.yaml")
        content = (
            f"path: {self.output_dir}\n"
            "train: images/train\n"
            "val: images/val\n"
            f"names:\n  0: {CLASS_NAMES[0]}\n  1: {CLASS_NAMES[1]}\n"
        )
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(content)
        os.makedirs(os.path.join(self.output_dir, "images", "val"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "labels", "val"), exist_ok=True)

    def ensure_metadata_header(self):
        metadata_path = os.path.join(self.output_dir, "capture_metadata.csv")
        if os.path.exists(metadata_path):
            return
        with open(metadata_path, "w", encoding="utf-8") as handle:
            handle.write("stem,labels,boxes,north_m,east_m,down_m,yaw_rad\n")

    def append_metadata(self, stem: str, detections: List[dict], n: float, e: float, d: float, yaw: float):
        labels = "|".join(sorted({det["label"] for det in detections}))
        metadata_path = os.path.join(self.output_dir, "capture_metadata.csv")
        with open(metadata_path, "a", encoding="utf-8") as handle:
            handle.write(f"{stem},{labels},{len(detections)},{n:.3f},{e:.3f},{d:.3f},{yaw:.5f}\n")

    @staticmethod
    def nms(detections: List[dict], iou_threshold: float) -> List[dict]:
        selected = []
        for det in sorted(detections, key=lambda item: item["score"], reverse=True):
            if all(DatasetCaptureNode.box_iou(det["bbox_xyxy"], keep["bbox_xyxy"]) < iou_threshold for keep in selected):
                selected.append(det)
        return selected

    @staticmethod
    def box_iou(a, b) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        intersection = iw * ih
        area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
        area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
        union = area_a + area_b - intersection
        if union <= 0:
            return 0.0
        return intersection / union


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
