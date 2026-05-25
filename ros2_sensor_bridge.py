import threading

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    ROS2_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    Image = None
    ROS2_AVAILABLE = False


def _image_bytes_to_array(msg):
    dtype = np.uint8

    if msg.encoding in ("32FC1", "32FC"):
        dtype = np.float32
    elif msg.encoding in ("16UC1", "mono16"):
        dtype = np.uint16

    row = np.frombuffer(msg.data, dtype=dtype)

    if msg.encoding in ("rgb8", "bgr8"):
        channels = 3
    elif msg.encoding in ("rgba8", "bgra8"):
        channels = 4
    else:
        channels = 1

    if channels == 1:
        return row.reshape(msg.height, msg.width)

    return row.reshape(msg.height, msg.width, channels)


def ros_image_to_bgr(msg):
    image = _image_bytes_to_array(msg)

    if msg.encoding == "bgr8":
        return image.copy()

    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if msg.encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    if msg.encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    if msg.encoding in ("mono8", "8UC1"):
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Unsupported ROS image encoding for RGB camera: {msg.encoding}")


def ros_image_to_depth_m(msg):
    depth = _image_bytes_to_array(msg)

    if msg.encoding in ("32FC1", "32FC"):
        return depth.astype(np.float32, copy=True)

    if msg.encoding in ("16UC1", "mono16"):
        return depth.astype(np.float32) / 1000.0

    raise ValueError(f"Unsupported ROS image encoding for depth camera: {msg.encoding}")


class _SensorBridgeNode(Node):
    def __init__(self, image_topic, depth_topic, on_bgr_frame, on_depth_frame):
        super().__init__("roboverse_sensor_bridge")
        self.on_bgr_frame = on_bgr_frame
        self.on_depth_frame = on_depth_frame

        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.create_subscription(Image, depth_topic, self._depth_callback, 10)

        self.get_logger().info(
            f"ROS2 sensor bridge subscribed image={image_topic} depth={depth_topic}"
        )

    def _image_callback(self, msg):
        try:
            self.on_bgr_frame(ros_image_to_bgr(msg))
        except Exception as error:
            self.get_logger().warn(f"RGB bridge conversion failed: {error}")

    def _depth_callback(self, msg):
        try:
            self.on_depth_frame(ros_image_to_depth_m(msg))
        except Exception as error:
            self.get_logger().warn(f"Depth bridge conversion failed: {error}")


class Ros2SensorBridge:
    def __init__(self, image_topic, depth_topic, on_bgr_frame, on_depth_frame):
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.on_bgr_frame = on_bgr_frame
        self.on_depth_frame = on_depth_frame
        self.node = None
        self.thread = None

    def start(self):
        if not ROS2_AVAILABLE:
            return False

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = _SensorBridgeNode(
            self.image_topic,
            self.depth_topic,
            self.on_bgr_frame,
            self.on_depth_frame,
        )
        self.thread = threading.Thread(
            target=rclpy.spin,
            args=(self.node,),
            daemon=True,
        )
        self.thread.start()
        return True

    def stop(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None

        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()

        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
