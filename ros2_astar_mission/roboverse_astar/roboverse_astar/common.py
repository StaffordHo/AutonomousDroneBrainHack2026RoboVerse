import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Quaternion


def normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose(frame_id: str, stamp, north: float, east: float, down: float, yaw_rad: float = 0.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(north)
    pose.pose.position.y = float(east)
    pose.pose.position.z = float(down)
    pose.pose.orientation = yaw_to_quaternion(yaw_rad)
    return pose


@dataclass
class GridSpec:
    size_m: float = 40.0
    resolution_m: float = 0.5
    origin_north_m: float = -20.0
    origin_east_m: float = -20.0

    @property
    def width(self) -> int:
        return int(round(self.size_m / self.resolution_m))

    @property
    def height(self) -> int:
        return int(round(self.size_m / self.resolution_m))

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def world_to_grid(self, north: float, east: float) -> Tuple[int, int]:
        gx = int(math.floor((north - self.origin_north_m) / self.resolution_m))
        gy = int(math.floor((east - self.origin_east_m) / self.resolution_m))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        north = self.origin_north_m + (gx + 0.5) * self.resolution_m
        east = self.origin_east_m + (gy + 0.5) * self.resolution_m
        return north, east


def line_cells(x0: int, y0: int, x1: int, y1: int) -> Iterable[Tuple[int, int]]:
    """Bresenham cells from start to end, inclusive."""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def pose_to_ne_down_yaw(pose: Optional[PoseStamped]) -> Tuple[float, float, float, float]:
    if pose is None:
        return 0.0, 0.0, -1.5, 0.0
    return (
        float(pose.pose.position.x),
        float(pose.pose.position.y),
        float(pose.pose.position.z),
        quaternion_to_yaw(pose.pose.orientation),
    )


def image_msg_to_array(msg, force_depth=False):
    """
    Convert sensor_msgs/Image without cv_bridge.

    cv_bridge in this ROS Humble install was built against NumPy 1.x and crashes
    with NumPy 2.x. This decoder covers the encodings produced by ros_gz_bridge
    for the RoboVerse IMX214 RGB stream and `/depth_camera`.
    """
    import numpy as np

    encoding = (msg.encoding or "").lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)

    if force_depth or encoding in ("32fc1", "32fc"):
        return _reshape_image(msg, np.float32, 1)

    if encoding in ("16uc1", "mono16"):
        return _reshape_image(msg, np.uint16, 1)

    if encoding in ("mono8", "8uc1"):
        return _reshape_image(msg, np.uint8, 1)

    if encoding in ("rgb8", "bgr8", "8uc3"):
        return _reshape_image(msg, np.uint8, 3)

    if encoding in ("rgba8", "bgra8", "8uc4"):
        return _reshape_image(msg, np.uint8, 4)

    # ros_gz_bridge occasionally leaves encoding sparse. Infer from row step.
    bytes_per_pixel = step // max(width, 1)
    if force_depth or bytes_per_pixel == 4:
        return _reshape_image(msg, np.float32, 1)
    if bytes_per_pixel == 3:
        return _reshape_image(msg, np.uint8, 3)
    if bytes_per_pixel == 1:
        return _reshape_image(msg, np.uint8, 1)

    raise ValueError(f"Unsupported image encoding={msg.encoding!r} step={step} width={width} height={height}")


def image_msg_to_depth(msg):
    import numpy as np

    encoding = (msg.encoding or "").lower()
    arr = image_msg_to_array(msg, force_depth=(encoding in ("", "32fc1", "32fc")))
    if arr.dtype == np.uint16:
        return arr.astype(np.float32) * 0.001
    return arr.astype(np.float32, copy=False)


def image_msg_to_bgr(msg):
    import cv2

    encoding = (msg.encoding or "").lower()
    arr = image_msg_to_array(msg, force_depth=False)

    if arr.ndim == 2:
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    if encoding == "rgb8" or encoding == "8uc3" or encoding == "":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return arr
    if encoding == "rgba8" or encoding == "8uc4":
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)

    # Sensible fallback for 3-channel images with nonstandard labels.
    if arr.shape[-1] == 3:
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if arr.shape[-1] == 4:
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)

    raise ValueError(f"Cannot convert image encoding={msg.encoding!r} to BGR")


def _reshape_image(msg, dtype, channels):
    import numpy as np

    dtype = np.dtype(dtype)
    wire_dtype = dtype
    if dtype.itemsize > 1:
        wire_dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")

    raw = np.frombuffer(msg.data, dtype=wire_dtype)
    if dtype.itemsize > 1 and wire_dtype.byteorder not in ("=", "|"):
        raw = raw.astype(dtype, copy=False)

    height = int(msg.height)
    width = int(msg.width)
    row_items = int(msg.step) // dtype.itemsize

    if channels == 1:
        if raw.size < height * row_items:
            raise ValueError("Image data is shorter than expected")
        return raw.reshape(height, row_items)[:, :width].copy()

    row_pixels = row_items // channels
    if raw.size < height * row_pixels * channels:
        raise ValueError("Image data is shorter than expected")
    return raw.reshape(height, row_pixels, channels)[:, :width, :].copy()
