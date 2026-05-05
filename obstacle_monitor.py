import time
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


DEPTH_TOPIC = "/depth_camera"


class ObstacleMonitor:
    def __init__(
        self,
        obstacle_distance_m=1.5,
        warning_distance_m=2.5,
        centre_crop_ratio=0.35,
    ):
        self.obstacle_distance_m = obstacle_distance_m
        self.warning_distance_m = warning_distance_m
        self.centre_crop_ratio = centre_crop_ratio

        self.latest_depth = None
        self.latest_timestamp = None

    def depth_callback(self, msg: Image):
        width = msg.width
        height = msg.height

        # R_FLOAT32 means each pixel is one float32 depth value in metres.
        depth = np.frombuffer(msg.data, dtype=np.float32)
        depth = depth.reshape((height, width))

        self.latest_depth = depth
        self.latest_timestamp = time.time()

    def has_recent_depth(self, timeout_s=1.0):
        if self.latest_timestamp is None:
            return False

        return (time.time() - self.latest_timestamp) < timeout_s

    def get_front_distance(self):
        """
        Returns a robust distance estimate from the centre region of the depth image.
        """
        if self.latest_depth is None:
            return None

        depth = self.latest_depth

        h, w = depth.shape

        crop_w = int(w * self.centre_crop_ratio)
        crop_h = int(h * self.centre_crop_ratio)

        x1 = max((w - crop_w) // 2, 0)
        x2 = min(x1 + crop_w, w)

        y1 = max((h - crop_h) // 2, 0)
        y2 = min(y1 + crop_h, h)

        centre = depth[y1:y2, x1:x2]

        # Keep only valid finite positive depths.
        valid = centre[np.isfinite(centre)]
        valid = valid[valid > 0.05]

        if valid.size == 0:
            return None

        # Use percentile instead of absolute min to reduce noise.
        return float(np.percentile(valid, 10))

    def obstacle_too_close(self):
        distance = self.get_front_distance()

        if distance is None:
            return False, None

        return distance < self.obstacle_distance_m, distance

    def obstacle_warning(self):
        distance = self.get_front_distance()

        if distance is None:
            return False, None

        return distance < self.warning_distance_m, distance


def create_obstacle_monitor():
    node = Node()
    monitor = ObstacleMonitor()
    node.subscribe(Image, DEPTH_TOPIC, monitor.depth_callback)
    return node, monitor


if __name__ == "__main__":
    node, monitor = create_obstacle_monitor()

    print("Obstacle monitor running. Press Ctrl+C to stop.")

    try:
        while True:
            too_close, distance = monitor.obstacle_too_close()

            if distance is None:
                print("No valid depth yet.")
            else:
                print(f"Front distance: {distance:.2f} m | obstacle_too_close={too_close}")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping obstacle monitor.")
