import time
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from obstacle_monitor import ObstacleMonitor


DEPTH_TOPIC = "/depth_camera"

monitor = ObstacleMonitor(obstacle_distance_m=1.45)


def depth_callback(msg: Image):
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    monitor.update_depth(depth)


def main():
    node = Node()
    node.subscribe(Image, DEPTH_TOPIC, depth_callback)

    print("Obstacle monitor test running.")
    print("Expected after v5 fix:")
    print("  If center is around 4.2 m and lower is around 1.2 m, too_close should be False.")
    print("Ctrl+C to stop.")

    try:
        while True:
            if monitor.latest_depth is None:
                print("Waiting for depth...")
                time.sleep(0.5)
                continue

            c = monitor.get_directional_clearance()
            too_close, front = monitor.obstacle_too_close()
            warning, warn_front = monitor.obstacle_warning()

            print(
                f"left={c['left']:.2f} m | "
                f"center={c['center']:.2f} m | "
                f"right={c['right']:.2f} m | "
                f"lower={c['lower_center']:.2f} m | "
                f"too_close={too_close} front_used={front:.2f} m | "
                f"warning={warning} warning_front={warn_front:.2f} m"
            )

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping obstacle monitor test.")


if __name__ == "__main__":
    main()
