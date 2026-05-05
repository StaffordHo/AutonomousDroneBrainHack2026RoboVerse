import time
import numpy as np
import cv2

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


IMAGE_TOPIC = "/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image"

latest_frame = None


def image_callback(msg: Image):
    global latest_frame

    width = msg.width
    height = msg.height
    pixel_format = msg.pixel_format_type

    print(f"Received image: {width}x{height}, format={pixel_format}")

    # RGB_INT8 means 3 channels: R, G, B
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((height, width, 3))

    # OpenCV uses BGR, Gazebo gives RGB
    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def main():
    global latest_frame

    node = Node()
    node.subscribe(Image, IMAGE_TOPIC, image_callback)

    print("Waiting for image frames...")

    start = time.time()

    while time.time() - start < 10:
        if latest_frame is not None:
            cv2.imwrite("camera_frame.png", latest_frame)
            print("Saved frame to camera_frame.png")
            return
        time.sleep(0.1)

    print("No image received within 10 seconds.")


if __name__ == "__main__":
    main()
