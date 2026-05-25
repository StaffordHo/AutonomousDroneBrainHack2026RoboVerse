import time
import numpy as np
import cv2
import os
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import asyncio
import queue
from datetime import datetime

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/Ultralytics")

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class GZPhotoDetectorSaver:
    def __init__(self, topic, save_dir="output", model_path="yolov8n.pt", burst_size=20, threshold=0.5):
        self.topic = topic
        self.save_dir = save_dir
        self.burst_size = burst_size
        self.threshold = threshold  # Confidence threshold for saving

        self.img_queue = queue.LifoQueue(maxsize=50)

        
        if YOLO is None:
            print("WARNING: ultralytics is not installed. Detection disabled; capture bursts still work.")
            self.model = None
        elif os.path.exists(model_path):
            print(f"Loading model: {model_path} (Threshold: {self.threshold})")
            self.model = YOLO(model_path)
        else:
            print(f"WARNING: Model file '{model_path}' not found. Detection disabled.")
            self.model = None
        
        self.is_detecting = False
        self.is_saving = False
        self.frames_remaining = 0
        self.show = False
        self.running = True
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def trigger_detection_burst(self,numofframes=30):
        if self.model:
            self.burst_size = numofframes
            self.frames_remaining = numofframes
            self.is_detecting = True
            self.is_saving = False
            print("Triggered Camera Detection Task")

    def trigger_capture_burst(self,numofframes=30):
        self.burst_size = numofframes
        self.frames_remaining = numofframes
        self.is_detecting = False
        self.is_saving = True
        print("Triggered Camera Capture Task")


    def _image_callback(self, msg: Image):
        if self.frames_remaining <= 0 or not (self.is_detecting or self.is_saving):
            return

        try:
            self.img_queue.put_nowait(msg)
        except queue.Full:
            with self.img_queue.mutex:
                self.img_queue.queue.clear()
            self.img_queue.put_nowait(msg)


    async def _worker(self):
        """The async background consumer."""
        print("Camera Background worker started.")
        while self.running:
            try:
                img = self.img_queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.02)
                continue

            await self.loop.run_in_executor(None, self._process_task, img)
            self.img_queue.task_done()

    def _process_task(self, img):
        """Blocking logic: YOLO inference and Disk I/O."""
        frame = np.frombuffer(img.data, dtype=np.uint8).reshape((img.height, img.width, 3))
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        displayframe = None
        if self.frames_remaining > 0 and (self.is_detecting or self.is_saving):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            if self.is_detecting and self.model:
                results = self.model(frame_bgr, conf=self.threshold, verbose=False)
                result = results[0] if results else None
                if result is not None and result.boxes is not None and len(result.boxes) > 0:
                    annotated = result.plot()
                    path = os.path.join(self.save_dir, f"det_{ts}.jpg")
                    cv2.imwrite(path, annotated)
                    self.show = True
                    displayframe = annotated

            elif self.is_saving:
                path = os.path.join(self.save_dir, f"raw_{ts}.jpg")
                cv2.imwrite(path, frame_bgr)
            self.frames_remaining = self.frames_remaining - 1
        else:
            if self.is_saving or self.is_detecting:
                print("Camera Task Complete")
            self.is_saving = False
            self.is_detecting = False

        if self.show and displayframe is not None:
            cv2.imshow("Gazebo Photo Booth", displayframe)
            cv2.waitKey(1)
            self.show = False

    async def run(self):
        """Entry point to start the subscription and worker."""
        self.loop = asyncio.get_running_loop()
        self.node = Node()

        if self.node.subscribe(Image, self.topic, self._image_callback):
            print(f"Subscribed to {self.topic}. No rendering (Headless).")
            await self._worker()
        else:
            print("Failed to subscribe.")
   
async def main():
    TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
    detector = GZPhotoDetectorSaver(topic=TOPIC,save_dir="output", model_path="yolov8n.pt", burst_size=20, threshold=0.5)
    # Run the detector
    await detector.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")
