import time
import numpy as np
import cv2
import os
from ultralytics import YOLO
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import asyncio
import queue

class GZPhotoDetectorSaver:
    def __init__(self, topic, save_dir="output", model_path="yolov8n.pt", burst_size=20, threshold=0.5):
        self.topic = topic
        self.save_dir = save_dir
        self.burst_size = burst_size
        self.threshold = threshold

        self.img_queue = queue.LifoQueue(maxsize=50)

        if os.path.exists(model_path):
            print(f"Loading model: {model_path} (Threshold: {self.threshold})")
            self.model = YOLO(model_path)
        else:
            print(f"WARNING: Model file '{model_path}' not found. Detection disabled.")
            self.model = None

        self.is_detecting = False
        self.is_saving = False
        self.frames_remaining = 0
        self.show = False

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def trigger_detection_burst(self, numofframes=30):
        if self.model and not self.is_detecting and not self.is_saving:  # FIX: guard
            self.burst_size = numofframes
            self.frames_remaining = numofframes
            self.is_detecting = True
            self.is_saving = False
            print("Triggered Camera Detection Task")

    def trigger_capture_burst(self, numofframes=30):
        if not self.is_detecting and not self.is_saving:  # FIX: guard
            self.burst_size = numofframes
            self.frames_remaining = numofframes
            self.is_detecting = False
            self.is_saving = True
            print("Triggered Camera Capture Task")

    def _image_callback(self, msg: Image):
        try:
            self.img_queue.put_nowait(msg)
        except queue.Full:
            # Drain one old item and replace with the latest
            try:
                self.img_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.img_queue.put_nowait(msg)
            except queue.Full:
                pass

    def _process_task(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, 3))
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        displayframe = None

        if self.frames_remaining > 0 and (self.is_detecting or self.is_saving):
            ts = int(time.time() * 1000)

            if self.is_detecting and self.model:
                results = self.model(frame_bgr, conf=self.threshold, verbose=False)
                if len(results[0].boxes) > 0:
                    annotated = results[0].plot()
                    path = os.path.join(self.save_dir, f"det_{ts}.jpg")
                    cv2.imwrite(path, annotated)
                    self.show = True

            elif self.is_saving:
                path = os.path.join(self.save_dir, f"raw_{ts}.jpg")
                cv2.imwrite(path, frame_bgr)

            self.frames_remaining -= 1

            # ✅ Reset only when the burst is fully consumed
            if self.frames_remaining == 0:
                self.is_saving = False
                self.is_detecting = False
                print("Camera Task Complete")

    async def _worker(self):
        """Async background consumer."""
        print("Camera background worker started.")
        while True:
            try:
                msg = self.img_queue.get_nowait()  # FIX: was referencing self.queue (undefined)
                await self.loop.run_in_executor(None, self._process_task, msg)
            except queue.Empty:
                await asyncio.sleep(0.01)  # FIX: yield to event loop instead of crashing
            except Exception as e:
                print(f"Worker error: {e}")
                await asyncio.sleep(0.01)

    async def run(self):
        """Entry point to start the subscription and worker."""
        self.loop = asyncio.get_running_loop()
        self.node = Node()

        if self.node.subscribe(Image, self.topic, self._image_callback):
            print(f"Subscribed to {self.topic}. No rendering (Headless).")
            worker_task = asyncio.create_task(self._worker())
            await asyncio.Future()  # Run forever
        else:
            print("Failed to subscribe.")


async def main():
    TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
    detector = GZPhotoDetectorSaver(topic=TOPIC, save_dir="output", model_path="yolov8n.pt", burst_size=20, threshold=0.5)
    await detector.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")