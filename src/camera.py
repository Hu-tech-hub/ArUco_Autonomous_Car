"""Latest-frame capture with explicit timestamps and shutdown."""
import threading
import time
from dataclasses import dataclass
import cv2


@dataclass
class Frame:
    sequence: int
    captured_at: float
    image: object


class LatestCamera:
    def __init__(self, device, resolution):
        self.device, self.resolution = device, tuple(resolution)
        self.condition = threading.Condition()
        self.closed = threading.Event()
        self.latest = None
        self.failure = None
        self.thread = None
        self.cap = None

    def start(self):
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError('Camera unavailable')
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()
        return self

    def _capture(self):
        sequence = 0
        try:
            while not self.closed.is_set():
                ok, image = self.cap.read()
                if not ok:
                    raise RuntimeError('Camera read failed')
                image = cv2.resize(image, self.resolution)
                sequence += 1
                with self.condition:
                    self.latest = Frame(sequence, time.monotonic(), image)
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.failure = error
                self.condition.notify_all()
        finally:
            self.cap.release()

    def read(self, after_sequence=0, after_time=0.0, timeout=0.5):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                if self.failure:
                    raise self.failure
                if self.closed.is_set():
                    return None
                if self.latest and self.latest.sequence > after_sequence and self.latest.captured_at > after_time:
                    frame = self.latest
                    return Frame(frame.sequence, frame.captured_at, frame.image.copy())
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)

    def close(self):
        self.closed.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread:
            self.thread.join(timeout=2)
