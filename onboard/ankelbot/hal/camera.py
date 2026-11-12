"""Camera / video-file source (OpenCV)."""
from __future__ import annotations


class Camera:
    """``source`` is a device index, a GStreamer pipeline string or a video file."""

    def __init__(self, source=0, width: int = 1280, height: int = 720):
        import cv2

        self.cv2 = cv2
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera source {source!r}")
        if isinstance(source, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # always work on the newest frame

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()
