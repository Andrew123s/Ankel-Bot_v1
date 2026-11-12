"""YOLO26 waste detector.

Runs locally through the Ultralytics runtime; the weights file decides the
backend, so the same code serves a ``.pt`` file on a laptop, a TensorRT
``.engine`` on a Jetson or an NCNN/ONNX export on a Raspberry Pi. No network
access is needed once the weights are on disk.

Class policy (from ``config/onboard.json``):
* ``collect_classes``   waste the vessel should pick up (size permitting)
* ``oversize_classes``  things that are waste but must be rejected regardless
                        of the measured width (branches, logs, tyres ...)
* ``avoid_classes``     things that are not waste at all (people, boats, birds)
Anything else the model outputs is ignored.
"""
from __future__ import annotations

from .hal.base import Detection


class Yolo26Detector:
    def __init__(self, weights: str, calib_size: tuple[int, int], imgsz: int = 640, conf: float = 0.35,
                 device=None, collect_classes=(), oversize_classes=(), avoid_classes=()):
        from ultralytics import YOLO      # imported here so the rest of the stack runs without it

        self.model = YOLO(weights)
        self.calib_w, self.calib_h = calib_size
        self.imgsz, self.conf, self.device = imgsz, conf, device
        self.collect = set(collect_classes)
        self.reject = set(oversize_classes) | set(avoid_classes)

    def infer(self, frame) -> list[Detection]:
        """Detect on one BGR frame; boxes are returned in calibration pixels."""
        res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)[0]
        h, w = frame.shape[:2]
        sx, sy = self.calib_w / w, self.calib_h / h
        out = []
        for box, cls, conf in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist()):
            name = res.names[int(cls)]
            if name not in self.collect and name not in self.reject:
                continue
            out.append(Detection((box[0] * sx, box[1] * sy, box[2] * sx, box[3] * sy), name, float(conf), name in self.reject))
        return out


class CameraPerception:
    """Camera + detector as a ``Perception`` source for the mission loop."""

    def __init__(self, camera, detector: Yolo26Detector):
        self.camera, self.detector = camera, detector
        self.frame_ok = False

    def detect(self) -> list[Detection]:
        frame = self.camera.read()
        self.frame_ok = frame is not None
        return self.detector.infer(frame) if self.frame_ok else []
