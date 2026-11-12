"""Export trained weights for the onboard computer. Run this ON the target device.

    Jetson Orin Nano : python tools/export_model.py --weights best.pt --format engine --half
    Raspberry Pi 5   : python tools/export_model.py --weights best.pt --format ncnn
    any CPU          : python tools/export_model.py --weights best.pt --format onnx

TensorRT engines are tied to the GPU and TensorRT version they were built on,
which is why the export happens on the vessel's own computer. Point
"detector.weights" in config/onboard.json at the exported file afterwards.
"""
import argparse

from ultralytics import YOLO


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--format", default="engine", choices=["engine", "onnx", "ncnn", "openvino"])
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--half", action="store_true", help="FP16 (TensorRT)")
    ap.add_argument("--device", default=None)
    a = ap.parse_args()
    out = YOLO(a.weights).export(format=a.format, imgsz=a.imgsz, half=a.half, device=a.device)
    print("exported:", out)


if __name__ == "__main__":
    main()
