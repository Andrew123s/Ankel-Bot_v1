"""Measure real inference speed of a YOLO26 model on THIS machine.

    python tools/bench_inference.py --weights yolo26n.pt --source clip.mp4 --frames 100

Prints per-frame latency (pre-process + inference + post-process, as reported
by the runtime) and the resulting FPS. Run it on the onboard computer with the
exported model to confirm the 10 Hz control loop has headroom.
"""
import argparse
import statistics
import time

import cv2
from ultralytics import YOLO


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="yolo26n.pt")
    ap.add_argument("--source", required=True, help="video file or camera index")
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default=None)
    a = ap.parse_args()

    model = YOLO(a.weights)
    cap = cv2.VideoCapture(int(a.source) if a.source.isdigit() else a.source)
    lat, n_det, names = [], 0, {}
    for i in range(a.frames + 5):
        ok, frame = cap.read()
        if not ok:
            break
        t0 = time.perf_counter()
        r = model.predict(frame, imgsz=a.imgsz, device=a.device, conf=0.25, verbose=False)[0]
        dt = (time.perf_counter() - t0) * 1000
        if i >= 5:                                   # skip warm-up frames
            lat.append(dt)
            n_det += len(r.boxes)
            for c in r.boxes.cls.tolist():
                names[r.names[int(c)]] = names.get(r.names[int(c)], 0) + 1
    cap.release()
    if not lat:
        raise SystemExit("no frames read")
    lat.sort()
    print(f"model {a.weights}  imgsz {a.imgsz}  frames {len(lat)}")
    print(f"latency ms: median {statistics.median(lat):.1f}  p90 {lat[int(0.9 * len(lat)) - 1]:.1f}  max {lat[-1]:.1f}")
    print(f"throughput: {1000 / statistics.median(lat):.1f} FPS")
    print(f"detections: {n_det} total  {dict(sorted(names.items(), key=lambda kv: -kv[1]))}")


if __name__ == "__main__":
    main()
