"""Fine-tune YOLO26 on a floating-waste dataset.

    python tools/train_yolo26.py --data datasets/floating_waste/data.yaml --model yolo26n.pt

The dataset is not part of this repository. It must be in Ultralytics YOLO
format and use the class names listed in config/onboard.json (collect,
oversize and avoid classes); see datasets/data.example.yaml. Suitable public
starting points are floating-waste sets such as FloW-Img plus your own frames
recorded from the vessel's camera at its mounting height - images taken from
the bank or from drones look different from the 0.55 m viewpoint.

Training needs a GPU workstation or a cloud notebook; only inference runs on
the vessel. The result (runs/.../weights/best.pt) is then exported for the
onboard computer with tools/export_model.py.
"""
import argparse

from ultralytics import YOLO


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="dataset YAML")
    ap.add_argument("--model", default="yolo26n.pt", help="pretrained checkpoint to start from")
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default=None)
    ap.add_argument("--name", default="ankelbot_yolo26n")
    a = ap.parse_args()

    model = YOLO(a.model)
    model.train(
        data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, device=a.device, name=a.name,
        patience=30,
        # water scenes: strong glare and colour shifts, objects never upside-down
        hsv_h=0.015, hsv_s=0.6, hsv_v=0.5, fliplr=0.5, flipud=0.0, degrees=5.0, scale=0.5, mosaic=1.0,
    )
    metrics = model.val(data=a.data, imgsz=a.imgsz, device=a.device)
    print(f"mAP50-95 {metrics.box.map:.3f}   mAP50 {metrics.box.map50:.3f}")
    for i, name in model.names.items():
        if i < len(metrics.box.maps):
            print(f"  {name:16s} mAP50-95 {metrics.box.maps[i]:.3f}")


if __name__ == "__main__":
    main()
