"""Command line entry point.

    python -m ankelbot sim  --scenario mixed            software-in-the-loop mission
    python -m ankelbot run                              real vessel (MAVLink + camera + YOLO26)
    python -m ankelbot detect --source clip.mp4         detector + size estimation on a video/camera
"""
from __future__ import annotations

import argparse
import json
import sys

from .config import load_onboard, load_params
from .hal.sim import Scenario, SimVehicle
from .mission import Mission
from .storage import Store

SCENARIOS = {
    "calm": Scenario(),
    "mixed": Scenario(n_over=6, n_border=4),
    "current": Scenario(n_over=4, current=(0.02, 0.06)),
    "noisy": Scenario(n_over=6, n_border=4, noise=2.5),
    "dense": Scenario(n_small=70, n_over=4, T=900),
    "low_battery": Scenario(soc0=20.3, T=500),
    "link_loss": Scenario(fail_time=120, T=200),
}


def cmd_sim(a) -> int:
    p = load_params()
    sc = SCENARIOS[a.scenario]
    sc.seed = a.seed
    veh = SimVehicle(p, sc)
    store = Store(a.db) if a.db else None
    m = Mission(p, veh, veh, store)

    def show(s, out, cmd):
        if a.verbose and round(s.t / p.dt) % 100 == 0:
            print(f"t={s.t:6.1f}s {out.state.name:9s} pos=({s.x:6.1f},{s.y:6.1f}) u={s.u:5.2f} soc={s.soc:5.1f}% fill={veh.fill:4.2f}")

    m.run(duration=sc.T, realtime=a.realtime, on_step=show)
    res = veh.summary() | {"backoffs": m.backoffs, "final_state": m.supervisor.state.name}
    print(json.dumps(res, indent=2))
    if store:
        store.close()
    return 0


def _detector(p, hw):
    from .detector import Yolo26Detector

    d = hw.detector
    return Yolo26Detector(d.weights, (int(p.cam.W), int(p.cam.H)), d.imgsz, d.conf, d.device,
                          d.collect_classes, d.oversize_classes, d.avoid_classes)


def cmd_run(a) -> int:
    from .detector import CameraPerception
    from .hal.camera import Camera
    from .hal.mavlink_link import MavlinkVehicle
    from .sync import Uploader

    p, hw = load_params(), load_onboard()
    cam = Camera(hw.camera.source, hw.camera.width, hw.camera.height)
    veh = MavlinkVehicle(p, hw.mavlink)
    store = Store(hw.storage.path)
    up = Uploader(store, hw.sync.endpoint if hw.sync.enabled else "")
    m = Mission(p, veh, CameraPerception(cam, _detector(p, hw)), store)
    veh.set_mode("MANUAL")
    if a.arm:
        veh.arm(True)
    store.event(0.0, "start", armed=bool(a.arm))

    def tick(s, out, cmd):
        if out.state.name == "DOCKED" and round(s.t) % 30 == 0:
            up.try_sync()                                     # only ever attempted while idle at the dock

    try:
        m.run(realtime=True, on_step=tick)
    except KeyboardInterrupt:
        pass
    finally:
        veh.close()
        cam.close()
        store.close()
    return 0


def cmd_detect(a) -> int:
    """Run the detector and the size estimator on a camera or video and print what it sees."""
    import time

    from . import geometry
    from .hal.camera import Camera

    p, hw = load_params(), load_onboard()
    if a.weights:
        hw.detector.weights = a.weights
    det = _detector(p, hw)
    src = int(a.source) if a.source.isdigit() else a.source
    cam = Camera(src)
    n, t0 = 0, time.perf_counter()
    while a.frames <= 0 or n < a.frames:
        frame = cam.read()
        if frame is None:
            break
        dets = det.infer(frame)
        n += 1
        for d in dets:
            lat, fwd, w, ok = geometry.estimate(d.bbox, p)
            pos = f"lat={lat:+.2f} fwd={fwd:.2f} width={w:.2f} m" if ok else "above horizon"
            print(f"frame {n:4d}  {d.label:14s} conf={d.conf:.2f}  {pos}{'  REJECT-CLASS' if d.over_class else ''}")
    dt = time.perf_counter() - t0
    print(f"{n} frames in {dt:.1f} s = {n / dt:.1f} FPS end-to-end (decode + inference + geometry)")
    cam.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ankelbot")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sim", help="software-in-the-loop mission")
    s.add_argument("--scenario", choices=sorted(SCENARIOS), default="mixed")
    s.add_argument("--seed", type=int, default=1)
    s.add_argument("--db", default="", help="write a SQLite log here")
    s.add_argument("--realtime", action="store_true")
    s.add_argument("--verbose", action="store_true")
    s.set_defaults(fn=cmd_sim)
    r = sub.add_parser("run", help="run on the vessel")
    r.add_argument("--arm", action="store_true", help="arm the autopilot (thrusters live)")
    r.set_defaults(fn=cmd_run)
    d = sub.add_parser("detect", help="detector + size estimation on a camera or video file")
    d.add_argument("--source", default="0")
    d.add_argument("--weights", default="")
    d.add_argument("--frames", type=int, default=0)
    d.set_defaults(fn=cmd_detect)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
