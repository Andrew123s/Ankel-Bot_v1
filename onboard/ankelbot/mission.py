"""The control loop: one pass every ``p.dt`` seconds.

    vehicle state + detections
        -> geometry   (box -> position and width on the water)
        -> tracker    (stable tracks, accept / reject)
        -> supervisor (what to do)
        -> controller (ESC commands)
        -> vehicle    (thrusters, wings, reject rake)
        -> store      (local log)

The same loop drives the simulator and the real vessel; only the ``vehicle``
and ``perception`` objects differ.
"""
from __future__ import annotations

import time

from . import geometry
from .control import Controller
from .hal.base import Command, Perception, Vehicle
from .supervisor import State, Supervisor, SupervisorInput
from .tracker import Measurement, Tracker


class Mission:
    def __init__(self, p, vehicle: Vehicle, perception: Perception, store=None):
        self.p, self.vehicle, self.perception, self.store = p, vehicle, perception, store
        self.tracker = Tracker(p)
        self.supervisor = Supervisor(p)
        self.controller = Controller(p)
        self.last_state = State.SEARCH
        self.backoffs = 0

    def step(self):
        p = self.p
        s = self.vehicle.read_state()
        dets = self.perception.detect()
        healthy = s.healthy and getattr(self.perception, "frame_ok", True)

        meas = []
        for d in dets:
            lat, fwd, w, valid = geometry.estimate(d.bbox, p, s.roll, s.dpitch)
            if not valid or w > 3.0 or fwd <= 0:
                continue
            m = Measurement(lat, fwd, w, geometry.is_truncated(d.bbox, p), d.over_class, d.label, d.conf)
            meas.append(m)
            if self.store:
                self.store.detection(s.t, m)

        tracks = self.tracker.update(s.t, s.x, s.y, s.psi, meas)
        out = self.supervisor.step(SupervisorInput(
            x=s.x, y=s.y, psi=s.psi, tracks=tracks, area_shift=tuple(self.tracker.area_shift),
            net_full=s.net_full, soc=s.soc, obst_range=s.obst_range,
            mouth_blocked=s.mouth_blocked, healthy=healthy))

        cmd = self.controller.step(out.psi_ref, out.u_ref, s.psi, s.u, s.r)
        if not out.enable:
            cmd = (0.0, 0.0)
        self.vehicle.send(Command(cmd[0], cmd[1], out.gate, out.wing[0], out.wing[1]))

        if out.state != self.last_state:
            if out.state == State.BACKOFF:
                self.backoffs += 1
            if self.store:
                self.store.event(s.t, "state", frm=self.last_state.name, to=out.state.name, soc=round(s.soc, 1))
            self.last_state = out.state
        if self.store:
            self.store.telemetry(s, out, cmd, len(tracks))
        return s, out, cmd

    def run(self, duration: float | None = None, realtime: bool = True, on_step=None) -> None:
        """Run until ``duration`` seconds of vessel time have passed (or forever)."""
        p = self.p
        nxt = time.monotonic()
        while True:
            s, out, cmd = self.step()
            if hasattr(self.vehicle, "advance"):               # simulator: move the world on
                self.vehicle.advance(int(self.supervisor.state), self.supervisor.t_state)
            if on_step:
                on_step(s, out, cmd)
            if duration is not None and s.t >= duration:
                break
            if realtime:
                nxt += p.dt
                time.sleep(max(0.0, nxt - time.monotonic()))
