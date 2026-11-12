"""Software-in-the-loop vessel.

Implements both ``Vehicle`` and ``Perception`` with the same physics as the
MATLAB model (``ankel.World``): plant, wing/rake actuators, battery, debris
field, collection at the tunnel mouth, and a camera model that turns debris
into detection boxes. Everything downstream of the boxes (size estimation,
tracker, supervisor, controller) is the real onboard code.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .. import geometry, plant
from .base import Command, Detection, VehicleState


@dataclass
class Scenario:
    T: float = 600.0
    seed: int = 1
    n_small: int = 30
    n_border: int = 0
    n_over: int = 0
    current: tuple = (0.0, 0.0)
    soc0: float = 100.0
    noise: float = 1.0
    fail_time: float = math.inf


@dataclass
class Debris:
    x: float
    y: float
    w: float
    h: float
    over_class: bool
    st: int = 0                 # 0 free, 1 collected, 2 held at rake, 3 jammed
    lat_hold: float = 0.0


@dataclass
class SimStats:
    jams: int = 0
    holds: int = 0
    swaps: int = 0
    energy_wh: float = 0.0


class SimVehicle:
    def __init__(self, p, sc: Scenario | None = None):
        self.p = p
        self.sc = sc or Scenario()
        self.rng = random.Random(self.sc.seed)
        a = p.sup.area
        self.debris: list[Debris] = []
        kinds = [(self.sc.n_small, 0.06, 0.22, False), (self.sc.n_border, 0.30, 0.12, False), (self.sc.n_over, 0.50, 0.70, True)]
        for n, w0, dw, over in kinds:
            for _ in range(n):
                self.debris.append(Debris(
                    x=a[0] + 1.5 + self.rng.random() * (a[1] - a[0] - 1.5),
                    y=a[2] + self.rng.random() * (a[3] - a[2]),
                    w=w0 + dw * self.rng.random(),
                    h=(0.10 + 0.15 * self.rng.random()) if over else (0.04 + 0.08 * self.rng.random()),
                    over_class=over))
        self.x = [0.0] * 6
        self.T = [0.0, 0.0]
        self.wing = [p.wing_nom, p.wing_nom]
        self.gate = 0.0
        self.soc = self.sc.soc0
        self.fill = 0.0
        self.t = 0.0
        self.cmd = Command(gate=0, wing_port=p.wing_nom, wing_stbd=p.wing_nom)
        self.docked_for = 0.0
        self.stats = SimStats()

    # ---- Vehicle ----
    def read_state(self) -> VehicleState:
        sc, x = self.sc, self.x
        ug = x[3] + sc.current[0] * math.cos(x[2]) + sc.current[1] * math.sin(x[2])
        n = sc.noise
        roll = n * 0.03 * math.sin(1.1 * self.t)
        pitch = n * 0.02 * math.sin(0.8 * self.t + 1)
        self._true_att = (roll, pitch)
        sig = self.p.det.sigma_att * n
        return VehicleState(
            t=self.t, x=x[0], y=x[1], psi=x[2], u=ug, r=x[5],
            roll=roll + self.rng.gauss(0, sig), dpitch=pitch + self.rng.gauss(0, sig),
            soc=self.soc, obst_range=99.0,
            mouth_blocked=any(d.st >= 2 for d in self.debris),
            net_full=self.fill >= 1.0, healthy=self.t < sc.fail_time)

    def send(self, c: Command) -> None:
        self.cmd = c

    def close(self) -> None:
        pass

    # ---- Perception ----
    def detect(self) -> list[Detection]:
        p, sc, x = self.p, self.sc, self.x
        cp, sp = math.cos(x[2]), math.sin(x[2])
        roll, pitch = getattr(self, "_true_att", (0.0, 0.0))
        out = []
        for d in self.debris:
            if d.st != 0:
                continue
            dx, dy = d.x - x[0], d.y - x[1]
            fwd, lat = dx * cp + dy * sp, -dx * sp + dy * cp
            if fwd < p.cam.fwd + 0.3 or math.hypot(lat, fwd) > p.det.max_range:
                continue
            bb, ok, _ = geometry.project(lat, fwd, d.w, d.h, p, roll, pitch)
            if not ok or bb[2] - bb[0] < p.det.min_px or self.rng.random() > p.det.p_detect:
                continue
            s = p.det.sigma_px * sc.noise
            bb = [v + self.rng.gauss(0, s) for v in bb]
            bb[3] = min(bb[3], float(p.cam.H))
            is_over = d.over_class and self.rng.random() < p.det.p_class
            out.append(Detection(tuple(bb), "branch" if is_over else "waste", 0.8, is_over))
        return out

    # ---- physics ----
    def advance(self, sup_state: int = 1, sup_t_state: float = 0.0) -> None:
        """Advance one control period using the last command sent."""
        p, sc, c, dt = self.p, self.sc, self.cmd, self.p.dt
        lim = p.wing_rate * dt
        for i, tgt in enumerate((c.wing_port, c.wing_stbd)):
            self.wing[i] += max(-lim, min(lim, tgt - self.wing[i]))
        g = dt / p.gate_time
        self.gate = max(0.0, min(1.0, self.gate + max(-g, min(g, c.gate - self.gate))))

        self.x, self.T, P = plant.step(self.x, self.T, (c.cmd_port, c.cmd_stbd), p, min(self.fill, 1.0), self.wing, sc.current)
        self.soc = max(0.0, self.soc - P * dt / 3600 / p.E_batt * 100)
        self.stats.energy_wh += P * dt / 3600
        self.t += dt
        self._collect()
        if sup_state == 6 and sup_t_state > 10 and self.fill > 0:      # DOCKED: operator swaps the net
            self.fill = 0.0
            self.stats.swaps += 1

    def _collect(self) -> None:
        p, x = self.p, self.x
        cp, sp = math.cos(x[2]), math.sin(x[2])
        busy = any(d.st >= 2 for d in self.debris)
        jam_present = any(d.st == 3 for d in self.debris)
        for d in self.debris:
            if d.st >= 2:
                if x[3] < -0.05:
                    d.st = 0                                         # washed off when reversing
                elif d.st == 2 and self.gate < 0.5 and not jam_present:
                    if d.w <= p.pass_w:
                        d.st = 1
                        self.fill += p.pack * 0.5 * d.w ** 2 * d.h / p.V_net
                    else:
                        d.st = 3
                        self.stats.jams += 1
                        jam_present = True
                if d.st >= 2:
                    f = p.hinge_fwd + 0.12
                    d.x = x[0] + f * cp - d.lat_hold * sp
                    d.y = x[1] + f * sp + d.lat_hold * cp
                continue
            if d.st != 0:
                continue
            d.x += self.sc.current[0] * p.dt
            d.y += self.sc.current[1] * p.dt
            dx, dy = d.x - x[0], d.y - x[1]
            fwd, lat = dx * cp + dy * sp, -dx * sp + dy * cp
            rr = min(d.w / 2, 0.25)
            aw = self.wing[0] if lat < 0 else self.wing[1]
            moved = False
            if p.hinge_fwd < fwd < p.hinge_fwd + p.wing_len * math.cos(aw):
                e = p.hinge_x + (fwd - p.hinge_fwd) * math.tan(aw)
                if e - rr < abs(lat) < e:                            # riding along the wing
                    lat = math.copysign(max(e - rr, 0.0), lat)
                    moved = True
            elif p.hinge_fwd - 0.15 < fwd <= p.hinge_fwd and abs(lat) < p.mouth_w / 2 + 0.05:
                if self.gate > 0.5 or busy:
                    d.st = 2
                    self.stats.holds += 1
                elif d.w > p.pass_w:
                    d.st = 3
                    self.stats.jams += 1
                    jam_present = True
                else:
                    d.st = 1
                    self.fill += p.pack * 0.5 * d.w ** 2 * d.h / p.V_net
                d.lat_hold = max(-0.2, min(0.2, lat))
                busy = busy or d.st >= 2
                continue
            elif abs(lat) < p.B / 2 + rr and -0.85 < fwd <= p.hinge_fwd - 0.15:
                lat = math.copysign(p.B / 2 + rr, lat)               # deflected by the hull side
                moved = True
            if moved:
                d.x = x[0] + fwd * cp - lat * sp
                d.y = x[1] + fwd * sp + lat * cp

    # ---- results ----
    def summary(self) -> dict:
        p = self.p
        fits = [d for d in self.debris if d.w <= p.pass_w]
        got = sum(1 for d in self.debris if d.st == 1)
        return {
            "n_collectable": len(fits), "collected": got,
            "collected_pct": 100.0 * got / max(1, len(fits)),
            "n_oversize": len(self.debris) - len(fits),
            "jams": self.stats.jams, "holds": self.stats.holds, "swaps": self.stats.swaps,
            "energy_wh": self.stats.energy_wh, "soc_end": self.soc,
        }
