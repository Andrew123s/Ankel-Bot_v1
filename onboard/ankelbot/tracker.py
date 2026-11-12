"""Object tracker and accept/reject classification.

Turns per-frame detections into stable tracks so one noisy box never opens or
closes the rake. Tracks live in the world frame, are remembered for a few
seconds after they leave the camera's view (objects disappear under the bow
long before they reach the mouth) and are carried along with the estimated
surface drift.

The classification rule is a port of ``matlab/+ankel/classifyTrack.m``.

Where this differs from the MATLAB model, and why: the MATLAB simulation knows
which detection belongs to which object. Here that has to be worked out, and
monocular range is poor far away (about 0.05 m error at 2 m but 0.7 m at 6 m
for 2 px / 0.7 deg of noise). So this tracker
* associates in bearing/range with a range gate that widens with distance,
* trusts far fixes less when updating a track's position,
* estimates drift only from fixes closer than ``NEAR_M``, as the median over
  the most recent tracks, and ignores estimates below ``DRIFT_DEADBAND``.
"""
from __future__ import annotations

import math
from collections import OrderedDict, deque
from dataclasses import dataclass, field
from itertools import count
from statistics import median

UNKNOWN, COLLECTABLE, OVERSIZE = 0, 1, 2

NEAR_M = 3.5            # fixes closer than this are accurate enough to measure drift
ANCHOR_N = 5            # near fixes averaged into a track's drift reference
DRIFT_TRACKS = 7        # recent tracks whose velocities form the drift estimate
MIN_DRIFT_TRACKS = 3
DRIFT_DEADBAND = 0.04   # [m/s] below this the estimate is indistinguishable from noise
RANGE_K = 0.02          # range noise ~ RANGE_K * range^2 [m]


@dataclass
class Measurement:
    lat: float
    fwd: float
    width: float
    truncated: bool = False
    over_class: bool = False       # detector class is one that must never be collected
    label: str = ""
    conf: float = 0.0


def classify(widths, n_obs: int, over_votes: int, p) -> int:
    """0 unknown (too few observations), 1 collectable, 2 oversize."""
    if n_obs < p.dec.min_obs:
        return UNKNOWN
    w = [v for v in widths if v is not None and not math.isnan(v)]
    if not w:
        return UNKNOWN
    if median(w) > p.dec.w_max or over_votes > n_obs / 2:
        return OVERSIZE
    return COLLECTABLE


@dataclass
class Track:
    id: int
    x: float
    y: float
    tlast: float
    n: int = 0
    votes: int = 0
    widths: deque = field(default_factory=deque)
    label: str = ""
    near_n: int = 0
    ax: float = 0.0
    ay: float = 0.0
    at: float = 0.0


class Tracker:
    def __init__(self, p, max_rows: int = 8):
        self.p = p
        self.max_rows = max_rows
        self.tracks: list[Track] = []
        self.drift = [0.0, 0.0]          # drift applied to tracks and search pattern [m/s]
        self.drift_raw = [0.0, 0.0]      # before the deadband
        self.area_shift = [0.0, 0.0]     # integral of drift
        self._vel: OrderedDict[int, tuple[float, float]] = OrderedDict()
        self._ids = count(1)

    def update(self, t: float, x: float, y: float, psi: float, measurements: list[Measurement]) -> list[list[float]]:
        """Fuse this frame's measurements; return rows for the supervisor."""
        p, d = self.p, self.p.dec
        cp, sp = math.cos(psi), math.sin(psi)

        def body(tr: Track) -> tuple[float, float]:
            dx, dy = tr.x - x, tr.y - y
            return -dx * sp + dy * cp, dx * cp + dy * sp          # lat, fwd

        used: set[int] = set()
        for m in measurements:
            r = math.hypot(m.lat, m.fwd)
            brg = math.atan2(m.lat, m.fwd)
            best, best_cost = None, 1.0
            for k, tr in enumerate(self.tracks):
                if k in used:
                    continue
                tl, tf = body(tr)
                tr_r = math.hypot(tl, tf)
                d_brg = abs(math.atan2(tl, tf) - brg) / (0.05 + 0.25 / max(tr_r, 0.5))
                d_rng = abs(tr_r - r) / (0.4 + 3 * RANGE_K * r * r)
                cost = max(d_brg, d_rng)
                if cost < best_cost:
                    best, best_cost = k, cost
            wx = x + m.fwd * cp - m.lat * sp
            wy = y + m.fwd * sp + m.lat * cp
            if best is None:
                tr = Track(next(self._ids), wx, wy, t, widths=deque(maxlen=int(d.buf)))
                self.tracks.append(tr)
                used.add(len(self.tracks) - 1)
            else:
                tr = self.tracks[best]
                gain = 0.4 * min(1.0, (3.0 / r) ** 2)             # far fixes are noisy in range
                if m.truncated:
                    gain = min(gain, 0.1)                         # a cut box misplaces the waterline
                gain = max(gain, 0.05)
                tr.x += gain * (wx - tr.x)
                tr.y += gain * (wy - tr.y)
                used.add(best)
            tr.n += 1
            tr.tlast = t
            tr.label = m.label or tr.label
            # a box cut by the image edge only bounds the width from below
            if (not m.truncated) or not tr.widths or m.width > median(tr.widths):
                tr.widths.append(m.width)
            if m.over_class:
                tr.votes += 1

            # drift evidence: how far the object moved between its first and latest near fixes
            if r < NEAR_M and not m.truncated:
                tr.near_n += 1
                if tr.near_n <= ANCHOR_N:
                    k = tr.near_n
                    tr.ax += (wx - tr.ax) / k
                    tr.ay += (wy - tr.ay) / k
                    tr.at += (t - tr.at) / k
                elif tr.near_n >= 2 * ANCHOR_N and t - tr.at > d.drift_min_age:
                    dt_a = t - tr.at
                    self._vel[tr.id] = ((tr.x - tr.ax) / dt_a, (tr.y - tr.ay) / dt_a)
                    self._vel.move_to_end(tr.id)
                    while len(self._vel) > DRIFT_TRACKS:
                        self._vel.popitem(last=False)

        # common drift: every floating object rides the same surface current
        if len(self._vel) >= MIN_DRIFT_TRACKS:
            mx = median(v[0] for v in self._vel.values())
            my = median(v[1] for v in self._vel.values())
            spd = math.hypot(mx, my)
            if spd > d.drift_max:
                mx, my = mx * d.drift_max / spd, my * d.drift_max / spd
            a = d.drift_alpha
            self.drift_raw = [(1 - a) * self.drift_raw[0] + a * mx, (1 - a) * self.drift_raw[1] + a * my]
        if math.hypot(*self.drift_raw) > DRIFT_DEADBAND:
            self.drift = list(self.drift_raw)
        else:
            self.drift = [0.0, 0.0]
        for tr in self.tracks:
            tr.x += self.drift[0] * p.dt
            tr.y += self.drift[1] * p.dt
        self.area_shift[0] += self.drift[0] * p.dt
        self.area_shift[1] += self.drift[1] * p.dt

        self.tracks = [tr for tr in self.tracks if t - tr.tlast <= d.t_keep]

        rows = []
        for tr in self.tracks:
            lat, fwd = body(tr)
            if fwd < -2.5:
                continue
            kind = classify(tr.widths, tr.n, tr.votes, p)
            rows.append([1.0, lat, fwd, median(tr.widths), float(kind)])
        rows.sort(key=lambda r: math.hypot(r[1], r[2]))
        return rows[: self.max_rows]
