"""Mission state machine.

Line-for-line port of ``matlab/+ankel/supervisorStep.m``, which is the
reference. ``tests/test_matlab_reference.py`` replays a full MATLAB mission
through this class and requires identical decisions at every step.

Track rows are ``[valid, lat, fwd, width, kind]`` in the body frame with
``kind`` 0 unknown, 1 collectable, 2 oversize.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

from .control import wrap_pi


class State(IntEnum):
    IDLE = 0
    SEARCH = 1
    APPROACH = 2
    AVOID = 3
    BACKOFF = 4
    RETURN = 5
    DOCKED = 6
    FAILSAFE = 7


@dataclass
class SupervisorInput:
    x: float
    y: float
    psi: float
    tracks: list            # rows [valid, lat, fwd, width, kind]
    area_shift: tuple = (0.0, 0.0)
    net_full: bool = False
    soc: float = 100.0
    obst_range: float = 99.0
    mouth_blocked: bool = False
    healthy: bool = True


@dataclass
class SupervisorOutput:
    state: State = State.SEARCH
    psi_ref: float = 0.0
    u_ref: float = 0.0
    gate: int = 0                       # 1 = reject rake closed
    wing: list = field(default_factory=lambda: [0.0, 0.0])   # [port, stbd] rad
    enable: int = 1                     # 0 = thrusters must be zero


class Supervisor:
    def __init__(self, p):
        self.p = p
        self.state = State.SEARCH
        self.t_state = 0.0
        self.wp = 1                     # 1-based, as in MATLAB
        self.hold_psi = 0.0
        self.back_psi = 0.0
        self.side = -1
        self.t_clear = 0.0
        self.t_block = 0.0
        self.gate = 0
        self.t_over = 99.0

    def step(self, i: SupervisorInput) -> SupervisorOutput:
        p, q, dt = self.p, self.p.sup, self.p.dt
        S = State
        self.t_state += dt
        corridor = p.hinge_x + p.wing_len * math.sin(p.wing_nom) + q.corridor_margin
        mg = q.hold_margin if self.state == S.AVOID else 0.0

        # An object resting on the CLOSED rake is expected; one blocking the
        # mouth while the rake is OPEN is a jam.
        if i.mouth_blocked and self.gate == 0:
            self.t_block += dt
        else:
            self.t_block = 0.0

        # ---- scan tracks ----
        threat, thr, best_thr = False, (0.0, 0.0, 0.0), math.inf
        target, tgt, best_tgt = False, (0.0, 0.0), math.inf
        funnel = over_near = False
        for tr in i.tracks:
            if tr[0] < 0.5:
                continue
            lat, fwd, w, kind = tr[1], tr[2], tr[3], tr[4]
            rng = math.hypot(lat, fwd)
            bad = kind == 2 or (kind == 0 and rng < q.r_gate)
            if bad and fwd > -2.0 and rng < q.r_avoid and abs(lat) < corridor + w / 2 + mg and rng < best_thr:
                threat, thr, best_thr = True, (lat, fwd, w), rng
            if kind == 2 and 0 < fwd < q.funnel_fwd and abs(lat) < q.funnel_lat:
                funnel = True
            if kind == 2 and 0 < fwd < q.near_fwd and abs(lat) < corridor:
                over_near = True
            if kind == 1 and fwd > 0.3 and rng < best_tgt:
                target, tgt, best_tgt = True, (lat, fwd), rng

        # ---- transitions (highest priority first) ----
        prev, reenter = self.state, False
        self.t_over = 0.0 if over_near else min(99.0, self.t_over + dt)
        # The mouth sensor cannot tell a waiting bottle from a log. If an oversize
        # object was tracked into the funnel moments ago, never open onto it.
        suspect = i.mouth_blocked and self.t_over < q.t_over
        trigger = i.obst_range < q.d_stop or self.t_block >= q.t_block or funnel or suspect
        d_dock = math.hypot(i.x - q.dock[0], i.y - q.dock[1])
        if (not i.healthy) or i.soc < q.soc_crit:
            ns = S.FAILSAFE
        elif prev == S.FAILSAFE:
            ns = S.SEARCH
        elif prev == S.BACKOFF and self.t_state < q.t_back + q.t_turn:
            ns = S.BACKOFF
        elif prev == S.DOCKED and (self.t_state < q.t_swap or i.soc < q.soc_return or i.net_full):
            ns = S.DOCKED
        elif trigger:
            ns = S.BACKOFF
            reenter = prev == S.BACKOFF
        elif i.net_full or i.soc < q.soc_return:
            ns = S.DOCKED if d_dock < q.dock_radius else S.RETURN
        elif threat:
            ns = S.AVOID
        elif prev == S.AVOID and self.t_clear < q.t_clear:
            ns = S.AVOID
        elif target:
            ns = S.APPROACH
        else:
            ns = S.SEARCH

        # ---- entry actions ----
        if ns != prev or reenter:
            self.t_state = 0.0
            if ns == S.BACKOFF:
                side = -1 if (threat and thr[0] > 0) else 1
                self.hold_psi = i.psi
                self.back_psi = wrap_pi(i.psi + side * q.back_turn)
            elif ns == S.AVOID:
                self.side = 1 if thr[0] < 0 else -1
                self.t_clear = 0.0
        self.state = ns

        # ---- outputs ----
        out = SupervisorOutput(state=ns, psi_ref=i.psi, u_ref=0.0, gate=0,
                               wing=[p.wing_nom, p.wing_nom], enable=1)
        if ns == S.FAILSAFE:
            out.enable, out.gate = 0, 1
        elif ns == S.BACKOFF:
            out.gate = 1
            if self.t_state < q.t_back:
                out.u_ref, out.psi_ref = q.u_back, self.hold_psi
            else:
                out.psi_ref = self.back_psi
        elif ns == S.DOCKED:
            out.gate, out.wing = 1, [p.wing_min, p.wing_min]
        elif ns == S.RETURN:
            out.gate, out.wing = 1, [p.wing_min, p.wing_min]
            out.psi_ref = math.atan2(q.dock[1] - i.y, q.dock[0] - i.x)
            out.u_ref = q.u_search
        elif ns == S.AVOID:
            out.gate, out.u_ref = 1, q.u_avoid
            if threat:
                rng = max(math.hypot(thr[0], thr[1]), 0.01)
                c = corridor + thr[2] / 2 + q.avoid_extra
                off = math.asin(min(1.0, c / rng))
                self.hold_psi = wrap_pi(i.psi + math.atan2(thr[0], thr[1]) + self.side * off)
                self.t_clear = 0.0
            else:
                self.t_clear += dt
            out.psi_ref = self.hold_psi
            if self.side < 0:
                out.wing[1] = p.wing_min
            else:
                out.wing[0] = p.wing_min
        elif ns == S.APPROACH:
            brg = math.atan2(tgt[0], tgt[1])
            out.psi_ref = wrap_pi(i.psi + brg)
            out.u_ref = q.u_approach if abs(brg) < q.bearing_slow else q.u_slow
        else:  # SEARCH
            wps = q.wps
            wx = wps[self.wp - 1][0] + i.area_shift[0]
            wy = wps[self.wp - 1][1] + i.area_shift[1]
            if math.hypot(wx - i.x, wy - i.y) < q.wp_radius:
                self.wp = self.wp % len(wps) + 1
                wx = wps[self.wp - 1][0] + i.area_shift[0]
                wy = wps[self.wp - 1][1] + i.area_shift[1]
            out.psi_ref = math.atan2(wy - i.y, wx - i.x)
            out.u_ref = q.u_search
        self.gate = out.gate
        return out
