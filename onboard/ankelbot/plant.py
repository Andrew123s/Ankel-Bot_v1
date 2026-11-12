"""Vessel + thruster dynamics for the software-in-the-loop backend.

Port of ``matlab/+ankel/{vesselDeriv,plantStep}.m`` so the onboard stack can be
exercised on a desk without MATLAB. Checked against a MATLAB trajectory in
``tests/test_matlab_reference.py``.
"""
from __future__ import annotations

import math

from .control import cmd_to_thrust, wrap_pi


def deriv(x, T, p, fill, wings, cur):
    """State derivative. x = [x, y, psi, u, v, r]; u, v are through the water."""
    psi, u, v, r = x[2], x[3], x[4], x[5]
    m11, m22, m33 = p.m + p.Xud, p.m + p.Yvd, p.Iz + p.Nrd
    wing_f = (math.sin(wings[0]) + math.sin(wings[1])) / (2 * math.sin(p.wing_nom))
    Xuu = p.Xuu + p.Xuu_wing * wing_f + p.Xuu_net * (1 + p.net_fill_drag * fill)
    du = (m22 * v * r - p.Xu * u - Xuu * u * abs(u) + T[0] + T[1]) / m11
    dv = (-m11 * u * r - p.Yv * v - p.Yvv * v * abs(v)) / m22
    dr = (-p.Nr * r - p.Nrr * r * abs(r) + p.b * (T[0] - T[1])) / m33
    return [
        u * math.cos(psi) - v * math.sin(psi) + cur[0],
        u * math.sin(psi) + v * math.cos(psi) + cur[1],
        r, du, dv, dr,
    ]


def step(x, T, cmd, p, fill, wings, cur):
    """Advance one control period. Returns (x, T, electrical_power_W)."""
    n = round(p.dt / p.dt_plant)
    h = p.dt / n
    Tss = [cmd_to_thrust(cmd[0], p), cmd_to_thrust(cmd[1], p)]
    a = 1 - math.exp(-h / p.tau_thr)
    x = list(x)
    T = list(T)
    for _ in range(n):
        T = [T[0] + (Tss[0] - T[0]) * a, T[1] + (Tss[1] - T[1]) * a]
        k1 = deriv(x, T, p, fill, wings, cur)
        k2 = deriv([x[i] + h / 2 * k1[i] for i in range(6)], T, p, fill, wings, cur)
        k3 = deriv([x[i] + h / 2 * k2[i] for i in range(6)], T, p, fill, wings, cur)
        k4 = deriv([x[i] + h * k3[i] for i in range(6)], T, p, fill, wings, cur)
        x = [x[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(6)]
    x[2] = wrap_pi(x[2])
    c0 = max(-1.0, min(1.0, cmd[0]))
    c1 = max(-1.0, min(1.0, cmd[1]))
    P = p.Pmax * (abs(c0) ** 3 + abs(c1) ** 3) + p.P_hotel
    return x, T, P
