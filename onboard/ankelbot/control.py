"""Thrust map, heading/speed controller and thrust allocation.

Port of ``matlab/+ankel/{cmdToThrust,thrustToCmd,controlLaw}.m``. The gains are
designed in MATLAB (``designGains.m``) and arrive through the parameter file.
``tests/test_matlab_reference.py`` checks this module against MATLAB output.
"""
from __future__ import annotations

import math


def wrap_pi(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def cmd_to_thrust(c: float, p) -> float:
    """Steady-state thrust [N] for an ESC command in [-1, 1] (thrust ~ c|c|)."""
    c = max(-1.0, min(1.0, c))
    if abs(c) < p.cmd_deadband:
        return 0.0
    if c > 0:
        return p.Tmax * c * c
    return -p.rev * p.Tmax * c * c


def thrust_to_cmd(T: float, p) -> float:
    """ESC command that produces thrust ``T`` [N]; inverse of cmd_to_thrust."""
    if T >= 0:
        return math.sqrt(min(T, p.Tmax) / p.Tmax)
    return -math.sqrt(min(-T, p.rev * p.Tmax) / (p.rev * p.Tmax))


class Controller:
    """PD heading loop + PI speed loop with drag feed-forward.

    Yaw has priority in the allocation: the differential thrust is granted
    first and the surge demand gets what is left. The speed integrator only
    advances while the surge demand is unsaturated.
    """

    def __init__(self, p):
        self.p = p
        self.iu = 0.0

    def reset(self) -> None:
        self.iu = 0.0

    def step(self, psi_ref: float, u_ref: float, psi: float, u: float, r: float) -> tuple[float, float]:
        """Return ESC commands (port, starboard) in [-1, 1]."""
        p, g = self.p, self.p.gains
        Tf, Tr = p.Tmax, p.rev * p.Tmax

        e = wrap_pi(psi_ref - psi)
        Mz = g.Kp_psi * e - g.Kd_psi * r
        dT = max(-(Tf + Tr) / 2, min((Tf + Tr) / 2, Mz / (2 * p.b)))

        eu = u_ref - u
        i_new = max(-g.iu_max, min(g.iu_max, self.iu + eu * p.dt))
        Fx = p.Xu * u_ref + p.Xuu_nom * u_ref * abs(u_ref) + g.Kp_u * eu + g.Ki_u * i_new

        c = Fx / 2
        cs = max(-Tr + abs(dT), min(Tf - abs(dT), c))
        if cs == c:
            self.iu = i_new
        if u_ref == 0:
            self.iu = 0.0
        return thrust_to_cmd(cs + dT, p), thrust_to_cmd(cs - dT, p)
