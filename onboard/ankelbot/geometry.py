"""Monocular position and size estimation for floating objects.

Port of ``matlab/+ankel/{cameraProject,estimateFromBBox}.m``.

Flat-water assumption: the bottom edge of a detection box lies on the water
surface, a known height below the lens. Each bottom corner therefore
back-projects to a unique point on the water, which gives range, bearing and
the object's width across the line of sight. Roll and pitch from the IMU
remove vessel motion.

Limits that follow from the method, not from the code:
* only the width ACROSS the view is measured; a log pointing straight at the
  camera looks narrow, which is why oversize classes also vote in the tracker
* waves lift objects off the assumed plane; error grows with range
* a box cut by the image edge only bounds the width from below
"""
from __future__ import annotations

import math


def project(lat: float, fwd: float, w: float, h_obj: float, p, roll: float = 0.0, dpitch: float = 0.0):
    """Pixel box ``(u1, v1, u2, v2)`` of an object at body position (lat, fwd).

    Used by the simulator to synthesise what the camera would see.
    Returns ``(bbox, ok, truncated)``.
    """
    c = p.cam
    th = c.pitch + dpitch
    Z = fwd - c.fwd
    pts = [
        (lat - w / 2, c.h, Z),
        (lat + w / 2, c.h, Z),
        (lat - w / 2, c.h - h_obj, Z),
        (lat + w / 2, c.h - h_obj, Z),
    ]
    us, vs = [], []
    cr, sr = math.cos(roll), math.sin(roll)
    for X, Y, Zp in pts:
        yc = Y * math.cos(th) - Zp * math.sin(th)
        zc = Y * math.sin(th) + Zp * math.cos(th)
        if zc <= 0.05:
            return (0.0, 0.0, 0.0, 0.0), False, False
        a = c.fx * X / zc
        b = c.fy * yc / zc
        us.append(c.cx + a * cr - b * sr)
        vs.append(c.cy + a * sr + b * cr)
    u1, v1, u2, v2 = min(us), min(vs), max(us), max(vs)
    if u2 <= 0 or u1 >= c.W or v2 <= 0 or v1 >= c.H:
        return (u1, v1, u2, v2), False, False
    trunc = u1 < 0 or u2 > c.W or v2 > c.H
    return (max(u1, 0.0), max(v1, 0.0), min(u2, float(c.W)), min(v2, float(c.H))), True, trunc


def estimate(bbox, p, roll: float = 0.0, dpitch: float = 0.0):
    """Body-frame position and width from a detection box.

    Returns ``(lat, fwd, width, valid)``; ``valid`` is False when a ray does
    not hit the water (box bottom above the horizon).
    """
    c = p.cam
    th = c.pitch + dpitch
    cr, sr = math.cos(roll), math.sin(roll)
    X, Z = [0.0, 0.0], [0.0, 0.0]
    for i, uu in enumerate((bbox[0], bbox[2])):
        a0 = uu - c.cx
        b0 = bbox[3] - c.cy
        a = a0 * cr + b0 * sr
        b = -a0 * sr + b0 * cr
        dx = a / c.fx
        dy = b / c.fy
        Yd = dy * math.cos(th) + math.sin(th)
        Zf = -dy * math.sin(th) + math.cos(th)
        if Yd <= 1e-6:
            return 0.0, 0.0, 0.0, False
        t = c.h / Yd
        X[i] = t * dx
        Z[i] = t * Zf
    w = math.hypot(X[1] - X[0], Z[1] - Z[0])
    return (X[0] + X[1]) / 2, (Z[0] + Z[1]) / 2 + c.fwd, w, True


def is_truncated(bbox, p, margin: float = 2.0) -> bool:
    """True if the box touches the left, right or bottom image edge."""
    c = p.cam
    return bbox[0] <= margin or bbox[2] >= c.W - margin or bbox[3] >= c.H - margin
