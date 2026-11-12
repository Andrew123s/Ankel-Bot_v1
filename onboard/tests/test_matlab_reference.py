"""The onboard Python code must reproduce the MATLAB reference model."""
import math

import pytest

from ankelbot import geometry, plant
from ankelbot.control import Controller, cmd_to_thrust, thrust_to_cmd
from ankelbot.supervisor import Supervisor, SupervisorInput
from ankelbot.tracker import classify

TOL = 1e-9


def test_thrust_map(p, ref):
    t = ref["thrust"]
    for T, c in zip(t["T"], t["cmd"]):
        assert thrust_to_cmd(T, p) == pytest.approx(c, abs=TOL)
    for c, T in zip(t["c"], t["T_of_c"]):
        assert cmd_to_thrust(c, p) == pytest.approx(T, abs=TOL)


def test_controller(p, ref):
    for row in ref["control"]:
        iu, psi_ref, u_ref, psi, u, r, cp, cs, iu_out = row
        c = Controller(p)
        c.iu = iu
        got = c.step(psi_ref, u_ref, psi, u, r)
        assert got[0] == pytest.approx(cp, abs=TOL)
        assert got[1] == pytest.approx(cs, abs=TOL)
        assert c.iu == pytest.approx(iu_out, abs=TOL)


def test_camera_projection_and_size_estimate(p, ref):
    n_ok = 0
    for row in ref["camera"]:
        lat, fwd, w, h, roll, pitch, u1, v1, u2, v2, ok, trunc, el, ef, ew, valid = row
        bb, got_ok, got_trunc = geometry.project(lat, fwd, w, h, p, roll, pitch)
        assert got_ok == bool(ok)
        if not ok:
            continue
        n_ok += 1
        assert got_trunc == bool(trunc)
        assert bb == pytest.approx((u1, v1, u2, v2), abs=1e-6)
        gl, gf, gw, gv = geometry.estimate(bb, p, roll, pitch)
        assert gv == bool(valid)
        assert (gl, gf, gw) == pytest.approx((el, ef, ew), abs=1e-6)
    assert n_ok > 100


def test_classifier(p, ref):
    for c in ref["classify"]:
        widths = c["widths"] if isinstance(c["widths"], list) else [c["widths"]]
        widths = [math.nan if v is None else v for v in widths]
        assert classify(widths, c["n"], c["votes"], p) == c["kind"]


def test_plant(p, ref):
    pl = ref["plant"]
    x, T = [0.0] * 6, [0.0, 0.0]
    for row in pl["trace"]:
        x, T, P = plant.step(x, T, row[0:2], p, pl["fill"], pl["wings"], pl["current"])
        assert x == pytest.approx(row[2:8], abs=1e-8)
        assert T == pytest.approx(row[8:10], abs=1e-8)
        assert P == pytest.approx(row[10], abs=1e-8)


def test_supervisor_replays_matlab_mission(p, ref):
    """Every decision of a 240 s MATLAB mission, step by step."""
    sup = Supervisor(p)
    seen = set()
    for k, step in enumerate(ref["supervisor"]):
        i, o = step["in"], step["out"]
        out = sup.step(SupervisorInput(
            x=i["x"], y=i["y"], psi=i["psi"], tracks=i["tracks"], area_shift=tuple(i["area_shift"]),
            net_full=bool(i["net_full"]), soc=i["soc"], obst_range=i["obst_range"],
            mouth_blocked=bool(i["mouth_blocked"]), healthy=bool(i["healthy"])))
        assert int(out.state) == o["state"], f"state differs at step {k}"
        assert out.gate == o["gate"], f"rake differs at step {k}"
        assert out.enable == o["enable"]
        assert out.u_ref == pytest.approx(o["u_ref"], abs=TOL)
        assert out.psi_ref == pytest.approx(o["psi_ref"], abs=1e-7)
        assert out.wing == pytest.approx(o["wing"], abs=TOL)
        seen.add(int(out.state))
    assert seen == {1, 2, 3, 4, 5, 6, 7}, "reference mission must visit every state"
