"""End-to-end missions on the software-in-the-loop vessel.

Unlike the MATLAB simulation these run the real onboard tracker, which has to
associate detections with objects itself.
"""
import math

from ankelbot.hal.sim import Scenario, SimVehicle
from ankelbot.mission import Mission
from ankelbot.storage import Store
from ankelbot.supervisor import State


def run(p, sc, store=None):
    veh = SimVehicle(p, sc)
    m = Mission(p, veh, veh, store)
    log = []
    m.run(duration=sc.T, realtime=False, on_step=lambda s, out, cmd: log.append((s.t, out.state, cmd, veh.gate)))
    return veh, m, log


def test_calm_water_collects_waste(p):
    veh, m, _ = run(p, Scenario())
    assert math.hypot(*m.tracker.drift) == 0.0          # no current: no phantom drift
    r = veh.summary()
    assert r["jams"] == 0
    assert r["collected_pct"] > 85


def test_oversize_objects_never_jam_the_tunnel(p):
    for seed in (1, 2, 3):
        veh, _, _ = run(p, Scenario(n_over=6, n_border=4, seed=seed))
        r = veh.summary()
        assert r["jams"] == 0, f"seed {seed}"
        assert r["collected_pct"] > 50, f"seed {seed}"


def test_drift_is_estimated_and_waste_still_collected(p):
    veh, m, _ = run(p, Scenario(n_over=4, current=(0.02, 0.06)))
    assert veh.summary()["jams"] == 0
    assert veh.summary()["collected_pct"] > 60
    assert math.hypot(m.tracker.drift_raw[0] - 0.02, m.tracker.drift_raw[1] - 0.06) < 0.04


def test_low_battery_returns_to_dock(p):
    veh, m, _ = run(p, Scenario(soc0=20.3, T=500))
    assert m.supervisor.state == State.DOCKED
    assert math.hypot(veh.x[0] - p.sup.dock[0], veh.x[1] - p.sup.dock[1]) < 3.0


def test_health_loss_stops_thrusters_and_closes_rake(p):
    veh, m, log = run(p, Scenario(fail_time=120, T=200))
    assert m.supervisor.state == State.FAILSAFE
    after = [row for row in log if row[0] > 121]
    assert all(row[2] == (0.0, 0.0) for row in after)
    assert all(row[3] == 1.0 for row in log if row[0] > 123)


def test_mission_is_logged_locally(p, tmp_path):
    store = Store(tmp_path / "log.sqlite")
    run(p, Scenario(T=60), store)
    assert store.count("telemetry") > 100
    assert store.count("detections") > 0
    assert store.count("events") >= 1
    store.close()
