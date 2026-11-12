"""MAVLink encoding/decoding against a simulated autopilot endpoint on TCP loopback.

This checks that our messages are well-formed and that incoming telemetry is
parsed into the right units and frames. It is NOT a test against ArduPilot.
"""
import math
import time

import pytest

mavutil = pytest.importorskip("pymavlink.mavutil")

from ankelbot.config import load_onboard  # noqa: E402
from ankelbot.hal.base import Command  # noqa: E402
from ankelbot.hal.mavlink_link import MavlinkVehicle  # noqa: E402

PORT = 14655


def test_link_round_trip(p):
    hw = load_onboard().mavlink
    hw.url = f"tcp:127.0.0.1:{PORT}"
    ap = mavutil.mavlink_connection(f"tcpin:127.0.0.1:{PORT}", source_system=1, source_component=1)
    ml = mavutil.mavlink

    def autopilot_heartbeat():
        ap.mav.heartbeat_send(ml.MAV_TYPE_SURFACE_BOAT, ml.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, ml.MAV_STATE_ACTIVE)

    # the vehicle link connects and sends first; the fake autopilot then answers
    import threading

    def answer():
        ap.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
        autopilot_heartbeat()

    th = threading.Thread(target=answer, daemon=True)
    th.start()
    veh = MavlinkVehicle(p, hw, connect_timeout=10)
    th.join()

    # --- telemetry in ---
    lat0, lon0 = 5.6037, -0.1870
    north, east = 12.0, -7.0
    lat = lat0 + math.degrees(north / 6_371_000.0)
    lon = lon0 + math.degrees(east / (6_371_000.0 * math.cos(math.radians(lat0))))
    yaw = 0.5
    ap.mav.global_position_int_send(0, int(lat0 * 1e7), int(lon0 * 1e7), 0, 0, 0, 0, 0, 0)
    ap.mav.attitude_send(0, 0.04, -0.02, yaw, 0, 0, 0.1)
    time.sleep(0.2)
    veh.read_state()
    ap.mav.global_position_int_send(0, int(round(lat * 1e7)), int(round(lon * 1e7)), 0, 0,
                                    int(100 * 0.5 * math.cos(yaw)), int(100 * 0.5 * math.sin(yaw)), 0, 0)
    ap.mav.sys_status_send(0, 0, 0, 0, 25000, 500, 63, 0, 0, 0, 0, 0, 0)
    ap.mav.distance_sensor_send(0, 5, 400, 12, 0, hw.mouth_sensor_id, 0, 0)
    ap.mav.distance_sensor_send(0, 5, 400, 90, 0, hw.obstacle_sensor_ids[0], 0, 0)
    autopilot_heartbeat()
    time.sleep(0.3)
    s = veh.read_state()
    assert s.x == pytest.approx(north, abs=0.05)
    assert s.y == pytest.approx(east, abs=0.05)
    assert s.psi == pytest.approx(yaw, abs=1e-5)
    assert s.r == pytest.approx(0.1, abs=1e-5)
    assert s.u == pytest.approx(0.5, abs=0.02)
    assert s.roll == pytest.approx(hw.roll_sign * 0.04, abs=1e-5)
    assert s.dpitch == pytest.approx(hw.pitch_sign * -0.02, abs=1e-5)
    assert s.soc == 63
    assert s.mouth_blocked is True              # 0.12 m < mouth_block_m
    assert s.net_full is False                  # no reading from that sensor
    assert s.obst_range == pytest.approx(0.90, abs=1e-6)
    assert s.healthy is True

    # --- commands out ---
    veh.send(Command(cmd_port=0.6, cmd_stbd=0.2, gate=1, wing_port=p.wing_min, wing_stbd=p.wing_max))
    rc, servos = None, {}
    end = time.time() + 3
    while time.time() < end and (rc is None or len(servos) < 3):
        m = ap.recv_match(blocking=True, timeout=0.5)
        if m is None:
            continue
        if m.get_type() == "RC_CHANNELS_OVERRIDE":
            rc = m
        elif m.get_type() == "COMMAND_LONG" and m.command == ml.MAV_CMD_DO_SET_SERVO:
            servos[int(m.param1)] = int(m.param2)
    assert rc is not None
    chans = [rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw]
    assert chans[hw.throttle_channel - 1] == 1700          # mean of 0.6 and 0.2
    assert chans[hw.steer_channel - 1] == 1600             # half the difference
    assert chans[1] == 65535 and chans[3] == 65535         # untouched channels are left alone
    assert servos[hw.servo_gate] == hw.gate_closed_pwm
    assert servos[hw.servo_wing_port] == hw.act_pwm_extended     # narrowest sweep = longest actuator
    assert servos[hw.servo_wing_stbd] == hw.act_pwm_retracted

    # --- stale telemetry must read as unhealthy ---
    time.sleep(2.2)
    assert veh.read_state().healthy is False
    veh.m.close()
    ap.close()
