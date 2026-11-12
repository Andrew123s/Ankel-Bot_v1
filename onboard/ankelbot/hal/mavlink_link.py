"""Vessel link to an ArduPilot Rover (boat, skid-steer) autopilot over MAVLink.

Architecture: the companion computer decides, the autopilot actuates and keeps
the safety functions (RC override by a human, arming checks, failsafes).

* Thrust: sent as RC overrides on the steering and throttle channels with the
  autopilot in MANUAL mode; ArduPilot's skid-steer mixer turns them into the
  two ESC outputs. ``throttle = (port + stbd) / 2``, ``steering = (port - stbd) / 2``.
* Wing actuators and reject rake: ``MAV_CMD_DO_SET_SERVO`` on spare outputs.
* State: ATTITUDE, GLOBAL_POSITION_INT, SYS_STATUS/BATTERY_STATUS and
  DISTANCE_SENSOR (mouth sensor, net-level sensor, forward rangers).

STATUS: the message encoding/decoding is exercised against a simulated
autopilot endpoint in ``tests/test_mavlink_link.py``. It has NOT been run
against a real autopilot or ArduPilot SITL. Before first use on the water:
confirm RCMAP/skid-steer mixing direction, servo output numbers, and the
roll/pitch sign options below on the bench with the propellers removed.

ArduPilot only accepts RC overrides from the ground-station system id
(``SYSID_MYGCS``, default 255), which is why this link identifies as 255.
"""
from __future__ import annotations

import math
import time

from .base import Command, VehicleState

EARTH_R = 6_371_000.0


class MavlinkVehicle:
    def __init__(self, p, hw, connect_timeout: float = 30.0):
        from pymavlink import mavutil

        self.mavutil = mavutil
        self.p, self.hw = p, hw
        self.m = mavutil.mavlink_connection(hw.url, baud=getattr(hw, "baud", 115200),
                                            source_system=getattr(hw, "gcs_sysid", 255), source_component=191)
        self._hb_sent = 0.0
        self._send_heartbeat()
        self.m.wait_heartbeat(timeout=connect_timeout)
        self.t0 = time.monotonic()
        self.origin = None
        self.s = VehicleState()
        self._seen = {"hb": time.monotonic(), "att": 0.0, "pos": 0.0}
        self._dist: dict[int, tuple[float, float]] = {}          # sensor id -> (metres, time)
        self._last_servo: dict[int, int] = {}

    # ---- setup helpers ----
    def set_mode(self, mode: str = "MANUAL") -> None:
        self.m.set_mode(mode)

    def arm(self, on: bool = True) -> None:
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if on else 0, 0, 0, 0, 0, 0, 0)

    # ---- Vehicle ----
    def read_state(self) -> VehicleState:
        now = time.monotonic()
        self._send_heartbeat()
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            self._handle(msg, now)
        hw, s = self.hw, self.s
        s.t = now - self.t0
        s.mouth_blocked = self._range(hw.mouth_sensor_id, now) < hw.mouth_block_m
        s.net_full = self._range(hw.net_sensor_id, now) < hw.net_full_m
        s.obst_range = min([self._range(i, now) for i in hw.obstacle_sensor_ids] or [99.0])
        s.healthy = (now - self._seen["hb"] < 2.0 and now - self._seen["att"] < 1.0 and now - self._seen["pos"] < 2.0)
        return s

    def send(self, c: Command) -> None:
        hw = self.hw
        thr = max(-1.0, min(1.0, (c.cmd_port + c.cmd_stbd) / 2))
        steer = max(-1.0, min(1.0, (c.cmd_port - c.cmd_stbd) / 2))
        ch = [65535] * 8                                          # 65535 = leave channel alone
        ch[hw.throttle_channel - 1] = int(round(1500 + 500 * thr))
        ch[hw.steer_channel - 1] = int(round(1500 + 500 * steer))
        self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *ch)
        self._servo(hw.servo_gate, hw.gate_closed_pwm if c.gate else hw.gate_open_pwm)
        self._servo(hw.servo_wing_port, self.wing_pwm(c.wing_port))
        self._servo(hw.servo_wing_stbd, self.wing_pwm(c.wing_stbd))

    def close(self) -> None:
        self.send(Command(0.0, 0.0, 1, self.p.wing_min, self.p.wing_min))
        self.m.close()

    # ---- internals ----
    def wing_pwm(self, angle: float) -> int:
        """Sweep angle -> linear-actuator PWM, from the hinge/actuator triangle."""
        hw, p = self.hw, self.p
        a = max(p.wing_min, min(p.wing_max, angle))

        def length(ang: float) -> float:
            bx = p.hinge_x + hw.act_arm * math.sin(ang)
            bz = p.hinge_fwd + hw.act_arm * math.cos(ang)
            return math.hypot(bx - hw.act_base[0], bz - hw.act_base[1])

        l_min, l_max = length(p.wing_max), length(p.wing_min)      # actuator is shortest at full sweep
        frac = (length(a) - l_min) / (l_max - l_min)
        return int(round(hw.act_pwm_retracted + frac * (hw.act_pwm_extended - hw.act_pwm_retracted)))

    def _servo(self, n: int, pwm: int) -> None:
        if self._last_servo.get(n) == pwm:
            return                                                # only send changes
        self._last_servo[n] = pwm
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     self.mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, n, pwm, 0, 0, 0, 0, 0)

    def _send_heartbeat(self) -> None:
        now = time.monotonic()
        if now - self._hb_sent >= 1.0:
            ml = self.mavutil.mavlink
            self.m.mav.heartbeat_send(ml.MAV_TYPE_GCS, ml.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._hb_sent = now

    def _range(self, sensor_id: int, now: float) -> float:
        d, t = self._dist.get(sensor_id, (99.0, 0.0))
        return d if now - t < 1.0 else 99.0                       # stale reading = nothing seen

    def _handle(self, msg, now: float) -> None:
        t, s, hw = msg.get_type(), self.s, self.hw
        if t == "HEARTBEAT":
            if msg.get_srcSystem() == self.m.target_system:
                self._seen["hb"] = now
        elif t == "ATTITUDE":
            s.psi, s.r = msg.yaw, msg.yawspeed
            s.roll = hw.roll_sign * msg.roll
            s.dpitch = hw.pitch_sign * msg.pitch
            self._seen["att"] = now
        elif t == "GLOBAL_POSITION_INT":
            lat, lon = math.radians(msg.lat / 1e7), math.radians(msg.lon / 1e7)
            if self.origin is None:
                self.origin = (lat, lon)
            s.x = (lat - self.origin[0]) * EARTH_R
            s.y = (lon - self.origin[1]) * EARTH_R * math.cos(self.origin[0])
            s.u = (msg.vx * math.cos(s.psi) + msg.vy * math.sin(s.psi)) / 100.0
            self._seen["pos"] = now
        elif t == "SYS_STATUS":
            if msg.battery_remaining >= 0:
                s.soc = float(msg.battery_remaining)
        elif t == "DISTANCE_SENSOR":
            self._dist[msg.id] = (msg.current_distance / 100.0, now)
