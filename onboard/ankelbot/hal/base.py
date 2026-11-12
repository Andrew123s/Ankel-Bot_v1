"""Interfaces between the mission logic and the outside world."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass
class VehicleState:
    t: float = 0.0
    x: float = 0.0              # north of origin [m]
    y: float = 0.0              # east of origin [m]
    psi: float = 0.0            # heading, clockwise from north [rad]
    u: float = 0.0              # ground speed along heading [m/s]
    r: float = 0.0              # yaw rate [rad/s]
    roll: float = 0.0           # image-plane roll used by geometry.estimate [rad]
    dpitch: float = 0.0         # extra camera tilt from vessel pitch [rad]
    soc: float = 100.0          # battery [%]
    obst_range: float = 99.0    # nearest obstacle ahead [m]
    mouth_blocked: bool = False
    net_full: bool = False
    healthy: bool = True


@dataclass
class Detection:
    bbox: tuple                 # (u1, v1, u2, v2) in calibration-image pixels
    label: str = ""
    conf: float = 0.0
    over_class: bool = False    # class that must never enter the tunnel


@dataclass
class Command:
    cmd_port: float = 0.0       # ESC command [-1, 1]
    cmd_stbd: float = 0.0
    gate: int = 1               # 1 = reject rake closed
    wing_port: float = 0.0      # sweep angle [rad]
    wing_stbd: float = 0.0


class Vehicle(Protocol):
    def read_state(self) -> VehicleState: ...
    def send(self, c: Command) -> None: ...
    def close(self) -> None: ...


class Perception(Protocol):
    def detect(self) -> list[Detection]: ...
