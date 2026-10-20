"""Planar reusable-booster landing simulation. Educational research code only."""

from dataclasses import dataclass
import math
import random


@dataclass
class State:
    x: float = 120.0
    z: float = 900.0
    vx: float = -8.0
    vz: float = -55.0
    theta: float = 0.08
    omega: float = 0.0
    fuel: float = 1.0


@dataclass(frozen=True)
class Command:
    throttle: float
    gimbal: float


class LandingController:
    """Gain-scheduled feedback controller with thrust/gimbal constraints."""

    def command(self, s: State) -> Command:
        # A suicide-burn-style envelope: high descent rate aloft, tapering as sqrt(altitude).
        desired_vz = -min(55.0, max(1.5, math.sqrt(max(0.0, 4.0 * s.z))))
        vertical_error = desired_vz - s.vz
        horizontal_target = -0.045 * s.x - 0.55 * s.vx
        desired_theta = max(-0.18, min(0.18, horizontal_target / 35.0))
        gimbal = 2.2 * (desired_theta - s.theta) - 0.75 * s.omega
        desired_az = max(-4.0, min(7.0, 0.35 * vertical_error))
        throttle = 22_000.0 * (9.81 + desired_az) / (430_000.0 * max(0.92, math.cos(s.theta)))
        return Command(max(0.0, min(1.0, throttle)), max(-0.12, min(0.12, gimbal)))


class BoosterSimulator:
    def __init__(self, seed: int = 0, dt: float = 0.05):
        self.rng, self.dt = random.Random(seed), dt
        self.mass, self.max_thrust, self.inertia = 22_000.0, 430_000.0, 1.8e6

    def step(self, s: State, u: Command) -> State:
        dt = self.dt
        gust = self.rng.gauss(0.0, 0.35)
        thrust = self.max_thrust * u.throttle if s.fuel > 0 else 0.0
        angle = s.theta + u.gimbal
        ax = thrust * math.sin(angle) / self.mass + gust
        az = thrust * math.cos(angle) / self.mass - 9.81
        alpha = 1.7 * u.gimbal - 0.12 * s.omega
        burn = thrust / (300 * 9.81) * dt / 7_000
        return State(s.x + s.vx * dt, max(0.0, s.z + s.vz * dt), s.vx + ax * dt,
                     s.vz + az * dt, s.theta + s.omega * dt, s.omega + alpha * dt,
                     max(0.0, s.fuel - burn))

    def run(self, state: State | None = None, seconds: float = 80.0):
        controller, s = LandingController(), state or State()
        trace = [s]
        for _ in range(int(seconds / self.dt)):
            if s.z <= 0:
                break
            s = self.step(s, controller.command(s))
            trace.append(s)
        return trace


def landing_quality(state: State) -> dict[str, float | bool]:
    return {"landed": state.z <= 0.01, "speed": math.hypot(state.vx, state.vz),
            "tilt_deg": abs(math.degrees(state.theta)),
            "soft": state.z <= 0.01 and abs(state.vz) < 6 and abs(state.vx) < 3 and abs(state.theta) < 0.15}
