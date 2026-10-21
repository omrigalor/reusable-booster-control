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

    def __init__(self, stopping_factor: float = 1.8, vertical_gain: float = 1.0):
        self.stopping_factor = stopping_factor
        self.vertical_gain = vertical_gain

    def command(self, s: State) -> Command:
        # A suicide-burn-style envelope: high descent rate aloft, tapering as sqrt(altitude).
        # A conservative stopping envelope begins braking early enough to leave
        # authority for gust rejection near the pad.
        desired_vz = -min(55.0, max(0.8, math.sqrt(max(0.0, self.stopping_factor * s.z))))
        vertical_error = desired_vz - s.vz
        horizontal_target = -0.045 * s.x - 0.55 * s.vx
        desired_theta = max(-0.18, min(0.18, horizontal_target / 35.0))
        gimbal = 2.2 * (desired_theta - s.theta) - 0.75 * s.omega
        desired_az = max(-4.0, min(9.0, self.vertical_gain * vertical_error))
        throttle = 22_000.0 * (9.81 + desired_az) / (430_000.0 * max(0.92, math.cos(s.theta)))
        return Command(max(0.0, min(1.0, throttle)), max(-0.12, min(0.12, gimbal)))


class BoosterSimulator:
    def __init__(self, seed: int = 0, dt: float = 0.05, gust_sigma: float = 0.35):
        self.rng, self.dt = random.Random(seed), dt
        self.gust_sigma = gust_sigma
        self.mass, self.max_thrust, self.inertia = 22_000.0, 430_000.0, 1.8e6

    def step(self, s: State, u: Command) -> State:
        dt = self.dt
        gust = self.rng.gauss(0.0, self.gust_sigma)
        thrust = self.max_thrust * u.throttle if s.fuel > 0 else 0.0
        angle = s.theta + u.gimbal
        ax = thrust * math.sin(angle) / self.mass + gust
        az = thrust * math.cos(angle) / self.mass - 9.81
        alpha = 1.7 * u.gimbal - 0.12 * s.omega
        burn = thrust / (300 * 9.81) * dt / 7_000
        return State(s.x + s.vx * dt, max(0.0, s.z + s.vz * dt), s.vx + ax * dt,
                     s.vz + az * dt, s.theta + s.omega * dt, s.omega + alpha * dt,
                     max(0.0, s.fuel - burn))

    def run(self, state: State | None = None, seconds: float = 80.0, controller: LandingController | None = None):
        controller, s = controller or LandingController(), state or State()
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


@dataclass(frozen=True)
class StressTestResult:
    trials: int
    soft_landings: int
    worst_touchdown_speed: float
    worst_tilt_deg: float
    minimum_fuel_margin: float

    @property
    def soft_landing_rate(self) -> float:
        return self.soft_landings / self.trials


def stress_test(
    trials: int = 100,
    seed: int = 17,
    controller: LandingController | None = None,
    gust_sigma: float = 0.35,
) -> StressTestResult:
    """Replay one controller across wind and initial-state uncertainty."""
    if trials < 1:
        raise ValueError("trials must be positive")
    scenarios = random.Random(seed)
    outcomes: list[tuple[State, dict[str, float | bool]]] = []
    for index in range(trials):
        state = State(
            x=scenarios.uniform(70.0, 160.0),
            z=scenarios.uniform(720.0, 1050.0),
            vx=scenarios.uniform(-12.0, -3.0),
            vz=scenarios.uniform(-62.0, -44.0),
            theta=scenarios.uniform(-0.11, 0.14),
            omega=scenarios.uniform(-0.025, 0.025),
            fuel=scenarios.uniform(0.82, 1.0),
        )
        final = BoosterSimulator(seed=seed + index, gust_sigma=gust_sigma).run(
            state, controller=controller
        )[-1]
        outcomes.append((final, landing_quality(final)))
    return StressTestResult(
        trials=trials,
        soft_landings=sum(bool(quality["soft"]) for _, quality in outcomes),
        worst_touchdown_speed=max(float(quality["speed"]) for _, quality in outcomes),
        worst_tilt_deg=max(float(quality["tilt_deg"]) for _, quality in outcomes),
        minimum_fuel_margin=min(final.fuel for final, _ in outcomes),
    )


@dataclass(frozen=True)
class TuningResult:
    controller: LandingController
    training: StressTestResult
    holdout: StressTestResult
    candidates_evaluated: int


def reliability_driven_tuning(
    training_trials: int = 32,
    holdout_trials: int = 64,
    seed: int = 101,
    gust_sigma: float = 0.55,
) -> TuningResult:
    """Tune offline on one weather ensemble, then score an unseen holdout ensemble."""
    candidates = [
        LandingController(stopping_factor, vertical_gain)
        for stopping_factor in (1.2, 1.5, 1.8, 2.1, 2.5)
        for vertical_gain in (0.7, 0.9, 1.1, 1.3)
    ]
    scored: list[tuple[tuple[float, float, float], LandingController, StressTestResult]] = []
    for controller in candidates:
        result = stress_test(training_trials, seed, controller, gust_sigma)
        score = (
            result.soft_landing_rate,
            -result.worst_touchdown_speed,
            result.minimum_fuel_margin,
        )
        scored.append((score, controller, result))
    _, selected, training = max(scored, key=lambda item: item[0])
    holdout = stress_test(
        holdout_trials, seed + 100_000, selected, gust_sigma
    )
    return TuningResult(selected, training, holdout, len(candidates))
