"""Tail-aware landing calibration inspired by quantile-constrained PDG research.

This is a small empirical policy search, not a reproduction of a neural/PCE
solver and not a comparison against proprietary launch-vehicle software.
"""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import random
from booster import BoosterSimulator, LandingController, State


class BurstWindSimulator(BoosterSimulator):
    """Persistent gust acceleration with occasional heavy-tailed impulses."""
    def __init__(self, seed=0, dt=0.05, gust_sigma=0.35):
        super().__init__(seed, dt, gust_sigma)
        self.wind = 0.0
        self.wind_rng = random.Random(seed + 9_000_000)

    def step(self, state, command):
        # Separate RNG leaves the original Gaussian draws identical across policies.
        self.wind *= math.exp(-self.dt / 1.5)
        if self.wind_rng.random() < 0.12 * self.dt:
            self.wind += self.wind_rng.choice((-1, 1)) * min(5.0, self.wind_rng.paretovariate(3))
        next_state = super().step(state, command)
        next_state.vx += self.wind * self.dt
        return next_state


def scenario_states(trials, seed):
    rng = random.Random(seed)
    return [State(x=rng.uniform(70,160), z=rng.uniform(720,1050),
                  vx=rng.uniform(-12,-3), vz=rng.uniform(-62,-44),
                  theta=rng.uniform(-.11,.14), omega=rng.uniform(-.025,.025),
                  fuel=rng.uniform(.82,1)) for _ in range(trials)]


def upper_tail_mean(values, fraction=.1):
    """Mean of the worst ceil(fraction*n) observations (empirical CVaR)."""
    if not values or not 0 < fraction <= 1:
        raise ValueError('nonempty values and fraction in (0, 1] required')
    count = max(1, math.ceil(len(values)*fraction))
    return sum(sorted(values, reverse=True)[:count]) / count


def wilson_lower(successes, trials):
    if trials < 1 or not 0 <= successes <= trials:
        raise ValueError('invalid binomial counts')
    z = 1.96
    p = successes / trials
    return (p + z*z/(2*trials) - z*math.sqrt(p*(1-p)/trials+z*z/(4*trials*trials))) / (1+z*z/trials)


@dataclass(frozen=True)
class RiskScore:
    trials: int
    successful_landings: int
    success_rate: float
    success_lower_95: float
    tail_touchdown_speed: float
    worst_pad_error: float
    mean_fuel_used: float
    minimum_fuel_remaining: float


def evaluate(controller, trials=80, seed=101):
    if trials < 1:
        raise ValueError('trials must be positive')
    states = scenario_states(trials, seed)
    finals = [BurstWindSimulator(seed+i).run(s, controller=controller)[-1]
              for i,s in enumerate(states)]
    successes = sum(s.z <= .01 and abs(s.x) <= 10 and abs(s.vz) < 6
                    and abs(s.vx) < 3 and abs(s.theta) < .15 and s.fuel > 0 for s in finals)
    return RiskScore(trials, successes, successes/trials, wilson_lower(successes,trials),
                     upper_tail_mean([math.hypot(s.vx,s.vz) for s in finals]),
                     max(abs(s.x) for s in finals),
                     sum(a.fuel-b.fuel for a,b in zip(states, finals))/trials,
                     min(s.fuel for s in finals))


def tail_aware_tuning(training_trials=80, seed=101):
    """Select on training only; holdout results cannot influence selection.

    First retain the highest observed success count, then minimize worst-decile
    speed, then fuel. Empirical success is not a certified chance constraint.
    """
    candidates = [LandingController(f,g) for f in (1.2,1.5,1.8,2.1,2.5)
                  for g in (.7,.9,1.1,1.3)]
    scored = [(c, evaluate(c, training_trials, seed)) for c in candidates]
    return min(scored, key=lambda item:(-item[1].successful_landings,
                                       item[1].tail_touchdown_speed,
                                       item[1].mean_fuel_used))


def experiment(training_trials=80, holdout_trials=200, seed=101):
    selected, training = tail_aware_tuning(training_trials, seed)
    holdout_seed=seed+100_000
    report = {
        'method':'Empirical tail-aware policy selection; 20 candidates',
        'seed':seed, 'holdout_seed':holdout_seed,
        'selected':vars(selected), 'training':asdict(training),
        'baseline_holdout':asdict(evaluate(LandingController(),holdout_trials,holdout_seed)),
        'selected_holdout':asdict(evaluate(selected,holdout_trials,holdout_seed)),
        'scope':'Planar Earth-gravity simulation, fixed mass, synthetic winds; no hardware or company comparison',
    }
    return selected, report


if __name__ == '__main__':
    controller, report = experiment()
    folder=Path('results');folder.mkdir(exist_ok=True)
    (folder/'tail-risk-benchmark.json').write_text(json.dumps(report,indent=2)+'\n')
    sim=BurstWindSimulator(seed=37)
    trace=sim.run(controller=controller)
    indices=list(range(0,len(trace),5))
    if indices[-1] != len(trace)-1:
        indices.append(len(trace)-1)
    replay={'dt':sim.dt,'seed':37,'controller':vars(controller),
            'scope':'Actual model trace; playback accelerated; drawing axes scaled independently',
            'frames':[{**{k:round(v,5) for k,v in asdict(trace[i]).items()},
                       't':round(i*sim.dt,2),
                       **{k:round(v,5) for k,v in asdict(controller.command(trace[i])).items()}}
                      for i in indices]}
    (folder/'landing-replay.json').write_text(json.dumps(replay,separators=(',',':'))+'\n')
    print(json.dumps(report,indent=2))
