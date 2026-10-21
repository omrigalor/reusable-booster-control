# Reusable Booster Control Lab

A planar simulation of terminal guidance and stabilization for a reusable launch-vehicle booster. The controller combines a height-dependent descent profile with constrained feedback for horizontal error, attitude, angular rate, throttle, and gimbal.

```bash
python - <<'PY'
from booster import BoosterSimulator, landing_quality
trace = BoosterSimulator(seed=7).run()
print(landing_quality(trace[-1]))
PY
```

The dynamics include gravity, mass, fuel depletion, actuator saturation, and stochastic lateral gusts. Runs are seeded so controller changes can be compared against identical disturbances. A reproducible stress harness perturbs altitude, crossrange, velocity, attitude, angular rate, fuel, and wind, then reports the soft-landing rate and worst touchdown rather than showcasing one ideal descent.

## Distinctive edge

The controller combines a height-dependent stopping envelope with bounded
throttle, gimbal, translation, and attitude feedback. Its contribution is the
way landing guidance and validation are tied together: the same controller is
judged across a seeded uncertainty envelope, including actuator and fuel
margins, so a beautiful nominal trajectory cannot hide a brittle landing law.

`reliability_driven_tuning` adds a reliability-driven offline optimization
layer inspired by recent research on landing under stochastic winds. It searches
stopping-envelope and feedback parameters across one wind/state ensemble, then
reports performance on a separate holdout ensemble. That provides an honest
within-model comparison against the untuned baseline without claiming access to
or superiority over proprietary flight software.

## Engineering questions

- How should desired vertical speed change as altitude closes?
- When do gimbal and throttle saturation remove controllability?
- How sensitive is touchdown quality to wind, initial attitude, and fuel margin?
- Can a robust or model-predictive controller outperform the baseline under the same disturbance draws?

## Safety and scope

This is an educational, low-fidelity simulation inspired by reusable launch vehicles. It is **not SpaceX software**, is not affiliated with any aerospace company, and is not flight-ready guidance, navigation, or control software. It omits essential six-degree-of-freedom dynamics, navigation uncertainty, structural modes, engine transients, aerodynamics, verification, and hardware qualification.

## Tail-aware landing experiment (2026 research direction)

Run `python risk_benchmark.py` to reproduce `results/tail-risk-benchmark.json`
and the actual trajectory in `results/landing-replay.json` used by the portfolio
animation. The replay uses Earth gravity and synthetic atmospheric winds; this
is a reusable booster experiment, not a lunar descent model.

Recent work on [landing chance constraints (He, Zhang & Li, 2026)](https://hkxb.buaa.edu.cn/EN/10.7527/S1000-6893.2026.33338)
focuses on large, infrequent terminal errors that mean/covariance summaries can
hide. Inspired by that question, this repository adds:

- Persistent wind with occasional heavy-tailed impulses, alongside the existing Gaussian gusts.
- Empirical policy selection across 20 settings on 80 training scenarios.
- Evaluation on 200 separate scenarios, with identical winds for both controllers.
- Worst-decile touchdown speed, target miss, fuel use, and a Wilson 95% lower confidence bound on landing success.
- A success definition requiring a touchdown within 10 m, low vertical and horizontal speed, near-upright attitude, and fuel remaining.

This implementation is an empirical parameter search. It does **not** implement
the paper's polynomial-chaos surrogate, neural guidance, reinforcement learning,
or a certified chance-constrained optimizer.

### Recorded result and tradeoffs

With seed 101 and holdout seed 100101, the selected controller landed successfully
in 111/200 cases (55.5%), versus 103/200 (51.5%) for the baseline. Worst-decile
speed changed from 3.69 to 3.61 m/s. Mean fuel used increased from 46.9% to 50.6%
of normalized full capacity, and worst target miss worsened from 42.14 to 43.40 m.
These modest differences are descriptive, not proof of a statistically reliable
advantage. The low success rates expose an unresolved crosswind-control problem.
All outcomes, including these weaknesses, remain in the report.

[Onboard Dual Quaternion Guidance (Kamath et al., 2025)](https://arxiv.org/abs/2508.10439)
is another relevant direction: real-time six-degree-of-freedom guidance and
hardware-in-the-loop testing. That solver is not implemented here. Neither this
study nor its references establish superiority over SpaceX or Blue Origin, or
that a method is unused by commercial teams.
