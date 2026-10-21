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
