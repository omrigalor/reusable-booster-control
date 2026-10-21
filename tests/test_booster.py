from booster import BoosterSimulator, State, landing_quality, reliability_driven_tuning, stress_test


def test_simulation_is_deterministic_for_a_seed():
    a = BoosterSimulator(seed=7).run()
    b = BoosterSimulator(seed=7).run()
    assert a == b
    assert a[-1].z == 0


def test_controller_improves_over_ballistic_fall():
    trace = BoosterSimulator(seed=2).run(State(x=60, z=500, vx=-3, vz=-35))
    result = landing_quality(trace[-1])
    assert result["landed"]
    assert result["soft"] is True


def test_seeded_stress_test_measures_worst_case_not_one_trajectory():
    result = stress_test(trials=24, seed=9)
    assert result == stress_test(trials=24, seed=9)
    assert result.soft_landing_rate >= 0.9
    assert result.worst_touchdown_speed < 6
    assert result.minimum_fuel_margin > 0


def test_reliability_tuning_uses_a_separate_holdout_ensemble():
    result = reliability_driven_tuning(training_trials=12, holdout_trials=16, seed=3)
    assert result.candidates_evaluated == 20
    assert result.training.trials == 12
    assert result.holdout.trials == 16
    assert result.holdout.soft_landing_rate >= 0.8
