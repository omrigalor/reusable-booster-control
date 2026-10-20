from booster import BoosterSimulator, State, landing_quality


def test_simulation_is_deterministic_for_a_seed():
    a = BoosterSimulator(seed=7).run()
    b = BoosterSimulator(seed=7).run()
    assert a == b
    assert a[-1].z == 0


def test_controller_improves_over_ballistic_fall():
    trace = BoosterSimulator(seed=2).run(State(x=60, z=500, vx=-3, vz=-35))
    result = landing_quality(trace[-1])
    assert result["landed"]
    assert result["speed"] < 35
