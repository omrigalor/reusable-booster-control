import math
import pytest
from booster import Command, LandingController, State
from risk_benchmark import BurstWindSimulator, evaluate, upper_tail_mean, wilson_lower


def test_tail_score_exposes_rare_large_errors():
    assert upper_tail_mean([1]*9+[30]) == 30
    assert upper_tail_mean([1]*9+[30],1) == 3.9
    with pytest.raises(ValueError):
        upper_tail_mean([])


def test_confidence_bound_does_not_certify_perfect_small_sample():
    assert wilson_lower(10,10) < .75
    assert wilson_lower(1000,1000) > .99
    assert wilson_lower(5,10) < .5


def test_wind_replay_is_repeatable_and_persistent():
    a,b=BurstWindSimulator(37),BurstWindSimulator(37)
    sa,sb=State(),State()
    seen=[]
    for _ in range(1000):
        sa=a.step(sa,Command(.5,0));sb=b.step(sb,Command(.5,0))
        assert sa == sb
        seen.append(a.wind)
    assert any(abs(v)>.5 for v in seen)
    assert any(abs(seen[i])>.5 and abs(seen[i+1])>.5 for i in range(len(seen)-1))


def test_evaluation_includes_target_error_and_fuel():
    result=evaluate(LandingController(),trials=12,seed=100101)
    assert result == evaluate(LandingController(),trials=12,seed=100101)
    assert 0 <= result.success_lower_95 <= result.success_rate <= 1
    assert result.worst_pad_error > 0
    assert 0 < result.minimum_fuel_remaining < 1
    assert math.isfinite(result.tail_touchdown_speed)
