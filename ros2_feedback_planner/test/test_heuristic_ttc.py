"""Tests for the heuristic TTC navigation baseline."""

import importlib
import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

heuristic_ttc = importlib.import_module(
    'ros2_feedback_planner.feedback.heuristic_ttc'
)
TTCConfig = heuristic_ttc.TTCConfig
TTCMonitor = heuristic_ttc.TTCMonitor
closest_valid_range = heuristic_ttc.closest_valid_range
estimate_closing_speed = heuristic_ttc.estimate_closing_speed
estimate_ttc_s = heuristic_ttc.estimate_ttc_s


def test_closest_valid_range_filters_invalid_and_out_of_bounds():
    """Check that invalid scan values and out-of-range values are ignored."""
    ranges = [math.inf, math.nan, 0.05, 4.5, 2.0, 1.2, 3.1]

    closest = closest_valid_range(
        ranges,
        min_distance_m=0.1,
        max_distance_m=3.0,
    )

    assert closest == 1.2


def test_closest_valid_range_returns_none_without_valid_values():
    """Check that an empty valid scan sector is represented as None."""
    ranges = [math.inf, math.nan, 0.05, 5.0]

    closest = closest_valid_range(
        ranges,
        min_distance_m=0.1,
        max_distance_m=3.0,
    )

    assert closest is None


def test_estimate_closing_speed_uses_only_approaching_motion():
    """Check that only decreasing distances produce positive closing speed."""
    assert estimate_closing_speed(2.0, 1.5, 1.0) == 0.5
    assert estimate_closing_speed(1.5, 2.0, 1.0) == 0.0
    assert estimate_closing_speed(2.0, 1.5, 0.0) == 0.0


def test_estimate_ttc_returns_none_when_not_closing_fast_enough():
    """Check that slow or static obstacles do not produce finite TTC."""
    ttc = estimate_ttc_s(
        distance_m=1.5,
        closing_speed_mps=0.01,
        safety_distance_m=0.5,
        min_closing_speed_mps=0.05,
    )

    assert ttc is None


def test_monitor_triggers_when_ttc_enters_prediction_horizon():
    """Check that approaching obstacles trigger inside the TTC horizon."""
    monitor = TTCMonitor(
        TTCConfig(
            safety_distance_m=0.5,
            horizon_s=1.0,
            min_closing_speed_mps=0.05,
        )
    )

    first = monitor.update(distance_m=2.0, stamp_s=10.0)
    second = monitor.update(distance_m=1.2, stamp_s=11.0)

    assert not first.should_trigger
    assert second.closing_speed_mps == 0.8
    assert second.ttc_s == pytest_approx(0.875)
    assert second.should_trigger


def test_monitor_does_not_trigger_for_receding_obstacle():
    """Check that receding obstacles do not trigger cancellation."""
    monitor = TTCMonitor(TTCConfig(safety_distance_m=0.5, horizon_s=1.0))

    monitor.update(distance_m=1.2, stamp_s=10.0)
    observation = monitor.update(distance_m=1.5, stamp_s=11.0)

    assert observation.closing_speed_mps == 0.0
    assert observation.ttc_s is None
    assert not observation.should_trigger


def test_monitor_triggers_immediately_inside_safety_distance():
    """Check that obstacles already inside the safety distance trigger."""
    monitor = TTCMonitor(TTCConfig(safety_distance_m=0.5, horizon_s=1.0))

    observation = monitor.update(distance_m=0.4, stamp_s=10.0)

    assert observation.ttc_s == 0.0
    assert observation.should_trigger


def test_monitor_triggers_only_once_until_reset():
    """Check that the monitor emits one trigger per action execution."""
    monitor = TTCMonitor(TTCConfig(safety_distance_m=0.5, horizon_s=1.0))

    monitor.update(distance_m=2.0, stamp_s=10.0)
    first_trigger = monitor.update(distance_m=1.2, stamp_s=11.0)
    second_trigger = monitor.update(distance_m=0.8, stamp_s=12.0)
    monitor.reset()
    monitor.update(distance_m=2.0, stamp_s=20.0)
    trigger_after_reset = monitor.update(distance_m=1.2, stamp_s=21.0)

    assert first_trigger.should_trigger
    assert not second_trigger.should_trigger
    assert trigger_after_reset.should_trigger


def pytest_approx(value):
    """Tiny wrapper keeps tests readable without importing pytest globally."""
    import pytest

    return pytest.approx(value)
