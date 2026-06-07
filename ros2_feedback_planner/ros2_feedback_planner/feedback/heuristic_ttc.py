"""Pure time-to-collision helpers for the navigation heuristic baseline."""

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class TTCConfig:
    """Configuration for time-to-collision trigger decisions."""

    safety_distance_m: float = 0.5
    horizon_s: float = 1.0
    min_closing_speed_mps: float = 0.05


@dataclass(frozen=True)
class TTCObservation:
    """One TTC estimate produced from a range observation."""

    distance_m: float
    closing_speed_mps: float
    ttc_s: float | None
    should_trigger: bool


def closest_valid_range(
    ranges: Iterable[float],
    min_distance_m: float,
    max_distance_m: float,
) -> float | None:
    """Return the closest finite range inside the valid distance interval."""
    valid = [
        value
        for value in ranges
        if math.isfinite(value)
        and min_distance_m <= value <= max_distance_m
    ]
    if not valid:
        return None
    return min(valid)


def estimate_closing_speed(
    previous_distance_m: float | None,
    current_distance_m: float,
    dt_s: float,
) -> float:
    """Estimate positive closing speed from two distance observations."""
    if previous_distance_m is None or dt_s <= 0.0:
        return 0.0
    speed = (previous_distance_m - current_distance_m) / dt_s
    return max(0.0, speed)


def estimate_ttc_s(
    distance_m: float,
    closing_speed_mps: float,
    safety_distance_m: float,
    min_closing_speed_mps: float,
) -> float | None:
    """Estimate time until the obstacle reaches the safety distance."""
    clearance_m = distance_m - safety_distance_m
    if clearance_m <= 0.0:
        return 0.0
    if closing_speed_mps < min_closing_speed_mps:
        return None
    return clearance_m / closing_speed_mps


class TTCMonitor:
    """Stateful TTC trigger monitor for a single action execution."""

    def __init__(self, config: TTCConfig | None = None):
        """Initialize the monitor with optional TTC configuration."""
        self.config = config or TTCConfig()
        self.previous_distance_m: float | None = None
        self.previous_stamp_s: float | None = None
        self.has_triggered = False

    def reset(self):
        """Clear state between action executions."""
        self.previous_distance_m = None
        self.previous_stamp_s = None
        self.has_triggered = False

    def update(self, distance_m: float, stamp_s: float) -> TTCObservation:
        """Update the monitor and return the current TTC decision."""
        if self.previous_stamp_s is None:
            dt_s = 0.0
        else:
            dt_s = stamp_s - self.previous_stamp_s

        closing_speed_mps = estimate_closing_speed(
            self.previous_distance_m,
            distance_m,
            dt_s,
        )
        ttc_s = estimate_ttc_s(
            distance_m,
            closing_speed_mps,
            self.config.safety_distance_m,
            self.config.min_closing_speed_mps,
        )
        should_trigger = (
            not self.has_triggered
            and ttc_s is not None
            and ttc_s <= self.config.horizon_s
        )

        if should_trigger:
            self.has_triggered = True

        self.previous_distance_m = distance_m
        self.previous_stamp_s = stamp_s

        return TTCObservation(
            distance_m=distance_m,
            closing_speed_mps=closing_speed_mps,
            ttc_s=ttc_s,
            should_trigger=should_trigger,
        )
