"""Shared arm-only Tianji Home parameters and cosine interpolation."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class TianjiHomeSettings:
    """Home motion settings authored in the robot assembly YAML."""

    control_hz: float
    peak_velocity_deg_s: float
    tolerance_deg: float
    settle_timeout_s: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"hardware.home_motion.{name} must be finite and positive")


def home_duration_s(start, targets, peak_velocity_deg_s: float) -> float:
    distance = max(float(np.max(np.abs(q - start[name]))) for name, q in targets.items())
    return (math.pi / 2.0) * distance / math.radians(peak_velocity_deg_s)


def cosine_fraction(elapsed_s: float, duration_s: float) -> float:
    return (
        1.0 if elapsed_s >= duration_s else 0.5 * (1.0 - math.cos(math.pi * elapsed_s / duration_s))
    )
