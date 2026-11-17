"""Absolute joint-position control for tools with multiple independent coordinates."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from manimux.embodiments.end_effector.base import EndEffectorBase
from manimux.kinematics.base import FloatArray, KinematicCoordinate


def _position(value: FloatArray) -> FloatArray:
    position = np.array(value, dtype=np.float64, copy=True)
    if position.ndim != 1 or not position.size or not np.isfinite(position).all():
        raise ValueError("joint tool position must be a non-empty finite vector")
    position.setflags(write=False)
    return position


@dataclass(frozen=True, slots=True)
class JointToolState:
    """Measured positions in the tool's declared coordinate order and units.

    timestamp is seconds in the station Clock's monotonic domain. Drivers must
    document acquisition versus host receipt, preserve timestamps and sample
    sequences on cached reads, and reject unavailable or stale feedback.
    sequence identifies a received sample; it is not a count of get_state calls.
    """

    position: FloatArray
    timestamp: float
    sequence: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _position(self.position))
        if not math.isfinite(self.timestamp):
            raise ValueError("timestamp must be finite")
        if (
            not isinstance(self.sequence, int)
            or isinstance(self.sequence, bool)
            or self.sequence < 0
        ):
            raise ValueError("sequence must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class JointToolCommand:
    """Absolute position targets; submission does not imply measured arrival."""

    position: FloatArray

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _position(self.position))


class JointToolBase(EndEffectorBase[JointToolState, JointToolCommand]):
    """A position-controlled tool whose coordinates use radians or metres.

    Coordinates and command limits describe the same order as the offline tool
    model. They are explicit configuration, not hardware discovery. This base
    opens no SDK connection and provides no enabling, Home or release behavior.
    Drivers own their lifecycle and call validation before submitting commands.
    Normalized gripper travel belongs to GripperBase, not this capability.
    """

    def __init__(
        self,
        *,
        coordinates: tuple[KinematicCoordinate, ...],
        joint_limits: FloatArray,
    ) -> None:
        layout = tuple(coordinates)
        if not layout or len({item.name for item in layout}) != len(layout):
            raise ValueError("joint tool coordinates must be non-empty and have unique names")
        if any(item.unit not in {"rad", "m"} for item in layout):
            raise ValueError("joint tool coordinates must use radians or metres")
        limits = np.array(joint_limits, dtype=np.float64, copy=True)
        if (
            limits.shape != (len(layout), 2)
            or not np.isfinite(limits).all()
            or np.any(limits[:, 0] > limits[:, 1])
        ):
            raise ValueError("joint tool limits must contain finite ordered lower/upper pairs")
        limits.setflags(write=False)
        self._coordinates = layout
        self._joint_limits = limits

    @property
    def coordinates(self) -> tuple[KinematicCoordinate, ...]:
        return self._coordinates

    @property
    def num_joints(self) -> int:
        return len(self.coordinates)

    def command_bounds(self) -> FloatArray:
        """Return lower/upper pairs in coordinate units, without hardware I/O."""
        return self._joint_limits.copy()

    def validate_state(self, state: JointToolState) -> None:
        """Check feedback width without clipping or replacing measured positions.

        Command bounds are not applied to feedback: drivers must report actual
        positions and document device-specific treatment of limit excursions.
        """
        if state.position.shape != (self.num_joints,):
            raise ValueError(f"joint tool feedback must have shape ({self.num_joints},)")

    def validate_command(self, command: JointToolCommand) -> None:
        """Check dimensions and declared position limits without hardware I/O."""
        position = command.position
        if position.shape != (self.num_joints,):
            raise ValueError(f"joint tool target must have shape ({self.num_joints},)")
        if np.any(position < self._joint_limits[:, 0]) or np.any(
            position > self._joint_limits[:, 1]
        ):
            raise ValueError("joint tool target exceeds joint limits")
