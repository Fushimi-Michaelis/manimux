from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

from manimux.runtime.executors.base import ExecutorError
from manimux.types import GroupVector


@dataclass(frozen=True, slots=True)
class ScalarLimits:
    max_velocity: float | None
    max_acceleration: float | None
    position_limit_abs: float | None
    mode: Literal["per_joint", "isotropic"] = "per_joint"
    max_step_dt_s: float | None = None

    def velocity_bound(self, dt_s: float) -> float | None:
        """Cap the time redeemable as a position step, keeping nominal tick timing."""
        if self.max_velocity is None or self.max_step_dt_s is None:
            return self.max_velocity
        return self.max_velocity * min(dt_s, self.max_step_dt_s) / dt_s


def motion_sections(
    group: str,
    width: int,
    limits: ScalarLimits,
    gripper_index: int | None,
    joint_tool: dict | None,
) -> list[tuple[slice, ScalarLimits, int | None]]:
    """Keep isotropic arm limits independent of a trailing joint tool."""
    bounds = (joint_tool or {}).get("group_slices", {}).get(group)
    if bounds is None:
        return [(slice(None), limits, gripper_index)]
    start, stop = bounds
    if start <= 0 or stop != width or gripper_index is not None:
        raise ValueError(f"{group}: joint tool slice does not match the grouped coordinates")
    tool_limits = ScalarLimits(
        joint_tool["max_velocity"], joint_tool["max_acceleration"],
        joint_tool["position_limit_abs"], joint_tool["mode"], joint_tool["max_step_dt_s"],
    )
    return [(slice(0, start), limits, None), (slice(start, stop), tool_limits, None)]


def _scale_vector(values: np.ndarray, bound: float, independent_index: int | None) -> np.ndarray:
    """One scale per arm; an embedded gripper never determines that scale."""
    arm = values if independent_index is None else np.delete(values, independent_index)
    peak = float(np.max(np.abs(arm), initial=0.0))
    result = values.copy() if peak <= bound else values * (bound / peak)
    if independent_index is not None:
        result[independent_index] = np.clip(values[independent_index], -bound, bound)
    return result


def limit_velocity(
    desired: np.ndarray,
    previous: np.ndarray,
    dt_s: float,
    max_velocity: float | None,
    max_acceleration: float | None,
    max_closing_velocity: float | None = None,
    *,
    mode: Literal["per_joint", "isotropic"] = "per_joint",
    independent_index: int | None = None,
) -> np.ndarray:
    """Limit velocity, then its change; each isotropic stage scales one vector.

    With acceleration enabled the second stage preserves the direction of the
    velocity *change*, not necessarily the requested position increment.
    """
    velocity = np.asarray(desired, dtype=np.float64).copy()
    if max_closing_velocity is not None:
        velocity = np.maximum(velocity, -max_closing_velocity)
    if max_velocity is not None:
        velocity = (
            _scale_vector(velocity, max_velocity, independent_index)
            if mode == "isotropic"
            else np.clip(velocity, -max_velocity, max_velocity)
        )
    if max_acceleration is not None:
        delta = max_acceleration * dt_s
        velocity = (
            previous + _scale_vector(velocity - previous, delta, independent_index)
            if mode == "isotropic"
            else np.clip(velocity, previous - delta, previous + delta)
        )
    return velocity


def decelerate_velocity(
    velocity: np.ndarray,
    max_acceleration: float | None,
    dt_s: float,
    *,
    mode: Literal["per_joint", "isotropic"] = "per_joint",
    independent_index: int | None = None,
) -> np.ndarray:
    if max_acceleration is None:
        return np.zeros_like(velocity)
    if mode == "isotropic":
        return limit_velocity(
            np.zeros_like(velocity),
            velocity,
            dt_s,
            None,
            max_acceleration,
            mode=mode,
            independent_index=independent_index,
        )
    return np.sign(velocity) * np.maximum(np.abs(velocity) - max_acceleration * dt_s, 0.0)


def braking_velocity(distance: np.ndarray, acceleration: float, dt_s: float) -> np.ndarray:
    """Conservative speed that leaves room for this tick and a complete stop.

    Solve distance = v * dt + v**2 / (2*a). The extra tick avoids the
    discrete-time overshoot of the continuous sqrt(2*a*distance) rule.
    """
    distance = np.maximum(distance, 0.0)
    step = acceleration * dt_s
    # Rationalized form avoids cancellation very close to the target.
    with np.errstate(invalid="ignore"):
        speed = 2 * acceleration * distance / (
            np.sqrt(step * step + 2 * acceleration * distance) + step
        )
    return np.where(np.isposinf(distance), np.inf, speed)


def bounded_step(
    target: np.ndarray,
    previous: np.ndarray,
    previous_velocity: np.ndarray,
    *,
    dt_s: float,
    lower: np.ndarray,
    upper: np.ndarray,
    max_velocity: np.ndarray,
    max_acceleration: np.ndarray,
    group: str,
) -> tuple[np.ndarray, np.ndarray]:
    """Intersect position/braking, velocity and acceleration bounds before moving.

    This does not repair an infeasible starting state by clipping its position.
    The returned velocity is always the actual emitted position increment.
    """
    if not np.isfinite(target).all() or not np.isfinite(previous_velocity).all():
        raise ExecutorError(f"{group}: nonfinite bounded command or velocity")
    outside = (~np.isfinite(previous)) | (previous < lower) | (previous > upper)
    if np.any(outside):
        index = int(np.flatnonzero(outside)[0])
        raise ExecutorError(f"{group}: coordinate {index} starts outside command bounds")
    lo = np.maximum((lower - previous) / dt_s, -max_velocity)
    hi = np.minimum((upper - previous) / dt_s, max_velocity)
    finite = np.isfinite(max_acceleration)
    if np.any(finite):
        acceleration = max_acceleration[finite]
        lo[finite] = np.maximum(
            lo[finite], -braking_velocity(previous[finite] - lower[finite], acceleration, dt_s),
        )
        hi[finite] = np.minimum(
            hi[finite], braking_velocity(upper[finite] - previous[finite], acceleration, dt_s),
        )
        lo[finite] = np.maximum(lo[finite], previous_velocity[finite] - acceleration * dt_s)
        hi[finite] = np.minimum(hi[finite], previous_velocity[finite] + acceleration * dt_s)
    if np.any(lo > hi):
        index = int(np.flatnonzero(lo > hi)[0])
        raise ExecutorError(
            f"{group}: infeasible command limits at coordinate {index}: "
            f"position={previous[index]:.9g}, velocity={previous_velocity[index]:.9g}, "
            f"bounds=[{lower[index]:.9g}, {upper[index]:.9g}], dt_s={dt_s:.9g}"
        )
    velocity = np.clip((target - previous) / dt_s, lo, hi)
    # Close floating-point roundoff at a boundary, then retain the emitted delta.
    command = np.clip(previous + velocity * dt_s, lower, upper)
    return command, (command - previous) / dt_s


def tracking_step(
    target: GroupVector,
    target_velocity: GroupVector,
    previous: GroupVector,
    previous_velocity: GroupVector,
    *,
    dt_s: float,
    position_gain: float,
    limits: ScalarLimits,
    gripper_indices: dict[str, int] | None = None,
    joint_tool: dict | None = None,
) -> tuple[GroupVector, GroupVector]:
    """Track a moving reference with feedforward and distance-aware braking.

    Preserve command velocity across replans. A new target inside the existing
    stopping distance can still be crossed: acceleration limits take precedence
    over teleporting the command onto it. Fixed reachable targets converge
    without the legacy repeated overshoot. This is not jerk-limited planning.
    """
    output, velocities = {}, {}
    for name, desired in target.items():
        output[name], velocities[name] = np.empty_like(desired), np.empty_like(desired)
        for part, selected, independent_index in motion_sections(
            name, len(desired), limits, (gripper_indices or {}).get(name), joint_tool
        ):
            acceleration = selected.max_acceleration
            max_velocity = selected.velocity_bound(dt_s)
            bound = selected.position_limit_abs
            bound = np.inf if bound is None else bound
            error = np.clip(desired[part], -bound, bound) - previous[name][part]
            correction = position_gain * error
            if acceleration is not None:
                correction = np.sign(error) * np.minimum(
                    np.abs(correction), braking_velocity(np.abs(error), acceleration, dt_s)
                )
            velocity = limit_velocity(
                target_velocity[name][part], previous_velocity[name][part], dt_s,
                max_velocity, None, mode=selected.mode, independent_index=independent_index,
            ) + correction
            if acceleration is None:
                lower = (-bound - previous[name][part]) / dt_s
                upper = (bound - previous[name][part]) / dt_s
            else:
                lower = -braking_velocity(previous[name][part] + bound, acceleration, dt_s)
                upper = braking_velocity(bound - previous[name][part], acceleration, dt_s)
            if max_velocity is not None and selected.mode == "per_joint":
                lower = np.maximum(lower, -max_velocity)
                upper = np.minimum(upper, max_velocity)
            velocity = limit_velocity(
                np.clip(velocity, lower, upper), previous_velocity[name][part], dt_s,
                max_velocity if selected.mode == "isotropic" else None, acceleration,
                mode=selected.mode, independent_index=independent_index,
            )
            output[name][part] = previous[name][part] + velocity * dt_s
            velocities[name][part] = velocity
    return output, velocities


def limit_step(
    target: GroupVector,
    previous: GroupVector,
    previous_velocity: GroupVector,
    *,
    dt_s: float,
    limits: ScalarLimits,
    gripper_indices: dict[str, int] | None = None,
    joint_tool: dict | None = None,
) -> tuple[GroupVector, GroupVector]:
    output: GroupVector = {}
    velocities: GroupVector = {}
    for name, desired in target.items():
        output[name], velocities[name] = np.empty_like(desired), np.empty_like(desired)
        for part, selected, independent_index in motion_sections(
            name, len(desired), limits, (gripper_indices or {}).get(name), joint_tool
        ):
            velocity = limit_velocity(
                (desired[part] - previous[name][part]) / dt_s,
                previous_velocity[name][part], dt_s, selected.velocity_bound(dt_s),
                selected.max_acceleration, mode=selected.mode, independent_index=independent_index,
            )
            command = (
                desired[part].copy()
                if selected.max_velocity is None and selected.max_acceleration is None
                else previous[name][part] + velocity * dt_s
            )
            if selected.position_limit_abs is not None:
                bound = selected.position_limit_abs
                if part.start is None:
                    command = np.clip(command, -bound, bound)
                else:
                    # New coordinate partitions retain actual emitted velocity
                    # and reject infeasible braking instead of clipping history.
                    command, velocity = bounded_step(
                        command, previous[name][part], previous_velocity[name][part],
                        dt_s=dt_s, lower=np.full_like(command, -bound),
                        upper=np.full_like(command, bound),
                        max_velocity=np.full_like(command, selected.velocity_bound(dt_s) or np.inf),
                        max_acceleration=np.full_like(command, selected.max_acceleration or np.inf),
                        group=name,
                    )
            output[name][part] = command
            velocities[name][part] = velocity
    return output, velocities


def arm_motion_parameters(**options) -> dict:
    """Fill in module defaults and return an independent dictionary without creating a configuration object."""

    values = {
        "max_velocity": None,
        "max_acceleration": None,
        "mode": "per_joint",
        "max_step_dt_s": None,
        **options,
    }
    # Nonfinite or nonpositive rates invalidate execution limits and must not reach the control loop.
    if any(
        values[key] is not None and (not math.isfinite(values[key]) or values[key] <= 0)
        for key in ["max_velocity", "max_acceleration", "max_step_dt_s"]
    ):
        raise ValueError("motion rates must be finite and positive")
    if values["mode"] not in {"per_joint", "isotropic"}:
        raise ValueError("unknown arm motion mode")
    return values


def gripper_motion_parameters(**options) -> dict:
    """Fill in module defaults and return an independent dictionary without creating a configuration object."""

    values = {
        "max_velocity": None,
        "max_acceleration": None,
        "max_closing_velocity": None,
        **options,
    }
    # Nonfinite or nonpositive rates invalidate execution limits and must not reach the control loop.
    if any(
        values[key] is not None and (not math.isfinite(values[key]) or values[key] <= 0)
        for key in ["max_velocity", "max_acceleration", "max_closing_velocity"]
    ):
        raise ValueError("motion rates must be finite and positive")
    return values


def motion_limits_parameters(**options) -> dict:
    """Fill in module defaults and return an independent dictionary without creating a configuration object."""

    values = {
        **options,
    }
    values["arm"] = arm_motion_parameters(**values["arm"])
    values["gripper"] = gripper_motion_parameters(**values["gripper"])
    if values.get("joint_tool") is not None:
        values["joint_tool"] = joint_tool_motion_parameters(**values["joint_tool"])
    return values


def joint_tool_motion_parameters(**options) -> dict:
    """Resolve explicit joint-tool slices and rates without gripper normalization."""
    from manimux.config_validation import check_fields

    check_fields(options, {
        "group_slices", "max_velocity", "max_acceleration", "position_limit_abs",
        "mode", "max_step_dt_s",
    }, "joint_tool")
    values = arm_motion_parameters(**{"position_limit_abs": None, **options})
    slices = values.get("group_slices", {})
    if not isinstance(slices, dict) or not slices or any(
        not isinstance(name, str) or not name.strip()
        or not isinstance(bounds, list | tuple) or len(bounds) != 2
        or any(type(index) is not int for index in bounds)
        or not 0 < bounds[0] < bounds[1]
        for name, bounds in slices.items()
    ):
        raise ValueError(
            "joint_tool.group_slices must contain non-empty trailing coordinate ranges"
        )
    bound = values["position_limit_abs"]
    if bound is not None and (not math.isfinite(bound) or bound <= 0):
        raise ValueError("joint tool position_limit_abs must be finite and positive")
    values["group_slices"] = {name: list(bounds) for name, bounds in slices.items()}
    return values
