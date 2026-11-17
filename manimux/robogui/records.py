"""Read finalized experiment trajectories without opening UI or hardware."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import zarr

from .robot_view import RobotView

SOURCES = {
    "state": "Measured state",
    "scheduled": "Scheduled reference",
    "command": "Executor command",
    "prediction": "Canonical joint prediction (one accepted plan, before scheduling)",
}


def load_episode_robot_view(
    episode: Path, *, view_config: dict | None = None, fallback: RobotView | None = None,
) -> RobotView:
    """Use recorded geometry; current view options affect presentation only.

    Legacy episodes without a snapshot need an explicitly selected view/model.
    An invalid snapshot is an error, never permission to use current geometry.
    """
    from manimux.embodiments.robot.base import RobotModel

    from .robot_view import load_robot_view

    if episode.name.endswith(".partial") or not (episode / "result.json").is_file():
        raise ValueError("Choose a finalized episode with result.json")
    metadata_path = episode / "meta.json"
    metadata = json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
    snapshot = metadata.get("robot_geometry")
    if snapshot is not None:
        options = view_config if view_config is not None else (
            fallback.options if fallback is not None else {}
        )
        return RobotView(RobotModel.from_snapshot(snapshot), options)
    if fallback is not None:
        return fallback
    if view_config is not None:
        return load_robot_view(view_config)
    raise ValueError("Episode has no robot geometry snapshot; select --config or --robot")


def load_episode_trajectory(
    episode: Path, source: str, robot: RobotView, *, plan_index: int | None = None,
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    """Read recorded tick samples or one joint prediction on its policy time grid."""
    if episode.name.endswith(".partial") or not (episode / "result.json").is_file():
        raise ValueError("Choose a finalized episode with result.json")
    if source not in SOURCES:
        raise ValueError("Choose state, scheduled, command or prediction")
    if not (episode / "data.zarr").is_dir():
        raise ValueError(f"No recorded trajectory at {episode / 'data.zarr'}")
    root = zarr.open_group(str(episode / "data.zarr"), mode="r")
    if source == "prediction":
        if type(plan_index) is not int or plan_index < 0:
            raise ValueError("Prediction replay requires a non-negative --plan-index")
        key = f"plans/{plan_index:06d}/canonical_raw"
        if key not in root:
            raise ValueError(f"Episode has no prediction {key}")
        chunk = root[key]
        if chunk.attrs.get("action_space") != "joint_position":
            raise ValueError("Prediction replay requires decoded joint-position actions")
        actions = robot.validate_groups(dict(chunk.arrays()), sequence=True)
        dt = chunk.attrs.get("dt_ns")
        origin = chunk.attrs.get("observation_time_ns")
        offset = chunk.attrs.get("source_offset_steps", 0)
        if (type(dt) is not int or dt <= 0 or type(origin) is not int
                or type(offset) is not int or offset < 0):
            raise ValueError("Prediction has invalid policy timestamps")
        count = len(next(iter(actions.values())))
        # These are policy-grid times, not achieved samples or command send times.
        return actions, origin + (offset + np.arange(count, dtype=np.int64)) * dt
    if plan_index is not None:
        raise ValueError("--plan-index applies only to prediction replay")
    ticks = root["ticks"]
    if source not in ticks or "monotonic_ns" not in ticks:
        raise ValueError(f"Episode has no ticks/{source} or ticks/monotonic_ns")
    actions = robot.validate_groups(
        {name: np.asarray(values) for name, values in ticks[source].arrays()}, sequence=True
    )
    stamps = np.asarray(ticks["monotonic_ns"], dtype=np.int64)
    count = len(next(iter(actions.values())))
    if stamps.shape != (count,) or np.any(np.diff(stamps) <= 0):
        raise ValueError("Recorded timestamps must match samples and strictly increase")
    return actions, stamps
