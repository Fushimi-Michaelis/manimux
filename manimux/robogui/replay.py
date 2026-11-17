"""Offline replay; the module CLI opens finalized experiment trajectories."""

from __future__ import annotations

from pathlib import Path


def create_recorded_replay(
    episode, robot=None, *, view_config=None, source="state", host="127.0.0.1", port=8087,
    plan_index=None,
):
    """Reuse recorded timestamps and the shared, hardware-free trajectory player."""
    from .action_replay import ActionReplayRoboGUI
    from .records import SOURCES, load_episode_robot_view, load_episode_trajectory

    robot = load_episode_robot_view(episode, view_config=view_config, fallback=robot)
    actions, stamps = load_episode_trajectory(episode, source, robot, plan_index=plan_index)
    label = SOURCES[source]
    if source == "prediction":
        label += f" · plan {plan_index} · policy time grid"
    return ActionReplayRoboGUI(
        actions, robot, action_dt_s=1.0, timestamps_ns=stamps,
        source_label=label, host=host, port=port,
    )


def main() -> None:
    import argparse

    from .action_replay import run_replay
    from .records import SOURCES
    from .robot_view import load_robogui_config

    parser = argparse.ArgumentParser(
        description="Replay a finalized experiment trajectory; no robot connection or RGB video playback."
    )
    parser.add_argument("--episode", type=Path, required=True, help="Finalized rollout directory")
    model = parser.add_mutually_exclusive_group()
    model.add_argument("--robot", help="RoboGUI robot preset, e.g. yam")
    model.add_argument("--config", type=Path, help="Custom RoboGUI YAML with offline RobotModel")
    parser.add_argument(
        "--local", type=Path, help="Mount bindings for legacy episodes without a snapshot",
    )
    parser.add_argument("--source", choices=tuple(SOURCES), default="state",
                        help="state, scheduled, command, or one canonical joint prediction")
    parser.add_argument("--plan-index", type=int,
                        help="Zero-based recorded plan index; required with --source prediction")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8087)
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    episode = args.episode.expanduser().resolve()
    try:
        if args.local is not None and args.config is None and args.robot is None:
            raise ValueError("--local requires --config or --robot for legacy replay")
        config = (
            load_robogui_config(
                args.config, robot=args.robot, local=args.local, validate_model_path=False,
            ) if args.config is not None or args.robot is not None else None
        )
        robogui = create_recorded_replay(
            episode, view_config=config, source=args.source, host=args.host, port=args.port,
            plan_index=args.plan_index,
        )
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(f"Recorded trajectory: {episode} · ticks/{args.source}")
    run_replay(robogui, host=args.host)


if __name__ == "__main__":
    main()
