"""Tianji bare arms composed with independently owned Wuji Hand 2 tools."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from manimux.clock import Clock, SystemClock
from manimux.embodiments.arm.tianji import TianjiArm, TianjiArmSettings, TianjiController
from manimux.embodiments.end_effector.joint_tool import JointToolCommand
from manimux.embodiments.end_effector.wuji import WujiHand2
from manimux.embodiments.robot.base import RobotBase, RobotModel
from manimux.embodiments.robot.release import ReleaseDecision, require_supported_release
from manimux.embodiments.robot.tianji_home import (
    TianjiHomeSettings,
    cosine_fraction,
    home_duration_s,
)
from manimux.types import RobotCommand


@dataclass(frozen=True, slots=True)
class _HandHomeSettings:
    """Explicit hand targets and motion settings from the assembly YAML."""

    joints_rad: tuple[float, ...]
    control_hz: float
    ramp_s: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    hold_s: float
    tolerance_rad: float
    settle_timeout_s: float

    def __post_init__(self) -> None:
        target = JointToolCommand(self.joints_rad).position
        if target.shape != (20,):
            raise ValueError("hardware.hand_home.joints_rad must contain twenty radians")
        object.__setattr__(self, "joints_rad", tuple(target))
        for name in (
            "control_hz", "ramp_s", "max_velocity_rad_s", "max_acceleration_rad_s2",
            "hold_s", "tolerance_rad", "settle_timeout_s",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0
            ):
                raise ValueError(f"hardware.hand_home.{name} must be finite and positive")

    def duration_s(self, distance_rad: float) -> float:
        """Bound every coordinate of a synchronized cosine by its largest move.

        For q=q0+d*(1-cos(pi*t/T))/2, peak speed is pi*abs(d)/(2*T)
        and peak acceleration is pi**2*abs(d)/(2*T**2). ramp_s is a minimum.
        """
        duration = max(
            self.ramp_s,
            (math.pi / 2.0) * distance_rad / self.max_velocity_rad_s,
            math.pi * math.sqrt(distance_rad / (2.0 * self.max_acceleration_rad_s2)),
        )
        if not math.isfinite(duration):
            raise ValueError("hardware.hand_home limits produce a non-finite duration")
        return duration


class TianjiWujiRobot(RobotBase):
    """One shared Marvin session and one independent SDK endpoint per hand.

    Groups contain seven arm radians followed by twenty hand radians. All
    model/driver construction is disconnected. Both command switches default
    off; tool execute is derived from those switches, never a station override.

    stop() inherits the existing controller behavior: Tianji disables its owned
    arms and Wuji requests a measured-pose hold. Close first stops all components,
    then requires the existing explicit support-confirmation gate before disabling
    an activated hand. Home retains measured hand targets until every arm arrives,
    then moves controlled hands to their explicit targets. No drag or UMI load is inherited.
    """

    def __init__(
        self,
        *,
        model: RobotModel,
        clock: Clock | None = None,
        hardware: Mapping | None = None,
        component_hardware: Mapping | None = None,
        execute: bool = False,
        end_effector_control: bool = False,
    ) -> None:
        clock = clock if clock is not None else SystemClock()
        control = {**model.hardware, **(hardware or {})}
        home_motion = {**model.hardware["home_motion"], **control.pop("home_motion")}
        self._home_motion = TianjiHomeSettings(**home_motion)
        hand_home = {**model.hardware["hand_home"], **control.pop("hand_home")}
        self._hand_home = _HandHomeSettings(**hand_home)
        self._home_cancel: threading.Event | None = None
        self._home_error = ""
        ratios = {key: control.pop(key) for key in ("velocity_ratio", "acceleration_ratio")}
        self._rated_velocity = control.pop("rated_joint_velocity_rad_s")
        if not math.isfinite(self._rated_velocity) or self._rated_velocity <= 0:
            raise ValueError("rated_joint_velocity_rad_s must be finite and positive")
        overrides = dict(component_hardware or {})
        if set(overrides) - set(model.components):
            raise ValueError("component_hardware contains unknown components")
        bound = {}
        for name, component in model.components.items():
            local = overrides.get(name, {})
            if {"execute", "side", "model_version", "velocity_ratio", "acceleration_ratio"} & set(
                local
            ):
                raise ValueError(
                    f"{name}: station bindings cannot change execution, model or shared ratios"
                )
            bound[name] = {**component["hardware"], **local}
            if {"side", "model_version", "velocity_ratio", "acceleration_ratio"} & set(bound[name]):
                raise ValueError(
                    f"{name}: hardware cannot override model identity or shared ratios"
                )
        settings, sides = {}, {}
        for name, group in model.groups.items():
            arm = model.components[group.arm_name]
            if not issubclass(arm["class"], TianjiArm) or group.end_effector_name is None:
                raise ValueError("Tianji-Wuji groups require a Tianji arm and Wuji hand")
            tool = model.components[group.end_effector_name]
            side = arm["options"]["side"]
            if side in settings or not issubclass(tool["class"], WujiHand2):
                raise ValueError("each Tianji side requires one distinct Wuji hand")
            if tool["options"]["side"] != side:
                raise ValueError("arm and hand must have the same configured side")
            if model.action_layouts.get(name) != {
                "arm_dofs": 7,
                "gripper_dofs": 0,
                "joint_tool_dofs": 20,
            }:
                raise ValueError("Tianji-Wuji requires the explicit 7+20 action layout")
            if bound[group.arm_name].get("drag_tool") is not None:
                raise ValueError("Tianji-Wuji does not support drag/load parameters yet")
            settings[side] = TianjiArmSettings(**ratios, **bound[group.arm_name])
            sides[name] = side
        controller = TianjiController(settings=settings, clock=clock, **control)
        arms, tools = {}, {}
        serials = []
        for name, group in model.groups.items():
            arms[name] = model.components[group.arm_name]["class"](
                controller,
                sides[name],
                kinematics=group.arm.kinematics,
            )
            component = model.components[group.end_effector_name]
            options = {key: component["options"][key] for key in ("side", "model_version")}
            options.update(bound[group.end_effector_name])
            options["execute"] = execute is True and end_effector_control is True
            tools[name] = component["class"](clock=clock, **options)
            if options.get("serial"):
                serials.append(options["serial"])
        if len(serials) != len(set(serials)):
            raise ValueError("each Wuji hand requires a distinct serial")
        if any(component["type"] == "sensor" for component in model.components.values()):
            raise ValueError(
                "bind cameras through existing sensor sources; no wrist camera is implied"
            )
        self.controller = controller
        super().__init__(
            model=model,
            arm_components=arms,
            end_effectors=tools,
            models=model.kinematics.models,
            clock=clock,
            stale_timeout_s=control.get("stale_timeout_s", 0.2),
            execute=execute,
            end_effector_control=end_effector_control,
        )
        if model.home_joints:
            self._validate_home_config()

    @classmethod
    def from_config(
        cls,
        path: Path | str,
        *,
        component_mounts: Mapping | None = None,
        **options,
    ) -> TianjiWujiRobot:
        return cls(
            model=RobotModel.from_config(path, component_mounts=component_mounts),
            **options,
        )

    def _prepare_end_effector_close(self) -> None:
        release_error = ""
        while True:
            active = [
                name
                for name in self._end_effector_open
                if self.end_effectors[name].release_required
            ]
            if not active:
                return

            self._ready = False
            decision = require_supported_release(
                f"Tianji stop disables arm control; secure ALL arms and payloads before "
                f"releasing Wuji hand actuation: {', '.join(sorted(active))}"
                f"{'; Home failed: ' + self._home_error if self._home_error else ''}"
                f"{release_error}"
            )
            if decision is ReleaseDecision.SUPPORTED_RELEASE:
                errors = []
                for name in active:
                    try:
                        self.end_effectors[name].release()
                    except Exception as error:
                        errors.append(error)
                if errors:
                    release_error = "; previous release failed: " + "; ".join(
                        f"{type(error).__name__}: {error}" for error in errors
                    )
                    # Keep the runtime inside hardware recovery. Another
                    # explicit support confirmation is required for each retry.
                    continue
                return

    def _validate_home_config(self) -> None:
        targets = self.model.home_joints
        if not targets:
            raise ValueError("Tianji-Wuji execution requires explicit home.joints_deg targets")
        if set(targets) != set(self.arm_components):
            raise ValueError("Home targets must contain every configured arm group")
        self.controller.validate_commands(
            {arm.channel: targets[name] for name, arm in self.arm_components.items()}
        )
        command = JointToolCommand(self._hand_home.joints_rad)
        for tool in self.end_effectors.values():
            tool.validate_command(command)
        available = min(
            self._rated_velocity * cfg.velocity_ratio / 100
            for cfg in self.controller.settings.values()
        )
        if math.radians(self._home_motion.peak_velocity_deg_s) > available:
            raise ValueError("Home peak velocity exceeds the configured arm controller rate")

    def connect(self) -> None:
        # A connected executing rollout must have a defined shutdown destination.
        # Validate before constructing either vendor's device/control objects.
        if self._execute:
            self._validate_home_config()
        super().connect()

    @property
    def requires_release_confirmation(self) -> bool:
        """An executing hand can retain control until explicit support confirmation."""
        return self._execute and self._end_effector_control

    @property
    def shutdown_home_available(self) -> bool:
        return bool(self._execute and self._ready and self.model.home_joints)

    def _check_home_cancel(self) -> None:
        if self._home_cancel is not None and self._home_cancel.is_set():
            raise RuntimeError("Tianji-Wuji Home cancelled")

    def _home_wait(self, period_s: float) -> None:
        if self._home_cancel is None:
            time.sleep(period_s)
        else:
            self._home_cancel.wait(period_s)
        self._check_home_cancel()

    def home_targets(self, state):
        if not self.model.home_joints:
            return None
        targets = {name: values.copy() for name, values in state.groups.items()}
        for name, q in self.model.home_joints.items():
            targets[name][:7] = q
            if self._end_effector_control:
                targets[name][7:] = self._hand_home.joints_rad
        return targets

    def home(self, stop: threading.Event | None = None) -> None:
        """Home arms, then controlled hands, using measured arrival for both.

        Completion verifies positions, not physical payload retention. Hand
        publication follows end_effector_control; receive-only Home performs no I/O.
        """
        if not self._execute:
            return
        with self._lock:
            self._home_cancel = stop
            try:
                self._validate_home_config()
                self._check_home_cancel()
                self._move_home()
                if self._end_effector_control:
                    self._move_hands_home()
            except BaseException as error:
                self._home_error = f"{type(error).__name__}: {error}"
                raise
            finally:
                self._home_cancel = None
            self._home_error = ""

    def _validate_arm_home_feedback(self) -> None:
        """Use the main controller's fault/freshness and measured-bound checks.

        Control mode and speed ratios are checked by its ordinary send_commands
        before each Home submission; no extra motor mode or recovery is added.
        """
        measured = self.controller.get_states()
        self.controller.validate_commands({
            channel: sample.joints for channel, sample in measured.items()
        })

    def _move_home(self) -> None:
        state = self.get_state()
        self._validate_arm_home_feedback()
        groups = {name: values.copy() for name, values in state.groups.items()}
        targets = self.model.home_joints
        start = {name: state.groups[name][:7].copy() for name in targets}
        home = self._home_motion
        period_s = 1.0 / home.control_hz
        duration_s = home_duration_s(start, targets, home.peak_velocity_deg_s)
        deadline = time.monotonic() + duration_s * 2.0 + home.settle_timeout_s
        tick = 0
        while True:
            self._check_home_cancel()
            if time.monotonic() > deadline:
                raise TimeoutError("Tianji-Wuji Home trajectory timed out")
            fraction = cosine_fraction(tick * period_s, duration_s)
            for name, q in targets.items():
                groups[name][:7] = start[name] + (q - start[name]) * fraction
            self.send_command(RobotCommand(groups, self._clock.now_ns(), None))
            if fraction >= 1.0:
                break
            tick += 1
            self._home_wait(period_s)

        # RobotState.sequence counts reads, not device samples. Use the actual
        # arm feedback identities after the final submission instead.
        previous = self.controller.get_states()
        deadline = time.monotonic() + home.settle_timeout_s
        tolerance = math.radians(home.tolerance_deg)
        while True:
            self._check_home_cancel()
            self.get_state()  # All arms and hands must remain healthy and fresh.
            self._validate_arm_home_feedback()
            measured = self.controller.get_states()
            fresh, errors = {}, {}
            for name, arm in self.arm_components.items():
                sample = measured[arm.channel]
                fresh[name] = sample.sequence != previous[arm.channel].sequence
                errors[name] = float(np.max(np.abs(sample.joints - targets[name])))
            if all(fresh.values()) and all(error <= tolerance for error in errors.values()):
                return
            if time.monotonic() >= deadline:
                degrees = {name: math.degrees(error) for name, error in errors.items()}
                raise TimeoutError(
                    f"Tianji-Wuji Home arrival timed out: errors_deg={degrees}, "
                    f"fresh_feedback={fresh}"
                )
            self.send_command(RobotCommand(groups, self._clock.now_ns(), None))
            self._home_wait(period_s)

    def _hand_home_feedback(self):
        state = self.get_state()
        self._validate_arm_home_feedback()
        tolerance = math.radians(self._home_motion.tolerance_deg)
        for name, target in self.model.home_joints.items():
            if np.max(np.abs(state.groups[name][:7] - target)) > tolerance:
                raise RuntimeError(f"{name}: arm left Home during hand Home")
        return {name: tool.get_state() for name, tool in self.end_effectors.items()}

    def _move_hands_home(self) -> None:
        self._check_home_cancel()
        home = self._hand_home
        measured = self._hand_home_feedback()
        start = {name: state.position.copy() for name, state in measured.items()}
        groups = {
            name: np.concatenate((target, start[name]))
            for name, target in self.model.home_joints.items()
        }
        target = np.asarray(home.joints_rad)
        period_s = 1.0 / home.control_hz
        distance = max(float(np.max(np.abs(target - q))) for q in start.values())
        duration_s = home.duration_s(distance)
        deadline = time.monotonic() + 2.0 * duration_s + home.settle_timeout_s
        tick = 0
        while True:
            self._check_home_cancel()
            if time.monotonic() > deadline:
                raise TimeoutError("Tianji-Wuji hand Home trajectory timed out")
            self._hand_home_feedback()
            self._check_home_cancel()
            if time.monotonic() > deadline:
                raise TimeoutError("Tianji-Wuji hand Home trajectory timed out")
            # Retain one nominal sample per submission. Slow SDK calls extend
            # the trajectory; they must not skip ahead or trigger catch-up sends.
            fraction = cosine_fraction(tick * period_s, duration_s)
            for name, q in start.items():
                groups[name][7:] = target if fraction >= 1.0 else q + (target - q) * fraction
            self.send_command(RobotCommand(groups, self._clock.now_ns(), None))
            if fraction >= 1.0:
                break
            tick += 1
            self._home_wait(period_s)

        previous = self._hand_home_feedback()
        deadline = time.monotonic() + home.settle_timeout_s
        while True:
            self._check_home_cancel()
            measured = self._hand_home_feedback()
            fresh = {
                name: sample.sequence != previous[name].sequence
                for name, sample in measured.items()
            }
            errors = {
                name: float(np.max(np.abs(sample.position - target)))
                for name, sample in measured.items()
            }
            if all(fresh.values()) and all(
                error <= home.tolerance_rad for error in errors.values()
            ):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Tianji-Wuji hand Home arrival timed out: errors_rad={errors}, "
                    f"fresh_feedback={fresh}"
                )
            self.send_command(RobotCommand(groups, self._clock.now_ns(), None))
            self._home_wait(period_s)

        deadline = time.monotonic() + home.hold_s
        while True:
            self._check_home_cancel()
            measured = self._hand_home_feedback()
            errors = {
                name: float(np.max(np.abs(sample.position - target)))
                for name, sample in measured.items()
            }
            if any(error > home.tolerance_rad for error in errors.values()):
                raise RuntimeError(f"Tianji-Wuji hand left Home during hold: errors_rad={errors}")
            if time.monotonic() >= deadline:
                return
            self.send_command(RobotCommand(groups, self._clock.now_ns(), None))
            self._home_wait(period_s)


def create_robot(config: dict, clock: Clock) -> TianjiWujiRobot:
    """Public module:factory entry, shared by runtime and offline construction."""
    robot = TianjiWujiRobot.from_config(config["config"], clock=clock, **config["options"])
    dimensions = {name: model.num_coordinates for name, model in robot.kinematics.models.items()}
    if config["group_dims"] != dimensions:
        raise ValueError("experiment group_dims must match the Tianji-Wuji assembly")
    return robot
