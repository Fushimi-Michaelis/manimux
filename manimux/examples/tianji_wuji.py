"""Synthetic Tianji-Wuji endpoints for the ordinary runtime and RoboGUI.

No motor SDK is imported or constructed. Geometry is the installed offline model.
The plant uses ideal position tracking on reads, not a physics or hardware model.
Home and shutdown use TianjiWujiRobot's implementation with these memory endpoints.
"""

from __future__ import annotations

import numpy as np

from manimux.embodiments.arm.base import ArmController, ArmState
from manimux.embodiments.arm.tianji import TianjiArm, TianjiArmSettings
from manimux.embodiments.end_effector.joint_tool import JointToolBase, JointToolState
from manimux.embodiments.robot.base import RobotBase, RobotModel
from manimux.embodiments.robot.tianji_home import TianjiHomeSettings
from manimux.embodiments.robot.tianji_wuji.robot import TianjiWujiRobot, _HandHomeSettings
from manimux.policies.capabilities import PolicyCapabilities


class MemoryController(ArmController):
    """One disconnected arm controller; sends update only an in-memory target."""

    def __init__(self, settings, initial, clock):
        self.settings, self.clock = settings, clock
        self.position = {name: q.copy() for name, q in initial.items()}
        self.target = {name: q.copy() for name, q in initial.items()}
        self.connected = False
        self.sequence = self.sent = 0

    def connect(self):
        self.connected = True

    def validate_motion_ready(self):
        if not self.connected:
            raise RuntimeError("synthetic arm controller is disconnected")

    def command_bounds(self):
        return {name: np.stack(cfg.joint_limits, axis=1) for name, cfg in self.settings.items()}

    def validate_commands(self, targets):
        for name, q in targets.items():
            if name not in self.settings or np.shape(q) != (7,) or not np.isfinite(q).all():
                raise ValueError("invalid synthetic arm target")
            lower, upper = self.settings[name].joint_limits
            if np.any(q < lower) or np.any(q > upper):
                raise ValueError("synthetic arm target exceeds model bounds")

    def send_commands(self, targets):
        self.validate_motion_ready()
        self.validate_commands(targets)
        self.target.update({name: q.copy() for name, q in targets.items()})
        self.sent += 1

    def get_states(self):
        self.validate_motion_ready()
        self.sequence += 1
        self.position = {name: q.copy() for name, q in self.target.items()}
        return {
            name: ArmState(q, self.clock.now_ns(), self.sequence)
            for name, q in self.position.items()
        }

    def stop(self):
        self.target = {name: q.copy() for name, q in self.position.items()}

    def close(self):
        self.connected = False


class MemoryHand(JointToolBase):
    """Synthetic hand samples; optional manual gate exercises the real close flow."""

    def __init__(self, coordinates, bounds, initial, clock, release_confirmation):
        super().__init__(coordinates=coordinates, joint_limits=bounds)
        self.position = np.array(initial, dtype=float)
        self.target = self.position.copy()
        self.clock, self.release_confirmation = clock, release_confirmation
        self.connected = self.active = False
        self.sequence = self.sent = 0

    def connect(self):
        self.connected = True

    def get_state(self):
        if not self.connected:
            raise RuntimeError("synthetic hand is disconnected")
        self.position = self.target.copy()
        self.sequence += 1
        return JointToolState(self.position, self.clock.now_ns() / 1e9, self.sequence)

    def send_command(self, command):
        if not self.connected:
            raise RuntimeError("synthetic hand is disconnected")
        self.validate_command(command)
        self.target = command.position.copy()
        self.active = True
        self.sent += 1

    @property
    def release_required(self):
        return self.active and self.release_confirmation

    def stop(self):
        self.target = self.position.copy()

    def release(self):
        self.active = False

    def close(self):
        if self.release_required:
            raise RuntimeError("synthetic hand still awaits release confirmation")
        self.active = self.connected = False


class DisconnectedTianjiWuji(TianjiWujiRobot):
    """Reuse assembly Home/close algorithms without constructing vendor drivers."""

    def __init__(self, model, clock, *, release_confirmation):
        self._release_confirmation = release_confirmation
        hardware = model.hardware
        self._home_motion = TianjiHomeSettings(**hardware["home_motion"])
        self._hand_home = _HandHomeSettings(**hardware["hand_home"])
        self._rated_velocity = hardware["rated_joint_velocity_rad_s"]
        self._home_cancel, self._home_error = None, ""
        sides, settings, initial = {}, {}, {}
        for name, group in model.groups.items():
            component = model.components[group.arm_name]
            side = component["options"]["side"]
            sides[name] = side
            settings[side] = TianjiArmSettings(
                joint_limits=component["hardware"]["joint_limits"],
                velocity_ratio=hardware["velocity_ratio"],
                acceleration_ratio=hardware["acceleration_ratio"],
            )
            initial[side] = model.home_joints[name]
        self.controller = MemoryController(settings, initial, clock)
        arms, tools = {}, {}
        for name, group in model.groups.items():
            arms[name] = TianjiArm(self.controller, sides[name], kinematics=group.arm.kinematics)
            tool = model.components[group.end_effector_name]
            tools[name] = MemoryHand(
                group.kinematics.coordinates[7:],
                tool["hardware"]["joint_limits"],
                self._hand_home.joints_rad,
                clock,
                release_confirmation,
            )
        RobotBase.__init__(
            self,
            model=model,
            arm_components=arms,
            end_effectors=tools,
            models=model.kinematics.models,
            clock=clock,
            execute=True,
            end_effector_control=True,
        )
        self._validate_home_config()

    @property
    def requires_release_confirmation(self):
        return self._release_confirmation

    def runtime_metadata(self):
        return {
            **super().runtime_metadata(),
            "source": "synthetic_memory_endpoints",
            "hardware_connected": False,
            "tracking": "ideal_position_on_read",
            "release_confirmation": self._release_confirmation,
        }


def build_robot(config, clock):
    """Explicit example factory: physical station/SDK options cannot enter it."""
    options = config["options"]
    unknown = set(options) - {
        "execute",
        "end_effector_control",
        "component_mounts",
        "release_confirmation",
    }
    if unknown:
        raise ValueError(
            f"Disconnected example rejects hardware/unknown options: {sorted(unknown)}"
        )
    if options.get("execute") is not True or options.get("end_effector_control") is not True:
        raise ValueError("Disconnected example requires both in-memory command paths enabled")
    confirmation = options.get("release_confirmation", False)
    if type(confirmation) is not bool:
        raise ValueError("release_confirmation must be boolean")
    model = RobotModel.from_robot_config(config)
    if config["group_dims"] != {name: 27 for name in model.groups} or any(
        layout != {"arm_dofs": 7, "gripper_dofs": 0, "joint_tool_dofs": 20}
        for layout in model.action_layouts.values()
    ):
        raise ValueError("Disconnected Tianji-Wuji example requires explicit 7+20 groups")
    return DisconnectedTianjiWuji(model, clock, release_confirmation=confirmation)


class SyntheticJointPolicy:
    """Bounded waves around the first synthetic observation, without accumulated drift."""

    def __init__(self, config):
        self.horizon = config["horizon_policy_steps"]
        self.options = config["options"]
        for key in ("arm_amplitude_rad", "hand_amplitude_rad"):
            value = self.options[key]
            if isinstance(value, bool) or not np.isfinite(value) or not 0 < value <= 0.1:
                raise ValueError(f"{key} must be in (0, 0.1] for the disconnected example")
        self.session = None
        self.initial = None

    def reset(self, session_id):
        self.session, self.initial = session_id, None

    def infer(self, request):
        if request.session_id != self.session:
            raise RuntimeError("synthetic policy session is not initialized")
        if self.initial is None:
            self.initial = {name: q.copy() for name, q in request.observation.state.groups.items()}
        groups = {}
        amplitude = np.r_[
            np.full(7, self.options["arm_amplitude_rad"]),
            np.full(20, self.options["hand_amplitude_rad"]),
        ]
        for index, (name, initial) in enumerate(self.initial.items()):
            if initial.shape != (27,):
                raise ValueError("synthetic Tianji-Wuji actions require 27 coordinates per group")
            phase = request.request_seq * 0.15 + np.arange(self.horizon)[:, None] * 0.04
            phase = phase + np.arange(27)[None, :] * 0.1 + index
            groups[name] = initial + amplitude * np.sin(phase)
        return {"format": "joint", "action_semantics": "absolute_joint_position", "actions": groups}

    def capabilities(self):
        return PolicyCapabilities(backend_metadata={"source": "synthetic_joint_waves"})

    def close(self):
        self.session = self.initial = None


def build_policy(config):
    return SyntheticJointPolicy(config)
