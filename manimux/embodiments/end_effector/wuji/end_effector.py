"""Owned Wuji Hand 2 SDK connection and absolute joint-position commands."""

from __future__ import annotations

import importlib
import math
import threading
import uuid
import xml.etree.ElementTree as ET
from contextlib import suppress

import numpy as np

from manimux.clock import Clock, SystemClock
from manimux.embodiments.end_effector.base import EndEffectorModel
from manimux.embodiments.end_effector.joint_tool import (
    JointToolBase,
    JointToolCommand,
    JointToolState,
)
from manimux.embodiments.end_effector.wuji.geometry import (
    COORDINATES,
    WujiGeometry,
    hand_model,
)

_MANAGER_LOCK = threading.RLock()


def _integer(value, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _vector(value, name: str, *, positive: bool) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim == 0:
        array = np.full(20, array.item())
    if array.shape != (20,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must be a finite scalar or length-20 vector")
    invalid = np.any(array <= 0) if positive else np.any(array < 0)
    if invalid:
        raise ValueError(f"{name} must be {'positive' if positive else 'non-negative'}")
    return array.copy()


class _Stream:
    """Drain an SDK FIFO and preserve sample identity across cached reads.

    Receipt means host dequeue, since recv() exposes no transport receipt time.
    Device time is kept separately; the minimum observed host/device offset
    detects growing delay, not absolute one-way latency or initial backlog.
    Sequence wrap, device reboot and clock regression require reconnection.
    """

    def __init__(self, subscription, decode, clock, timeout_s, max_frames) -> None:
        self.subscription = subscription
        self.decode = decode
        self.clock = clock
        self.timeout_s = timeout_s
        self.max_frames = max_frames
        self.value = None
        self.sequence = None
        self.device_timestamp_us = None
        self.receipt_s = None
        self._offset_s = None

    def read(self):
        try:
            for _ in range(self.max_frames):
                frame = self.subscription.recv()
                if frame is None:
                    break
                value = self.decode(frame)
                sequence = _integer(frame.header.seq, "frame sequence")
                device_us = _integer(frame.header.timestamp_us, "device timestamp_us")
                if device_us == 0:
                    raise ValueError("Wuji device timestamp_us must be positive")
                if self.sequence is not None:
                    if sequence == self.sequence:
                        if device_us != self.device_timestamp_us or (
                            self.value is not None and not np.array_equal(value, self.value)
                        ):
                            raise ValueError("Wuji duplicate frame changed timestamp or payload")
                        continue
                    if sequence < self.sequence or device_us <= self.device_timestamp_us:
                        raise ValueError("Wuji frame sequence or device clock regressed")
                now = self.clock.now_ns() / 1e9
                offset = now - device_us / 1e6
                self._offset_s = offset if self._offset_s is None else min(self._offset_s, offset)
                self.sequence, self.device_timestamp_us = sequence, device_us
                self.receipt_s, self.value = now, value
            else:
                raise RuntimeError("Wuji feedback queue exceeds max_frames_per_read")
            now = self.clock.now_ns() / 1e9
            if self.value is None:
                raise TimeoutError("Wuji feedback is unavailable")
            age = now - self.receipt_s
            delay = now - self.device_timestamp_us / 1e6 - self._offset_s
            if age < 0 or max(age, delay) > self.timeout_s:
                raise TimeoutError(
                    f"Wuji feedback is stale (receipt={age:.3f}s, delay={delay:.3f}s)"
                )
            return self.value
        except Exception:
            # A bad frame must not leave old feedback usable on the next call.
            self.value = None
            raise


class WujiHand2(JointToolBase):
    """Own one exact serial and side, without implicit discovery or activation.

    Construction/load_model import no SDK and open no hardware. connect selects
    the serial, disables the SDK multi-process bridge, checks type, side and the
    explicitly configured hardware version, and subscribes to feedback only.
    No stream rate, gains, current limit, enable, fault reset or Home is written.

    The first command with execute=True checks all joints are Ready/Stopped and
    fault-free, writes explicit MIT gains/current limits, enables, and verifies
    all 20 joints Enabled before submitting targets. Firmware must already use
    MIT mode (the SDK's documented Hand 2 default); mode switching is unsupported.
    A failed activation remains owned and blocks commands until explicit release.

    stop submits one fresh measured pose with zero velocity/effort, retaining
    MIT actuation. It requests position hold, not measured settling or an
    emergency stop. No background command loop or watchdog guarantee is added.
    A later command resumes position submission. close refuses while activation
    is owned/uncertain: the assembly must first secure the payload and explicitly
    call release(). Read-only close never disables another owner's actuators.
    """

    @classmethod
    def load_model(
        cls,
        *,
        side: str,
        model_version: str,
        tcp_transform=None,
        base_frame: str | None = None,
        tcp_frame: str | None = None,
    ):
        visual = hand_model(side, model_version, tcp_transform=tcp_transform)
        return EndEffectorModel(
            WujiGeometry(
                side=side,
                model_version=model_version,
                tcp_transform=tcp_transform,
                base_frame=base_frame,
                tcp_frame=tcp_frame,
            ),
            visual,
        )

    def __init__(
        self,
        *,
        side: str,
        model_version: str,
        joint_limits,
        serial: str | None = None,
        expected_hw_version: tuple[int, int, int] | None = None,
        execute: bool = False,
        kp=None,
        kd=None,
        effort_limit_a=None,
        clock: Clock | None = None,
        stale_timeout_s: float = 0.2,
        ready_timeout_s: float = 3.0,
        timeout_ms: int = 1000,
        max_frames_per_read: int = 1024,
    ) -> None:
        visual = hand_model(side, model_version)
        super().__init__(coordinates=COORDINATES, joint_limits=joint_limits)
        joints = {j.get("name"): j for j in ET.parse(visual.urdf_path).getroot().findall("joint")}
        model_limits = np.array(
            [
                [float(joints[d.joint].find("limit").get(key)) for key in ("lower", "upper")]
                for d in sorted(visual.spec.joints, key=lambda d: d.input)
            ]
        )
        if np.any(self._joint_limits[:, 0] < model_limits[:, 0]) or np.any(
            self._joint_limits[:, 1] > model_limits[:, 1]
        ):
            raise ValueError("Wuji joint_limits must stay inside the selected model's limits")
        if not isinstance(execute, bool):
            raise ValueError("execute must be boolean")
        for name, value in (
            ("stale_timeout_s", stale_timeout_s),
            ("ready_timeout_s", ready_timeout_s),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if _integer(timeout_ms, "timeout_ms") == 0:
            raise ValueError("timeout_ms must be positive")
        if _integer(max_frames_per_read, "max_frames_per_read") == 0:
            raise ValueError("max_frames_per_read must be positive")
        self._side, self._serial = side, serial
        self._expected_hw_version = expected_hw_version
        self._execute = execute
        self._kp = None if kp is None else _vector(kp, "kp", positive=True)
        self._kd = None if kd is None else _vector(kd, "kd", positive=False)
        self._effort = (
            None
            if effort_limit_a is None
            else _vector(effort_limit_a, "effort_limit_a", positive=True)
        )
        self._clock = clock if clock is not None else SystemClock()
        self._stale_timeout_s, self._ready_timeout_s = stale_timeout_s, ready_timeout_s
        self._timeout_ms, self._max_frames = timeout_ms, max_frames_per_read
        self._name = f"manimux_wuji_{side}_{uuid.uuid4().hex}"
        self._sdk = self._manager = self._device = None
        self._states = self._diagnostics = self._publisher = None
        self._connection_owned = self._activation_owned = False
        self._enabled = self._ready = False
        self._command_fault = False
        self._lock = threading.RLock()

    def _decode(self, frame, *, diagnostics: bool):
        if _integer(frame.num_joints, "num_joints") != 20 or len(frame.joints) != 20:
            raise ValueError("Wuji feedback must contain all 20 joints")
        values = np.empty((20, 2 if diagnostics else 3), dtype=np.float64)
        seen = set()
        for entry in frame.joints:
            nid = _integer(entry.nid, "joint NID")
            index = self._sdk.WujiHand2.nid_to_joint_index(nid)
            if not isinstance(index, int) or not 0 <= index < 20 or index in seen:
                raise ValueError(f"Wuji duplicate or invalid joint NID {nid}")
            seen.add(index)
            if diagnostics:
                status = _integer(entry.status_word.ext_state, "joint ext_state")
                if status > 3:
                    raise ValueError(f"Wuji unknown joint state {status} at NID {nid}")
                values[index] = status, _integer(entry.error_code_current, "joint error code")
            else:
                values[index] = entry.position, entry.velocity, entry.effort
        if len(seen) != 20 or not np.isfinite(values).all():
            raise ValueError("Wuji feedback has missing or non-finite joint values")
        return values

    def _stream(self, subscription, *, diagnostics=False):
        return _Stream(
            subscription,
            lambda frame: self._decode(frame, diagnostics=diagnostics),
            self._clock,
            self._stale_timeout_s,
            self._max_frames,
        )

    def _wait(self, check) -> None:
        deadline = self._clock.now_ns() + int(self._ready_timeout_s * 1e9)
        while True:
            try:
                if check():
                    return
            except TimeoutError:
                pass
            if self._clock.now_ns() >= deadline:
                raise TimeoutError("Wuji timed out waiting for complete fresh feedback/readiness")
            self._clock.sleep_until_ns(min(deadline, self._clock.now_ns() + 5_000_000))

    def connect(self) -> None:
        with self._lock:
            if self._ready:
                return
            if self._connection_owned:
                raise RuntimeError("Wuji cleanup incomplete; close before reconnecting")
            if not isinstance(self._serial, str) or not self._serial.strip():
                raise ValueError("Wuji connect requires an exact serial")
            version = self._expected_hw_version
            if not isinstance(version, (tuple, list)) or len(version) != 3:
                raise ValueError("Wuji connect requires an explicit expected_hw_version triple")
            for value in version:
                _integer(value, "expected_hw_version")
            if tuple(version) == (0, 0, 0):
                raise ValueError("Wuji expected_hw_version must be provisioned (nonzero)")
            if self._execute and any(v is None for v in (self._kp, self._kd, self._effort)):
                raise ValueError("Wuji execute requires explicit kp, kd and effort_limit_a")
            self._sdk = importlib.import_module("wuji_sdk")
            self._manager = self._sdk.SdkManager.instance()
            with _MANAGER_LOCK:
                if any(
                    d.serial_number == self._serial
                    for _, d in self._manager.get_connected_devices()
                ):
                    raise RuntimeError("Wuji serial already belongs to another SDK connection")
                self._connection_owned = True
                try:
                    self._device = self._manager.connect(
                        sn=self._serial,
                        device_name=self._name,
                        options=self._sdk.ConnectOptions(
                            timeout_ms=self._timeout_ms,
                            enable_bridge=False,
                        ),
                    )
                    if not isinstance(self._device, self._sdk.WujiHand2):
                        raise ValueError("configured device is not a Wuji Hand 2")
                    if self._device.serial_number != self._serial:
                        raise ValueError("Wuji connected serial does not match configuration")
                    if self._device.handedness().get() != self._side:
                        raise ValueError("Wuji handedness does not match configuration")
                    hw = self._device.hw_version().get()
                    if (hw.major, hw.minor, hw.patch) != tuple(version):
                        raise ValueError("Wuji hardware version does not match configuration")
                    self._states = self._stream(self._device.joint_states().subscribe())
                    self._diagnostics = self._stream(
                        self._device.joint_diagnostics().subscribe(),
                        diagnostics=True,
                    )
                    self._wait(lambda: self._initial_feedback())
                    self._ready = True
                except Exception as error:
                    try:
                        self.close()
                    except Exception as cleanup_error:
                        raise ExceptionGroup(
                            "Wuji connect and cleanup failed", [error, cleanup_error]
                        ) from None
                    raise

    def _initial_feedback(self):
        self._states.read()
        self._healthy_diagnostics()
        return True

    def _healthy_diagnostics(self):
        values = self._diagnostics.read()
        if np.any(values[:, 1] != 0):
            bad = np.flatnonzero(values[:, 1] != 0).tolist()
            raise RuntimeError(f"Wuji joint faults at flat indices {bad}; recovery is explicit")
        return values[:, 0]

    def _require_connection(self):
        if not self._ready or self._device is None or not self._device.is_connected:
            raise RuntimeError("Wuji hand is not connected/ready")

    def get_state(self) -> JointToolState:
        with self._lock:
            self._require_connection()
            values = self._states.read()
            self._healthy_diagnostics()
            return JointToolState(values[:, 0], self._states.receipt_s, self._states.sequence)

    def _send(self, command):
        self._publisher.send(
            [self._sdk.JointCommand(float(position), 0.0, 0.0) for position in command.position]
        )

    def _changed_status(self, previous, expected) -> bool:
        status = self._healthy_diagnostics()
        return self._diagnostics.sequence != previous and bool(np.isin(status, expected).all())

    def send_command(self, command: JointToolCommand) -> None:
        with self._lock:
            self.validate_command(command)
            self._require_connection()
            if not self._execute:
                raise RuntimeError("Wuji commands require execute=True")
            if self._command_fault:
                raise RuntimeError("Wuji command failed; explicit release and reconnect required")
            self.get_state()
            status = self._healthy_diagnostics()
            try:
                if not self._activation_owned:
                    if not np.isin(status, (1, 3)).all():
                        raise RuntimeError("Wuji activation requires all joints Ready/Stopped")
                    self._device.effort_limit().set(self._effort.tolist())
                    gains = list(zip(self._kp.tolist(), self._kd.tolist(), strict=True))
                    self._device.mit_params().set(gains)
                    self._publisher = self._device.joint_command().publish()
                    previous = self._diagnostics.sequence
                    # enable() can partially succeed before raising; retain ownership.
                    self._activation_owned = True
                    self._device.enable()
                    self._wait(lambda: self._changed_status(previous, (2,)))
                    self._enabled = True
                if not self._enabled or not np.all(self._healthy_diagnostics() == 2):
                    raise RuntimeError("Wuji activation incomplete or drive left Enabled state")
                self.get_state()
                self._send(command)
            except Exception:
                self._command_fault = True
                raise

    def stop(self) -> None:
        with self._lock:
            if not self._activation_owned:
                return
            self._require_connection()
            if not self._enabled or not np.all(self._healthy_diagnostics() == 2):
                raise RuntimeError("cannot request Wuji hold without healthy owned actuation")
            hold = JointToolCommand(self.get_state().position)
            self.validate_command(hold)
            self._send(hold)

    @property
    def release_required(self) -> bool:
        """Whether owned or uncertain actuation must be explicitly released."""
        with self._lock:
            return self._activation_owned

    def release(self) -> None:
        """Explicitly disable owned actuation after external payload support.

        This may release a grasp. No open/Home motion is sent. Disable submission
        alone is insufficient: wait for complete fresh non-Enabled diagnostics.
        Failure preserves ownership and connection so release can be retried.
        """
        with self._lock:
            if not self._activation_owned:
                return
            # Explicit disable remains available with stale/faulty feedback;
            # completion still needs a new valid diagnostics frame.
            with suppress(Exception):
                self._diagnostics.read()
            previous = self._diagnostics.sequence
            self._device.disable()
            # Fault codes do not authorize release, but must not prevent an
            # explicitly requested disable from being acknowledged either.
            self._wait(lambda: self._released_status(previous))
            self._activation_owned = self._enabled = False
            self._command_fault = True

    def _released_status(self, previous) -> bool:
        status = self._diagnostics.read()[:, 0]
        return self._diagnostics.sequence != previous and bool(np.isin(status, (1, 3)).all())

    def close(self) -> None:
        with self._lock, _MANAGER_LOCK:
            if self._activation_owned:
                raise RuntimeError(
                    "Wuji actuation is owned/uncertain; secure payload and explicitly release first"
                )
            self._ready = False
            errors = []
            for attr in ("_publisher", "_diagnostics", "_states"):
                resource = getattr(self, attr)
                if resource is None:
                    continue
                try:
                    (resource.subscription if isinstance(resource, _Stream) else resource).close()
                    setattr(self, attr, None)
                except Exception as error:
                    errors.append(error)
            # Keep the endpoint available while any stream cleanup needs retry.
            if not errors and self._connection_owned:
                try:
                    if self._manager.get_device(self._name) is not None:
                        self._manager.disconnect(self._name)
                    self._connection_owned = False
                    self._device = None
                    self._command_fault = False
                except Exception as error:
                    errors.append(error)
            if errors:
                raise ExceptionGroup("Wuji cleanup incomplete", errors)
