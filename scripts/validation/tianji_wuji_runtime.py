"""Exercise Tianji-Wuji through loopback XPolicyLab RPC and memory endpoints.

This is a synthetic protocol peer, not a model server or a hardware simulator.
Raw MessagePack requests/replies are saved before replies reach the client.
Run from a checkout with ManiMux[xpolicylab] and native offline geometry installed.
"""

from __future__ import annotations

import argparse
import builtins
import importlib
import itertools
import json
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml

# Support direct execution from a source checkout.
if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import manimux  # noqa: E402
from manimux.cli import load_config  # noqa: E402
from manimux.policies.xpolicylab.ws_client import pack_frame, unpack_frame  # noqa: E402
from manimux.runtime import build_runtime  # noqa: E402

POSE_SEMANTICS = "absolute_per_arm_base_xyz_wxyz"
EXAMPLES = Path(manimux.__file__).parent / "configs/examples/tianji_wuji"


@contextmanager
def forbid_motor_imports():
    """Fail on a motor SDK import in the process that constructs the robot."""
    original, dynamic = builtins.__import__, importlib.import_module

    def check(name):
        if name.split(".")[0] == "wuji_sdk" or "fx_robot" in name.split("."):
            raise RuntimeError(f"Motor SDK import forbidden by disconnected validation: {name}")

    def guarded(name, *args, **kwargs):
        check(name)
        return original(name, *args, **kwargs)

    def guarded_dynamic(name, *args, **kwargs):
        check(name)
        return dynamic(name, *args, **kwargs)

    with patch.object(builtins, "__import__", guarded), patch.object(
        importlib, "import_module", guarded_dynamic
    ):
        yield


class SyntheticServer:
    """A loopback-only protocol peer with bounded, inspectable fault injection."""

    def __init__(self, output, *, representation="joint", horizon=16, fault=None):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.representation, self.horizon, self.fault = representation, horizon, fault
        self.semantics = POSE_SEMANTICS if representation == "pose" else "absolute_joint_position"
        self.frames = []
        self.errors = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._connections = set()

    def __enter__(self):
        from websockets.sync.server import serve

        self.server = serve(self._handle, "127.0.0.1", 0, max_size=None)
        self.url = f"ws://127.0.0.1:{self.server.socket.getsockname()[1]}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        return self

    def _record(self, direction, raw):
        frame = unpack_frame(raw)
        with self._lock:
            index = len(self.frames)
            filename = f"{index:06d}-{direction}.msgpack"
            (self.output / filename).write_bytes(raw)
            self.frames.append((direction, frame))
            with (self.output / "index.jsonl").open("a") as stream:
                stream.write(json.dumps({
                    "file": filename, "direction": direction, "monotonic_ns": time.monotonic_ns(),
                    "message_type": frame["message_type"], "message_id": frame["message_id"],
                    "trial_id": frame.get("trial_id"), "step": frame.get("step"),
                }) + "\n")
        return frame

    def _send(self, connection, request, kind, payload, *, message_id=None):
        raw = pack_frame({
            **request, "message_type": kind, "payload": payload,
            "message_id": request["message_id"] if message_id is None else message_id,
        })
        self._record("reply", raw)
        connection.send(raw)

    def _actions(self, observation, anchor, count):
        assert observation["data_format_version"] == "v1.0"
        assert observation["vision"] == {}  # These examples have no camera inputs.
        state = observation["state"]
        prefixes = sorted(key.removesuffix("_arm_joint_state") for key in state
                          if key.endswith("_arm_joint_state"))
        assert prefixes in [["right"], ["left", "right"]]
        rows = [{} for _ in range(self.horizon)]
        for prefix in prefixes:
            arm_key, hand_key = f"{prefix}_arm_joint_state", f"{prefix}_ee_joint_state"
            assert np.shape(state[arm_key]) == (7,) and np.shape(state[hand_key]) == (20,)
            for step, row in enumerate(rows):
                phase = count * 0.15 + step * 0.04
                row[hand_key] = anchor[hand_key] + 0.02 * np.sin(phase + np.arange(20) * 0.1)
                if self.representation == "joint":
                    row[arm_key] = anchor[arm_key] + 0.01 * np.sin(phase + np.arange(7) * 0.1)
                else:
                    pose_key = f"{prefix}_ee_pose"
                    row[pose_key] = np.array(anchor[pose_key], copy=True)
                    row[pose_key][0] += 0.0001 * np.sin(phase)
        return rows

    def _handle(self, connection):
        from websockets.exceptions import ConnectionClosed

        with self._lock:
            self._connections.add(connection)
        anchor, count, pending = None, 0, None
        try:
            for raw in connection:
                request = self._record("request", raw)
                kind = request["message_type"]
                if kind == "hello":
                    self._send(connection, request, "hello_ack", {
                        "server": "synthetic-loopback-validation",
                        "capabilities": {"sampling_modes": ["default"], "multi_inflight": True},
                        "model_metadata": {"source": "synthetic", "action_semantics": (
                            "wrong_semantics" if self.fault == "identity" else self.semantics
                        )},
                    })
                elif kind == "reset":
                    if self.fault == "reset_error":
                        self._send(connection, request, "error", {
                            "message": "synthetic reset failure",
                        })
                        continue
                    anchor, count = None, 0
                    self._send(connection, request, "reset_result", {})
                elif kind == "infer":
                    count += 1
                    observation = request["payload"]["observation"]
                    assert request["payload"]["sampling"] == {"mode": "default"}
                    if anchor is None:
                        anchor = observation["state"]
                    if count >= 2 and self.fault == "disconnect":
                        connection.close()
                        return
                    if count >= 2 and self.fault == "timeout":
                        self._stop.wait(2.0)
                    if count >= 2 and self.fault == "late":
                        self._stop.wait(0.2)
                    if count >= 2 and self.fault == "error":
                        self._send(connection, request, "error", {"message": "synthetic failure"})
                        continue
                    actions = self._actions(observation, anchor, count)
                    if count >= 2 and self.fault == "shape":
                        actions[0]["right_ee_joint_state"] = np.zeros(19)
                    payload = {"actions": actions, "action_semantics": self.semantics}
                    if self.fault == "reverse_pair":
                        if pending is None:
                            pending = (request, payload)
                            continue
                        self._send(connection, request, "infer_result", payload)
                        self._send(connection, pending[0], "infer_result", pending[1])
                        pending = None
                        continue
                    if self.fault == "unrelated_id":
                        self._send(connection, request, "infer_result", {"actions": []},
                                   message_id="unrelated-synthetic-request")
                    self._send(connection, request, "infer_result", payload)
                else:
                    raise ValueError(f"Unexpected synthetic request: {kind}")
        except ConnectionClosed:
            pass
        except Exception as error:
            self.errors.append(repr(error))
            connection.close()
        finally:
            with self._lock:
                self._connections.discard(connection)

    def __exit__(self, *_):
        self._stop.set()
        with self._lock:
            connections = tuple(self._connections)
        for connection in connections:
            connection.close()
        self.server.shutdown()
        self.thread.join(5)
        if self.thread.is_alive():
            raise RuntimeError("synthetic server did not stop")


def case_config(output, server, *, sides="right", representation="joint", executor="direct",
                algorithm="serial", streaming=False, decoding="inline", steps=100):
    """Compose a complete temporary experiment through the public config loader."""
    suffix = "_right" if sides == "right" else ""
    semantics = POSE_SEMANTICS if representation == "pose" else "absolute_joint_position"
    raw = {
        "_base_": str(EXAMPLES / f"disconnected{suffix}.yaml"),
        "run": {"max_control_steps": steps},
        "policy": {
            "worker": "xpolicylab_ws", "timeout_s": 0.5, "startup_timeout_s": 20,
            "action_decoding": decoding,
            "options": {"server": server, "action_format": representation,
                        "request_timeout_s": 0.5},
            "expected_backend": {"model": {"action_semantics": semantics}},
            "adapter": {"type": f"manimux.policy_adapter.{representation}:"
                        + ("PoseAdapter" if representation == "pose" else "JointAdapter")},
        },
        "executor": {"type": executor},
        "inference": {"algorithm": algorithm},
    }
    if representation == "pose":
        raw["policy"]["adapter"]["action_semantics"] = semantics
    if streaming:
        raw["inference"].update(inference_schedule="multi_inflight", request_trigger="continuous",
                                observation_hz=20)
    elif algorithm == "async":
        raw["inference"]["refill_threshold_s"] = 0.08
    path = Path(output) / "experiment.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(raw, sort_keys=False))
    return load_config(path)


def run_case(output, **options):
    """Return concrete evidence from the actual runtime, never a driver mock."""
    output = Path(output).resolve()
    with forbid_motor_imports(), SyntheticServer(
        output / "wire", representation=options.get("representation", "joint")
    ) as server:
        config = case_config(output, server.url, **options)
        runtime = build_runtime(config, output / "runtime")
        robot = runtime._robot
        sent, samples = robot.send_command, []

        def capture(command):
            sent(command)
            # Read back the disconnected endpoints, not the requested command.
            # These are memory writes, not firmware acknowledgments.
            groups = {
                name: np.r_[robot.controller.target[arm.channel],
                            robot.end_effectors[name].target]
                for name, arm in robot.arm_components.items()
            }
            for name, values in command.groups.items():
                np.testing.assert_allclose(groups[name], values, atol=1e-12)
            samples.append((command.monotonic_ns, groups))

        with patch.object(robot, "send_command", capture):
            result = runtime.run()
        if result.accepted_plans < 2 or result.rejected_plans:
            raise AssertionError(f"Unexpected admission result: {result}")
        assert not runtime._robot.controller.connected
        assert not any(tool.connected for tool in runtime._robot.end_effectors.values())
        assert not server.errors, server.errors
        np.savez(output / "memory_commands.npz",
                 monotonic_ns=[stamp for stamp, _ in samples],
                 **{name: np.stack([groups[name] for _, groups in samples])
                    for name in config["robot"]["group_dims"]})
        verify_recording(result.episode_dir, output / "memory_commands.npz",
                         config["robot"]["group_dims"])
        evidence = {
            **options, "source": "synthetic_loopback_and_memory_endpoints",
            "hardware_connected": False, "episode": str(result.episode_dir),
            "accepted_plans": result.accepted_plans, "rejected_plans": result.rejected_plans,
            "wire_frames": len(server.frames),
            "memory_command_samples": len(samples),
        }
        (output / "report.json").write_text(json.dumps(evidence, indent=2) + "\n")
        return evidence


def verify_recording(episode, memory_commands, group_dims):
    """Compare recorded commands to endpoint writes on exactly the same ticks."""
    import zarr

    store = zarr.open_group(str(Path(episode) / "data.zarr"), mode="r")
    times = store["ticks/monotonic_ns"][:]
    assert np.all(np.diff(times) > 0)
    with np.load(memory_commands) as sent:
        indices = np.searchsorted(sent["monotonic_ns"], times)
        np.testing.assert_array_equal(sent["monotonic_ns"][indices], times)
        for source in ("state", "scheduled", "command"):
            assert set(store[f"ticks/{source}"]) == set(group_dims)
            for name, dim in group_dims.items():
                values = store[f"ticks/{source}/{name}"][:]
                assert values.shape == (len(times), dim) and np.isfinite(values).all()
                if source == "command":
                    np.testing.assert_allclose(values, sent[name][indices], atol=1e-12)
                    assert np.ptp(values[:, 7:], axis=0).min() > 0
    for plan in store["plans"].values():
        assert set(plan["canonical_raw"]) == set(group_dims)
        for name, dim in group_dims.items():
            rows = plan[f"canonical_raw/{name}"][:]
            assert rows.shape[1] == dim and np.isfinite(rows).all()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New artifact directory")
    parser.add_argument("--matrix", action="store_true", help="Run all 16 basic combinations")
    args = parser.parse_args()
    cases = itertools.product(
        ["right", "dual"] if args.matrix else ["right"],
        ["joint", "pose"] if args.matrix else ["joint"],
        ["direct", "smooth"] if args.matrix else ["direct"],
        ["serial", "async"] if args.matrix else ["async"],
    )
    reports = []
    for sides, representation, executor, algorithm in cases:
        label = f"{sides}-{representation}-{executor}-{algorithm}"
        report = run_case(args.output / label, sides=sides, representation=representation,
                          executor=executor, algorithm=algorithm)
        reports.append(report)
        print(json.dumps(report), flush=True)
    (args.output / "report.json").write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
