"""Audit extracted Tianji-Wuji geometry using FK only, without motor SDK objects."""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from yourdfpy import URDF

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from manimux.embodiments.arm.base import ArmModel  # noqa: E402
from manimux.embodiments.end_effector.wuji import WujiHand2  # noqa: E402
from manimux.embodiments.robot.base import MountedGroup  # noqa: E402
from manimux.kinematics.base import FlangeKinematicsBase, KinematicCoordinate  # noqa: E402
from manimux.kinematics.composed import ComposedManipulatorKinematics  # noqa: E402
from manimux.kinematics.end_effector import Frame  # noqa: E402
from scripts.assets.prepare_tianji_wuji import DEFAULT_OUTPUT, sha256  # noqa: E402

# This nonzero fixture tests composition; it is never installed as a station mount.
SYNTHETIC_MOUNT = Frame(xyz=(0.012, -0.023, 0.034), rpy=(0.1, -0.2, 0.3))


def origin(joint: ET.Element) -> np.ndarray:
    pose = np.eye(4)
    value = joint.find("origin")
    if value is not None:
        pose[:3, 3] = np.fromstring(value.get("xyz", "0 0 0"), sep=" ")
        pose[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(value.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return pose


def source_fk(root: ET.Element, base: str, link: str, configuration: dict) -> np.ndarray:
    """Evaluate the source tree; unselected lift/waist joints stay at zero."""
    parents = {j.find("child").get("link"): j for j in root.findall("joint")}
    path, visited = [], set()
    while link != base:
        if link in visited:
            raise ValueError("source URDF contains a cycle")
        visited.add(link)
        joint = parents[link]
        path.append(joint)
        link = joint.find("parent").get("link")
    pose = np.eye(4)
    for joint in reversed(path):
        pose = pose @ origin(joint)
        if joint.get("type") in {"revolute", "continuous", "prismatic"}:
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            value = configuration.get(joint.get("name"), 0.0)
            motion = np.eye(4)
            if joint.get("type") == "prismatic":
                motion[:3, 3] = axis * value
            else:
                motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
            pose = pose @ motion
    return pose


class DescriptionFlange(FlangeKinematicsBase):
    """Probe-only source FK; it provides no IK, driver or runtime factory."""

    num_arm_joints = 7

    def __init__(self, path: Path, flange: str):
        self.visual = URDF.load(str(path), load_meshes=True)
        self.flange = flange
        if len(self.visual.actuated_joint_names) != 7:
            raise ValueError("extracted arm must have exactly seven actuated joints")

    def fk_flange(self, joints):
        self.visual.update_cfg(dict(zip(self.visual.actuated_joint_names, joints, strict=True)))
        return self.visual.get_transform(self.flange).copy()

    def ik_flange(self, target_flange, seed_joints):
        raise NotImplementedError("description audit provides FK only, not a control solver")


def axis_distance(first, second) -> float:
    """Shortest distance between two infinite rotation axes (metres).

    For adjacent joints this is invariant to base/tool transforms, joint zero
    offsets and sign reversals. It can disprove a proposed index correspondence.
    """
    p, u = first
    q, v = second
    cross = np.cross(u, v)
    norm = np.linalg.norm(cross)
    if norm < 1e-8:
        return float(np.linalg.norm(np.cross(q - p, u)))
    return float(abs(np.dot(q - p, cross)) / norm)


def urdf_axes(root: ET.Element, names: list[str]) -> list:
    pose, axes = np.eye(4), []
    for name in names:
        joint = root.find(f"joint[@name='{name}']")
        pose = pose @ origin(joint)
        axis = pose[:3, :3] @ np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        axes.append((pose[:3, 3].copy(), axis / np.linalg.norm(axis)))
    return axes


def audit(
    description: Path,
    *,
    sdk_config: Path | None = None,
    samples: int = 32,
    description_only: bool = False,
) -> dict:
    if samples < 2:
        raise ValueError("samples must be at least two")
    manifest = json.loads((description / "source_manifest.json").read_text())
    source_path = Path(manifest["source_urdf"])
    if sha256(source_path) != manifest["source_sha256"]:
        raise ValueError("source URDF has changed since asset preparation")
    source = ET.parse(source_path).getroot()
    report = {
        "hardware_connected": False,
        "runtime_model_changed": False,
        "qualified_for_hardware": False,
        "source_sha256": manifest["source_sha256"],
        "samples_per_side": samples,
        "hand_mounts_calibrated": False,
        "hand_mount_source": "synthetic nonzero fixture for offline composition only",
        "sdk_joint_mapping": "Joint1..7_L/R -> SDK J1..7; same signs/zeros checked offline",
        "sides": {},
    }
    rng = np.random.default_rng(0)
    groups = {}
    for side, entry in manifest["sides"].items():
        path = description / entry["urdf"]
        if sha256(path) != entry["urdf_sha256"]:
            raise ValueError(f"{side}: extracted URDF has changed")
        for name, data in entry["meshes"].items():
            if sha256(path.parent / "meshes" / name) != data["sha256"]:
                raise ValueError(f"{side}: extracted mesh has changed: {name}")
        arm = DescriptionFlange(path, entry["flange_link"])
        if arm.visual.actuated_joint_names != entry["joint_names"]:
            raise ValueError(f"{side}: unexpected visual joint order")
        tool = WujiHand2.load_model(side=side, model_version="2.2")
        hand_names = [d.joint for d in sorted(tool.visual.spec.joints, key=lambda d: d.input)]
        hand_root = ET.parse(tool.visual.urdf_path).getroot()
        hand_visual = URDF.load(str(tool.visual.urdf_path), load_meshes=True)
        coordinates = tuple(KinematicCoordinate(f"joint_{i}", "rad") for i in range(1, 8))
        mount = SYNTHETIC_MOUNT
        kin = ComposedManipulatorKinematics(
            arm,
            tool.geometry,
            arm_coordinates=coordinates,
            mount=mount.matrix(),
            base_frame=entry["base_link"],
        )
        group = MountedGroup(
            f"{side}_arm",
            ArmModel(arm, coordinates, path, entry["flange_link"]),
            f"{side}_hand",
            tool,
            mount,
            kin,
        )
        groups[f"{side}_arm"] = group
        visual = URDF.load(str(group.visual_urdf()), load_meshes=True)
        if len(visual.actuated_joint_names) != 27:
            raise ValueError(f"{side}: expected 27 visual joints")
        arm_root = ET.parse(path).getroot()
        arm_limits = np.array(
            [
                [float(source.find(f"joint[@name='{n}']/limit").get(k)) for k in ("lower", "upper")]
                for n in entry["joint_names"]
            ]
        )
        hand_limits = np.array(
            [
                [
                    float(hand_root.find(f"joint[@name='{n}']/limit").get(k))
                    for k in ("lower", "upper")
                ]
                for n in hand_names
            ]
        )
        # The zero pose and deterministic in-range poses are synthetic observations.
        poses = [np.clip(np.zeros(27), *np.r_[arm_limits, hand_limits].T)]
        for _ in range(samples - 1):
            limits = np.r_[arm_limits, hand_limits]
            poses.append(rng.uniform(limits[:, 0], limits[:, 1]))
        max_flange = max_tcp = max_visual = 0.0
        for q in poses:
            values = dict(zip(entry["joint_names"], q[:7], strict=True))
            flange = source_fk(source, entry["base_link"], entry["source_flange_link"], values)
            hand_visual.update_cfg(dict(zip(hand_names, q[7:], strict=True)))
            tcp = flange @ mount.matrix() @ hand_visual.get_transform(f"{side[0]}_wrist")
            max_flange = max(max_flange, float(np.max(abs(flange - arm.fk_flange(q[:7])))))
            max_tcp = max(max_tcp, float(np.max(abs(tcp - kin.fk(q)))))
            visual.update_cfg(
                dict(
                    zip(
                        visual.actuated_joint_names,
                        group.visual_configuration(q),
                        strict=True,
                    )
                )
            )
            max_visual = max(max_visual, float(np.max(abs(tcp - visual.get_transform("ee_tcp")))))
        if max(max_flange, max_tcp, max_visual) > 1e-10:
            raise ValueError(f"{side}: source/extracted/composed visual FK mismatch")
        result = {
            "joint_names": entry["joint_names"],
            "hand_joint_names": hand_names,
            "group_dim": kin.num_coordinates,
            "mesh_count": len(entry["meshes"]),
            "source_flange_max_matrix_error": max_flange,
            "source_tcp_max_matrix_error": max_tcp,
            "visual_tcp_max_matrix_error": max_visual,
            "source_has_wuji_geometry": False,
            "source_hand_mount": None,
            "synthetic_test_mount": mount.model_dump(),
        }
        if not description_only:
            result["sdk"] = compare_sdk(side, sdk_config, arm, arm_root, entry, poses)
        report["sides"][side] = result
    report["layouts"] = {
        "right_only": {"right_arm": groups["right_arm"].kinematics.num_coordinates},
        "dual": {name: group.kinematics.num_coordinates for name, group in groups.items()},
    }
    report["description_assembly_passed"] = True
    report["sdk_geometry_compatible"] = (
        None if description_only else all(r["sdk"]["compatible"] for r in report["sides"].values())
    )
    return report


def compare_sdk(side, config, arm, root, entry, poses) -> dict:
    # Import only the vendor kinematics library. No Marvin_Robot or Wuji manager is created.
    from manimux.embodiments.arm.tianji.kinematics import TianjiSDKKinematics, _link_transform

    sdk = TianjiSDKKinematics(side, **({"config_path": config} if config is not None else {}))
    axes = []
    pose = np.eye(4)
    for row in sdk._native_dh[:7]:
        pose = pose @ _link_transform(*row)
        axes.append((pose[:3, 3].copy(), pose[:3, 2].copy()))
    source_axes = urdf_axes(root, entry["joint_names"])
    distances = [
        {
            "joints": [i + 1, i + 2],
            "urdf_m": axis_distance(source_axes[i], source_axes[i + 1]),
            "sdk_m": axis_distance(axes[i], axes[i + 1]),
        }
        for i in range(6)
    ]
    max_distance = max(abs(d["urdf_m"] - d["sdk_m"]) for d in distances)
    errors = []
    for q in poses:
        native = sdk.fk_flange(q[:7])
        dh = np.eye(4)
        for i, row in enumerate(sdk._native_dh):
            alpha, a, d, theta = row
            angle = np.degrees(q[i]) if i < 7 else 0.0
            dh = dh @ _link_transform(alpha, a, d, theta + angle)
        np.testing.assert_allclose(dh, native, atol=1e-10)
        expected, actual = arm.fk_flange(q[:7]), native
        errors.append(
            {
                "joints_rad": q[:7].tolist(),
                "position_error_m": float(np.linalg.norm(expected[:3, 3] - actual[:3, 3])),
                "rotation_error_rad": float(
                    Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude()
                ),
            }
        )
    return {
        "config_path": str(sdk.config_path),
        "config_sha256": sha256(sdk.config_path),
        "adjacent_axis_distances": distances,
        "max_axis_distance_difference_m": max_distance,
        "axis_distance_tolerance_m": 1e-4,
        "compatible": max_distance <= 1e-4
        and all(e["position_error_m"] <= 1e-4 and e["rotation_error_rad"] <= 1e-3 for e in errors),
        "pose_comparison": "direct Base_L/R to TCP_Link_L/R; no alignment, sign or zero fitting",
        "urdf_joint_limits_deg": [
            [
                float(np.degrees(float(root.find(f"joint[@name='{n}']/limit").get(k))))
                for k in ("lower", "upper")
            ]
            for n in entry["joint_names"]
        ],
        "sdk_joint_limits_deg": [[r[1], r[0]] for r in sdk._pnva],
        "urdf_velocity_limits_rad_s": [
            float(root.find(f"joint[@name='{n}']/limit").get("velocity"))
            for n in entry["joint_names"]
        ],
        "sdk_velocity_limits_rad_s": [float(np.radians(r[2])) for r in sdk._pnva],
        "poses": errors,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sdk-config", type=Path)
    parser.add_argument(
        "--description-only", action="store_true", help="skip native SDK comparison"
    )
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = audit(
        args.description,
        sdk_config=args.sdk_config,
        samples=args.samples,
        description_only=args.description_only,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Description assembly passed; SDK geometry compatible: {result['sdk_geometry_compatible']}"
    )
    print(f"Report: {args.output}")
    if result["sdk_geometry_compatible"] is False:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
