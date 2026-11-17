"""Extract local bare-arm descriptions without changing the selected SDK model."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[2] / "manimux/embodiments/arm/tianji/assets/marvin"
)
PACKAGE = "package://marvin_waist_mobile_description/"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mesh_path(filename: str, package_root: Path) -> Path:
    if not filename.startswith(PACKAGE):
        raise ValueError(f"unexpected mesh URI: {filename}")
    path = (package_root / filename.removeprefix(PACKAGE)).resolve()
    if not path.is_relative_to(package_root.resolve()):
        raise ValueError(f"mesh escapes package root: {filename}")
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def prepare(urdf: Path, package_root: Path, destination: Path) -> dict:
    """Preserve source joints/meshes; keep generated assets separate from active visuals.

    The source parent of J1 is the extracted arm base. The link after the source
    fixed JointTCP is renamed Flange_L/R. No zero, sign, limit or flange offset
    is fitted. The source grippers are omitted; no Wuji mount is inferred.
    """
    root = ET.parse(urdf).getroot()
    if root.get("name") != "marvin_waist_mobile":
        raise ValueError("expected the selected marvin_waist_mobile description")
    elements = root.findall("link") + root.findall("joint")
    if len({e.get("name") for e in elements}) != len(elements):
        raise ValueError("source link/joint names must be unique")
    links = {e.get("name"): e for e in root.findall("link")}
    joints = {e.get("name"): e for e in root.findall("joint")}
    if destination.exists():
        raise FileExistsError(f"refusing to replace existing assets: {destination}")
    manifest = {
        "source_urdf": str(urdf.resolve()),
        "source_sha256": sha256(urdf),
        "source_robot": root.get("name"),
        "sdk_mapping_verified": False,
        "hand_mounts_calibrated": False,
        "source_has_wuji_geometry": False,
        "sides": {},
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        staging = Path(temporary) / "description"
        for side, suffix in (("left", "L"), ("right", "R")):
            chain = [joints[f"Joint{i}_{suffix}"] for i in range(1, 8)]
            chain.append(joints[f"JointTCP_{suffix}"])
            if any(j.get("type") != "revolute" for j in chain[:7]):
                raise ValueError(f"{side}: expected seven revolute joints")
            if chain[7].get("type") != "fixed":
                raise ValueError(f"{side}: JointTCP must be the fixed source flange")
            base = chain[0].find("parent").get("link")
            children = [j.find("child").get("link") for j in chain]
            if [j.find("parent").get("link") for j in chain[1:]] != children[:-1]:
                raise ValueError(f"{side}: arm joints must form a serial chain")
            flange = f"Flange_{suffix}"
            result = ET.Element("robot", name=f"tianji_description_{side}")
            # The torso, lift, opposite arm and hands are separate from this bare arm.
            for material in root.findall("material"):
                result.append(deepcopy(material))
            output = staging / side
            (output / "meshes").mkdir(parents=True)
            meshes = {}
            for child in [base, *children]:
                link = deepcopy(links[child])
                if child == children[-1]:
                    link.set("name", flange)
                for mesh in link.findall(".//mesh"):
                    source = _mesh_path(mesh.get("filename"), package_root)
                    name = source.name
                    digest = sha256(source)
                    if name in meshes and meshes[name]["sha256"] != digest:
                        raise ValueError(f"conflicting mesh basenames: {name}")
                    meshes[name] = {"source": str(source), "sha256": digest}
                    shutil.copyfile(source, output / "meshes" / name)
                    mesh.set("filename", f"meshes/{name}")
                result.append(link)
            for joint in chain:
                joint = deepcopy(joint)
                if joint.find("child").get("link") == children[-1]:
                    joint.find("child").set("link", flange)
                result.append(joint)
            ET.indent(result)
            ET.ElementTree(result).write(
                output / "arm.urdf", encoding="utf-8", xml_declaration=True
            )
            manifest["sides"][side] = {
                "urdf": f"{side}/arm.urdf",
                "urdf_sha256": sha256(output / "arm.urdf"),
                "base_link": base,
                "flange_link": flange,
                "source_flange_link": children[-1],
                "joint_names": [j.get("name") for j in chain[:7]],
                "hand_mount": None,
                "meshes": meshes,
            }
        (staging / "source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        staging.rename(destination)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--package-root", required=True, type=Path, help="URDF package directory")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    manifest = prepare(args.urdf, args.package_root, args.output)
    print(f"Prepared {manifest['source_robot']} in {args.output}; SDK mapping remains unverified.")


if __name__ == "__main__":
    main()
