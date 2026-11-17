"""Offline Wuji Hand 2 coordinates and fixed palm reference geometry."""

from dataclasses import replace
from pathlib import Path

from scipy.spatial.transform import Rotation

from manimux.kinematics.base import KinematicCoordinate, rigid_transform
from manimux.kinematics.end_effector import Frame, load_end_effector
from manimux.kinematics.tool import FixedToolGeometry

COORDINATES = tuple(
    KinematicCoordinate(f"{finger}_joint{joint}", "rad")
    for finger in ("thumb", "index", "middle", "ring", "pinky")
    for joint in range(1, 5)
)


def hand_model(side: str, model_version: str, *, tcp_transform=None):
    if side not in {"left", "right"}:
        raise ValueError("Wuji side must be 'left' or 'right'")
    if model_version != "2.2":
        raise ValueError("only the explicit Wuji Hand 2.2 description is bundled")
    visual = load_end_effector(Path(__file__).parent / "assets" / "hand2_2" / side)
    if tcp_transform is not None:
        pose = rigid_transform(tcp_transform, "Wuji palm TCP")
        tcp = Frame(
            xyz=tuple(pose[:3, 3]),
            rpy=tuple(Rotation.from_matrix(pose[:3, :3]).as_euler("xyz")),
        )
        visual = replace(visual, spec=visual.spec.model_copy(update={"tcp": tcp}))
    return visual


class WujiGeometry(FixedToolGeometry):
    """Twenty radian inputs, with a palm TCP fixed relative to the hand mount.

    The default TCP is the upstream CAD wrist reference, not a calibrated
    grasp/contact point. Finger motion is displayed through the visual model's
    explicit joint drives; it does not move this TCP. Installation transforms
    and payload calibration belong to the robot assembly. No SDK is imported.
    """

    def __init__(
        self,
        *,
        side: str,
        model_version: str,
        tcp_transform=None,
        base_frame: str | None = None,
        tcp_frame: str | None = None,
    ) -> None:
        visual = hand_model(side, model_version)
        super().__init__(
            visual.spec.tcp.matrix() if tcp_transform is None else tcp_transform,
            coordinates=COORDINATES,
            base_frame=base_frame or f"wuji_{side}_mount",
            tcp_frame=tcp_frame or f"wuji_{side}_palm_tcp",
        )
