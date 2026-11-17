"""Per-group action layout, independent of policy transports and hardware SDKs."""

from copy import deepcopy
from pathlib import Path

import yaml


def _layout_widths(layout: dict, label: str) -> tuple[int, int, int]:
    arm = layout.get("arm_dofs", 0)
    gripper = layout.get("gripper_dofs", 0)
    joint_tool = layout.get("joint_tool_dofs", 0)
    if any(
        not isinstance(value, int) or isinstance(value, bool) or value < 0
        for value in (arm, gripper, joint_tool)
    ):
        raise ValueError(f"invalid action layout for {label}")
    if gripper and joint_tool:
        raise ValueError(f"{label}: gripper and joint tool coordinates cannot share a group")
    return arm, gripper, joint_tool


def assembly_action_contract(path: str | Path) -> dict | None:
    """Resolve component action widths without loading geometry or opening hardware.

    Integrated arms declare both widths. A separate end effector contributes its
    own coordinates. Existing assemblies with action_contract remain readable.
    """
    source = Path(path)
    assembly = yaml.safe_load(source.read_text())
    contract = dict(assembly.get("action_contract") or {})
    layouts = {}
    for name, group in assembly["groups"].items():
        layout = {"arm_dofs": 0, "gripper_dofs": 0}
        for component_name in (group["arm"], group.get("end_effector")):
            if component_name is None:
                continue
            entry = assembly["components"][component_name]
            component = yaml.safe_load((source.parent / entry["config"]).read_text())
            declared = component.get("action_layout")
            if declared is None:
                # Compatibility for assemblies whose SDK/component metadata is not migrated.
                return contract or None
            arm, gripper, joint_tool = _layout_widths(declared, f"component {component_name!r}")
            layout["arm_dofs"] += arm
            layout["gripper_dofs"] += gripper
            if joint_tool:
                layout["joint_tool_dofs"] = layout.get("joint_tool_dofs", 0) + joint_tool
        layouts[name] = layout
    contract["group_layouts"] = layouts
    return contract


def group_layouts(group_dims: dict, options: dict) -> dict[str, dict[str, int]]:
    """Resolve arm, scalar-gripper and joint-tool coordinate ownership.

    Joint tools require explicit per-group joint_tool_dofs. Former arm/gripper
    layouts retain their representation and uniform split at the boundary.
    """
    declared = options.get("group_layouts")
    if declared is None:
        if options.get("joint_tool_dofs", 0):
            raise ValueError("joint tools require explicit per-group action layouts")
        gripper = options.get("gripper_dofs", 1)
        declared = {
            name: {"arm_dofs": dim - gripper, "gripper_dofs": gripper}
            for name, dim in group_dims.items()
        }
    if (
        "group_layouts" in options
        and "gripper_dofs" in options
        and any(
            item.get("gripper_dofs", 0) != options["gripper_dofs"] for item in declared.values()
        )
    ):
        raise ValueError("uniform gripper_dofs conflicts with the per-group layout")
    if set(declared) != set(group_dims):
        raise ValueError("action layout groups must match robot groups")
    resolved = {}
    for name in group_dims:
        layout = dict(declared[name])
        arm, gripper, joint_tool = _layout_widths(layout, repr(name))
        if (
            not isinstance(group_dims[name], int)
            or isinstance(group_dims[name], bool)
            or arm <= 0
            or arm + gripper + joint_tool != group_dims[name]
        ):
            raise ValueError(f"invalid action layout for {name!r}")
        layout["gripper_dofs"] = gripper
        resolved[name] = layout
    return resolved


def joint_tool_slices(layouts: dict) -> dict[str, slice]:
    """Locate declared joint tools without assigning scalar gripper semantics."""
    result = {}
    for name, layout in layouts.items():
        arm, _, joint_tool = _layout_widths(layout, repr(name))
        if joint_tool:
            result[name] = slice(arm, arm + joint_tool)
    return result


def gripper_indices(layouts: dict) -> dict[str, int]:
    """Return only normalized scalar grippers; joint tools have separate slices."""
    if any(layout["gripper_dofs"] > 1 for layout in layouts.values()):
        raise ValueError("multi-coordinate end effectors require a matching executor capability")
    return {name: layout["arm_dofs"] for name, layout in layouts.items() if layout["gripper_dofs"]}


def _set_contract_value(target: dict, name: str, value: object, label: str) -> None:
    if name in target and target[name] != value:
        raise ValueError(f"{label} conflicts with embodiment action_contract")
    target[name] = deepcopy(value)


def action_contract_group_indices(config: dict) -> dict[str, int] | None:
    """Return each group's gripper coordinate from its resolved action contract."""

    dims = config.get("robot", {}).get("group_dims")
    options = config.get("policy", {}).get("adapter", {})
    if not dims or not ({"group_layouts", "gripper_dofs"} & options.keys()):
        return None
    return gripper_indices(group_layouts(dims, options))


def apply_action_contract(config: dict, contract: dict) -> dict[str, int]:
    """Apply one embodiment-owned action layout to policy and execution sections."""

    dimensions = contract.get("group_dims")
    if "group_layouts" in contract:
        dimensions = {
            name: sum(_layout_widths(item, repr(name)))
            for name, item in contract["group_layouts"].items()
        }
        if "group_dims" in contract and dimensions != contract["group_dims"]:
            raise ValueError("component action layout conflicts with assembly dimensions")
    layouts = group_layouts(dimensions, contract)
    indices = gripper_indices(layouts)
    execution = config.setdefault("executor", {})
    apply_executor_layout(execution, layouts)
    robot = config.setdefault("robot", {})
    adapter = config.setdefault("policy", {}).setdefault("adapter", {})
    _set_contract_value(robot, "group_dims", dimensions, "robot.group_dims")
    _set_contract_value(robot, "group_layouts", layouts, "robot.group_layouts")
    for name, value in {
        "group_order": list(dimensions),
        "group_prefixes": contract.get("group_prefixes", {name: name for name in dimensions}),
        "group_layouts": layouts,
    }.items():
        _set_contract_value(adapter, name, value, f"policy.adapter.{name}")
    return indices


def apply_executor_layout(
    execution: dict,
    layouts: dict,
    *,
    require_joint_tools: bool = False,
) -> None:
    """Bind only the declared tool capabilities to execution and control profiles."""
    indices = gripper_indices(layouts)
    tools = {name: [part.start, part.stop] for name, part in joint_tool_slices(layouts).items()}
    if tools and execution.get("type", "smooth") not in {"direct", "smooth"}:
        raise ValueError("joint tools require a direct or smooth executor")
    motion = execution.get("motion_limits")
    if isinstance(motion, dict):
        _set_contract_value(
            motion.setdefault("gripper", {}),
            "group_indices",
            indices,
            "executor.motion_limits.gripper.group_indices",
        )
    smooth = execution.get("smooth")
    if isinstance(smooth, dict) and isinstance(smooth.get("gripper"), dict):
        _set_contract_value(
            smooth["gripper"],
            "group_indices",
            indices,
            "executor.smooth.gripper.group_indices",
        )
    for label, section in (("motion_limits", motion), ("smooth", smooth)):
        if not isinstance(section, dict):
            continue
        declared_tools = section.get("joint_tool")
        if tools:
            if label == "smooth" and declared_tools is None and isinstance(motion, dict):
                declared_tools = deepcopy(motion.get("joint_tool"))
                if declared_tools is not None:
                    section["joint_tool"] = declared_tools
            if not isinstance(declared_tools, dict):
                if require_joint_tools and (
                    label == "motion_limits" or execution.get("type", "smooth") == "smooth"
                ):
                    raise ValueError(
                        f"executor.{label}.joint_tool limits are required for joint tools"
                    )
                continue
            _set_contract_value(
                declared_tools, "group_slices", tools, f"executor.{label}.joint_tool.group_slices"
            )
        elif declared_tools is not None:
            raise ValueError(f"executor.{label}.joint_tool requires a joint-tool action layout")
    if (
        require_joint_tools
        and tools
        and execution.get("type", "smooth") == "smooth"
        and not isinstance(smooth, dict)
        and (not isinstance(motion, dict) or motion.get("joint_tool") is None)
    ):
        raise ValueError("smooth execution requires explicit joint_tool limits")
