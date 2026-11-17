# Wuji Hand 2

- `end_effector.py`: owned SDK connection and 20-coordinate position control.
- `geometry.py`: offline finger-major coordinates and fixed palm TCP.
- `assets/`: pinned official Hand 2.2 URDFs, referenced meshes and MIT notice.
- `configs/embodiment/end_effector/wuji_hand2_{left,right}.yaml`: component
  selections with explicit model limits and unbound station hardware settings.

SDK identity, units, timestamps, lifecycle and verification boundaries are
maintained in the [component contract](../../../../.agents/skills/manimux-development/references/wuji-hand2.md).
This component does not define Tianji mounting, payload, Home or rollout recovery.
