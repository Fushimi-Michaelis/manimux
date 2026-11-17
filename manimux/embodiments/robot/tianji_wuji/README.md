# Tianji-Wuji assembly

- `robot.py`: component binding, one shared Tianji controller, independent Wuji
  hand endpoints, configured runtime Home and assembly-owned tool release gate.
- `configs/embodiment/robot/tianji_wuji{,_right}.yaml`: dual/right-only component
  assemblies, with required physical mounts left unbound.
- `configs/embodiment/robot/tianji_wuji{,_right}_control.yaml`: explicit arm/tool
  envelopes; Direct and Smooth selections live under `configs/executor/`.
- `configs/examples/tianji_wuji/`: synthetic-policy config inspection examples.
- `../tianji_home.py`: shared arm-only Home settings and cosine interpolation.

The [assembly contract](../../../../.agents/skills/manimux-development/references/tianji-wuji.md)
owns coordinates, mounting, SDK preparation, lifecycle and validation scope.
This package does not define collection, a trained model or idle hand guiding.
Arm Home targets match ManiMux's `tianji_taccap_pack_plate.yaml`. Runtime Home
holds the measured hand pose until every arm arrives, then ramps controlled
hands to their configured zero targets and verifies arrival and holding.
