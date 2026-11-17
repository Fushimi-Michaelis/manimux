# Tianji-Wuji offline examples

`offline_right.yaml` selects one 27-coordinate group; `offline.yaml` selects two
groups (54 coordinates). Both use a synthetic policy and disable arm/tool
execution. They are configuration/assembly examples, not learned-policy or
physical-station qualification.

`disconnected_right.yaml` and `disconnected.yaml` select explicit in-memory robot
endpoints and a synthetic worker for complete runtime/recording demonstrations.
`disconnected_gui*.yaml` enables the manual GUI support-confirmation flow.
`synthetic_mounts*.yaml` is the single source of their uncalibrated test transforms;
the examples explicitly select this local file instead of the default station.
`disconnected_control*.yaml` inherits production limits, changing only the factory.

`home_targets.template.yaml` is an optional fragment for a private assembly copy,
not a runnable experiment. Its arm targets match the shipped assemblies and
ManiMux's pack-plate preset. Executing connections validate all Home targets;
read-only inspection does not require them. The assembly contract owns measured
arrival, hand zeroing and shutdown recovery.

Inspect through `manimux.cli.load_config()`. Model construction additionally needs
the separately installed native Marvin geometry and arm visual resources. The
physical factory needs measured station bindings; the disconnected factory rejects
hardware bindings and constructs no motor SDK.
The selecting files, required bindings and lifecycle are documented in the
[assembly contract](../../../../.agents/skills/manimux-development/references/tianji-wuji.md).
