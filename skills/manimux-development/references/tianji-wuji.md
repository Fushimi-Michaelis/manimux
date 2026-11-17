# Tianji-Wuji assembly

`manimux.embodiments.robot.tianji_wuji:create_robot` is a normal `module:factory`
selection. It creates `TianjiWujiRobot` without a runtime robot-name branch.
It reuses `RobotModel`, `RobotBase`, the existing `TianjiArm` /
`TianjiController`, the selected SDK bare-flange FK/IK and the
[Wuji Hand 2 component](wuji-hand2.md).
The selected M6_40 geometry passes offline comparison with the corrected
`robot/urdf/marvin_waist_mobile.urdf`; see the geometry audit below.

## Configuration and ownership

| Selection | Files under `manimux/configs/` |
| --- | --- |
| Right arm + right hand | `embodiment/robot/tianji_wuji_right.yaml`, `tianji_wuji_right_control.yaml` |
| Both arms + hands | `embodiment/robot/tianji_wuji.yaml`, `tianji_wuji_control.yaml` |
| Direct / Smooth | `executor/tianji_wuji_direct.yaml`, `tianji_wuji_smooth.yaml` |
| Config inspection | `examples/tianji_wuji/offline_right.yaml`, `offline.yaml` |
| Disconnected runtime | `examples/tianji_wuji/disconnected_right.yaml`, `disconnected.yaml` |
| Disconnected GUI session | `examples/tianji_wuji/disconnected_gui_right.yaml`, `disconnected_gui.yaml` |
| Station templates | `local/tianji_wuji_right_example.yaml`, `tianji_wuji_example.yaml` |

Group order is `right_arm` alone, or `left_arm`, `right_arm`. Each group is
`[J1..J7, thumb1..4, index1..4, middle1..4, ring1..4, pinky1..4]`, all radians.
The explicit action layout is `{arm_dofs: 7, gripper_dofs: 0, joint_tool_dofs: 20}`.
The existing scalar-gripper path is not used. State preserves measured arm and
hand values; `RobotBase` splits every validated 27-vector into seven arm values
and one complete `JointToolCommand`. Independent arm/hand endpoints are not atomic.

## State and action contract

Public `RobotState.groups` contains measured positions only, in the order above.
The XPolicyLab codec splits each group into `*_arm_joint_state[7]` and
`*_ee_joint_state[20]`; the single right-arm assembly keeps the `right_` prefix.
SDK velocity/effort are not currently exported as public auxiliary observations.
Do not synthesize missing signals as zeros. State and sensor timestamps retain
their existing sample/receipt meaning; see the component's clock limitations.

Two generic action paths are available:

| Adapter | `policy.options.action_format` | Input per group | Output per group |
| --- | --- | --- | --- |
| `manimux.policy_adapter.joint:JointAdapter` | `joint` | 27 absolute joint positions (rad) | 27 absolute joint positions (rad) |
| `manimux.policy_adapter.pose:PoseAdapter` | `pose` | `[xyz, qw, qx, qy, qz, hand[20]]` | 27 absolute joint positions (rad) |

The pose path keeps all 20 hand coordinates fixed by their geometry names and
calls the existing Tianji arm IK. The configured palm TCP includes the station
mount and hand TCP transform. It is neither a fingertip target nor a GUI scene
pose. Translation uses metres; quaternion order is `wxyz` and must have unit norm.
Both paths can have width 27: explicit `format` and action semantics distinguish
them. There is no automatic interpretation based on width.

Supported pose semantics are `absolute_per_arm_base_xyz_wxyz`,
`delta_observation_base_xyz_wxyz` and `delta_observation_tool_xyz_wxyz`.
The first uses the selected arm base. Base deltas add translation in that base
and left-multiply the observation rotation; tool deltas right-multiply the
observation TCP transform. All delta rows share the same request-observation
anchor. **Hand targets are absolute radians in every mode**, without a `[0,1]`
mapping, latch or coupling to the arm's speed scaling. FK poses added to the
request do not replace the original measured arm/hand positions.

For a backend that explicitly implements this contract, the following policy
fragment selects absolute pose decoding. It must be combined with a complete
experiment, the selected assembly and explicit station mounts; it is not a
hardware launch recipe or a declaration of a compatible checkpoint.

```yaml
policy:
  worker: xpolicylab_ws
  action_dt_s: 0.03333333333333333
  horizon_policy_steps: 16
  action_decoding: inline
  options:
    action_format: pose
  expected_backend:
    model:
      action_semantics: absolute_per_arm_base_xyz_wxyz
  adapter:
    type: manimux.policy_adapter.pose:PoseAdapter
    action_semantics: absolute_per_arm_base_xyz_wxyz
    camera_map: {}  # Bind required real camera names explicitly for the backend.
execution:
  ik:
    mode: chunk
    failure_policy: reject
```

Group order, prefixes and `{7, 0, 20}` layouts come from the assembly, not a copy
of UMI's camera or aperture configuration. XPolicyLab pose responses provide
`<prefix>_ee_pose[7]` and `<prefix>_ee_joint_state[20]` at each policy step.
Requests additionally contain the measured `<prefix>_arm_joint_state[7]`.
Joint responses instead provide the arm/hand joint-state key pair with
`absolute_joint_position` semantics. The configured horizon and action interval
define the chunk's policy time grid; no UMI-specific timing is inferred.

Inline and process pose decoding are supported, including `parallel_ik: true`
with process decoding. A failed arm solve rejects the entire chunk; no other
arm's partial solution is accepted. Joint-tool step IK, per-step feedback deltas,
pose sampling/conditioning beyond `default`, candidate scoring, independent-group
fallback and waypoint handoff remain unsupported and are rejected explicitly.
This does not restrict the already supported joint adapter's capabilities or
declare new sampler support in any backend.

Adapters reject malformed groups, widths, horizons, nonfinite values and semantic
mismatches. Pose decoding also checks quaternions and fixed-coordinate integrity.
Position and rate constraints remain in the native arm solver, configured
executor, shared `SafetyGuard` and component command validation. Executors may
limit a raw target when constructing a feasible command; an out-of-bounds final
command is rejected before dispatch. Decoder success alone is not a position
limit certification for the hand. Both raw targets and final commands keep the
declared radian meaning.

Disconnected checks exercise the actual XPolicyLab client encoder/decoder against
an in-memory response source, both adapters, the timeline, Direct/Smooth, and
real component dispatch code with fake SDK endpoints. Separate tests use native
offline Tianji FK/IK with nonzero mounts, including spawned decoders and atomic
IK failures. These checks do not establish network-service, model or hardware
qualification; full simulated-service runtime acceptance is a separate stage.

## Control and connection ownership

All configured arms borrow one `TianjiController`; one batch contains targets for
all selected arm channels. Each hand has a distinct serial/owned SDK alias, with
the Wuji manager shared as described by the component. Hardware switches default
off. Hand execute derives from both assembly `execute` and `end_effector_control`;
station components cannot override execute, model side/version or shared ratios.
Wrong group dimensions, side mismatches, duplicate serials and unknown bindings
fail before opening a connection. Predictable binding checks inside individual
drivers may still report an error during partial assembly connection; cleanup
owns those acquired endpoints.

`rate_contract.joint_tool` adds a typed hand section to the existing Tianji
controller-rate expansion. Arm speed still derives from rated joint velocity,
controller percentage and `command_margin`. Hand velocity/acceleration are
explicit development envelopes (0.5 rad/s and 2 rad/s² in the shipped examples),
not measured firmware capabilities. Review them against the actual installation
before physical qualification. The loader derives `[7, 27]` slices and propagates
separate arm/hand rates to Direct, Smooth and command safety. No scalar opening
curve, close latch or automatic open command applies to the hand. Software
position ceilings follow the selected arm/hand model; device bounds can narrow
them further. The assembly owns a separate hand interpolation only during Home.

## Mounts and local resources

The shipped hand mounts are `null`: no Tianji-Wuji flange calibration is assumed.
Bind `robot.mounts.<component>` in the private station; the CLI maps this to
`robot.options.component_mounts`. `RobotModel.from_config(..., component_mounts=...)`
uses it consistently in FK/IK and visual assembly. Each mount contains `xyz`
in metres and URDF `rpy` radians, representing **flange to hand mount**, not arm
scene placement. Unknown components, arm-mount overrides and missing/invalid
tool transforms are rejected. Use the station template matching the selected
single/dual assembly so it contains only selected component names.

Offline adapters, including the separate action-decoder process, use
`RobotModel.from_robot_config(robot)` to retain the runtime's bound mounts.
For a custom RoboGUI YAML selecting this assembly, pass the same station with
`manimux-robogui --config /path/to/robogui.yaml --local /path/to/station.yaml`.
This reads only physical mount bindings into the offline view; GUI network
settings remain separate. Without `--local`, no station is read implicitly.
Programmatic callers may pass `robot_config=resolved_config["robot"]` to
`load_robot_view()`; its assembly path must match the display YAML.

Runtime records the constructed model's parameter snapshot in
`meta.json.robot_geometry`. Finalized-episode replay restores mounts, component
options, group order/layout and arm Home targets from it even if the original
YAML files change or disappear. Installed component code, private offline Tianji
SDK and model assets are still required. Current display styling may be supplied,
but cannot replace recorded geometry. Legacy episodes need an explicit model and,
for this assembly, its station mounts. Current AAC scoring assumes scalar grippers;
AAC and BID-with-AAC reject Wuji joint-tool layouts before runtime startup.

The hand TCP defaults to the explicit Hand 2.2 CAD wrist/palm reference. A private
hand component can provide `options.tcp_transform`; only model-relevant options
reach the offline model, while hardware construction retains side/version.
SDK origin/zero, hand model revision and actual flange/TCP calibration still
require installation verification. CAD fingertip masses are not validated
controller payload parameters. Arm Home explicitly uses the ManiMux pack-plate
preset; no UMI load, camera or drag setting is inherited. Select real cameras through the existing external sensor sources;
removing TacCap does not supply a replacement wrist image stream.

Marvin's local installation remains the existing ignored directory
`manimux/embodiments/arm/tianji/sdk/marvin/`. Install the unmodified vendor
`SDK_PYTHON/fx_robot.py`, `fx_kine.py`, Linux libraries `libMarvinSDK.so`,
`libKine.so`, `libInterfCheck.so`, the vendor license and
`CommonConfig/ccs_m6_40.MvKDCfg` there. Do not change `sys.path` to import another
checkout. Only the kinematics library is needed for offline native FK/IK;
constructing a model does not create a `Marvin_Robot` control object.
Tianji CAD URDF/mesh resources under the existing ignored arm `assets/` directories
are additionally needed for its full RoboGUI scene. Neither a source checkout
nor a wheel installs these private resources. Missing resources are not replaced
by another robot or an approximate solver.

The optional `wuji` extra supplies the pinned hand SDK. Its connection also needs
the exact serial, side, explicit provisioned hardware version and, when executing,
MIT gains/current limits from the private station. See the component contract.

## Source description and offline geometry audit

Prepare the selected `marvin_waist_mobile.urdf` with an explicit mesh package root:

```bash
python scripts/assets/prepare_tianji_wuji.py \
  --urdf /path/to/robot/urdf/marvin_waist_mobile.urdf \
  --package-root /path/to/robot
python scripts/validation/tianji_wuji_geometry.py \
  --output /tmp/tianji-wuji-geometry.json
```

The importer writes ignored `manimux/embodiments/arm/tianji/assets/marvin/`.
Use `--output` to prepare a separate copy; existing output is never overwritten.
It preserves each arm's seven revolute joints and fixed `JointTCP_L/R`, renames
the final link to `Flange_L` / `Flange_R`, and copies nine unchanged meshes per
side, including the base and flange. The base is `Base_L/R`. Torso, lift, waist
and the source's original grippers are omitted from these component descriptions.
The source has no Wuji hand or mounting transform: the manifest records that
absence, and no old D3 transform is reused. The importer never writes station
mounts or SDK settings. After the audit passes, install the prepared side folders
at the existing `assets/<side>/` runtime paths; preserve existing resources before
any replacement. This installation filled previously missing visual resources.

The audit verifies hashes, extracted/source arm FK, 27-coordinate `MountedGroup`
composition and mesh-backed visual TCP transforms. It checks right-only and dual
layout widths. Existing Hand 2.2 component resources supply the hands. A declared
nonzero synthetic mount tests composition exactly once; it is never written to
station configuration or represented as a measurement. Default sampling uses
32 deterministic synthetic joint configurations per side, within source limits.
These are geometry samples, not an executable trajectory or device commands.

For native comparison, the probe constructs only `Marvin_Kine` through the
existing `TianjiSDKKinematics`, never a `Marvin_Robot` or Wuji manager. Supply
`--sdk-config /path/to/model.MvKDCfg` to inspect another vendor model. Without
the private kinematics library, `--description-only` performs the source checks
and records SDK compatibility as unknown. No source-only FK solver is registered
as a production robot, and the probe implements no substitute IK.

**Current local result (2026-10-23): source assembly and SDK geometry pass.**
`Joint1..7_L/R` map directly to SDK J1..J7, with the same signs/zeros. Frames are
source `Base_L/R` to `TCP_Link_L/R` and SDK arm base to bare flange. No alignment
or parameter fitting is applied. Across 32 configurations per side, the maximum
position error is below 4.92 micrometres and rotation error below `2.63e-5` rad
(about 0.00151 degrees), within 0.1 mm / 0.001 rad acceptance tolerances. Source
Euler angles such as `1.5708` retain their original rounding. Native FK is also
checked against the DH parameters actually loaded by that SDK instance.

| Adjacent rotation axes | Source distance | SDK distance |
| --- | --- | --- |
| J3 / J4 | 18 mm | 18 mm |
| J4 / J5 | 17.9977 mm | 18 mm |
| J6 / J7 | 0 mm | 0 mm |

Raw J1/J3/J5 model limits still differ: the source uses approximately +/-178
degrees and the SDK +/-170. Other arm ranges agree within source rounding;
source speed is 3.1416 rad/s versus SDK pi rad/s. Existing tighter command/IK
bounds remain unchanged. Geometry agreement does not widen those limits or
qualify hardware, calibration, collision-free motion, payloads or Home.

The earlier `benchmark/robot/urdf/d3_wujihand2.urdf` had 0/0/70 mm for the three
axis pairs above and was rejected. It is a superseded input, not the selected
model; its disagreement is not the current SDK qualification result.

The JSON report separates `description_assembly_passed`,
`sdk_geometry_compatible` and `qualified_for_hardware`. SDK disagreement writes
the report and exits **2**, rather than reporting success because meshes loaded.
The SDK solver/configuration and `assets/<side>/arm.urdf` selection mechanism
remain unchanged. Real-factory checks use native SDK FK with these source visuals
and synthetic tool mounts, without connecting either device.

## Stop, close and unsupported operations

**TianjiController.stop() retains its existing behavior: disable owned arms
(mode 0), confirm advancing feedback, and retain ownership when confirmation
fails.** No hold mode or new arm stop implementation is introduced. Generic
assembly stop attempts the shared controller and every opened hand, even after
an earlier component fails. Wuji stop requests one fresh measured-pose hold;
it does not disable hand actuation or establish that a payload remains held.

`RobotBase.close()` retains existing controller checks and stop ordering, then
calls the default-no-op `_prepare_end_effector_close()` hook. Tianji-Wuji uses
that hook only when a hand owns/possibly owns actuation. It blocks in the existing
RoboGUI support-confirmation flow before releasing any hand or connection.
The controller's existing stop has already run at this point; this gate is not
arm gravity compensation. The operator must secure ALL arms and payloads.
Read-only or never-activated hands need no release permission and receive no
disable command. No terminal prompt, lost UI or interrupt grants support permission.

After explicit support confirmation, release attempts every activated hand.
Incomplete release retains the assembly and all connection resources inside
RoboGUI recovery; another support confirmation is required for each retry and
another Prepare stays blocked. Once releases succeed, normal close shuts each
hand's streams/owned alias and the shared arm connection. No destructor adds
implicit hand release. This software ownership behavior does not protect against
power loss, failed drives, SIGKILL, or an unvalidated firmware watchdog.

## Home and shutdown

Executing `connect()` requires explicit `home.joints_deg` for every selected arm,
before either device SDK is opened. Shipped assemblies explicitly copy the arm
targets from ManiMux `tianji_taccap_pack_plate.yaml` (`pack_plate_start`):

```yaml
left_arm: [89.549, -32.95, -52.6334, -127.1007, -54.3386, 4.6073, 54.6561]
right_arm: [-88.6985, -13.1926, 37.6683, -117.9514, 50.1699, -34.0801, -32.0042]
```

Right-only assemblies contain only the right target. The optional
`examples/tianji_wuji/home_targets.template.yaml` fragment contains the same
angles for a private assembly copy, selected with experiment `robot.config`.
Rebase component `config` paths when moving the assembly: from
`configs/local/`, the arm reference is `../embodiment/arm/tianji_right.yaml` and
the hand reference is `../embodiment/end_effector/wuji_hand2_right.yaml`.
Do not infer Home from URDF display poses or rollout-start joints. Read-only inspection and
`home()` remain free of motion and do not require targets.

`hardware.home_motion` contains `control_hz: 100`, `peak_velocity_deg_s: 9`,
`tolerance_deg: 0.5` and `settle_timeout_s: 5`, matching the existing Tianji
arm trajectory profile. Station/experiment `hardware.home_motion` can override
individual fields. All values must be finite and positive. Targets must pass
the configured controller bounds; Home peak velocity cannot exceed the rated
joint velocity scaled by the controller ratio. These software defaults are not
tool/payload or collision qualification.

`TianjiHomeSettings`, duration and cosine interpolation are shared through
`embodiments/robot/tianji_home.py`. TacCap retains its prior arm interpolation and
post-arrival scalar-gripper opening. Wuji Home captures the current measured
20-coordinate hand targets once and retains them throughout arm motion. Only
after every arm has measured arrival does it reread the hands and interpolate
to `hardware.hand_home.joints_rad` (twenty explicit zeros). The hand motion follows
the cosine-to-zero and hold behavior in `wuji-control/move2start.py`, integrated
through the existing assembly/driver rather than a second SDK connection.
`hardware.hand_home` additionally declares `control_hz: 200`, `ramp_s: 1.5`,
`max_velocity_rad_s: 0.5`, `max_acceleration_rad_s2: 2`, `hold_s: 2`,
`tolerance_rad: 0.02` and `settle_timeout_s: 5`. `ramp_s` is the minimum duration.
For the largest absolute displacement `D` across all selected hands/coordinates,
the synchronized cosine uses
`T = max(ramp_s, pi*D/(2*v_max), pi*sqrt(D/(2*a_max)))`.
This bounds its sampled reference speed and acceleration per coordinate, including
entry/exit at zero reference velocity. A 2.094 rad move with the defaults takes at
least 6.58 s rather than 1.5 s. Zero displacement retains the minimum ramp and the
measured arrival/hold checks. The trajectory deadline uses `2*T + settle_timeout_s`;
arrival and hold keep their independent deadlines.

The component exposes position bounds but no reusable motion-rate capability.
These are therefore explicit assembly-owned Home envelopes, initially matching
the shipped executor's hand development limits. Home does not invoke an executor
or automatically inherit later `rate_contract.joint_tool` edits. Configure Home
limits separately through station/experiment `hardware.hand_home`; keep them
consistent with the installation. All settings must be finite positive numbers
(not booleans); all twenty targets must fit every hand's configured command bounds,
checked before connecting. An unrepresentable duration fails before hand motion.
Private copies of older assemblies must supply both new rate fields in their
`hardware.hand_home` or its explicit overrides; there is no implicit unlimited mode.

The existing fixed-step submission loop is retained: slow feedback or SDK calls
stretch the schedule without skipping reference samples or sending catch-up bursts.
Velocity and acceleration bounds describe the command reference sampled at
`control_hz`, not measured motor motion or a guarantee about device-side timing.
In particular, irregular transport intervals do not guarantee bounded physical
acceleration. SDK interpolation, physical tracking, payloads and paths still need
hardware qualification. Home still submits absolute positions through the same
driver with zero velocity/effort fields; it adds no SDK mode or connection.

Hand publication follows `end_effector_control`; disabled tool control emits no
hand commands and performs no hand-zero stage. `home_targets(state)` returns
copied final arm/hand targets without I/O; uncontrolled hands retain input
positions. Optional `home(stop=threading.Event())` observes cancellation
before motion and while waiting; cancellation and interrupts remain failures.

Home reads all arm/hand feedback before motion. The controller's read-only
`validate_motion_ready()` checks every configured arm's faults, position bounds,
owned mode and enabled speed ratios, without clearing faults or enabling drives.
The existing command path performs normal position-mode preparation on first
submission. Readiness and feedback checks stay active during measured arrival.
After submitting the final target, Home captures actual arm feedback sequences
and requires later advancing sequences plus the configured position tolerance
for every arm. `RobotState.sequence` counts reads and is not used as proof of
new feedback. This is the SDK's receive-side sample evidence, not a calibrated
device acquisition timestamp. Trajectory and arrival deadlines report failures;
arrival errors include per-group degree errors and feedback progress. Hand arrival
also requires feedback sequences newer than the post-final-command baseline and
twenty measured positions within `tolerance_rad`, followed by the full `hold_s`
interval with healthy feedback and measured positions still in tolerance. Merely
publishing zeros or waiting a fixed duration does not complete Home. Every arm
must remain at its Home throughout hand motion and hold. Hand failures retain
actuation/resources for the same explicit support/release flow as arm failures.

The common runtime requests Home before close for normal Finish, limits and
software faults. Active Return Home is available from the explicit target
contract independently of idle recovery actions. A failed Home is reported and
is not automatically retried. Stop then retains its existing arm-disable
semantics; hand release still requires explicit support confirmation, with the
Home failure included in the gate reason. Missing UI, interruption or a failed
retry does not authorize hand release.

`RobotBase.shutdown_home_available` is a pure query, defaulting to existing
retry behavior for other embodiments. Wuji withdraws it when cleanup invalidates
whole-robot motion. `ShutdownStatus` rechecks that availability both for the UI
and before a requested retry. Unavailable/failed Home requests rotate the recovery
token; support must be explicitly confirmed against the current request. The
reporter never polls or commands hardware. After arm stop and entry into the
Wuji hand release gate, only support/release remains available.

Idle Home, drag, clear-error and release recovery actions remain unadvertised;
the idle service does not yet supply this hand's support-confirmation channel.
Use active-rollout Home and normal shutdown through the owning runtime. Software
checks do not qualify the Home path, grasp retention or physical execution.

## RoboGUI and disconnected runtime

The `tianji_wuji_right` and `tianji_wuji` display presets use the same assembled
model as the runtime. They render all 27/54 coordinates, including all twenty
hand joints per group, without an aperture strip or implicit UMI camera slots.
Scene placement is display-only. Supply `--local` explicitly: missing physical
mounts remain an error. `synthetic_mounts*.yaml` supplies clearly marked test
transforms for these examples, not installation calibration.

With ManiMux's GUI/recording dependencies and the offline Tianji model resources
installed, run from the repository root in the selected Python environment:

```bash
# One headless synthetic rollout, using Direct by default.
python -m manimux run \
  --config manimux/configs/examples/tianji_wuji/disconnected_right.yaml --once
# Both arms, using the existing Smooth executor.
python -m manimux run \
  --config manimux/configs/examples/tianji_wuji/disconnected.yaml --executor smooth --once
```

These examples explicitly select `manimux.examples.tianji_wuji:build_robot` and
its in-memory controller/hand endpoints. No motor SDK is constructed or imported.
`execute: true` enables these memory command paths only; the factory rejects
physical hardware bindings and unknown options. The selected synthetic local YAML
prevents accidental loading of the default station. The control profiles inherit
the production motion limits through `_base_`, overriding only the robot factory.
The synthetic worker produces small absolute-joint waves through the ordinary
`JointAdapter`, worker process, runtime, executor and recorder. Native offline
geometry, assembly Home and close algorithms are reused. Feedback follows targets
ideally on read, with synthetic sample times; there is no dynamics/contact model.
`meta.json.robot_backend` records `source: synthetic_memory_endpoints`,
`hardware_connected: false` and the tracking/confirmation mode.

For the manual GUI flow, use two terminals:

```bash
# Terminal 1: dashboard.
python -m manimux.robogui.dashboard --robot tianji_wuji_right \
  --local manimux/configs/examples/tianji_wuji/synthetic_mounts_right.yaml \
  --host 127.0.0.1
# Terminal 2: managed session, no automatic step-limit exit.
python -m manimux run \
  --config manimux/configs/examples/tianji_wuji/disconnected_gui_right.yaml
```

Open the dashboard at `http://127.0.0.1:8086`. Prepare Run, Start, active Home and
Finish & Home use the existing controls. After Finish, the support checkbox and
explicit release confirmation authorize simulated release; until then the runtime
retains its simulated resources and another Prepare is blocked. Home availability
comes from the active runtime, independently of idle recovery capabilities. Idle
Home, drag and clear-error controls remain unavailable. Remove `_right` from the
preset and both filenames to use both arms. For display alone, append `--demo`
to the dashboard command and omit the runtime.

The headless example disables only the memory endpoint's manual release gate.
The GUI example enables it to exercise the real support-confirmation flow.
The hardware assembly requires `robogui.enabled: true` whenever both execution
and hand control are enabled; the runtime rejects a missing configured channel
before connecting. A configured but unavailable GUI still cannot authorize release.

Replay a finalized `rollout-...` directory reported by the runtime:

```bash
python -m manimux.robogui.replay --episode /path/to/rollout-001 --source state
python -m manimux.robogui.replay --episode /path/to/rollout-001 --source command
python -m manimux.robogui.replay --episode /path/to/rollout-001 \
  --source prediction --plan-index 0
```

State, scheduled reference and command replay use recorded tick times. Prediction
replay selects one accepted plan's decoded `canonical_raw` joint chunk on its
policy time grid; it is not a raw network response, achieved motion or a concatenation
of overlapping predictions. Pose-space chunks are explicitly unsupported by this
player. All coordinates and recorded mounts are restored from the geometry snapshot;
offline component code/assets remain required. Playback starts paused and does not
construct a robot driver, camera or policy service.

## Offline checks

Config inspection requires no devices, model service, private SDK or mount:

```python
from manimux.cli import load_config

cfg = load_config("manimux/configs/examples/tianji_wuji/offline_right.yaml")
assert cfg["robot"]["group_dims"] == {"right_arm": 27}
assert cfg["robot"]["options"]["execute"] is False
```

With installed offline SDK resources and measured station bindings, construct
`build_robot(cfg["robot"], clock)` without calling `connect()`. Missing mounts
must produce a configuration error rather than an identity-transform assumption.
The examples use a synthetic policy only; they do not qualify GR00T inference.

Local ignored tests exercise the real factory/loader and real Python controller
and hand drivers against fake vendor SDKs, with synthetic arm geometry and real
hand assets. Coverage includes 27/54 dimensions, Direct/Smooth, visual/TCP
composition, command switches, partial startup/dispatch, arm disable semantics,
support gates and cleanup retry. Home tests provide explicitly synthetic target
angles in temporary assembly copies; they are not station presets. Native Marvin FK/IK can also be checked offline
with explicitly synthetic mounts. Such transforms and tests are not calibration,
connected-device evidence or physical task success.

The geometry probe separately exercises the source arm meshes and bundled hands.
Local factory tests use native SDK FK/IK and a temporary source visual path in
the real single/dual loader, with execution disabled and motor SDK imports
prohibited. The FK-only description probe is never selected as a runtime solver.
Fake-SDK Home fault injection covers moving/stale/cached feedback, peer faults,
mode/rate changes, cancellation, deadlines, release failure and retry fencing.
Reference-rate checks inspect complete published hand sequences for zero/small
and joint-boundary moves, nonzero targets, asymmetric dual-hand displacement,
acceleration-dominated duration and delayed waits/SDK calls. They validate the
nominal sampled speed/acceleration and configured position bounds, including
the stationary entry/exit, without claiming physical rate measurements.
Actual `EdgeRuntime` runs cover single/dual Finish, limits, camera/model loss,
interrupt and Home failure using the real assembly with disconnected SDK doubles.

## Network and runtime validation

From a source checkout with the `xpolicylab` extra and native offline arm resources,
use a new output directory for each run:

```bash
python scripts/validation/tianji_wuji_runtime.py \
  --matrix --output /tmp/tianji-wuji-validation
```

Without `--matrix`, this runs right-arm joint/Direct/async. The matrix covers
right/dual × joint/absolute-pose × Direct/Smooth × serial/async, with 100 formal
control ticks per case. The peer binds only an ephemeral loopback port and
implements synthetic HELLO/RESET/INFER responses. The normal XPolicyLab client,
worker process, adapter, native offline FK/IK, scheduler and runtime are used.
The robot factory is always the explicit memory example; the parent process also
blocks motor SDK imports. No model weights, real observations or device connections
participate. This is a wire-contract check, not qualification of a model server.

Each case saves `experiment.yaml`, `report.json`, the normal runtime episode and:

- `wire/*.msgpack`: untouched requests/replies, written before sending responses.
  `wire/index.jsonl` links message IDs, trial IDs, wire steps and local timestamps.
- `memory_commands.npz`: values actually written to the memory arm/hand endpoints,
  including Home calls that use the robot command path. Recorded formal tick times
  select matching samples; all 27/54 command coordinates are compared exactly.
  These samples are not firmware acknowledgments or physical feedback.

The validator also checks full state/reference/command widths, finite values,
strictly advancing recorded tick times, all twenty hand coordinates changing and
the complete accepted joint predictions. It requires multiple accepted plans and
zero rejected plans in successful cases, then verifies endpoint cleanup.

Local ignored regression tests additionally exercise streaming requests, pose
decoding in a subprocess, unrelated/reordered replies, reset after a late reply,
backend-identity mismatch, bad response shape, server error, timeout and disconnect.
An initial RESET failure closes the provisional WebSocket before retry; a regression
checks the peer has no remaining connection before initializing a fresh session.
The ordinary response-error contract rejects that response and retains the previous
accepted plan/terminal hold; it does not necessarily abort the rollout. The bounded
fault tests finish and Home at their step limit and must not be called successful
inference. Backend-identity and runtime endpoint/recording exceptions instead fail
the run and preserve an incomplete result with the original failure details.
Partial tool dispatch records submitted and attempting channels separately.

GUI process tests use real loopback ZeroMQ and Viser, a separate managed runtime
process, the synthetic WebSocket peer, and programmatic GUI callbacks. They cover
Prepare, Start, Pause, active Home acknowledgment, Finish and explicit simulated
support/release, plus stale-control pausing. They do not test a human browser session
or physical support. Existing fake-SDK tests retain the device-specific feedback,
Home-failure, release-failure and retry checks.

Wheel verification must run outside the source checkout. The wheel includes the
public Wuji URDFs/meshes/license/provenance, factories, display presets and example
YAMLs. Private Tianji SDK/visual resources and station configuration remain excluded:
config-only examples load without them; model construction requires them to be
provisioned separately at the component's documented local paths. Verify the
installed disconnected CLI after supplying those resources, without importing motor
SDKs or using a physical station. A built wheel is not a deployment authorization.

## Home target provenance

The hand Home target matches the standalone Wuji `move2start.py` reference: all
20 joint positions are zero radians, with a 200 Hz cosine transition, a nominal
1.5-second ramp and a 2-second hold. Home may extend the ramp to honor its
configured velocity/acceleration limits. SDK discovery, gains, current limits and
disable behavior remain owned by the configured component/station, not copied
from the standalone script. Missing feedback is rejected, never filled with zeros.

The Tianji arm driver and existing TacCap assembly follow upstream main unchanged.
Arm/tool rate expansion belongs to configuration; Wuji Home uses the main
controller feedback and command validation. Joint-tool pose examples explicitly
select `execution.ik.failure_policy: reject`; other supported layouts retain
the selected shared IK failure policy.

Reference inspected 2026-11-17: `move2start.py` SHA-256
`bd90e65b48700d0480b654bc9237201f0e24525dca43e4c4a2b12c1f670ebc2f`.
