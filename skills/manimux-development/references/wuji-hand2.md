# Wuji Hand 2 component

This component implements `JointToolBase`, independently of a carrying arm.
`manimux/configs/embodiment/end_effector/wuji_hand2_{left,right}.yaml` selects the
explicit **Hand 2.2** visual model and declares 20 radian coordinates. The model
revision is not inferred from the reported hardware version. An operator must
check that the chosen description matches the installed hand before connection.
The [Tianji-Wuji assembly](tianji-wuji.md) supplies right/dual composition and its
explicit hand-release gate. Mounting/payload calibration, Home/recovery and
learned-policy recipes still require qualification; component files are not rollout presets.

## SDK and identity

The optional `wuji` dependency pins `wuji-sdk==2026.9.22`. Only `connect()` imports
the SDK or obtains its singleton manager. Construction and `load_model()` need
no SDK, discovery or physical device. Production code does not import the
neighboring `wuji-control` checkout.

Bind an exact `serial`, `expected_hw_version: [major, minor, patch]` and side in
the private station. An unprovisioned `0.0.0` version is rejected. Connect uses
the exact serial, a unique owned alias and `ConnectOptions(enable_bridge=False)`;
it never calls `scan()` or selects by discovery order. Type, serial, handedness
and reported hardware version must all match. An already connected serial in
the shared manager is rejected rather than borrowed. Each hand releases only
its own alias; `disconnect_all()` is never used. Multiple independent processes
or other hosts are outside this in-process ownership guarantee.

The SDK connection performs its documented handshake/time synchronization and
subscriptions. The driver calls no actuator setters during connection or
read-only stop/close. Native SDK/firmware behavior still needs physical review;
fake tests establish the Python calls, not electrical behavior.

## Coordinates, feedback and clocks

The stable coordinate order is finger-major, thumb/index/middle/ring/pinky,
four radians each (`thumb_joint1` through `pinky_joint4`). Decode each entry using
`WujiHand2.nid_to_joint_index()`. Firmware NIDs are 1–4, 6–9, 11–14, 16–19 and
21–24; stream list order is not coordinate order. Both sides map directly to the
official anatomical URDF joints without sign changes or normalized clipping.
The pinned model preserves the upstream left/right axes and limits.
SDK user-origin/calibration must agree with the description before motion;
the driver does not reset origins or infer that agreement from a serial match.

Every state/diagnostics frame must contain exactly 20 unique known joints.
Positions, velocities and effort must all be finite. Diagnostics must report
known states; nonzero joint fault codes block state/commands without clearing
faults. Measured positions are not clipped to command bounds or replaced by
targets. SDK effort is current-equivalent amperes (Kt is a placeholder), not
calibrated joint torque or tactile force. `JointToolState` exports positions only.

Each nonblocking read drains the FIFO to its newest frame, with a configurable
`max_frames_per_read` bound. Invalid frames, read errors and excessive backlog
invalidate the cache; a later cached read cannot hide them. Duplicate frames
retain sequence and host timestamp. Changed duplicates, sequence regression,
zero/regressing device timestamps and sequence wrap require a new valid stream
or reconnection; a device restart must not silently reset identity.

`JointToolState.timestamp` is seconds in the supplied station Clock's monotonic
domain, measured when a new frame is **dequeued**, not firmware acquisition or
SDK transport receipt. Device `header.timestamp_us` remains separate. The
minimum observed host/device offset supplies a lower bound on increasing delay;
both that bound and host dequeue age must fit `stale_timeout_s`. This detects
growing backlog after an anchor, but does not prove initial/constant one-way
latency or cross-device synchronization. FIFO draining and mock timing checks
are not a hardware latency measurement. Automatic device time synchronization
may cause timestamp regressions; these are rejected instead of relabelled fresh.

## Commands and lifecycle

`execute` defaults to false. Command validation checks width, finite values and
explicit lower/upper `joint_limits` before any configuration or actuation.
Component YAML contains the CAD limits as ceilings; station limits must stay
inside them. They are not validated collision or task envelopes. Gains/current
have no automatic historical defaults: execution requires explicit positive
`kp`, nonnegative `kd`, and positive `effort_limit_a`, each scalar or length 20.

The first command verifies fresh feedback, no faults and all joints Ready or
Stopped, then sets limits/gains, creates its publisher and enables the hand.
It waits for a strictly advancing complete diagnostics frame with all joints
Enabled before publishing 20 `JointCommand(q_i, 0.0, 0.0)` values. Firmware must
already use MIT mode, the SDK's documented Hand 2 default. Switching modes,
calibration, fault reset and user-origin changes are unsupported. Activation
energizes motors; its initial target and transient behavior need hardware tests.
Submission means SDK publication, not arrival, force closure or grasp success.

| Operation | Behavior |
| --- | --- |
| `connect()` | Feedback-only preparation, including when execute is true. |
| `send_command()` | Explicit gains/current and enable on first authorized command; absolute position publication thereafter. |
| `stop()` | One fresh measured-position command with zero velocity/effort, retaining MIT actuation. No enable/disable, Home or opening. |
| `release()` | Explicit disable of owned/possibly partially enabled actuation; wait for advancing complete non-Enabled diagnostics. May release a grasp. |
| `close()` | Close publisher/subscriptions, then only the owned alias. Refuse while actuation is owned or uncertain. |

Stop requests a position hold; it does not wait for measured settling or prove
that a payload remains held. Stale/faulty feedback prevents a new hold command.
No background command loop is added, and the firmware's command watchdog/hold
duration has not been established. Stop is not an emergency stop. A healthy
subsequent command can resume; a failed activation/publication latches command
failure and requires explicit release and close/reconnect. A completed release
also ends the activation session; close/reconnect before commanding again.

Enable may partly succeed before throwing. Such failures retain all control
resources and refuse automatic disable/disconnect. Release must be called only
after external support secures the payload; it remains callable when feedback
has failed, but success still requires new non-Enabled diagnostics. Disable,
stream-close or disconnect failure remains visible and retryable. No destructor
adds an implicit actuator release. Assembly-level support confirmation and exit
handling must be implemented before exposing this driver in a live rollout.

## Offline resources and validation scope

The official description is pinned to `wuji-technology/wuji-description`
`v2026.10.12`, commit `829c9aa27cb12c3a49d5b8d63c9869dd8f656702`, under MIT.
See the component assets README, source hashes and retained license. Meshes,
joint origins, axes, limits and inertias come from the standalone hand package,
not the unrelated combined D3 benchmark URDF.

The fixed palm TCP is the upstream CAD wrist frame relative to the mount.
`tcp_transform` can explicitly override it; visual and computational TCP use
the same transform. This is not a fingertip/contact model, measured Tianji
mount or payload calibration. All 20 inputs remain available for display.
Upstream inertias, including provisional fingertip-pad masses, are not approved
controller load parameters.

Local ignored tests use fake managers, streams, clocks and publishers for both
sides, validation, cache identity, timing, activation failure and cleanup retry.
The installed native SDK's static NID conversions and command value objects
can be checked without obtaining a manager. Software checks do not establish
hardware safety, real latency, motor stopping, watchdog behavior or grasp success.
