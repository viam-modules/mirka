# Autochanger remover service: design

Status: design approved in conversation 2026-09-24, not yet implemented.
Companion to the sanding module's maintenance design
(`docs/pad_change_orchestration_design.md` in `viamrobotics/sanding`, branch
`pad-change-design`), which owns the trigger, the button, and the return trip.
This document owns everything from "remove" being called to the arm sitting in
free space with the disc off.

## 1. Goal

A generic service, `viam:mirka:autochanger-remover-svc`, that runs the Mirka
AutoChanger removal cycle end to end when asked: stops the spindle, plans the
arm to the remover avoiding the piece, alternates arm waypoints and knife moves
per the Mirka manual, and reports exactly where it stopped if anything fails. It
drives the merged `viam:mirka:autochanger-remover` gantry driver and does not
modify it.

Non-goals: the adder, sensing, the air nozzle, the bumper model, any knowledge
of the sanding pass. Each is under Deferred.

## 2. The removal cycle, from the Mirka AutoChanger manual

Arm and knife strictly alternate. Every knife move happens with the arm parked.

| Step | Arm | Knife |
|---|---|---|
| centring (recommended, deferred) | pad pushed against bumper, right then left | flush (0) |
| fold-down (recommended, deferred) | disc edge pulling stroke against bumper grooves | flush |
| wp1 | pad parallel to sliding plate, just below knife | grip |
| wp2 | press into plate until the spring-loaded front plate gives about 10 mm | grip |
| wp3 | straight up until knife tip is 10 to 20 mm under the pad | grip (knife wedges between disc and pad) |
| clamp | stationary | grip to flush |
| wp4 | tilt back about 20 degrees | flush |
| wp5 | back and up at an angle | flush |
| release | stationary | flush to release (25), nozzle 0.5 s (not installed), back to grip |

Knife offsets, as the driver's continuous 0 to 25 mm joint: `flush` 0, `grip`
about 0.7 mm and disc dependent, `release` 25. Grip is a process parameter the
caller holds; the driver has no notion of it. The front plate is spring loaded,
so the press at wp2 is position controlled against a spring and needs no force
feedback.

## 3. What the service needs from the cell

### 3.1 Obstacles

The sanding module's snapshot mesh vision service caches the pass mesh under an
empty camera name at snapshot time and returns it on any call without a command
key, with no recompute. The sanding executor reads through the same wrapper, so
this service gets byte-identical geometry to what the pass planned against, in
world coordinates. Mesh and octree geometries serialise to proto in RDK 1.9.0.

### 3.2 Joint limits

The sanding planner applies `input_range_override` joint limits in-process,
only-tighten, before every plan, and they are load-bearing (cable wrap). The
motion service API cannot take joint limits per request. This service therefore
holds its own copy and plans with `armplanning` in-process. A missing copy is a
refusal, never "use model limits."

### 3.3 Spindle

The service stops the Mirka AIROS spindle in preflight and leaves it stopped.
Restarting it is the caller's job (the sanding executor re-asserts `start` after
any maintenance). The dependency is the `viam:mirka:airos-550cv` component, not
the sanding module's `sander-model`.

## 4. Service

### 4.1 Config

```json
{
  "remover": "remover",
  "arm": "arm",
  "mirka": "airos",
  "tool_frame": "compliance",
  "obstacle_visions": ["snapshot_mesh_vision_service"],
  "input_range_override": { "arm": { "2": { "min": -4.0, "max": 0.0 } } },
  "grip_offset_mm": 0.7
}
```

Every field is something the service cannot derive: which resources it drives,
which frame is the pad, where obstacles come from, the limits this cell decided
on, and the one disc-dependent number the manual hands to the integrator. Pad
radius is read from the geometry parented to `tool_frame`; missing geometry is a
refusal, since the planner would also have no pad to check. Deps: remover, arm,
mirka, each obstacle vision, frame-system service.

### 4.2 Sequence in code

The manual's sequence is a fixed slice of typed steps, arm or knife, so the
alternation invariant is a compile-time property and the test is a table of the
expected order. Waypoints derive from the remover origin (plate front face, top,
centred in Y; +X knife extension, +Z up), the pad radius, and three constants
from the manual: press depth about 10 mm, lift until the knife tip is 10 to
20 mm under the pad, tilt back 20 degrees. A constant moves to config only if rig
tuning shows it is per-disc.

Knife steps are `MoveToPosition([offset])` on the gantry: `grip` =
`grip_offset_mm`, `flush` = 0, `release` = 25. Belt and braces around the
structural invariant: before a knife step assert the arm is not moving; before an
arm step assert the remover is not moving.

Contact steps (wp2, wp3, wp4, wp5) are straight-line constrained moves from the
previous waypoint with collisions between the tool frame's geometries and the
remover's three geometries (body, blade, head) allowed via
`motionplan.CollisionSpecification`. Without that, the planner correctly refuses
wp3 because the knife is inside the pad. wp5 starts from wp4, where the pad is
still over the knife, so it is a contact step too; a free plan refuses a start
state in collision. wp1 is the only free step.

Allow entries name individual geometries, never the remover component: a
component name expands to every geometry it owns, which would defeat the
fine-grained model. The planner does not see `model.json`'s `label` fields: a
frame system built the way the frame-system service builds one names the three
geometries `<model name>:<link id>`, that is `autochanger-remover:body`,
`autochanger-remover:blade`, `autochanger-remover:head` (probed against RDK
1.3.0 and 1.9.0). Those are the allow entries. Preflight checks all three exist in the
frame system's geometries and refuses otherwise, and a test builds a frame system
from `autochanger.KinematicModel()` to pin them. A tool frame's configured
geometry lands on `<tool_frame>_origin`; naming `tool_frame` in an allow entry
covers it.

### 4.3 Planning

`armplanning` in-process against the frame-system service's frame system, with
the service's own `input_range_override` applied (copy of the sanding lib's
`ApplyJointLimits`, only-tighten, with a comment pointing there), and obstacles
from `obstacle_visions`. The service logs the limits it planned with on every
`remove` so a pass artifact shows what it believed.

### 4.4 Contract

`remove` plans from wherever the arm is and ends with the arm in free space,
clear of the changer, at wp5. It does not return the arm anywhere; the caller
owns the return. It stops the spindle in preflight and leaves it stopped.
Precondition owned by the caller: arm retreated from contact and not moving.

### 4.5 API

| Call | Behaviour |
|---|---|
| `DoCommand {"command": "remove"}` | preflight, run steps. Blocks. One at a time; a second call while running is refused. Context cancel aborts: arm stops via planner cancel, knife via `Stop`, no recovery attempted. Refused while the last result is a latched failure (section 5). |
| `DoCommand {"command": "reset"}` | clears a latched failure back to idle. Moves nothing and reads nothing; it records that a human has recovered the cell. Refused while running. |
| `Status` | idle; running with current step; or failed with last error, step, knife offset, drive ready flag, and whether the arm is at wp5 |

### 4.6 Preflight

Fetch obstacles (at least one geometry, else refuse). Apply limits (else refuse).
Read pad geometry (else refuse). Arm not moving. Send `stop` to the Mirka. Move
knife to grip: this is the readiness check, because the driver's move path
already refuses on a latched fault with the `quit_error` hint, and grip is where
the knife must be anyway. The arm is far from the changer at this point so the
knife move is safe.

### 4.7 State

In memory only: mutex for one cycle at a time, current step, last result
(including the failure latch). The cell's real state is readable from hardware
after a restart, by the driver's design. A module restart clears the latch; the
operator doc says to check the cell before calling `remove` after a restart.

## 5. Failure handling

Policy: the service never auto-recovers. On any failure it stops, leaves every
actuator where it is, and reports the step index, knife offset from the driver,
drive ready flag, and whether the arm is at wp5. Recovery is a human decision
because a disc half-clamped in the knife and a pad pressed against the plate both
get worse if you guess. The service never sends `quit_error`; the fault latched
for a reason, and clearing and retrying blind is how a jam becomes a bent blade.

A failure at any step after preflight latches: `remove` is refused until `reset`.
Without the latch, a second `remove` after an `in_contact` failure passes
preflight (the arm is not moving) and drives the knife to grip with the pad
possibly still against the plate; preflight's "arm is far from the changer"
assumption only holds if a human has recovered the cell. `not_started` does not
latch, because the arm never moved toward the changer.

| Step | Failure leaves | Disc | Reported as |
|---|---|---|---|
| preflight | nothing moved, except knife possibly moved to grip and spindle stopped | on pad | `not_started` plus reason |
| wp1 transit | arm in free space | on pad | `arm_displaced` |
| wp2 press, wp3 lift | pad against plate or knife partly under disc | on pad, edge may be lifted | `in_contact` |
| knife clamp | pad at wp3, knife between grip and flush | edge maybe pinched | `in_contact` |
| wp4 tilt, wp5 back | pad peeling away, disc clamped | partly or fully off, held by knife | `in_contact` |
| knife release, knife grip | arm clear at wp5 | in knife or dropped, unknown | `knife_holding` |

Failure classes: plan failure (nothing moved for that step); arm move error (arm
between waypoints); knife timeout (driver `timeout_ms`, offset still readable);
latched drive fault (every knife move refused until `quit_error`); abort (context
cancel of `remove`).

Operator recovery sequence, for the operator doc: send the arm home, then
`quit_error` on the driver if faulted, then `reset` on this service, then resume
the pass.

## 6. Unattended operation

Verdict: not unattended, for three independent reasons.

1. No adder. After `remove` the pad is bare hook-face.
2. No sensing. Nothing says the disc came off. Benign without an adder (old disc
   keeps sanding); with an adder it means a stacked pair, which the manual
   requires the integrator to detect.
3. No nozzle. A released disc can hang on the rear of the knife; the next cycle
   carries it into a fresh pad and the first symptom is a latched drive fault.

Minimum sensing to change the verdict, deferred:

- Pad bare after remove: fixed camera on the changer frame looking at the pad face
  at wp5, colour threshold on the pad region. Owned by this service behind an
  optional `verify` block; with the block configured, `remove` fails when the
  check fails.
- Knife and drop path clear after release: through-beam optical sensor across the
  drop path under the remover, on a spare AL1342 digital input, read through the
  driver. A falling disc breaks the beam; a disc hanging on the blade never does.
- Double disc: not sensed. Guarded procedurally: apply only after pad-bare passed,
  plus the adder's own pad-covered check. The knife cannot be the sensor: the
  Festo drive publishes position and status bits but no force or torque.

## 7. Coordination points

Remover driver (no changes required now):

- The model name `autochanger-remover` and the link ids `body`, `blade`, `head`
  in `model.json` are API: the contact-step allow list names the geometries
  `autochanger-remover:<link id>`. The `label` fields do not reach the planner.
  The model name is fixed, so two removers in one frame system would collide on
  geometry names; one remover per cell is assumed.
- Digital input read, only when the through-beam lands.

Cell:

- Frame system has the remover with a calibrated frame (touch-off before first
  `remove`; all waypoints are relative to it), the arm, and `tool_frame` carrying
  the pad geometry.
- `input_range_override` identical here and in the sanding config. Fleet
  convention: one fragment referenced twice. Drift is silent (both planners
  succeed, one wraps the cable), which is why both sides log the limits they used.
- Snapshot mesh vision holds the pass mesh for the pass and is never `clear`ed
  mid-pass. No snapshot means this service refuses.

Sanding module:

- Calls `remove` only from its pad-change button, after its own retreat, and
  re-asserts the spindle afterwards. Documented in the sanding design.
- The Viam app renders DoCommand for every resource, so `remove` can be sent
  mid-pass from the app. Preflight refuses while the arm is moving; the operator
  doc says to use the webapp control.

## 8. Changes in this repo

Builds on RDK 1.9.0 (PR #16), which has mesh proto, `armplanning`, and
`CollisionSpecification`.

- New generic service package: config and validation, fixed step slice, planner
  wrapper, `remove` and `reset` DoCommands and `Status`, preflight including spindle stop,
  state.
- Copied `applyJointLimits`.
- `cmd/module/cmd.go` registration; `meta.json` model entry; README section
  (config table, DoCommand, preconditions, failure reports, default sequence
  with manual references, geometry-name contract).
- Tests: step-order table; allow-list and constraint construction against a fake
  frame system; DoCommand state machine with fake arm, gantry, and Mirka,
  including latch on failure, no latch on `not_started`, and `reset`; preflight
  refusals; runtime geometry-name resolution against a frame system built from
  `model.json`.

## 9. Deferred

- **Bumper into the kinematic model.** Centring and fold-down are Mirka's
  recommended pre-steps and go at the head of the step slice once the bumper has
  a frame. Until then waypoints assume a centred orbit.
- **Sensing** per section 6: `verify` block, camera, through-beam, driver
  digital input.
- **Air nozzle**, if Mirka's pneumatic kit is ever fitted: one more step after
  release, a valve the service owns.
- **Frame calibration procedure** for the remover mount, by touching off with
  the arm on the three front-plate holes the manual names. A doc and possibly a
  CLI helper; the service assumes it has been done.
- **Per-disc constants.** Press depth, lift height, tilt angle move to config
  only if rig tuning shows they vary by disc.
- **Adder** handoff: after `remove`, the arm is at wp5. The adder plans from
  there with the same obstacle and limit contracts.
