# Remover service: removal cycle and waypoints

How the `viam:mirka:autochanger-remover-svc` service takes a used disc off the
pad. Configuration and the DoCommand API are in the [module README](../README.md);
this file is the removal process itself, and how each arm pose is derived.

## The removal cycle

The sequence follows the removal steps in *Manual - Mirka AutoChanger*,
downloadable from
[Mirka's technical documents](https://www.mirka.com/en-us/support/downloads/technical-documents/?currentPage=1&searchTerm=autochanger). Arm and
knife strictly alternate: every knife move happens with the arm parked.

The cycle opens with the manual's bumper steps. The bumper is the black cap on
the knife head, so the knife sits flush, its zero-point, while the pad works
against it. With the spindle stopped the pad hangs on a free eccentric, 2.5 mm
off the spindle for the AIROS 550CV's 5 mm orbit. Pushing it up into the
bumper's V swings it to hang straight below the spindle, the position the
magazine picks from; pushing from the right and then the left makes that
repeatable. Then the left corner's grooves fold a curled disc edge back down,
which would otherwise catch the knife. Both corners are grooved so either can
suit the cell; on lab-sander-1 the right corner sits 43 mm from the arm's
pedestal, closer than the pad's radius. +Y is right, facing the plate.

| Step | Arm | Knife | Manual |
|---|---|---|---|
| centre-clear | Pad facing the plate, face 8 mm off it, below the bumper | flush | step 1 |
| centre-r-start | 50 mm back from the seat along the right slope's normal | flush | step 1 |
| centre-r | Push into the right slope until the pad seats against both slopes, then 1 mm further right and 3 mm up | flush | step 1, right-hand side |
| centre-l-start | 50 mm back from the seat along the left slope's normal | flush | step 1 |
| centre-l | Push into the left slope the same way, 1 mm further left and 3 mm up | flush | step 1, left-hand side |
| centre-drop | Straight down to below the bumper | flush | |
| fold-l-below | Below the left corner | flush | step 2 |
| fold-l-press | Up until the pad edge rides 1 mm past the groove faces | flush | step 2 |
| fold-l-stroke | Out along +X, 10 mm past the bumper's front | flush | step 2, pulling stroke |
| grip | Stationary, clear of the bumper | flush to grip | step 3 |
| wp1 | Pad parallel to the sliding plate, face 5 mm off it, pad top edge 25 mm below the knife tip | grip | step 3, waypoint 1 |
| wp2 | Press straight in 10 mm, the spring-loaded front plate's give | grip | step 4, waypoint 2 |
| wp3 | Straight up past the knife, until its tip shows 15 mm below the pad's bottom edge | grip; the knife runs between disc and pad | step 5, waypoint 3 |
| clamp | Stationary | grip to flush, pinching the disc against the plate | after step 5 |
| wp4 | Tilt the pad's top 20° away from the plate, pivoting on its bottom edge, easing 5 mm off the plate | flush | step 6, waypoint 4 |
| wp5 | Back 60 mm and up 60 mm | flush | step 7, waypoint 5 |
| release | Stationary | flush to release (25 mm); the disc drops | step 8 |
| regrip | Stationary | release to grip | step 8 |

Knife offsets are the remover's continuous 0 to 25 mm joint: flush is 0,
release is 25, and grip is `grip_offset_mm`, about 0.7 mm for a standard disc,
tuned by hand so a disc slides into the slot with slight clearance.

## Frames the waypoints use

Every waypoint is a pose of the tool frame (the pad) in the remover's origin
frame:

- **Origin**: the sliding plate's front face, at the top edge the body and
  blade share, centred across the plate.
- **Axes**: +X out of the plate (the direction the knife extends), +Y across
  the plate, +Z up. Every remover geometry hangs below z = 0.
- **Tool frame contract**: its origin is the centre of the pad face, its +Z is
  the outward pad normal, and its one geometry (the pad) sits behind the face
  along −Z. Facing the plate is tool +Z along −X. The pad radius is read from
  that geometry, half the smaller of its box's x and y.

Features are read off the remover's own kinematic model, the same geometry
the planner checks collisions against, so the waypoints and the collision model
cannot disagree:

- **Plate face** `plateFaceX`: the `body` geometry's +X face, x = 0.
- **Knife tip** `bladeTipZ`: the `blade` geometry's bottom edge, z = −260.
- **Bumper**, read with the knife flush across `head`, `head-left` and
  `head-right`: its top `headTopZ` (z = 0), its lowest corner `headBottomZ`
  (z ≈ −124) and its front `headFrontX` (x = 22).

The V's shape (`bumperFlatDropMM`, `bumperFlatHalfMM`, `bumperSlopeDeg`) is in
constants, measured from `headTopZ`, since the model's turned boxes do not give
it up as box extents. `TestModelTracesTheBumperV` holds the model and the
constants to the same V within half a millimetre.

The groove corners (`bumperCornerDropMM`, `bumperCornerYMM`) are constants too:
the model's slope boxes overhang the real bumper's bottom edge.

## How each waypoint is computed

### Bumper steps

With pad radius `r`, slope angle `θ` = 40°, the flat at `zf = headTopZ − 60`
reaching `y = ±20`, and orbit radius `e` = 2.5:

| Waypoint | Position (x, y, z) |
|---|---|
| below the bumper (`centre-clear`, `centre-drop`, `fold-l-below`) | `plateFaceX + 8`, `0` or `−82`, `headBottomZ − 5 − r − e` |
| seat | `plateFaceX + 8`, `0`, `(20 sin θ + zf cos θ − r) / cos θ + e` |
| push ends (`centre-r`, `centre-l`) | the seat plus `(±1, +3)` in (y, z), pressing about 3 mm into the pushed slope |
| push starts | each push end minus 50 mm along its slope's normal, `(±sin θ, cos θ)` in (y, z) |
| fold press | below-pose `x` and `y`, `headTopZ − 111 + 1 − r + e` |
| fold stroke | the press with `x = headFrontX + 10` |

Every bumper pose faces the plate. The seat is where a pad of radius `r`
touches both slopes. The tool frame is the spindle axis, and seated, the pad
hangs `e` below it, so the tool frame aims `e` higher. Below the bumper, the
pad could still sit `e` above the spindle, so the clearance allows for it.

Each push heads straight at its own slope, and moves away from the other slope
at only cos 80° of its travel. 50 mm leaves 9 mm, over the orbit plus what
the touch-off left uncertain.

The 8 mm standoff keeps a sander housing that starts 15 mm behind the pad face
(lab-sander-1's stack) clear of the bumper's 22 mm front. The pad still
overlaps the bumper's 20 mm depth.

### Removal steps

With pad radius `r`:

| Waypoint | Position (x, z) | Orientation |
|---|---|---|
| wp1 | `plateFaceX + 5`, `bladeTipZ − 25 − r` | pad facing the plate |
| wp2 | `plateFaceX − 10`, same z as wp1 | pad facing the plate |
| wp3 | `plateFaceX − 10`, `bladeTipZ + 15 + r` | pad facing the plate |
| wp4 | wp3 rotated 20° about +Y around the pivot (`plateFaceX − 10`, `bladeTipZ + 15`), then +5 along x | pad face turned 20° upward |
| wp5 | wp4 plus (+60, +60) | as wp4 |

All waypoints are centred in y. The wp4 pivot is the pad face's bottom edge at
wp3: the rotation swings the pad's top away from the plate and peels the disc's
outer edges while the clamped knife holds it, and wp5's back-and-up move then
releases the bottom. The 5 mm back-off eases the pad off the sprung plate so the
tilt does not load the end-effector stack.

## Tuning

The distances are constants in `steps.go`, from the manual:

| Constant | Value | Source |
|---|---|---|
| `bumperFlatDropMM`, `bumperFlatHalfMM`, `bumperSlopeDeg` | 68.4 mm, 20 mm, 40° | width and angle from a photo of lab-sander-1; depth touched off with the pad, 8.4 mm below the photo's bevelled front edge |
| `bumperCornerDropMM`, `bumperCornerYMM` | 119.4 mm, 82 mm | photo of lab-sander-1, shifted 8.4 mm with the V; not yet touched off |
| `orbitRadiusMM` | 2.5 mm | Mirka: the AIROS 550CV's 5 mm orbit |
| `bumperStandoffMM` | 8 mm | integrator's choice: pad face off the plate at the bumper |
| `centreApproachMM` | 50 mm | integrator's choice: each centring push's travel |
| `foldPressMM` | 1 mm | integrator's choice: pad edge past the groove faces |
| `foldPastMM` | 10 mm | integrator's choice: fold stroke past the bumper's front |
| `bumperClearMM` | 5 mm | integrator's choice: pad below the bumper between its moves |
| `centrePushOutMM`, `centrePushUpMM` | 1 mm, 3 mm | tuned on lab-sander-1: each push's press past the seat |
| `approachStandoffMM` | 5 mm | integrator's choice: pad face off the plate at wp1 |
| `belowTipMM` | 25 mm | tuned: pad top edge below the knife tip at wp1 |
| `pressDepthMM` | 10 mm | manual: the front plate gives about 10 mm |
| `liftOverlapMM` | 15 mm | manual: knife tip 10 to 20 mm under the pad at wp3 |
| `tiltBackDeg` | 20° | manual |
| `tiltBackoffMM` | 5 mm | tuned: eases the tilt off the plate |
| `retreatMM` | 60 mm | integrator's choice: wp5 along +X and +Z |

Changing one changes the cycle for every disc; a constant should move to config only
if tuning shows it varies by disc.

The bumper does not give, unlike the sprung plate, and the arm is position
controlled, so a target past real contact loads the force-torque sensor until it
trips. The seat was touched off with the pad, eccentric hanging down; the
corners were not, so pause before `fold-l-press` and check the pad's clearance
before running whole cycles. The seat also scales with the pad radius read
from `tool_frame`'s geometry, so that geometry must match the real pad.

## Planning

`centre-clear`, `fold-l-below` and wp1 are free moves: the arm
plans from wherever it is around the cell, checked against every geometry in
the frame system and the pass mesh from `obstacle_visions`. Every other arm
step is a contact step:

- **Straight line**: the tool frame stays within 1 mm of the straight line
  between consecutive waypoints, with orientation held within 2° of the
  direct turn between them. wp4's 20° tilt is held to that same 2° band
  around its rotation.
- **Contact allowed**: the planner models the 10 mm press as the pad sinking
  into a solid body, where the real plate springs back. So in contact steps the
  tool frame, and each frame in `contact_frames`, may touch the remover's
  `body`, `blade`, `head`, `head-left` and `head-right` geometries. Nothing
  else may, and free moves still avoid the remover entirely.
- **Joint limits**: the arm model's limits, tightened by
  `input_range_override` when set; use the sanding config's values.

The contact exemption names individual geometries, never the remover component,
which would expand to every geometry it owns. The planner labels each one
`<model name>:<link id>`, and the model name depends on how the frame system
was built: in process the model keeps its own name (`autochanger-remover:body`),
but a module receives the frame system as a proto, and RDK renames a model
rebuilt from one after its frame, the component (`remover:body`). So preflight
reads the labels from the remover frame's own geometries by link id.

## Running a cycle

Preflight plans every arm step, `centre-clear` to wp5, before anything moves. Each step
is planned from where the previous one ends, with the knife joint set to where
the cycle will have driven it, since the knife geometry moves between grip and
flush. If any step cannot be planned, the cycle is refused as `not_started`
with nothing moved, rather than failing later with the pad pressed into the
plate. Only then does preflight stop the spindle and move the knife to flush,
where the bumper steps need it.

Each arm step then runs its stored path, and refuses unless every arm joint is
within 0.01 rad (about 2 mm at the pad) of where that path starts, and fails
unless the arm ends within 0.01 rad of where the path ends: the xArm driver
returns once the arm stops moving, so a blocked arm or a controller fault would
otherwise read as success. Free moves
run at the arm's configured speed; the contact steps run at
`contact_speed_degs_per_sec` and `contact_acceleration_degs_per_sec_per_sec`
(10 °/s and 20 °/s² by default), whatever the arm is set to.

## Step mode

`step` runs the cycle one step per call and pauses between them: the first
call is preflight, which plans the whole cycle, and each later call runs one
step of the stored plan. A paused cycle resumes only with `step`, and only if
every arm joint is still within 0.01 rad of where the last arm step left it;
otherwise the call moves nothing and the cycle stays paused. `remove` refuses a
paused cycle rather than inherit its plan, and `reset` abandons it. People jog
the arm to measure while paused, and the remaining steps are paths planned from
that spot.
