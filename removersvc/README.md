# Remover service: removal cycle and waypoints

How the `viam:mirka:autochanger-remover-svc` service takes a used disc off the
pad: the removal process itself, and how each arm pose is derived.

## The removal cycle

The sequence follows the Mirka AutoChanger manual's removal steps. Arm and
knife strictly alternate: every knife move happens with the arm parked. The
manual's bumper pre-steps (orbit centring and folding the disc edge down) are
not implemented, so the waypoints assume a centred orbit.

| Step | Arm | Knife | Manual |
|---|---|---|---|
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

Two features are read off the remover's own kinematic model, the same geometry
the planner checks collisions against, so the waypoints and the collision model
cannot disagree:

- **Plate face** `plateFaceX`: the `body` geometry's +X face, x = 0.
- **Knife tip** `bladeTipZ`: the `blade` geometry's bottom edge, z = −260.

## How each waypoint is computed

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

The distances are constants in `steps.go`, from the manual or tuned on
lab-sander-1:

| Constant | Value | Source |
|---|---|---|
| `approachStandoffMM` | 5 mm | integrator's choice: pad face off the plate at wp1 |
| `belowTipMM` | 25 mm | tuned: pad top edge below the knife tip at wp1 |
| `pressDepthMM` | 10 mm | manual: the front plate gives about 10 mm |
| `liftOverlapMM` | 15 mm | manual: knife tip 10 to 20 mm under the pad at wp3 |
| `tiltBackDeg` | 20° | manual |
| `tiltBackoffMM` | 5 mm | tuned: eases the tilt off the plate |
| `retreatMM` | 60 mm | integrator's choice: wp5 along +X and +Z |

Changing one changes the cycle for every disc; a constant moves to config only
if tuning shows it varies by disc.
