# Autochanger Remover Service Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A generic service `viam:mirka:autochanger-remover-svc` whose `remove` DoCommand runs the Mirka AutoChanger removal cycle end to end and stops, latched, wherever it fails.

**Architecture:** One new package `removersvc/`. A fixed slice of typed steps (arm waypoint or knife offset) is the sequence. Arm steps plan in-process with `armplanning` against a frame system rebuilt from the frame-system service's config, with the service's own joint limits applied. Contact steps add a linear constraint and a per-geometry collision allow list. Knife steps are `MoveToPosition` on the existing `autochanger-remover` gantry. State lives in memory, and a failure latch blocks the next `remove` until `reset`.

**Tech Stack:** Go, RDK v1.9.0 (`motionplan`, `motionplan/armplanning`, `referenceframe`, `spatialmath`, `robot/framesystem`, `testutils/inject`), `go.viam.com/test`.

**Spec:** `docs/remover_service_design.md`

## Global Constraints

- RDK is `go.viam.com/rdk v1.9.0` (PR #16); this branch is stacked on `bump-rdk-1.9.0`. No further bump.
- The driver package `autochanger/` is not modified.
- Model triplet: `viam:mirka:autochanger-remover-svc`, API `rdk:service:generic`.
- Knife offsets: `flush` = 0, `grip` = `grip_offset_mm`, `release` = `autochanger.KnifeTravelMM` (25).
- Contact-step allow entries are exactly `autochanger-remover:body`, `autochanger-remover:blade`, `autochanger-remover:head`, each paired with `tool_frame`. Never the whole `remover` component.
- Contact steps: wp2, wp3, wp4, wp5. Free step: wp1.
- A missing `input_range_override` is a refusal, never "use model limits". Limits only tighten.
- The service never sends `quit_error` and never auto-recovers.
- A failure after preflight latches; `not_started` does not latch. Only `reset` clears the latch.
- Comments explain the code as it is; no change-narration (user rule `code-comments.md`).
- Run `make lint` (gofmt) and `go test ./...` before every commit.

## Review Focus

1. **A vision service returns an object with a nil geometry, or no objects at all.** Preflight should refuse with the vision's name, not panic and not plan with no obstacles. Test in Task 5.
2. **Context cancelled mid-arm-step** (client disconnects). Arm `Stop` and knife `Stop` should both be called on a context that is not the cancelled one, and the result should latch as `in_contact`. Test in Task 5.
3. **Drive unreachable while collecting the failure report.** The original step error must survive: `Status` still returns `failed` with the step, and `drive_ready`/`knife_offset_mm` are simply absent. Test in Task 5.
4. **A second `remove` while one is running.** It's refused immediately and the running cycle is untouched; `reset` while running is refused the same way. Test in Task 5.
5. **`grip_offset_mm` of 0, negative, or ≥ 25** (a typo, or grip confused with release). Validate refuses. Test in Task 1.

---

## File Structure

| File | Responsibility |
|---|---|
| `removersvc/config.go` | `Config`, `Validate`, the dependency list |
| `removersvc/joints.go` | `applyJointLimits`, copied from the sanding module |
| `removersvc/steps.go` | step types, `removalSteps`, `waypoints`, manual constants |
| `removersvc/planning.go` | frame-system prep, pad radius, allow list, plan-request construction, default planner |
| `removersvc/service.go` | registration, constructor, `DoCommand`, `Status`, preflight, cycle, failure latch |
| `removersvc/fixtures_test.go` | shared test frame system and fakes |
| `removersvc/*_test.go` | one per file above |
| `cmd/module/cmd.go` | registration line |
| `meta.json` | model entry |
| `README.md` | service section |

---

### Task 1: Config and validation

**Files:**
- Create: `removersvc/config.go`
- Test: `removersvc/config_test.go`

**Interfaces:**
- Produces: `type Config struct { Remover, Arm, Mirka, ToolFrame string; ObstacleVisions []string; InputRangeOverride map[string]map[string]referenceframe.Limit; GripOffsetMM float64 }` and `func (c *Config) Validate(path string) ([]string, []string, error)`.

- [ ] **Step 1: Write the failing test**

```go
package removersvc

import (
	"testing"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

func validConfig() *Config {
	return &Config{
		Remover:         "remover",
		Arm:             "arm",
		Mirka:           "airos",
		ToolFrame:       "compliance",
		ObstacleVisions: []string{"snapshot_mesh_vision_service"},
		InputRangeOverride: map[string]map[string]referenceframe.Limit{
			"arm": {"2": {Min: -4, Max: 0}},
		},
		GripOffsetMM: 0.7,
	}
}

func TestValidate(t *testing.T) {
	deps, opt, err := validConfig().Validate("svc")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, opt, test.ShouldBeEmpty)
	test.That(t, deps, test.ShouldResemble,
		[]string{"remover", "arm", "airos", "snapshot_mesh_vision_service"})

	cases := []struct {
		name   string
		mutate func(*Config)
		want   string
	}{
		{"no remover", func(c *Config) { c.Remover = "" }, "remover"},
		{"no arm", func(c *Config) { c.Arm = "" }, "arm"},
		{"no mirka", func(c *Config) { c.Mirka = "" }, "mirka"},
		{"no tool frame", func(c *Config) { c.ToolFrame = "" }, "tool_frame"},
		{"no visions", func(c *Config) { c.ObstacleVisions = nil }, "obstacle_visions"},
		{"no limits", func(c *Config) { c.InputRangeOverride = nil }, "input_range_override"},
		{"grip zero", func(c *Config) { c.GripOffsetMM = 0 }, "grip_offset_mm"},
		{"grip negative", func(c *Config) { c.GripOffsetMM = -0.7 }, "grip_offset_mm"},
		{"grip at release", func(c *Config) { c.GripOffsetMM = 25 }, "grip_offset_mm"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			c := validConfig()
			tc.mutate(c)
			_, _, err := c.Validate("svc")
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, tc.want)
		})
	}
}
```

- [ ] **Step 2: Run it and confirm it fails**

Run: `go test ./removersvc -run TestValidate -v`
Expected: FAIL, `undefined: Config`.

- [ ] **Step 3: Implement**

```go
package removersvc

import (
	"fmt"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/utils"

	"github.com/viam-modules/mirka/autochanger"
)

// Config names what the service cannot derive: the resources it drives, the
// pad's frame, where obstacles come from, the cell's joint limits, and the one
// disc-dependent number the Mirka manual leaves to the integrator.
type Config struct {
	Remover         string   `json:"remover"`
	Arm             string   `json:"arm"`
	Mirka           string   `json:"mirka"`
	ToolFrame       string   `json:"tool_frame"`
	ObstacleVisions []string `json:"obstacle_visions"`
	// Must match the sanding config's copy. Drift is silent: both planners
	// succeed and one wraps the cable.
	InputRangeOverride map[string]map[string]referenceframe.Limit `json:"input_range_override"`
	GripOffsetMM       float64                                    `json:"grip_offset_mm"`
}

// Validate returns the remover, arm, Mirka and obstacle visions as required
// dependencies. The frame-system service is provided to modules implicitly.
func (c *Config) Validate(path string) ([]string, []string, error) {
	required := []struct{ field, value string }{
		{"remover", c.Remover}, {"arm", c.Arm}, {"mirka", c.Mirka}, {"tool_frame", c.ToolFrame},
	}
	for _, r := range required {
		if r.value == "" {
			return nil, nil, utils.NewConfigValidationFieldRequiredError(path, r.field)
		}
	}
	if len(c.ObstacleVisions) == 0 {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "obstacle_visions")
	}
	// The limits are load-bearing (cable wrap); model limits are not a fallback.
	if len(c.InputRangeOverride) == 0 {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "input_range_override")
	}
	if c.GripOffsetMM <= 0 || c.GripOffsetMM >= autochanger.KnifeTravelMM {
		return nil, nil, fmt.Errorf("%s: grip_offset_mm must be between 0 and %v exclusive, got %v",
			path, autochanger.KnifeTravelMM, c.GripOffsetMM)
	}
	deps := append([]string{c.Remover, c.Arm, c.Mirka}, c.ObstacleVisions...)
	return deps, nil, nil
}
```

- [ ] **Step 4: Run the test and confirm it passes**

Run: `go test ./removersvc -run TestValidate -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
make lint && go test ./...
git add removersvc/config.go removersvc/config_test.go
git commit -m "Add the remover service config"
```

---

### Task 2: Joint limits and the test frame system

**Files:**
- Create: `removersvc/joints.go`
- Create: `removersvc/fixtures_test.go`
- Test: `removersvc/joints_test.go`

**Interfaces:**
- Produces: `func applyJointLimits(logger logging.Logger, fs *referenceframe.FrameSystem, overrides map[string]map[string]referenceframe.Limit) error`.
- Produces (tests only): `func testParts(t *testing.T) []*referenceframe.FrameSystemPart` gives an arm `arm` (one revolute joint, ±360°), the remover `remover` from `autochanger.KinematicModel()`, and a tool frame `compliance` carrying a pad box of 150 × 150 × 20 mm whose face is the frame origin. `func testFrameSystem(t *testing.T) *referenceframe.FrameSystem`. Constant `testPadRadiusMM = 75.0`.

- [ ] **Step 1: Write the fixtures and the failing test**

`removersvc/fixtures_test.go`:

```go
package removersvc

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-modules/mirka/autochanger"
)

const testPadRadiusMM = 75.0

// One revolute joint is enough: planning is stubbed in service tests, and the
// joint-limit tests only need a SimpleModel with a known range.
var testArmJSON = []byte(`{
  "name": "arm",
  "links": [{"id": "base", "parent": "world", "translation": {"x": 0, "y": 0, "z": 0}}],
  "joints": [{"id": "j0", "type": "revolute", "parent": "base",
              "axis": {"x": 0, "y": 0, "z": 1}, "min": -360, "max": 360}]
}`)

func testParts(t *testing.T) []*referenceframe.FrameSystemPart {
	t.Helper()
	armModel, err := referenceframe.UnmarshalModelJSON(testArmJSON, "arm")
	test.That(t, err, test.ShouldBeNil)
	removerModel, err := autochanger.KinematicModel()
	test.That(t, err, test.ShouldBeNil)
	// The pad hangs behind its face, as in the sanding cell's compliance frame.
	pad, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Z: -10}),
		r3.Vector{X: 2 * testPadRadiusMM, Y: 2 * testPadRadiusMM, Z: 20}, "")
	test.That(t, err, test.ShouldBeNil)
	return []*referenceframe.FrameSystemPart{
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil), ModelFrame: armModel},
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "remover", nil), ModelFrame: removerModel},
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World,
			spatialmath.NewPoseFromPoint(r3.Vector{X: 1000}), "compliance", pad)},
	}
}

func testFrameSystem(t *testing.T) *referenceframe.FrameSystem {
	t.Helper()
	fs, err := referenceframe.NewFrameSystem("", testParts(t), nil)
	test.That(t, err, test.ShouldBeNil)
	return fs
}
```

`removersvc/joints_test.go`:

```go
package removersvc

import (
	"math"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

func TestApplyJointLimits(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("tightens and never loosens", func(t *testing.T) {
		fs := testFrameSystem(t)
		err := applyJointLimits(logger, fs, map[string]map[string]referenceframe.Limit{
			"arm": {"0": {Min: -1, Max: 100}},
		})
		test.That(t, err, test.ShouldBeNil)
		dof := fs.Frame("arm").DoF()
		test.That(t, dof[0].Min, test.ShouldAlmostEqual, -1)
		test.That(t, dof[0].Max, test.ShouldAlmostEqual, 2*math.Pi)
	})

	t.Run("matches joints by name", func(t *testing.T) {
		fs := testFrameSystem(t)
		err := applyJointLimits(logger, fs, map[string]map[string]referenceframe.Limit{
			"arm": {"j0": {Min: -1, Max: 1}},
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fs.Frame("arm").DoF()[0].Max, test.ShouldAlmostEqual, 1)
	})

	t.Run("unknown frame refuses", func(t *testing.T) {
		err := applyJointLimits(logger, testFrameSystem(t), map[string]map[string]referenceframe.Limit{
			"elbow": {"0": {Min: -1, Max: 1}},
		})
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "elbow")
	})

	t.Run("unknown joint refuses", func(t *testing.T) {
		err := applyJointLimits(logger, testFrameSystem(t), map[string]map[string]referenceframe.Limit{
			"arm": {"7": {Min: -1, Max: 1}},
		})
		test.That(t, err, test.ShouldNotBeNil)
	})
}
```

- [ ] **Step 2: Run it and confirm it fails**

Run: `go test ./removersvc -run TestApplyJointLimits -v`
Expected: FAIL, `undefined: applyJointLimits`.

- [ ] **Step 3: Implement**

`removersvc/joints.go` is a verbatim copy of `ApplyJointLimits` from `viamrobotics/sanding` `lib/framesys/joints.go`, renamed to `applyJointLimits`, with this doc comment in place of the upstream one:

```go
// applyJointLimits is a copy of ApplyJointLimits in viamrobotics/sanding
// lib/framesys/joints.go. Keep the two in step: both planners must plan with
// the same limits, and a divergence is silent. Limits only tighten, because
// the arm driver enforces its own kinematics at execution and rejects
// waypoints outside them.
```

The body stays as upstream: look up each frame, require `*referenceframe.SimpleModel`, match keys by moveable-frame name or stringified index, clamp to `math.Max(min)`/`math.Min(max)` with a `Warnf` when it tightens, then call `referenceframe.NewModelWithLimitOverrides` and `fs.ReplaceFrame`. Imports: `fmt`, `math`, `strconv`, `go.viam.com/rdk/logging`, `go.viam.com/rdk/referenceframe`. Drop the upstream `logger.Debugf` line, which passes a surplus argument.

- [ ] **Step 4: Run the test and confirm it passes**

Run: `go mod tidy && go test ./removersvc -run TestApplyJointLimits -v`
Expected: PASS. `go mod tidy` promotes `github.com/golang/geo` from indirect to direct.

- [ ] **Step 5: Commit**

```bash
make lint && go test ./...
git add go.mod go.sum removersvc/joints.go removersvc/joints_test.go removersvc/fixtures_test.go
git commit -m "Copy the sanding joint-limit override into the remover service"
```

---

### Task 3: The step sequence and waypoints

**Files:**
- Create: `removersvc/steps.go`
- Test: `removersvc/steps_test.go`

**Interfaces:**
- Produces:
  - `type stepKind int` with `armStep`, `knifeStep`
  - `type knifeTarget int` with `knifeGrip`, `knifeFlush`, `knifeRelease`
  - `type report string` with `reportNotStarted = "not_started"`, `reportArmDisplaced = "arm_displaced"`, `reportInContact = "in_contact"`, `reportKnifeHolding = "knife_holding"`
  - `type step struct { name string; kind stepKind; waypoint int; contact bool; knife knifeTarget; report report }`
  - `var removalSteps []step`
  - `type removerFeatures struct { plateFaceX, bladeTipZ float64 }`
  - `func readFeatures(gripMM float64) (removerFeatures, error)`
  - `func waypoints(f removerFeatures, padRadiusMM float64) [5]spatialmath.Pose`: tool-frame poses wp1..wp5 in the remover's origin frame (`<remover>_origin`)

**Geometry (confirmed by Vijay):** the knife tip is the blade's bottom edge (z = −260 in `model.json`). The pad works from below it. At wp1 the pad sits face-on to the plate with its top edge below the tip. At wp2 it presses 10 mm into the spring-loaded plate. At wp3 it lifts until its top edge is 15 mm above the tip, so the blade wedges in behind the disc. At wp4 it tilts back 20° about the tip line. At wp5 it retreats back (+X) and up (+Z). Every position is a model feature plus a manual distance, so re-measuring `model.json` moves the waypoints with it.

- [ ] **Step 1: Write the failing tests**

```go
package removersvc

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-modules/mirka/autochanger"
)

func TestRemovalStepOrder(t *testing.T) {
	type row struct {
		name    string
		kind    stepKind
		contact bool
		knife   knifeTarget
		report  report
	}
	want := []row{
		{"wp1", armStep, false, 0, reportArmDisplaced},
		{"wp2", armStep, true, 0, reportInContact},
		{"wp3", armStep, true, 0, reportInContact},
		{"clamp", knifeStep, false, knifeFlush, reportInContact},
		{"wp4", armStep, true, 0, reportInContact},
		{"wp5", armStep, true, 0, reportInContact},
		{"release", knifeStep, false, knifeRelease, reportKnifeHolding},
		{"regrip", knifeStep, false, knifeGrip, reportKnifeHolding},
	}
	test.That(t, len(removalSteps), test.ShouldEqual, len(want))
	wp := 0
	for i, w := range want {
		s := removalSteps[i]
		test.That(t, s.name, test.ShouldEqual, w.name)
		test.That(t, s.kind, test.ShouldEqual, w.kind)
		test.That(t, s.contact, test.ShouldEqual, w.contact)
		test.That(t, s.report, test.ShouldEqual, w.report)
		if s.kind == knifeStep {
			test.That(t, s.knife, test.ShouldEqual, w.knife)
		} else {
			test.That(t, s.waypoint, test.ShouldEqual, wp)
			wp++
		}
	}
}

func TestReadFeatures(t *testing.T) {
	f, err := readFeatures(0.7)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f.plateFaceX, test.ShouldAlmostEqual, 0, 1e-9)
	test.That(t, f.bladeTipZ, test.ShouldAlmostEqual, -260, 1e-9)
}

// padAt places the test pad (the fixtures' box) at a tool-frame pose.
func padAt(t *testing.T, pose spatialmath.Pose) spatialmath.Geometry {
	t.Helper()
	pad, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: -10}),
		r3.Vector{X: 2 * testPadRadiusMM, Y: 2 * testPadRadiusMM, Z: 20}, "pad")
	test.That(t, err, test.ShouldBeNil)
	return pad.Transform(pose)
}

func removerGeometries(t *testing.T, knifeMM float64) map[string]spatialmath.Geometry {
	t.Helper()
	m, err := autochanger.KinematicModel()
	test.That(t, err, test.ShouldBeNil)
	gif, err := m.Geometries([]referenceframe.Input{knifeMM})
	test.That(t, err, test.ShouldBeNil)
	out := map[string]spatialmath.Geometry{}
	for _, g := range gif.Geometries() {
		out[g.Label()] = g
	}
	return out
}

func collidesWith(t *testing.T, pad spatialmath.Geometry, geoms map[string]spatialmath.Geometry) []string {
	t.Helper()
	var hit []string
	for name, g := range geoms {
		c, _, err := pad.CollidesWith(g, 0)
		test.That(t, err, test.ShouldBeNil)
		if c {
			hit = append(hit, name)
		}
	}
	return hit
}

func toolZ(p spatialmath.Pose) r3.Vector {
	return spatialmath.Compose(p, spatialmath.NewPoseFromPoint(r3.Vector{Z: 1})).Point().Sub(p.Point())
}

func TestWaypoints(t *testing.T) {
	f, err := readFeatures(0.7)
	test.That(t, err, test.ShouldBeNil)
	wps := waypoints(f, testPadRadiusMM)
	grip := removerGeometries(t, 0.7)
	flush := removerGeometries(t, 0)

	t.Run("wp1 is clear of the remover, pad top below the tip", func(t *testing.T) {
		test.That(t, collidesWith(t, padAt(t, wps[0]), grip), test.ShouldBeEmpty)
		test.That(t, wps[0].Point().Z+testPadRadiusMM, test.ShouldBeLessThan, f.bladeTipZ)
	})
	t.Run("pad faces the plate through wp3", func(t *testing.T) {
		for _, i := range []int{0, 1, 2} {
			test.That(t, toolZ(wps[i]).X, test.ShouldAlmostEqual, -1, 1e-9)
		}
	})
	t.Run("wp2 presses 10 mm into the plate along -X", func(t *testing.T) {
		d := wps[1].Point().Sub(wps[0].Point())
		test.That(t, d.Y, test.ShouldAlmostEqual, 0)
		test.That(t, d.Z, test.ShouldAlmostEqual, 0)
		test.That(t, wps[1].Point().X, test.ShouldAlmostEqual, f.plateFaceX-pressDepthMM, 1e-9)
	})
	t.Run("wp3 lifts straight up until the tip is 15 mm into the pad", func(t *testing.T) {
		d := wps[2].Point().Sub(wps[1].Point())
		test.That(t, d.X, test.ShouldAlmostEqual, 0)
		test.That(t, d.Y, test.ShouldAlmostEqual, 0)
		test.That(t, wps[2].Point().Z+testPadRadiusMM, test.ShouldAlmostEqual, f.bladeTipZ+liftOverlapMM, 1e-9)
		// Spec 4.2: without the allow list the planner refuses wp3, because the
		// blade is inside the pad.
		test.That(t, collidesWith(t, padAt(t, wps[2]), grip), test.ShouldContain, "autochanger-remover:blade")
	})
	t.Run("wp4 tilts 20 degrees about the tip line, lower half away from the plate", func(t *testing.T) {
		theta := spatialmath.OrientationBetween(wps[2].Orientation(), wps[3].Orientation()).AxisAngles().Theta
		test.That(t, theta*180/math.Pi, test.ShouldAlmostEqual, tiltBackDeg, 1e-6)
		pivot := r3.Vector{X: f.plateFaceX, Z: f.bladeTipZ}
		test.That(t, wps[3].Point().Distance(pivot), test.ShouldAlmostEqual, wps[2].Point().Distance(pivot), 1e-6)
		// The pad centre is below the tip, so tilting back moves it away from the plate.
		test.That(t, wps[3].Point().X, test.ShouldBeGreaterThan, wps[2].Point().X)
		test.That(t, toolZ(wps[3]).Z, test.ShouldBeLessThan, 0)
	})
	t.Run("wp5 goes back and up and is clear with the knife flush", func(t *testing.T) {
		d := wps[4].Point().Sub(wps[3].Point())
		test.That(t, d.X, test.ShouldBeGreaterThan, 0)
		test.That(t, d.Z, test.ShouldBeGreaterThan, 0)
		test.That(t, spatialmath.OrientationAlmostEqual(wps[4].Orientation(), wps[3].Orientation()), test.ShouldBeTrue)
		test.That(t, collidesWith(t, padAt(t, wps[4]), flush), test.ShouldBeEmpty)
	})
}
```

- [ ] **Step 2: Run them and confirm they fail**

Run: `go test ./removersvc -run 'TestRemovalStepOrder|TestReadFeatures|TestWaypoints' -v`
Expected: FAIL, `undefined: removalSteps`.

- [ ] **Step 3: Implement**

```go
package removersvc

import (
	"fmt"
	"math"
	"strings"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-modules/mirka/autochanger"
)

type stepKind int

const (
	armStep stepKind = iota
	knifeStep
)

type knifeTarget int

const (
	knifeGrip knifeTarget = iota
	knifeFlush
	knifeRelease
)

// report is what a failure during a step leaves in the cell (spec section 5).
type report string

const (
	reportNotStarted   report = "not_started"
	reportArmDisplaced report = "arm_displaced"
	reportInContact    report = "in_contact"
	reportKnifeHolding report = "knife_holding"
)

type step struct {
	name string
	kind stepKind
	// Arm steps: index into waypoints, and whether the pad may touch the remover.
	waypoint int
	contact  bool
	// Knife steps.
	knife  knifeTarget
	report report
}

// removalSteps is the removal cycle from the Mirka AutoChanger manual. Every
// knife step sits between arm steps, so the knife only moves with the arm
// parked. wp5 is a contact step because it starts from wp4, with the pad still
// over the knife.
var removalSteps = []step{
	{name: "wp1", kind: armStep, waypoint: 0, report: reportArmDisplaced},
	{name: "wp2", kind: armStep, waypoint: 1, contact: true, report: reportInContact},
	{name: "wp3", kind: armStep, waypoint: 2, contact: true, report: reportInContact},
	{name: "clamp", kind: knifeStep, knife: knifeFlush, report: reportInContact},
	{name: "wp4", kind: armStep, waypoint: 3, contact: true, report: reportInContact},
	{name: "wp5", kind: armStep, waypoint: 4, contact: true, report: reportInContact},
	{name: "release", kind: knifeStep, knife: knifeRelease, report: reportKnifeHolding},
	{name: "regrip", kind: knifeStep, knife: knifeGrip, report: reportKnifeHolding},
}

// From the Mirka AutoChanger manual's removal cycle. Each moves to config only
// if rig tuning shows it varies by disc.
const (
	pressDepthMM  = 10.0 // the spring-loaded front plate gives about 10 mm
	liftOverlapMM = 15.0 // knife tip 10 to 20 mm under the pad
	tiltBackDeg   = 20.0
)

// Approach and retreat distances the manual leaves to the integrator.
const (
	approachStandoffMM = 5.0  // pad face off the plate at wp1
	belowTipMM         = 10.0 // pad top edge below the knife tip at wp1
	retreatMM          = 60.0 // wp5, along +X and along +Z
)

// removerFeatures are the remover faces the waypoints are measured from.
type removerFeatures struct {
	plateFaceX float64 // the body's +X face: the sliding plate
	bladeTipZ  float64 // the blade's bottom edge: the knife tip
}

// readFeatures reads the features from the remover's kinematic model, the
// same geometry the planner checks collisions against, so the waypoints and
// the collision model cannot disagree.
func readFeatures(gripMM float64) (removerFeatures, error) {
	m, err := autochanger.KinematicModel()
	if err != nil {
		return removerFeatures{}, err
	}
	gif, err := m.Geometries([]referenceframe.Input{gripMM})
	if err != nil {
		return removerFeatures{}, fmt.Errorf("remover geometries: %w", err)
	}
	type extent struct{ min, max r3.Vector }
	boxes := map[string]extent{}
	for _, g := range gif.Geometries() {
		box := g.ToProtobuf().GetBox()
		if box == nil {
			continue
		}
		// The model's boxes are axis-aligned in the remover's origin frame.
		half := r3.Vector{X: box.GetDimsMm().GetX(), Y: box.GetDimsMm().GetY(), Z: box.GetDimsMm().GetZ()}.Mul(0.5)
		c := g.Pose().Point()
		link := g.Label()[strings.LastIndex(g.Label(), ":")+1:]
		boxes[link] = extent{min: c.Sub(half), max: c.Add(half)}
	}
	body, okBody := boxes["body"]
	blade, okBlade := boxes["blade"]
	if !okBody || !okBlade {
		return removerFeatures{}, fmt.Errorf("remover model is missing its body or blade box; have %v", boxes)
	}
	return removerFeatures{plateFaceX: body.max.X, bladeTipZ: blade.min.Z}, nil
}

// waypoints returns wp1..wp5 as tool-frame poses in the remover's origin frame:
// plate front face, top, centred in Y; +X knife extension, +Z up. The tool
// frame's +Z is the pad normal, out of the pad face, so facing the plate is
// tool +Z along -X. The pad works from below the knife tip.
func waypoints(f removerFeatures, padRadiusMM float64) [5]spatialmath.Pose {
	facePlate := &spatialmath.OrientationVectorDegrees{OX: -1}
	belowZ := f.bladeTipZ - belowTipMM - padRadiusMM
	pressedX := f.plateFaceX - pressDepthMM

	wp1 := spatialmath.NewPose(r3.Vector{X: f.plateFaceX + approachStandoffMM, Z: belowZ}, facePlate)
	wp2 := spatialmath.NewPose(r3.Vector{X: pressedX, Z: belowZ}, facePlate)
	wp3 := spatialmath.NewPose(r3.Vector{X: pressedX, Z: f.bladeTipZ + liftOverlapMM - padRadiusMM}, facePlate)

	// Tilting back about the tip line swings the pad's lower half off the plate
	// while the disc edge stays pinned under the knife. A negative angle about
	// +Y moves points below the pivot toward +X.
	pivot := spatialmath.NewPoseFromPoint(r3.Vector{X: f.plateFaceX, Z: f.bladeTipZ})
	tilt := spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{Theta: -tiltBackDeg * math.Pi / 180, RY: 1})
	wp4 := spatialmath.Compose(pivot, spatialmath.Compose(tilt, spatialmath.Compose(spatialmath.PoseInverse(pivot), wp3)))

	wp5 := spatialmath.Compose(spatialmath.NewPoseFromPoint(r3.Vector{X: retreatMM, Z: retreatMM}), wp4)
	return [5]spatialmath.Pose{wp1, wp2, wp3, wp4, wp5}
}
```

If `wp5 ... clear with the knife flush` fails, the square test pad's corner is catching the head. Raise `retreatMM`; don't loosen the test.

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `go test ./removersvc -run 'TestRemovalStepOrder|TestReadFeatures|TestWaypoints' -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
make lint && go test ./...
git add removersvc/steps.go removersvc/steps_test.go
git commit -m "Derive the removal waypoints from the remover model"
```

---

### Task 4: Planning: frame system, pad, allow list, request

**Files:**
- Create: `removersvc/planning.go`
- Test: `removersvc/planning_test.go`

**Interfaces:**
- Consumes: `applyJointLimits` (Task 2).
- Produces:
  - `var removerGeometryNames = []string{"autochanger-remover:body", "autochanger-remover:blade", "autochanger-remover:head"}`
  - `func originFrame(remover string) string` returns `remover + "_origin"`
  - `func prepareFrameSystem(parts []*referenceframe.FrameSystemPart, cfg *Config, logger logging.Logger) (*referenceframe.FrameSystem, error)`
  - `func padRadius(fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs, toolFrame string) (float64, error)`
  - `func checkRemoverGeometries(fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs) error`
  - `func buildRequest(fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs, obstacles *referenceframe.GeometriesInFrame, cfg *Config, goal spatialmath.Pose, contact bool) *armplanning.PlanRequest`
  - `type planArmFunc func(ctx context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error)`
  - `func newPlanArm(logger logging.Logger, armName string) planArmFunc`

- [ ] **Step 1: Write the failing tests**

```go
package removersvc

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// Pins the runtime geometry names against a frame system built the way the
// frame-system service builds one. model.json's label fields do not survive;
// the planner sees <model name>:<link id>.
func TestRemoverGeometryNamesMatchFrameSystem(t *testing.T) {
	fs := testFrameSystem(t)
	test.That(t, checkRemoverGeometries(fs, referenceframe.NewZeroInputs(fs)), test.ShouldBeNil)
}

func TestCheckRemoverGeometriesRefusesWithoutRemover(t *testing.T) {
	parts := testParts(t)
	fs, err := referenceframe.NewFrameSystem("", []*referenceframe.FrameSystemPart{parts[0], parts[2]}, nil)
	test.That(t, err, test.ShouldBeNil)
	err = checkRemoverGeometries(fs, referenceframe.NewZeroInputs(fs))
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "autochanger-remover:body")
}

func TestPrepareFrameSystem(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cfg := validConfig()
	cfg.InputRangeOverride = map[string]map[string]referenceframe.Limit{"arm": {"0": {Min: -1, Max: 1}}}

	fs, err := prepareFrameSystem(testParts(t), cfg, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fs.Frame("arm").DoF()[0].Max, test.ShouldAlmostEqual, 1)

	cfg.ToolFrame = "nozzle"
	_, err = prepareFrameSystem(testParts(t), cfg, logger)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "nozzle")

	cfg = validConfig()
	cfg.InputRangeOverride = map[string]map[string]referenceframe.Limit{"elbow": {"0": {Min: -1, Max: 1}}}
	_, err = prepareFrameSystem(testParts(t), cfg, logger)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestPadRadius(t *testing.T) {
	fs := testFrameSystem(t)
	r, err := padRadius(fs, referenceframe.NewZeroInputs(fs), "compliance")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, r, test.ShouldAlmostEqual, testPadRadiusMM)

	// A frame with no geometry is a refusal: the planner would have no pad either.
	bare := referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "bare", nil)
	fs2, err := referenceframe.NewFrameSystem("", append(testParts(t),
		&referenceframe.FrameSystemPart{FrameConfig: bare}), nil)
	test.That(t, err, test.ShouldBeNil)
	_, err = padRadius(fs2, referenceframe.NewZeroInputs(fs2), "bare")
	test.That(t, err, test.ShouldNotBeNil)

	// The sanding sim's 1 mm placeholder box is not a pad.
	tiny, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{X: 1, Y: 1, Z: 1}, "")
	test.That(t, err, test.ShouldBeNil)
	fs3, err := referenceframe.NewFrameSystem("", append(testParts(t), &referenceframe.FrameSystemPart{
		FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "tiny", tiny)}), nil)
	test.That(t, err, test.ShouldBeNil)
	_, err = padRadius(fs3, referenceframe.NewZeroInputs(fs3), "tiny")
	test.That(t, err, test.ShouldNotBeNil)
}

func TestBuildRequest(t *testing.T) {
	fs := testFrameSystem(t)
	cfg := validConfig()
	inputs := referenceframe.NewZeroInputs(fs)
	obstacles := referenceframe.NewGeometriesInFrame(referenceframe.World, nil)
	goal := spatialmath.NewPoseFromPoint(r3.Vector{X: 5})

	free := buildRequest(fs, inputs, obstacles, cfg, goal, false)
	test.That(t, free.Constraints.LinearConstraint, test.ShouldBeEmpty)
	test.That(t, free.Constraints.CollisionSpecification, test.ShouldBeEmpty)
	test.That(t, free.ObstaclesInWorldFrame, test.ShouldEqual, obstacles)
	test.That(t, len(free.Goals), test.ShouldEqual, 1)
	pif := free.Goals[0].Poses()["compliance"]
	test.That(t, pif.Parent(), test.ShouldEqual, "remover_origin")
	test.That(t, spatialmath.PoseAlmostEqual(pif.Pose(), goal), test.ShouldBeTrue)

	contact := buildRequest(fs, inputs, obstacles, cfg, goal, true)
	test.That(t, len(contact.Constraints.LinearConstraint), test.ShouldEqual, 1)
	test.That(t, len(contact.Constraints.CollisionSpecification), test.ShouldEqual, 1)
	var got []string
	for _, a := range contact.Constraints.CollisionSpecification[0].Allows {
		test.That(t, a.Frame1, test.ShouldEqual, "compliance")
		test.That(t, a.Frame2, test.ShouldNotEqual, "remover")
		got = append(got, a.Frame2)
	}
	test.That(t, got, test.ShouldResemble, removerGeometryNames)
}
```

- [ ] **Step 2: Run them and confirm they fail**

Run: `go test ./removersvc -run 'TestRemoverGeometry|TestCheckRemover|TestPrepareFrameSystem|TestPadRadius|TestBuildRequest' -v`
Expected: FAIL, undefined symbols.

- [ ] **Step 3: Implement**

```go
package removersvc

import (
	"context"
	"fmt"
	"math"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// removerGeometryNames are the remover's geometries as the planner names them:
// <model name>:<link id>. model.json's label fields do not reach the planner.
// Allowing the remover component instead would allow every geometry it owns
// and let the pad drive through the body.
var removerGeometryNames = []string{
	"autochanger-remover:body",
	"autochanger-remover:blade",
	"autochanger-remover:head",
}

// Contact-step tolerances. The linear constraint makes wp3 to wp4 a chord of
// the tilt's arc; for a pad-centre distance of about 60 mm from the tip line
// the two differ by under 1 mm.
const (
	lineToleranceMM         = 1.0
	orientationToleranceDeg = 2.0
)

// Mirka pads are 77 to 150 mm across. Outside this range the tool frame's
// geometry is a placeholder, not a pad.
const (
	minPadRadiusMM = 30.0
	maxPadRadiusMM = 90.0
)

// originFrame is the remover's static mount frame. Waypoints are expressed in
// it, not in the remover frame, which rides the knife joint.
func originFrame(remover string) string {
	return remover + "_origin"
}

// prepareFrameSystem builds a frame system the way the frame-system service
// does and applies the service's own joint limits to it.
func prepareFrameSystem(
	parts []*referenceframe.FrameSystemPart, cfg *Config, logger logging.Logger,
) (*referenceframe.FrameSystem, error) {
	fs, err := referenceframe.NewFrameSystem("", parts, nil)
	if err != nil {
		return nil, err
	}
	for _, name := range []string{cfg.ToolFrame, cfg.Remover, cfg.Arm} {
		if fs.Frame(name) == nil {
			return nil, referenceframe.NewFrameMissingError(name)
		}
	}
	if err := applyJointLimits(logger, fs, cfg.InputRangeOverride); err != nil {
		return nil, fmt.Errorf("applying input_range_override: %w", err)
	}
	return fs, nil
}

// padRadius reads the pad from the tool frame's one geometry. A frame's
// configured geometry lands on its _origin frame.
func padRadius(fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs, toolFrame string) (float64, error) {
	all, err := referenceframe.FrameSystemGeometries(fs, inputs)
	if err != nil {
		return 0, err
	}
	var geoms []spatialmath.Geometry
	for _, name := range []string{toolFrame, toolFrame + "_origin"} {
		if gif, ok := all[name]; ok {
			geoms = append(geoms, gif.Geometries()...)
		}
	}
	if len(geoms) != 1 {
		return 0, fmt.Errorf("tool frame %q must carry exactly one geometry, the pad; found %d", toolFrame, len(geoms))
	}
	pb := geoms[0].ToProtobuf()
	var r float64
	switch {
	case pb.GetBox() != nil:
		d := pb.GetBox().GetDimsMm()
		r = math.Min(d.GetX(), d.GetY()) / 2
	case pb.GetCapsule() != nil:
		r = pb.GetCapsule().GetRadiusMm()
	default:
		return 0, fmt.Errorf("tool frame %q geometry must be a box or capsule to read the pad radius", toolFrame)
	}
	if r < minPadRadiusMM || r > maxPadRadiusMM {
		return 0, fmt.Errorf("tool frame %q pad radius %.1f mm is outside %v to %v mm; is the pad geometry configured?",
			toolFrame, r, minPadRadiusMM, maxPadRadiusMM)
	}
	return r, nil
}

// checkRemoverGeometries refuses when an allow entry would name nothing, which
// the planner reports far less legibly mid-cycle.
func checkRemoverGeometries(fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs) error {
	all, err := referenceframe.FrameSystemGeometries(fs, inputs)
	if err != nil {
		return err
	}
	have := map[string]bool{}
	for _, gif := range all {
		for _, g := range gif.Geometries() {
			have[g.Label()] = true
		}
	}
	for _, name := range removerGeometryNames {
		if !have[name] {
			return fmt.Errorf("frame system has no geometry %q; is the remover in the frame system with the autochanger-remover model?", name)
		}
	}
	return nil
}

func buildRequest(
	fs *referenceframe.FrameSystem,
	inputs referenceframe.FrameSystemInputs,
	obstacles *referenceframe.GeometriesInFrame,
	cfg *Config,
	goal spatialmath.Pose,
	contact bool,
) *armplanning.PlanRequest {
	constraints := motionplan.NewEmptyConstraints()
	if contact {
		constraints.AddLinearConstraint(motionplan.LinearConstraint{
			LineToleranceMm:          lineToleranceMM,
			OrientationToleranceDegs: orientationToleranceDeg,
		})
		allows := make([]motionplan.CollisionSpecificationAllowedFrameCollisions, 0, len(removerGeometryNames))
		for _, name := range removerGeometryNames {
			allows = append(allows, motionplan.CollisionSpecificationAllowedFrameCollisions{Frame1: cfg.ToolFrame, Frame2: name})
		}
		constraints.AddCollisionSpecification(motionplan.CollisionSpecification{Allows: allows})
	}
	return &armplanning.PlanRequest{
		FrameSystem:           fs,
		StartState:            armplanning.NewPlanState(nil, inputs),
		ObstaclesInWorldFrame: obstacles,
		Constraints:           constraints,
		Goals: []*armplanning.PlanState{armplanning.NewPlanState(referenceframe.FrameSystemPoses{
			cfg.ToolFrame: referenceframe.NewPoseInFrame(originFrame(cfg.Remover), goal),
		}, nil)},
	}
}

// planArmFunc plans one arm step and returns the arm's joint path. The service
// holds one so tests can stand in for the planner.
type planArmFunc func(ctx context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error)

func newPlanArm(logger logging.Logger, armName string) planArmFunc {
	return func(ctx context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		plan, _, err := armplanning.PlanMotion(ctx, logger, req)
		if err != nil {
			return nil, err
		}
		armFrame := req.FrameSystem.Frame(armName)
		path := make([][]referenceframe.Input, 0, len(plan.Trajectory()))
		for _, fsInputs := range plan.Trajectory() {
			in, err := fsInputs.GetFrameInputs(armFrame)
			if err != nil {
				return nil, fmt.Errorf("arm inputs from trajectory: %w", err)
			}
			path = append(path, in)
		}
		return path, nil
	}
}
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `go test ./removersvc -v`
Expected: PASS for everything so far.

- [ ] **Step 5: Commit**

```bash
make lint && go test ./...
git add removersvc/planning.go removersvc/planning_test.go
git commit -m "Build remover plan requests with a per-geometry allow list"
```

---

### Task 5: The service: preflight, cycle, latch, Status

**Files:**
- Create: `removersvc/service.go`
- Modify: `removersvc/fixtures_test.go` (append the fakes)
- Test: `removersvc/service_test.go`

**Interfaces:**
- Consumes: everything from Tasks 1–4, including `readFeatures` and `waypoints(f removerFeatures, padRadiusMM float64)` from Task 3.
- Produces:
  - `var Model = resource.NewModel("viam", "mirka", "autochanger-remover-svc")`
  - `func newService(name resource.Name, cfg *Config, a arm.Arm, remover gantry.Gantry, mirka resource.Resource, visions []vision.Service, fsSvc framesystem.Service, planArm planArmFunc, logger logging.Logger) *service`
  - DoCommand `{"command":"remove"}` returns `{"removed": true}`; `{"command":"reset"}` returns `{"state":"idle"}`.
  - Status keys: `state` (`idle` | `running` | `failed`), `step`, `step_index`, `error`, `report`, `latched`, `knife_offset_mm`, `drive_ready`, `arm_at_wp5`. The last three are omitted when unknown.

- [ ] **Step 1: Append the fakes to `removersvc/fixtures_test.go`**

Add these imports to the file's import block: `"context"`, `"sync"`, `"go.viam.com/rdk/components/arm"`, `"go.viam.com/rdk/logging"`, `"go.viam.com/rdk/motionplan/armplanning"`, `"go.viam.com/rdk/resource"`, `"go.viam.com/rdk/robot/framesystem"`, `"go.viam.com/rdk/services/vision"`, `"go.viam.com/rdk/testutils/inject"`, `viz "go.viam.com/rdk/vision"`.

```go
func newFakeFS(t *testing.T) *inject.FrameSystemService {
	t.Helper()
	parts := testParts(t)
	fs, err := referenceframe.NewFrameSystem("", parts, nil)
	test.That(t, err, test.ShouldBeNil)
	f := inject.NewFrameSystemService("fs")
	f.FrameSystemConfigFunc = func(context.Context) (*framesystem.Config, error) {
		return &framesystem.Config{Parts: parts}, nil
	}
	f.CurrentInputsFunc = func(context.Context) (referenceframe.FrameSystemInputs, error) {
		return referenceframe.NewZeroInputs(fs), nil
	}
	return f
}

// rig records every actuator call in order, so tests can assert the sequence.
type rig struct {
	mu    sync.Mutex
	calls []string

	arm     *inject.Arm
	remover *inject.Gantry
	mirka   *inject.GenericComponent
	vision  *inject.VisionService
	svc     *service
}

func (r *rig) record(s string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.calls = append(r.calls, s)
}

func (r *rig) log() []string {
	r.mu.Lock()
	defer r.mu.Unlock()
	return append([]string(nil), r.calls...)
}

func newRig(t *testing.T) *rig {
	t.Helper()
	r := &rig{
		arm:     inject.NewArm("arm"),
		remover: inject.NewGantry("remover"),
		mirka:   inject.NewGenericComponent("airos"),
		vision:  inject.NewVisionService("snapshot_mesh_vision_service"),
	}
	r.arm.IsMovingFunc = func(context.Context) (bool, error) { return false, nil }
	r.arm.StopFunc = func(context.Context, map[string]interface{}) error { r.record("arm.stop"); return nil }
	r.arm.MoveThroughJointPositionsFunc = func(ctx context.Context, p [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		r.record("arm.move")
		return nil
	}
	r.remover.IsMovingFunc = func(context.Context) (bool, error) { return false, nil }
	r.remover.StopFunc = func(context.Context, map[string]interface{}) error { r.record("knife.stop"); return nil }
	r.remover.MoveToPositionFunc = func(ctx context.Context, pos, _ []float64, _ map[string]interface{}) error {
		r.record(fmt.Sprintf("knife.%g", pos[0]))
		return nil
	}
	r.remover.PositionFunc = func(context.Context, map[string]interface{}) ([]float64, error) { return []float64{0.7}, nil }
	r.remover.StatusFunc = func(context.Context) (map[string]interface{}, error) {
		return map[string]interface{}{"ready": true}, nil
	}
	r.mirka.DoFunc = func(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		r.record("mirka." + cmd["command"].(string))
		return map[string]interface{}{"running": false}, nil
	}
	obstacle, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: -500}),
		r3.Vector{X: 100, Y: 100, Z: 100}, "piece")
	test.That(t, err, test.ShouldBeNil)
	r.vision.GetObjectPointCloudsFunc = func(context.Context, string, map[string]interface{}) ([]*viz.Object, error) {
		return []*viz.Object{{Geometry: obstacle}}, nil
	}
	planArm := func(ctx context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		r.record("plan")
		return [][]referenceframe.Input{{0}}, nil
	}
	// The test arm has one joint, so the override names joint 0.
	cfg := validConfig()
	cfg.InputRangeOverride = map[string]map[string]referenceframe.Limit{"arm": {"0": {Min: -1, Max: 1}}}
	r.svc = newService(resource.NewName(genericservice.API, "remover-svc"), cfg,
		r.arm, r.remover, r.mirka, []vision.Service{r.vision},
		newFakeFS(t), planArm, logging.NewTestLogger(t))
	return r
}
```

Also add `"fmt"` and `genericservice "go.viam.com/rdk/services/generic"` to the imports.

- [ ] **Step 2: Write the failing tests**

```go
package removersvc

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	viz "go.viam.com/rdk/vision"
	"go.viam.com/test"
)

func doRemove(r *rig) (map[string]interface{}, error) {
	return r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "remove"})
}

func status(t *testing.T, r *rig) map[string]interface{} {
	t.Helper()
	st, err := r.svc.Status(context.Background())
	test.That(t, err, test.ShouldBeNil)
	return st
}

func TestRemoveRunsTheManualSequence(t *testing.T) {
	r := newRig(t)
	out, err := doRemove(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["removed"], test.ShouldEqual, true)
	test.That(t, r.log(), test.ShouldResemble, []string{
		"mirka.stop", "knife.0.7", // preflight
		"plan", "arm.move", // wp1
		"plan", "arm.move", // wp2
		"plan", "arm.move", // wp3
		"knife.0",          // clamp
		"plan", "arm.move", // wp4
		"plan", "arm.move", // wp5
		"knife.25", // release
		"knife.0.7", // regrip
	})
	test.That(t, status(t, r)["state"], test.ShouldEqual, "idle")
}

func TestPreflightRefusals(t *testing.T) {
	cases := []struct {
		name  string
		setup func(r *rig)
		want  string
	}{
		{"no obstacles", func(r *rig) {
			r.vision.GetObjectPointCloudsFunc = func(context.Context, string, map[string]interface{}) ([]*viz.Object, error) {
				return nil, nil
			}
		}, "no obstacle geometry"},
		{"nil geometry", func(r *rig) {
			r.vision.GetObjectPointCloudsFunc = func(context.Context, string, map[string]interface{}) ([]*viz.Object, error) {
				return []*viz.Object{{}}, nil
			}
		}, "snapshot_mesh_vision_service"},
		{"vision error", func(r *rig) {
			r.vision.GetObjectPointCloudsFunc = func(context.Context, string, map[string]interface{}) ([]*viz.Object, error) {
				return nil, errors.New("no snapshot")
			}
		}, "no snapshot"},
		{"bad limits", func(r *rig) {
			r.svc.cfg.InputRangeOverride = map[string]map[string]referenceframe.Limit{"elbow": {"0": {Min: -1, Max: 1}}}
		}, "input_range_override"},
		{"arm moving", func(r *rig) {
			r.arm.IsMovingFunc = func(context.Context) (bool, error) { return true, nil }
		}, "arm is moving"},
		{"drive faulted", func(r *rig) {
			r.remover.MoveToPositionFunc = func(context.Context, []float64, []float64, map[string]interface{}) error {
				return errors.New("remover: drive is not ready (pd=0x0000); acknowledge faults with quit_error")
			}
		}, "quit_error"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			r := newRig(t)
			tc.setup(r)
			_, err := doRemove(r)
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, tc.want)
			st := status(t, r)
			test.That(t, st["state"], test.ShouldEqual, "failed")
			test.That(t, st["report"], test.ShouldEqual, "not_started")
			test.That(t, st["latched"], test.ShouldEqual, false)
			test.That(t, r.log(), test.ShouldNotContain, "arm.move")
		})
	}
}

func TestNotStartedDoesNotLatch(t *testing.T) {
	r := newRig(t)
	r.arm.IsMovingFunc = func(context.Context) (bool, error) { return true, nil }
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	r.arm.IsMovingFunc = func(context.Context) (bool, error) { return false, nil }
	_, err = doRemove(r)
	test.That(t, err, test.ShouldBeNil)
}

func TestFailureLatchesUntilReset(t *testing.T) {
	r := newRig(t)
	moves := 0
	r.arm.MoveThroughJointPositionsFunc = func(context.Context, [][]referenceframe.Input, *arm.MoveOptions, map[string]interface{}) error {
		moves++
		if moves == 2 { // wp2, the press
			return errors.New("protective stop")
		}
		return nil
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	st := status(t, r)
	test.That(t, st["step"], test.ShouldEqual, "wp2")
	test.That(t, st["step_index"], test.ShouldEqual, 1)
	test.That(t, st["report"], test.ShouldEqual, "in_contact")
	test.That(t, st["latched"], test.ShouldEqual, true)
	test.That(t, st["knife_offset_mm"], test.ShouldEqual, 0.7)
	test.That(t, st["drive_ready"], test.ShouldEqual, true)
	test.That(t, st["arm_at_wp5"], test.ShouldEqual, false)
	test.That(t, st["error"], test.ShouldContainSubstring, "protective stop")

	before := len(r.log())
	_, err = doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "reset")
	test.That(t, len(r.log()), test.ShouldEqual, before) // nothing moved, not even the knife

	out, err := r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "reset"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["state"], test.ShouldEqual, "idle")
	test.That(t, status(t, r)["state"], test.ShouldEqual, "idle")
}

func TestFailureReportClasses(t *testing.T) {
	cases := []struct {
		failKnifeAt float64
		failArmAt   int
		step        string
		report      string
	}{
		{failArmAt: 1, step: "wp1", report: "arm_displaced"},
		{failArmAt: 4, step: "wp4", report: "in_contact"},
		{failKnifeAt: 0, step: "clamp", report: "in_contact"},
		{failKnifeAt: 25, step: "release", report: "knife_holding"},
	}
	for _, tc := range cases {
		t.Run(tc.step, func(t *testing.T) {
			r := newRig(t)
			moves := 0
			r.arm.MoveThroughJointPositionsFunc = func(context.Context, [][]referenceframe.Input, *arm.MoveOptions, map[string]interface{}) error {
				moves++
				if moves == tc.failArmAt {
					return errors.New("arm fault")
				}
				return nil
			}
			if tc.failArmAt == 0 {
				r.remover.MoveToPositionFunc = func(_ context.Context, pos, _ []float64, _ map[string]interface{}) error {
					if pos[0] == tc.failKnifeAt {
						return errors.New("remover: move timed out")
					}
					return nil
				}
			}
			_, err := doRemove(r)
			test.That(t, err, test.ShouldNotBeNil)
			st := status(t, r)
			test.That(t, st["step"], test.ShouldEqual, tc.step)
			test.That(t, st["report"], test.ShouldEqual, tc.report)
			test.That(t, st["latched"], test.ShouldEqual, true)
		})
	}
}

func TestPlanFailureMovesNothingForThatStep(t *testing.T) {
	r := newRig(t)
	r.svc.planArm = func(context.Context, *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		return nil, errors.New("no IK solution")
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, r.log(), test.ShouldNotContain, "arm.move")
	test.That(t, status(t, r)["report"], test.ShouldEqual, "arm_displaced")
}

func TestFailureReportSurvivesUnreachableDrive(t *testing.T) {
	r := newRig(t)
	r.remover.MoveToPositionFunc = func(_ context.Context, pos, _ []float64, _ map[string]interface{}) error {
		if pos[0] == 0 {
			return errors.New("remover: cannot reach the drive")
		}
		return nil
	}
	r.remover.PositionFunc = func(context.Context, map[string]interface{}) ([]float64, error) {
		return nil, errors.New("unreachable")
	}
	r.remover.StatusFunc = func(context.Context) (map[string]interface{}, error) {
		return nil, errors.New("unreachable")
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	st := status(t, r)
	test.That(t, st["step"], test.ShouldEqual, "clamp")
	test.That(t, st["error"], test.ShouldContainSubstring, "cannot reach the drive")
	_, hasOffset := st["knife_offset_mm"]
	_, hasReady := st["drive_ready"]
	test.That(t, hasOffset, test.ShouldBeFalse)
	test.That(t, hasReady, test.ShouldBeFalse)
}

func TestCancelStopsBothActuators(t *testing.T) {
	r := newRig(t)
	ctx, cancel := context.WithCancel(context.Background())
	var stopCtxErr error
	r.arm.StopFunc = func(c context.Context, _ map[string]interface{}) error {
		stopCtxErr = c.Err()
		r.record("arm.stop")
		return nil
	}
	moves := 0
	r.arm.MoveThroughJointPositionsFunc = func(c context.Context, _ [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		moves++
		if moves == 3 { // wp3
			cancel()
			return c.Err()
		}
		return nil
	}
	_, err := r.svc.DoCommand(ctx, map[string]interface{}{"command": "remove"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, r.log(), test.ShouldContain, "arm.stop")
	test.That(t, r.log(), test.ShouldContain, "knife.stop")
	test.That(t, stopCtxErr, test.ShouldBeNil)
	st := status(t, r)
	test.That(t, st["report"], test.ShouldEqual, "in_contact")
	test.That(t, st["latched"], test.ShouldEqual, true)
}

func TestOneCycleAtATime(t *testing.T) {
	r := newRig(t)
	entered := make(chan struct{})
	release := make(chan struct{})
	r.arm.MoveThroughJointPositionsFunc = func(context.Context, [][]referenceframe.Input, *arm.MoveOptions, map[string]interface{}) error {
		select {
		case entered <- struct{}{}:
			<-release
		default:
		}
		return nil
	}
	done := make(chan error)
	go func() { _, err := doRemove(r); done <- err }()
	<-entered

	test.That(t, status(t, r)["state"], test.ShouldEqual, "running")
	test.That(t, status(t, r)["step"], test.ShouldEqual, "wp1")
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "already running")
	_, err = r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "reset"})
	test.That(t, err, test.ShouldNotBeNil)

	close(release)
	select {
	case err := <-done:
		test.That(t, err, test.ShouldBeNil)
	case <-time.After(5 * time.Second):
		t.Fatal("remove did not finish")
	}
}

func TestArmMovingBeforeKnifeStepRefuses(t *testing.T) {
	r := newRig(t)
	calls := 0
	r.arm.IsMovingFunc = func(context.Context) (bool, error) {
		calls++
		return calls > 1, nil // still at preflight, moving by the clamp
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, r.log(), test.ShouldNotContain, "knife.0")
}

func TestUnknownCommand(t *testing.T) {
	r := newRig(t)
	_, err := r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "quit_error"})
	test.That(t, err, test.ShouldNotBeNil)
}
```

Add `"go.viam.com/rdk/motionplan/armplanning"` to the test imports.

`TestArmMovingBeforeKnifeStepRefuses` depends on `IsMoving` being called once in preflight and next before the clamp. If the implementation also checks the arm before each arm step, adjust `calls > 1` to the count at the clamp and leave a comment naming each call.

- [ ] **Step 3: Run them and confirm they fail**

Run: `go test ./removersvc -run 'TestRemove|TestPreflight|TestNotStarted|TestFailure|TestPlanFailure|TestCancel|TestOneCycle|TestArmMoving|TestUnknown' -v`
Expected: FAIL, `undefined: newService`.

- [ ] **Step 4: Implement `removersvc/service.go`**

```go
package removersvc

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gantry"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-modules/mirka/autochanger"
)

var Model = resource.NewModel("viam", "mirka", "autochanger-remover-svc")

func init() {
	resource.RegisterService(genericservice.API, Model, resource.Registration[resource.Resource, *Config]{
		Constructor: newFromConfig,
	})
}

// Tolerance for reporting the arm at wp5 after a failure.
const (
	atWaypointMM  = 2.0
	atWaypointDeg = 2.0
)

// stopTimeout bounds the actuator stops sent after a cancelled remove, on a
// context that is not the cancelled one.
const stopTimeout = 5 * time.Second

type cycleState struct {
	state     string // idle, running, failed
	step      string
	stepIndex int
	err       error
	report    report
	latched   bool
	// Nil when the hardware could not be read while reporting.
	knifeOffsetMM *float64
	driveReady    *bool
	armAtWp5      *bool
}

type service struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	cfg     *Config
	arm     arm.Arm
	remover gantry.Gantry
	mirka   resource.Resource
	visions []vision.Service
	fsSvc   framesystem.Service
	planArm planArmFunc
	logger  logging.Logger

	// Held for a whole remove, and taken with TryLock by remove and reset, so a
	// second caller is refused rather than queued.
	cycle sync.Mutex
	mu    sync.Mutex // guards st
	st    cycleState
}

func newFromConfig(
	ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger,
) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	a, err := arm.FromDependencies(deps, cfg.Arm)
	if err != nil {
		return nil, err
	}
	rem, err := gantry.FromDependencies(deps, cfg.Remover)
	if err != nil {
		return nil, err
	}
	mirka, err := generic.FromDependencies(deps, cfg.Mirka)
	if err != nil {
		return nil, err
	}
	visions := make([]vision.Service, 0, len(cfg.ObstacleVisions))
	for _, name := range cfg.ObstacleVisions {
		v, err := vision.FromDependencies(deps, name)
		if err != nil {
			return nil, err
		}
		visions = append(visions, v)
	}
	fsSvc, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}
	return newService(conf.ResourceName(), cfg, a, rem, mirka, visions, fsSvc,
		newPlanArm(logger, cfg.Arm), logger), nil
}

func newService(
	name resource.Name, cfg *Config, a arm.Arm, remover gantry.Gantry, mirka resource.Resource,
	visions []vision.Service, fsSvc framesystem.Service, planArm planArmFunc, logger logging.Logger,
) *service {
	return &service{
		Named: name.AsNamed(), cfg: cfg, arm: a, remover: remover, mirka: mirka,
		visions: visions, fsSvc: fsSvc, planArm: planArm, logger: logger,
		st: cycleState{state: "idle", stepIndex: -1},
	}
}

func (s *service) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "remove":
		return s.remove(ctx)
	case "reset":
		return s.reset()
	default:
		return nil, fmt.Errorf("unknown command %v (supported: remove, reset)", cmd["command"])
	}
}

// reset records that a human has recovered the cell. It moves and reads nothing.
func (s *service) reset() (map[string]interface{}, error) {
	if !s.cycle.TryLock() {
		return nil, errors.New("remove is running; reset applies only to a stopped cycle")
	}
	defer s.cycle.Unlock()
	s.mu.Lock()
	s.st = cycleState{state: "idle", stepIndex: -1}
	s.mu.Unlock()
	return map[string]interface{}{"state": "idle"}, nil
}

// prepared is what preflight establishes for the whole cycle.
type prepared struct {
	fs        *referenceframe.FrameSystem
	obstacles *referenceframe.GeometriesInFrame
	waypoints [5]spatialmath.Pose
}

func (s *service) remove(ctx context.Context) (map[string]interface{}, error) {
	if !s.cycle.TryLock() {
		return nil, errors.New("remove is already running")
	}
	defer s.cycle.Unlock()

	s.mu.Lock()
	if s.st.latched {
		st := s.st
		s.mu.Unlock()
		return nil, fmt.Errorf("previous remove failed at %s (%s): %v; recover the cell, then send reset",
			st.step, st.report, st.err)
	}
	s.st = cycleState{state: "running", step: "preflight", stepIndex: -1}
	s.mu.Unlock()

	p, err := s.preflight(ctx)
	if err != nil {
		s.fail(ctx, -1, "preflight", reportNotStarted, err, p)
		return nil, err
	}
	for i, st := range removalSteps {
		s.mu.Lock()
		s.st.step, s.st.stepIndex = st.name, i
		s.mu.Unlock()
		if err := s.runStep(ctx, p, st); err != nil {
			if ctx.Err() != nil {
				s.stopActuators(ctx)
			}
			err = fmt.Errorf("remove step %s: %w", st.name, err)
			s.fail(ctx, i, st.name, st.report, err, p)
			return nil, err
		}
	}
	s.mu.Lock()
	s.st = cycleState{state: "idle", stepIndex: -1}
	s.mu.Unlock()
	return map[string]interface{}{"removed": true}, nil
}

// preflight refuses before anything moves, except the knife to grip and the
// spindle stopped. The knife move doubles as the drive readiness check: the
// driver refuses a move on a latched fault.
func (s *service) preflight(ctx context.Context) (*prepared, error) {
	obstacles, err := s.obstacles(ctx)
	if err != nil {
		return nil, err
	}
	fsCfg, err := s.fsSvc.FrameSystemConfig(ctx)
	if err != nil {
		return nil, fmt.Errorf("reading the frame system: %w", err)
	}
	fs, err := prepareFrameSystem(fsCfg.Parts, s.cfg, s.logger)
	if err != nil {
		return nil, err
	}
	s.logger.CInfow(ctx, "planning remove with joint limits", "input_range_override", s.cfg.InputRangeOverride)
	p := &prepared{fs: fs, obstacles: obstacles}

	inputs, err := s.fsSvc.CurrentInputs(ctx)
	if err != nil {
		return p, fmt.Errorf("reading current inputs: %w", err)
	}
	radius, err := padRadius(fs, inputs, s.cfg.ToolFrame)
	if err != nil {
		return p, err
	}
	if err := checkRemoverGeometries(fs, inputs); err != nil {
		return p, err
	}
	features, err := readFeatures(s.cfg.GripOffsetMM)
	if err != nil {
		return p, err
	}
	p.waypoints = waypoints(features, radius)

	if err := s.armParked(ctx); err != nil {
		return p, err
	}
	if _, err := s.mirka.DoCommand(ctx, map[string]interface{}{"command": "stop"}); err != nil {
		return p, fmt.Errorf("stopping the spindle: %w", err)
	}
	if err := s.remover.MoveToPosition(ctx, []float64{s.cfg.GripOffsetMM}, nil, nil); err != nil {
		return p, fmt.Errorf("moving the knife to grip: %w", err)
	}
	return p, nil
}

func (s *service) obstacles(ctx context.Context) (*referenceframe.GeometriesInFrame, error) {
	var geoms []spatialmath.Geometry
	for _, v := range s.visions {
		objs, err := v.GetObjectPointClouds(ctx, "", nil)
		if err != nil {
			return nil, fmt.Errorf("obstacles from %s: %w", v.Name().ShortName(), err)
		}
		for _, o := range objs {
			if o == nil || o.Geometry == nil {
				return nil, fmt.Errorf("obstacles from %s: object with no geometry", v.Name().ShortName())
			}
			geoms = append(geoms, o.Geometry)
		}
	}
	if len(geoms) == 0 {
		return nil, errors.New("no obstacle geometry from obstacle_visions; has the pass snapshot been taken?")
	}
	return referenceframe.NewGeometriesInFrame(referenceframe.World, geoms), nil
}

func (s *service) armParked(ctx context.Context) error {
	moving, err := s.arm.IsMoving(ctx)
	if err != nil {
		return fmt.Errorf("reading arm motion: %w", err)
	}
	if moving {
		return errors.New("arm is moving")
	}
	return nil
}

func (s *service) runStep(ctx context.Context, p *prepared, st step) error {
	switch st.kind {
	case knifeStep:
		if err := s.armParked(ctx); err != nil {
			return err
		}
		return s.remover.MoveToPosition(ctx, []float64{s.knifeOffset(st.knife)}, nil, nil)
	case armStep:
		moving, err := s.remover.IsMoving(ctx)
		if err != nil {
			return fmt.Errorf("reading knife motion: %w", err)
		}
		if moving {
			return errors.New("knife is moving")
		}
		inputs, err := s.fsSvc.CurrentInputs(ctx)
		if err != nil {
			return fmt.Errorf("reading current inputs: %w", err)
		}
		req := buildRequest(p.fs, inputs, p.obstacles, s.cfg, p.waypoints[st.waypoint], st.contact)
		path, err := s.planArm(ctx, req)
		if err != nil {
			return fmt.Errorf("planning: %w", err)
		}
		return s.arm.MoveThroughJointPositions(ctx, path, nil, nil)
	default:
		return fmt.Errorf("unknown step kind %d", st.kind)
	}
}

func (s *service) knifeOffset(k knifeTarget) float64 {
	switch k {
	case knifeGrip:
		return s.cfg.GripOffsetMM
	case knifeRelease:
		return autochanger.KnifeTravelMM
	default:
		return 0
	}
}

// stopActuators halts both actuators after a cancelled remove. The request
// context is already done, so the stops run on one that is not.
func (s *service) stopActuators(ctx context.Context) {
	stopCtx, cancel := context.WithTimeout(context.WithoutCancel(ctx), stopTimeout)
	defer cancel()
	if err := s.arm.Stop(stopCtx, nil); err != nil {
		s.logger.CErrorw(ctx, "stopping the arm after cancel", "err", err)
	}
	if err := s.remover.Stop(stopCtx, nil); err != nil {
		s.logger.CErrorw(ctx, "stopping the knife after cancel", "err", err)
	}
}

// fail records where the cycle stopped and what the hardware says. Reads that
// fail leave their field unknown rather than masking the step's error.
func (s *service) fail(ctx context.Context, index int, name string, rep report, cause error, p *prepared) {
	readCtx, cancel := context.WithTimeout(context.WithoutCancel(ctx), stopTimeout)
	defer cancel()
	st := cycleState{
		state: "failed", step: name, stepIndex: index, err: cause, report: rep,
		latched: rep != reportNotStarted,
	}
	if pos, err := s.remover.Position(readCtx, nil); err == nil && len(pos) == 1 {
		st.knifeOffsetMM = &pos[0]
	}
	if rs, err := s.remover.Status(readCtx); err == nil {
		if ready, ok := rs["ready"].(bool); ok {
			st.driveReady = &ready
		}
	}
	if p != nil && p.fs != nil {
		if at, err := s.armAtWaypoint(readCtx, p, 4); err == nil {
			st.armAtWp5 = &at
		}
	}
	s.logger.CErrorw(ctx, "remove failed", "step", name, "report", rep, "latched", st.latched, "err", cause)
	s.mu.Lock()
	s.st = st
	s.mu.Unlock()
}

func (s *service) armAtWaypoint(ctx context.Context, p *prepared, i int) (bool, error) {
	inputs, err := s.fsSvc.CurrentInputs(ctx)
	if err != nil {
		return false, err
	}
	tf, err := p.fs.Transform(inputs.ToLinearInputs(),
		referenceframe.NewPoseInFrame(s.cfg.ToolFrame, spatialmath.NewZeroPose()), originFrame(s.cfg.Remover))
	if err != nil {
		return false, err
	}
	pose := tf.(*referenceframe.PoseInFrame).Pose()
	want := p.waypoints[i]
	angle := spatialmath.OrientationBetween(pose.Orientation(), want.Orientation()).AxisAngles().Theta
	return pose.Point().Distance(want.Point()) <= atWaypointMM && math.Abs(angle)*180/math.Pi <= atWaypointDeg, nil
}

func (s *service) Status(ctx context.Context) (map[string]interface{}, error) {
	s.mu.Lock()
	st := s.st
	s.mu.Unlock()
	out := map[string]interface{}{"state": st.state}
	if st.state == "idle" {
		return out, nil
	}
	out["step"] = st.step
	out["step_index"] = st.stepIndex
	if st.state == "running" {
		return out, nil
	}
	out["error"] = st.err.Error()
	out["report"] = string(st.report)
	out["latched"] = st.latched
	if st.knifeOffsetMM != nil {
		out["knife_offset_mm"] = *st.knifeOffsetMM
	}
	if st.driveReady != nil {
		out["drive_ready"] = *st.driveReady
	}
	if st.armAtWp5 != nil {
		out["arm_at_wp5"] = *st.armAtWp5
	}
	return out, nil
}
```

In `newRig`, the fixture's tool frame sits at x = 1000 in world, far from any waypoint, so `arm_at_wp5` is `false` in `TestFailureLatchesUntilReset`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `go test ./removersvc -race -v`
Expected: PASS, with no race reports.

- [ ] **Step 6: Commit**

```bash
make lint && go test ./...
git add removersvc/service.go removersvc/service_test.go removersvc/fixtures_test.go
git commit -m "Add the remover service's remove and reset commands"
```

---

### Task 6: Registration, meta.json, README

**Files:**
- Modify: `cmd/module/cmd.go`
- Modify: `meta.json`
- Modify: `README.md` (new section after the autochanger-remover section)
- Modify: `docs/remover_service_design.md:3` (status line)

- [ ] **Step 1: Register the model**

In `cmd/module/cmd.go`, add the import `genericservice "go.viam.com/rdk/services/generic"` and `"github.com/viam-modules/mirka/removersvc"`, then append to `ModularMain`:

```go
		resource.APIModel{API: genericservice.API, Model: removersvc.Model},
```

- [ ] **Step 2: Add the meta.json entry**

Append to `models`:

```json
    {
      "api": "rdk:service:generic",
      "model": "viam:mirka:autochanger-remover-svc",
      "short_description": "Runs the Mirka AutoChanger disc-removal cycle: arm waypoints and knife moves in the manual's order"
    }
```

- [ ] **Step 3: Write the README section**

Add a `## viam:mirka:autochanger-remover-svc` section covering, in this order:
- One paragraph: what `remove` does, and that it ends at wp5 and returns the arm nowhere.
- Config table with each field from spec §4.1, required or not, and the reason it can't be derived.
- DoCommand table: `remove` and `reset`, copied from spec §4.5.
- Preconditions: the arm is retreated from contact and not moving; a pass snapshot exists; the remover's frame is calibrated; `input_range_override` matches the sanding config.
- Status fields and the failure table from spec §5, including the latch and the operator recovery sequence (arm home, `quit_error` if faulted, `reset`).
- Geometry-name contract: the allow list names `autochanger-remover:body|blade|head`, and why the `label` fields in `model.json` don't matter to it.
- The default sequence table from spec §2, with the manual constants from `removersvc/steps.go`, and how each waypoint is measured from the model (plate face, blade tip).
- Not unattended: the three reasons from spec §6, one line each.

- [ ] **Step 4: Update the spec's status line**

Replace `docs/remover_service_design.md` line 3 with:

```
Status: implemented in `removersvc/` (plan: `docs/superpowers/plans/2026-09-24-remover-service.md`).
```

- [ ] **Step 5: Build and test everything**

Run: `make lint && go vet ./... && go test -race ./... && make bin/viam-mirka`
Expected: all pass; the binary builds.

- [ ] **Step 6: Commit**

```bash
git add cmd/module/cmd.go meta.json README.md docs/remover_service_design.md
git commit -m "Register and document the autochanger remover service"
```

---

## Not in this plan (spec §9)

Bumper steps, sensing and `verify`, the air nozzle, the frame calibration procedure, per-disc constants in config, and the adder. The spec's deferred list is unchanged.

## Rig verification (after merge, by hand)

Unit tests stub the planner. The first real `remove` runs on the rig with a person at the e-stop. Check wp1 clearance and the wp3 overlap by eye before the clamp, and confirm the logged joint limits match the sanding config's.
