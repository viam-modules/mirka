package removersvc

import (
	"context"
	"fmt"
	"math"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm/kinematics"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-modules/mirka/autochanger"
)

// ur5eStart has the flange pointing along world -X, about 50 mm short of wp1.
var ur5eStart = []referenceframe.Input{0, -math.Pi / 2, math.Pi / 2, 0, math.Pi / 2, 0}

// reachableCell puts the remover in front of a UR5e's tool at ur5eStart.
func reachableCell(t *testing.T) (*referenceframe.FrameSystem, referenceframe.FrameSystemInputs) {
	t.Helper()
	armModel, err := kinematics.ModelFromName(kinematics.UR5e, "arm")
	test.That(t, err, test.ShouldBeNil)
	removerModel, err := autochanger.KinematicModel()
	test.That(t, err, test.ShouldBeNil)
	pad, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: -10}),
		r3.Vector{X: 2 * testPadRadiusMM, Y: 2 * testPadRadiusMM, Z: 20}, "")
	test.That(t, err, test.ShouldBeNil)
	parts := []*referenceframe.FrameSystemPart{
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil), ModelFrame: armModel},
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World,
			spatialmath.NewPoseFromPoint(r3.Vector{X: -650, Y: -133, Z: 800}), "remover", nil), ModelFrame: removerModel},
		{FrameConfig: referenceframe.NewLinkInFrame("arm", spatialmath.NewPoseFromPoint(r3.Vector{Z: 100}), "compliance", pad)},
	}
	fs, err := referenceframe.NewFrameSystem("", parts, nil)
	test.That(t, err, test.ShouldBeNil)
	inputs := referenceframe.NewZeroInputs(fs)
	inputs["arm"] = ur5eStart
	return fs, inputs
}

// Runs the real planner through the whole cycle, so an unsatisfiable
// constraint fails here rather than on hardware.
func TestRealPlannerRunsTheCycle(t *testing.T) {
	fs, inputs := reachableCell(t)
	if err := runCycle(t, fs, inputs, validConfig()); err != nil {
		t.Fatal(err)
	}
}

// labSanderCell is reachableCell with lab-sander-1's end-of-arm stack.
func labSanderCell(t *testing.T) (*referenceframe.FrameSystem, referenceframe.FrameSystemInputs) {
	t.Helper()
	armModel, err := kinematics.ModelFromName(kinematics.UR5e, "arm")
	test.That(t, err, test.ShouldBeNil)
	removerModel, err := autochanger.KinematicModel()
	test.That(t, err, test.ShouldBeNil)
	link := func(parent string, z float64, name string, dims r3.Vector, geomZ float64) *referenceframe.FrameSystemPart {
		box, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: geomZ}), dims, "")
		test.That(t, err, test.ShouldBeNil)
		return &referenceframe.FrameSystemPart{
			FrameConfig: referenceframe.NewLinkInFrame(parent, spatialmath.NewPoseFromPoint(r3.Vector{Z: z}), name, box),
		}
	}
	parts := []*referenceframe.FrameSystemPart{
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil), ModelFrame: armModel},
		{FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World,
			spatialmath.NewPoseFromPoint(r3.Vector{X: -748, Y: -133, Z: 785}), "remover", nil), ModelFrame: removerModel},
		link("arm", 38, "ft-sensor", r3.Vector{X: 55, Y: 55, Z: 75}, 0),
		link("ft-sensor", 44, "plate", r3.Vector{X: 250, Y: 100, Z: 22}, 0),
		link("plate", 56, "mirka-body", r3.Vector{X: 120, Y: 120, Z: 90}, 0),
		link("mirka-body", 50, "sander", r3.Vector{X: 120, Y: 120, Z: 10}, 0),
		link("sander", 10, "compliance", r3.Vector{X: 120, Y: 120, Z: 10}, -5),
	}
	fs, err := referenceframe.NewFrameSystem("", parts, nil)
	test.That(t, err, test.ShouldBeNil)
	inputs := referenceframe.NewZeroInputs(fs)
	inputs["arm"] = ur5eStart
	return fs, inputs
}

// The sander housing sits inside wp2's press depth, so this cell needs
// contact_frames; the sander alone is enough.
func TestLabSanderCycleNeedsContactFrames(t *testing.T) {
	fs, inputs := labSanderCell(t)
	cfg := validConfig()
	err := runCycle(t, fs, inputs, cfg)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "planning wp2")
	test.That(t, err.Error(), test.ShouldContainSubstring, "sander_origin")

	fs, inputs = labSanderCell(t)
	cfg.ContactFrames = []string{"sander"}
	if err := runCycle(t, fs, inputs, cfg); err != nil {
		t.Fatal(err)
	}
}

// runCycle plans every arm step, checking each lands on its waypoint.
func runCycle(t *testing.T, fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs, cfg *Config) error {
	t.Helper()
	logger := logging.NewTestLogger(t)
	radius, err := padRadius(fs, inputs, cfg.ToolFrame)
	test.That(t, err, test.ShouldBeNil)
	features, err := readFeatures(cfg.GripOffsetMM)
	test.That(t, err, test.ShouldBeNil)
	wps := waypoints(features, radius)
	obstacles := referenceframe.NewGeometriesInFrame(referenceframe.World, nil)
	geoms, err := removerGeometryNames(fs, inputs, cfg.Remover)
	test.That(t, err, test.ShouldBeNil)
	plan := newPlanArm(logger, cfg.Arm)

	ctx, cancel := context.WithTimeout(context.Background(), time.Minute)
	defer cancel()
	inputs[cfg.Remover] = []referenceframe.Input{cfg.GripOffsetMM}
	for _, st := range removalSteps {
		if st.kind == knifeStep {
			inputs[cfg.Remover] = []referenceframe.Input{cfg.knifeOffset(st.knife)}
			continue
		}
		prev := wps[max(st.waypoint-1, 0)]
		req, err := buildRequest(fs, inputs, obstacles, geoms, cfg, prev, wps[st.waypoint], st.contact)
		test.That(t, err, test.ShouldBeNil)
		path, err := plan(ctx, req)
		if err != nil {
			return fmt.Errorf("planning %s: %w", st.name, err)
		}
		inputs[cfg.Arm] = path[len(path)-1]
		tf, err := fs.Transform(inputs.ToLinearInputs(),
			referenceframe.NewPoseInFrame(cfg.ToolFrame, spatialmath.NewZeroPose()), originFrame(cfg.Remover))
		test.That(t, err, test.ShouldBeNil)
		got := tf.(*referenceframe.PoseInFrame).Pose()
		want := wps[st.waypoint]
		test.That(t, got.Point().Distance(want.Point()), test.ShouldBeLessThan, 1)
		test.That(t, motionplan.OrientDist(got.Orientation(), want.Orientation()), test.ShouldBeLessThan, 1)
	}
	return nil
}
