package removersvc

import (
	"context"
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

// reachableCell is a UR5e with the remover in front of it at the height and
// offset of the arm's tool in ur5eStart, and a pad 100 mm out from the flange.
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

// Runs the real planner through every arm step of the cycle, with the knife
// where the cycle has it, so a constraint the planner cannot satisfy (such as
// a fixed orientation tolerance on the tilt) fails here rather than on a
// clamped disc.
func TestRealPlannerRunsTheCycle(t *testing.T) {
	logger := logging.NewTestLogger(t)
	fs, inputs := reachableCell(t)
	cfg := validConfig()
	radius, err := padRadius(fs, inputs, cfg.ToolFrame)
	test.That(t, err, test.ShouldBeNil)
	features, err := readFeatures(cfg.GripOffsetMM)
	test.That(t, err, test.ShouldBeNil)
	wps := waypoints(features, radius)
	obstacles := referenceframe.NewGeometriesInFrame(referenceframe.World, nil)
	plan := newPlanArm(logger, cfg.Arm)
	svc := &service{cfg: cfg}

	ctx, cancel := context.WithTimeout(context.Background(), time.Minute)
	defer cancel()
	inputs[cfg.Remover] = []referenceframe.Input{cfg.GripOffsetMM}
	for _, st := range removalSteps {
		if st.kind == knifeStep {
			inputs[cfg.Remover] = []referenceframe.Input{svc.knifeOffset(st.knife)}
			continue
		}
		prev := wps[max(st.waypoint-1, 0)]
		req, err := buildRequest(fs, inputs, obstacles, cfg, prev, wps[st.waypoint], st.contact)
		test.That(t, err, test.ShouldBeNil)
		path, err := plan(ctx, req)
		if err != nil {
			t.Fatalf("planning %s: %v", st.name, err)
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
}
