package removersvc

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
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

// The waypoints assume the tool frame's origin is the pad face's centre with
// +Z out of the face; each way a pad geometry can break that is a refusal.
func TestPadRadiusRefusesOffContractGeometry(t *testing.T) {
	dims := r3.Vector{X: 2 * testPadRadiusMM, Y: 2 * testPadRadiusMM, Z: 20}
	cases := []struct {
		name string
		pose spatialmath.Pose
		want string
	}{
		{"face offset", spatialmath.NewPoseFromPoint(r3.Vector{Z: -40}), "pad face is at z=-30.0"},
		{"off centre", spatialmath.NewPoseFromPoint(r3.Vector{X: 5, Z: -10}), "centred at x=5.0"},
		{"rotated", spatialmath.NewPose(r3.Vector{Z: -10}, &spatialmath.R4AA{Theta: math.Pi / 18, RX: 1}), "rotated 10.0 degrees"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			pad, err := spatialmath.NewBox(tc.pose, dims, "")
			test.That(t, err, test.ShouldBeNil)
			parts := testParts(t)
			parts[2].FrameConfig = referenceframe.NewLinkInFrame(referenceframe.World,
				spatialmath.NewPoseFromPoint(r3.Vector{X: 1000}), "compliance", pad)
			fs, err := referenceframe.NewFrameSystem("", parts, nil)
			test.That(t, err, test.ShouldBeNil)
			_, err = padRadius(fs, referenceframe.NewZeroInputs(fs), "compliance")
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, `"compliance"`)
			test.That(t, err.Error(), test.ShouldContainSubstring, tc.want)
		})
	}
}

// The contract is checked in the tool frame, not world: a pad on a frame that
// is itself rotated and offset in world still passes.
func TestPadRadiusChecksInToolFrame(t *testing.T) {
	fs, inputs := reachableCell(t)
	r, err := padRadius(fs, inputs, "compliance")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, r, test.ShouldAlmostEqual, testPadRadiusMM)
}

func TestBuildRequest(t *testing.T) {
	fs := testFrameSystem(t)
	cfg := validConfig()
	inputs := referenceframe.NewZeroInputs(fs)
	obstacles := referenceframe.NewGeometriesInFrame(referenceframe.World, nil)
	goal := spatialmath.NewPoseFromPoint(r3.Vector{X: 5})

	free, err := buildRequest(fs, inputs, obstacles, cfg, goal, goal, false)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, free.Constraints.LinearConstraint, test.ShouldBeEmpty)
	test.That(t, free.Constraints.CollisionSpecification, test.ShouldBeEmpty)
	test.That(t, free.ObstaclesInWorldFrame, test.ShouldEqual, obstacles)
	test.That(t, len(free.Goals), test.ShouldEqual, 1)
	pif := free.Goals[0].Poses()["compliance"]
	test.That(t, pif.Parent(), test.ShouldEqual, referenceframe.World)
	test.That(t, spatialmath.PoseAlmostEqual(pif.Pose(), goal), test.ShouldBeTrue)

	contact, err := buildRequest(fs, inputs, obstacles, cfg, goal, goal, true)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, contact.Constraints.LinearConstraint, test.ShouldResemble, []motionplan.LinearConstraint{
		{LineToleranceMm: lineToleranceMM, OrientationToleranceDegs: orientationToleranceDeg},
	})
	test.That(t, contact.Constraints.PseudolinearConstraint, test.ShouldBeEmpty)
	test.That(t, len(contact.Constraints.CollisionSpecification), test.ShouldEqual, 1)
	var got []string
	for _, a := range contact.Constraints.CollisionSpecification[0].Allows {
		test.That(t, a.Frame1, test.ShouldEqual, "compliance")
		test.That(t, a.Frame2, test.ShouldNotEqual, "remover")
		got = append(got, a.Frame2)
	}
	test.That(t, got, test.ShouldResemble, removerGeometryNames)
}

// wp3 to wp4 rotates 20 degrees, so a fixed orientation tolerance would refuse
// its slerped midpoint; the tolerance must scale with the step's rotation.
func TestBuildRequestRotatingStep(t *testing.T) {
	fs := testFrameSystem(t)
	features, err := readFeatures(0.7)
	test.That(t, err, test.ShouldBeNil)
	wps := waypoints(features, testPadRadiusMM)
	req, err := buildRequest(fs, referenceframe.NewZeroInputs(fs), referenceframe.NewGeometriesInFrame(referenceframe.World, nil),
		validConfig(), wps[2], wps[3], true)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, req.Constraints.LinearConstraint, test.ShouldResemble, []motionplan.LinearConstraint{{LineToleranceMm: lineToleranceMM}})
	test.That(t, req.Constraints.PseudolinearConstraint, test.ShouldResemble, []motionplan.PseudolinearConstraint{
		{OrientationToleranceFactor: rotatingOrientationFactor},
	})
	test.That(t, rotatingOrientationFactor, test.ShouldBeGreaterThanOrEqualTo, 0.5)
}

// The goal reaches the planner in world, where armplanning computes the start
// pose; a remover away from world must still land the goal on the waypoint.
func TestBuildRequestGoalInWorld(t *testing.T) {
	fs, inputs := reachableCell(t)
	goal := spatialmath.NewPoseFromPoint(r3.Vector{X: 5, Z: -300})
	req, err := buildRequest(fs, inputs, referenceframe.NewGeometriesInFrame(referenceframe.World, nil),
		validConfig(), goal, goal, true)
	test.That(t, err, test.ShouldBeNil)
	pif := req.Goals[0].Poses()["compliance"]
	test.That(t, pif.Parent(), test.ShouldEqual, referenceframe.World)
	test.That(t, spatialmath.R3VectorAlmostEqual(pif.Pose().Point(), r3.Vector{X: -645, Y: -133, Z: 500}, 1e-6), test.ShouldBeTrue)
}
