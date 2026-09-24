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
