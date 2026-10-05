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

// One joint is enough: service tests stub the planner.
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
	// The pad sits behind its face.
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
