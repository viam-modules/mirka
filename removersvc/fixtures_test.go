package removersvc

import (
	"context"
	"fmt"
	"sync"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	viz "go.viam.com/rdk/vision"
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

func newFakeFS(t *testing.T) *inject.FrameSystemService {
	t.Helper()
	return newFakeFSFromParts(t, testParts(t))
}

// newFakeFSFromParts is newFakeFS parameterized on the frame-system parts, so
// a test can swap in a tool-frame geometry the service's preflight checks
// refuse on.
func newFakeFSFromParts(t *testing.T, parts []*referenceframe.FrameSystemPart) *inject.FrameSystemService {
	t.Helper()
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
