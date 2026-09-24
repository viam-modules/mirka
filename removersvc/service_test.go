package removersvc

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
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
		"knife.25",  // release
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
		{"pad geometry isn't a pad", func(r *rig) {
			parts := testParts(t)
			placeholder, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: -10}),
				r3.Vector{X: 1, Y: 1, Z: 1}, "")
			test.That(t, err, test.ShouldBeNil)
			parts[2].FrameConfig = referenceframe.NewLinkInFrame(referenceframe.World,
				spatialmath.NewPoseFromPoint(r3.Vector{X: 1000}), "compliance", placeholder)
			r.svc.fsSvc = newFakeFSFromParts(t, parts)
		}, "pad radius"},
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

func TestCancelDuringPreflightStopsBothActuators(t *testing.T) {
	r := newRig(t)
	ctx, cancel := context.WithCancel(context.Background())
	var stopCtxErr error
	r.arm.StopFunc = func(c context.Context, _ map[string]interface{}) error {
		stopCtxErr = c.Err()
		r.record("arm.stop")
		return nil
	}
	r.remover.MoveToPositionFunc = func(c context.Context, pos, _ []float64, _ map[string]interface{}) error {
		cancel() // preflight's grip move, the only knife move reached before a cancel
		return c.Err()
	}
	_, err := r.svc.DoCommand(ctx, map[string]interface{}{"command": "remove"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, r.log(), test.ShouldContain, "arm.stop")
	test.That(t, r.log(), test.ShouldContain, "knife.stop")
	test.That(t, stopCtxErr, test.ShouldBeNil)
	st := status(t, r)
	test.That(t, st["report"], test.ShouldEqual, "not_started")
	test.That(t, st["latched"], test.ShouldEqual, false)
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
