package removersvc

import (
	"context"
	"errors"
	"math"
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
		"plan", "plan", "plan", "plan", "plan", // preflight plans wp1 to wp5
		"mirka.stop", "knife.0.7", // preflight
		"arm.move",  // wp1
		"arm.move",  // wp2
		"arm.move",  // wp3
		"knife.0",   // clamp
		"arm.move",  // wp4
		"arm.move",  // wp5
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

// Obstacles are optional.
func TestRemoveWithoutObstacles(t *testing.T) {
	r := newRig(t)
	r.vision.GetObjectPointCloudsFunc = func(context.Context, string, map[string]interface{}) ([]*viz.Object, error) {
		return nil, nil
	}
	out, err := doRemove(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["removed"], test.ShouldEqual, true)

	r = newRig(t)
	r.svc.cfg.ObstacleVisions, r.svc.visions = nil, nil
	out, err = doRemove(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["removed"], test.ShouldEqual, true)
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
		// Which move to failKnifeAt fails, from 0, counting preflight's grip.
		failKnifeNth int
		failArmAt    int
		step         string
		report       string
	}{
		{failArmAt: 1, step: "wp1", report: "arm_displaced"},
		{failArmAt: 3, step: "wp3", report: "in_contact"},
		{failArmAt: 4, step: "wp4", report: "in_contact"},
		{failArmAt: 5, step: "wp5", report: "in_contact"},
		{failKnifeAt: 0, step: "clamp", report: "in_contact"},
		{failKnifeAt: 25, step: "release", report: "knife_holding"},
		{failKnifeAt: 0.7, failKnifeNth: 1, step: "regrip", report: "knife_holding"},
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
				seen := 0
				r.remover.MoveToPositionFunc = func(_ context.Context, pos, _ []float64, _ map[string]interface{}) error {
					if pos[0] != tc.failKnifeAt {
						return nil
					}
					seen++
					if seen > tc.failKnifeNth {
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

// An unplannable step is refused at preflight, before anything moves.
func TestPlanFailureRefusesBeforeAnythingMoves(t *testing.T) {
	r := newRig(t)
	plans := 0
	r.svc.planArm = func(context.Context, *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		plans++
		if plans == 4 {
			return nil, errors.New("zero IK solutions")
		}
		return [][]referenceframe.Input{{0}}, nil
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "planning wp4")
	test.That(t, r.log(), test.ShouldBeEmpty)
	st := status(t, r)
	test.That(t, st["step"], test.ShouldEqual, "preflight")
	test.That(t, st["report"], test.ShouldEqual, "not_started")
	test.That(t, st["latched"], test.ShouldEqual, false)
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

// A reconfigured-away instance must stop driving the arm.
func TestCloseStopsARunningRemove(t *testing.T) {
	r := newRig(t)
	entered := make(chan struct{})
	r.arm.MoveThroughJointPositionsFunc = func(ctx context.Context, _ [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		r.record("arm.move")
		close(entered)
		<-ctx.Done()
		return ctx.Err()
	}
	done := make(chan error)
	go func() { _, err := doRemove(r); done <- err }()
	<-entered

	closed := make(chan error)
	go func() { closed <- r.svc.Close(context.Background()) }()
	select {
	case err := <-closed:
		test.That(t, err, test.ShouldBeNil)
	case <-time.After(5 * time.Second):
		t.Fatal("Close did not return")
	}
	// Seeing the stops now shows Close waited.
	test.That(t, r.log(), test.ShouldContain, "arm.stop")
	test.That(t, r.log(), test.ShouldContain, "knife.stop")
	test.That(t, <-done, test.ShouldNotBeNil)

	test.That(t, r.svc.Close(context.Background()), test.ShouldBeNil)
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "closed")
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
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm is moving")
	test.That(t, status(t, r)["step"], test.ShouldEqual, "clamp")
	test.That(t, r.log(), test.ShouldNotContain, "knife.0")
}

func TestUnknownCommand(t *testing.T) {
	r := newRig(t)
	_, err := r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "quit_error"})
	test.That(t, err, test.ShouldNotBeNil)
}

// Plans chain, with the knife simulated, and the arm runs exactly them.
func TestPreflightPlansTheWholeCycleChained(t *testing.T) {
	r := newRig(t)
	var reqs []*armplanning.PlanRequest
	r.svc.planArm = func(_ context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		reqs = append(reqs, req)
		start := req.StartState.Configuration()["arm"][0]
		return [][]referenceframe.Input{{start}, {float64(len(reqs)) / 10}}, nil
	}
	var moved [][][]referenceframe.Input
	r.arm.MoveThroughJointPositionsFunc = func(_ context.Context, p [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		moved = append(moved, p)
		r.moveArm(p[len(p)-1])
		return nil
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, reqs, test.ShouldHaveLength, 5)

	knife := func(i int) float64 { return reqs[i].StartState.Configuration()["remover"][0] }
	test.That(t, []float64{knife(0), knife(1), knife(2), knife(3), knife(4)}, test.ShouldResemble,
		[]float64{0.7, 0.7, 0.7, 0, 0})
	test.That(t, reqs[0].StartState.Configuration()["arm"][0], test.ShouldEqual, 0)
	for i := 1; i < 5; i++ {
		test.That(t, reqs[i].StartState.Configuration()["arm"][0], test.ShouldAlmostEqual, float64(i)/10)
	}
	test.That(t, moved, test.ShouldHaveLength, 5)
	for i, p := range moved {
		test.That(t, p[len(p)-1][0], test.ShouldAlmostEqual, float64(i+1)/10)
	}
}

// An arm off the next path's start is not driven along it.
func TestArmOffThePlannedStartFailsBeforeMoving(t *testing.T) {
	r := newRig(t)
	r.svc.planArm = func(_ context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		start := req.StartState.Configuration()["arm"][0]
		return [][]referenceframe.Input{{start}, {start + 0.1}}, nil
	}
	moves := 0
	r.arm.MoveThroughJointPositionsFunc = func(context.Context, [][]referenceframe.Input, *arm.MoveOptions, map[string]interface{}) error {
		moves++ // the arm reports success but stays where it was
		return nil
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "not where the planned path starts")
	test.That(t, moves, test.ShouldEqual, 1)
	st := status(t, r)
	test.That(t, st["step"], test.ShouldEqual, "wp2")
	test.That(t, st["latched"], test.ShouldEqual, true)
}

// Contact steps use the service's limits; wp1 keeps the arm's own.
func TestContactStepsMoveSlowly(t *testing.T) {
	for _, tc := range []struct {
		name          string
		speed, accel  float64
		wantV, wantAc float64
	}{
		{"defaults", 0, 0, 10, 20},
		{"configured", 5, 15, 5, 15},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r := newRig(t)
			r.svc.cfg.ContactSpeedDegsPerSec = tc.speed
			r.svc.cfg.ContactAccelerationDegsPerSecPerSec = tc.accel
			var opts []*arm.MoveOptions
			r.arm.MoveThroughJointPositionsFunc = func(_ context.Context, _ [][]referenceframe.Input, o *arm.MoveOptions, _ map[string]interface{}) error {
				opts = append(opts, o)
				return nil
			}
			_, err := doRemove(r)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, opts, test.ShouldHaveLength, 5)
			test.That(t, opts[0], test.ShouldBeNil)
			for _, o := range opts[1:] {
				test.That(t, o, test.ShouldNotBeNil)
				test.That(t, o.MaxVelRads, test.ShouldAlmostEqual, tc.wantV*math.Pi/180, 1e-12)
				test.That(t, o.MaxAccRads, test.ShouldAlmostEqual, tc.wantAc*math.Pi/180, 1e-12)
			}
		})
	}
}
