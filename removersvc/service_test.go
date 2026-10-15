package removersvc

import (
	"context"
	"errors"
	"fmt"
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

// armMove is how many arm moves a cycle has made once step name finishes.
func armMove(t *testing.T, name string) int {
	t.Helper()
	n := 0
	for _, s := range removalSteps {
		if s.kind == armStep {
			n++
		}
		if s.name == name {
			return n
		}
	}
	t.Fatalf("no step %s", name)
	return 0
}

func TestRemoveRunsTheManualSequence(t *testing.T) {
	r := newRig(t)
	out, err := doRemove(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["removed"], test.ShouldEqual, true)
	test.That(t, r.log(), test.ShouldResemble, []string{
		"plan", "plan", "plan", "plan", "plan", "plan", "plan", "plan", // preflight plans every arm step
		"mirka.stop", "knife.0", // preflight
		"arm.move",  // fold-l-below
		"arm.move",  // fold-l-press
		"arm.move",  // fold-l-stroke
		"knife.0.7", // grip
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
		{"pad touching the remover", func(r *rig) {
			parts := testParts(t)
			pad, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: -10}),
				r3.Vector{X: 2 * testPadRadiusMM, Y: 2 * testPadRadiusMM, Z: 20}, "")
			test.That(t, err, test.ShouldBeNil)
			// Pressed into the plate face, as a failed cycle can leave it.
			parts[2].FrameConfig = referenceframe.NewLinkInFrame(referenceframe.World,
				spatialmath.NewPose(r3.Vector{X: -5, Z: -300}, &spatialmath.OrientationVectorDegrees{OX: -1}), "compliance", pad)
			r.svc.fsSvc = newFakeFSFromParts(t, parts)
		}, "touching the remover"},
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

// A failure is reported, not latched: the next remove starts afresh with
// preflight, which refuses only if the arm is still in contact.
func TestFailureDoesNotLatch(t *testing.T) {
	r := newRig(t)
	moves := 0
	r.arm.MoveThroughJointPositionsFunc = func(context.Context, [][]referenceframe.Input, *arm.MoveOptions, map[string]interface{}) error {
		moves++
		if moves == armMove(t, "wp2") { // the press
			return errors.New("protective stop")
		}
		return nil
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	st := status(t, r)
	test.That(t, st["state"], test.ShouldEqual, "failed")
	test.That(t, st["step"], test.ShouldEqual, "wp2")
	test.That(t, st["step_index"], test.ShouldEqual, 5)
	test.That(t, st["report"], test.ShouldEqual, "in_contact")
	test.That(t, st["knife_offset_mm"], test.ShouldEqual, 0.7)
	test.That(t, st["drive_ready"], test.ShouldEqual, true)
	test.That(t, st["arm_at_wp5"], test.ShouldEqual, false)
	test.That(t, st["error"], test.ShouldContainSubstring, "protective stop")

	out, err := doRemove(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["removed"], test.ShouldEqual, true)
	test.That(t, status(t, r)["state"], test.ShouldEqual, "idle")
}

func TestFailureReportClasses(t *testing.T) {
	cases := []struct {
		failKnifeAt float64
		// Which move to failKnifeAt fails, from 0, counting preflight's flush.
		failKnifeNth int
		knife        bool
		step         string
		report       string
	}{
		{step: "fold-l-below", report: "arm_displaced"},
		{step: "fold-l-stroke", report: "in_contact"},
		{knife: true, failKnifeAt: 0.7, step: "grip", report: "arm_displaced"},
		{step: "wp1", report: "arm_displaced"},
		{step: "wp3", report: "in_contact"},
		{step: "wp4", report: "in_contact"},
		{step: "wp5", report: "in_contact"},
		{knife: true, failKnifeAt: 0, failKnifeNth: 1, step: "clamp", report: "in_contact"},
		{knife: true, failKnifeAt: 25, step: "release", report: "knife_holding"},
		{knife: true, failKnifeAt: 0.7, failKnifeNth: 1, step: "regrip", report: "knife_holding"},
	}
	for _, tc := range cases {
		t.Run(tc.step, func(t *testing.T) {
			r := newRig(t)
			moves := 0
			r.arm.MoveThroughJointPositionsFunc = func(context.Context, [][]referenceframe.Input, *arm.MoveOptions, map[string]interface{}) error {
				moves++
				if !tc.knife && moves == armMove(t, tc.step) {
					return errors.New("arm fault")
				}
				return nil
			}
			if tc.knife {
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
		})
	}
}

// An unplannable step is refused at preflight, before anything moves.
func TestPlanFailureRefusesBeforeAnythingMoves(t *testing.T) {
	r := newRig(t)
	plans := 0
	r.svc.planArm = func(context.Context, *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		plans++
		if plans == armMove(t, "wp4") {
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
}

func TestFailureReportSurvivesUnreachableDrive(t *testing.T) {
	r := newRig(t)
	flushes := 0
	r.remover.MoveToPositionFunc = func(_ context.Context, pos, _ []float64, _ map[string]interface{}) error {
		if pos[0] == 0 {
			if flushes++; flushes > 1 { // preflight's flush passes; the clamp fails
				return errors.New("remover: cannot reach the drive")
			}
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
		if moves == armMove(t, "wp3") {
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
		cancel() // preflight's flush move, the only knife move reached before a cancel
		return c.Err()
	}
	_, err := r.svc.DoCommand(ctx, map[string]interface{}{"command": "remove"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, r.log(), test.ShouldContain, "arm.stop")
	test.That(t, r.log(), test.ShouldContain, "knife.stop")
	test.That(t, stopCtxErr, test.ShouldBeNil)
	st := status(t, r)
	test.That(t, st["report"], test.ShouldEqual, "not_started")
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
	test.That(t, status(t, r)["step"], test.ShouldEqual, "fold-l-below")
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "already running")

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
		return calls > 1, nil // still at preflight, moving by the grip
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "arm is moving")
	test.That(t, status(t, r)["step"], test.ShouldEqual, "grip")
	test.That(t, r.log(), test.ShouldNotContain, "knife.0.7")
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
	n := armMove(t, "wp5")
	test.That(t, reqs, test.ShouldHaveLength, n)

	// Flush through the bumper steps, grip from wp1 to wp3, flush once clamped.
	var knife []float64
	for _, req := range reqs {
		knife = append(knife, req.StartState.Configuration()["remover"][0])
	}
	want := make([]float64, n)
	for i := armMove(t, "wp1") - 1; i < armMove(t, "wp3"); i++ {
		want[i] = 0.7
	}
	test.That(t, knife, test.ShouldResemble, want)
	test.That(t, reqs[0].StartState.Configuration()["arm"][0], test.ShouldEqual, 0)
	for i := 1; i < n; i++ {
		test.That(t, reqs[i].StartState.Configuration()["arm"][0], test.ShouldAlmostEqual, float64(i)/10)
	}
	test.That(t, moved, test.ShouldHaveLength, n)
	for i, p := range moved {
		test.That(t, p[len(p)-1][0], test.ShouldAlmostEqual, float64(i+1)/10)
	}
}

// An arm off the next path's start is not driven along it.
// A move the driver reports done without arriving fails at that step: the
// xArm driver returns once the arm stops, so a blocked arm reads as success.
func TestMoveThatDoesNotArriveFails(t *testing.T) {
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
	test.That(t, err.Error(), test.ShouldContainSubstring, "did not reach the end of its planned path")
	test.That(t, moves, test.ShouldEqual, 1)
	test.That(t, status(t, r)["step"], test.ShouldEqual, removalSteps[0].name)
}

// An arm moved between steps is not driven along the next path.
func TestArmOffThePlannedStartFailsBeforeMoving(t *testing.T) {
	r := newRig(t)
	r.svc.planArm = func(_ context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		start := req.StartState.Configuration()["arm"][0]
		return [][]referenceframe.Input{{start}, {start + 0.1}}, nil
	}
	moves := 0
	r.arm.MoveThroughJointPositionsFunc = func(_ context.Context, p [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		moves++
		r.moveArm(p[len(p)-1])
		return nil
	}
	flushes := 0
	r.remover.MoveToPositionFunc = func(_ context.Context, pos, _ []float64, _ map[string]interface{}) error {
		if pos[0] == 0 {
			if flushes++; flushes == 2 { // the clamp, after preflight's flush; the arm is knocked while the knife moves
				r.moveArm([]referenceframe.Input{-1})
			}
		}
		return nil
	}
	_, err := doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "not where the planned path starts")
	test.That(t, moves, test.ShouldEqual, armMove(t, "wp3"))
	test.That(t, status(t, r)["step"], test.ShouldEqual, "wp4")
}

// Contact steps use the service's limits; free moves keep the arm's own.
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
			var arms []step
			for _, s := range removalSteps {
				if s.kind == armStep {
					arms = append(arms, s)
				}
			}
			test.That(t, opts, test.ShouldHaveLength, len(arms))
			for i, o := range opts {
				if !arms[i].contact {
					test.That(t, o, test.ShouldBeNil)
					continue
				}
				test.That(t, o, test.ShouldNotBeNil)
				test.That(t, o.MaxVelRads, test.ShouldAlmostEqual, tc.wantV*math.Pi/180, 1e-12)
				test.That(t, o.MaxAccRads, test.ShouldAlmostEqual, tc.wantAc*math.Pi/180, 1e-12)
			}
		})
	}
}

func doStep(r *rig) (map[string]interface{}, error) {
	return r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "step"})
}

// preflightSequence plans every arm step, stops the spindle and flushes the knife.
var preflightSequence = func() []string {
	var out []string
	for _, s := range removalSteps {
		if s.kind == armStep {
			out = append(out, "plan")
		}
	}
	return append(out, "mirka.stop", "knife.0")
}()

var fullSequence = func() []string {
	out := append([]string(nil), preflightSequence...)
	for _, s := range removalSteps {
		if s.kind == armStep {
			out = append(out, "arm.move")
		} else {
			out = append(out, fmt.Sprintf("knife.%g", validConfig().knifeOffset(s.knife)))
		}
	}
	return out
}()

func TestStepWalksTheSequence(t *testing.T) {
	r := newRig(t)

	out, err := doStep(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out, test.ShouldResemble, map[string]interface{}{"completed": "preflight", "next": "fold-l-below"})
	test.That(t, r.log(), test.ShouldResemble, preflightSequence)
	st := status(t, r)
	test.That(t, st["state"], test.ShouldEqual, "paused")
	test.That(t, st["completed"], test.ShouldEqual, "preflight")
	test.That(t, st["next"], test.ShouldEqual, "fold-l-below")

	out, err = doStep(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out, test.ShouldResemble, map[string]interface{}{"completed": "fold-l-below", "next": "fold-l-press"})
	test.That(t, r.log(), test.ShouldResemble, append(append([]string(nil), preflightSequence...), "arm.move"))

	for i := 1; i < len(removalSteps)-1; i++ {
		out, err = doStep(r)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, out["completed"], test.ShouldEqual, removalSteps[i].name)
		test.That(t, out["next"], test.ShouldEqual, removalSteps[i+1].name)
	}
	out, err = doStep(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out, test.ShouldResemble, map[string]interface{}{"completed": "regrip", "removed": true})
	test.That(t, r.log(), test.ShouldResemble, fullSequence)
	test.That(t, status(t, r)["state"], test.ShouldEqual, "idle")
}

// remove never inherits a plan a step cycle paused.
func TestRemoveRefusesAPausedCycle(t *testing.T) {
	r := newRig(t)
	_, err := doStep(r)
	test.That(t, err, test.ShouldBeNil)
	_, err = doStep(r)
	test.That(t, err, test.ShouldBeNil)
	before := len(r.log())

	_, err = doRemove(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "paused before fold-l-press")
	test.That(t, len(r.log()), test.ShouldEqual, before)
	test.That(t, status(t, r)["state"], test.ShouldEqual, "paused")
}

func TestResetWithNothingPausedErrors(t *testing.T) {
	r := newRig(t)
	_, err := r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "reset"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "nothing to reset")
}

func TestResetAbandonsAPausedCycle(t *testing.T) {
	r := newRig(t)
	_, err := doStep(r)
	test.That(t, err, test.ShouldBeNil)

	out, err := r.svc.DoCommand(context.Background(), map[string]interface{}{"command": "reset"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["state"], test.ShouldEqual, "idle")
	test.That(t, status(t, r)["state"], test.ShouldEqual, "idle")

	// The next step starts over at preflight.
	out, err = doStep(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["completed"], test.ShouldEqual, "preflight")
	test.That(t, r.log(), test.ShouldResemble, append(append([]string(nil), preflightSequence...), preflightSequence...))
}

func TestResumeRefusedWhenArmHasMoved(t *testing.T) {
	r := newRig(t)
	_, err := doStep(r)
	test.That(t, err, test.ShouldBeNil)
	// Resuming at the first arm step checks nothing.
	_, err = doStep(r)
	test.That(t, err, test.ShouldBeNil)
	before := len(r.log())

	r.moveArm([]referenceframe.Input{0.5}) // jogged while paused
	_, err = doStep(r)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "not at fold-l-below")
	test.That(t, len(r.log()), test.ShouldEqual, before)
	st := status(t, r)
	test.That(t, st["state"], test.ShouldEqual, "paused")
	test.That(t, st["next"], test.ShouldEqual, "fold-l-press")

	r.moveArm([]referenceframe.Input{0}) // returned to fold-l-below
	out, err := doStep(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["completed"], test.ShouldEqual, "fold-l-press")
}

func TestResumeChecksTheLastWaypointBeforeAKnifeStep(t *testing.T) {
	r := newRig(t)
	for _, s := range append([]step{{name: "preflight"}}, removalSteps...) { // through wp3
		_, err := doStep(r)
		test.That(t, err, test.ShouldBeNil)
		if s.name == "wp3" {
			break
		}
	}
	before := len(r.log())
	r.moveArm([]referenceframe.Input{0.5})
	_, err := doStep(r) // clamp, which needs the arm still at wp3
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "not at wp3")
	test.That(t, len(r.log()), test.ShouldEqual, before)
}

// A failed step ends the cycle without latching; the next step starts afresh.
func TestStepFailureEndsTheCycle(t *testing.T) {
	r := newRig(t)
	_, err := doStep(r)
	test.That(t, err, test.ShouldBeNil)
	_, err = doStep(r)
	test.That(t, err, test.ShouldBeNil)
	r.arm.MoveThroughJointPositionsFunc = func(context.Context, [][]referenceframe.Input, *arm.MoveOptions, map[string]interface{}) error {
		return errors.New("protective stop")
	}

	_, err = doStep(r)
	test.That(t, err, test.ShouldNotBeNil)
	st := status(t, r)
	test.That(t, st["state"], test.ShouldEqual, "failed")
	test.That(t, st["step"], test.ShouldEqual, "fold-l-press")
	test.That(t, st["report"], test.ShouldEqual, "in_contact")

	r.arm.MoveThroughJointPositionsFunc = func(_ context.Context, p [][]referenceframe.Input, _ *arm.MoveOptions, _ map[string]interface{}) error {
		r.moveArm(p[len(p)-1])
		return nil
	}
	out, err := doStep(r)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, out["completed"], test.ShouldEqual, "preflight")
}
