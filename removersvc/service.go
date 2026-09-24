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

// Model is the remover service's model triplet, viam:mirka:autochanger-remover-svc.
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

// stopTimeout bounds the hardware calls made after a remove has stopped: the
// actuator stops after a cancel, and the reads a failure report takes. Both
// run on a context detached from the remove's, which may already be done.
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

	// Cancelled by Close, which ends a running remove: a rebuilt service must
	// not share the arm with an old instance still driving it.
	closeCtx context.Context
	cancel   context.CancelFunc
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
	closeCtx, cancel := context.WithCancel(context.Background())
	return &service{
		Named: name.AsNamed(), cfg: cfg, arm: a, remover: remover, mirka: mirka,
		visions: visions, fsSvc: fsSvc, planArm: planArm, logger: logger,
		st:       cycleState{state: "idle", stepIndex: -1},
		closeCtx: closeCtx, cancel: cancel,
	}
}

// Close cancels a running remove, whose cancel path stops both actuators, and
// returns once it has finished. Safe to call more than once.
func (s *service) Close(context.Context) error {
	s.cancel()
	s.cycle.Lock()
	defer s.cycle.Unlock()
	return nil
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
	if s.closeCtx.Err() != nil {
		return nil, errors.New("service is closed")
	}
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	defer context.AfterFunc(s.closeCtx, cancel)()

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
		s.failCycle(ctx, -1, "preflight", reportNotStarted, err, p)
		return nil, err
	}
	for i, st := range removalSteps {
		s.mu.Lock()
		s.st.step, s.st.stepIndex = st.name, i
		s.mu.Unlock()
		if err := s.runStep(ctx, p, st); err != nil {
			err = fmt.Errorf("remove step %s: %w", st.name, err)
			s.failCycle(ctx, i, st.name, st.report, err, p)
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
		prev := p.waypoints[max(st.waypoint-1, 0)]
		req, err := buildRequest(p.fs, inputs, p.obstacles, s.cfg, prev, p.waypoints[st.waypoint], st.contact)
		if err != nil {
			return err
		}
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

// failCycle stops both actuators on a cancelled remove before fail reads the
// hardware, so a step's own cancellation (including the knife move inside
// preflight) always reaches Stop rather than only the step-loop's.
func (s *service) failCycle(ctx context.Context, index int, name string, rep report, cause error, p *prepared) {
	if ctx.Err() != nil {
		s.stopActuators(ctx)
	}
	s.fail(ctx, index, name, rep, cause, p)
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
	// wp5 only means something once the arm has moved toward the changer;
	// a preflight refusal leaves p.waypoints unset.
	if rep != reportNotStarted && p != nil && p.fs != nil {
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
