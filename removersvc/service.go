package removersvc

import (
	"context"
	"errors"
	"fmt"
	"maps"
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
)

// Model is viam:mirka:autochanger-remover-svc.
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

// startToleranceRad is how far an arm joint may sit from a stored path's start:
// about 2 mm at the pad.
const startToleranceRad = 0.01

// stopTimeout bounds the stops and reads after a remove ends, which run on a
// detached context.
const stopTimeout = 5 * time.Second

type cycleState struct {
	state     string // idle, running, failed
	step      string
	stepIndex int
	err       error
	report    report
	latched   bool
	// Nil when unreadable.
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

	// Held for a whole cycle; TryLock refuses a second caller rather than queue it.
	cycle sync.Mutex
	mu    sync.Mutex // guards st
	st    cycleState

	// Whether the tool frame is at waypoint i; a field so tests can fake it.
	atWaypoint func(ctx context.Context, p *prepared, i int) (bool, error)

	// Cancelled by Close, so a rebuilt service never shares the arm with this one.
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
	s := &service{
		Named: name.AsNamed(), cfg: cfg, arm: a, remover: remover, mirka: mirka,
		visions: visions, fsSvc: fsSvc, planArm: planArm, logger: logger,
		st:       cycleState{state: "idle", stepIndex: -1},
		closeCtx: closeCtx, cancel: cancel,
	}
	s.atWaypoint = s.armAtWaypoint
	return s
}

// Close cancels a running remove and returns once it has stopped.
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

// reset records that a human has recovered the cell, clearing a latched
// failure. It moves and reads nothing.
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
	// Remover geometries contact steps may touch.
	removerGeoms []string
	waypoints    [5]spatialmath.Pose
	// Joint path to each waypoint, planned at preflight.
	paths [5][][]referenceframe.Input
}

// remove runs the whole cycle: preflight, which plans every arm step, then
// each step's stored path in order.
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
	s.mu.Unlock()

	s.setRunning("preflight", -1)
	p, err := s.preflight(ctx)
	if err != nil {
		s.failCycle(ctx, -1, "preflight", reportNotStarted, err, p)
		return nil, err
	}
	for i, st := range removalSteps {
		s.setRunning(st.name, i)
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

// armAtJoints refuses unless every arm joint is within startToleranceRad of want.
func (s *service) armAtJoints(ctx context.Context, want []referenceframe.Input) error {
	inputs, err := s.fsSvc.CurrentInputs(ctx)
	if err != nil {
		return fmt.Errorf("reading current inputs: %w", err)
	}
	now := inputs[s.cfg.Arm]
	if len(now) != len(want) {
		return fmt.Errorf("arm reports %d joints, the plan has %d", len(now), len(want))
	}
	for j := range want {
		if d := math.Abs(now[j] - want[j]); d > startToleranceRad {
			return fmt.Errorf("joint %d is %.3f rad off", j, d)
		}
	}
	return nil
}

func (s *service) setRunning(step string, index int) {
	s.mu.Lock()
	s.st = cycleState{state: "running", step: step, stepIndex: index}
	s.mu.Unlock()
}

// preflight plans the whole cycle before anything moves, so an unplannable one
// fails here rather than in contact. The knife move to grip doubles as the
// drive readiness check.
func (s *service) preflight(ctx context.Context) (*prepared, error) {
	// A moving arm would make the plan's start stale.
	if err := s.armParked(ctx); err != nil {
		return nil, err
	}
	p, inputs, err := s.prepare(ctx)
	if err != nil {
		return p, err
	}
	if err := s.planCycle(ctx, p, inputs); err != nil {
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

// prepare is preflight's read-only half.
func (s *service) prepare(ctx context.Context) (*prepared, referenceframe.FrameSystemInputs, error) {
	obstacles, err := s.obstacles(ctx)
	if err != nil {
		return nil, nil, err
	}
	fsCfg, err := s.fsSvc.FrameSystemConfig(ctx)
	if err != nil {
		return nil, nil, fmt.Errorf("reading the frame system: %w", err)
	}
	fs, err := prepareFrameSystem(fsCfg.Parts, s.cfg, s.logger)
	if err != nil {
		return nil, nil, err
	}
	s.logger.CInfow(ctx, "planning remove with joint limits", "input_range_override", s.cfg.InputRangeOverride)
	p := &prepared{fs: fs, obstacles: obstacles}

	inputs, err := s.fsSvc.CurrentInputs(ctx)
	if err != nil {
		return p, nil, fmt.Errorf("reading current inputs: %w", err)
	}
	radius, err := padRadius(fs, inputs, s.cfg.ToolFrame)
	if err != nil {
		return p, nil, err
	}
	if p.removerGeoms, err = removerGeometryNames(fs, inputs, s.cfg.Remover); err != nil {
		return p, nil, err
	}
	features, err := readFeatures(s.cfg.GripOffsetMM)
	if err != nil {
		return p, nil, err
	}
	p.waypoints = waypoints(features, radius)
	return p, inputs, nil
}

// planCycle plans every arm step into p.paths, chaining each from the last and
// setting the knife joint as the cycle will.
func (s *service) planCycle(
	ctx context.Context, p *prepared, current referenceframe.FrameSystemInputs,
) error {
	inputs := maps.Clone(current)
	inputs[s.cfg.Remover] = []referenceframe.Input{s.cfg.knifeOffset(knifeGrip)}
	for _, st := range removalSteps {
		if st.kind == knifeStep {
			inputs[s.cfg.Remover] = []referenceframe.Input{s.cfg.knifeOffset(st.knife)}
			continue
		}
		prev := p.waypoints[max(st.waypoint-1, 0)]
		req, err := buildRequest(p.fs, maps.Clone(inputs), p.obstacles, p.removerGeoms, s.cfg, prev, p.waypoints[st.waypoint], st.contact)
		if err != nil {
			return fmt.Errorf("planning %s: %w", st.name, err)
		}
		path, err := s.planArm(ctx, req)
		if err != nil {
			return fmt.Errorf("planning %s: %w", st.name, err)
		}
		if len(path) == 0 {
			return fmt.Errorf("planning %s: empty path", st.name)
		}
		p.paths[st.waypoint] = path
		inputs[s.cfg.Arm] = path[len(path)-1]
	}
	return nil
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
		return s.remover.MoveToPosition(ctx, []float64{s.cfg.knifeOffset(st.knife)}, nil, nil)
	case armStep:
		moving, err := s.remover.IsMoving(ctx)
		if err != nil {
			return fmt.Errorf("reading knife motion: %w", err)
		}
		if moving {
			return errors.New("knife is moving")
		}
		path := p.paths[st.waypoint]
		if err := s.armAtJoints(ctx, path[0]); err != nil {
			return fmt.Errorf("arm is not where the planned path starts: %w", err)
		}
		var opts *arm.MoveOptions
		if st.contact {
			opts = s.cfg.contactMoveOptions()
		}
		return s.arm.MoveThroughJointPositions(ctx, path, opts, nil)
	default:
		return fmt.Errorf("unknown step kind %d", st.kind)
	}
}

// stopActuators halts both actuators after a cancel, on a live context.
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

// failCycle stops both actuators on a cancel before fail reads the hardware.
func (s *service) failCycle(ctx context.Context, index int, name string, rep report, cause error, p *prepared) {
	if ctx.Err() != nil {
		s.stopActuators(ctx)
	}
	s.fail(ctx, index, name, rep, cause, p)
}

// fail records where the cycle stopped; unreadable fields stay unknown.
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
	// wp5 is meaningless before the arm has moved.
	if rep != reportNotStarted && p != nil && p.fs != nil {
		if at, err := s.atWaypoint(readCtx, p, 4); err == nil {
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
	switch st.state {
	case "idle":
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
