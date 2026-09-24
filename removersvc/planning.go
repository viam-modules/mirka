package removersvc

import (
	"context"
	"fmt"
	"math"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// removerGeometryNames are the remover's geometries as the planner names them:
// <model name>:<link id>. model.json's label fields do not reach the planner.
// Allowing the remover component instead would allow every geometry it owns
// and let the pad drive through the body.
var removerGeometryNames = []string{
	"autochanger-remover:body",
	"autochanger-remover:blade",
	"autochanger-remover:head",
}

// Contact-step tolerances. The linear constraint makes wp3 to wp4 a chord of
// the tilt's arc; for a pad-centre distance of about 60 mm from the tip line
// the two differ by under 1 mm.
const (
	lineToleranceMM         = 1.0
	orientationToleranceDeg = 2.0
)

// Mirka pads are 77 to 150 mm across. Outside this range the tool frame's
// geometry is a placeholder, not a pad.
const (
	minPadRadiusMM = 30.0
	maxPadRadiusMM = 90.0
)

// originFrame is the remover's static mount frame. Waypoints are expressed in
// it, not in the remover frame, which rides the knife joint.
func originFrame(remover string) string {
	return remover + "_origin"
}

// prepareFrameSystem builds a frame system the way the frame-system service
// does and applies the service's own joint limits to it.
func prepareFrameSystem(
	parts []*referenceframe.FrameSystemPart, cfg *Config, logger logging.Logger,
) (*referenceframe.FrameSystem, error) {
	fs, err := referenceframe.NewFrameSystem("", parts, nil)
	if err != nil {
		return nil, err
	}
	for _, name := range []string{cfg.ToolFrame, cfg.Remover, cfg.Arm} {
		if fs.Frame(name) == nil {
			return nil, referenceframe.NewFrameMissingError(name)
		}
	}
	if err := applyJointLimits(logger, fs, cfg.InputRangeOverride); err != nil {
		return nil, fmt.Errorf("applying input_range_override: %w", err)
	}
	return fs, nil
}

// padRadius reads the pad from the tool frame's one geometry. A frame's
// configured geometry lands on its _origin frame.
func padRadius(fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs, toolFrame string) (float64, error) {
	all, err := referenceframe.FrameSystemGeometries(fs, inputs)
	if err != nil {
		return 0, err
	}
	var geoms []spatialmath.Geometry
	for _, name := range []string{toolFrame, toolFrame + "_origin"} {
		if gif, ok := all[name]; ok {
			geoms = append(geoms, gif.Geometries()...)
		}
	}
	if len(geoms) != 1 {
		return 0, fmt.Errorf("tool frame %q must carry exactly one geometry, the pad; found %d", toolFrame, len(geoms))
	}
	pb := geoms[0].ToProtobuf()
	var r float64
	switch {
	case pb.GetBox() != nil:
		d := pb.GetBox().GetDimsMm()
		r = math.Min(d.GetX(), d.GetY()) / 2
	case pb.GetCapsule() != nil:
		r = pb.GetCapsule().GetRadiusMm()
	default:
		return 0, fmt.Errorf("tool frame %q geometry must be a box or capsule to read the pad radius", toolFrame)
	}
	if r < minPadRadiusMM || r > maxPadRadiusMM {
		return 0, fmt.Errorf("tool frame %q pad radius %.1f mm is outside %v to %v mm; is the pad geometry configured?",
			toolFrame, r, minPadRadiusMM, maxPadRadiusMM)
	}
	return r, nil
}

// checkRemoverGeometries refuses when an allow entry would name nothing, which
// the planner reports far less legibly mid-cycle.
func checkRemoverGeometries(fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs) error {
	all, err := referenceframe.FrameSystemGeometries(fs, inputs)
	if err != nil {
		return err
	}
	have := map[string]bool{}
	for _, gif := range all {
		for _, g := range gif.Geometries() {
			have[g.Label()] = true
		}
	}
	for _, name := range removerGeometryNames {
		if !have[name] {
			return fmt.Errorf("frame system has no geometry %q; is the remover in the frame system with the autochanger-remover model?", name)
		}
	}
	return nil
}

// buildRequest builds one plan request for one arm-step goal, expressed as
// the tool frame's target pose in the remover's origin frame. contact steps
// add the linear-approach constraint and allow the pad to touch the remover's
// own geometries, which a free-motion step must still avoid.
func buildRequest(
	fs *referenceframe.FrameSystem,
	inputs referenceframe.FrameSystemInputs,
	obstacles *referenceframe.GeometriesInFrame,
	cfg *Config,
	goal spatialmath.Pose,
	contact bool,
) *armplanning.PlanRequest {
	constraints := motionplan.NewEmptyConstraints()
	if contact {
		constraints.AddLinearConstraint(motionplan.LinearConstraint{
			LineToleranceMm:          lineToleranceMM,
			OrientationToleranceDegs: orientationToleranceDeg,
		})
		allows := make([]motionplan.CollisionSpecificationAllowedFrameCollisions, 0, len(removerGeometryNames))
		for _, name := range removerGeometryNames {
			allows = append(allows, motionplan.CollisionSpecificationAllowedFrameCollisions{Frame1: cfg.ToolFrame, Frame2: name})
		}
		constraints.AddCollisionSpecification(motionplan.CollisionSpecification{Allows: allows})
	}
	return &armplanning.PlanRequest{
		FrameSystem:           fs,
		StartState:            armplanning.NewPlanState(nil, inputs),
		ObstaclesInWorldFrame: obstacles,
		Constraints:           constraints,
		Goals: []*armplanning.PlanState{armplanning.NewPlanState(referenceframe.FrameSystemPoses{
			cfg.ToolFrame: referenceframe.NewPoseInFrame(originFrame(cfg.Remover), goal),
		}, nil)},
	}
}

// planArmFunc plans one arm step and returns the arm's joint path. The service
// holds one so tests can stand in for the planner.
type planArmFunc func(ctx context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error)

func newPlanArm(logger logging.Logger, armName string) planArmFunc {
	return func(ctx context.Context, req *armplanning.PlanRequest) ([][]referenceframe.Input, error) {
		plan, _, err := armplanning.PlanMotion(ctx, logger, req)
		if err != nil {
			return nil, err
		}
		armFrame := req.FrameSystem.Frame(armName)
		path := make([][]referenceframe.Input, 0, len(plan.Trajectory()))
		for _, fsInputs := range plan.Trajectory() {
			in, err := fsInputs.GetFrameInputs(armFrame)
			if err != nil {
				return nil, fmt.Errorf("arm inputs from trajectory: %w", err)
			}
			path = append(path, in)
		}
		return path, nil
	}
}
