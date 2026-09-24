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

// Contact-step tolerances. Every contact step holds the pad within 1 mm of the
// straight line between waypoints; wp3 to wp4 is a chord of the tilt's arc,
// and for a pad-centre distance of about 60 mm from the tip line the two
// differ by under 1 mm.
//
// Orientation is held to a fixed 2 degrees where the step does not rotate.
// A rotating step (wp4's 20 degree tilt) cannot use a fixed tolerance: the
// planner accepts a pose only within the tolerance of one end of the segment
// it checks, and a slerped midpoint is half the segment's rotation from both.
// There the tolerance is a fraction of the segment's own rotation instead,
// which admits the midpoint at any segment length. armplanning v1.9.0 splits a
// linear-constrained move into subgoals of about 2 degrees and checks each
// against its own ends, which a fixed 2 degrees happens to pass; the scaled
// tolerance does not depend on that split.
const (
	lineToleranceMM           = 1.0
	orientationToleranceDeg   = 2.0
	rotatingOrientationFactor = 0.6
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

// Tolerances on the tool-frame contract padRadius checks.
const (
	padContractMM  = 1.0
	padContractDeg = 1.0
)

// padRadius reads the pad from the tool frame's one geometry and checks the
// contract the waypoints are built on: the tool frame's origin is the pad
// face's centre and its +Z the outward pad normal, with the pad behind the
// face. A tool frame 30 mm above the face would otherwise press 40 mm at wp2.
// A frame's configured geometry lands on its _origin frame.
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
	// FrameSystemGeometries answers in world; the contract is in the tool frame.
	tf, err := fs.Transform(inputs.ToLinearInputs(), referenceframe.NewZeroPoseInFrame(toolFrame), referenceframe.World)
	if err != nil {
		return 0, err
	}
	pad := geoms[0].Transform(spatialmath.PoseInverse(tf.(*referenceframe.PoseInFrame).Pose()))
	pb := pad.ToProtobuf()
	var r, halfZ float64
	switch {
	case pb.GetBox() != nil:
		d := pb.GetBox().GetDimsMm()
		r = math.Min(d.GetX(), d.GetY()) / 2
		halfZ = d.GetZ() / 2
	case pb.GetCapsule() != nil:
		r = pb.GetCapsule().GetRadiusMm()
		halfZ = pb.GetCapsule().GetLengthMm() / 2
	default:
		return 0, fmt.Errorf("tool frame %q geometry must be a box or capsule to read the pad radius", toolFrame)
	}
	if r < minPadRadiusMM || r > maxPadRadiusMM {
		return 0, fmt.Errorf("tool frame %q pad radius %.1f mm is outside %v to %v mm; is the pad geometry configured?",
			toolFrame, r, minPadRadiusMM, maxPadRadiusMM)
	}
	if deg := motionplan.OrientDist(pad.Pose().Orientation(), spatialmath.NewZeroOrientation()); deg > padContractDeg {
		return 0, fmt.Errorf("tool frame %q pad geometry is rotated %.1f degrees in the tool frame; "+
			"the tool frame's +Z must be the pad normal", toolFrame, deg)
	}
	c := pad.Pose().Point()
	if math.Abs(c.X) > padContractMM || math.Abs(c.Y) > padContractMM {
		return 0, fmt.Errorf("tool frame %q pad geometry is centred at x=%.1f y=%.1f mm in the tool frame; "+
			"the tool frame's origin must be the pad face's centre", toolFrame, c.X, c.Y)
	}
	if face := c.Z + halfZ; math.Abs(face) > padContractMM {
		return 0, fmt.Errorf("tool frame %q pad face is at z=%.1f mm in the tool frame; "+
			"the tool frame's origin must be on the pad face, with the pad behind it along -Z", toolFrame, face)
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

// buildRequest builds one plan request for one arm-step goal, given as the
// tool frame's target pose in the remover's origin frame. contact steps add
// the linear-approach constraint and allow the pad to touch the remover's own
// geometries, which a free-motion step must still avoid.
//
// The goal is handed to the planner in world: armplanning computes start poses
// in world and refuses a linear-constrained goal in any other frame ("frame
// mismatch world remover_origin").
func buildRequest(
	fs *referenceframe.FrameSystem,
	inputs referenceframe.FrameSystemInputs,
	obstacles *referenceframe.GeometriesInFrame,
	cfg *Config,
	prev, goal spatialmath.Pose,
	contact bool,
) (*armplanning.PlanRequest, error) {
	tf, err := fs.Transform(inputs.ToLinearInputs(),
		referenceframe.NewPoseInFrame(originFrame(cfg.Remover), goal), referenceframe.World)
	if err != nil {
		return nil, fmt.Errorf("goal into world: %w", err)
	}
	constraints := motionplan.NewEmptyConstraints()
	if contact {
		if motionplan.OrientDist(prev.Orientation(), goal.Orientation()) > orientationToleranceDeg {
			constraints.AddLinearConstraint(motionplan.LinearConstraint{LineToleranceMm: lineToleranceMM})
			constraints.AddPseudolinearConstraint(motionplan.PseudolinearConstraint{
				OrientationToleranceFactor: rotatingOrientationFactor,
			})
		} else {
			constraints.AddLinearConstraint(motionplan.LinearConstraint{
				LineToleranceMm:          lineToleranceMM,
				OrientationToleranceDegs: orientationToleranceDeg,
			})
		}
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
			cfg.ToolFrame: tf.(*referenceframe.PoseInFrame),
		}, nil)},
	}, nil
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
