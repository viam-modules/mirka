package removersvc

import (
	"context"
	"fmt"
	"math"
	"strings"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// removerLinks are the links the pad may touch in contact steps. Allowing the
// whole component would let the pad drive through the body.
var removerLinks = []string{"body", "blade", "head"}

// Contact-step tolerances. wp4's tilt is a chord of an arc of radius r, off by
// r(1 - cos 10 deg): within 1 mm up to a 120 mm pad, not for 150 mm.
//
// A rotating step scales its orientation tolerance with its own rotation: the
// planner checks a slerped midpoint against the segment's ends, and a fixed 2
// degrees would refuse it.
const (
	lineToleranceMM           = 1.0
	orientationToleranceDeg   = 2.0
	rotatingOrientationFactor = 0.6
)

// Mirka pads are 77 to 150 mm across; anything else is a placeholder geometry.
const (
	minPadRadiusMM = 30.0
	maxPadRadiusMM = 90.0
)

// originFrame is the remover's static mount frame; the remover frame itself
// rides the knife joint.
func originFrame(remover string) string {
	return remover + "_origin"
}

// prepareFrameSystem builds the frame system and applies the service's limits.
func prepareFrameSystem(
	parts []*referenceframe.FrameSystemPart, cfg *Config, logger logging.Logger,
) (*referenceframe.FrameSystem, error) {
	fs, err := referenceframe.NewFrameSystem("", parts, nil)
	if err != nil {
		return nil, err
	}
	for _, name := range append([]string{cfg.ToolFrame, cfg.Remover, cfg.Arm}, cfg.ContactFrames...) {
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

// padRadius reads the pad from the tool frame's one geometry, refusing one off
// the tool-frame contract. A frame's configured geometry lands on its _origin
// frame.
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

// removerGeometryNames returns the planner's labels for the remover's contact
// geometries, <model name>:<link id>. A module gets the frame system as a
// proto, and RDK renames a model rebuilt from one after its component, so the
// labels are read rather than assumed.
func removerGeometryNames(
	fs *referenceframe.FrameSystem, inputs referenceframe.FrameSystemInputs, remover string,
) ([]string, error) {
	f := fs.Frame(remover)
	if f == nil {
		return nil, referenceframe.NewFrameMissingError(remover)
	}
	gif, err := f.Geometries(inputs[remover])
	if err != nil {
		return nil, fmt.Errorf("remover geometries: %w", err)
	}
	byLink := map[string]string{}
	for _, g := range gif.Geometries() {
		label := g.Label()
		byLink[label[strings.LastIndex(label, ":")+1:]] = label
	}
	names := make([]string, 0, len(removerLinks))
	for _, link := range removerLinks {
		name, ok := byLink[link]
		if !ok {
			return nil, fmt.Errorf("remover frame %q has no %q geometry; is it the autochanger-remover model?", remover, link)
		}
		names = append(names, name)
	}
	return names, nil
}

// buildRequest builds the plan request to one waypoint. Contact steps add the
// line constraint and the remover allow list. The goal goes in world, since
// armplanning refuses a linear-constrained goal in any other frame.
func buildRequest(
	fs *referenceframe.FrameSystem,
	inputs referenceframe.FrameSystemInputs,
	obstacles *referenceframe.GeometriesInFrame,
	removerGeoms []string,
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
		touching := append([]string{cfg.ToolFrame}, cfg.ContactFrames...)
		allows := make([]motionplan.CollisionSpecificationAllowedFrameCollisions, 0, len(touching)*len(removerGeoms))
		for _, frame := range touching {
			for _, name := range removerGeoms {
				allows = append(allows, motionplan.CollisionSpecificationAllowedFrameCollisions{Frame1: frame, Frame2: name})
			}
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

// planArmFunc plans one arm step and returns the arm's joint path.
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
