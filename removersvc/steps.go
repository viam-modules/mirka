package removersvc

import (
	"fmt"
	"math"
	"strings"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-modules/mirka/autochanger"
)

type stepKind int

const (
	armStep stepKind = iota
	knifeStep
)

type knifeTarget int

const (
	knifeGrip knifeTarget = iota
	knifeFlush
	knifeRelease
)

// report is what a failure during a step leaves in the cell (spec section 5).
type report string

const (
	reportNotStarted   report = "not_started"
	reportArmDisplaced report = "arm_displaced"
	reportInContact    report = "in_contact"
	reportKnifeHolding report = "knife_holding"
)

type step struct {
	name string
	kind stepKind
	// Arm steps: index into waypoints, and whether the pad may touch the remover.
	waypoint int
	contact  bool
	// Knife steps.
	knife  knifeTarget
	report report
}

// removalSteps is the removal cycle from the Mirka AutoChanger manual. Every
// knife step sits between arm steps, so the knife only moves with the arm
// parked. wp5 is a contact step because it starts from wp4, with the pad still
// over the knife.
var removalSteps = []step{
	{name: "wp1", kind: armStep, waypoint: 0, report: reportArmDisplaced},
	{name: "wp2", kind: armStep, waypoint: 1, contact: true, report: reportInContact},
	{name: "wp3", kind: armStep, waypoint: 2, contact: true, report: reportInContact},
	{name: "clamp", kind: knifeStep, knife: knifeFlush, report: reportInContact},
	{name: "wp4", kind: armStep, waypoint: 3, contact: true, report: reportInContact},
	{name: "wp5", kind: armStep, waypoint: 4, contact: true, report: reportInContact},
	{name: "release", kind: knifeStep, knife: knifeRelease, report: reportKnifeHolding},
	{name: "regrip", kind: knifeStep, knife: knifeGrip, report: reportKnifeHolding},
}

// From the Mirka AutoChanger manual's removal cycle. Each moves to config only
// if rig tuning shows it varies by disc.
const (
	pressDepthMM  = 10.0 // the spring-loaded front plate gives about 10 mm
	liftOverlapMM = 15.0 // knife tip 10 to 20 mm under the pad
	tiltBackDeg   = 20.0
)

// Approach and retreat distances the manual leaves to the integrator.
const (
	approachStandoffMM = 5.0  // pad face off the plate at wp1
	belowTipMM         = 10.0 // pad top edge below the knife tip at wp1
	retreatMM          = 60.0 // wp5, along +X and along +Z
)

// removerFeatures are the remover faces the waypoints are measured from.
type removerFeatures struct {
	plateFaceX float64 // the body's +X face: the sliding plate
	bladeTipZ  float64 // the blade's bottom edge: the knife tip
}

// readFeatures reads the features from the remover's kinematic model, the
// same geometry the planner checks collisions against, so the waypoints and
// the collision model cannot disagree.
func readFeatures(gripMM float64) (removerFeatures, error) {
	m, err := autochanger.KinematicModel()
	if err != nil {
		return removerFeatures{}, err
	}
	gif, err := m.Geometries([]referenceframe.Input{gripMM})
	if err != nil {
		return removerFeatures{}, fmt.Errorf("remover geometries: %w", err)
	}
	type extent struct{ min, max r3.Vector }
	boxes := map[string]extent{}
	for _, g := range gif.Geometries() {
		box := g.ToProtobuf().GetBox()
		if box == nil {
			continue
		}
		// The model's boxes are axis-aligned in the remover's origin frame.
		half := r3.Vector{X: box.GetDimsMm().GetX(), Y: box.GetDimsMm().GetY(), Z: box.GetDimsMm().GetZ()}.Mul(0.5)
		c := g.Pose().Point()
		link := g.Label()[strings.LastIndex(g.Label(), ":")+1:]
		boxes[link] = extent{min: c.Sub(half), max: c.Add(half)}
	}
	body, okBody := boxes["body"]
	blade, okBlade := boxes["blade"]
	if !okBody || !okBlade {
		return removerFeatures{}, fmt.Errorf("remover model is missing its body or blade box; have %v", boxes)
	}
	return removerFeatures{plateFaceX: body.max.X, bladeTipZ: blade.min.Z}, nil
}

// waypoints returns wp1..wp5 as tool-frame poses in the remover's origin frame:
// plate front face, top, centred in Y; +X knife extension, +Z up. The tool
// frame's +Z is the pad normal, out of the pad face, so facing the plate is
// tool +Z along -X. The pad works from below the knife tip.
func waypoints(f removerFeatures, padRadiusMM float64) [5]spatialmath.Pose {
	facePlate := &spatialmath.OrientationVectorDegrees{OX: -1}
	belowZ := f.bladeTipZ - belowTipMM - padRadiusMM
	pressedX := f.plateFaceX - pressDepthMM

	wp1 := spatialmath.NewPose(r3.Vector{X: f.plateFaceX + approachStandoffMM, Z: belowZ}, facePlate)
	wp2 := spatialmath.NewPose(r3.Vector{X: pressedX, Z: belowZ}, facePlate)
	wp3 := spatialmath.NewPose(r3.Vector{X: pressedX, Z: f.bladeTipZ + liftOverlapMM - padRadiusMM}, facePlate)

	// Tilting back about the tip line swings the pad's lower half off the plate
	// while the disc edge stays pinned under the knife. A negative angle about
	// +Y moves points below the pivot toward +X.
	pivot := spatialmath.NewPoseFromPoint(r3.Vector{X: f.plateFaceX, Z: f.bladeTipZ})
	tilt := spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{Theta: -tiltBackDeg * math.Pi / 180, RY: 1})
	wp4 := spatialmath.Compose(pivot, spatialmath.Compose(tilt, spatialmath.Compose(spatialmath.PoseInverse(pivot), wp3)))

	wp5 := spatialmath.Compose(spatialmath.NewPoseFromPoint(r3.Vector{X: retreatMM, Z: retreatMM}), wp4)
	return [5]spatialmath.Pose{wp1, wp2, wp3, wp4, wp5}
}
