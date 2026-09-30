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

// knifeOffset is the knife joint position for k, in mm.
func (c *Config) knifeOffset(k knifeTarget) float64 {
	switch k {
	case knifeGrip:
		return c.GripOffsetMM
	case knifeRelease:
		return autochanger.KnifeTravelMM
	default:
		return 0
	}
}

// report is what a failure during a step leaves in the cell.
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
	// Arm steps.
	waypoint int
	contact  bool
	// Knife steps.
	knife  knifeTarget
	report report
}

// removalSteps is the manual's removal cycle; the knife only moves with the arm
// parked. wp5 is a contact step: it starts with the pad still over the knife.
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

// From the Mirka AutoChanger manual.
const (
	pressDepthMM  = 10.0 // the sprung plate's give
	liftOverlapMM = 15.0 // knife tip below the pad's bottom edge at wp3
	tiltBackDeg   = 20.0
)

// Left to the integrator; tuned on lab-sander-1.
const (
	approachStandoffMM = 5.0  // pad face off the plate at wp1
	belowTipMM         = 25.0 // pad top edge below the knife tip at wp1
	tiltBackoffMM      = 5.0  // wp4 eases off the plate so the tilt does not load the tool
	retreatMM          = 60.0 // wp5, along +X and +Z
)

// removerFeatures are the faces the waypoints are measured from.
type removerFeatures struct {
	plateFaceX float64 // the body's +X face: the sliding plate
	bladeTipZ  float64 // the blade's bottom edge: the knife tip
}

// readFeatures reads the features off the planner's own collision model, so
// waypoints and collisions cannot disagree.
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

// waypoints returns wp1..wp5 as tool-frame poses in the remover's origin frame;
// removersvc/README.md derives each one.
func waypoints(f removerFeatures, padRadiusMM float64) [5]spatialmath.Pose {
	facePlate := &spatialmath.OrientationVectorDegrees{OX: -1}
	belowZ := f.bladeTipZ - belowTipMM - padRadiusMM
	pressedX := f.plateFaceX - pressDepthMM

	wp1 := spatialmath.NewPose(r3.Vector{X: f.plateFaceX + approachStandoffMM, Z: belowZ}, facePlate)
	wp2 := spatialmath.NewPose(r3.Vector{X: pressedX, Z: belowZ}, facePlate)
	// The whole pad rides up past the knife, between disc and pad.
	padBottomZ := f.bladeTipZ + liftOverlapMM
	wp3 := spatialmath.NewPose(r3.Vector{X: pressedX, Z: padBottomZ + padRadiusMM}, facePlate)

	// Pivoting on the pad's bottom edge swings its top off the plate. A positive
	// angle about +Y moves points above the pivot toward +X.
	pivot := spatialmath.NewPoseFromPoint(r3.Vector{X: pressedX, Z: padBottomZ})
	tilt := spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{Theta: tiltBackDeg * math.Pi / 180, RY: 1})
	tilted := spatialmath.Compose(pivot, spatialmath.Compose(tilt, spatialmath.Compose(spatialmath.PoseInverse(pivot), wp3)))
	wp4 := spatialmath.Compose(spatialmath.NewPoseFromPoint(r3.Vector{X: tiltBackoffMM}), tilted)

	wp5 := spatialmath.Compose(spatialmath.NewPoseFromPoint(r3.Vector{X: retreatMM, Z: retreatMM}), wp4)
	return [5]spatialmath.Pose{wp1, wp2, wp3, wp4, wp5}
}
