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

// removalSteps is the manual's removal cycle, steps 3 to 8; the knife only
// moves with the arm parked. The bumper steps are left out: step 1's open-loop
// press into the rigid bumper loaded the arm's force-torque sensor by however
// the disc happened to sit, and step 2's edge fold pressed the same bumper.
// wp5 is a contact step: it starts with the pad still over the knife.
var removalSteps = []step{
	{name: "grip", kind: knifeStep, knife: knifeGrip, report: reportArmDisplaced},
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
	plateFaceX  float64 // the body's +X face: the sliding plate
	bladeTipZ   float64 // the blade's bottom edge: the knife tip
	headTopZ    float64 // the bumper's top
	headBottomZ float64 // its lowest point, a corner
	headFrontX  float64 // with the knife flush
}

// headLinks trace the bumper.
var headLinks = []string{"head", "head-left", "head-right"}

// readFeatures reads the features off the planner's own collision model, so
// waypoints and collisions cannot disagree. The head rides the knife; it is
// read flush.
func readFeatures() (removerFeatures, error) {
	m, err := autochanger.KinematicModel()
	if err != nil {
		return removerFeatures{}, err
	}
	gif, err := m.Geometries([]referenceframe.Input{0})
	if err != nil {
		return removerFeatures{}, fmt.Errorf("remover geometries: %w", err)
	}
	boxes := map[string]*extent{}
	for _, g := range gif.Geometries() {
		box := g.ToProtobuf().GetBox()
		if box == nil {
			continue
		}
		// The bumper's slope boxes are turned, so bound every box by its corners.
		e := newExtent()
		for _, sx := range []float64{-1, 1} {
			for _, sy := range []float64{-1, 1} {
				for _, sz := range []float64{-1, 1} {
					local := r3.Vector{X: sx * box.GetDimsMm().GetX(), Y: sy * box.GetDimsMm().GetY(), Z: sz * box.GetDimsMm().GetZ()}.Mul(0.5)
					e.add(spatialmath.Compose(g.Pose(), spatialmath.NewPoseFromPoint(local)).Point())
				}
			}
		}
		boxes[g.Label()[strings.LastIndex(g.Label(), ":")+1:]] = e
	}
	body, okBody := boxes["body"]
	blade, okBlade := boxes["blade"]
	if !okBody || !okBlade {
		return removerFeatures{}, fmt.Errorf("remover model is missing its body or blade box; have %v", boxes)
	}
	head := newExtent()
	for _, link := range headLinks {
		e, ok := boxes[link]
		if !ok {
			return removerFeatures{}, fmt.Errorf("remover model is missing its %s box; have %v", link, boxes)
		}
		head.add(e.min)
		head.add(e.max)
	}
	return removerFeatures{
		plateFaceX: body.max.X, bladeTipZ: blade.min.Z,
		headTopZ: head.max.Z, headBottomZ: head.min.Z, headFrontX: head.max.X,
	}, nil
}

type extent struct{ min, max r3.Vector }

func newExtent() *extent {
	inf := math.Inf(1)
	return &extent{min: r3.Vector{X: inf, Y: inf, Z: inf}, max: r3.Vector{X: -inf, Y: -inf, Z: -inf}}
}

func (e *extent) add(p r3.Vector) {
	e.min = r3.Vector{X: math.Min(e.min.X, p.X), Y: math.Min(e.min.Y, p.Y), Z: math.Min(e.min.Z, p.Z)}
	e.max = r3.Vector{X: math.Max(e.max.X, p.X), Y: math.Max(e.max.Y, p.Y), Z: math.Max(e.max.Z, p.Z)}
}

var facePlate = &spatialmath.OrientationVectorDegrees{OX: -1}

// waypoints returns every arm step's target, indexed by step.waypoint, as
// tool-frame poses in the remover's origin frame; removersvc/README.md derives
// each one.
func waypoints(f removerFeatures, padRadiusMM float64) []spatialmath.Pose {
	r := removalWaypoints(f, padRadiusMM)
	return r[:]
}

// removalWaypoints returns wp1..wp5.
func removalWaypoints(f removerFeatures, padRadiusMM float64) [5]spatialmath.Pose {
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
