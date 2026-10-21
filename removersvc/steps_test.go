package removersvc

import (
	"math"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-modules/mirka/autochanger"
)

func TestRemovalStepOrder(t *testing.T) {
	type row struct {
		name    string
		kind    stepKind
		contact bool
		knife   knifeTarget
		report  report
	}
	want := []row{
		{"grip", knifeStep, false, knifeGrip, reportArmDisplaced},
		{"wp1", armStep, false, 0, reportArmDisplaced},
		{"wp2", armStep, true, 0, reportInContact},
		{"wp3", armStep, true, 0, reportInContact},
		{"clamp", knifeStep, false, knifeFlush, reportInContact},
		{"wp4", armStep, true, 0, reportInContact},
		{"wp5", armStep, true, 0, reportInContact},
		{"release", knifeStep, false, knifeRelease, reportKnifeHolding},
		{"regrip", knifeStep, false, knifeGrip, reportKnifeHolding},
	}
	test.That(t, len(removalSteps), test.ShouldEqual, len(want))
	wp := 0
	for i, w := range want {
		s := removalSteps[i]
		test.That(t, s.name, test.ShouldEqual, w.name)
		test.That(t, s.kind, test.ShouldEqual, w.kind)
		test.That(t, s.contact, test.ShouldEqual, w.contact)
		test.That(t, s.report, test.ShouldEqual, w.report)
		if s.kind == knifeStep {
			test.That(t, s.knife, test.ShouldEqual, w.knife)
		} else {
			test.That(t, s.waypoint, test.ShouldEqual, wp)
			wp++
		}
	}
}

func TestReadFeatures(t *testing.T) {
	f, err := readFeatures()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, f.plateFaceX, test.ShouldAlmostEqual, 0, 1e-9)
	test.That(t, f.bladeTipZ, test.ShouldAlmostEqual, -260, 1e-9)
	test.That(t, f.headTopZ, test.ShouldAlmostEqual, 0, 1e-9)
	test.That(t, f.headBottomZ, test.ShouldAlmostEqual, -68.4-87*math.Sin(bumperSlopeDeg*math.Pi/180), 1e-3)
	test.That(t, f.headFrontX, test.ShouldAlmostEqual, 22, 1e-9)
}

// padAt places the fixtures' pad at a tool-frame pose.
func padAt(t *testing.T, pose spatialmath.Pose) spatialmath.Geometry {
	t.Helper()
	pad, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: -10}),
		r3.Vector{X: 2 * testPadRadiusMM, Y: 2 * testPadRadiusMM, Z: 20}, "pad")
	test.That(t, err, test.ShouldBeNil)
	return pad.Transform(pose)
}

func removerGeometries(t *testing.T, knifeMM float64) map[string]spatialmath.Geometry {
	t.Helper()
	m, err := autochanger.KinematicModel()
	test.That(t, err, test.ShouldBeNil)
	gif, err := m.Geometries([]referenceframe.Input{knifeMM})
	test.That(t, err, test.ShouldBeNil)
	out := map[string]spatialmath.Geometry{}
	for _, g := range gif.Geometries() {
		out[g.Label()] = g
	}
	return out
}

func collidesWith(t *testing.T, pad spatialmath.Geometry, geoms map[string]spatialmath.Geometry) []string {
	t.Helper()
	var hit []string
	for name, g := range geoms {
		c, _, err := pad.CollidesWith(g, 0)
		test.That(t, err, test.ShouldBeNil)
		if c {
			hit = append(hit, name)
		}
	}
	return hit
}

func toolZ(p spatialmath.Pose) r3.Vector {
	return spatialmath.Compose(p, spatialmath.NewPoseFromPoint(r3.Vector{Z: 1})).Point().Sub(p.Point())
}

func TestWaypointsFollowTheSteps(t *testing.T) {
	f, err := readFeatures()
	test.That(t, err, test.ShouldBeNil)
	wps := waypoints(f, testPadRadiusMM)
	arm := 0
	for _, s := range removalSteps {
		if s.kind == armStep {
			arm++
		}
	}
	test.That(t, wps, test.ShouldHaveLength, arm)
	rm := removalWaypoints(f, testPadRadiusMM)
	test.That(t, wps, test.ShouldResemble, rm[:])
}

// The bumper's V as touched off on lab-sander-1: the flat top below the head's
// top, half its width, and each slope from horizontal.
const (
	bumperFlatDropMM = 68.4
	bumperFlatHalfMM = 20.0
	bumperSlopeDeg   = 40.0
)

// The model's bumper traces the measured V: a point half a millimetre inside
// the V is free, half a millimetre past its face is in the head.
func TestModelTracesTheBumperV(t *testing.T) {
	f, err := readFeatures()
	test.That(t, err, test.ShouldBeNil)
	head := map[string]spatialmath.Geometry{}
	for name, g := range removerGeometries(t, 0) {
		if strings.Contains(name, ":head") {
			head[name] = g
		}
	}
	test.That(t, head, test.ShouldHaveLength, 3)
	probe := func(p r3.Vector) []string {
		s, err := spatialmath.NewSphere(spatialmath.NewPoseFromPoint(r3.Vector{X: f.headFrontX - 10, Y: p.Y, Z: p.Z}), 0.1, "")
		test.That(t, err, test.ShouldBeNil)
		return collidesWith(t, s, head)
	}
	flatZ := f.headTopZ - bumperFlatDropMM
	for _, y := range []float64{-15, 0, 15} {
		test.That(t, probe(r3.Vector{Y: y, Z: flatZ - 0.5}), test.ShouldBeEmpty)
		test.That(t, probe(r3.Vector{Y: y, Z: flatZ + 0.5}), test.ShouldNotBeEmpty)
	}
	th := bumperSlopeDeg * math.Pi / 180
	for _, side := range []float64{1, -1} {
		into := r3.Vector{Y: side * math.Sin(th), Z: math.Cos(th)}
		for _, along := range []float64{5, 40, 75} {
			p := r3.Vector{Y: side * (bumperFlatHalfMM + along*math.Cos(th)), Z: flatZ - along*math.Sin(th)}
			test.That(t, probe(p.Sub(into.Mul(0.5))), test.ShouldBeEmpty)
			test.That(t, probe(p.Add(into.Mul(0.5))), test.ShouldNotBeEmpty)
		}
	}
}

func TestWaypoints(t *testing.T) {
	f, err := readFeatures()
	test.That(t, err, test.ShouldBeNil)
	wps := removalWaypoints(f, testPadRadiusMM)
	grip := removerGeometries(t, 0.7)
	flush := removerGeometries(t, 0)

	t.Run("wp1 is clear of the remover, pad top below the tip", func(t *testing.T) {
		test.That(t, collidesWith(t, padAt(t, wps[0]), grip), test.ShouldBeEmpty)
		test.That(t, wps[0].Point().Z+testPadRadiusMM, test.ShouldBeLessThan, f.bladeTipZ)
	})
	t.Run("pad faces the plate through wp3", func(t *testing.T) {
		for _, i := range []int{0, 1, 2} {
			test.That(t, toolZ(wps[i]).X, test.ShouldAlmostEqual, -1, 1e-9)
		}
	})
	t.Run("wp2 presses 10 mm into the plate along -X", func(t *testing.T) {
		d := wps[1].Point().Sub(wps[0].Point())
		test.That(t, d.Y, test.ShouldAlmostEqual, 0)
		test.That(t, d.Z, test.ShouldAlmostEqual, 0)
		test.That(t, wps[1].Point().X, test.ShouldAlmostEqual, f.plateFaceX-pressDepthMM, 1e-9)
	})
	t.Run("wp3 lifts straight up until the tip shows 15 mm under the pad", func(t *testing.T) {
		d := wps[2].Point().Sub(wps[1].Point())
		test.That(t, d.X, test.ShouldAlmostEqual, 0)
		test.That(t, d.Y, test.ShouldAlmostEqual, 0)
		test.That(t, wps[2].Point().Z-testPadRadiusMM, test.ShouldAlmostEqual, f.bladeTipZ+liftOverlapMM, 1e-9)
		// So wp3 needs the contact allow list.
		test.That(t, collidesWith(t, padAt(t, wps[2]), grip), test.ShouldContain, "autochanger-remover:blade")
	})
	t.Run("wp4 tilts 20 degrees about the pad's bottom edge, top away from the plate, and backs off", func(t *testing.T) {
		theta := spatialmath.OrientationBetween(wps[2].Orientation(), wps[3].Orientation()).AxisAngles().Theta
		test.That(t, theta*180/math.Pi, test.ShouldAlmostEqual, tiltBackDeg, 1e-6)
		pivot := r3.Vector{X: f.plateFaceX - pressDepthMM, Z: f.bladeTipZ + liftOverlapMM}
		tilted := wps[3].Point().Sub(r3.Vector{X: tiltBackoffMM})
		test.That(t, tilted.Distance(pivot), test.ShouldAlmostEqual, wps[2].Point().Distance(pivot), 1e-6)
		// The centre is above the pivot, so it moves off the plate.
		test.That(t, wps[3].Point().X, test.ShouldBeGreaterThan, wps[2].Point().X)
		test.That(t, toolZ(wps[3]).Z, test.ShouldBeGreaterThan, 0)
	})
	t.Run("wp5 goes back and up and is clear with the knife flush", func(t *testing.T) {
		d := wps[4].Point().Sub(wps[3].Point())
		test.That(t, d.X, test.ShouldBeGreaterThan, 0)
		test.That(t, d.Z, test.ShouldBeGreaterThan, 0)
		test.That(t, spatialmath.OrientationAlmostEqual(wps[4].Orientation(), wps[3].Orientation()), test.ShouldBeTrue)
		test.That(t, collidesWith(t, padAt(t, wps[4]), flush), test.ShouldBeEmpty)
	})
}
