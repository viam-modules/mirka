package autochanger

import (
	"math"
	"strings"
	"testing"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

func TestModelHasOnePrismaticAxis(t *testing.T) {
	m, err := KinematicModel()
	if err != nil {
		t.Fatalf("KinematicModel: %v", err)
	}
	dof := m.DoF()
	if len(dof) != 1 {
		t.Fatalf("got %d degrees of freedom, want 1", len(dof))
	}
	if dof[0].Min != 0 || math.Abs(dof[0].Max-KnifeTravelMM) > 0.001 {
		t.Fatalf("joint limits %v, want [0, %v]", dof[0], KnifeTravelMM)
	}
}

func modelGeoms(t *testing.T, input float64) map[string]spatialmath.Geometry {
	t.Helper()
	m, err := KinematicModel()
	if err != nil {
		t.Fatalf("KinematicModel: %v", err)
	}
	gif, err := m.Geometries([]referenceframe.Input{input})
	if err != nil {
		t.Fatalf("Geometries: %v", err)
	}
	out := map[string]spatialmath.Geometry{}
	for _, g := range gif.Geometries() {
		// Labels come back namespaced by the model, e.g. "remover:blade".
		name := g.Label()
		if i := strings.LastIndex(name, ":"); i >= 0 {
			name = name[i+1:]
		}
		out[name] = g
	}
	return out
}

func TestModelCarriesThreeGeometries(t *testing.T) {
	// The whole reason this component is a gantry: a LinkInFrame holds one
	// geometry, a kinematic model holds one per link.
	g := modelGeoms(t, 0)
	if len(g) != 3 {
		t.Fatalf("got %d geometries, want 3: %v", len(g), g)
	}
	for _, want := range []string{"body", "blade", "head"} {
		if _, ok := g[want]; !ok {
			t.Fatalf("missing link %q, got %v", want, g)
		}
	}
}

func TestBladeAndHeadRideTheJoint(t *testing.T) {
	zero := modelGeoms(t, 0)
	out := modelGeoms(t, KnifeTravelMM)

	for _, link := range []string{"blade", "head"} {
		moved := out[link].Pose().Point().X - zero[link].Pose().Point().X
		if math.Abs(moved-KnifeTravelMM) > 0.001 {
			t.Fatalf("%s moved %v mm, want %v", link, moved, KnifeTravelMM)
		}
	}
	if moved := out["body"].Pose().Point().X - zero["body"].Pose().Point().X; math.Abs(moved) > 0.001 {
		t.Fatalf("body moved %v mm, want 0", moved)
	}
}

func TestCommandForOffset(t *testing.T) {
	// The ends route to the mechanical end stops rather than asking the fitted
	// calibration to find a hard stop. Everything between is a taught
	// intermediate, whose target is a gap -- PosImpOffsetMM above the joint
	// displacement, because the knife stands that far proud of the plate at its
	// own zero.
	for _, tc := range []struct {
		offset  float64
		pos     Position
		refused bool
	}{
		{offset: 0, pos: PositionIn},
		{offset: KnifeTravelMM, pos: PositionOut},
		{offset: 0.68, pos: PositionIntermediate},
		{offset: 10, pos: PositionIntermediate},
		{offset: 20, pos: PositionIntermediate},
		{offset: -1, refused: true},
		{offset: KnifeTravelMM + 1, refused: true},
	} {
		pos, err := commandForOffset(tc.offset)
		if tc.refused {
			if err == nil || !strings.Contains(err.Error(), "out of reach") {
				t.Fatalf("commandForOffset(%v) should be out of reach, got %v", tc.offset, err)
			}
			continue
		}
		if err != nil {
			t.Fatalf("commandForOffset(%v): %v", tc.offset, err)
		}
		if pos != tc.pos {
			t.Fatalf("commandForOffset(%v) = %v, want %v", tc.offset, pos, tc.pos)
		}
	}
}

func TestGeometriesAreAnchoredToTheTopFace(t *testing.T) {
	// Every link hangs below z=0, and the body sits behind the plate face. The
	// origin is anchored to features a probe can touch, so re-measuring a
	// dimension changes a box's size and never the mounting frame.
	tops := map[string]float64{"body": 225, "blade": 130, "head": 60}
	for name, g := range modelGeoms(t, 0) {
		if top := g.Pose().Point().Z + tops[name]; math.Abs(top) > 0.001 {
			t.Fatalf("%s top face at z=%v, want 0", name, top)
		}
	}
	if x := modelGeoms(t, 0)["body"].Pose().Point().X; x >= 0 {
		t.Fatalf("body centre at %v, want behind the plate face", x)
	}
}
