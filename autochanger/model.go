package autochanger

import (
	_ "embed"
	"fmt"
	"sync"

	"go.viam.com/rdk/referenceframe"
)

// Envelope in millimetres, measured 2026-09-18 and -23. Not configurable:
// there is one remover in existence, and a wrong envelope does not error, it
// plans a path that clips something.
//
//	body   195 x 250 x 450   at (-97.5, 0, -225)
//	blade    2 x 180 x 260   at (    1, 0, -130)  rides the joint
//	head    20 x 180 x 120   at (   12, 0,  -60)  rides the blade
//
// The body is the widest part of the tool, not the blade.
//
// Origin: plate front face, centred in Y, on the top face body and blade share
// -- all three touchable, so the mounting frame can be calibrated by touching
// off with the arm, and re-measuring a dimension never moves the origin. +X is
// the direction the knife extends, +Z up, so every geometry hangs below z=0.
//
//go:embed model.json
var modelJSON []byte

var (
	modelOnce sync.Once
	modelVal  referenceframe.Model
	modelErr  error
)

// KinematicModel is why this is a gantry and not generic: a LinkInFrame holds
// one geometry, so a generic component's three would collapse to the first.
func KinematicModel() (referenceframe.Model, error) {
	modelOnce.Do(func() {
		modelVal, modelErr = referenceframe.UnmarshalModelJSON(modelJSON, "autochanger-remover")
	})
	if modelErr != nil {
		return nil, fmt.Errorf("parsing the remover kinematic model: %w", modelErr)
	}
	return modelVal, nil
}

// commandForOffset maps a joint displacement, in mm out from the retracted end,
// onto one of the drive's three commands.
func commandForOffset(offsetMM float64) (Position, error) {
	const toleranceMM = 0.05
	switch {
	case offsetMM < -toleranceMM || offsetMM > KnifeTravelMM+toleranceMM:
		return PositionUnknown, fmt.Errorf(
			"remover: %.3f mm is out of reach; the knife travels 0 to %.3f mm",
			offsetMM, KnifeTravelMM)
	case offsetMM <= toleranceMM:
		return PositionIn, nil
	case offsetMM >= KnifeTravelMM-toleranceMM:
		return PositionOut, nil
	default:
		return PositionIntermediate, nil
	}
}

// clampToTravel holds a reading inside the joint's limits. The drive settles a
// count or two past an end stop, and referenceframe rejects an out-of-bounds
// input outright, failing every pose query on the machine.
func clampToTravel(offsetMM float64) float64 {
	switch {
	case offsetMM < 0:
		return 0
	case offsetMM > KnifeTravelMM:
		return KnifeTravelMM
	default:
		return offsetMM
	}
}
