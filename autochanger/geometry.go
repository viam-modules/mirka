package autochanger

import (
	"context"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// Remover envelope, in the remover frame: origin at the knife zero point,
// +Y = knife extension direction (toward the robot), +Z = up.
//
// Conservative estimates scaled from the manual's drawings (pp. 39, 48-49;
// no vendor CAD is published). Verified/corrected against the physical
// unit at bring-up. Uncertain dimensions are rounded UP: an oversized
// obstacle costs clearance, an undersized one costs a collision.
const (
	bodyWidthMM  = 240
	bodyDepthMM  = 180
	bodyHeightMM = 650

	plateWidthMM  = 200
	plateDepthMM  = 12
	plateHeightMM = 340

	bladeWidthMM  = 160
	bladeTravelMM = 25 // full linear-unit extension (knife position 3)
	bladeHeightMM = 8
)

func box(x, y, z, cx, cy, cz float64, label string) spatialmath.Geometry {
	b, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: cx, Y: cy, Z: cz}),
		r3.Vector{X: x, Y: y, Z: z}, label)
	if err != nil {
		// Only reachable with non-positive dims, which are constants above.
		panic(err)
	}
	return b
}

func (r *remover) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return []spatialmath.Geometry{
		// Structure behind the front plate, extending downward from the
		// knife area (the knife sits near the top of the plate).
		box(bodyWidthMM, bodyDepthMM, bodyHeightMM, 0, -bodyDepthMM/2, -bodyHeightMM/2+50, "remover_body"),
		// Sliding plate on the front face, hanging below the knife line.
		box(plateWidthMM, plateDepthMM, plateHeightMM, 0, plateDepthMM/2, -plateHeightMM/2, "remover_sliding_plate"),
		// Blade swept volume: y from 0 (retracted zero) to full travel.
		box(bladeWidthMM, bladeTravelMM, bladeHeightMM, 0, bladeTravelMM/2, 0, "remover_blade"),
	}, nil
}
