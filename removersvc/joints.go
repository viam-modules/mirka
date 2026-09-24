package removersvc

import (
	"fmt"
	"math"
	"strconv"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
)

// applyJointLimits is a copy of ApplyJointLimits in viamrobotics/sanding
// lib/framesys/joints.go. Keep the two in step: both planners must plan with
// the same limits, and a divergence is silent. Limits only tighten, because
// the arm driver enforces its own kinematics at execution and rejects
// waypoints outside them.
func applyJointLimits(logger logging.Logger, fs *referenceframe.FrameSystem, inputRangeOverride map[string]map[string]referenceframe.Limit) error {
	for fName, mods := range inputRangeOverride {
		f := fs.Frame(fName)
		if f == nil {
			return fmt.Errorf("frame (%s) in input_range_override doesn't exist", fName)
		}

		sm, ok := f.(*referenceframe.SimpleModel)
		if !ok {
			return fmt.Errorf("can only override joints for SimpleModel for now, not %T", f)
		}

		// Resolve override keys: match by name first, then by stringified moveable-frame index
		resolved := make(map[string]referenceframe.Limit, len(mods))
		moveableNames := sm.MoveableFrameNames()
		// Never loosen: the driver enforces its own kinematics at execution, so a
		// looser override plans waypoints the driver then rejects.
		existingLimits := sm.DoF()
		for key, limit := range mods {
			matched := false
			for i, name := range moveableNames {
				if key == name || key == strconv.Itoa(i) {
					existing := existingLimits[i]
					tightened := referenceframe.Limit{
						Min: math.Max(limit.Min, existing.Min),
						Max: math.Min(limit.Max, existing.Max),
					}
					if tightened.Min != limit.Min || tightened.Max != limit.Max {
						logger.Warnf(
							"input_range_override for frame %q joint %q would loosen limits: requested [%.6f, %.6f], model declares [%.6f, %.6f]; tightening to [%.6f, %.6f]",
							fName, name,
							limit.Min, limit.Max,
							existing.Min, existing.Max,
							tightened.Min, tightened.Max,
						)
					}
					resolved[name] = tightened
					matched = true
					break
				}
			}
			if !matched {
				return fmt.Errorf("can't find mod (%s)", key)
			}
		}

		newModel, err := referenceframe.NewModelWithLimitOverrides(sm, resolved)
		if err != nil {
			return err
		}

		err = fs.ReplaceFrame(newModel)
		if err != nil {
			return err
		}
	}
	return nil
}
