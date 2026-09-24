package removersvc

import (
	"fmt"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/utils"

	"github.com/viam-modules/mirka/autochanger"
)

// Config names what the service cannot derive: the resources it drives, the
// pad's frame, where obstacles come from, the cell's joint limits, and the one
// disc-dependent number the Mirka manual leaves to the integrator.
type Config struct {
	Remover         string   `json:"remover"`
	Arm             string   `json:"arm"`
	Mirka           string   `json:"mirka"`
	ToolFrame       string   `json:"tool_frame"`
	ObstacleVisions []string `json:"obstacle_visions"`
	// Must match the sanding config's copy. Drift is silent: both planners
	// succeed and one wraps the cable.
	InputRangeOverride map[string]map[string]referenceframe.Limit `json:"input_range_override"`
	GripOffsetMM       float64                                    `json:"grip_offset_mm"`
}

// Validate returns the remover, arm, Mirka and obstacle visions as required
// dependencies. The frame-system service is provided to modules implicitly.
func (c *Config) Validate(path string) ([]string, []string, error) {
	required := []struct{ field, value string }{
		{"remover", c.Remover}, {"arm", c.Arm}, {"mirka", c.Mirka}, {"tool_frame", c.ToolFrame},
	}
	for _, r := range required {
		if r.value == "" {
			return nil, nil, utils.NewConfigValidationFieldRequiredError(path, r.field)
		}
	}
	if len(c.ObstacleVisions) == 0 {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "obstacle_visions")
	}
	// The limits are load-bearing (cable wrap); model limits are not a fallback.
	if len(c.InputRangeOverride) == 0 {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "input_range_override")
	}
	if c.GripOffsetMM <= 0 || c.GripOffsetMM >= autochanger.KnifeTravelMM {
		return nil, nil, fmt.Errorf("%s: grip_offset_mm must be between 0 and %v exclusive, got %v",
			path, autochanger.KnifeTravelMM, c.GripOffsetMM)
	}
	deps := append([]string{c.Remover, c.Arm, c.Mirka}, c.ObstacleVisions...)
	return deps, nil, nil
}
