package removersvc

import (
	"fmt"
	"math"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/utils"

	"github.com/viam-modules/mirka/autochanger"
)

// Config names what the service cannot derive: resources, frames, obstacles and limits.
type Config struct {
	Remover            string                                     `json:"remover"`
	Arm                string                                     `json:"arm"`
	Mirka              string                                     `json:"mirka"`
	ToolFrame          string                                     `json:"tool_frame"`
	ObstacleVisions    []string                                   `json:"obstacle_visions,omitempty"`
	InputRangeOverride map[string]map[string]referenceframe.Limit `json:"input_range_override,omitempty"`
	GripOffsetMM       float64                                    `json:"grip_offset_mm"`
	// Frames besides tool_frame allowed to touch the remover in contact steps.
	ContactFrames []string `json:"contact_frames,omitempty"`
	// Contact-step limits, overriding the arm's; zero means the default.
	ContactSpeedDegsPerSec              float64 `json:"contact_speed_degs_per_sec,omitempty"`
	ContactAccelerationDegsPerSecPerSec float64 `json:"contact_acceleration_degs_per_sec_per_sec,omitempty"`
}

const (
	defaultContactSpeedDegsPerSec              = 10.0
	defaultContactAccelerationDegsPerSecPerSec = 20.0
)

// contactMoveOptions returns the arm limits for a contact step.
func (c *Config) contactMoveOptions() *arm.MoveOptions {
	v, a := c.ContactSpeedDegsPerSec, c.ContactAccelerationDegsPerSecPerSec
	if v == 0 {
		v = defaultContactSpeedDegsPerSec
	}
	if a == 0 {
		a = defaultContactAccelerationDegsPerSecPerSec
	}
	return &arm.MoveOptions{MaxVelRads: v * math.Pi / 180, MaxAccRads: a * math.Pi / 180}
}

// Validate returns the required dependencies; the frame system is implicit.
func (c *Config) Validate(path string) ([]string, []string, error) {
	required := []struct{ field, value string }{
		{"remover", c.Remover}, {"arm", c.Arm}, {"mirka", c.Mirka}, {"tool_frame", c.ToolFrame},
	}
	for _, r := range required {
		if r.value == "" {
			return nil, nil, utils.NewConfigValidationFieldRequiredError(path, r.field)
		}
	}
	if c.GripOffsetMM <= 0 || c.GripOffsetMM >= autochanger.KnifeTravelMM {
		return nil, nil, fmt.Errorf("%s: grip_offset_mm must be between 0 and %v exclusive, got %v",
			path, autochanger.KnifeTravelMM, c.GripOffsetMM)
	}
	if c.ContactSpeedDegsPerSec < 0 {
		return nil, nil, fmt.Errorf("%s: contact_speed_degs_per_sec must not be negative, got %v", path, c.ContactSpeedDegsPerSec)
	}
	if c.ContactAccelerationDegsPerSecPerSec < 0 {
		return nil, nil, fmt.Errorf("%s: contact_acceleration_degs_per_sec_per_sec must not be negative, got %v",
			path, c.ContactAccelerationDegsPerSecPerSec)
	}
	for _, f := range c.ContactFrames {
		if f == c.ToolFrame {
			return nil, nil, fmt.Errorf("%s: contact_frames need not list tool_frame %q", path, f)
		}
	}
	deps := append([]string{c.Remover, c.Arm, c.Mirka}, c.ObstacleVisions...)
	return deps, nil, nil
}
