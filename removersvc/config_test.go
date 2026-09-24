package removersvc

import (
	"testing"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

func validConfig() *Config {
	return &Config{
		Remover:         "remover",
		Arm:             "arm",
		Mirka:           "airos",
		ToolFrame:       "compliance",
		ObstacleVisions: []string{"snapshot_mesh_vision_service"},
		InputRangeOverride: map[string]map[string]referenceframe.Limit{
			"arm": {"2": {Min: -4, Max: 0}},
		},
		GripOffsetMM: 0.7,
	}
}

func TestValidate(t *testing.T) {
	deps, opt, err := validConfig().Validate("svc")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, opt, test.ShouldBeEmpty)
	test.That(t, deps, test.ShouldResemble,
		[]string{"remover", "arm", "airos", "snapshot_mesh_vision_service"})

	cases := []struct {
		name   string
		mutate func(*Config)
		want   string
	}{
		{"no remover", func(c *Config) { c.Remover = "" }, "remover"},
		{"no arm", func(c *Config) { c.Arm = "" }, "arm"},
		{"no mirka", func(c *Config) { c.Mirka = "" }, "mirka"},
		{"no tool frame", func(c *Config) { c.ToolFrame = "" }, "tool_frame"},
		{"no visions", func(c *Config) { c.ObstacleVisions = nil }, "obstacle_visions"},
		{"no limits", func(c *Config) { c.InputRangeOverride = nil }, "input_range_override"},
		{"no limits for the arm", func(c *Config) {
			c.InputRangeOverride = map[string]map[string]referenceframe.Limit{"gantry": {"0": {Min: 0, Max: 1}}}
		}, `input_range_override has no entry for arm "arm"`},
		{"grip zero", func(c *Config) { c.GripOffsetMM = 0 }, "grip_offset_mm"},
		{"grip negative", func(c *Config) { c.GripOffsetMM = -0.7 }, "grip_offset_mm"},
		{"grip at release", func(c *Config) { c.GripOffsetMM = 25 }, "grip_offset_mm"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			c := validConfig()
			tc.mutate(c)
			_, _, err := c.Validate("svc")
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, tc.want)
		})
	}
}
