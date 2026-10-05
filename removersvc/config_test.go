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

	noVisions := validConfig()
	noVisions.ObstacleVisions = nil
	deps, _, err = noVisions.Validate("svc")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"remover", "arm", "airos"})

	noLimits := validConfig()
	noLimits.InputRangeOverride = nil
	_, _, err = noLimits.Validate("svc")
	test.That(t, err, test.ShouldBeNil)

	withContact := validConfig()
	withContact.ContactFrames = []string{"sander", "mirka-body"}
	_, _, err = withContact.Validate("svc")
	test.That(t, err, test.ShouldBeNil)

	cases := []struct {
		name   string
		mutate func(*Config)
		want   string
	}{
		{"no remover", func(c *Config) { c.Remover = "" }, "remover"},
		{"no arm", func(c *Config) { c.Arm = "" }, "arm"},
		{"no mirka", func(c *Config) { c.Mirka = "" }, "mirka"},
		{"no tool frame", func(c *Config) { c.ToolFrame = "" }, "tool_frame"},
		{"grip zero", func(c *Config) { c.GripOffsetMM = 0 }, "grip_offset_mm"},
		{"grip negative", func(c *Config) { c.GripOffsetMM = -0.7 }, "grip_offset_mm"},
		{"grip at release", func(c *Config) { c.GripOffsetMM = 25 }, "grip_offset_mm"},
		{"contact frame is the tool frame", func(c *Config) { c.ContactFrames = []string{"compliance"} }, "need not list tool_frame"},
		{"negative contact speed", func(c *Config) { c.ContactSpeedDegsPerSec = -1 }, "contact_speed_degs_per_sec"},
		{"negative contact acceleration", func(c *Config) { c.ContactAccelerationDegsPerSecPerSec = -1 },
			"contact_acceleration_degs_per_sec_per_sec"},
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
