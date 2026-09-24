package removersvc

import (
	"math"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

func TestApplyJointLimits(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("tightens and never loosens", func(t *testing.T) {
		fs := testFrameSystem(t)
		err := applyJointLimits(logger, fs, map[string]map[string]referenceframe.Limit{
			"arm": {"0": {Min: -1, Max: 100}},
		})
		test.That(t, err, test.ShouldBeNil)
		dof := fs.Frame("arm").DoF()
		test.That(t, dof[0].Min, test.ShouldAlmostEqual, -1)
		test.That(t, dof[0].Max, test.ShouldAlmostEqual, 2*math.Pi)
	})

	t.Run("matches joints by name", func(t *testing.T) {
		fs := testFrameSystem(t)
		err := applyJointLimits(logger, fs, map[string]map[string]referenceframe.Limit{
			"arm": {"j0": {Min: -1, Max: 1}},
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fs.Frame("arm").DoF()[0].Max, test.ShouldAlmostEqual, 1)
	})

	t.Run("unknown frame refuses", func(t *testing.T) {
		err := applyJointLimits(logger, testFrameSystem(t), map[string]map[string]referenceframe.Limit{
			"elbow": {"0": {Min: -1, Max: 1}},
		})
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "elbow")
	})

	t.Run("unknown joint refuses", func(t *testing.T) {
		err := applyJointLimits(logger, testFrameSystem(t), map[string]map[string]referenceframe.Limit{
			"arm": {"7": {Min: -1, Max: 1}},
		})
		test.That(t, err, test.ShouldNotBeNil)
	})
}
