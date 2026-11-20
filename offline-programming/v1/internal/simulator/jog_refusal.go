package simulator

import "errors"

// GoHome refuses saved-pose playback through the Cartesian jog endpoint.
func (manager *JogManager) GoHome() (JogState, error) {
	return manager.State(), errors.New("saved-pose Go Home is unavailable for rt_core Cartesian jog")
}

// StepJoint refuses joint motion through the Cartesian jog endpoint.
func (manager *JogManager) StepJoint(jointIndex int, deltaRad float64) error {
	return errors.New("joint steps are unavailable for rt_core Cartesian jog")
}
