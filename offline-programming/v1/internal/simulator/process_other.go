//go:build !linux && !windows

package simulator

import (
	"errors"
	"os/exec"
)

type commandContainment struct{}

func boundedProcessSupported() bool {
	return false
}

func prepareBoundedCommand(*exec.Cmd) (*commandContainment, error) {
	return nil, errors.New("canonical planner process containment is unsupported")
}

func (*commandContainment) AfterStart() error {
	return errors.New("canonical planner process containment is unsupported")
}

func (*commandContainment) Close() {
}
