//go:build linux

package simulator

import (
	"errors"
	"os"
	"os/exec"
	"syscall"
	"time"
)

type commandContainment struct{}

func boundedProcessSupported() bool {
	return true
}

func prepareBoundedCommand(command *exec.Cmd) (*commandContainment, error) {
	command.SysProcAttr = &syscall.SysProcAttr{
		Setpgid:   true,
		Pdeathsig: syscall.SIGKILL,
	}
	command.Cancel = func() error {
		if command.Process == nil {
			return os.ErrProcessDone
		}
		err := syscall.Kill(-command.Process.Pid, syscall.SIGKILL)
		if errors.Is(err, syscall.ESRCH) {
			return os.ErrProcessDone
		}
		return err
	}
	command.WaitDelay = 2 * time.Second
	return &commandContainment{}, nil
}

func (*commandContainment) AfterStart() error {
	return nil
}

func (*commandContainment) Close() {
}
