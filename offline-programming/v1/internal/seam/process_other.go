//go:build !linux

package seam

import (
	"os"
	"os/exec"
	"time"
)

// containProcessTree bounds Wait where there is no process group to kill (see
// process_linux.go): a grandchild still holding the pipes stops blocking it
// after WaitDelay, though it is left to finish on its own.
func containProcessTree(command *exec.Cmd) {
	command.WaitDelay = 2 * time.Second
}

func killProcessTree(command *exec.Cmd) error {
	if command.Process == nil {
		return os.ErrProcessDone
	}
	return command.Process.Kill()
}
