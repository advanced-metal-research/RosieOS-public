//go:build linux

package seam

import (
	"errors"
	"os"
	"os/exec"
	"syscall"
	"time"
)

// containProcessTree makes a worker's whole process tree killable as one.
//
// The worker is rarely the process started: through pixi it is `pixi run`,
// whose child is the python doing the work. Killing only the launcher left
// that child running and still holding stdout, so Wait blocked until it
// finished -- measured with `sh -c 'cat >/dev/null; sleep 30'`, a canceled
// request waited the full 30 s. So the tree gets its own process group,
// killProcessTree kills the group, and the tree dies with this server
// (`internal/simulator` contains its planner the same way).
//
// No `Cancel` here: exec refuses to start a command that has one without a
// context, and a spare is started before any request -- so any context -- exists.
// The runner that does have one sets it.
func containProcessTree(command *exec.Cmd) {
	command.SysProcAttr = &syscall.SysProcAttr{
		Setpgid:   true,
		Pdeathsig: syscall.SIGKILL,
	}
	command.WaitDelay = 2 * time.Second
}

func killProcessTree(command *exec.Cmd) error {
	if command.Process == nil {
		return os.ErrProcessDone
	}
	err := syscall.Kill(-command.Process.Pid, syscall.SIGKILL)
	if errors.Is(err, syscall.ESRCH) {
		return os.ErrProcessDone
	}
	return err
}
