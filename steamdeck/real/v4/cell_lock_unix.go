//go:build !windows

package main

import (
	"os"
	"os/exec"
	"syscall"
)

// cellLockDetach puts the holder's ssh in its own process group: a terminal
// Ctrl-C reaches this process, which releases the lock in order.
func cellLockDetach(cmd *exec.Cmd) {
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
}

// remoteKillGroup ends a detached local ssh and its children (the real ssh
// under `tailscale ssh`): TERM, or KILL when hard.
func remoteKillGroup(p *os.Process, hard bool) {
	sig := syscall.SIGTERM
	if hard {
		sig = syscall.SIGKILL
	}
	if syscall.Kill(-p.Pid, sig) != nil {
		_ = p.Signal(sig)
	}
}
