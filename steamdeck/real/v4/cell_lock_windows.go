//go:build windows

package main

import (
	"os"
	"os/exec"
)

func cellLockDetach(*exec.Cmd) {}

func remoteKillGroup(p *os.Process, _ bool) { _ = p.Kill() }
