//go:build !linux

package rtctl

import (
	"fmt"
	"io"
)

// The public control commands talk to the Linux adapter's socket. Off Linux
// rtctl compiles configurations only, so no control command is recognised.
func IsControlCommand(string) bool { return false }

func RunControlCLI(_ []string, _, stderr io.Writer) int {
	fmt.Fprintln(stderr, "rtctl control commands need Linux")
	return 2
}
