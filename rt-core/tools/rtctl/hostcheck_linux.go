//go:build linux

package rtctl

import "golang.org/x/sys/unix"

// hostTimerSlack reads the calling thread's timer slack (read-only prctl).
func hostTimerSlack() (int, error) {
	return unix.PrctlRetInt(unix.PR_GET_TIMERSLACK, 0, 0, 0, 0)
}

// hostMonotonicResolution reads CLOCK_MONOTONIC's resolution in ns.
func hostMonotonicResolution() (int64, error) {
	var value unix.Timespec
	if err := unix.ClockGetres(unix.CLOCK_MONOTONIC, &value); err != nil {
		return 0, err
	}
	return value.Nano(), nil
}
