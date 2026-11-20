//go:build !linux

package rtctl

import "errors"

// hostcheck inspects a Linux servo or OLP host; off Linux the live timer reads
// fail as unreadable rather than reporting the analysis machine's clock.
var errHostcheckNeedsLinux = errors.New("host timer checks need Linux")

func hostTimerSlack() (int, error) { return 0, errHostcheckNeedsLinux }

func hostMonotonicResolution() (int64, error) { return 0, errHostcheckNeedsLinux }
