//go:build !linux

package denseexec

// DriveHealthFrom needs the Linux rt-core SDK, as every rt-core read does.
func DriveHealthFrom(*StatusBody) (*DriveHealth, error) { return nil, ErrDriveHealthUnavailable }
