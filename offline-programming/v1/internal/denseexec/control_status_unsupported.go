//go:build !linux

package denseexec

// Native execution is unavailable on this platform, so there is no native
// handle history to project. Keep the ordinary unavailable/status contract.
func ControlStatusBody(body *StatusBody) *StatusBody { return body }
