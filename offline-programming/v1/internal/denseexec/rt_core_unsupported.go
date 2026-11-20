//go:build !linux

package denseexec

import "errors"

// The public Go SDK currently builds on Linux. Keep the legacy application
// buildable elsewhere and reject public selection before any transport opens.
type RTCoreConfig struct{}
type RTCoreManifest struct{}
type RTCoreObservation struct{}

func DecodeRTCoreConfig([]byte) (RTCoreConfig, error) {
	return RTCoreConfig{}, errors.New("rt_core requires the Linux Go SDK; run the OLP server in WSL or Linux")
}

func NewRTCore(RTCoreConfig, Config, any) (Backend, error) {
	return nil, errors.New("rt_core requires the Linux Go SDK; run the OLP server in WSL or Linux")
}
