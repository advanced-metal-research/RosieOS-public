//go:build !linux

package simulator

import "errors"

func startNativeJogSession(string, string, []string) (*jogSession, error) {
	return nil, errors.New("local rt_core jog requires Linux/WSL")
}
func stopNativeJogSession(*jogSession) {}
