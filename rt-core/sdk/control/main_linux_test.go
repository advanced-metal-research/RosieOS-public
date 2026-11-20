//go:build linux

package control

import (
	"fmt"
	"os"
	"testing"

	"rosieos/rt-core/internal/simfixture"
)

func TestMain(m *testing.M) {
	restore, err := simfixture.ReserveSimulationCPU()
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(2)
	}
	code := m.Run()
	restore()
	os.Exit(code)
}
