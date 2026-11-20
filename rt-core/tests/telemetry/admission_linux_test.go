//go:build linux

package tests_test

import (
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/tools/rtctl"
)

// Both admission paths accept the inclusive byte bounds and reject their
// neighbours, preventing a compiler/daemon dead budget range or JSON precision loss.
func TestTelemetryDumpByteAdmission(t *testing.T) {
	if ipcclient.TelemetryDumpMinBytes != 529680 || ipcclient.TelemetryDumpMaxBytes != 9007199254740991 {
		t.Fatal("protocol/control.json telemetry bounds must admit 529680..9007199254740991 bytes")
	}
	root, err := filepath.Abs("../..")
	simfixture.Must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "telemetry-admission-")
	simfixture.Must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation.json"))
	simfixture.Must(t, err)
	var machine map[string]any
	simfixture.Must(t, json.Unmarshal(raw, &machine))
	_, argv, err := simfixture.Compile(root, filepath.Join(root, "config/machines/simulation/simulation.json"), build)
	simfixture.Must(t, err)
	for _, bytes := range []int{529680, 529679, 9007199254740991, 9007199254740992, 0, -1} {
		machine["runtime"].(map[string]any)["telemetry"] = map[string]any{"dump_bytes": bytes}
		raw, err = json.Marshal(machine)
		simfixture.Must(t, err)
		compiled, compileErr := simfixture.CompileRaw(raw, filepath.Join(root, "config/drives"), rtctl.Compile)
		command := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(append([]string{}, argv...), "--telemetry-dump-bytes", strconv.Itoa(bytes), "--validate-config")...)
		output, nativeErr := command.CombinedOutput()
		if bytes == 529680 || bytes == 9007199254740991 {
			if compileErr != nil || nativeErr != nil || !strings.Contains(strings.Join(compiled.Argv, " "), "--telemetry-dump-bytes "+strconv.Itoa(bytes)) {
				t.Fatalf("%d byte admission: compiler=%v native=%v %s", bytes, compileErr, nativeErr, output)
			}
		} else {
			if compileErr == nil || nativeErr == nil || command.ProcessState.ExitCode() != 2 || !strings.Contains(string(output), "two headers and one 16-axis record") {
				t.Fatalf("%d byte refusal: compiler=%v native=%v %s", bytes, compileErr, nativeErr, output)
			}
		}
	}
}
