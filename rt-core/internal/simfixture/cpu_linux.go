//go:build linux

package simfixture

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"

	"golang.org/x/sys/unix"
)

const (
	reservedCPUEnv = "ROSIE_SIMFIXTURE_RESERVED_CPU"
	processCPUsEnv = "ROSIE_SIMFIXTURE_PROCESS_CPUS"
)

func setProcessThreadsAffinity(mask *unix.CPUSet) error {
	tasks, err := filepath.Glob("/proc/self/task/*")
	if err != nil {
		return err
	}
	for _, task := range tasks {
		tid, err := strconv.Atoi(filepath.Base(task))
		if err != nil {
			return err
		}
		if err := unix.SchedSetaffinity(tid, mask); err != nil && err != unix.ESRCH {
			return fmt.Errorf("set test thread %d affinity: %w", tid, err)
		}
	}
	return nil
}

// CPURequirementError reports the measured daemon requirement: one allowed RT
// CPU plus one housekeeping CPU. Fixture callers may skip this unavailable host.
type CPURequirementError struct {
	Allowed int
}

func (e *CPURequirementError) Error() string {
	return fmt.Sprintf("simulation fixture requires at least two allowed CPUs (one RT CPU and one housekeeping CPU); allowed=%d", e.Allowed)
}

// CheckCPURequirement refuses unavailable fixture hosts before any build or
// daemon startup, preventing repeated housekeeping failures in composed suites.
func CheckCPURequirement() error {
	var allowed unix.CPUSet
	if err := unix.SchedGetaffinity(0, &allowed); err != nil {
		return fmt.Errorf("read simulation fixture affinity: %w", err)
	}
	return cpuRequirement(allowed)
}

func cpuRequirement(allowed unix.CPUSet) error {
	if allowed.Count() < 2 {
		return &CPURequirementError{Allowed: allowed.Count()}
	}
	return nil
}

// simulationCPUConfig replaces an unavailable simulation CPU before compilation,
// keeping the deployment digest consistent and leaving live host decisions intact.
func simulationCPUConfig(raw []byte, allowed unix.CPUSet) ([]byte, error) {
	var machine map[string]json.RawMessage
	if err := json.Unmarshal(raw, &machine); err != nil {
		return nil, err
	}
	var backend string
	if err := json.Unmarshal(machine["backend"], &backend); err != nil || backend != "simulation" {
		return raw, nil
	}
	var runtime map[string]json.RawMessage
	if err := json.Unmarshal(machine["runtime"], &runtime); err != nil {
		return raw, nil
	}
	var configured *uint32
	if err := json.Unmarshal(runtime["cpu"], &configured); err != nil || configured == nil {
		return raw, nil
	}
	// Measured compiler contract: RT CPU indices are integers from 0 through 1023.
	const maximumCompiledCPU = 1023
	if *configured > maximumCompiledCPU {
		return raw, nil
	}
	if err := cpuRequirement(allowed); err != nil {
		return nil, err
	}
	if allowed.IsSet(int(*configured)) {
		return raw, nil
	}
	remaining := allowed.Count()
	selected := 0
	for cpu := 0; remaining > 0; cpu++ {
		if allowed.IsSet(cpu) {
			selected = cpu
			remaining--
		}
	}
	runtimeStart, runtimeEnd, err := objectValueSpan(raw, "runtime")
	if err != nil {
		return nil, err
	}
	cpuStart, cpuEnd, err := objectValueSpan(raw[runtimeStart:runtimeEnd], "cpu")
	if err != nil {
		return nil, err
	}
	result := append([]byte(nil), raw[:runtimeStart+cpuStart]...)
	result = strconv.AppendInt(result, int64(selected), 10)
	return append(result, raw[runtimeStart+cpuEnd:]...), nil
}

// objectValueSpan locates the last decoded property, matching json.Unmarshal's
// duplicate-key policy while leaving whitespace and escaped names untouched.
func objectValueSpan(raw []byte, name string) (int, int, error) {
	decoder := json.NewDecoder(bytes.NewReader(raw))
	opening, err := decoder.Token()
	if err != nil || opening != json.Delim('{') {
		return 0, 0, fmt.Errorf("locate %s: expected JSON object", name)
	}
	start, end := -1, -1
	for decoder.More() {
		key, err := decoder.Token()
		if err != nil {
			return 0, 0, err
		}
		var value json.RawMessage
		if err := decoder.Decode(&value); err != nil {
			return 0, 0, err
		}
		if key == name {
			end = int(decoder.InputOffset())
			start = end - len(value)
		}
	}
	if _, err := decoder.Token(); err != nil {
		return 0, 0, err
	}
	if start < 0 {
		return 0, 0, fmt.Errorf("locate %s: missing JSON property", name)
	}
	return start, end, nil
}

// CompileRaw selects the simulation CPU before invoking an in-process compiler,
// preserving its typed errors without importing a client-dependent package.
func CompileRaw[T any](raw []byte, profiles string, compile func([]byte, string) (T, error)) (T, error) {
	selected, err := currentCPUConfig(raw)
	if err != nil {
		var zero T
		return zero, err
	}
	return compile(selected, profiles)
}

// currentCPUConfig observes the calling thread's allowed CPUs before any fixture
// compiler runs, preventing raw and file inputs from selecting different policies.
func currentCPUConfig(raw []byte) ([]byte, error) {
	if value := os.Getenv(reservedCPUEnv); value != "" {
		cpu, err := strconv.Atoi(value)
		if err != nil || cpu < 0 || cpu > 1023 {
			return nil, fmt.Errorf("invalid reserved simulation CPU %q", value)
		}
		return forceSimulationCPU(raw, cpu)
	}
	var allowed unix.CPUSet
	if err := unix.SchedGetaffinity(0, &allowed); err != nil {
		return nil, fmt.Errorf("read simulation fixture affinity: %w", err)
	}
	return simulationCPUConfig(raw, allowed)
}

func forceSimulationCPU(raw []byte, cpu int) ([]byte, error) {
	var machine map[string]json.RawMessage
	if err := json.Unmarshal(raw, &machine); err != nil {
		return nil, err
	}
	var backend string
	if err := json.Unmarshal(machine["backend"], &backend); err != nil || backend != "simulation" {
		return raw, nil
	}
	runtimeStart, runtimeEnd, err := objectValueSpan(raw, "runtime")
	if err != nil {
		return nil, err
	}
	cpuStart, cpuEnd, err := objectValueSpan(raw[runtimeStart:runtimeEnd], "cpu")
	if err != nil {
		return nil, err
	}
	result := append([]byte(nil), raw[:runtimeStart+cpuStart]...)
	result = strconv.AppendInt(result, int64(cpu), 10)
	return append(result, raw[runtimeStart+cpuEnd:]...), nil
}

// ReserveSimulationCPU removes one CPU from this Go test process and records a
// two-CPU mask for simulator children. The native cycle expands onto the
// reserved CPU while its housekeeping stays on one ordinary CPU, so host-side
// race instrumentation cannot manufacture cycle-deadline faults. Live
// binaries and production affinity are unchanged.
func ReserveSimulationCPU() (func(), error) {
	if os.Getenv(reservedCPUEnv) != "" {
		return func() {}, nil
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	var original unix.CPUSet
	if err := unix.SchedGetaffinity(0, &original); err != nil {
		return nil, fmt.Errorf("read test affinity: %w", err)
	}
	if original.Count() < 2 {
		return nil, cpuRequirement(original)
	}
	reserved, housekeepingCPU := -1, -1
	var reservation *os.File
	for cpu := 0; cpu < 1024; cpu++ {
		if original.IsSet(cpu) && housekeepingCPU < 0 {
			housekeepingCPU = cpu
		}
	}
	for cpu := 1023; cpu >= 0; cpu-- {
		if !original.IsSet(cpu) || cpu == housekeepingCPU {
			continue
		}
		path := filepath.Join("/dev/shm", fmt.Sprintf("rosie-simfixture-cpu-%d.lock", cpu))
		file, err := os.OpenFile(path, os.O_CREATE|os.O_RDWR, 0600)
		if err != nil {
			return nil, err
		}
		if err := unix.Flock(int(file.Fd()), unix.LOCK_EX|unix.LOCK_NB); err == nil {
			reserved, reservation = cpu, file
			break
		}
		_ = file.Close()
	}
	if reserved < 0 {
		return nil, errors.New("no unclaimed simulation RT CPU")
	}
	var housekeeping unix.CPUSet
	for cpu := 0; cpu < 1024; cpu++ {
		if cpu != reserved && original.IsSet(cpu) {
			housekeeping.Set(cpu)
		}
	}
	if err := setProcessThreadsAffinity(&housekeeping); err != nil {
		_ = reservation.Close()
		return nil, err
	}
	if err := os.Setenv(reservedCPUEnv, strconv.Itoa(reserved)); err != nil {
		_ = setProcessThreadsAffinity(&original)
		_ = reservation.Close()
		return nil, err
	}
	var childCPUs []string
	for cpu := 0; cpu < 1024; cpu++ {
		if original.IsSet(cpu) {
			childCPUs = append(childCPUs, strconv.Itoa(cpu))
		}
	}
	if err := os.Setenv(processCPUsEnv, strings.Join(childCPUs, ",")); err != nil {
		_ = os.Unsetenv(reservedCPUEnv)
		_ = setProcessThreadsAffinity(&original)
		_ = reservation.Close()
		return nil, err
	}
	return func() {
		_ = os.Unsetenv(processCPUsEnv)
		_ = os.Unsetenv(reservedCPUEnv)
		_ = setProcessThreadsAffinity(&original)
		_ = unix.Flock(int(reservation.Fd()), unix.LOCK_UN)
		_ = reservation.Close()
	}, nil
}

// SuspendSimulationCPUReservation restores the package's original affinity for
// tests whose subject is CPU selection itself. The reservation lock remains
// held, preventing another package from borrowing that CPU during the probe.
func SuspendSimulationCPUReservation() (func(), error) {
	cpus, reservedText := os.Getenv(processCPUsEnv), os.Getenv(reservedCPUEnv)
	if cpus == "" || reservedText == "" {
		return func() {}, nil
	}
	var original, housekeeping unix.CPUSet
	for _, item := range strings.Split(cpus, ",") {
		cpu, err := strconv.Atoi(item)
		if err != nil || cpu < 0 || cpu >= 1024 {
			return nil, fmt.Errorf("invalid simulation process CPU %q", item)
		}
		original.Set(cpu)
		housekeeping.Set(cpu)
	}
	reserved, err := strconv.Atoi(reservedText)
	if err != nil || !original.IsSet(reserved) {
		return nil, fmt.Errorf("invalid reserved simulation CPU %q", reservedText)
	}
	var withoutReserved unix.CPUSet
	for cpu := 0; cpu < 1024; cpu++ {
		if housekeeping.IsSet(cpu) && cpu != reserved {
			withoutReserved.Set(cpu)
		}
	}
	if err := setProcessThreadsAffinity(&original); err != nil {
		return nil, err
	}
	_ = os.Unsetenv(processCPUsEnv)
	_ = os.Unsetenv(reservedCPUEnv)
	return func() {
		_ = setProcessThreadsAffinity(&withoutReserved)
		_ = os.Setenv(reservedCPUEnv, reservedText)
		_ = os.Setenv(processCPUsEnv, cpus)
	}, nil
}

// SelectMachine writes a private input only when affinity requires a simulation
// override; source files and their unrelated numeric values cannot be rewritten.
// The caller must invoke cleanup even when selection returns an error.
func SelectMachine(machine, output string) (string, func(), error) {
	cleanup := func() {}
	raw, err := os.ReadFile(machine)
	if err != nil {
		return "", cleanup, err
	}
	configured, err := currentCPUConfig(raw)
	if err != nil {
		return "", cleanup, err
	}
	if string(configured) == string(raw) {
		return machine, cleanup, nil
	}
	if err := os.MkdirAll(output, 0700); err != nil {
		return "", cleanup, err
	}
	file, err := os.CreateTemp(output, "simulation-machine-*.json")
	if err != nil {
		return "", cleanup, err
	}
	cleanup = func() { _ = os.Remove(file.Name()) }
	_, err = file.Write(configured)
	closeErr := file.Close()
	if err == nil {
		err = closeErr
	}
	return file.Name(), cleanup, err
}
