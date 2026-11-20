//go:build linux

// Package simfixture exposes the shared simulation compiler and the
// simulator's first-fault metrics reader to consumer tests outside the rt-core
// module. CPU selection must precede deployment identity, and a failed
// composed test must name the native fault, not only a generic cancellation.
package simfixture

import (
	"time"

	internal "rosieos/rt-core/internal/simfixture"
)

// CPURequirementError identifies a host that cannot separate RT and housekeeping.
type CPURequirementError = internal.CPURequirementError

// CheckCPURequirement permits suite-level skipping before any daemon or build;
// non-capability errors remain failures rather than hiding broken host queries.
func CheckCPURequirement() error { return internal.CheckCPURequirement() }

// Compile selects an allowed simulation CPU before compiling the input, so
// consumers cannot silently depend on a different daemon fallback CPU.
func Compile(root, machine, output string) (string, []string, error) {
	return internal.Compile(root, machine, output)
}

// CompileProfiles retains synthetic profiles while applying the same CPU policy
// as native fixtures, preventing consumer and daemon identities from diverging.
func CompileProfiles(root, machine, profiles, output string) (string, []string, error) {
	return internal.CompileProfiles(root, machine, profiles, output)
}

// FirstFault names the simulator's retained Home failure, latched faults and
// cycle timing from the metrics.json beside its IPC socket, so a composed test
// that fails with a generic cancellation reports which native detector fired.
// It waits up to budget for fresh publications and is diagnostic only.
func FirstFault(metricsPath string, budget time.Duration) string {
	return internal.FirstFault(metricsPath, budget)
}
