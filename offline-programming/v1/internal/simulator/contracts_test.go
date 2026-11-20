package simulator

import (
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

func TestPCTopologyRejectsExternalOrCommandPaths(t *testing.T) {
	valid := NewPCTopology(
		"local-simulator",
		planning.RobotModelRosie1400V3,
		"127.0.0.1",
	)
	if err := valid.Validate(); err != nil {
		t.Fatalf("valid topology: %v", err)
	}
	tests := []struct {
		name string
		edit func(*Topology)
	}{
		{"external host", func(value *Topology) { value.LoopbackHost = "192.0.2.10" }},
		{"wildcard host", func(value *Topology) { value.LoopbackHost = "0.0.0.0" }},
		{"UDP plan transport", func(value *Topology) { value.PlanTransport = "udp" }},
		{"physical service start", func(value *Topology) { value.PhysicalServiceStart = true }},
		{"command transport", func(value *Topology) { value.CommandTransport = true }},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			topology := valid
			test.edit(&topology)
			if err := topology.Validate(); err == nil {
				t.Fatalf("unsafe topology accepted: %+v", topology)
			}
		})
	}
}

func TestHTTPWiringPathsAreIsolatedFromPhysicalEndpoints(t *testing.T) {
	paths := []string{HTTPStatusPath, HTTPStartPath, HTTPStopPath, HTTPPlansPath}
	seen := map[string]bool{}
	for _, path := range paths {
		if !strings.HasPrefix(path, "/api/offline-programming/v1/local-simulator/") ||
			seen[path] {
			t.Fatalf("invalid or duplicate local wiring path %q", path)
		}
		seen[path] = true
	}
}

func TestSafetyProofRequiresEveryPositiveSafeFact(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 0, 0, 0, time.UTC)
	safe := safeProof(now)
	if err := safe.Validate(now, "local-simulator"); err != nil {
		t.Fatalf("safe proof rejected: %v", err)
	}
	tests := []struct {
		name string
		edit func(*SafetyProof)
	}{
		{"simulation false", func(value *SafetyProof) { value.Simulation = false }},
		{"real movement possible", func(value *SafetyProof) { value.NoRealMovement = false }},
		{"not loopback", func(value *SafetyProof) { value.LoopbackOnly = false }},
		{"EtherCAT output", func(value *SafetyProof) { value.EtherCATOutputEnabled = true }},
		{"motion output", func(value *SafetyProof) { value.MotionOutputEnabled = true }},
		{"hardware access", func(value *SafetyProof) { value.HardwareAccess = true }},
		{"physical services", func(value *SafetyProof) { value.PhysicalServicesStarted = true }},
		{"command submission", func(value *SafetyProof) { value.CommandSubmissionEnabled = true }},
		{"stale", func(value *SafetyProof) { value.FreshUntil = now }},
		{"future", func(value *SafetyProof) { value.ObservedAt = now.Add(time.Second) }},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			proof := safe
			test.edit(&proof)
			if err := proof.Validate(now, "local-simulator"); err == nil {
				t.Fatalf("unsafe proof accepted: %+v", proof)
			}
		})
	}
}

func TestResourceLimitsAreBoundedAndSingleFlight(t *testing.T) {
	limits := DefaultResourceLimits()
	if err := limits.Validate(); err != nil {
		t.Fatal(err)
	}
	limits.MaxConcurrentPlans = 2
	if err := limits.Validate(); err == nil {
		t.Fatal("multiple concurrent plans were accepted")
	}
	limits = DefaultResourceLimits()
	limits.MaxPlanReportBytes = 257 << 20
	if err := limits.Validate(); err == nil {
		t.Fatal("unbounded report memory was accepted")
	}
}

func safeProof(now time.Time) SafetyProof {
	return SafetyProof{
		Schema:                   SafetyProofSchema,
		ProfileID:                "local-simulator",
		RuntimeBootID:            "runtime-boot-1",
		StatusSequence:           7,
		ObservedAt:               now.Add(-time.Millisecond),
		FreshUntil:               now.Add(time.Second),
		Simulation:               true,
		NoRealMovement:           true,
		LoopbackOnly:             true,
		EtherCATOutputEnabled:    false,
		MotionOutputEnabled:      false,
		HardwareAccess:           false,
		PhysicalServicesStarted:  false,
		CommandSubmissionEnabled: false,
	}
}
