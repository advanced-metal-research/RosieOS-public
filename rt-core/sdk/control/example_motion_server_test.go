//go:build linux

package control_test

import (
	"context"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Unverified example observation ceiling: 10000 ms, independent of lease and motion duration.
const profileObservation = 10000 * time.Millisecond

// Unverified renewal cadence: 100 ms = one fifth of the protocol's 500 ms lease.
const profileRenewal = 100 * time.Millisecond

// Unverified polling cadence: 5 ms; only a received observation establishes success.
const profilePoll = 5 * time.Millisecond

// profileService starts both real processes; consumers below use only the public
// SDK, preventing native IPC shortcuts from hiding application contract gaps.
func profileService(t *testing.T) (*control.Client, string, string, string) {
	t.Helper()
	root, err := filepath.Abs("../..")
	simfixture.Must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "profile-")
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "p-")
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	compiled, args, err := simfixture.Compile(root, filepath.Join(root, "clients/cpp/examples/testdata/profile-machine.json"), filepath.Join(build, "config"))
	simfixture.Must(t, err)
	coreSocket, socket := filepath.Join(dir, "ipc.sock"), filepath.Join(dir, "control.sock")
	simfixture.SpawnLogged(t, exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", coreSocket)...), filepath.Join(build, "core.log"), profileObservation, 5000, simfixture.CheckExit(t))
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), profileObservation, profilePoll, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	simfixture.SpawnLogged(t, simfixture.StartControl(root, coreSocket, socket, filepath.Base(compiled), "bench", 7), filepath.Join(build, "control.log"), profileObservation, 5000, simfixture.CheckExit(t))
	c, err := control.Dial(control.UnixPath(socket))
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = c.Close() })
	ctx, cancel := context.WithTimeout(context.Background(), profileObservation)
	defer cancel()
	simfixture.Must(t, simfixture.Poll(ctx, profilePoll, func() (bool, error) {
		state, err := c.Status(ctx)
		return err == nil && state.Motion.TimeNS != 0, nil
	}))
	return c, root, compiled, filepath.Join(dir, "telemetry.ring")
}

// profileRejected checks the catalogue label while retaining contextual details;
// a transport failure can never stand in for an admission refusal.
func profileRejected(t *testing.T, err error, symbol string) *control.Rejected {
	t.Helper()
	label := profileCatalogue(t)[symbol]
	var rejected *control.Rejected
	if label == "" || !errors.As(err, &rejected) || strings.SplitN(rejected.Reason, ":", 2)[0] != label {
		t.Fatalf("want %s (%s), got %v", symbol, label, err)
	}
	return rejected
}

// profileReceipt requires a typed successful command envelope and native sequence;
// a syntactically valid empty response is not acceptance.
func profileReceipt(t *testing.T, response control.Response, err error, operation string) {
	t.Helper()
	simfixture.Must(t, err)
	if response.Schema != "rosie.rt-control.response.v1" || response.Operation != operation || response.Sequence == 0 || response.Error != "" {
		t.Fatalf("%s receipt: %+v", operation, response)
	}
}

// TestMotionServerIntegrationProfile is an executable consumer sequence. It
// observes completion and inhibition, never inferring either from elapsed time.
func TestMotionServerIntegrationProfile(t *testing.T) {
	c, root, compiled, ringPath := profileService(t)
	ring, err := ipcclient.OpenTelemetry(ringPath)
	simfixture.Must(t, err)
	defer ring.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 60000*time.Millisecond)
	defer cancel()
	t.Cleanup(func() {
		cleanup, cancel := context.WithTimeout(context.Background(), profileObservation)
		defer cancel()
		c.StopRenewal()
		if c.Fence().Session != "" {
			_, _ = c.Stop(cleanup)
			_, _ = c.Release(cleanup)
		}
	})
	description, err := c.Describe(ctx)
	simfixture.Must(t, err)
	schema, err := os.ReadFile(filepath.Join(root, "protocol/application-v1.schema.json"))
	simfixture.Must(t, err)
	if description.ContractVersion != 2 || description.CapabilitiesDigest != fmt.Sprintf("%x", sha256.Sum256(schema)) || description.ConfigurationSHA256 != filepath.Base(compiled) {
		t.Fatalf("incompatible description: %+v", description)
	}
	argvBytes, err := os.ReadFile(filepath.Join(compiled, "argv.json"))
	simfixture.Must(t, err)
	var argv []string
	simfixture.Must(t, json.Unmarshal(argvBytes, &argv))
	for flag, want := range map[string]string{"--machine-sha256": description.MachineSHA256, "--deployment-sha256": description.DeploymentSHA256} {
		found := false
		for i := 0; i+1 < len(argv); i++ {
			if argv[i] == flag && argv[i+1] == want && len(want) == 64 {
				found = true
			}
		}
		if !found {
			t.Fatalf("deployment does not bind %s=%s", flag, want)
		}
	}
	for operation := range profileDocumentOperations(t) {
		found := false
		for _, capability := range description.Capabilities {
			if capability.Name == operation && capability.State == "implemented" {
				found = true
			}
		}
		if !found {
			t.Fatalf("required capability unavailable: %s", operation)
		}
	}
	if len(description.Axes) != 9 || description.MaxTrajectoryPoints != 250000 || description.ResidentPlans != 3 {
		t.Fatalf("admission limits: %+v", description)
	}
	for _, axis := range description.Axes {
		if !axis.NativeHome || !axis.RequireHome || axis.MaxAcceleration == nil || *axis.MaxAcceleration != 1 || axis.MaxVelocity != .1 {
			t.Fatalf("axis contract: %+v", axis)
		}
	}
	binding := control.Binding{PairID: "bench", Revision: 7, MachineSHA256: description.MachineSHA256}
	grant, err := c.Acquire(ctx, "motion-server-profile", binding)
	simfixture.Must(t, err)
	if grant.Session == "" || grant.Generation != 1 || grant.LeaseMS != 500 || grant.Stopping || grant.Binding.MachineSHA256 != binding.MachineSHA256 {
		t.Fatalf("grant: %+v", grant)
	}
	renewed, err := c.Renew(ctx)
	simfixture.Must(t, err)
	if renewed.Session != grant.Session || renewed.Generation != 1 || renewed.DeadlineHostNS <= grant.DeadlineHostNS || renewed.Stopping {
		t.Fatalf("renew: %+v", renewed)
	}
	updates, err := c.StartRenewal(ctx, profileRenewal)
	simfixture.Must(t, err)
	checkRenewal := func() {
		select {
		case update := <-updates:
			simfixture.Must(t, update.Err)
			if update.Grant.Stopping && c.Grant().Stopping {
				t.Fatal("renewal is stopping; motion forbidden")
			}
		default:
		}
	}
	observe := func(predicate func(control.Status) bool) control.Status {
		t.Helper()
		deadline, cancel := context.WithTimeout(ctx, profileObservation)
		defer cancel()
		var state control.Status
		simfixture.Must(t, simfixture.Poll(deadline, profilePoll, func() (bool, error) {
			checkRenewal()
			var err error
			state, err = c.Status(deadline)
			if err == nil && state.Core.SafetyFaultMask != 0 {
				return false, fmt.Errorf("fault mask %d", state.Core.SafetyFaultMask)
			}
			return err == nil && predicate(state), err
		}))
		return state
	}
	// Required Home evidence gates Enable; resolve it before asking for Arm.
	_, err = c.Enable(ctx, 511)
	unhomed := profileRejected(t, err, "ReasonNativeRejected")
	if unhomed.NativeResult == nil || unhomed.NativeResult.Reason != 2 {
		t.Fatalf("unhomed Enable: %+v", unhomed)
	}
	response, err := c.Home(ctx, 511)
	profileReceipt(t, response, err, "home")
	observe(func(s control.Status) bool {
		return s.Core.HomeValidMask == 511 && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.CommissioningPhase == 0
	})
	// This fixture has no persisted absolute anchor. Rejection must preserve Home.
	_, err = c.Command(ctx, &control.Request{Operation: "restore_anchor", AxisMask: 511})
	profileRejected(t, err, "ReasonAnchorMissing")
	response, err = c.Command(ctx, &control.Request{Operation: "recovery_status"})
	simfixture.Must(t, err)
	// The SDK exposes the recovery wire type through Status.Core.Recovery.
	var recovery control.Status
	simfixture.Must(t, json.Unmarshal(response.Data, &recovery.Core.Recovery))
	if response.Operation != "recovery_status" || recovery.Core.Recovery == nil || recovery.Core.Recovery.HomeValidMask != 511 || len(recovery.Core.Recovery.Faults) != 0 {
		t.Fatalf("recovery: %s", response.Data)
	}
	readProgram := func(name string) []byte {
		raw, err := os.ReadFile(filepath.Join(root, "clients/cpp/examples/testdata", name+".rdt"))
		simfixture.Must(t, err)
		return raw
	}
	// The invalid fixture requests 5 rad/s^2 against the declared 1 rad/s^2.
	program, err := c.PrepareProgram(ctx, readProgram("acceleration"))
	rejected := profileRejected(t, err, "ReasonNativeLimitExceeded")
	if rejected.NativeResult != nil || program.Identity.PlanID != "" || !strings.Contains(rejected.Reason, "segment=0 sample=1 axis=J1") {
		t.Fatalf("acceleration rejection: %+v %+v", rejected, program)
	}
	state, err := c.Status(ctx)
	simfixture.Must(t, err)
	if len(state.Execution.Handles) != 0 {
		t.Fatalf("rejected acceleration left handles: %+v", state.Execution)
	}
	_, err = c.PrepareProgram(ctx, readProgram("velocity"))
	profileRejected(t, err, "ReasonNativeLimitExceeded")
	program, err = c.PrepareProgram(ctx, readProgram("valid"))
	simfixture.Must(t, err)
	if program.Samples != 11 || program.NormalisedSamples != 11 || program.Segments != 1 || program.AxisMask != 511 || program.RequiresProcessIO || program.Identity.ManifestRevision != 7 || program.Identity.SourceDigest == "" || program.Identity.NormalisedDigest == "" {
		t.Fatalf("program: %+v", program)
	}
	// A definite not-ready refusal preserves the same prepared handle for retry.
	_, err = c.StartProgram(ctx, program.Identity)
	profileRejected(t, err, "ReasonNotReady")
	state, err = c.Status(ctx)
	simfixture.Must(t, err)
	if len(state.Execution.Handles) != 1 || state.Execution.Handles[0].State != "prepared" || state.Execution.Generation != 0 {
		t.Fatalf("rejected Start consumed preparation: %+v", state.Execution)
	}
	response, err = c.Enable(ctx, 511)
	profileReceipt(t, response, err, "enable")
	response, err = c.Arm(ctx)
	profileReceipt(t, response, err, "arm")
	observe(func(s control.Status) bool {
		if s.Core.Armed != 1 || s.Core.AxisEnableMask != 511 || len(s.Core.Axes) != 9 {
			return false
		}
		for _, axis := range s.Core.Axes {
			if axis.Readiness != "ready" {
				return false
			}
		}
		return true
	})
	batch, err := c.EventsContext(ctx, 0)
	simfixture.Must(t, err)
	cursor, incarnation := batch.NextSequence, batch.AdapterIncarnation
	telemetry, err := c.TelemetryBatches(ctx, 0)
	simfixture.Must(t, err)
	telemetryCursor := telemetry.Header.LastSequence
	telemetryCount := 0
	checkTelemetry := func() {
		b, e := c.TelemetryBatches(ctx, telemetryCursor)
		simfixture.Must(t, e)
		if len(b.Records) > 0 {
			if b.Header.FirstSequence != telemetryCursor+b.Header.Dropped+1 {
				t.Fatal("telemetry discontinuity")
			}
			var records []ipcclient.CycleCaptureRecordV2
			deadline, cancel := context.WithTimeout(ctx, profileObservation)
			err := simfixture.Poll(deadline, profilePoll, func() (bool, error) {
				observed, err := ring.ReadFrom(b.Header.FirstSequence+uint64(len(records)), len(b.Records)-len(records))
				if errors.Is(err, ipcclient.ErrTelemetryBusy) {
					return false, nil
				}
				if err != nil {
					return false, err
				}
				if observed.Drops != 0 {
					t.Fatal("live ring lost conformance records")
				}
				records = append(records, observed.Records...)
				return len(records) == len(b.Records), nil
			})
			cancel()
			simfixture.Must(t, err)
			if !reflect.DeepEqual(b.Records, records) {
				t.Fatal("public telemetry differs from live ring")
			}
			telemetryCount += len(b.Records)
		}
		telemetryCursor = b.Header.LastSequence
	}
	start := control.Request{Operation: "start_program", Identity: program.Identity}
	response, err = c.Command(ctx, &start)
	profileReceipt(t, response, err, "start_program")
	replayed, err := c.Command(ctx, &start)
	profileReceipt(t, replayed, err, "start_program")
	if start.RequestID == nil || replayed.Sequence != response.Sequence {
		t.Fatal("request_id replay double-started")
	}
	completed := false
	eventCtx, eventCancel := context.WithTimeout(ctx, profileObservation)
	defer eventCancel()
	simfixture.Must(t, simfixture.Poll(eventCtx, profilePoll, func() (bool, error) {
		checkRenewal()
		checkTelemetry()
		batch, err := c.EventsContext(eventCtx, cursor)
		if err != nil {
			return false, err
		}
		if batch.AdapterIncarnation != incarnation {
			return false, errors.New("adapter incarnation changed; re-describe")
		}
		for _, event := range batch.Events {
			if event.Type == "events_dropped" || event.Type == "publisher_overflow" {
				return false, errors.New("event history lost; reconcile status before motion")
			}
			if event.Sequence != cursor+1 {
				return false, fmt.Errorf("event sequence %d after %d", event.Sequence, cursor)
			}
			cursor = event.Sequence
			if event.Type == "execution_completed" && event.Execution != nil && event.Execution.Generation == 1 && event.Execution.NativeSequence == response.Sequence {
				completed = true
			}
		}
		return completed, nil
	}))
	state = observe(func(s control.Status) bool { return s.Execution.State == "completed" && s.Execution.Generation == 1 })
	checkTelemetry()
	if telemetryCount < 2 {
		t.Fatalf("program telemetry records: %d; need at least 2", telemetryCount)
	}
	if state.Core.Axes[0].PositionCounts < 19 || state.Core.Axes[0].PositionCounts > 23 {
		t.Fatalf("endpoint %d counts; want 19..23", state.Core.Axes[0].PositionCounts)
	}
	stopped, err := c.Stop(ctx)
	simfixture.Must(t, err)
	if stopped.Generation != 2 || stopped.Session != grant.Session || stopped.Stopping {
		t.Fatalf("stop: %+v", stopped)
	}
	observe(func(s control.Status) bool { return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 })
	_, err = c.PrepareProgram(ctx, readProgram("valid"))
	simfixture.Must(t, err)
	c.StopRenewal()
	updates = nil
	// Expiry is proved by inhibited outputs and a retired pending handle, not a sleep.
	observe(func(s control.Status) bool {
		if s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 || s.Execution.State != "cancelled" {
			return false
		}
		for _, handle := range s.Execution.Handles {
			if handle.State == "prepared" || handle.State == "started" {
				return false
			}
		}
		return true
	})
	_, err = c.Renew(ctx)
	profileRejected(t, err, "ReasonExpired")
	_, err = c.Arm(ctx)
	profileRejected(t, err, "ReasonControlSessionStale")
	stopped, err = c.Stop(ctx)
	simfixture.Must(t, err)
	if stopped.Session != "" {
		t.Fatalf("expired Stop restored authority: %+v", stopped)
	}
	successor, err := c.Acquire(ctx, "motion-server-profile", binding)
	simfixture.Must(t, err)
	// An expired Stop either joins expiry's generation 3 receipt or advances to 4.
	if successor.Session == "" || successor.Session == grant.Session ||
		!((stopped.Generation == 3 && successor.Generation == 4) || (stopped.Generation == 4 && successor.Generation == 5)) {
		t.Fatalf("successor: %+v", successor)
	}
	released, err := c.Release(ctx)
	simfixture.Must(t, err)
	if released.Session != "" || c.Fence().Session != "" {
		t.Fatalf("release retained authority: %+v", released)
	}
}
