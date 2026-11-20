//go:build linux

package denseexec

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"reflect"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type fakeRTCore struct {
	mu             sync.Mutex
	description    control.Description
	status         control.Status
	grant          control.Grant
	program        control.Program
	started        control.Identity
	calls          []string
	updates        chan control.Renewal
	changed        chan struct{}
	prepareEntered chan struct{}
	prepareWait    chan struct{}
	acquireEntered chan struct{}
	acquireWait    chan struct{}
	acquireErr     error
	renewEntered   chan struct{}
	renewWait      chan struct{}
	renewCancel    context.CancelFunc
	renewDone      chan struct{}
	renewStarts    int
	renewActive    int
	refuse         string
	homeMasks      []uint32
	statusErr      error
	statusGate     func() // called before each Status, outside f.mu
	stopErr        error
	releaseErr     error
	event          *control.Event
	resources      map[string][]byte // digest -> bytes, served by Resource
	resourceReads  int
}

// Pinned discovery inputs remain mandatory on reacquisition, preventing a
// remote listener restart from silently changing the selected deployment.
func TestRTCoreRemoteDescriptionPins(t *testing.T) {
	r, f, _ := rtFake(t)
	r.cfg.ExpectedBackend = "ethercat"
	r.cfg.Binding.ConfigurationSHA256 = strings.Repeat("d", 64)
	for _, backend := range []string{"simulation", "ethercat"} {
		f.mu.Lock()
		f.description.Backend = backend
		f.mu.Unlock()
		_, err := r.describe(context.Background())
		var refused *control.Rejected
		if !errors.As(err, &refused) || refused.Reason != "rt_core_identity_mismatch" {
			t.Fatalf("unmatched pin: %v", err)
		}
	}
	f.mu.Lock()
	f.description.ConfigurationSHA256 = strings.Repeat("d", 64)
	f.mu.Unlock()
	if _, err := r.describe(context.Background()); err != nil {
		t.Fatal(err)
	}
}

// Capabilities publish a host and description only, never client credentials.
func TestRTCoreRemoteManifestDisplay(t *testing.T) {
	r, _, _ := rtFake(t)
	r.cfg.Address = "https://127.0.0.1:8443"
	r.cfg.CAFile, r.cfg.CertificateFile, r.cfg.KeyFile = "/private/ca", "/private/cert", "/private/key"
	caps := r.Capabilities(context.Background())
	if caps.RTCore == nil || caps.RTCore.AddressHost != "127.0.0.1:8443" {
		t.Fatalf("remote manifest: %+v", caps)
	}
	raw, err := json.Marshal(caps)
	if err != nil || strings.Contains(string(raw), "/private") {
		t.Fatalf("credential disclosure: %s %v", raw, err)
	}
	for _, address := range []string{"/private/key", "https://secret@host", "https://host/private/key", "https://host?key=secret", "https://host#secret"} {
		if rtCoreAddressHost(address) != "" {
			t.Fatalf("unsafe display address %q", address)
		}
	}
}

func rtFake(t *testing.T) (*RTCoreController, *fakeRTCore, Identity) {
	t.Helper()
	blob, plan := fixtureBlob(t)
	id := fixtureIdentity(plan)
	f := &fakeRTCore{updates: make(chan control.Renewal, 8), changed: make(chan struct{}, 64)}
	f.description.ContractVersion = 2
	f.description.CapabilitiesDigest = strings.Repeat("c", 64)
	f.description.MachineSHA256 = strings.Repeat("a", 64)
	f.description.DeploymentSHA256 = strings.Repeat("b", 64)
	f.description.Axes = make([]ipcclient.AxisDescription, 9)
	for i := range f.description.Axes {
		f.description.Axes[i].ID = fmt.Sprintf("J%d", i+1)
		f.description.Axes[i].PositionUnit = "rad"
	}
	// A flat machine with its compiled configuration loaded: it describes its
	// drives and binds no robot. An adapter that describes neither is one
	// started without --compiled-config, which Load refuses (rt_core_robot.go).
	f.description.Drives = []control.DriveDescription{{Axis: "J1", ConfigName: "cia402_sim", ConfigSHA256: strings.Repeat("d", 64)}}
	for _, name := range []string{"acquire", "renew", "release", "enable", "arm", "home", "stop", "prepare_program", "start_program", "status", "subscribe_events"} {
		f.description.Capabilities = append(f.description.Capabilities, control.CapabilityInfo{Name: name, State: "implemented"})
	}
	f.status.AdapterIncarnation, f.status.DaemonIncarnation = "adapter", "daemon"
	f.status.TimeNS, f.status.Motion.TimeNS = 1, 1
	f.status.Core.HomeValidMask = 511
	f.status.Axes = make([]ipcclient.LogicalAxisStatus, 9)
	for i := range f.status.Axes {
		f.status.Axes[i] = ipcclient.LogicalAxisStatus{LogicalValid: true, Readiness: "ready", HomeValid: true}
	}
	f.program = control.Program{AxisMask: 511, Identity: control.Identity{PlanID: id.PlanID, ProgramID: id.ProgramID, ProgramDigest: id.ProgramDigest, TrajectoryDigest: id.TrajectoryDigest, ManifestRevision: id.ManifestRevision, PlanRevision: id.PlanRevision, SourceDigest: id.TrajectoryDigest, NormalisedDigest: strings.Repeat("e", 64)}}
	r, err := NewRTCore(RTCoreConfig{Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: f.description.MachineSHA256}, DeploymentSHA256: f.description.DeploymentSHA256, CapabilitiesDigest: f.description.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:test", FetchBlob: func(context.Context, string) ([]byte, error) { return blob, nil }}, f)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	return r, f, id
}

func (f *fakeRTCore) record(name string) {
	f.calls = append(f.calls, name)
	select {
	case f.changed <- struct{}{}:
	default:
	}
}
func (f *fakeRTCore) Describe(context.Context) (control.Description, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.description, nil
}
func (f *fakeRTCore) Status(context.Context) (control.Status, error) {
	if f.statusGate != nil {
		f.statusGate()
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.status, f.statusErr
}
func (f *fakeRTCore) EventsContext(_ context.Context, after uint64) (control.EventBatch, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	b := control.EventBatch{AdapterIncarnation: "adapter", NextSequence: after, LatestSequence: after}
	if f.event != nil {
		b.Events = []control.Event{*f.event}
		b.NextSequence = after + 1
		b.LatestSequence = after + 1
	}
	return b, nil
}
func (f *fakeRTCore) Acquire(_ context.Context, _ string, binding control.Binding) (control.Grant, error) {
	f.mu.Lock()
	f.record("acquire")
	entered, wait := f.acquireEntered, f.acquireWait
	f.mu.Unlock()
	if entered != nil {
		close(entered)
		<-wait
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.acquireErr != nil {
		return control.Grant{}, f.acquireErr
	}
	f.grant = control.Grant{Session: "grant", Generation: 1, LeaseMS: 500, Binding: binding}
	f.status.Execution = control.Execution{}
	return f.grant, nil
}
func (f *fakeRTCore) Grant() control.Grant { f.mu.Lock(); defer f.mu.Unlock(); return f.grant }
func (f *fakeRTCore) StartRenewal(ctx context.Context, _ time.Duration) (<-chan control.Renewal, error) {
	f.mu.Lock()
	ctx, f.renewCancel = context.WithCancel(ctx)
	f.renewDone = make(chan struct{})
	done, entered, wait := f.renewDone, f.renewEntered, f.renewWait
	f.renewStarts++
	f.renewActive++
	f.mu.Unlock()
	go func() {
		<-ctx.Done()
		f.mu.Lock()
		f.renewActive--
		f.mu.Unlock()
		close(done)
	}()
	if entered != nil {
		close(entered)
		<-wait
	}
	return f.updates, nil
}
func (f *fakeRTCore) StopRenewal() {
	f.mu.Lock()
	cancel, done := f.renewCancel, f.renewDone
	f.mu.Unlock()
	if cancel != nil {
		cancel()
		<-done
	}
}
func (f *fakeRTCore) Close() error { return nil }
func (f *fakeRTCore) Command(ctx context.Context, request *control.Request) (control.Response, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if err := ctx.Err(); err != nil {
		return control.Response{}, err
	}
	f.record(request.Operation)
	if request.Operation == f.refuse {
		return control.Response{}, &control.Rejected{Reason: "native_rejected: axis 1"}
	}
	switch request.Operation {
	case "arm":
		f.status.Core.Armed = 1
	case "reset_fault":
		f.status.Core.SafetyFaultMask = 0
		f.status.Core.ExecutionFaultReasons = 0
		f.status.Core.Recovery = nil
		f.status.Execution.Error = ""
		for i := range f.status.Axes {
			f.status.Axes[i].Readiness = "ready"
		}
	case "home":
		f.status.Core.HomeValidMask |= request.AxisMask
		f.homeMasks = append(f.homeMasks, request.AxisMask)
	case "start_program":
		f.started = request.Identity
		f.status.Execution = control.Execution{Identity: request.Identity, State: "executing", Generation: 1, NativeSequence: 1}
	}
	return control.Response{Operation: request.Operation, Sequence: 1}, nil
}
func (f *fakeRTCore) PrepareProgram(ctx context.Context, _ []byte) (control.Program, error) {
	f.mu.Lock()
	f.record("prepare_program")
	entered, wait := f.prepareEntered, f.prepareWait
	p := f.program
	f.mu.Unlock()
	if entered != nil {
		close(entered)
		select {
		case <-wait:
		case <-ctx.Done():
			return control.Program{}, ctx.Err()
		}
	}
	return p, nil
}
func (f *fakeRTCore) Stop(context.Context) (control.Grant, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.record("stop")
	if f.stopErr != nil {
		return control.Grant{}, f.stopErr
	}
	f.grant.Generation++
	f.status.Core.Armed = 0
	f.status.Execution.State = "cancelled"
	return f.grant, nil
}
func (f *fakeRTCore) Release(context.Context) (control.Grant, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.record("release")
	if f.releaseErr != nil {
		return control.Grant{}, f.releaseErr
	}
	f.grant = control.Grant{}
	return f.grant, nil
}

func rtAwait(t *testing.T, predicate func() bool) {
	t.Helper()
	// Unverified test observation ceiling and cadence; only predicate success passes.
	ctx, cancel := context.WithTimeout(context.Background(), 3000*time.Millisecond)
	defer cancel()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	for !predicate() {
		select {
		case <-ctx.Done():
			t.Fatal("observation not reached")
		case <-ticker.C:
		}
	}
}

func TestRTCoreIdentityCompletionAndExternalStop(t *testing.T) {
	r, f, id := rtFake(t)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	if f.started != f.program.Identity {
		t.Fatal("full source and normalised identity were not echoed")
	}
	f.status.Execution.State = "completed"
	f.mu.Unlock()
	rtAwait(t, func() bool { return r.Status(ctx).Session.State == "done" })
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	calls := append([]string(nil), f.calls...)
	f.mu.Unlock()
	if !reflect.DeepEqual(calls, []string{"acquire", "enable", "arm", "prepare_program", "start_program", "stop", "release"}) {
		t.Fatalf("calls: %v", calls)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err == nil {
		t.Fatal("stale prepared session replayed")
	}
}

func TestRTCoreLossStopsAndSurfacesReason(t *testing.T) {
	for _, cause := range []string{"renewal", "generation", "transport", "fault", "restart", "events", "heartbeat", "external_stop"} {
		t.Run(cause, func(t *testing.T) {
			r, f, id := rtFake(t)
			loaded, err := r.Load(context.Background(), LoadRequest{Identity: id})
			if err != nil {
				t.Fatal(err)
			}
			if _, err := r.Arm(context.Background(), true); err != nil {
				t.Fatal(err)
			}
			if _, err := r.Play(context.Background(), loaded.SessionID); err != nil {
				t.Fatal(err)
			}
			f.mu.Lock()
			switch cause {
			case "renewal":
				f.updates <- control.Renewal{Err: &control.Rejected{Reason: "control_session_stale"}}
			case "generation":
				f.updates <- control.Renewal{Grant: control.Grant{Session: "grant", Generation: 2}}
			case "transport":
				f.statusErr = control.ErrTransport
				f.updates <- control.Renewal{Err: control.ErrTransport}
			case "fault":
				f.status.Core.SafetyFaultMask = 4
			case "restart":
				f.status.DaemonIncarnation = "replacement"
			case "events":
				f.event = &control.Event{Type: "events_dropped"}
			case "external_stop":
				f.status.Execution.State = "cancelled"
			}
			f.mu.Unlock()
			if cause == "heartbeat" {
				r.mu.Lock()
				r.lastHeartbeat = time.Now().Add(-6000 * time.Millisecond)
				r.mu.Unlock()
			}
			rtAwait(t, func() bool {
				f.mu.Lock()
				defer f.mu.Unlock()
				return len(f.calls) >= 7 && f.calls[len(f.calls)-1] == "release"
			})
			status := r.Status(context.Background())
			if status.Session.State != "stopped" || status.Session.Lease.Held || status.RTCore.Reason == "" {
				t.Fatalf("status: %+v", status)
			}
		})
	}
}

// Play on a loaded but disarmed robot is refused at once with its own code;
// it used to wait for an Arm that its operation lock kept out, for the whole
// operation budget, and fail with whatever went stale first.
func TestRTCorePlayWhileDisarmedRefusesAtOnce(t *testing.T) {
	r, f, id := rtFake(t)
	loaded, err := r.Load(context.Background(), LoadRequest{Identity: id})
	if err != nil {
		t.Fatal(err)
	}
	began := time.Now()
	_, err = r.Play(context.Background(), loaded.SessionID)
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "play_requires_arm" || failure.Status != 409 {
		t.Fatalf("play while disarmed: %v", err)
	}
	// Unverified ceiling: 1000 ms, against the 30 s operation budget it used to wait out.
	if elapsed := time.Since(began); elapsed > 1000*time.Millisecond {
		t.Fatalf("refused after %s", elapsed)
	}
	f.mu.Lock()
	calls := append([]string(nil), f.calls...)
	f.mu.Unlock()
	if rtCallIndex(calls, "start_program") >= 0 {
		t.Fatalf("start_program sent while disarmed: %v", calls)
	}
	if s := r.Status(context.Background()).Session; s == nil || s.State != "loaded" {
		t.Fatalf("the loaded session did not survive the refusal: %+v", s)
	}
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(context.Background(), loaded.SessionID); err != nil {
		t.Fatalf("play once armed: %v", err)
	}
}

func TestRTCoreUnacknowledgedStopDoesNotRelease(t *testing.T) {
	r, f, _ := rtFake(t)
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.stopErr = &control.Rejected{Reason: "control_session_stale"}
	f.mu.Unlock()
	result, err := r.Stop(context.Background())
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "control_session_stale" || result.Stopped {
		t.Fatalf("stop: %+v %v", result, err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if !reflect.DeepEqual(f.calls, []string{"acquire", "enable", "arm", "stop"}) {
		t.Fatal(f.calls)
	}
}

func TestRTCoreUncertainStopInhibitsMotionUntilRecovery(t *testing.T) {
	for _, failureAt := range []string{"stop", "release"} {
		t.Run(failureAt, func(t *testing.T) {
			r, f, id := rtFake(t)
			ctx := context.Background()
			if _, err := r.Arm(ctx, true); err != nil {
				t.Fatal(err)
			}
			f.mu.Lock()
			if failureAt == "stop" {
				f.stopErr = control.ErrTransport
			} else {
				f.releaseErr = control.ErrTransport
			}
			f.mu.Unlock()
			if _, err := r.Stop(ctx); err == nil {
				t.Fatal("uncertain Stop/Release accepted")
			}
			reason := r.Status(ctx).RTCore.Reason
			observation := r.Status(ctx).RTCore
			if observation.Inhibited != (failureAt == "release") || observation.StopUncertain != (failureAt == "stop") ||
				!strings.Contains(observation.RecoveryAction, "POST /dense-execution/stop") || !strings.Contains(observation.RecoveryAction, "without a server restart") {
				t.Fatalf("recovery status: %+v", observation)
			}
			raw, err := json.Marshal(r.Status(ctx))
			if err != nil || !strings.Contains(string(raw), `"inhibited":`) || !strings.Contains(string(raw), `"stop_uncertain":`) || !strings.Contains(string(raw), `"recovery_action":`) {
				t.Fatalf("status JSON: %s %v", raw, err)
			}
			f.mu.Lock()
			f.stopErr, f.releaseErr = nil, nil
			f.mu.Unlock()
			for _, motion := range []struct {
				name string
				run  func() error
			}{
				{"arm", func() error { _, err := r.Arm(ctx, true); return err }},
				{"home", func() error { return r.Home(ctx) }},
				{"load", func() error { _, err := r.Load(ctx, LoadRequest{Identity: id}); return err }},
				{"play", func() error { _, err := r.Play(ctx, "stale"); return err }},
			} {
				var failure *Error
				if err := motion.run(); !errors.As(err, &failure) || failure.Code != "rt_core_inhibited" {
					t.Fatalf("%s refusal: %v", motion.name, err)
				}
				if !strings.Contains(failure.Detail, "POST /dense-execution/stop") || !strings.Contains(failure.Detail, "server restart") {
					t.Fatalf("unactionable refusal: %s", failure.Detail)
				}
			}
			status := r.Status(ctx)
			if status.RTCore.Reason != reason || status.Session.Lease.Held || status.Session.State != "stopped" {
				t.Fatalf("uncertainty cleared without recovery: %+v", status)
			}
			f.mu.Lock()
			calls := append([]string(nil), f.calls...)
			f.mu.Unlock()
			want := []string{"acquire", "enable", "arm", "stop"}
			if failureAt == "release" {
				want = append(want, "release")
			}
			if !reflect.DeepEqual(calls, want) {
				t.Fatalf("inhibited motion sent requests: %v", calls)
			}
			if result, err := r.Stop(ctx); err != nil || !result.Stopped {
				t.Fatalf("recovery: %+v %v", result, err)
			}
			if recovered := r.Status(ctx).RTCore; recovered.Inhibited || recovered.StopUncertain || recovered.RecoveryAction != "" {
				t.Fatalf("recovery remained latched: %+v", recovered)
			}
			if _, err := r.Arm(ctx, true); err != nil {
				t.Fatalf("explicit recovery did not permit Arm: %v", err)
			}
			f.mu.Lock()
			defer f.mu.Unlock()
			want = append(want, "stop", "release", "acquire", "enable", "arm")
			if !reflect.DeepEqual(f.calls, want) {
				t.Fatalf("recovery requests: %v", f.calls)
			}
		})
	}
}

func TestRTCoreHomeAndReturnedIdentity(t *testing.T) {
	r, f, id := rtFake(t)
	if err := r.Home(context.Background()); err != nil {
		t.Fatal(err)
	}
	if s := r.Status(context.Background()); s.RTCore.Status.Core.HomeValidMask != 511 || s.Session.ID == "" {
		t.Fatalf("home observation: %+v", s)
	}
	f.mu.Lock()
	f.program.Identity.PlanRevision++
	f.mu.Unlock()
	if _, err := r.Load(context.Background(), LoadRequest{Identity: id}); err == nil {
		t.Fatal("wrong returned Program identity accepted")
	}
}

// One joint can be re-homed on its own: the native Home carries that axis
// alone, the wait is for that axis, the others keep their Home. A mask outside
// the configured axes, or an empty one, is a typed refusal before any command.
func TestRTCoreHomeAxesHomesOneJoint(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	f.status.Core.HomeValidMask = 0
	f.mu.Unlock()
	if err := r.HomeAxes(context.Background(), 1<<3); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	masks, valid := append([]uint32(nil), f.homeMasks...), f.status.Core.HomeValidMask
	f.mu.Unlock()
	if !reflect.DeepEqual(masks, []uint32{1 << 3}) || valid != 1<<3 {
		t.Fatalf("home masks %v valid %#x", masks, valid)
	}
	for _, bad := range []uint32{0, 1 << 9, 511 | 1<<12} {
		err := r.HomeAxes(context.Background(), bad)
		var failure *Error
		if !errors.As(err, &failure) || failure.Code != "home_axes_invalid" {
			t.Fatalf("mask %#x: %v", bad, err)
		}
	}
	f.mu.Lock()
	sent := len(f.homeMasks)
	f.mu.Unlock()
	if sent != 1 {
		t.Fatalf("a refused mask must not reach the machine: %d home commands", sent)
	}
}

func TestRTCoreConfigRejectsUnknownAndTrailingFields(t *testing.T) {
	for _, raw := range []string{`{"legacy_bridge":true}`, `{} {}`, `{} garbage`} {
		if _, err := DecodeRTCoreConfig([]byte(raw)); err == nil {
			t.Fatalf("accepted ambiguous config %s", raw)
		}
	}
}

func TestRTCoreRefusalStopsBeforeRelease(t *testing.T) {
	r, f, _ := rtFake(t)
	f.refuse = "arm"
	_, err := r.Arm(context.Background(), true)
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "native_rejected" || !strings.Contains(failure.Detail, "axis 1") {
		t.Fatalf("typed refusal: %v", err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if !reflect.DeepEqual(f.calls, []string{"acquire", "enable", "arm", "stop", "release"}) {
		t.Fatal(f.calls)
	}
}

func TestRTCoreLocalRefusalsPreserveTypedReason(t *testing.T) {
	for _, name := range []string{
		"pause", "go_home", "capability_unimplemented", "rt_core_identity_mismatch",
		"dense_identity_invalid", "dense_trajectory_invalid", "dense_identity_mismatch",
		"dense_blob_source_unavailable", "program_identity_or_process_mismatch",
		"control_session_stale", "program_already_executing", "rt_core_protocol_error",
		"rt_core_incarnation_changed", "rt_core_fault", "axis_fault", "rt_core_status_stale",
		"program_identity_mismatch", "execution_sequence_mismatch", "rt_core_readiness_lost",
		"execution_interrupted", "execution_error",
	} {
		t.Run(name, func(t *testing.T) {
			r, f, id := rtFake(t)
			// Drive observations explicitly so each refusal is returned to its
			// caller instead of racing the watcher's asynchronous Stop.
			r.cancel()
			<-r.done
			ctx := context.Background()
			code, status := name, 409
			var err error
			switch name {
			case "pause":
				code = "capability_unimplemented"
				f.stopErr = control.ErrTransport
				_, err = r.Pause(ctx, "")
			case "go_home":
				code = "capability_unimplemented"
				_, err = r.GoHome(ctx)
			case "capability_unimplemented":
				f.description.Capabilities = nil
				_, err = r.Arm(ctx, true)
			case "rt_core_identity_mismatch":
				f.description.MachineSHA256 = strings.Repeat("f", 64)
				_, err = r.Arm(ctx, true)
			case "dense_identity_invalid":
				status = 400
				_, err = r.Load(ctx, LoadRequest{})
			case "dense_trajectory_invalid":
				status = 400
				r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return nil, nil }
				_, err = r.Load(ctx, LoadRequest{Identity: id})
			case "dense_identity_mismatch":
				id.PlanID = "different-plan"
				_, err = r.Load(ctx, LoadRequest{Identity: id})
			case "dense_blob_source_unavailable":
				status = 503
				r.source.cfg.FetchBlob = nil
				_, err = r.Load(ctx, LoadRequest{Identity: id})
			case "program_identity_or_process_mismatch":
				f.program.RequiresProcessIO = true
				_, err = r.Load(ctx, LoadRequest{Identity: id})
			case "control_session_stale":
				_, err = r.Play(ctx, "unknown")
			case "rt_core_protocol_error":
				f.status.TimeNS = 0
				_, err = r.Arm(ctx, true)
			default:
				if _, err := r.Arm(ctx, true); err != nil {
					t.Fatal(err)
				}
				loaded, errLoad := r.Load(ctx, LoadRequest{Identity: id})
				if errLoad != nil {
					t.Fatal(errLoad)
				}
				if _, err := r.Play(ctx, loaded.SessionID); err != nil {
					t.Fatal(err)
				}
				switch name {
				case "program_already_executing":
					_, err = r.Load(ctx, LoadRequest{Identity: id})
				case "rt_core_incarnation_changed":
					f.status.DaemonIncarnation = "replacement"
				case "rt_core_fault":
					f.status.Core.SafetyFaultMask = 4
				case "axis_fault":
					code = "rt_core_fault"
					f.status.Axes[0].Readiness = "faulted"
				case "rt_core_status_stale":
					r.nativeSeen = time.Now().Add(-1000 * time.Millisecond)
				case "program_identity_mismatch":
					f.status.Execution.Identity.PlanRevision++
				case "execution_sequence_mismatch":
					f.status.Execution.NativeSequence = 2
				case "rt_core_readiness_lost":
					f.status.Axes[0].Readiness = "inhibited"
				case "execution_interrupted":
					f.status.Execution.State = "cancelled"
				case "execution_error":
					code = "native_rejected"
					f.status.Execution.Error = "native_rejected: execution failed"
				}
				if name != "program_already_executing" {
					err = r.operation(ctx, r.observe)
				}
			}
			var failure *Error
			if !errors.As(err, &failure) || failure.Code != code || failure.Status != status {
				t.Fatalf("typed refusal: %v; want %s HTTP %d", err, code, status)
			}
			if !strings.Contains(failure.Detail, strings.SplitN(r.Status(ctx).RTCore.Reason, ":", 2)[0]) {
				t.Fatalf("diagnostic lost: %+v", failure)
			}
		})
	}
}

func TestRTCoreFailurePreservesSDKAndContextReasons(t *testing.T) {
	for _, tc := range []struct {
		cause error
		code  string
	}{
		{control.ErrTransport, "rt_core_request_slow"},
		{control.ErrProtocol, "rt_core_protocol_error"},
		{control.ErrClosed, "rt_core_closed"},
		{context.Canceled, "rt_core_cancelled"},
		{context.DeadlineExceeded, "rt_core_request_slow"},
		{rtRefused("control_renewal_lost"), "control_renewal_lost"},
		{rtRefused("ui_heartbeat_lost"), "ui_heartbeat_lost"},
		{rtRefused("rt_core_event: events_dropped"), "rt_core_event"},
	} {
		t.Run(tc.code, func(t *testing.T) {
			for _, stopErr := range []error{nil, rtRefused("native_rejected: Stop failed")} {
				r, f, _ := rtFake(t)
				if _, err := r.Arm(context.Background(), true); err != nil {
					t.Fatal(err)
				}
				f.mu.Lock()
				f.stopErr = stopErr
				f.mu.Unlock()
				var failure *Error
				if err := r.fail(tc.cause); !errors.As(err, &failure) || failure.Code != tc.code {
					t.Fatalf("typed failure: %v", err)
				}
				if tc.code == "rt_core_request_slow" {
					if !r.statusSnapshot().Session.Lease.Held {
						t.Fatal("request failure retired the grant")
					}
				} else if stopErr != nil && !strings.Contains(failure.Detail, "Stop failed") {
					t.Fatalf("Stop diagnostic lost: %+v", failure)
				}
			}
		})
	}
}

func TestRTCoreStopPreemptsPreparation(t *testing.T) {
	r, f, id := rtFake(t)
	f.prepareEntered, f.prepareWait = make(chan struct{}), make(chan struct{})
	done := make(chan error, 1)
	go func() { _, err := r.Load(context.Background(), LoadRequest{Identity: id}); done <- err }()
	select {
	case <-f.prepareEntered:
	case <-time.After(3000 * time.Millisecond):
		t.Fatal("preparation did not start")
	}
	if result, err := r.Stop(context.Background()); err != nil || !result.Stopped {
		t.Fatalf("stop: %+v %v", result, err)
	}
	select {
	case err := <-done:
		if err == nil {
			t.Fatal("cancelled upload resurrected session")
		}
	case <-time.After(3000 * time.Millisecond):
		t.Fatal("upload not cancelled")
	}
	if s := r.Status(context.Background()).Session; s != nil && s.TrajectoryDigest != "" {
		t.Fatal("late upload installed session")
	}
}

// rtEntryContext observes the first budget the operation establishes (Arm's
// status read, before it queues), allowing Stop to overtake it without
// scheduling sleeps.
type rtEntryContext struct {
	context.Context
	entered chan struct{}
	once    *sync.Once
}

func (c rtEntryContext) Deadline() (time.Time, bool) {
	c.once.Do(func() { close(c.entered) })
	return c.Context.Deadline()
}

func TestRTCoreQueuedArmBeforeStopIsRefused(t *testing.T) {
	r, f, _ := rtFake(t)
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	r.opMu.Lock()
	entered := make(chan struct{})
	done := make(chan error, 1)
	go func() {
		_, err := r.Arm(rtEntryContext{Context: context.Background(), entered: entered, once: &sync.Once{}}, true)
		done <- err
	}()
	select {
	case <-entered:
	case <-time.After(3000 * time.Millisecond):
		r.opMu.Unlock()
		t.Fatal("Arm did not enter operation queue")
	}
	result, err := r.Stop(context.Background())
	r.opMu.Unlock()
	if err != nil || !result.Stopped {
		t.Fatalf("Stop: %+v %v", result, err)
	}
	select {
	case err := <-done:
		var failure *Error
		if !errors.As(err, &failure) || failure.Code != "control_session_stale" {
			t.Fatalf("queued Arm lost typed refusal: %v", err)
		}
	case <-time.After(3000 * time.Millisecond):
		t.Fatal("queued Arm did not return")
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if !reflect.DeepEqual(f.calls, []string{"acquire", "enable", "arm", "stop", "release"}) {
		t.Fatalf("queued Arm sent requests: %v", f.calls)
	}
}

func TestRTCoreCancelledAcquisitionJoinsRenewal(t *testing.T) {
	for _, duringRenewal := range []bool{false, true} {
		name := "delayed_acquire_after_failed_stop"
		if duringRenewal {
			name = "cancel_during_keeper_start"
		}
		t.Run(name, func(t *testing.T) {
			r, f, _ := rtFake(t)
			entered, resume := make(chan struct{}), make(chan struct{})
			if duringRenewal {
				f.renewEntered, f.renewWait = entered, resume
			} else {
				f.acquireEntered, f.acquireWait = entered, resume
				f.stopErr = control.ErrTransport
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			done := make(chan error, 1)
			go func() { _, err := r.Arm(ctx, true); done <- err }()
			select {
			case <-entered:
			case <-time.After(3000 * time.Millisecond):
				t.Fatal("acquisition did not reach barrier")
			}
			if duringRenewal {
				cancel()
			} else if result, err := r.Stop(context.Background()); err == nil || result.Stopped {
				t.Fatalf("expected uncertain Stop: %+v %v", result, err)
			}
			close(resume)
			select {
			case err := <-done:
				if err == nil {
					t.Fatal("cancelled acquisition accepted")
				}
			case <-time.After(3000 * time.Millisecond):
				t.Fatal("cancelled acquisition did not return")
			}
			f.mu.Lock()
			defer f.mu.Unlock()
			wantStarts := 0
			if duringRenewal {
				wantStarts = 1
			}
			if f.renewStarts != wantStarts || f.renewActive != 0 {
				t.Fatalf("keeper starts=%d active=%d", f.renewStarts, f.renewActive)
			}
			if f.renewDone != nil {
				select {
				case <-f.renewDone:
				default:
					t.Fatal("keeper was not joined")
				}
			}
			wantCalls := []string{"acquire", "stop"}
			if duringRenewal {
				wantCalls = append(wantCalls, "release")
			}
			if !reflect.DeepEqual(f.calls, wantCalls) {
				t.Fatalf("requests after cancellation: %v", f.calls)
			}
			r.mu.Lock()
			defer r.mu.Unlock()
			if r.held || r.renewals != nil {
				t.Fatal("cancelled acquisition installed authority or renewal")
			}
		})
	}
}

func TestRTCoreRenewalLossDuringPreparationPreservesReason(t *testing.T) {
	r, f, id := rtFake(t)
	f.prepareEntered, f.prepareWait = make(chan struct{}), make(chan struct{})
	done := make(chan error, 1)
	go func() { _, err := r.Load(context.Background(), LoadRequest{Identity: id}); done <- err }()
	select {
	case <-f.prepareEntered:
	case <-time.After(3000 * time.Millisecond):
		t.Fatal("preparation did not start")
	}
	// Hold the urgent lane so cancellation must return its cause before Stop
	// can retire authority. This deterministically exercises publication order.
	r.stopMu.Lock()
	defer r.stopMu.Unlock()
	f.updates <- control.Renewal{Err: &control.Rejected{Reason: "control_session_stale"}}
	select {
	case err := <-done:
		var failure *Error
		if !errors.As(err, &failure) || failure.Code != "control_session_stale" {
			t.Fatalf("lost typed cause: %v", err)
		}
	case <-time.After(3000 * time.Millisecond):
		t.Fatal("renewal loss did not cancel upload")
	}
	if reason := r.Status(context.Background()).RTCore.Reason; reason != "control_session_stale" {
		t.Fatalf("operator reason: %s", reason)
	}
}

func TestRTCoreManifestAndIdentityRefuseBeforeMotion(t *testing.T) {
	r, f, id := rtFake(t)
	f.description.MachineSHA256 = strings.Repeat("f", 64)
	if _, err := r.Load(context.Background(), LoadRequest{Identity: id}); err == nil {
		t.Fatal("deployment mismatch accepted")
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	for _, call := range f.calls {
		if call != "stop" && call != "release" {
			t.Fatalf("unexpected command %s", call)
		}
	}
}

// A latched fault whose recovery policy is reset_clears (a missed cycle
// deadline on the simulator, a refused Start on the bench) is recovered by
// Arm: Arm's first step sends reset_fault from a fresh status before enable,
// and the mask is watched down to zero. A fault that needs a re-Home or a
// restart is refused before anything is sent. target_lead (AMR-720) is reset
// by Arm and never by Home.
func TestRTCoreArmResetsResettableFault(t *testing.T) {
	for _, tc := range []struct {
		name        string
		recovery    string
		armed       bool
		refuseReset bool
	}{{"reset_clears", "reset_clears", true, false}, {"following_error_cleared", "reset_after_condition_clears", true, false}, {"following_error_persists", "reset_after_condition_clears", false, true}, {"target_lead_cleared", "reset_after_condition_clears", true, false}, {"restart_required", "restart_required", false, false}} {
		t.Run(tc.name, func(t *testing.T) {
			r, c := resetFixture(t)
			f := c.fakeRTCore
			f.mu.Lock()
			f.status.Core.SafetyFaultMask = 4
			fault := ipcclient.FaultRecovery{Bit: 2, Name: "cycle_deadline", AxisMask: 511, Recovery: tc.recovery, Outcome: "persists"}
			if tc.name == "target_lead_cleared" {
				f.status.Core.ExecutionFaultReasons = 16
				fault = ipcclient.FaultRecovery{Bit: 4, Name: "target_lead", AxisMask: 4, Recovery: tc.recovery, Outcome: "persists"}
			}
			f.status.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{fault}}
			f.status.Execution.Error = "RTCore application grant outcome uncertain: RTCore upload interrupted by disarm"
			if tc.refuseReset {
				f.refuse = "reset_fault"
			}
			for i := range f.status.Axes {
				f.status.Axes[i].Readiness = "faulted"
			}
			f.mu.Unlock()
			_, err := r.Arm(context.Background(), true)
			f.mu.Lock()
			calls := append([]string(nil), f.calls...)
			f.mu.Unlock()
			reset, enable := rtCallIndex(calls, "reset_fault"), rtCallIndex(calls, "enable")
			if rtCallIndex(calls, "home") >= 0 {
				t.Fatalf("fault recovery ran Home: %v", calls)
			}
			if tc.armed {
				if err != nil {
					t.Fatalf("arm did not recover the fault: %v (calls %v)", err, calls)
				}
				if reset < 0 || enable < 0 || reset > enable {
					t.Fatalf("reset_fault must precede enable: %v", calls)
				}
				// The reset retires the grant: a fresh acquisition sits between it and Enable.
				if rtCallIndex(calls[reset:], "acquire") < 0 || reset+rtCallIndex(calls[reset:], "acquire") > enable {
					t.Fatalf("a fresh acquire must follow reset_fault before enable: %v", calls)
				}
				return
			}
			if tc.refuseReset {
				if err == nil || reset < 0 || enable >= 0 {
					t.Fatalf("rejected reset must block Enable: %v %v", calls, err)
				}
				return
			}
			var failure *Error
			if !errors.As(err, &failure) || failure.Code != "fault_not_resettable" || !strings.Contains(failure.Detail, "needs a controller restart") {
				t.Fatalf("expected a fault_not_resettable refusal, got %v", err)
			}
			if reset >= 0 || enable >= 0 {
				t.Fatalf("reset sent for a fault a reset cannot clear: %v", calls)
			}
		})
	}
}

func rtCallIndex(calls []string, want string) int {
	for i, call := range calls {
		if call == want {
			return i
		}
	}
	return -1
}

// The pendant learns from Capabilities whether to offer its program arm; the
// value is the cell manifest's unattended_loop_allowed, never a default.
func TestRTCoreCapabilitiesCarryUnattendedLoopFlag(t *testing.T) {
	for _, allowed := range []bool{false, true} {
		r, _, _ := rtFake(t)
		r.cfg.UnattendedLoopAllowed = allowed
		if r.Capabilities(context.Background()).UnattendedLoopAllowed != allowed || r.UnattendedLoopAllowed() != allowed {
			t.Fatalf("allowed=%t not reported", allowed)
		}
	}
}
