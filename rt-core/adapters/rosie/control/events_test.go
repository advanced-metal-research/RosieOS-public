//go:build linux

package control

import (
	"bufio"
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"reflect"
	"rosieos/rt-core/internal/controltest"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

func publicEventStatus(t *testing.T, client *http.Client) ProcessStatus {
	t.Helper()
	var result struct {
		Error string        `json:"error"`
		Data  ProcessStatus `json:"data"`
	}
	if _, err := (controltest.Client{HTTP: client, Base: "http://unix"}).Decode(context.Background(), http.MethodGet, "/v1/status", nil, nil, &result); err != nil || result.Error != "" {
		t.Fatalf("status: %s %v", result.Error, err)
	}
	return result.Data
}

func publicEvents(t *testing.T, client *http.Client, after uint64) EventBatch {
	t.Helper()
	var result struct {
		Error string     `json:"error"`
		Data  EventBatch `json:"data"`
	}
	if code, err := (controltest.Client{HTTP: client, Base: "http://unix"}).Decode(context.Background(), http.MethodGet, fmt.Sprintf("/v1/events?after=%d", after), nil, nil, &result); err != nil || code != 200 || result.Error != "" {
		t.Fatalf("events: HTTP %d %s %v", code, result.Error, err)
	}
	if len(result.Data.Events) > 64 {
		t.Fatalf("batch has %d records", len(result.Data.Events))
	}
	return result.Data
}

func TestStatusAxesAreCoherentWithDaemonTimestamps(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.ready(t, 511)
	points := handlePoints(600 * time.Millisecond)
	for axis := range points[1].Position {
		points[1].Position[axis] = 0.01
	}
	h := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: points}, "").Handle
	f.command(t, Request{Operation: "start_trajectory", Handle: h}, "")
	var observed ProcessStatus
	awaitHandleCondition(t, func() bool { observed = publicEventStatus(t, f.client); return observed.Axes[0].PositionCounts >= 20 })
	descriptionResponse, code, err := f.request(http.MethodGet, "/v1/describe", nil, nil)
	if err != nil || code != 200 {
		t.Fatal(err)
	}
	var description Description
	if err = json.Unmarshal(descriptionResponse.Data, &description); err != nil {
		t.Fatal(err)
	}
	if len(observed.Axes) != 9 || observed.TimeNS == 0 || observed.TimeNS != observed.Core.TimeNS {
		t.Fatalf("axis snapshot: %+v", observed)
	}
	for i, a := range observed.Axes {
		native := observed.Core.Axes[i]
		profile := description.Axes[i]
		if a.CollisionWatchdog != native.CollisionWatchdog || a.TorqueRaw != native.TorqueRaw {
			t.Fatalf("axis %d collision status differs from native", i)
		}
		if a.BrakeHoldDiagnostic != native.BrakeHoldDiagnostic || a.BrakeHoldDiagnostic.Severity != "none" {
			t.Fatalf("axis %d brake-hold diagnostic differs from native or warns without a brake: %+v", i, a.BrakeHoldDiagnostic)
		}
		if profile.Sign != 1 || math.Abs(profile.CountsPerUnit-131072/(2*math.Pi)) > 0.000001 {
			t.Fatalf("unexpected described coordinate profile: %+v", profile)
		}
		if a.TimeNS != observed.Core.TimeNS || a.TimeNS != native.TimeNS || a.PositionCounts != native.PositionCounts || a.Statusword != native.Statusword || a.BrakeState != native.BrakeState || a.HomeValid != native.HomeValid || a.LogicalTarget != native.LogicalTarget || a.LogicalPosition != native.LogicalPosition || !a.LogicalValid || a.LogicalVelocity != nil {
			t.Fatalf("axis %d mixes publications: %+v native=%+v", i, a, native)
		}
		if a.VelocityActualCountsPerS != native.VelocityActualCountsPerS || a.FollowingErrorCounts != native.FollowingErrorCounts || a.VelocityActualValid != native.VelocityActualValid || a.FollowingErrorValid != native.FollowingErrorValid || a.MaxAbsVelocityActualCountsPerS != native.MaxAbsVelocityActualCountsPerS || a.MaxAbsFollowingErrorCounts != native.MaxAbsFollowingErrorCounts {
			t.Fatalf("axis %d drive diagnostics differ: %+v native=%+v", i, a, native)
		}
		// simulation-program describes 131072 counts/revolution and positive sign.
		want := float64(a.PositionCounts) * 2 * math.Pi / 131072
		if math.Abs(a.LogicalPosition-want) > 2*math.Pi/131072 {
			t.Fatalf("axis %d position=%g want=%g within one count", i, a.LogicalPosition, want)
		}
	}
	if observed.Generations.Native != observed.Core.ControlGeneration || observed.Generations.ConfigurationEpoch != observed.Core.ConfigurationEpoch || observed.Generations.HomeEpoch != observed.Core.HomeEpoch || observed.Generations.Execution != 1 || observed.Generations.Grant != 1 {
		t.Fatalf("generations: %+v", observed.Generations)
	}
	if observed.Grant == nil || observed.Jog == nil || observed.Generations.GrantTimeNS != observed.Grant.NowHostNs || observed.Generations.JogTimeNS != observed.Jog.NowHostNs {
		t.Fatalf("generation source timestamps: %+v", observed.Generations)
	}
	if observed.PlanCursor.ActiveHandle != h || observed.PlanCursor.NativeClockNS == 0 || observed.PlanCursor.TimeNS != observed.PlanCursor.NativeClockNS {
		t.Fatalf("cursor: %+v", observed.PlanCursor)
	}
	var slots uint32
	for _, count := range observed.BufferHealth.NativeSlotOccupancy {
		slots += count
	}
	if slots != 3 || observed.BufferHealth.TimeNS != observed.Core.TimeNS || observed.Adapter.HeapInuse == 0 || observed.Adapter.Sys == 0 || observed.Adapter.Goroutines == 0 || observed.Adapter.OpenPreparationSlots != 2 {
		t.Fatalf("health: %+v %+v", observed.BufferHealth, observed.Adapter)
	}
}

func TestReadinessReasonMatchesNativeRefusal(t *testing.T) {
	t.Run("faulted", func(t *testing.T) {
		f := startPublicReset(t, "simulation-program.json", 511,
			"--sim-statusword-fault-after-enabled-ns", "1", "--sim-mode-mismatch-after-enabled-ns", "1")
		f.command(t, Request{Operation: "enable", AxisMask: 511}, "")
		f.command(t, Request{Operation: "arm"}, "")
		awaitHandleCondition(t, func() bool { return publicEventStatus(t, f.client).Axes[0].Readiness == "faulted" })
		// The fault fences the native grant. Stop establishes fresh authority
		// without clearing the fault, isolating the axis gate from that fence.
		f.command(t, Request{Operation: "stop"}, "")
		r := f.command(t, Request{Operation: "enable", AxisMask: 511}, "native_rejected")
		if r.NativeResult == nil || r.NativeResult.Reason != ipcclient.ReasonNotReady {
			t.Fatalf("fault refusal: %+v receipt=%+v", r, r.NativeResult)
		}
		if s := publicEventStatus(t, f.client); s.Axes[0].Readiness != "faulted" || s.Axes[0].Statusword&8 == 0 {
			t.Fatalf("fault refusal publication disagrees: %+v", s.Axes[0])
		}
	})
	t.Run("home_required", func(t *testing.T) {
		f := startPublicReset(t, "simulation-home.json", 1)
		s := publicEventStatus(t, f.client)
		if s.Axes[0].Readiness != "home_required" {
			t.Fatalf("home readiness: %+v", s.Axes[0])
		}
		h := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 1, Points: []ipcclient.Point{{Position: []float64{0}}, {Time: time.Second, Position: []float64{0}}}}, "").Handle
		r := f.command(t, Request{Operation: "start_trajectory", Handle: h}, "not_ready")
		if r.NativeResult == nil || r.NativeResult.Reason != ipcclient.ReasonNotReady {
			t.Fatalf("home refusal: %+v", r)
		}
	})
	t.Run("mode_mismatch", func(t *testing.T) {
		f := startPublicReset(t, "simulation-program.json", 511, "--sim-mode-mismatch-after-enabled-ns", "500000000")
		f.ready(t, 511)
		h := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: handlePoints(time.Second)}, "").Handle
		awaitHandleCondition(t, func() bool { return publicEventStatus(t, f.client).Axes[0].Readiness == "mode_mismatch" })
		r := f.command(t, Request{Operation: "start_trajectory", Handle: h}, "not_ready")
		if r.NativeResult == nil || r.NativeResult.Reason != ipcclient.ReasonNotReady {
			t.Fatalf("mode refusal: %+v", r)
		}
	})
	t.Run("brake_wait", func(t *testing.T) {
		root, err := filepath.Abs("../../..")
		if err != nil {
			t.Fatal(err)
		}
		raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-program.json"))
		if err != nil {
			t.Fatal(err)
		}
		var config map[string]any
		if err = json.Unmarshal(raw, &config); err != nil {
			t.Fatal(err)
		}
		config["axes"].([]any)[0].(map[string]any)["brake"] = map[string]any{"present": true, "gravity_axis": false, "release_delay_ms": 1200, "hold_delay_ms": 100}
		dir, err := os.MkdirTemp(filepath.Join(root, "build"), "event-brake-")
		if err != nil {
			t.Fatal(err)
		}
		t.Cleanup(func() { os.RemoveAll(dir) })
		path := filepath.Join(dir, "machine.json")
		raw, err = json.Marshal(config)
		if err != nil {
			t.Fatal(err)
		}
		if err = os.WriteFile(path, raw, 0600); err != nil {
			t.Fatal(err)
		}
		relative, err := filepath.Rel(filepath.Join(root, "config"), path)
		if err != nil {
			t.Fatal(err)
		}
		f := startPublicReset(t, relative, 511)
		f.ready(t, 511)
		h := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: handlePoints(time.Second)}, "").Handle
		awaitHandleCondition(t, func() bool { return publicEventStatus(t, f.client).Axes[0].Readiness == "brake_wait" })
		r := f.command(t, Request{Operation: "start_trajectory", Handle: h}, "not_ready")
		if r.NativeResult == nil || r.NativeResult.Reason != ipcclient.ReasonNotReady {
			t.Fatalf("brake refusal: %+v", r)
		}
	})
	t.Run("enable_timeout", func(t *testing.T) {
		// Unverified simulator injection: a drive needing 1200 ms to enable
		// exceeds the uniform 500 ms bound. The Arm attempt ends with a public
		// reason and event; nothing latches and the grant survives.
		f := startPublicReset(t, "simulation-program.json", 511, "--sim-enable-delay-cycles", "1200")
		f.command(t, Request{Operation: "enable", AxisMask: 511}, "")
		f.command(t, Request{Operation: "arm"}, "")
		awaitHandleCondition(t, func() bool { return publicEventStatus(t, f.client).Axes[0].DriveSupervisor.EventSequence != 0 })
		s := publicEventStatus(t, f.client)
		d := s.Axes[0].DriveSupervisor
		if d.Kind != "enable_timeout" || d.Severity != "arm_ended" || !d.ArmEnded || !d.Current || d.BoundNS != 500_000_000 ||
			!strings.HasPrefix(d.Message, "J1 did not reach Operation Enabled within 500 ms (statusword 0x") {
			t.Fatalf("enable timeout publication: %+v", d)
		}
		if s.Core.Armed != 0 || s.Core.SafetyFaultMask != 0 || s.Core.ExecutionFaultReasons != 0 || s.Core.AxisEnableMask != 511 {
			t.Fatalf("ended Arm attempt must not latch or drop the enable selection: %+v", s.Core)
		}
		found := false
		for _, e := range publicEvents(t, f.client, 0).Events {
			found = found || (e.Type == "drive_supervisor" && e.DriveSupervisor != nil && e.DriveSupervisor.Diagnostic.Kind == "enable_timeout")
		}
		if !found {
			t.Fatal("drive_supervisor event missing")
		}
	})
	// Unverified simulator injection: 300 enable cycles keeps a visible
	// not_operation_enabled window inside the 500 ms enable bound.
	f := startPublicReset(t, "simulation-program.json", 511, "--sim-enable-delay-cycles", "300")
	h := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: handlePoints(time.Second)}, "").Handle
	s := publicEventStatus(t, f.client)
	if s.Axes[0].Readiness != "not_enabled" {
		t.Fatalf("disabled readiness: %s", s.Axes[0].Readiness)
	}
	refused := f.command(t, Request{Operation: "start_trajectory", Handle: h}, "not_ready")
	if refused.NativeResult == nil || refused.NativeResult.Reason != ipcclient.ReasonNotReady {
		t.Fatalf("native refusal missing: %+v", refused)
	}
	f.command(t, Request{Operation: "enable", AxisMask: 511}, "")
	f.command(t, Request{Operation: "arm"}, "")
	awaitHandleCondition(t, func() bool { s = publicEventStatus(t, f.client); return s.Axes[0].Readiness == "not_operation_enabled" })
	refused = f.command(t, Request{Operation: "start_trajectory", Handle: h}, "not_ready")
	if refused.NativeResult == nil || refused.NativeResult.Reason != ipcclient.ReasonNotReady {
		t.Fatalf("native drive refusal missing: %+v", refused)
	}
	awaitHandleCondition(t, func() bool { s = publicEventStatus(t, f.client); return s.Axes[0].Readiness == "ready" })
	f.command(t, Request{Operation: "start_trajectory", Handle: h}, "")
}

// A brake-hold warning is its own event, once per new per-axis warning_sequence.
// It must never be reported as fault_latched or carry a recovery class.
func TestBrakeWarningEventIsNotAFault(t *testing.T) {
	c := &Controller{}
	status := func(time, sequence uint64) ipcclient.Status {
		s := ipcclient.Status{TimeNS: time, Axes: make([]ipcclient.AxisStatus, 2)}
		s.Axes[1].BrakeHoldDiagnostic = ipcclient.BrakeHoldDiagnostic{Severity: "none", Transition: "none", Cause: "none"}
		if sequence != 0 {
			s.Axes[1].BrakeHoldDiagnostic = ipcclient.BrakeHoldDiagnostic{Severity: "warning", WarningSequence: sequence,
				Kind: "settle", Transition: "torque_off", Cause: "velocity_cap", Phase: "settling", DisplacementCounts: 24,
				VelocityCountsPerS: 20479, VelocityValid: true, PeakVelocityCountsPerS: 20479, SameDirectionCount: uint32(sequence),
				SameDirectionDisplacementCounts: 24 * int64(sequence)}
		}
		// Slip is the same warning channel with kind=slip, never a fault.
		if sequence == 2 {
			s.Axes[1].BrakeHoldDiagnostic.Kind = "slip"
		}
		return s
	}
	for i, sequence := range []uint64{0, 1, 1, 2} {
		c.observeEventsLocked(status(uint64(i+1)*10, sequence))
	}
	b, _, err := c.events.batch(0)
	if err != nil {
		t.Fatal(err)
	}
	var warnings []Event
	for _, e := range b.Events {
		if e.Fault != nil || e.Type == "fault_latched" || e.Type == "fault_cleared" {
			t.Fatalf("brake warning surfaced as a fault: %+v", e)
		}
		if e.Type == "brake_warning" {
			warnings = append(warnings, e)
		}
	}
	if len(warnings) != 2 || warnings[0].TimeNS != 20 || warnings[1].TimeNS != 40 {
		t.Fatalf("brake warnings: %+v", warnings)
	}
	for i, e := range warnings {
		w := e.BrakeWarning
		if w == nil || w.Axis != 1 || w.Diagnostic.WarningSequence != uint64(i+1) || w.Diagnostic.Severity != "warning" || w.Diagnostic.DisplacementCounts != 24 {
			t.Fatalf("brake warning payload %d: %+v", i, w)
		}
		if kind := []string{"settle", "slip"}[i]; w.Diagnostic.Kind != kind || w.Diagnostic.SameDirectionCount != uint32(i+1) {
			t.Fatalf("brake warning %d kind: %+v", i, w.Diagnostic)
		}
	}
}

// A supervisor event ends an Arm attempt (or warns on disarm motion) with an
// operator reason. It is announced once per event_sequence, alongside the
// ordinary arm_changed event, and never as a fault.
func TestDriveSupervisorEventIsNotAFault(t *testing.T) {
	c := &Controller{}
	message := "J4 did not reach Operation Enabled within 500 ms (statusword 0x0233). Arm ended; retry Arm."
	status := func(time, sequence uint64, armed uint32) ipcclient.Status {
		s := ipcclient.Status{TimeNS: time, Armed: armed, Axes: make([]ipcclient.AxisStatus, 4)}
		if sequence != 0 {
			s.Axes[3].DriveSupervisor = ipcclient.DriveSupervisorDiagnostic{Severity: "arm_ended", EventSequence: sequence,
				Kind: "enable_timeout", Cause: "none", Message: message, Statusword: 0x0233, ArmEnded: true, Current: true,
				ElapsedNS: 500000000, BoundNS: 500000000}
		}
		return s
	}
	for i, step := range []struct {
		sequence uint64
		armed    uint32
	}{{0, 1}, {1, 0}, {1, 0}, {1, 1}} {
		c.observeEventsLocked(status(uint64(i+1)*10, step.sequence, step.armed))
	}
	b, _, err := c.events.batch(0)
	if err != nil {
		t.Fatal(err)
	}
	var supervised, armChanges []Event
	for _, e := range b.Events {
		if e.Fault != nil || e.Type == "fault_latched" || e.Type == "fault_cleared" || e.Type == "execution_faulted" {
			t.Fatalf("supervisor event surfaced as a fault: %+v", e)
		}
		switch e.Type {
		case "drive_supervisor":
			supervised = append(supervised, e)
		case "arm_changed":
			armChanges = append(armChanges, e)
		}
	}
	if len(supervised) != 1 || supervised[0].TimeNS != 20 {
		t.Fatalf("supervisor events: %+v", supervised)
	}
	if d := supervised[0].DriveSupervisor; d == nil || d.Axis != 3 || d.Diagnostic.Kind != "enable_timeout" ||
		d.Diagnostic.Message != message || !d.Diagnostic.ArmEnded {
		t.Fatalf("supervisor payload: %+v", d)
	}
	if len(armChanges) != 3 || !armChanges[0].Enable.Armed || armChanges[1].Enable.Armed || armChanges[1].TimeNS != 20 ||
		!armChanges[2].Enable.Armed {
		t.Fatalf("arm changes (armed, ended attempt, retry): %+v", armChanges)
	}
}

func TestEventSequenceIsGaplessUntilOverflow(t *testing.T) {
	f := newHandleFixture(t)
	a := serveHandleAPI(t, f.controller, nil)
	initial := publicEvents(t, a.client, 0)
	for i := 0; i < 20; i++ {
		a.call(t, controlRequest(f.fence, "renew"), "")
	}
	b := publicEvents(t, a.client, initial.NextSequence)
	for i, e := range b.Events {
		if e.Sequence != initial.NextSequence+uint64(i)+1 || e.Type == "events_dropped" || e.TimeNS == 0 {
			t.Fatalf("gap before wrap: %+v", e)
		}
	}
	if len(b.Events) != 20 {
		t.Fatalf("renewal count=%d want=20", len(b.Events))
	}
	after := b.NextSequence
	for i := 0; i < 300; i++ {
		a.call(t, controlRequest(f.fence, "renew"), "")
	}
	b = publicEvents(t, a.client, after)
	first := b.Events[0]
	if first.Type != "events_dropped" || first.Dropped == nil || first.Dropped.FirstLostSequence != after+1 || first.Dropped.LastLostSequence != b.LatestSequence-256 || first.Sequence != first.Dropped.LastLostSequence {
		t.Fatalf("overflow bounds: %+v latest=%d after=%d", first, b.LatestSequence, after)
	}
	lostRecords := 0
	last := after
	for {
		for _, e := range b.Events {
			if e.Type == "events_dropped" {
				lostRecords++
				last = e.Sequence
				continue
			}
			if e.Sequence != last+1 {
				t.Fatalf("retained sequence %d after %d", e.Sequence, last)
			}
			last = e.Sequence
		}
		if b.NextSequence == b.LatestSequence {
			break
		}
		b = publicEvents(t, a.client, b.NextSequence)
	}
	if lostRecords != 1 {
		t.Fatalf("drop records=%d want=1", lostRecords)
	}
}

// readSSE consumes one complete generated record; comments never count as
// successful delivery and the HTTP/context deadline only bounds failure.
func readSSE(t *testing.T, scan *bufio.Scanner) Event {
	t.Helper()
	var body, id, kind string
	for scan.Scan() {
		line := scan.Text()
		if line == "" && body != "" {
			var event Event
			if err := json.Unmarshal([]byte(body), &event); err != nil {
				t.Fatal(err)
			}
			if id != fmt.Sprint(event.Sequence) || kind != event.Type {
				t.Fatalf("SSE frame mismatch: %s %s %+v", id, kind, event)
			}
			return event
		}
		if strings.HasPrefix(line, "data: ") {
			body = strings.TrimPrefix(line, "data: ")
		}
		if strings.HasPrefix(line, "id: ") {
			id = strings.TrimPrefix(line, "id: ")
		}
		if strings.HasPrefix(line, "event: ") {
			kind = strings.TrimPrefix(line, "event: ")
		}
	}
	t.Fatalf("SSE ended before event: %v", scan.Err())
	return Event{}
}

func TestEventStreamResumesAfterDisconnect(t *testing.T) {
	f := newHandleFixture(t)
	a := serveHandleAPI(t, f.controller, nil)
	response, err := a.client.Get("http://unix/v1/events/stream?after=0")
	if err != nil {
		t.Fatal(err)
	}
	first := readSSE(t, bufio.NewScanner(response.Body))
	response.Body.Close()
	for i := 0; i < 12; i++ {
		a.call(t, controlRequest(f.fence, "renew"), "")
	}
	expected := publicEvents(t, a.client, first.Sequence)
	response, err = a.client.Get(fmt.Sprintf("http://unix/v1/events/stream?after=%d", first.Sequence))
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	scanner := bufio.NewScanner(response.Body)
	for _, want := range expected.Events {
		if got := readSSE(t, scanner); !reflect.DeepEqual(got, want) {
			t.Fatalf("resumed event: %+v want %+v", got, want)
		}
	}
	response.Body.Close()
	for i := 0; i < 300; i++ {
		a.call(t, controlRequest(f.fence, "renew"), "")
	}
	missing := publicEvents(t, a.client, expected.NextSequence)
	response, err = a.client.Get(fmt.Sprintf("http://unix/v1/events/stream?after=%d", expected.NextSequence))
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	if got := readSSE(t, bufio.NewScanner(response.Body)); got.Type != "events_dropped" || !reflect.DeepEqual(got, missing.Events[0]) {
		t.Fatalf("resumed loss: %+v expected %+v", got, missing.Events[0])
	}
}

func TestEventCursorRejectsMalformedAndFutureValues(t *testing.T) {
	c := &Controller{}
	a := serveHandleAPI(t, c, nil)
	for _, endpoint := range []string{"/v1/events", "/v1/events/stream"} {
		for _, query := range []string{"after=", "after=-1", "after=1.2", "after=abc", "after=18446744073709551616", "after=0&after=0", "after=1", "after=18446744073709551615"} {
			t.Run(endpoint+"?"+query, func(t *testing.T) {
				response, err := a.client.Get("http://unix" + endpoint + "?" + query)
				if err != nil {
					t.Fatal(err)
				}
				defer response.Body.Close()
				want := ReasonInvalidEventCursor
				if query == "after=1" || query == "after=18446744073709551615" {
					want = ReasonEventCursorAhead
				}
				var envelope RawResponse
				decoder := json.NewDecoder(response.Body)
				if err := decoder.Decode(&envelope); err != nil {
					t.Fatal(err)
				}
				if response.StatusCode != http.StatusConflict || response.Header.Get("Content-Type") != "application/json" ||
					envelope.Schema != ResponseSchema || envelope.Operation != OperationSubscribeEvents || envelope.Error != want {
					t.Fatalf("cursor rejection: HTTP %d %+v", response.StatusCode, envelope)
				}
				if err := decoder.Decode(new(any)); err != io.EOF {
					t.Fatalf("cursor rejection did not end the response: %v", err)
				}
			})
		}
	}
}

func TestExecutionEventsMatchStatusTransitions(t *testing.T) {
	for _, fault := range []bool{false, true} {
		t.Run(fmt.Sprint("fault=", fault), func(t *testing.T) {
			var extra []string
			if fault {
				extra = []string{"--sim-mode-mismatch-after-enabled-ns", "800000000"}
			}
			f := startPublicReset(t, "simulation-program.json", 511, extra...)
			f.ready(t, 511)
			duration := 300 * time.Millisecond
			if fault {
				duration = 2 * time.Second
			}
			h := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: handlePoints(duration)}, "").Handle
			started := f.command(t, Request{Operation: "start_trajectory", Handle: h}, "")
			if fault {
				origin := publicEventStatus(t, f.client).Core.TimeNS
				f.wait(t, func(s publicResetStatus) bool { return s.Core.TimeNS >= origin+350000000 })
				f.command(t, Request{Operation: "renew"}, "")
				// Stop fixture renewal before the deliberate native generation
				// fence. Expected fault revocation is not a keepalive failure.
				f.mu.Lock()
				f.fence = Fence{}
				f.mu.Unlock()
			}
			want := "execution_completed"
			if fault {
				want = "execution_faulted"
			}
			var terminal Event
			var events []Event
			var after uint64
			awaitHandleCondition(t, func() bool {
				b := publicEvents(t, f.client, after)
				after = b.NextSequence
				events = append(events, b.Events...)
				for _, e := range b.Events {
					if e.Type == want {
						terminal = e
						return true
					}
				}
				return false
			})
			status := publicEventStatus(t, f.client)
			if terminal.Execution == nil || terminal.Execution.Handle != h || terminal.Execution.Generation != status.Generations.Execution || terminal.Execution.NativeSequence != started.Sequence || terminal.TimeNS == 0 {
				t.Fatalf("terminal does not match status: %+v %+v", terminal, status.Generations)
			}
			if fault && (terminal.Execution.FaultBits == 0 || terminal.Execution.FaultBits != status.Core.ExecutionFaultReasons || len(terminal.Execution.Recovery) == 0) {
				t.Fatalf("fault evidence: %+v %+v", terminal, status.Core)
			}
			starts, ends := 0, 0
			for _, e := range events {
				if e.Type == "execution_started" {
					starts++
					if e.Execution.Generation != 1 || e.Execution.Handle != h {
						t.Fatalf("start: %+v", e)
					}
				}
				if e.Type == want {
					ends++
				}
			}
			if starts != 1 || ends != 1 {
				t.Fatalf("start/terminal counts=%d/%d", starts, ends)
			}
		})
	}
}

func TestEventsOverRemoteListenerRequirePrincipal(t *testing.T) {
	f := newHandleFixture(t)
	pki := newRemoteTestPKI(t, f.controller.binding.PairID)
	remote := NewRemoteHandler(f.controller, NewHandler(f.controller), JogClockLimits{})
	t.Cleanup(remote.Close)
	address := remoteTestServer(t, pki, remote)
	valid := pki.certificate(t, "reader", f.controller.binding.PairID, false, false)
	missing := pki.certificate(t, "", f.controller.binding.PairID, false, false)
	wrong := pki.certificate(t, "reader", "wrong-pair", false, false)
	for _, path := range []string{"/v1/events", "/v1/events/stream"} {
		for _, certificate := range []*tls.Certificate{nil, &missing, &wrong} {
			response, err := pki.client(t, certificate).Get("https://" + address + path)
			if response != nil {
				response.Body.Close()
				t.Fatalf("unauthorized events reached HTTP: %d %v", response.StatusCode, err)
			}
			if err == nil {
				t.Fatal("unauthorized events succeeded")
			}
		}
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		request, _ := http.NewRequestWithContext(ctx, http.MethodGet, "https://"+address+path, nil)
		response, err := pki.client(t, &valid).Do(request)
		if err != nil {
			cancel()
			t.Fatal(err)
		}
		if response.StatusCode != 200 {
			t.Fatalf("authorized events: %d", response.StatusCode)
		}
		if strings.HasSuffix(path, "stream") {
			if e := readSSE(t, bufio.NewScanner(response.Body)); e.Sequence != 1 {
				t.Fatalf("remote initial event: %+v", e)
			}
		}
		response.Body.Close()
		cancel()
	}
}
