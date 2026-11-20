package telemetrypatterns

import (
	"encoding/json"
	"errors"
	"math"
	"testing"
	"time"
)

func TestDetectActivationChurn(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "servo_activation", Axis: 0, Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start, TestSessionID: "test-session-1"},
		{Event: "servo_deactivation", Axis: 0, Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(200 * time.Millisecond), TestSessionID: "test-session-1"},
		{Event: "servo_activation", Axis: 0, Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(400 * time.Millisecond), TestSessionID: "test-session-1"},
	}, DetectorOptions{Host: "other-bench-servo"})
	if len(events) != 1 {
		t.Fatalf("Detect activation churn events = %d, want 1: %+v", len(events), events)
	}
	event := events[0]
	if event.Pattern != "activation-churn" || event.Severity != SeverityError || event.Axis != 0 {
		t.Fatalf("unexpected activation churn event: %+v", event)
	}
	if event.TestSessionID != "test-session-1" {
		t.Fatalf("activation churn session = %q, want test-session-1", event.TestSessionID)
	}
	if event.Evidence["activations"] != 2 || event.Evidence["deactivations"] != 1 {
		t.Fatalf("activation churn evidence = %+v", event.Evidence)
	}
}

func TestDetectFaultAfterCommand(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "command.position", Command: "position", Subject: "robot/v4/rtcore.other-bench-servo.command.position", Time: start, TestSessionID: "test-session-2"},
		{Event: "servo_fault", AxisFaultMask: 1 << 3, Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(500 * time.Millisecond), TestSessionID: "test-session-2"},
	}, DetectorOptions{Host: "other-bench-servo"})
	if len(events) != 1 {
		t.Fatalf("Detect fault-after-command events = %d, want 1: %+v", len(events), events)
	}
	event := events[0]
	if event.Pattern != "fault-after-command" || event.Severity != SeverityError || event.Axis != 3 {
		t.Fatalf("unexpected fault-after-command event: %+v", event)
	}
	if event.Evidence["axis_fault_mask"] != uint32(1<<3) {
		t.Fatalf("fault-after-command evidence = %+v", event.Evidence)
	}
}

func TestDetectCommandSpam(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "command.arm", Command: "disarm", Subject: "robot/v4/rtcore.other-bench-servo.command.arm", Time: start, TestSessionID: "test-session-3"},
		{Event: "command.arm", Command: "disarm", Subject: "robot/v4/rtcore.other-bench-servo.command.arm", Time: start.Add(300 * time.Millisecond), TestSessionID: "test-session-3"},
		{Event: "command.arm", Command: "disarm", Subject: "robot/v4/rtcore.other-bench-servo.command.arm", Time: start.Add(600 * time.Millisecond), TestSessionID: "test-session-3"},
	}, DetectorOptions{Host: "other-bench-servo"})
	if len(events) != 1 {
		t.Fatalf("Detect command spam events = %d, want 1: %+v", len(events), events)
	}
	event := events[0]
	if event.Pattern != "command-spam" || event.Severity != SeverityWarning || event.Axis != 0 {
		t.Fatalf("unexpected command-spam event: %+v", event)
	}
	if event.TestSessionID != "test-session-3" {
		t.Fatalf("command-spam session = %q, want test-session-3", event.TestSessionID)
	}
	if event.Evidence["command"] != "disarm" || event.Evidence["count"] != 3 {
		t.Fatalf("command-spam evidence = %+v", event.Evidence)
	}
}

func TestDetectOutOfWindowRecovery(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{{
		Event:         "timeseries",
		Subject:       "robot/v4/rtcore.other-bench-servo.telemetry.batch",
		Time:          start,
		TestSessionID: "test-session-4",
		PositionDeg:   []float64{220},
		TargetDeg:     []float64{175},
	}}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "out-of-window-recovery", SeverityWarning, 0)
	if events[0].Evidence["limit_deg"] != float64(180) {
		t.Fatalf("out-of-window evidence = %+v", events[0].Evidence)
	}
}

func TestDetectTargetErrorGrowth(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start, TestSessionID: "test-session-5", PositionDeg: []float64{0}, TargetDeg: []float64{1}},
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(100 * time.Millisecond), TestSessionID: "test-session-5", PositionDeg: []float64{0}, TargetDeg: []float64{2}},
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(200 * time.Millisecond), TestSessionID: "test-session-5", PositionDeg: []float64{0}, TargetDeg: []float64{3}},
	}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "target-error-growth", SeverityError, 0)
	if events[0].Evidence["count"] != 3 {
		t.Fatalf("target-error evidence = %+v", events[0].Evidence)
	}
}

func TestDetectHomeZeroMismatch(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{{
		Event:         "home-zero-check",
		Subject:       "robot/v4/rtcore.other-bench-servo.telemetry.batch",
		Time:          start,
		TestSessionID: "test-session-6",
		PositionDeg:   []float64{0, 0.5},
	}}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "home-zero-mismatch", SeverityError, 1)
	if events[0].Evidence["tolerance_deg"] != float64(0.25) {
		t.Fatalf("home-zero evidence = %+v", events[0].Evidence)
	}
}

func TestDetectTelemetryGap(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start, TestSessionID: "test-session-7"},
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(2500 * time.Millisecond), TestSessionID: "test-session-7"},
	}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "telemetry-gap", SeverityWarning, 0)
	if events[0].Evidence["gap_ms"] != int64(2500) {
		t.Fatalf("telemetry-gap evidence = %+v", events[0].Evidence)
	}
}

func TestDetectJetStreamGrowth(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{{
		Event:          "jetstream-state",
		Subject:        "robot/v4/rtcore.other-bench-servo.telemetry.state",
		Time:           start,
		TestSessionID:  "test-session-8",
		JetStreamBytes: 65 * 1024 * 1024,
	}}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "jetstream-growth", SeverityWarning, 0)
	if events[0].Evidence["jetstream_bytes"] != int64(65*1024*1024) {
		t.Fatalf("jetstream-growth evidence = %+v", events[0].Evidence)
	}
}

func TestDetectCommandRoundTrip(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{{
		Event:                         "timeseries",
		Subject:                       "robot/v4/rtcore.other-bench-servo.telemetry.batch",
		Time:                          start,
		TestSessionID:                 "test-session-9",
		CommandReferenceRad:           []float64{0},
		FeedbackReferenceRad:          []float64{0.2},
		CommandReferenceCounts:        []float64{1000},
		FeedbackReferenceCounts:       []float64{1700},
		CommandRoundTripToleranceRad:  []float64{0.05},
		CommandRoundTripToleranceCnts: []float64{100},
	}}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "command-roundtrip", SeverityError, 0)
	evidence := events[0].Evidence
	if evidence["command_reference_rad"] != float64(0) || evidence["feedback_reference_rad"] != float64(0.2) {
		t.Fatalf("command-roundtrip rad evidence = %+v", evidence)
	}
	if evidence["command_reference_counts"] != float64(1000) || evidence["feedback_reference_counts"] != float64(1700) {
		t.Fatalf("command-roundtrip count evidence = %+v", evidence)
	}
	if evidence["tolerance_rad"] != float64(0.05) || evidence["tolerance_counts"] != float64(100) {
		t.Fatalf("command-roundtrip tolerance evidence = %+v", evidence)
	}
}

func TestDetectCommandRoundTripUsesShortestPeriodicError(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{{
		Event:                         "timeseries",
		Subject:                       "robot/v4/rtcore.other-bench-servo.telemetry.batch",
		Time:                          start,
		TestSessionID:                 "test-session-10",
		CommandReferenceRad:           []float64{math.Pi - 0.004},
		FeedbackReferenceRad:          []float64{-math.Pi + 0.004},
		CommandReferenceCounts:        []float64{6553000},
		FeedbackReferenceCounts:       []float64{-6553000},
		CommandRoundTripToleranceRad:  []float64{0.02},
		CommandRoundTripToleranceCnts: []float64{100},
	}}, DetectorOptions{Host: "other-bench-servo"})
	if len(events) != 0 {
		t.Fatalf("Detect seam-safe command roundtrip events = %+v, want none", events)
	}
}

func TestDetectStaleHomeAnchor(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{{
		Event:                   "timeseries",
		Subject:                 "robot/v4/rtcore.other-bench-servo.telemetry.batch",
		Time:                    start,
		TestSessionID:           "test-session-11",
		HomeAnchorCounts:        []float64{1000},
		DriveNativeHomeCounts:   []float64{1015},
		HomeAnchorToleranceCnts: []float64{2},
		HomeAnchorStale:         []bool{true},
	}}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "stale-home-anchor", SeverityError, 0)
	evidence := events[0].Evidence
	if evidence["home_anchor_counts"] != float64(1000) || evidence["drive_native_home_counts"] != float64(1015) {
		t.Fatalf("stale-home-anchor evidence = %+v", evidence)
	}
	if evidence["home_anchor_error_counts"] != float64(15) || evidence["tolerance_counts"] != float64(2) || evidence["home_anchor_stale"] != true {
		t.Fatalf("stale-home-anchor drift evidence = %+v", evidence)
	}
}

func TestDetectAmpFluctuation(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start, TestSessionID: "test-session-12", AxisCurrentMA: []float64{120, 80}},
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(50 * time.Millisecond), TestSessionID: "test-session-12", AxisCurrentMA: []float64{780, 90}},
	}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "amp-fluctuation", SeverityWarning, 0)
	evidence := events[0].Evidence
	if evidence["max_adjacent_delta_ma"] != float64(660) || evidence["delta_threshold_ma"] != float64(500) {
		t.Fatalf("amp-fluctuation evidence = %+v", evidence)
	}
}

func TestDetectActiveJogStall(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{
			Event:                   "timeseries",
			Subject:                 "robot/v4/rtcore.other-bench-servo.telemetry.batch",
			Time:                    start,
			TestSessionID:           "test-session-13",
			ActiveJog:               true,
			ServosArmedRequested:    true,
			ServoControlEnabled:     true,
			CommandReferenceCounts:  []float64{120000},
			FeedbackReferenceCounts: []float64{100000},
		},
		{
			Event:                   "timeseries",
			Subject:                 "robot/v4/rtcore.other-bench-servo.telemetry.batch",
			Time:                    start.Add(100 * time.Millisecond),
			TestSessionID:           "test-session-13",
			ActiveJog:               true,
			ServosArmedRequested:    true,
			ServoControlEnabled:     true,
			CommandReferenceCounts:  []float64{121000},
			FeedbackReferenceCounts: []float64{100010},
		},
		{
			Event:                   "timeseries",
			Subject:                 "robot/v4/rtcore.other-bench-servo.telemetry.batch",
			Time:                    start.Add(200 * time.Millisecond),
			TestSessionID:           "test-session-13",
			ActiveJog:               true,
			ServosArmedRequested:    true,
			ServoControlEnabled:     true,
			CommandReferenceCounts:  []float64{122000},
			FeedbackReferenceCounts: []float64{100020},
		},
	}, DetectorOptions{Host: "other-bench-servo"})
	assertPattern(t, events, "active-jog-stall", SeverityError, 0)
	evidence := events[0].Evidence
	if evidence["active_jog"] != true || evidence["units"] != "counts" {
		t.Fatalf("active-jog-stall evidence = %+v", evidence)
	}
	if evidence["position_delta"] != float64(20) || evidence["position_delta_threshold"] != float64(512) {
		t.Fatalf("active-jog-stall position evidence = %+v", evidence)
	}
	if evidence["max_target_error"] != float64(21980) || evidence["target_error_threshold"] != float64(2048) {
		t.Fatalf("active-jog-stall target evidence = %+v", evidence)
	}
}

func TestDetectMovingActiveJogDoesNotStall(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start, TestSessionID: "test-session-14", ActiveJog: true, ServosArmedRequested: true, ServoControlEnabled: true, CommandReferenceCounts: []float64{120000}, FeedbackReferenceCounts: []float64{100000}},
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(100 * time.Millisecond), TestSessionID: "test-session-14", ActiveJog: true, ServosArmedRequested: true, ServoControlEnabled: true, CommandReferenceCounts: []float64{121000}, FeedbackReferenceCounts: []float64{101000}},
		{Event: "timeseries", Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(200 * time.Millisecond), TestSessionID: "test-session-14", ActiveJog: true, ServosArmedRequested: true, ServoControlEnabled: true, CommandReferenceCounts: []float64{122000}, FeedbackReferenceCounts: []float64{102000}},
	}, DetectorOptions{Host: "other-bench-servo"})
	if len(events) != 0 {
		t.Fatalf("Detect moving active jog events = %+v, want none", events)
	}
}

func TestDetectNormalTelemetryDoesNotWarn(t *testing.T) {
	start := time.Date(2026, 5, 28, 0, 0, 0, 0, time.UTC)
	events := Detect([]Record{
		{Event: "command.position", Command: "position", Subject: "robot/v4/rtcore.other-bench-servo.command.position", Time: start},
		{Event: "servo_activation", Axis: 0, Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(100 * time.Millisecond)},
		{Event: "timeseries", AxisEnableMask: 1, Subject: "robot/v4/rtcore.other-bench-servo.telemetry.batch", Time: start.Add(200 * time.Millisecond)},
	}, DetectorOptions{Host: "other-bench-servo"})
	if len(events) != 0 {
		t.Fatalf("Detect normal telemetry events = %+v, want none", events)
	}
}

func TestPatternSubjectSanitizesHost(t *testing.T) {
	got := PatternSubject(" other-bench-servo.local ")
	want := "robot/v4/rtcore.other-bench-servolocal.telemetry.pattern"
	if got != want {
		t.Fatalf("PatternSubject() = %q, want %q", got, want)
	}
}

func TestPublishEventsPublishesNormalizedPatternEvent(t *testing.T) {
	publisher := &capturePublisher{}
	err := PublishEvents(publisher, "other-bench-servo", []Event{{
		Pattern:       "activation-churn",
		Severity:      SeverityError,
		Axis:          0,
		WindowMS:      12000,
		TestSessionID: "test-session-1",
		Evidence: map[string]any{
			"activations":   2,
			"deactivations": 1,
		},
		SourceSubjects:  []string{"robot/v4/rtcore.other-bench-servo.telemetry.batch"},
		SuggestedAction: "stop motion",
	}})
	if err != nil {
		t.Fatalf("PublishEvents() error = %v", err)
	}
	if publisher.subject != "robot/v4/rtcore.other-bench-servo.telemetry.pattern" {
		t.Fatalf("published subject = %q", publisher.subject)
	}
	var payload map[string]any
	if err := json.Unmarshal(publisher.payload, &payload); err != nil {
		t.Fatalf("published payload is not JSON: %v", err)
	}
	assertString(t, payload, "event", "telemetry-pattern")
	assertString(t, payload, "type", "telemetry-pattern")
	assertString(t, payload, "pattern", "activation-churn")
	assertString(t, payload, "severity", "error")
	assertString(t, payload, "host", "other-bench-servo")
	assertString(t, payload, "test_session_id", "test-session-1")
	assertString(t, payload, "suggested_action", "stop motion")
	if payload["axis"] != float64(0) {
		t.Fatalf("payload axis = %#v, want 0", payload["axis"])
	}
	if payload["evidence"] == nil {
		t.Fatalf("payload evidence missing: %+v", payload)
	}
}

func TestPublishEventsRejectsNilPublisher(t *testing.T) {
	if err := PublishEvents(nil, "other-bench-servo", []Event{{Pattern: "activation-churn"}}); err == nil {
		t.Fatal("PublishEvents(nil) error = nil, want error")
	}
}

func TestPublishEventsReturnsPublisherError(t *testing.T) {
	want := errors.New("publish failed")
	publisher := &capturePublisher{err: want}
	if err := PublishEvents(publisher, "other-bench-servo", []Event{{Pattern: "activation-churn"}}); !errors.Is(err, want) {
		t.Fatalf("PublishEvents() error = %v, want %v", err, want)
	}
}

type capturePublisher struct {
	subject string
	payload []byte
	err     error
}

func (publisher *capturePublisher) Publish(subject string, payload []byte) error {
	publisher.subject = subject
	publisher.payload = append([]byte(nil), payload...)
	return publisher.err
}

func assertString(t *testing.T, payload map[string]any, key string, want string) {
	t.Helper()
	if payload[key] != want {
		t.Fatalf("payload[%q] = %#v, want %q", key, payload[key], want)
	}
}

func assertPattern(t *testing.T, events []Event, pattern string, severity Severity, axis int) {
	t.Helper()
	if len(events) != 1 {
		t.Fatalf("Detect %s events = %d, want 1: %+v", pattern, len(events), events)
	}
	event := events[0]
	if event.Pattern != pattern || event.Severity != severity || event.Axis != axis {
		t.Fatalf("unexpected %s event: %+v", pattern, event)
	}
	if event.TestSessionID == "" {
		t.Fatalf("%s event missing test session: %+v", pattern, event)
	}
	if event.SuggestedAction == "" {
		t.Fatalf("%s event missing suggested action: %+v", pattern, event)
	}
}
