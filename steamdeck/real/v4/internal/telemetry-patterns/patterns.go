package telemetrypatterns

import (
	"encoding/json"
	"fmt"
	"math"
	"strings"
	"time"
)

type Severity string

const (
	SeverityWarning Severity = "warning"
	SeverityError   Severity = "error"
)

type Record struct {
	Event                string
	Axis                 int
	AxisEnableMask       uint32
	AxisErrorMask        uint32
	AxisFaultMask        uint32
	ActiveJog            bool
	ServoControlEnabled  bool
	ServosArmedRequested bool
	Subject              string
	Time                 time.Time
	Command              string
	CommandID            string
	TestSessionID        string
	CommandResult        string
	CommandError         string
	CommandSenderID      string
	CommandMode          string
	CommandIOSlot        int
	CommandIOOn          *bool
	PositionDeg          []float64
	TargetDeg            []float64
	AxisCurrentMA        []float64
	AxisVoltageMV        []float64
	JetStreamBytes       int64

	CommandReferenceRad           []float64
	FeedbackReferenceRad          []float64
	CommandReferenceCounts        []float64
	FeedbackReferenceCounts       []float64
	CommandRoundTripToleranceRad  []float64
	CommandRoundTripToleranceCnts []float64
	HomeAnchorCounts              []float64
	DriveNativeHomeCounts         []float64
	HomeAnchorToleranceCnts       []float64
	HomeAnchorStale               []bool
}

type Event struct {
	Event           string         `json:"event,omitempty"`
	Type            string         `json:"type"`
	Pattern         string         `json:"pattern"`
	Severity        Severity       `json:"severity"`
	Host            string         `json:"host,omitempty"`
	Axis            int            `json:"axis"`
	WindowMS        int64          `json:"window_ms,omitempty"`
	Evidence        map[string]any `json:"evidence"`
	SourceSubjects  []string       `json:"source_subjects,omitempty"`
	TestSessionID   string         `json:"test_session_id,omitempty"`
	SuggestedAction string         `json:"suggested_action"`
}

type Publisher interface {
	Publish(subject string, payload []byte) error
}

const DefaultSubjectRoot = "robot/v4/rtcore"

type DetectorOptions struct {
	Host                          string
	AxisCount                     int
	ActivationChurnThreshold      int
	ActivationChurnWindow         time.Duration
	CommandSpamThreshold          int
	CommandSpamWindow             time.Duration
	FaultAfterCommandWindow       time.Duration
	FaultAfterCommandPatternName  string
	OutOfWindowLimitDeg           float64
	TargetErrorGrowthThreshold    int
	TargetErrorGrowthWindow       time.Duration
	HomeZeroToleranceDeg          float64
	TelemetryGapWindow            time.Duration
	JetStreamGrowthMaxBytes       int64
	AmpFluctuationWindow          time.Duration
	AmpFluctuationDeltaMA         float64
	AmpFluctuationRangeMA         float64
	ActiveJogStallWindow          time.Duration
	ActiveJogStallMinSamples      int
	ActiveJogStallTargetErrorDeg  float64
	ActiveJogStallPositionDeg     float64
	ActiveJogStallTargetErrorCnt  float64
	ActiveJogStallPositionCnt     float64
	CommandRoundTripToleranceRad  float64
	CommandRoundTripToleranceCnts float64
	HomeAnchorToleranceCnts       float64
}

func Detect(records []Record, options DetectorOptions) []Event {
	options = options.withDefaults()
	var events []Event
	events = append(events, detectActivationChurn(records, options)...)
	events = append(events, detectCommandSpam(records, options)...)
	events = append(events, detectFaultAfterCommand(records, options)...)
	events = append(events, detectOutOfWindowRecovery(records, options)...)
	events = append(events, detectTargetErrorGrowth(records, options)...)
	events = append(events, detectHomeZeroMismatch(records, options)...)
	events = append(events, detectCommandRoundTrip(records, options)...)
	events = append(events, detectStaleHomeAnchor(records, options)...)
	events = append(events, detectActiveJogStall(records, options)...)
	events = append(events, detectAmpFluctuation(records, options)...)
	events = append(events, detectTelemetryGap(records, options)...)
	events = append(events, detectJetStreamGrowth(records, options)...)
	return events
}

func PatternSubject(host string) string {
	host = topicToken(host)
	if host == "" {
		host = "unknown"
	}
	return DefaultSubjectRoot + "." + host + ".telemetry.pattern"
}

func PublishEvents(publisher Publisher, host string, events []Event) error {
	if publisher == nil {
		return fmt.Errorf("telemetry-pattern publisher is nil")
	}
	subject := PatternSubject(host)
	for _, event := range events {
		event = normalizeEvent(event, host)
		raw, err := json.Marshal(event)
		if err != nil {
			return err
		}
		if err := publisher.Publish(subject, raw); err != nil {
			return err
		}
	}
	return nil
}

func normalizeEvent(event Event, host string) Event {
	if event.Event == "" {
		event.Event = "telemetry-pattern"
	}
	if event.Type == "" {
		event.Type = "telemetry-pattern"
	}
	if event.Host == "" {
		event.Host = host
	}
	if event.Evidence == nil {
		event.Evidence = map[string]any{}
	}
	return event
}

func (options DetectorOptions) withDefaults() DetectorOptions {
	if options.AxisCount <= 0 {
		options.AxisCount = 7
	}
	if options.ActivationChurnThreshold <= 0 {
		options.ActivationChurnThreshold = 2
	}
	if options.ActivationChurnWindow <= 0 {
		options.ActivationChurnWindow = 12 * time.Second
	}
	if options.CommandSpamThreshold <= 0 {
		options.CommandSpamThreshold = 3
	}
	if options.CommandSpamWindow <= 0 {
		options.CommandSpamWindow = 5 * time.Second
	}
	if options.FaultAfterCommandWindow <= 0 {
		options.FaultAfterCommandWindow = 10 * time.Second
	}
	if options.FaultAfterCommandPatternName == "" {
		options.FaultAfterCommandPatternName = "fault-after-command"
	}
	if options.OutOfWindowLimitDeg <= 0 {
		options.OutOfWindowLimitDeg = 180
	}
	if options.TargetErrorGrowthThreshold <= 0 {
		options.TargetErrorGrowthThreshold = 3
	}
	if options.TargetErrorGrowthWindow <= 0 {
		options.TargetErrorGrowthWindow = 8 * time.Second
	}
	if options.HomeZeroToleranceDeg <= 0 {
		options.HomeZeroToleranceDeg = 0.25
	}
	if options.TelemetryGapWindow <= 0 {
		options.TelemetryGapWindow = 2 * time.Second
	}
	if options.JetStreamGrowthMaxBytes <= 0 {
		options.JetStreamGrowthMaxBytes = 64 * 1024 * 1024
	}
	if options.AmpFluctuationWindow <= 0 {
		options.AmpFluctuationWindow = 750 * time.Millisecond
	}
	if options.AmpFluctuationDeltaMA <= 0 {
		options.AmpFluctuationDeltaMA = 500
	}
	if options.AmpFluctuationRangeMA <= 0 {
		options.AmpFluctuationRangeMA = 1000
	}
	if options.ActiveJogStallWindow <= 0 {
		options.ActiveJogStallWindow = 500 * time.Millisecond
	}
	if options.ActiveJogStallMinSamples <= 0 {
		options.ActiveJogStallMinSamples = 3
	}
	if options.ActiveJogStallTargetErrorDeg <= 0 {
		options.ActiveJogStallTargetErrorDeg = 0.5
	}
	if options.ActiveJogStallPositionDeg <= 0 {
		options.ActiveJogStallPositionDeg = 0.05
	}
	if options.ActiveJogStallTargetErrorCnt <= 0 {
		options.ActiveJogStallTargetErrorCnt = 2048
	}
	if options.ActiveJogStallPositionCnt <= 0 {
		options.ActiveJogStallPositionCnt = 512
	}
	if options.CommandRoundTripToleranceRad <= 0 {
		options.CommandRoundTripToleranceRad = math.Pi / 180
	}
	if options.CommandRoundTripToleranceCnts <= 0 {
		options.CommandRoundTripToleranceCnts = 2048
	}
	if options.HomeAnchorToleranceCnts <= 0 {
		options.HomeAnchorToleranceCnts = 1
	}
	return options
}

func detectActivationChurn(records []Record, options DetectorOptions) []Event {
	type counts struct {
		activations   int
		deactivations int
		start         time.Time
		subjects      []string
		sessionID     string
	}
	axisCounts := make([]counts, options.AxisCount)
	for _, record := range records {
		axes := affectedAxes(record, options.AxisCount)
		for _, axis := range axes {
			item := axisCounts[axis]
			if item.start.IsZero() {
				item.start = record.Time
			}
			if record.Time.Sub(item.start) > options.ActivationChurnWindow {
				item = counts{start: record.Time}
			}
			switch record.Event {
			case "servo_activation":
				item.activations++
			case "servo_deactivation":
				item.deactivations++
			}
			item.subjects = appendUnique(item.subjects, record.Subject)
			if item.sessionID == "" {
				item.sessionID = record.TestSessionID
			}
			axisCounts[axis] = item
		}
	}
	var events []Event
	for axis, item := range axisCounts {
		if item.activations+item.deactivations < options.ActivationChurnThreshold || item.deactivations == 0 {
			continue
		}
		events = append(events, Event{
			Type:           "telemetry-pattern",
			Pattern:        "activation-churn",
			Severity:       SeverityError,
			Host:           options.Host,
			Axis:           axis,
			WindowMS:       options.ActivationChurnWindow.Milliseconds(),
			SourceSubjects: item.subjects,
			TestSessionID:  item.sessionID,
			Evidence: map[string]any{
				"activations":   item.activations,
				"deactivations": item.deactivations,
			},
			SuggestedAction: "stop motion commands, hold servo power state, inspect recent target/statusword samples, and do not rerun live motion until the activation cause is fixed",
		})
	}
	return events
}

func detectCommandSpam(records []Record, options DetectorOptions) []Event {
	type commandCounts struct {
		count     int
		start     time.Time
		subjects  []string
		sessionID string
	}
	countsByCommand := map[string]commandCounts{}
	for _, record := range records {
		command := commandName(record)
		if command == "" {
			continue
		}
		item := countsByCommand[command]
		if item.start.IsZero() {
			item.start = record.Time
		}
		if !record.Time.IsZero() && !item.start.IsZero() && record.Time.Sub(item.start) > options.CommandSpamWindow {
			item = commandCounts{start: record.Time}
		}
		item.count++
		item.subjects = appendUnique(item.subjects, record.Subject)
		if item.sessionID == "" {
			item.sessionID = record.TestSessionID
		}
		countsByCommand[command] = item
	}
	for command, item := range countsByCommand {
		if item.count < options.CommandSpamThreshold {
			continue
		}
		return []Event{{
			Type:           "telemetry-pattern",
			Pattern:        "command-spam",
			Severity:       SeverityWarning,
			Host:           options.Host,
			Axis:           0,
			WindowMS:       options.CommandSpamWindow.Milliseconds(),
			SourceSubjects: item.subjects,
			TestSessionID:  item.sessionID,
			Evidence: map[string]any{
				"command": command,
				"count":   item.count,
			},
			SuggestedAction: "stop issuing duplicate commands, verify UI/test idempotence, and inspect trace plus command subjects before running live motion again",
		}}
	}
	return nil
}

func detectFaultAfterCommand(records []Record, options DetectorOptions) []Event {
	var command *Record
	var events []Event
	for _, record := range records {
		if isCommandEvent(record) {
			copy := record
			command = &copy
			continue
		}
		if command == nil || (!record.Time.IsZero() && !command.Time.IsZero() && record.Time.Sub(command.Time) > options.FaultAfterCommandWindow) {
			continue
		}
		for axis := 0; axis < options.AxisCount; axis++ {
			errorSet := maskSet(record.AxisErrorMask, axis)
			faultSet := maskSet(record.AxisFaultMask, axis)
			eventSet := record.AxisErrorMask == 0 && record.AxisFaultMask == 0 && record.Axis == axis && (record.Event == "servo_error" || record.Event == "servo_fault")
			if !errorSet && !faultSet && !eventSet {
				continue
			}
			events = append(events, Event{
				Type:           "telemetry-pattern",
				Pattern:        options.FaultAfterCommandPatternName,
				Severity:       SeverityError,
				Host:           options.Host,
				Axis:           axis,
				WindowMS:       options.FaultAfterCommandWindow.Milliseconds(),
				SourceSubjects: appendUnique([]string{command.Subject}, record.Subject),
				TestSessionID:  firstNonEmpty(record.TestSessionID, command.TestSessionID),
				Evidence: map[string]any{
					"axis_error_mask": record.AxisErrorMask,
					"axis_fault_mask": record.AxisFaultMask,
					"command":         command.Command,
					"event":           record.Event,
				},
				SuggestedAction: "preserve JetStream evidence, inspect drive fault/statusword details, hold servo power state, and recover the drive before sending another command",
			})
			return events
		}
	}
	return events
}

func detectOutOfWindowRecovery(records []Record, options DetectorOptions) []Event {
	for _, record := range records {
		for axis := 0; axis < options.AxisCount; axis++ {
			position, ok := valueAt(record.PositionDeg, axis)
			if !ok || abs(position) <= options.OutOfWindowLimitDeg {
				continue
			}
			target, ok := valueAt(record.TargetDeg, axis)
			if !ok {
				continue
			}
			if abs(target) >= abs(position) {
				continue
			}
			return []Event{patternEvent("out-of-window-recovery", SeverityWarning, options, record, axis, map[string]any{
				"position_deg": position,
				"target_deg":   target,
				"limit_deg":    options.OutOfWindowLimitDeg,
			}, "continue bounded inward recovery toward the nearest home-relative limit; do not jump across the valid joint window")}
		}
	}
	return nil
}

func detectTargetErrorGrowth(records []Record, options DetectorOptions) []Event {
	type growthState struct {
		count       int
		firstTime   time.Time
		previousErr float64
		record      Record
	}
	states := make([]growthState, options.AxisCount)
	for _, record := range records {
		for axis := 0; axis < options.AxisCount; axis++ {
			position, ok := valueAt(record.PositionDeg, axis)
			if !ok {
				continue
			}
			target, ok := valueAt(record.TargetDeg, axis)
			if !ok {
				continue
			}
			currentErr := abs(target - position)
			state := states[axis]
			if state.firstTime.IsZero() || (!record.Time.IsZero() && !state.firstTime.IsZero() && record.Time.Sub(state.firstTime) > options.TargetErrorGrowthWindow) || currentErr <= state.previousErr {
				state = growthState{count: 1, firstTime: record.Time, previousErr: currentErr, record: record}
			} else {
				state.count++
				state.previousErr = currentErr
				state.record = record
			}
			states[axis] = state
			if state.count >= options.TargetErrorGrowthThreshold {
				return []Event{patternEvent("target-error-growth", SeverityError, options, record, axis, map[string]any{
					"count":             state.count,
					"target_deg":        target,
					"position_deg":      position,
					"target_error_deg":  currentErr,
					"growth_window_ms":  options.TargetErrorGrowthWindow.Milliseconds(),
					"previous_subject":  state.record.Subject,
					"current_timestamp": record.Time.Format(time.RFC3339Nano),
				}, "stop the move, inspect target generation and feedback direction, and verify sign/home mapping before sending more position commands")}
			}
		}
	}
	return nil
}

func detectHomeZeroMismatch(records []Record, options DetectorOptions) []Event {
	for _, record := range records {
		if record.Event != "home-zero-check" && record.Event != "home-zero-mismatch" {
			continue
		}
		for axis := 0; axis < options.AxisCount; axis++ {
			position, ok := valueAt(record.PositionDeg, axis)
			if !ok || abs(position) <= options.HomeZeroToleranceDeg {
				continue
			}
			return []Event{patternEvent("home-zero-mismatch", SeverityError, options, record, axis, map[string]any{
				"position_deg":  position,
				"tolerance_deg": options.HomeZeroToleranceDeg,
			}, "preserve home evidence, compare page and NATS home-relative positions, and do not run motion until zero agrees")}
		}
	}
	return nil
}

func detectTelemetryGap(records []Record, options DetectorOptions) []Event {
	var previous *Record
	for _, record := range records {
		if record.Time.IsZero() {
			continue
		}
		if previous != nil && record.Time.Sub(previous.Time) > options.TelemetryGapWindow {
			return []Event{patternEvent("telemetry-gap", SeverityWarning, options, record, 0, map[string]any{
				"gap_ms":           record.Time.Sub(previous.Time).Milliseconds(),
				"threshold_ms":     options.TelemetryGapWindow.Milliseconds(),
				"previous_subject": previous.Subject,
			}, "inspect NATS bridge, JetStream, and realtime-core telemetry publishers before trusting motion plots")}
		}
		copy := record
		previous = &copy
	}
	return nil
}

func detectJetStreamGrowth(records []Record, options DetectorOptions) []Event {
	for _, record := range records {
		if record.JetStreamBytes <= options.JetStreamGrowthMaxBytes {
			continue
		}
		return []Event{patternEvent("jetstream-growth", SeverityWarning, options, record, 0, map[string]any{
			"jetstream_bytes": record.JetStreamBytes,
			"max_bytes":       options.JetStreamGrowthMaxBytes,
		}, "reduce telemetry sample rate, rotate captures, or inspect high-rate publishers before continuing long tests")}
	}
	return nil
}

func detectCommandRoundTrip(records []Record, options DetectorOptions) []Event {
	for _, record := range records {
		if record.ActiveJog {
			continue
		}
		for axis := 0; axis < options.AxisCount; axis++ {
			evidence := map[string]any{}
			radExceeded := false
			countsExceeded := false
			hasRad := false
			hasCounts := false

			commandRad, commandRadOK := valueAt(record.CommandReferenceRad, axis)
			feedbackRad, feedbackRadOK := valueAt(record.FeedbackReferenceRad, axis)
			if commandRadOK && feedbackRadOK {
				hasRad = true
				toleranceRad := toleranceAt(record.CommandRoundTripToleranceRad, axis, options.CommandRoundTripToleranceRad)
				errorRad := abs(shortestPeriodicErrorRad(commandRad, feedbackRad))
				radExceeded = errorRad > toleranceRad
				evidence["command_reference_rad"] = commandRad
				evidence["feedback_reference_rad"] = feedbackRad
				evidence["error_rad"] = errorRad
				evidence["tolerance_rad"] = toleranceRad
			}

			commandCounts, commandCountsOK := valueAt(record.CommandReferenceCounts, axis)
			feedbackCounts, feedbackCountsOK := valueAt(record.FeedbackReferenceCounts, axis)
			if commandCountsOK && feedbackCountsOK {
				hasCounts = true
				toleranceCounts := toleranceAt(record.CommandRoundTripToleranceCnts, axis, options.CommandRoundTripToleranceCnts)
				errorCounts := abs(feedbackCounts - commandCounts)
				countsExceeded = errorCounts > toleranceCounts
				evidence["command_reference_counts"] = commandCounts
				evidence["feedback_reference_counts"] = feedbackCounts
				evidence["error_counts"] = errorCounts
				evidence["tolerance_counts"] = toleranceCounts
			}

			switch {
			case hasRad && !radExceeded:
				continue
			case !hasRad && (!hasCounts || !countsExceeded):
				continue
			}

			return []Event{patternEvent("command-roundtrip", SeverityError, options, record, axis, evidence, "stop the move, compare command and feedback references with shortest-periodic error math, and verify home/sign/count mapping before sending another position command")}
		}
	}
	return nil
}

func detectStaleHomeAnchor(records []Record, options DetectorOptions) []Event {
	for _, record := range records {
		for axis := 0; axis < options.AxisCount; axis++ {
			homeAnchor, homeOK := valueAt(record.HomeAnchorCounts, axis)
			driveNativeHome, driveOK := valueAt(record.DriveNativeHomeCounts, axis)
			stale, staleOK := boolAt(record.HomeAnchorStale, axis)
			toleranceCounts := toleranceAt(record.HomeAnchorToleranceCnts, axis, options.HomeAnchorToleranceCnts)
			errorCounts := 0.0
			if homeOK && driveOK {
				errorCounts = abs(driveNativeHome - homeAnchor)
				stale = stale || errorCounts > toleranceCounts
			}
			if !stale && !staleOK {
				continue
			}
			if !stale {
				continue
			}
			evidence := map[string]any{
				"home_anchor_stale": stale,
				"tolerance_counts":  toleranceCounts,
			}
			if homeOK {
				evidence["home_anchor_counts"] = homeAnchor
			}
			if driveOK {
				evidence["drive_native_home_counts"] = driveNativeHome
			}
			if homeOK && driveOK {
				evidence["home_anchor_error_counts"] = errorCounts
			}
			return []Event{patternEvent("stale-home-anchor", SeverityError, options, record, axis, evidence, "stop home-relative motion, refresh HM35 drive-native home truth, and keep the stale anchor evidence before issuing more target commands")}
		}
	}
	return nil
}

func detectActiveJogStall(records []Record, options DetectorOptions) []Event {
	type stallState struct {
		start              time.Time
		samples            int
		minPosition        float64
		maxPosition        float64
		maxTargetError     float64
		countsMode         bool
		subjects           []string
		sessionID          string
		lastRecord         Record
		lastTargetPosition float64
	}
	states := make([]stallState, options.AxisCount)
	for _, record := range records {
		for axis := 0; axis < options.AxisCount; axis++ {
			targetError, position, targetPosition, countsMode, ok := activeJogStallValues(record, axis)
			if !ok ||
				!record.ActiveJog ||
				!record.ServosArmedRequested ||
				!record.ServoControlEnabled ||
				maskSet(record.AxisErrorMask|record.AxisFaultMask, axis) ||
				targetError < activeJogStallTargetThreshold(options, countsMode) {
				states[axis] = stallState{}
				continue
			}

			state := states[axis]
			if state.start.IsZero() ||
				state.countsMode != countsMode ||
				(!record.Time.IsZero() && !state.start.IsZero() && record.Time.Sub(state.start) > options.ActiveJogStallWindow) {
				state = stallState{
					start:       record.Time,
					minPosition: position,
					maxPosition: position,
					countsMode:  countsMode,
				}
			}
			if state.samples == 0 {
				state.minPosition = position
				state.maxPosition = position
			}
			state.samples++
			state.minPosition = math.Min(state.minPosition, position)
			state.maxPosition = math.Max(state.maxPosition, position)
			state.maxTargetError = math.Max(state.maxTargetError, targetError)
			state.subjects = appendUnique(state.subjects, record.Subject)
			state.lastRecord = record
			state.lastTargetPosition = targetPosition
			if state.sessionID == "" {
				state.sessionID = record.TestSessionID
			}
			states[axis] = state

			positionDelta := state.maxPosition - state.minPosition
			if state.samples < options.ActiveJogStallMinSamples ||
				positionDelta > activeJogStallPositionThreshold(options, countsMode) {
				continue
			}
			units := "deg"
			if countsMode {
				units = "counts"
			}
			return []Event{{
				Type:           "telemetry-pattern",
				Pattern:        "active-jog-stall",
				Severity:       SeverityError,
				Host:           options.Host,
				Axis:           axis,
				WindowMS:       options.ActiveJogStallWindow.Milliseconds(),
				SourceSubjects: state.subjects,
				TestSessionID:  state.sessionID,
				Evidence: map[string]any{
					"active_jog":               true,
					"samples":                  state.samples,
					"units":                    units,
					"position_delta":           positionDelta,
					"position_delta_threshold": activeJogStallPositionThreshold(options, countsMode),
					"max_target_error":         state.maxTargetError,
					"target_error_threshold":   activeJogStallTargetThreshold(options, countsMode),
					"last_position":            position,
					"last_target":              state.lastTargetPosition,
				},
				SuggestedAction: "stop the active jog stream, keep servo power state stable, inspect owner/latency/target evidence for this test-session-id, and do not rerun integrated motion until the stalled axis is explained",
			}}
		}
	}
	return nil
}

func activeJogStallValues(record Record, axis int) (targetError float64, position float64, targetPosition float64, countsMode bool, ok bool) {
	targetCounts, targetCountsOK := valueAt(record.CommandReferenceCounts, axis)
	positionCounts, positionCountsOK := valueAt(record.FeedbackReferenceCounts, axis)
	if targetCountsOK && positionCountsOK {
		return abs(targetCounts - positionCounts), positionCounts, targetCounts, true, true
	}
	targetDeg, targetDegOK := valueAt(record.TargetDeg, axis)
	positionDeg, positionDegOK := valueAt(record.PositionDeg, axis)
	if targetDegOK && positionDegOK {
		return abs(targetDeg - positionDeg), positionDeg, targetDeg, false, true
	}
	return 0, 0, 0, false, false
}

func activeJogStallTargetThreshold(options DetectorOptions, countsMode bool) float64 {
	if countsMode {
		return options.ActiveJogStallTargetErrorCnt
	}
	return options.ActiveJogStallTargetErrorDeg
}

func activeJogStallPositionThreshold(options DetectorOptions, countsMode bool) float64 {
	if countsMode {
		return options.ActiveJogStallPositionCnt
	}
	return options.ActiveJogStallPositionDeg
}

func detectAmpFluctuation(records []Record, options DetectorOptions) []Event {
	type ampState struct {
		start             time.Time
		previousCurrentMA float64
		havePrevious      bool
		minCurrentMA      float64
		maxCurrentMA      float64
		maxDeltaMA        float64
		samples           int
		subjects          []string
		sessionID         string
	}
	states := make([]ampState, options.AxisCount)
	for _, record := range records {
		if len(record.AxisCurrentMA) == 0 {
			continue
		}
		for axis := 0; axis < options.AxisCount; axis++ {
			currentMA, ok := valueAt(record.AxisCurrentMA, axis)
			if !ok || math.IsNaN(currentMA) || math.IsInf(currentMA, 0) {
				continue
			}
			state := states[axis]
			if state.start.IsZero() || (!record.Time.IsZero() && !state.start.IsZero() && record.Time.Sub(state.start) > options.AmpFluctuationWindow) {
				state = ampState{start: record.Time, minCurrentMA: currentMA, maxCurrentMA: currentMA}
			}
			if state.samples == 0 {
				state.minCurrentMA = currentMA
				state.maxCurrentMA = currentMA
			}
			if state.havePrevious {
				state.maxDeltaMA = math.Max(state.maxDeltaMA, abs(currentMA-state.previousCurrentMA))
			}
			state.previousCurrentMA = currentMA
			state.havePrevious = true
			state.minCurrentMA = math.Min(state.minCurrentMA, currentMA)
			state.maxCurrentMA = math.Max(state.maxCurrentMA, currentMA)
			state.samples++
			state.subjects = appendUnique(state.subjects, record.Subject)
			if state.sessionID == "" {
				state.sessionID = record.TestSessionID
			}
			rangeMA := state.maxCurrentMA - state.minCurrentMA
			if state.maxDeltaMA >= options.AmpFluctuationDeltaMA || (state.samples >= 3 && rangeMA >= options.AmpFluctuationRangeMA) {
				return []Event{{
					Type:           "telemetry-pattern",
					Pattern:        "amp-fluctuation",
					Severity:       SeverityWarning,
					Host:           options.Host,
					Axis:           axis,
					WindowMS:       options.AmpFluctuationWindow.Milliseconds(),
					SourceSubjects: state.subjects,
					TestSessionID:  state.sessionID,
					Evidence: map[string]any{
						"current_ma":            currentMA,
						"min_current_ma":        state.minCurrentMA,
						"max_current_ma":        state.maxCurrentMA,
						"range_ma":              rangeMA,
						"max_adjacent_delta_ma": state.maxDeltaMA,
						"samples":               state.samples,
						"delta_threshold_ma":    options.AmpFluctuationDeltaMA,
						"range_threshold_ma":    options.AmpFluctuationRangeMA,
					},
					SuggestedAction: "treat the motion as electrically jerky, correlate current spikes with velocity commands and zero-hold events, then smooth or stop the command stream before rerunning live motion",
				}}
			}
			states[axis] = state
		}
	}
	return nil
}

func patternEvent(pattern string, severity Severity, options DetectorOptions, record Record, axis int, evidence map[string]any, action string) Event {
	return Event{
		Type:            "telemetry-pattern",
		Pattern:         pattern,
		Severity:        severity,
		Host:            options.Host,
		Axis:            axis,
		SourceSubjects:  appendUnique(nil, record.Subject),
		TestSessionID:   record.TestSessionID,
		Evidence:        evidence,
		SuggestedAction: action,
	}
}

func affectedAxes(record Record, axisCount int) []int {
	if record.Axis >= 0 && record.Axis < axisCount {
		return []int{record.Axis}
	}
	axes := make([]int, 0, axisCount)
	for axis := 0; axis < axisCount; axis++ {
		if maskSet(record.AxisEnableMask|record.AxisErrorMask|record.AxisFaultMask, axis) {
			axes = append(axes, axis)
		}
	}
	if len(axes) > 0 {
		return axes
	}
	return []int{0}
}

func isCommandEvent(record Record) bool {
	return record.Event == "command.arm" || record.Event == "command.home" || record.Event == "command.position" || record.Command != ""
}

func commandName(record Record) string {
	command := strings.TrimSpace(record.Command)
	if command != "" {
		return command
	}
	switch record.Event {
	case "command.arm":
		if record.ServosArmedRequested {
			return "arm"
		}
		return "disarm"
	case "command.home":
		return "hm35-home"
	case "command.position":
		return "position"
	default:
		if strings.HasPrefix(record.Event, "command.") {
			return strings.TrimPrefix(record.Event, "command.")
		}
		return ""
	}
}

func maskSet(mask uint32, axis int) bool {
	return mask&(1<<uint(axis)) != 0
}

func appendUnique(items []string, item string) []string {
	if item == "" {
		return items
	}
	for _, existing := range items {
		if existing == item {
			return items
		}
	}
	return append(items, item)
}

func firstNonEmpty(values ...string) string {
	for _, value := range values {
		if value != "" {
			return value
		}
	}
	return ""
}

func valueAt(values []float64, index int) (float64, bool) {
	if index < 0 || index >= len(values) {
		return 0, false
	}
	return values[index], true
}

func boolAt(values []bool, index int) (bool, bool) {
	if index < 0 || index >= len(values) {
		return false, false
	}
	return values[index], true
}

func toleranceAt(values []float64, index int, fallback float64) float64 {
	if value, ok := valueAt(values, index); ok && value > 0 {
		return value
	}
	return fallback
}

func shortestPeriodicErrorRad(commandRad float64, feedbackRad float64) float64 {
	return math.Remainder(feedbackRad-commandRad, 2*math.Pi)
}

func abs(value float64) float64 {
	if value < 0 {
		return -value
	}
	return value
}

func topicToken(value string) string {
	value = strings.TrimSpace(value)
	var builder strings.Builder
	for _, char := range value {
		switch {
		case char >= 'a' && char <= 'z':
			builder.WriteRune(char)
		case char >= 'A' && char <= 'Z':
			builder.WriteRune(char)
		case char >= '0' && char <= '9':
			builder.WriteRune(char)
		case char == '-' || char == '_':
			builder.WriteRune(char)
		}
	}
	return builder.String()
}
