package denseexec

import (
	"bytes"
	"context"
	"errors"
	"net/http"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// telemetryReader is the SDK surface the ring window needs. The public
// telemetry endpoint has no cursor for "the newest N seconds": a cursor
// behind the ring returns the oldest records, a cursor past the head is
// refused, and every read below the head costs a full batch. The window
// therefore anchors (sequence, host time) once per daemon incarnation with a
// peek that reads one record, estimates the head from the daemon's status
// time, and reads only the records it wants.
type telemetryReader interface {
	TelemetryBatches(context.Context, uint64) (control.TelemetryBatch, error)
	TelemetryPeek(context.Context, uint64) (ipcclient.TelemetryBatchHeaderV1, *ipcclient.CycleCaptureRecordV2, error)
}

// telemetryAnchor is one (sequence, daemon host time) pair from the ring for
// one daemon incarnation; the head is that pair plus elapsed cycles.
type telemetryAnchor struct {
	daemon   string
	sequence uint64
	timeNS   uint64
	periodNS uint64
}

// Bounds an operator can ask for: the ring keeps about 100 s at 1 kHz and one
// second is 4.9 MB on the wire before decimation, so a window is short.
const (
	telemetryWindowMaxSeconds = 100.0
	telemetryWindowMaxStride  = 100
	// Unverified bound: batches read for one window, at most 4096 records each.
	telemetryWindowMaxBatches = 64
)

// TelemetryWindow is the newest `seconds` of the ring, every `stride`-th
// record, with the fields the operator's strip plots: per axis the encoder
// position and target, the drive's velocity and its target, the following
// error and the torque, each with its validity; per record the cycle timing
// and the group state that says whether the motion was commanded.
type TelemetryWindow struct {
	DaemonIncarnation   string                  `json:"daemon_incarnation"`
	ConfigurationSHA256 string                  `json:"configuration_sha256"`
	CyclePeriodNS       uint64                  `json:"cycle_period_ns"`
	AxisCount           int                     `json:"axis_count"`
	Stride              int                     `json:"stride"`
	Seconds             float64                 `json:"seconds"`
	FirstSequence       uint64                  `json:"first_sequence"`
	LastSequence        uint64                  `json:"last_sequence"`
	HeadEstimate        uint64                  `json:"head_estimate"`
	FetchedRecords      int                     `json:"fetched_records"`
	FetchMS             int64                   `json:"fetch_ms"`
	Records             []TelemetryWindowRecord `json:"records"`
}

type TelemetryWindowRecord struct {
	ConfigurationEpoch    uint64                `json:"configuration_epoch"`
	HomeEpoch             uint64                `json:"home_epoch"`
	TrajectoryID          uint64                `json:"trajectory_id"`
	SubmittedMask         uint32                `json:"submitted_mask"`
	Seq                   uint64                `json:"seq"`
	TNs                   uint64                `json:"t_ns"`
	WorkNs                uint64                `json:"work_ns"`
	JitterNs              int64                 `json:"jitter_ns"`
	Armed                 bool                  `json:"armed"`
	JogOpen               bool                  `json:"jog_open"`
	State                 uint32                `json:"state"`
	ActiveMode            uint32                `json:"mode"`
	SafetyFaultMask       uint32                `json:"fault_mask"`
	ExecutionFaultReasons uint32                `json:"exec_fault"`
	Ax                    []TelemetryWindowAxis `json:"ax"`
}

type TelemetryWindowAxis struct {
	CoordinateCounts int64 `json:"coordinate_counts"`
	CoordinateOK     bool  `json:"coordinate_ok"`
	HomeOffset       int32 `json:"home_offset"`
	P                int32 `json:"p"`
	TP               int32 `json:"tp"`
	V                int32 `json:"v"`
	VOK              bool  `json:"v_ok"`
	TV               int32 `json:"tv"`
	TVOK             bool  `json:"tv_ok"`
	FE               int32 `json:"fe"`
	FEOK             bool  `json:"fe_ok"`
	TQ               int16 `json:"tq"`
	TQOK             bool  `json:"tq_ok"`
}

// TelemetryWindow reads the newest `seconds` of the daemon's ring through the
// public telemetry endpoint and returns every `stride`-th record.
func (r *RTCoreController) TelemetryWindow(ctx context.Context, seconds float64, stride int) (*TelemetryWindow, error) {
	if !(seconds > 0) || seconds > telemetryWindowMaxSeconds {
		return nil, httpError(http.StatusBadRequest, "telemetry_window_invalid", "seconds must be in (0, %g]", telemetryWindowMaxSeconds)
	}
	if stride < 1 || stride > telemetryWindowMaxStride {
		return nil, httpError(http.StatusBadRequest, "telemetry_window_invalid", "stride must be in [1, %d]", telemetryWindowMaxStride)
	}
	reader, ok := r.c.(telemetryReader)
	if !ok {
		return nil, httpError(http.StatusServiceUnavailable, "telemetry_unavailable", "the connected client does not read telemetry")
	}
	started := time.Now()

	// The daemon's clock now, from the latest observation or a fresh status.
	r.mu.Lock()
	observation := r.observation
	anchor := r.telemetryAnchor
	configurationSHA256 := r.description.ConfigurationSHA256
	r.mu.Unlock()
	var nowNS uint64
	var daemon string
	if observation != nil && observation.TimeNS > 0 {
		nowNS, daemon = observation.TimeNS, observation.DaemonIncarnation
	} else {
		status, err := r.c.Status(ctx)
		if err != nil {
			return nil, err
		}
		nowNS, daemon = status.TimeNS, status.DaemonIncarnation
		observation = &status
	}

	// One (sequence, time) anchor per daemon incarnation.
	if anchor == nil || anchor.daemon != daemon {
		header, first, err := reader.TelemetryPeek(ctx, 0)
		if err != nil {
			return nil, err
		}
		period := header.CyclePeriodNs
		if period == 0 {
			period = 1_000_000
		}
		if first == nil {
			return &TelemetryWindow{CyclePeriodNS: period, AxisCount: int(header.AxisCount), Stride: stride, Seconds: seconds, FetchMS: time.Since(started).Milliseconds(), Records: []TelemetryWindowRecord{}}, nil
		}
		anchor = &telemetryAnchor{daemon: incarnationString(header.DaemonIncarnation), sequence: first.Seq, timeNS: first.TNs, periodNS: period}
		if anchor.daemon != daemon && daemon != "" {
			// The status and the ring name different daemons: the daemon restarted
			// between the two reads. The ring's word stands for the ring.
			daemon = anchor.daemon
		}
		r.mu.Lock()
		r.telemetryAnchor = anchor
		r.mu.Unlock()
	}
	period := anchor.periodNS
	head := anchor.sequence
	if nowNS > anchor.timeNS {
		head += (nowNS - anchor.timeNS) / period
	}
	want := uint64(seconds * 1e9 / float64(period))
	if want == 0 {
		want = 1
	}
	// Unverified margin: a tenth of the window plus 50 cycles of clock slack
	// between the status observation and the ring head.
	margin := want/10 + 50
	after := uint64(0)
	if head > want+margin {
		after = head - want - margin
	}

	var records []ipcclient.CycleCaptureRecordV2
	var axisCount uint32
	backoffs := 0
	for batches := 0; batches < telemetryWindowMaxBatches; batches++ {
		batch, err := reader.TelemetryBatches(ctx, after)
		if err != nil {
			var rejected *control.Rejected
			if errors.As(err, &rejected) && rejected.Reason == "telemetry_cursor_ahead" && backoffs < 3 {
				// The estimate ran past the head (the status clock leads the ring):
				// step the cursor back by a window and try again.
				backoffs++
				if after > want {
					after -= want
				} else {
					after = 0
				}
				continue
			}
			return nil, err
		}
		axisCount = batch.Header.AxisCount
		if incarnationString(batch.Header.DaemonIncarnation) != anchor.daemon {
			return nil, httpError(http.StatusConflict, "telemetry_identity_changed", "controller restarted during capture; capture again")
		}
		if len(batch.Records) == 0 {
			break
		}
		records = append(records, batch.Records...)
		after = batch.Header.LastSequence
		if uint32(len(batch.Records)) < ipcclient.TelemetryBatchMaxRecords || uint64(len(records)) >= want*3 {
			break
		}
	}
	// A status clock that leads the ring leaves the first read short of the
	// window: fill the front from older sequences, bounded, until the window is
	// full or the ring's oldest record is reached.
	frontAfter := after
	if len(records) > 0 {
		frontAfter = records[0].Seq - 1
	}
	for fill := 0; fill < 3 && len(records) > 0 && uint64(len(records)) < want && frontAfter > 0; fill++ {
		need := want - uint64(len(records))
		start := uint64(0)
		if frontAfter > need+margin {
			start = frontAfter - need - margin
		}
		batch, err := reader.TelemetryBatches(ctx, start)
		if err != nil {
			break
		}
		var older []ipcclient.CycleCaptureRecordV2
		if incarnationString(batch.Header.DaemonIncarnation) != anchor.daemon {
			return nil, httpError(http.StatusConflict, "telemetry_identity_changed", "controller restarted during capture; capture again")
		}
		for _, v := range batch.Records {
			if v.Seq <= frontAfter {
				older = append(older, v)
			}
		}
		if len(older) == 0 {
			break
		}
		records = append(older, records...)
		if batch.Header.Dropped > 0 || start == 0 || older[0].Seq <= start+1 {
			// The ring has nothing older than this.
			if batch.Header.FirstSequence > start+1 {
				break
			}
		}
		frontAfter = older[0].Seq - 1
	}
	window := &TelemetryWindow{CyclePeriodNS: period, AxisCount: int(axisCount), Stride: stride, Seconds: seconds, HeadEstimate: head, FetchedRecords: len(records), Records: []TelemetryWindowRecord{}}
	window.DaemonIncarnation, window.ConfigurationSHA256 = anchor.daemon, configurationSHA256
	if len(records) == 0 {
		window.FetchMS = time.Since(started).Milliseconds()
		return window, nil
	}
	// The newest record is the better anchor for the next window.
	last := records[len(records)-1]
	if observation != nil && observation.Generations.ConfigurationEpoch != 0 &&
		(last.ConfigurationEpoch != observation.Generations.ConfigurationEpoch || observation.Core.ConfigurationSHA256 != configurationSHA256) {
		return nil, httpError(http.StatusConflict, "telemetry_identity_changed", "configuration changed during capture; refresh the machine description")
	}
	r.mu.Lock()
	r.telemetryAnchor = &telemetryAnchor{daemon: anchor.daemon, sequence: last.Seq, timeNS: last.TNs, periodNS: period}
	r.mu.Unlock()
	if uint64(len(records)) > want {
		records = records[uint64(len(records))-want:]
	}
	window.FirstSequence, window.LastSequence = records[0].Seq, last.Seq
	for i := 0; i < len(records); i += stride {
		window.Records = append(window.Records, windowRecord(records[i], int(axisCount)))
	}
	window.FetchMS = time.Since(started).Milliseconds()
	return window, nil
}

func windowRecord(v ipcclient.CycleCaptureRecordV2, axisCount int) TelemetryWindowRecord {
	if axisCount > len(v.Ax) {
		axisCount = len(v.Ax)
	}
	out := TelemetryWindowRecord{
		ConfigurationEpoch: v.ConfigurationEpoch, HomeEpoch: v.HomeEpoch,
		TrajectoryID: v.ActiveTrajId, SubmittedMask: v.SubmittedAxisMask,
		Seq: v.Seq, TNs: v.TNs, WorkNs: v.WorkNs, JitterNs: v.CycleJitterNs,
		Armed: v.Armed != 0, JogOpen: v.JogSessionOpen != 0, State: v.State, ActiveMode: v.ActiveMode,
		SafetyFaultMask: v.SafetyFaultMask, ExecutionFaultReasons: v.ExecutionFaultReasons,
		Ax: make([]TelemetryWindowAxis, 0, axisCount),
	}
	for i := 0; i < axisCount; i++ {
		a := v.Ax[i]
		out.Ax = append(out.Ax, TelemetryWindowAxis{
			CoordinateCounts: a.CoordinateCounts, HomeOffset: a.NativeHomePositionOffset,
			CoordinateOK: a.CoordinateValid != 0 && a.CoordinateSourceValid != 0 && a.PdoFresh != 0 && (v.HomeValidMask&(1<<i)) != 0,
			P:            a.Position, TP: a.Target,
			V: a.VelocityActualCountsPerS, VOK: a.VelocityActualValid != 0,
			TV: a.TargetVelocityCountsPerS, TVOK: a.TargetVelocityValid != 0,
			FE: a.FollowingErrorCounts, FEOK: a.FollowingErrorValid != 0,
			TQ: a.TorqueRaw, TQOK: a.TorqueValid != 0,
		})
	}
	return out
}

func incarnationString(raw [32]byte) string {
	return string(bytes.TrimRight(raw[:], "\x00"))
}
