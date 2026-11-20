//go:build linux

package control

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"sort"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

// Identity binds both the verified source sample digest and the canonical
// segment-record digest (including local clocks and process bits). StartProgram's
// full struct comparison requires both returned digests, even when they match.
type Identity struct {
	PlanID           string `json:"plan_id"`
	ProgramID        string `json:"program_id"`
	ProgramDigest    string `json:"program_digest"`
	TrajectoryDigest string `json:"trajectory_digest"`
	SourceDigest     string `json:"source_digest"`
	NormalisedDigest string `json:"normalised_digest"`
	ManifestRevision uint64 `json:"manifest_revision"`
	PlanRevision     uint64 `json:"plan_revision"`
}

type ProcessMarker struct {
	Monitor   string `json:"monitor,omitempty"`
	TimeoutMS uint32 `json:"timeout_ms,omitempty"`
	StableMS  uint32 `json:"stable_ms,omitempty"`
	TimeNS    int64  `json:"time_ns"`
	Segment   int    `json:"segment"`
	Sample    int    `json:"sample"`
	Action    string `json:"action"`
	Value     bool   `json:"value"`
}

type Program struct {
	Markers           []ProcessMarker `json:"process_markers"`
	Identity          Identity        `json:"identity"`
	RequiresProcessIO bool            `json:"requires_process_io"`
	Segments          int             `json:"segments"`
	// Samples preserves the source-count API; NormalisedSamples counts the
	// execution points after coincident endpoints have become shared samples.
	Samples           int    `json:"samples"`
	NormalisedSamples int    `json:"normalised_samples"`
	AxisMask          uint32 `json:"axis_mask"`
	points            []ipcclient.Point
	boundaries        []int
	torch             bool
	legacyTorch       bool
	torchQualified    bool
	handle            uint64
}

// DecodeProgram verifies source digests, canonicalises stationary seams, then
// enforces native limits without interpolation slack. Both sample identities are
// retained so Start cannot execute normalised content under a source-only identity.
// Axis order and segment clocks are preserved; no bridge interval is inserted.
// Optional homeOffsets are native-axis-ordered CSP offsets; omission selects the
// zero-Home frame for offline producers. Live preparation supplies current Home.
func DecodeProgram[D ipcclient.Description | Description](blob []byte, source D, axisIDs []string, homeOffsets ...int32) (*Program, error) {
	// Public descriptions carry the same native limits; accept only these two
	// description types so metadata cannot substitute an unrelated limit source.
	var description ipcclient.Description
	switch source := any(source).(type) {
	case ipcclient.Description:
		description = source
	case Description:
		description = source.NativeDescription
	}
	if len(homeOffsets) != 0 && len(homeOffsets) != len(description.Axes) {
		return nil, errors.New(ReasonInvalidDenseAxisMap)
	}
	if int64(len(blob)) > programPreparationBytes {
		return nil, errBodyTooLarge
	}
	if len(blob) >= 4 && int64(binary.BigEndian.Uint32(blob[:4])) > programEnvelopeBytes-4 {
		return nil, errBodyTooLarge
	}
	plan, decodeErr := densejointtraj.Inspect(blob)
	if plan == nil {
		return nil, decodeErr
	}
	// The wire is nine columns; a cell maps as many of them as it has rotary
	// axes (a six-axis Rosie 1420 maps six), the map derived from its own
	// description (DescribedDenseAxes). No rotary axis, or more than the wire
	// holds, keeps the catalogue's refusal: the reason's name still says
	// "nine ids" because the reason catalogue is protocol and unchanged.
	if len(axisIDs) == 0 || len(axisIDs) > densejointtraj.AxisCount {
		return nil, errors.New(ReasonDenseAxisMapRequiresNineIds)
	}
	mapping := make([]int, len(axisIDs))
	var mask uint32
	for source, id := range axisIDs {
		found := false
		for index, axis := range description.Axes {
			if id != axis.ID {
				continue
			}
			if axis.PositionUnit != "rad" || mask&(1<<index) != 0 ||
				(axis.Sign != -1 && axis.Sign != 1) || !(axis.CountsPerUnit > 0) || math.IsInf(axis.CountsPerUnit, 0) {
				return nil, errors.New(ReasonInvalidDenseAxisMap)
			}
			mapping[source] = index
			mask |= 1 << index
			found = true
			break
		}
		if !found {
			return nil, fmt.Errorf("%s: %s", ReasonUnmappedDenseAxis, id)
		}
	}
	// The columns past the map are the wire's spare axes. A program may not
	// command them (the header's mask names what the daemon sends) and every
	// sample must hold them still; a spare that is commanded or moves would be
	// motion the cell cannot map to a drive.
	for source := len(axisIDs); source < densejointtraj.AxisCount; source++ {
		if plan.Header.Axes.AxisMask&(1<<uint(source)) != 0 {
			return nil, fmt.Errorf("%s: source axis %d is commanded but the cell maps %d axes", ReasonUnmappedDenseAxis, source+1, len(axisIDs))
		}
		var first *densejointtraj.Sample
		for _, segment := range plan.Segments {
			for i := range segment.Samples {
				sample := &segment.Samples[i]
				if first == nil {
					first = sample
				}
				if sample.QRad[source] != first.QRad[source] || sample.QdRadS[source] != 0 {
					return nil, fmt.Errorf("%s: source axis %d moves but the cell maps %d axes", ReasonUnmappedDenseAxis, source+1, len(axisIDs))
				}
			}
		}
	}
	// Describe's native axis index breaks simultaneous ties even when the
	// producer supplied a different axis map; source storage remains unchanged.
	axisOrder := make([]int, len(mapping))
	for source := range mapping {
		axisOrder[source] = source
	}
	sort.Slice(axisOrder, func(i, j int) bool { return mapping[axisOrder[i]] < mapping[axisOrder[j]] })
	// Only seam validation errors receive the adapter's count-bearing reason;
	// corruption, invalid samples and unrelated format failures retain priority.
	var rejection *densejointtraj.RejectionError
	if errors.As(decodeErr, &rejection) && (rejection.Reason == ReasonBoundaryQdNonzero || rejection.Reason == ReasonBoundaryQDiscontinuity) {
		if boundary, gaps, moving := densejointtraj.MovingBoundary(plan); moving {
			counts := make([]float64, len(mapping))
			for source, index := range mapping {
				counts[source] = gaps[source] * description.Axes[index].CountsPerUnit
			}
			return nil, &SegmentBoundaryError{Boundary: boundary, GapCounts: counts}
		}
	}
	if decodeErr != nil {
		return nil, decodeErr
	}
	segments, err := densejointtraj.NormaliseStationary(plan)
	if err != nil {
		return nil, err
	}
	h := plan.Header
	result := &Program{Identity: Identity{
		PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest,
		TrajectoryDigest: h.TrajectoryDigest, SourceDigest: h.TrajectoryDigest,
		NormalisedDigest: densejointtraj.TrajectoryDigest(h.Axes.AxisMask, h.Sampling.DtS, h.Limits.MaxAbsQdRadS, segments),
		ManifestRevision: h.ManifestRevision, PlanRevision: h.PlanRevision},
		Segments: len(segments), Samples: h.Sampling.TotalSampleCount,
		NormalisedSamples: h.Sampling.TotalSampleCount - len(segments) + 1, AxisMask: mask}
	if h.PlanID == "" || h.ProgramID == "" || h.ProgramDigest == "" {
		return nil, errors.New(ReasonProgramIdentityMissing)
	}
	result.points = make([]ipcclient.Point, 0, result.NormalisedSamples)
	var offset time.Duration
	for segmentIndex, segment := range segments {
		start := len(result.points)
		if segmentIndex > 0 {
			start--
		}
		result.boundaries = append(result.boundaries, start)
		for sampleIndex, sample := range segment.Samples {
			// Native clocks are signed nanoseconds at the adapter boundary.
			// Reject unrepresentable/collapsed clocks before IPC can emit an
			// unclassified point error or overflow a later segment's offset.
			ns := math.Round(sample.TimeS * 1e9)
			if ns < 0 || ns >= float64(math.MaxInt64) || offset > time.Duration(math.MaxInt64)-time.Duration(ns) ||
				(sampleIndex > 0 && time.Duration(ns)+offset <= result.points[len(result.points)-1].Time) {
				return nil, fmt.Errorf("%s: segment=%d sample=%d", ReasonTimeGridInvalid, segmentIndex, sampleIndex)
			}
			point := ipcclient.Point{Time: time.Duration(ns),
				Position: make([]float64, len(description.Axes)), Velocity: make([]float64, len(description.Axes))}
			result.RequiresProcessIO = result.RequiresProcessIO || sample.TorchOn
			result.torch = result.torch || sample.TorchOn
			for _, source := range axisOrder {
				index := mapping[source]
				axis := description.Axes[index]
				q, qd := sample.QRad[source], sample.QdRadS[source]
				frame := densejointtraj.CommandFrame{CountsPerUnit: axis.CountsPerUnit, Sign: axis.Sign}
				if len(homeOffsets) != 0 {
					frame.HomeOffset = homeOffsets[index]
				}
				if sampleIndex == 0 {
					if err := densejointtraj.AdmitHermiteKnot(frame,
						densejointtraj.HermiteKnot{Position: q, Velocity: qd}, axis.MinPosition, axis.MaxPosition); err != nil {
						return nil, segmentPositionError(err, frame, segmentIndex, sampleIndex, axis.ID)
					}
				}
				if math.Abs(qd) > axis.MaxVelocity {
					return nil, fmt.Errorf("%s: segment=%d sample=%d axis=%s", ReasonNativeLimitExceeded, segmentIndex, sampleIndex, axis.ID)
				}
				if sampleIndex > 0 {
					previous := segment.Samples[sampleIndex-1]
					// Use rounded native clocks and the same continuous delta as
					// the sampler; check source and effective native bounds.
					bounds, err := densejointtraj.AdmitHermiteSegment(frame,
						densejointtraj.HermiteKnot{TimeNS: int64(math.Round(previous.TimeS * 1e9)), Position: previous.QRad[source], Velocity: previous.QdRadS[source]},
						densejointtraj.HermiteKnot{TimeNS: int64(point.Time), Position: q, Velocity: qd}, axis.MinPosition, axis.MaxPosition)
					if err != nil {
						return nil, segmentPositionError(err, frame, segmentIndex, sampleIndex, axis.ID)
					}
					peak := bounds.PeakVelocity
					limit := math.Min(axis.MaxVelocity, h.Limits.MaxAbsQdRadS[source])
					if math.IsNaN(peak) || math.IsInf(peak, 0) || peak > limit*axis.CountsPerUnit {
						return nil, &SegmentVelocityError{Segment: segmentIndex, Sample: sampleIndex, Axis: axis.ID, Peak: peak / axis.CountsPerUnit, Limit: limit}
					}
					// Dense samples always supply qd. Check its derivative using the
					// rounded native clock, so refusal precedes handle invalidation.
					if axis.MaxAcceleration != nil {
						dt := (point.Time - time.Duration(math.Round(previous.TimeS*1e9))).Seconds()
						acceleration := (qd - previous.QdRadS[source]) / dt
						limit := *axis.MaxAcceleration
						if dt <= 0 || math.IsNaN(acceleration) || math.IsInf(acceleration, 0) ||
							math.IsNaN(limit) || math.IsInf(limit, 0) || limit <= 0 || math.Abs(acceleration) > limit {
							return nil, fmt.Errorf("%s: segment=%d sample=%d axis=%s", ReasonNativeLimitExceeded, segmentIndex, sampleIndex, axis.ID)
						}
					}
				}
				point.Position[index] = q
				point.Velocity[index] = qd
			}
			point.Time += offset
			if len(result.Markers) == 0 || result.Markers[len(result.Markers)-1].Value != sample.TorchOn {
				result.Markers = append(result.Markers, ProcessMarker{TimeNS: int64(point.Time), Segment: segmentIndex, Sample: sampleIndex, Action: "torch", Value: sample.TorchOn})
			}
			if segmentIndex > 0 && sampleIndex == 0 {
				result.points[len(result.points)-1] = point
			} else {
				result.points = append(result.points, point)
			}
		}
		offset = result.points[len(result.points)-1].Time
	}
	// Header markers preserve source segment/sample association across collapsed
	// stationary endpoints. Bind the whole source envelope when this extension is
	// present so changing an output intent cannot retain its prepared identity.
	result.legacyTorch = result.torch
	if len(h.ProcessMarkers) > 0 {
		// Discard the synthetic initial OFF when the source has no torch bits.
		// Explicit header actions (including an output literally named torch) own their schedule.
		if !result.legacyTorch {
			result.Markers = nil
		}
		if len(h.ProcessMarkers) > len(result.points)*16 {
			return nil, errors.New(ReasonIoMarkerInvalid)
		}
		segmentOffsets := make([]time.Duration, len(segments))
		for i := 1; i < len(segments); i++ {
			previous := segments[i-1].Samples
			segmentOffsets[i] = segmentOffsets[i-1] + time.Duration(math.Round(previous[len(previous)-1].TimeS*1e9))
		}
		for _, marker := range h.ProcessMarkers {
			if marker.Segment < 0 || marker.Segment >= len(segments) || marker.Sample < 0 || marker.Sample >= len(segments[marker.Segment].Samples) || marker.Action == "" {
				return nil, errors.New(ReasonIoMarkerInvalid)
			}
			timeNS := int64(segmentOffsets[marker.Segment]) + int64(math.Round(segments[marker.Segment].Samples[marker.Sample].TimeS*1e9))
			result.Markers = append(result.Markers, ProcessMarker{TimeNS: timeNS, Segment: marker.Segment, Sample: marker.Sample, Action: marker.Action, Value: marker.Value, TimeoutMS: marker.TimeoutMS, StableMS: marker.StableMS, Monitor: marker.Monitor})
			result.RequiresProcessIO = true
			if marker.Action == "torch" && marker.TimeoutMS == 0 && marker.Monitor == "" {
				result.torch = true
			}
		}
		sourceSum := sha256.Sum256(blob)
		result.Identity.SourceDigest = "sha256:" + hex.EncodeToString(sourceSum[:])
		encoded, _ := json.Marshal(h.ProcessMarkers)
		normalisedSum := sha256.Sum256(append([]byte(result.Identity.NormalisedDigest), encoded...))
		result.Identity.NormalisedDigest = "sha256:" + hex.EncodeToString(normalisedSum[:])
	}
	return result, nil
}

// Preserve the same sample and native axis ordering for knot and interval
// refusals; converting count details must not discard the recovery reason.
func segmentPositionError(err error, frame densejointtraj.CommandFrame, segment, sample int, axis string) error {
	var position *densejointtraj.PositionLimitError
	if !errors.As(err, &position) {
		return errors.New(ReasonNativeSegmentRateExceeded)
	}
	scale, home := frame.CountsPerUnit*float64(frame.Sign), float64(frame.HomeOffset)
	detail := SegmentPositionError{Segment: segment, Sample: sample, Axis: axis,
		Position: (position.Position + home) / scale, Limit: (position.Limit + home) / scale}
	if position.Recovery {
		return &SegmentRecoveryError{SegmentPositionError: detail}
	}
	return &detail
}

// Decode in the current native Home frame so offset cancellation cannot change
// a velocity-boundary decision between the adapter and the native bulk worker.
func (c *Controller) decodeProgram(blob []byte) (*Program, error) {
	if c.native == nil {
		return DecodeProgram(blob, c.description, c.axes)
	}
	state, err := c.core.Status()
	if err != nil {
		// A new frame read must preserve the existing native-loss cleanup;
		// otherwise an early return can leave a dead prepared handle published.
		if c.core.Err() != nil || errors.Is(err, ipcclient.ErrRevoked) {
			c.mu.Lock()
			previous := c.program
			c.mu.Unlock()
			c.forgetProgram(previous, err)
		}
		return nil, err
	}
	if len(state.Axes) != len(c.description.Axes) {
		return nil, errors.New(ReasonInvalidDenseAxisMap)
	}
	offsets := make([]int32, len(state.Axes))
	for i, axis := range state.Axes {
		offsets[i] = axis.HomeOffset
	}
	return DecodeProgram(blob, c.description, c.axes, offsets...)
}

// SegmentPositionError names the logical position and bound that prevent a
// prepared handle from admitting a knot or an interior command-frame overshoot.
type SegmentPositionError struct {
	Segment, Sample int
	Axis            string
	Position, Limit float64
}

func (e *SegmentPositionError) Error() string {
	return fmt.Sprintf("%s: segment=%d sample=%d axis=%s", ReasonNativeLimitExceeded, e.Segment, e.Sample, e.Axis)
}

// SegmentRecoveryError preserves the exterior-direction reason before native
// upload, preventing a rejected recovery curve from replacing a prepared plan.
type SegmentRecoveryError struct{ SegmentPositionError }

func (e *SegmentRecoveryError) Error() string { return ReasonOutsideLimitsOutward }

// SegmentVelocityError identifies a continuous Hermite velocity refusal in rad/s.
// The stable public reason prevents clients from treating a bad curve as prepared;
// typed callers also retain the source axis, segment, ending sample and bound.
type SegmentVelocityError struct {
	Segment, Sample int
	Axis            string
	Peak, Limit     float64
}

func (e *SegmentVelocityError) Error() string { return ReasonNativeSegmentRateExceeded }

// SegmentBoundaryError reports the first moving discontinuity in source axis
// order. Boundary zero joins segments zero and one; gaps are second minus first.
type SegmentBoundaryError struct {
	Boundary  int
	GapCounts []float64
}

func (e *SegmentBoundaryError) Error() string {
	return fmt.Sprintf("%s: boundary=%d gap_counts=%v", ReasonSegmentBoundaryDiscontinuous, e.Boundary, e.GapCounts)
}

// DescribeProgram returns detached prepared metadata under the controller lock,
// preventing callers from mutating the identity or process markers used by Start.
func (c *Controller) DescribeProgram() *Program {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.program == nil {
		return nil
	}
	view := *c.program
	view.Markers = append([]ProcessMarker(nil), c.program.Markers...)
	view.points, view.boundaries = nil, nil
	return &view
}
