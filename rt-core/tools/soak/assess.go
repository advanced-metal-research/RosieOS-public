//go:build linux

package main

import (
	"fmt"
	"rosieos/rt-core/ipcclient"
	"sort"
)

// assessMeasurements checks raw iteration boundaries and event cursors, so a
// final zero slot count or a saved passing summary cannot hide earlier failures.
func (r *record) assessMeasurements(add func(string, bool, string)) {
	add("record_schema", r.Schema == "rosie.rt-core.soak.v2", r.Schema)
	add("measurements_available", len(r.Unavailable) == 0, fmt.Sprint(r.Unavailable))
	complete, slots, overlap := true, true, true
	count := 0
	for _, it := range r.Iterations {
		if it.Phase != "run" {
			continue
		}
		count++
		complete = complete && it.Number == count && it.Handle != 0 && it.StartSequence != 0 && it.Completed && it.ExecutedDiscarded && it.OverlapDiscarded && it.OverlapHandle != 0 && it.JogSent >= jogUpdates && it.UploadCompletedNS > it.UploadStartedNS && it.OverlapCompletedNS > it.OverlapStartedNS
		slots = slots && emptySlots(it.End)
		seen := false
		for _, j := range r.Jog {
			if j.Phase == "run" && j.Iteration == it.Number && j.DuringUpload && j.Observation.FirstObservedHostNs >= it.OverlapStartedNS && j.Observation.FirstObservedHostNs <= it.OverlapCompletedNS && j.Observation.HasInput == 1 && j.Observation.Open == 1 && j.Observation.UpdateReason == ipcclient.JogReasonNone {
				seen = true
			}
		}
		overlap = overlap && seen
	}
	add("iteration_workload", complete && count > 0 && count == r.IterationsCompleted && r.Configuration.Points == planPoints && r.Configuration.Iterations >= 2, fmt.Sprintf("%d full prepare/execute/discard iterations", count))
	add("iteration_slots_released", slots && count > 0, "every iteration has zero live handles/native reservations and two open adapter preparation slots")
	add("jog_during_each_upload", overlap && count > 0, "accepted native input observed inside every overlap upload")
	available, counters, rings := len(r.Samples) > 0, true, true
	var previous *sample
	for i := range r.Samples {
		s := &r.Samples[i]
		valid := s.Adapter.HeapInuse != nil && s.DaemonFaults != nil && s.IO != nil && s.NativeSlots != nil && s.EventRing != nil && s.PreparationSlots != nil && s.BufferHealth != nil && s.Daemon.RSS > 0 && s.Adapter.RSS > 0 && s.Metrics.TimeNS > 0 && len(s.Unavailable) == 0
		available = available && valid
		if !valid {
			continue
		}
		used, err := occupiedSlots(*s.BufferHealth)
		slots = slots && err == nil && used == *s.NativeSlots && *s.PreparationSlots <= 2
		r.Summary.NativeSlotsPeak = max(r.Summary.NativeSlotsPeak, used)
		r.Summary.PreparedHandlesPeak = max(r.Summary.PreparedHandlesPeak, s.PreparedHandles)
		r.Summary.EventRingPeak = max(r.Summary.EventRingPeak, s.EventRing.Occupancy)
		rings = rings && s.EventRing.Occupancy <= eventRingCapacity && ((s.EventRing.Latest == 0 && s.EventRing.Occupancy == 0) || (s.EventRing.First > 0 && s.EventRing.Latest >= s.EventRing.First && s.EventRing.Occupancy == s.EventRing.Latest-s.EventRing.First+1))
		if previous != nil {
			counters = counters && s.HostNS >= previous.HostNS && countersMonotonic(*previous, *s)
		}
		previous = s
	}
	add("sample_fields_present", available, "all process/status/event fields have explicit measurements")
	endpoints, reconciled := true, true
	var endpoint *sample
	for _, it := range r.Iterations {
		if endpoint != nil {
			endpoints = endpoints && it.End.HostNS >= endpoint.HostNS && countersMonotonic(*endpoint, it.End)
		}
		end := it.End
		endpoint = &end
		reconciled = reconciled && endpointMatchesSamples(end, r.Samples)
	}
	for _, end := range []*sample{r.Baseline, r.Final} {
		reconciled = reconciled && end != nil && endpointMatchesSamples(*end, r.Samples)
	}
	add("iteration_counter_continuity", endpoints, "cumulative counters never decrease across iteration endpoints")
	add("endpoint_sample_consistency", reconciled, "endpoint counters equal the sample at their timestamp or lie between the nearest surrounding samples")
	counters = counters && endpoints && reconciled
	add("slot_counts_consistent", slots, "three native slots including reusable retired slots")
	add("event_ring_bounded", rings && available, fmt.Sprintf("peak %d <= 256 retained events", r.Summary.EventRingPeak))
	cursor, continuous := uint64(0), len(r.EventBatches) > 0
	incarnation := ""
	for _, v := range r.EventBatches {
		b := v.Batch
		continuous = continuous && v.After == cursor && b.LatestSequence >= b.NextSequence
		if incarnation != "" && b.AdapterIncarnation != incarnation {
			continuous = false
		}
		if b.AdapterIncarnation != "" {
			incarnation = b.AdapterIncarnation
		}
		for _, event := range b.Events {
			continuous = continuous && event.Sequence == cursor+1 && event.Dropped == nil && event.Type != "events_dropped" && event.Overflow == nil && event.TimeNS > 0 && event.AdapterIncarnation == b.AdapterIncarnation
			cursor = event.Sequence
			r.Summary.EventCount++
		}
		continuous = continuous && b.NextSequence == cursor
	}
	if len(r.EventBatches) > 0 {
		continuous = continuous && r.EventBatches[len(r.EventBatches)-1].Batch.LatestSequence == cursor
	}
	if r.Final != nil && r.Final.EventRing != nil {
		continuous = continuous && r.Final.EventRing.Latest <= cursor
	}
	add("event_sequence_continuous", continuous && cursor > 0, fmt.Sprintf("%d ordered public events through sequence %d", r.Summary.EventCount, cursor))
	agesValid := len(r.Jog) > 0
	for _, j := range r.Jog {
		agesValid = agesValid && j.Observation.FirstObservedHostNs >= j.Observation.ObservedOriginHostNs && j.AgeNS == j.Observation.FirstObservedHostNs-j.Observation.ObservedOriginHostNs
	}
	add("jog_age_matches_receipt", agesValid, "recorded age equals native receipt minus original input timestamp")
	if r.Baseline != nil && r.Final != nil {
		b, f := r.Baseline, r.Final
		counters = counters && countersMonotonic(*b, *f)
		add("final_slots_released", emptySlots(*f), "no live handles/native reservations or occupied preparation lanes after release")
		if counters {
			r.Summary.DaemonFaults = &faults{Minor: f.DaemonFaults.Minor - b.DaemonFaults.Minor, Major: f.DaemonFaults.Major - b.DaemonFaults.Major}
			r.Summary.IO = &ioCounters{ReadBytes: f.IO.ReadBytes - b.IO.ReadBytes, WriteBytes: f.IO.WriteBytes - b.IO.WriteBytes, ReadChars: f.IO.ReadChars - b.IO.ReadChars, WriteChars: f.IO.WriteChars - b.IO.WriteChars}
		} else {
			r.summaryUnavailable("daemon_faults_delta", "missing or inconsistent process counters")
			r.summaryUnavailable("daemon_io_delta", "missing or inconsistent process counters")
		}
	}
	add("process_counter_continuity", counters && available, "daemon faults, I/O and timing counters never decrease at iteration boundaries")
}

func emptySlots(s sample) bool {
	return s.PreparedHandles == 0 && s.NativeSlots != nil && *s.NativeSlots == 0 && s.PreparationSlots != nil && *s.PreparationSlots == 2 && s.BufferHealth != nil && !s.BufferHealth.StagingInProgress
}

func countersMonotonic(b, f sample) bool {
	return b.DaemonFaults != nil && f.DaemonFaults != nil && b.IO != nil && f.IO != nil && b.BufferHealth != nil && f.BufferHealth != nil && b.EventRing != nil && f.EventRing != nil &&
		f.DaemonFaults.Minor >= b.DaemonFaults.Minor && f.DaemonFaults.Major >= b.DaemonFaults.Major &&
		f.IO.ReadBytes >= b.IO.ReadBytes && f.IO.WriteBytes >= b.IO.WriteBytes && f.IO.ReadChars >= b.IO.ReadChars && f.IO.WriteChars >= b.IO.WriteChars &&
		f.RT.Minor >= b.RT.Minor && f.RT.Major >= b.RT.Major &&
		f.Metrics.TimeNS >= b.Metrics.TimeNS && f.Metrics.Overruns >= b.Metrics.Overruns && f.Metrics.DiagnosticsDropped >= b.Metrics.DiagnosticsDropped &&
		f.Metrics.MaxJitterNS >= b.Metrics.MaxJitterNS && f.Metrics.MaxWorkNS >= b.Metrics.MaxWorkNS &&
		f.BufferHealth.TimeNS >= b.BufferHealth.TimeNS && f.BufferHealth.RingDrops >= b.BufferHealth.RingDrops &&
		f.EventRing.Latest >= b.EventRing.Latest && f.Daemon.HWM >= b.Daemon.HWM && f.Adapter.HWM >= b.Adapter.HWM
}

// endpointMatchesSamples requires an equal observation or two bracketing
// observations; an endpoint outside the stream cannot provide unchecked deltas.
func endpointMatchesSamples(end sample, samples []sample) bool {
	i := sort.Search(len(samples), func(i int) bool { return samples[i].HostNS >= end.HostNS })
	if i < len(samples) && samples[i].HostNS == end.HostNS {
		return countersMonotonic(samples[i], end) && countersMonotonic(end, samples[i])
	}
	return i > 0 && i < len(samples) && countersMonotonic(samples[i-1], end) && countersMonotonic(end, samples[i])
}
