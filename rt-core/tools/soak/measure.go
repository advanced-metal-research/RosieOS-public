//go:build linux

package main

import (
	"context"
	"errors"
	"fmt"
	"os"
	"strconv"
	"strings"

	"rosieos/rt-core/ipcclient"
	control "rosieos/rt-core/sdk/control"
)

// Unverified adapter retention contract: events.go bounds history to 256 entries.
const eventRingCapacity uint64 = 256

type ioCounters struct {
	ReadBytes  uint64 `json:"read_bytes"`
	WriteBytes uint64 `json:"write_bytes"`
	ReadChars  uint64 `json:"rchar"`
	WriteChars uint64 `json:"wchar"`
}

type eventRingSample struct {
	First              uint64 `json:"first_retained_sequence"`
	Latest             uint64 `json:"latest_sequence"`
	Occupancy          uint64 `json:"occupancy"`
	AdapterIncarnation string `json:"adapter_incarnation"`
}

type eventBatchSample struct {
	HostNS    uint64             `json:"host_ns"`
	Phase     string             `json:"phase"`
	Iteration int                `json:"iteration"`
	After     uint64             `json:"after"`
	Batch     control.EventBatch `json:"batch"`
}

// parseIO requires named kernel byte counters; absent or malformed rows cannot
// masquerade as zero I/O. rchar/wchar also expose traffic served without disk I/O.
func parseIO(raw string) (ioCounters, error) {
	var result ioCounters
	wanted := map[string]*uint64{"read_bytes:": &result.ReadBytes, "write_bytes:": &result.WriteBytes, "rchar:": &result.ReadChars, "wchar:": &result.WriteChars}
	seen := map[string]bool{}
	for _, line := range strings.Split(raw, "\n") {
		fields := strings.Fields(line)
		if len(fields) == 0 {
			continue
		}
		p, ok := wanted[fields[0]]
		if !ok {
			continue
		}
		if len(fields) != 2 || seen[fields[0]] {
			return result, errors.New("invalid /proc I/O row")
		}
		n, err := strconv.ParseUint(fields[1], 10, 64)
		if err != nil {
			return result, err
		}
		*p, seen[fields[0]] = n, true
	}
	if len(seen) != len(wanted) {
		return result, errors.New("/proc I/O byte counters absent")
	}
	return result, nil
}

// retainedEvents uses a cursor-zero probe's explicit oldest retained sequence;
// a bounded response batch is not mistaken for the entire ring occupancy.
func retainedEvents(b control.EventBatch) (eventRingSample, error) {
	r := eventRingSample{Latest: b.LatestSequence, AdapterIncarnation: b.AdapterIncarnation}
	if b.LatestSequence == 0 && len(b.Events) == 0 {
		return r, nil
	}
	if len(b.Events) == 0 {
		return r, errors.New("event retention boundary absent")
	}
	first := b.Events[0]
	r.First = first.Sequence
	if first.Dropped != nil {
		if first.Dropped.FirstLostSequence != 1 || first.Dropped.LastLostSequence != first.Sequence {
			return r, errors.New("invalid retention drop range")
		}
		r.First++
	}
	if r.First == 0 || r.First > r.Latest {
		return r, errors.New("invalid retained event range")
	}
	r.Occupancy = r.Latest - r.First + 1
	return r, nil
}

// occupiedSlots validates every native state and treats retired reservations as
// reusable, matching TrajectoryStore::begin rather than falsely reporting leaks.
func occupiedSlots(b ipcclient.BufferHealth) (uint64, error) {
	if b.TimeNS == 0 {
		return 0, errors.New("native occupancy timestamp absent")
	}
	var total, used uint64
	for _, key := range []string{"free", "writing", "prepared", "ready", "active", "retired"} {
		n, ok := b.NativeSlotOccupancy[key]
		if !ok {
			return 0, fmt.Errorf("native slot state %s absent", key)
		}
		total += uint64(n)
		if key != "free" && key != "retired" {
			used += uint64(n)
		}
	}
	if total != nativeSlotCapacity || uint64(b.SlotsFree)+used != total {
		return 0, errors.New("inconsistent native slot counts")
	}
	return used, nil
}

func (o *observer) extraSample(ctx context.Context, s *sample, status control.Status) {
	s.Unavailable = map[string]string{}
	missing := func(key string, err error) { s.Unavailable[key] = err.Error() }
	if status.Adapter.HeapInuse == 0 {
		missing("adapter_heap_inuse_bytes", errors.New("public status adapter.heap_inuse absent or zero; /proc does not identify Go heap"))
	} else {
		heap := status.Adapter.HeapInuse
		s.Adapter.HeapInuse = &heap
	}
	s.PreparationSlots = &status.Adapter.OpenPreparationSlots
	s.BufferHealth = &status.BufferHealth
	if used, err := occupiedSlots(status.BufferHealth); err != nil {
		missing("native_slots_in_use", err)
	} else {
		s.NativeSlots = &used
	}
	raw, err := os.ReadFile(fmt.Sprintf("/proc/%d/stat", o.r.Configuration.DaemonPID))
	if err == nil {
		var f faults
		f, err = parseFaults(string(raw))
		if err == nil {
			s.DaemonFaults = &f
		}
	}
	if err != nil {
		missing("daemon_faults", err)
	}
	raw, err = os.ReadFile(fmt.Sprintf("/proc/%d/io", o.r.Configuration.DaemonPID))
	if err == nil {
		var v ioCounters
		v, err = parseIO(string(raw))
		if err == nil {
			s.IO = &v
		}
	}
	if err != nil {
		missing("daemon_io", err)
	}
	b, err := o.s.client.EventsContext(ctx, 0)
	if err == nil {
		var ring eventRingSample
		ring, err = retainedEvents(b)
		if err == nil {
			s.EventRing = &ring
		}
	}
	if err != nil {
		missing("event_ring", err)
	}
}

// observeEvents drains real public event batches from one serialized cursor;
// explicit dropped ranges remain in the record and fail continuity assessment.
func (o *observer) observeEvents(ctx context.Context) error {
	o.eventMu.Lock()
	defer o.eventMu.Unlock()
	for {
		b, err := o.s.client.EventsContext(ctx, o.eventCursor)
		if err != nil {
			return err
		}
		o.mu.Lock()
		o.r.EventBatches = append(o.r.EventBatches, eventBatchSample{ipcclient.HostMonotonicNS(), o.phase, o.iteration, o.eventCursor, b})
		o.mu.Unlock()
		o.eventCursor = b.NextSequence
		if b.NextSequence == b.LatestSequence {
			return nil
		}
	}
}
