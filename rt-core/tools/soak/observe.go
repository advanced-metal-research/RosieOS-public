//go:build linux

package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/procfs"
	"strconv"
	"strings"
	"sync"
	"time"

	"rosieos/rt-core/ipcclient"
	control "rosieos/rt-core/sdk/control"
)

var processMemory = procfs.ProcessMemory
var parseMemory = procfs.ParseMemory
var cycleTID = procfs.CycleTID
var threadFaults = procfs.ThreadFaults
var parseFaults = procfs.ParseFaults

func loadAverage() (float64, error) {
	raw, err := os.ReadFile("/proc/loadavg")
	if err != nil {
		return 0, err
	}
	f := strings.Fields(string(raw))
	if len(f) != 5 {
		return 0, errors.New("invalid /proc/loadavg")
	}
	return strconv.ParseFloat(f[0], 64)
}

// readMetrics requires every native timing counter in the published file. The
// SDK's typed Status omits these fields, so it cannot supply timing measurements.
func readMetrics(path string) (nativeMetrics, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nativeMetrics{}, err
	}
	var fields map[string]json.RawMessage
	if err := json.Unmarshal(raw, &fields); err != nil {
		return nativeMetrics{}, err
	}
	var m nativeMetrics
	for key, dest := range map[string]*uint64{"time_ns": &m.TimeNS, "rt_max_abs_jitter_ns": &m.MaxJitterNS, "rt_max_cycle_work_ns": &m.MaxWorkNS, "rt_overrun_count": &m.Overruns, "diagnostics_dropped": &m.DiagnosticsDropped} {
		v, ok := fields[key]
		if !ok || string(v) == "null" {
			return m, fmt.Errorf("native metric %s unavailable", key)
		}
		if err := json.Unmarshal(v, dest); err != nil {
			return m, err
		}
	}
	if m.TimeNS == 0 {
		return m, errors.New("native metrics timestamp absent")
	}
	return m, nil
}

type observer struct {
	sampleMu                     sync.Mutex
	eventMu                      sync.Mutex
	eventCursor                  uint64
	mu                           sync.Mutex
	r                            *record
	s                            *service
	phase                        string
	iteration                    int
	upload                       bool
	previousStatus, previousPoll uint64
	previousMotion               ipcclient.MotionState
	previousFault                uint32
	previousJog                  ipcclient.JogObservation
	maxGap                       uint64
	ages                         []uint64
}

func (o *observer) stage(phase string, iteration int) {
	o.mu.Lock()
	defer o.mu.Unlock()
	o.phase, o.iteration = phase, iteration
}

func (o *observer) uploading(value bool) {
	o.mu.Lock()
	defer o.mu.Unlock()
	o.upload = value
}

func (o *observer) observe(ctx context.Context) error {
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	if err := o.observeEvents(call); err != nil {
		return err
	}
	state, err := o.s.client.Status(call)
	if err != nil {
		return err
	}
	var jog ipcclient.JogObservation
	err = poll(call, operationBudget, func() (bool, error) {
		var err error
		jog, err = o.s.client.JogStatus(call)
		var rejected *control.Rejected
		// A seqlock reader can lose its bounded attempts to a concurrent writer.
		// Retry only this explicit snapshot miss; transport/admission errors fail.
		if errors.As(err, &rejected) && rejected.Reason == "RTCore jog observation unavailable" {
			return false, nil
		}
		return err == nil, err
	})
	if err != nil {
		return err
	}
	now := ipcclient.HostMonotonicNS()
	o.mu.Lock()
	defer o.mu.Unlock()
	if state.Core.TimeNS < o.previousStatus {
		return errors.New("status time regressed")
	}
	if state.Core.TimeNS != o.previousStatus {
		p := publication{HostNS: now, TimeNS: state.Core.TimeNS}
		if o.previousStatus != 0 {
			p.GapNS = state.Core.TimeNS - o.previousStatus
		}
		if o.previousPoll != 0 {
			p.ObserverGapNS = now - o.previousPoll
		}
		o.maxGap = max(o.maxGap, p.GapNS)
		o.r.Publications = append(o.r.Publications, p)
		o.previousStatus = state.Core.TimeNS
	}
	o.previousPoll = now
	m := state.Motion
	previous := o.previousMotion
	if m.CommandSequence != previous.CommandSequence || m.State != previous.State || m.Mode != previous.Mode || m.LastEvent != previous.LastEvent || state.Core.ExecutionFaultReasons != o.previousFault {
		o.r.Events = append(o.r.Events, event{now, o.phase, o.iteration, m, state.Execution})
		o.previousMotion, o.previousFault = m, state.Core.ExecutionFaultReasons
	}
	if newJogObservation(o.previousJog, jog) {
		if jog.FirstObservedHostNs < jog.ObservedOriginHostNs {
			return errors.New("jog observation predates source")
		}
		age := jog.FirstObservedHostNs - jog.ObservedOriginHostNs
		o.r.Jog = append(o.r.Jog, jogSample{now, o.phase, o.iteration, o.upload, age, jog})
		o.ages = append(o.ages, age)
		o.previousJog = jog
	}
	return nil
}

// newJogObservation identifies the first native receipt, not the current grant.
// Stop/reset can change the grant while retaining the last input observation;
// counting that retained receipt again would inflate latency sample counts.
func newJogObservation(previous, next ipcclient.JogObservation) bool {
	return next.ObservedSourceSequence != 0 && next.FirstObservedHostNs != 0 && next.FirstObservedHostNs > previous.FirstObservedHostNs
}

func (o *observer) sample(ctx context.Context, kind string) (s sample, err error) {
	o.sampleMu.Lock()
	defer o.sampleMu.Unlock()
	s.HostNS = ipcclient.HostMonotonicNS()
	s.Kind = kind
	c := o.r.Configuration
	s.Daemon, err = processMemory(c.DaemonPID)
	if err != nil {
		o.unavailable("daemon_rss_bytes", err)
		return s, err
	}
	s.Adapter, err = processMemory(c.AdapterPID)
	if err != nil {
		o.unavailable("adapter_rss_bytes", err)
		return s, err
	}
	s.RT, err = threadFaults(c.DaemonPID, c.RTTID)
	if err != nil {
		o.unavailable("rt_thread_faults", err)
		return s, err
	}
	s.Metrics, err = readMetrics(filepath.Join(o.s.sockets, "metrics.json"))
	if err != nil {
		o.unavailable("native_metrics", err)
		return s, err
	}
	s.Load1, err = loadAverage()
	if err != nil {
		return s, err
	}
	s.HostLoaded = s.Load1 > float64(c.CPUCount)*loadPerCPU
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	status, err := o.s.client.Status(call)
	if err != nil {
		return s, err
	}
	for _, h := range status.Execution.Handles {
		if h.State == "prepared" || h.State == "started" {
			s.PreparedHandles++
		}
	}
	o.extraSample(call, &s, status)
	o.mu.Lock()
	defer o.mu.Unlock()
	s.Phase, s.Iteration = o.phase, o.iteration
	s.PublicationGapNS, s.Jog = o.maxGap, distribution(o.ages)
	for field, reason := range s.Unavailable {
		o.r.Unavailable[field] = reason
	}
	o.r.Samples = append(o.r.Samples, s)
	return s, nil
}

func (o *observer) unavailable(field string, err error) {
	o.mu.Lock()
	defer o.mu.Unlock()
	o.r.Unavailable[field] = err.Error()
}

func (o *observer) run(ctx context.Context) error {
	poll := time.NewTicker(observationPeriod)
	defer poll.Stop()
	samples := time.NewTicker(samplePeriod)
	defer samples.Stop()
	for {
		select {
		case <-ctx.Done():
			return nil
		case <-poll.C:
			if err := o.observe(ctx); err != nil {
				if ctx.Err() != nil {
					return nil
				}
				return err
			}
		case <-samples.C:
			if _, err := o.sample(ctx, "periodic"); err != nil {
				if ctx.Err() != nil {
					return nil
				}
				return err
			}
		}
	}
}
