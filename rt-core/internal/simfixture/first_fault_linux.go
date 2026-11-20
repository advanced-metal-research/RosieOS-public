//go:build linux

package simfixture

import (
	"encoding/json"
	"fmt"
	"os"
	"sort"
	"strings"
	"time"
)

// nativeFirstFault holds only the metrics.json fields that name why a native
// operation ended, so a consumer's generic "context canceled" receipt cannot
// stand in for the detector that fired.
type nativeFirstFault struct {
	TimeNS      uint64 `json:"time_ns"`
	MaxJitterNS int64  `json:"rt_max_abs_jitter_ns"`
	MaxWorkNS   uint64 `json:"rt_max_cycle_work_ns"`
	Overruns    uint64 `json:"rt_overrun_count"`
	// The configured deadline (opt.max_cycle_lateness_ns), not a measurement.
	CycleLatenessLimitNS uint64 `json:"max_cycle_lateness_ns"`
	SafetyFaultMask      uint32 `json:"safety_fault_mask"`
	Recovery             *struct {
		Faults []struct {
			Name     string `json:"name"`
			AxisMask uint32 `json:"axis_mask"`
		} `json:"faults"`
	} `json:"recovery"`
	Axes []struct {
		// Set by every Home exit, including precondition refusals that never
		// reach the worker's correlated native_home_failure.
		HomeState   uint32 `json:"native_home_state"`
		HomeAbort   uint32 `json:"native_home_last_abort_code"`
		HomeFailure *struct {
			Axis               uint32 `json:"axis"`
			Step               uint32 `json:"step"`
			Operation          string `json:"operation"`
			Reason             string `json:"reason"`
			CancellationReason string `json:"cancellation_reason"`
			AbortCode          uint32 `json:"abort_code"`
		} `json:"native_home_failure"`
	} `json:"axes"`
}

// firstFaultPublications is how many metrics publications must follow the
// first read. The second one began its snapshot after the first read, so it
// holds every failure the daemon published before the caller asked.
const firstFaultPublications = 2

// FirstFault returns one line naming the daemon's retained evidence from the
// metrics.json beside its IPC socket: each distinct Home failure with its axes
// and abort code, each axis's last Home abort, the latched faults, and the
// measured cycle timing counters, printed apart from the configured lateness
// limit they are judged against. It waits for fresh publications (the signal) up to
// budget and says "stale" when they do not arrive, so a stopped daemon is
// reported rather than hidden. It is a diagnostic only: it never fails the
// test and never changes the run.
func FirstFault(metricsPath string, budget time.Duration) string {
	read := func() (nativeFirstFault, error) {
		var m nativeFirstFault
		raw, err := os.ReadFile(metricsPath)
		if err == nil {
			err = json.Unmarshal(raw, &m)
		}
		return m, err
	}
	m, err := read()
	if err != nil {
		return fmt.Sprintf("rt-core first fault: metrics unavailable: %v", err)
	}
	// Depends on the native publisher period, 100 ms
	// (kDiagnosticsPublisherPeriodNs, rt-core/engine/diagnostics.hpp): a
	// quarter period, so no publication is missed. metrics.json does not
	// carry the period, so deriving it needs a second copy of that number.
	const cadence = 25 * time.Millisecond
	deadline := time.Now().Add(budget)
	fresh := 0
	for fresh < firstFaultPublications && time.Now().Before(deadline) {
		time.Sleep(cadence)
		if next, e := read(); e == nil && next.TimeNS > m.TimeNS {
			m, fresh = next, fresh+1
		}
	}
	return formatFirstFault(m, fresh >= firstFaultPublications)
}

func formatFirstFault(m nativeFirstFault, fresh bool) string {
	// Group identical failures: a deadline cancels every axis with one cause.
	type key struct {
		operation, reason, cancellation string
		step, abort                     uint32
	}
	axes := map[key][]string{}
	for _, a := range m.Axes {
		if f := a.HomeFailure; f != nil {
			k := key{f.Operation, f.Reason, f.CancellationReason, f.Step, f.AbortCode}
			axes[k] = append(axes[k], fmt.Sprint(f.Axis))
		}
	}
	type abortKey struct{ state, abort uint32 }
	aborts := map[abortKey][]string{}
	for i, a := range m.Axes {
		if a.HomeAbort != 0 {
			k := abortKey{a.HomeState, a.HomeAbort}
			aborts[k] = append(aborts[k], fmt.Sprint(i))
		}
	}
	exits := []string{}
	for k, list := range aborts {
		exits = append(exits, fmt.Sprintf("{axes=%s state=%d abort=0x%08x}", strings.Join(list, ","), k.state, k.abort))
	}
	sort.Strings(exits)
	homes := []string{}
	for k, list := range axes {
		homes = append(homes, fmt.Sprintf("{axes=%s step=%d op=%s reason=%s cancellation=%s abort=0x%08x}",
			strings.Join(list, ","), k.step, k.operation, k.reason, k.cancellation, k.abort))
	}
	sort.Strings(homes)
	faults := []string{}
	if m.Recovery != nil {
		for _, f := range m.Recovery.Faults {
			faults = append(faults, fmt.Sprintf("%s(axes=0x%x)", f.Name, f.AxisMask))
		}
	}
	freshness := "fresh"
	if !fresh {
		freshness = "stale"
	}
	return fmt.Sprintf("rt-core first fault: native_home_failure=[%s] native_home_abort=[%s] faults=[%s] safety_fault_mask=0x%x "+
		"measured: rt_max_abs_jitter_ns=%d rt_overrun_count=%d rt_max_cycle_work_ns=%d; "+
		"configured: cycle_lateness_limit_ns=%d; metrics_time_ns=%d (%s)",
		strings.Join(homes, " "), strings.Join(exits, " "), strings.Join(faults, " "), m.SafetyFaultMask,
		m.MaxJitterNS, m.Overruns, m.MaxWorkNS, m.CycleLatenessLimitNS, m.TimeNS, freshness)
}
