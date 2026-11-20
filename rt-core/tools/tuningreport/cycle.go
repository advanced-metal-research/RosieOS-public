package main

import (
	"math"
	"sort"
)

// CycleClass is the servo host's cycle cost in one state: work_ns (time the
// cycle spent working) and |jitter_ns| (wake-up error) percentiles in us.
type CycleClass struct {
	Name      string  `json:"state"`
	Cycles    int     `json:"cycles"`
	WorkP50   float64 `json:"work_p50_us"`
	WorkP99   float64 `json:"work_p99_us"`
	WorkP999  float64 `json:"work_p999_us"`
	WorkMax   float64 `json:"work_max_us"`
	JitterP50 float64 `json:"jitter_p50_us"`
	JitterP99 float64 `json:"jitter_p99_us"`
	JitterMax float64 `json:"jitter_max_us"`
}

type CycleMetrics struct {
	Classes   []CycleClass `json:"classes"`
	Records   int          `json:"records"`
	Overruns  int          `json:"overruns_work_over_period"`
	WkcShort  int          `json:"wkc_short_cycles"`
	WkcValues []int64      `json:"wkc_values"`
	WkcExpect []int64      `json:"wkc_expected"`
	SeqGaps   int          `json:"sequence_gaps_between_windows"`
	Missing   uint64       `json:"missing_records_in_dumps"`
	Safety    []int64      `json:"safety_fault_masks"`
	Execution []int64      `json:"execution_fault_reasons"`
}

func (a *Analysis) cycle() CycleMetrics {
	cm := CycleMetrics{}
	type acc struct{ work, jit []float64 }
	classes := map[string]*acc{}
	names := []string{"jog lane open/stopping", "Move-to", "armed idle", "disarmed"}
	for _, n := range names {
		classes[n] = &acc{}
	}
	wkc, wke, saf, exe := map[int64]bool{}, map[int64]bool{}, map[int64]bool{}, map[int64]bool{}
	seen := map[string]bool{}
	for _, w := range a.wins {
		s := w.s
		for _, src := range s.Sources {
			if !seen[src.File] {
				seen[src.File] = true
				cm.Missing += src.Missing
			}
		}
		for k := 0; k < w.n; k++ {
			cm.Records++
			name := "disarmed"
			switch {
			case w.armed[k] && s.Mode[k] == 3 && (s.JogOpen[k] != 0 || s.JogRamping[k] != 0):
				name = "jog lane open/stopping"
			case w.armed[k] && s.Mode[k] == 2 && a.anyMoving(w, k):
				name = "Move-to"
			case w.armed[k]:
				name = "armed idle"
			}
			c := classes[name]
			c.work = append(c.work, float64(s.WorkNs[k])/1e3)
			c.jit = append(c.jit, math.Abs(float64(s.JitterNs[k]))/1e3)
			if uint64(s.WorkNs[k]) > s.PeriodNs {
				cm.Overruns++
			}
			if s.Wkc[k] < s.WkcExp[k] {
				cm.WkcShort++
			}
			wkc[s.Wkc[k]], wke[s.WkcExp[k]], saf[s.Safety[k]], exe[s.Exec[k]] = true, true, true, true
		}
	}
	cm.SeqGaps = len(a.wins) - 1
	for _, n := range names {
		c := classes[n]
		if len(c.work) == 0 {
			continue
		}
		cm.Classes = append(cm.Classes, CycleClass{Name: n, Cycles: len(c.work),
			WorkP50: percentile(c.work, 50), WorkP99: percentile(c.work, 99), WorkP999: percentile(c.work, 99.9), WorkMax: percentile(c.work, 100),
			JitterP50: percentile(c.jit, 50), JitterP99: percentile(c.jit, 99), JitterMax: percentile(c.jit, 100)})
	}
	keys := func(m map[int64]bool) []int64 {
		var out []int64
		for k := range m {
			out = append(out, k)
		}
		sort.Slice(out, func(i, j int) bool { return out[i] < out[j] })
		return out
	}
	cm.WkcValues, cm.WkcExpect, cm.Safety, cm.Execution = keys(wkc), keys(wke), keys(saf), keys(exe)
	return cm
}
