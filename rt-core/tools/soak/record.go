//go:build linux

package main

import (
	"encoding/json"
	"fmt"
	"rosieos/rt-core/internal/procfs"
	"sort"
	"time"

	"rosieos/rt-core/ipcclient"
	control "rosieos/rt-core/sdk/control"
)

// Unverified workload policy: two complete warm-up iterations precede the run.
const warmupIterations = 2

// Unverified soak RSS allowance: one Linux page per iteration after iteration 2.
const rssPagesPerIteration uint64 = 1

// Unverified load policy: loadavg's 1-minute runnable/uninterruptible average
// exceeding one task per available CPU marks the entire record host_loaded.
const loadPerCPU = 1.0

type configuration struct {
	options
	GitSHA               string            `json:"git_sha"`
	MachineSHA           string            `json:"machine_sha256"`
	Machine              json.RawMessage   `json:"machine"`
	DaemonArgs           []string          `json:"daemon_argv"`
	AdapterArgs          []string          `json:"adapter_argv"`
	DaemonPID            int               `json:"daemon_pid"`
	AdapterPID           int               `json:"adapter_pid"`
	RTTID                int               `json:"rt_tid"`
	CPUCount             int               `json:"cpu_count"`
	PageBytes            uint64            `json:"page_bytes"`
	RSSPagesPerIteration uint64            `json:"rss_growth_pages_per_iteration"`
	WarmupIterations     int               `json:"warmup_iterations"`
	Points               int               `json:"points_per_plan"`
	PlanDurationNS       uint64            `json:"plan_duration_ns"`
	JogUpdates           int               `json:"jog_updates_per_iteration"`
	JogCadenceNS         uint64            `json:"jog_cadence_ns"`
	JogVelocity          float64           `json:"jog_velocity_rad_s"`
	SamplePeriodNS       uint64            `json:"sample_period_ns"`
	ObservationPeriodNS  uint64            `json:"observation_period_ns"`
	FaultOffsetCounts    int               `json:"simulator_feedback_offset_counts"`
	ArtifactDirectory    string            `json:"artifact_directory"`
	Kernel               string            `json:"kernel"`
	CPUModel             string            `json:"cpu_model"`
	JogAgeBoundNS        uint64            `json:"published_jog_input_age_bound_ns"`
	CycleLatenessBoundNS uint64            `json:"cycle_lateness_bound_ns"`
	BinarySHA256         map[string]string `json:"binary_sha256"`
}

type memory = procfs.Memory

type faults = procfs.Faults

type nativeMetrics = ipcclient.Metrics

type percentiles struct {
	Count int    `json:"count"`
	P50NS uint64 `json:"p50_ns"`
	P99NS uint64 `json:"p99_ns"`
	MaxNS uint64 `json:"max_ns"`
}

type sample struct {
	HostNS           uint64                  `json:"host_ns"`
	Phase            string                  `json:"phase"`
	Iteration        int                     `json:"iteration"`
	Kind             string                  `json:"kind"`
	Daemon           memory                  `json:"daemon"`
	Adapter          memory                  `json:"adapter"`
	RT               faults                  `json:"rt_thread"`
	Metrics          nativeMetrics           `json:"native_metrics"`
	Load1            float64                 `json:"loadavg_1"`
	HostLoaded       bool                    `json:"host_loaded"`
	PreparedHandles  int                     `json:"prepared_handles"`
	NativeSlots      *uint64                 `json:"native_slots_in_use"`
	PublicationGapNS uint64                  `json:"status_publication_gap_max_ns"`
	Jog              percentiles             `json:"jog_input_age"`
	DaemonFaults     *faults                 `json:"daemon_faults"`
	IO               *ioCounters             `json:"daemon_io"`
	BufferHealth     *ipcclient.BufferHealth `json:"buffer_health"`
	PreparationSlots *uint32                 `json:"adapter_open_preparation_slots"`
	EventRing        *eventRingSample        `json:"event_ring"`
	Unavailable      map[string]string       `json:"unavailable,omitempty"`
}

type publication struct {
	HostNS        uint64 `json:"observed_host_ns"`
	TimeNS        uint64 `json:"time_ns"`
	GapNS         uint64 `json:"gap_ns"`
	ObserverGapNS uint64 `json:"observer_gap_ns"`
}

type jogSample struct {
	HostNS       uint64                   `json:"observed_host_ns"`
	Phase        string                   `json:"phase"`
	Iteration    int                      `json:"iteration"`
	DuringUpload bool                     `json:"during_upload"`
	AgeNS        uint64                   `json:"input_age_ns"`
	Observation  ipcclient.JogObservation `json:"jog_observation"`
}

type event struct {
	HostNS    uint64                `json:"host_ns"`
	Phase     string                `json:"phase"`
	Iteration int                   `json:"iteration"`
	Motion    ipcclient.MotionState `json:"motion"`
	Execution control.Execution     `json:"execution"`
}

type iteration struct {
	Phase                    string `json:"phase"`
	Number                   int    `json:"iteration"`
	RequestID                string `json:"request_id"`
	UploadStartedNS          uint64 `json:"upload_started_ns"`
	UploadCompletedNS        uint64 `json:"upload_completed_ns"`
	Handle                   uint64 `json:"handle"`
	StartSequence            uint64 `json:"start_sequence"`
	Cancelled                bool   `json:"cancelled"`
	Discarded                bool   `json:"discarded"`
	DiscardedHandle          uint64 `json:"discarded_handle"`
	DiscardedRequestID       string `json:"discarded_request_id"`
	DiscardUploadStartedNS   uint64 `json:"discard_upload_started_ns"`
	DiscardUploadCompletedNS uint64 `json:"discard_upload_completed_ns"`
	FaultReasons             uint32 `json:"injected_fault_reasons"`
	FaultEnableNS            uint64 `json:"fault_enable_host_ns"`
	FaultObservedNS          uint64 `json:"fault_observed_host_ns"`
	FaultPublicationNS       uint64 `json:"fault_publication_time_ns"`
	ResetSequence            uint64 `json:"reset_sequence"`
	JogSent                  int    `json:"jog_updates_sent"`
	JogCadenceMaxNS          uint64 `json:"jog_send_gap_max_ns"`
	SlotCapacityProbe        int    `json:"simultaneous_small_preparations_accepted"`
	Completed                bool   `json:"execution_completed"`
	ExecutedDiscarded        bool   `json:"executed_handle_discarded"`
	OverlapHandle            uint64 `json:"overlap_handle"`
	OverlapStartedNS         uint64 `json:"overlap_upload_started_ns"`
	OverlapCompletedNS       uint64 `json:"overlap_upload_completed_ns"`
	OverlapDiscarded         bool   `json:"overlap_discarded"`
	End                      sample `json:"end"`
}

type assertion struct {
	Name   string `json:"name"`
	Result string `json:"result"`
	Detail string `json:"detail"`
}

type summary struct {
	DaemonPeakRSS       uint64            `json:"daemon_peak_rss_bytes"`
	DaemonPeakHWM       uint64            `json:"daemon_peak_hwm_bytes"`
	AdapterPeakRSS      uint64            `json:"adapter_peak_rss_bytes"`
	AdapterPeakHWM      uint64            `json:"adapter_peak_hwm_bytes"`
	AdapterPeakHeap     *uint64           `json:"adapter_peak_heap_inuse_bytes"`
	DaemonRSSGrowth     int64             `json:"daemon_rss_growth_after_iteration_2_bytes"`
	RSSAllowance        uint64            `json:"daemon_rss_growth_allowance_bytes"`
	MaxJitterNS         uint64            `json:"rt_max_abs_jitter_ns"`
	MaxWorkNS           uint64            `json:"rt_max_cycle_work_ns"`
	Overruns            uint64            `json:"rt_overruns_after_warmup"`
	MinorFaults         uint64            `json:"rt_minor_faults_after_warmup"`
	MajorFaults         uint64            `json:"rt_major_faults_after_warmup"`
	DiagnosticsDropped  uint64            `json:"diagnostics_dropped_after_warmup"`
	PublicationGapNS    uint64            `json:"status_publication_gap_max_ns"`
	Jog                 percentiles       `json:"jog_input_age"`
	PreparedHandles     int               `json:"prepared_handles"`
	NativeSlots         *uint64           `json:"native_slots_in_use"`
	AdapterHeapStart    *uint64           `json:"adapter_heap_start_bytes"`
	AdapterHeapEnd      *uint64           `json:"adapter_heap_end_bytes"`
	AdapterHeapMinima   []heapMinimum     `json:"adapter_heap_after_iteration_minima"`
	Unavailable         map[string]string `json:"unavailable,omitempty"`
	DaemonRSSStart      uint64            `json:"daemon_rss_start_bytes"`
	DaemonRSSEnd        uint64            `json:"daemon_rss_end_bytes"`
	DaemonFaults        *faults           `json:"daemon_faults_after_warmup"`
	IO                  *ioCounters       `json:"daemon_io_after_warmup"`
	EventCount          int               `json:"events_observed"`
	EventRingPeak       uint64            `json:"event_ring_peak_occupancy"`
	NativeSlotsPeak     uint64            `json:"native_slots_peak_in_use"`
	PreparedHandlesPeak int               `json:"prepared_handles_peak"`
}

type record struct {
	Schema              string                    `json:"schema"`
	Configuration       configuration             `json:"configuration"`
	StartedUTC          time.Time                 `json:"started_utc"`
	CompletedUTC        time.Time                 `json:"completed_utc"`
	IterationsCompleted int                       `json:"iterations_completed"`
	HostLoaded          bool                      `json:"host_loaded"`
	Status              string                    `json:"status"`
	Errors              []string                  `json:"errors"`
	Unavailable         map[string]string         `json:"unavailable"`
	Samples             []sample                  `json:"samples"`
	Publications        []publication             `json:"status_publications"`
	Jog                 []jogSample               `json:"jog_observations"`
	Events              []event                   `json:"events"`
	Iterations          []iteration               `json:"iterations"`
	Baseline            *sample                   `json:"after_warmup"`
	Final               *sample                   `json:"final"`
	Summary             summary                   `json:"summary"`
	Assertions          []assertion               `json:"assertions"`
	EventBatches        []eventBatchSample        `json:"event_batches"`
	FailureStatus       *control.Status           `json:"failure_status,omitempty"`
	FailureJog          *ipcclient.JogObservation `json:"failure_jog,omitempty"`
}

// distribution uses nearest-rank percentiles over unique native observations;
// repeated polls of one source sequence cannot bias the input-age distribution.
func distribution(values []uint64) percentiles {
	if len(values) == 0 {
		return percentiles{}
	}
	v := append([]uint64(nil), values...)
	sort.Slice(v, func(i, j int) bool { return v[i] < v[j] })
	return percentiles{len(v), v[(len(v)*50+99)/100-1], v[(len(v)*99+99)/100-1], v[len(v)-1]}
}

// assess recomputes acceptance from observations. Missing measurements and host
// load cannot excuse deadline violations or turn an incomplete run into a pass.
func (r *record) assess() bool {
	r.Assertions = nil
	r.Summary = summary{}
	if r.Baseline == nil || r.Final == nil {
		if r.Unavailable == nil {
			r.Unavailable = map[string]string{}
		}
		r.Unavailable["post_warmup_summary"] = "warm-up baseline or final observation absent; summary deltas and heap endpoints are unavailable, not measured zero"
		for _, field := range []string{"rt_overruns_after_warmup", "rt_minor_faults_after_warmup", "rt_major_faults_after_warmup", "diagnostics_dropped_after_warmup", "daemon_rss_growth_bytes", "daemon_io_delta", "daemon_faults_delta"} {
			r.summaryUnavailable(field, r.Unavailable["post_warmup_summary"])
		}
	}
	add := func(name string, pass bool, detail string) {
		result := "fail"
		if pass {
			result = "pass"
		}
		r.Assertions = append(r.Assertions, assertion{name, result, detail})
	}
	add("iterations_completed", r.IterationsCompleted == r.Configuration.Iterations, fmt.Sprintf("%d/%d", r.IterationsCompleted, r.Configuration.Iterations))
	r.assessMeasurements(add)
	for _, s := range r.Samples {
		r.HostLoaded = r.HostLoaded || s.HostLoaded
		r.Summary.DaemonPeakRSS = max(r.Summary.DaemonPeakRSS, s.Daemon.RSS)
		r.Summary.DaemonPeakHWM = max(r.Summary.DaemonPeakHWM, s.Daemon.HWM)
		r.Summary.AdapterPeakRSS = max(r.Summary.AdapterPeakRSS, s.Adapter.RSS)
		r.Summary.AdapterPeakHWM = max(r.Summary.AdapterPeakHWM, s.Adapter.HWM)
		r.Summary.MaxJitterNS = max(r.Summary.MaxJitterNS, s.Metrics.MaxJitterNS)
		r.Summary.MaxWorkNS = max(r.Summary.MaxWorkNS, s.Metrics.MaxWorkNS)
		if s.Adapter.HeapInuse != nil && s.Phase != "warmup" {
			if r.Summary.AdapterPeakHeap == nil {
				r.Summary.AdapterPeakHeap = new(uint64)
			}
			*r.Summary.AdapterPeakHeap = max(*r.Summary.AdapterPeakHeap, *s.Adapter.HeapInuse)
		}
	}
	var ages []uint64
	for _, j := range r.Jog {
		if j.Phase == "run" {
			ages = append(ages, j.AgeNS)
		}
	}
	r.Summary.Jog = distribution(ages)
	for _, p := range r.Publications {
		r.Summary.PublicationGapNS = max(r.Summary.PublicationGapNS, p.GapNS)
	}
	if r.Baseline == nil || r.Final == nil {
		add("measurement_complete", false, "warm-up baseline or final observation absent")
		return false
	}
	b, f := r.Baseline, r.Final
	r.Summary.AdapterHeapStart, r.Summary.AdapterHeapEnd = b.Adapter.HeapInuse, f.Adapter.HeapInuse
	r.Summary.DaemonRSSStart, r.Summary.DaemonRSSEnd = b.Daemon.RSS, f.Daemon.RSS
	r.Summary.PreparedHandles, r.Summary.NativeSlots = f.PreparedHandles, f.NativeSlots
	r.Summary.PublicationGapNS = f.PublicationGapNS
	add("prepared_handles_zero", f.PreparedHandles == 0, fmt.Sprintf("%d handles", f.PreparedHandles))
	if f.NativeSlots == nil {
		r.Assertions = append(r.Assertions, assertion{"native_slots_zero", "unavailable", r.Unavailable["native_slots_in_use"]})
	} else {
		add("native_slots_zero", *f.NativeSlots == 0, fmt.Sprintf("%d slots", *f.NativeSlots))
	}
	r.assessHeap(add)
	var second *sample
	for i := range r.Iterations {
		it := &r.Iterations[i]
		if it.Phase == "run" && it.Number == 2 {
			second = &it.End
		}
	}
	if second == nil {
		add("daemon_rss_growth", false, "iteration 2 observation absent")
		r.summaryUnavailable("daemon_rss_growth_bytes", "iteration 2 observation absent")
	} else {
		r.Summary.DaemonRSSGrowth = int64(f.Daemon.RSS) - int64(second.Daemon.RSS)
		r.Summary.RSSAllowance = uint64(max(0, r.IterationsCompleted-2)) * r.Configuration.PageBytes * rssPagesPerIteration
		add("daemon_rss_growth", r.Summary.DaemonRSSGrowth <= int64(r.Summary.RSSAllowance), fmt.Sprintf("%d <= %d bytes", r.Summary.DaemonRSSGrowth, r.Summary.RSSAllowance))
	}
	monotonic := f.RT.Minor >= b.RT.Minor && f.RT.Major >= b.RT.Major && f.Metrics.Overruns >= b.Metrics.Overruns && f.Metrics.DiagnosticsDropped >= b.Metrics.DiagnosticsDropped
	add("counter_continuity", monotonic, "same daemon PID and rt-cycle TID throughout")
	if monotonic {
		r.Summary.MinorFaults, r.Summary.MajorFaults = f.RT.Minor-b.RT.Minor, f.RT.Major-b.RT.Major
		r.Summary.Overruns = f.Metrics.Overruns - b.Metrics.Overruns
		r.Summary.DiagnosticsDropped = f.Metrics.DiagnosticsDropped - b.Metrics.DiagnosticsDropped
	} else {
		for _, field := range []string{"rt_overruns_after_warmup", "rt_minor_faults_after_warmup", "rt_major_faults_after_warmup", "diagnostics_dropped_after_warmup"} {
			r.summaryUnavailable(field, "baseline/final counters regressed")
		}
	}
	add("rt_minor_faults_zero", monotonic && r.Summary.MinorFaults == 0, fmt.Sprintf("%d faults after warm-up", r.Summary.MinorFaults))
	add("rt_major_faults_zero", monotonic && r.Summary.MajorFaults == 0, fmt.Sprintf("%d faults after warm-up", r.Summary.MajorFaults))
	add("rt_overruns_unchanged", monotonic && r.Summary.Overruns == 0, fmt.Sprintf("%d overruns after warm-up; host_loaded=%t", r.Summary.Overruns, r.HostLoaded))
	add("cycle_lateness_bounded", r.Configuration.CycleLatenessBoundNS > 0 && r.Summary.MaxJitterNS <= r.Configuration.CycleLatenessBoundNS, fmt.Sprintf("%d <= %d ns (published maximum absolute wake jitter)", r.Summary.MaxJitterNS, r.Configuration.CycleLatenessBoundNS))
	add("jog_input_age_bounded", r.Configuration.JogAgeBoundNS > 0 && r.Summary.Jog.MaxNS <= r.Configuration.JogAgeBoundNS, fmt.Sprintf("%d <= %d ns", r.Summary.Jog.MaxNS, r.Configuration.JogAgeBoundNS))
	add("jog_observations_present", len(ages) > 0, fmt.Sprintf("%d unique observations", len(ages)))
	return r.verdict() == "PASS"
}

// verdict keeps an undeclared bound distinct from a satisfied bound, while any
// failed or missing measurement takes precedence over an unqualified bound.
func (r record) verdict() string {
	if len(r.Errors) != 0 || len(r.Unavailable) != 0 {
		return "FAIL"
	}
	result := "PASS"
	for _, a := range r.Assertions {
		switch a.Result {
		case "pass", "report_only":
		case "unqualified":
			result = "UNQUALIFIED"
		default:
			return "FAIL"
		}
	}
	return result
}

func (r *record) summaryUnavailable(field, reason string) {
	if r.Summary.Unavailable == nil {
		r.Summary.Unavailable = map[string]string{}
	}
	r.Summary.Unavailable[field] = reason
}
