package main

import (
	"math"
	"sort"
	"time"
)

const deg = 180 / math.Pi

// Params are the analysis's detection settings: where a sample counts as
// moving, how a stop or ring episode is found. They select data; they are not
// acceptance bounds. Every acceptance bound (following error, target lead,
// lag allowance, jerk, acceleration, completion tolerance) comes from the
// machine config. Defaults are the values the 2026-10-06/07 analyses of one Rosie
// 1430 arm used, so their published numbers reproduce.
type Params struct {
	LagMinSpeedDeg     float64     `json:"lag_min_speed_deg_s"`       // best-shift samples: |v_cmd| >= this
	MedianMinSpeedDeg  float64     `json:"median_min_speed_deg_s"`    // median error/speed samples
	FitMinAccDeg       float64     `json:"fit_min_acc_deg_s2"`        // fit also keeps slow samples accelerating this hard
	LagCredibleR2      float64     `json:"lag_credible_r2"`           // a lag fit this good sets the joint's residual shift
	MaxShiftMs         int         `json:"max_shift_ms"`              // best-shift search range
	StopMinSpeedDeg    float64     `json:"stop_min_speed_deg_s"`      // a stop follows motion at least this fast
	StopHoldMs         int         `json:"stop_hold_ms"`              // commanded still at least this long after
	StopWindowMs       int         `json:"stop_window_ms"`            // overshoot/settle window
	RingTriggerFactor  float64     `json:"ring_trigger_x_completion"` // episode starts at this x completion_tolerance
	RingQuietMs        int         `json:"ring_quiet_ms"`             // episode ends after this long inside completion_tolerance
	RingMergeMs        int         `json:"ring_merge_ms"`             // episodes closer than this merge
	SpectrumMinHz      float64     `json:"spectrum_min_hz"`
	SpectrumMaxHz      float64     `json:"spectrum_max_hz"`
	PlateauSpeedFrac   float64     `json:"plateau_speed_x_velocity_limit"` // overshoot plateaus at |v| >= this x limit
	PlateauMs          int         `json:"plateau_ms"`
	OvershootWindowMs  int         `json:"overshoot_window_ms"`
	ActualAccSmoothCyc int         `json:"actual_acc_smooth_cycles"` // moving mean of velocity_actual
	ActualAccDiffCyc   int         `json:"actual_acc_diff_cycles"`
	JerkWindowCyc      int         `json:"jerk_window_cycles"` // centred second difference spacing (20 ms window at 10)
	HoldTailMs         int         `json:"hold_tail_ms"`
	Servo              ServoParams `json:"servo"`
}

// ServoParams select the data of the servo-loop quality section (servo.go).
// The buzz thresholds are absolute torque values (per mille of rated, RMS in
// the loop band over one timeline window), taken from one Rosie 1430 arm's
// measurements below; they are detection settings, UNVERIFIED as acceptance
// limits, and printed in every report.
type ServoParams struct {
	BandsHz        [][2]float64 `json:"bands_hz"`             // band energies per state/hold/timeline
	LoopBandHz     [2]float64   `json:"loop_band_hz"`         // the band buzz episodes are detected in
	SegmentCycles  int          `json:"welch_segment_cycles"` // state/hold spectra: Hann, 50 % overlap
	TimelineMs     int          `json:"timeline_step_ms"`
	TimelineCycles int          `json:"timeline_window_cycles"` // Hann window centred on each step
	BuzzStillOn    float64      `json:"buzz_on_still_permille_rms"`
	BuzzMovingOn   float64      `json:"buzz_on_moving_permille_rms"`
	BuzzMinMs      int          `json:"buzz_min_ms"`
	BuzzMergeMs    int          `json:"buzz_merge_ms"`
	LineMinHz      float64      `json:"line_min_hz"`
	LineOverFloor  float64      `json:"line_over_floor_x"` // line peak bin power over local median power
	LineMinAmp     float64      `json:"line_min_amplitude_permille"`
	CoherenceMin   float64      `json:"coupling_coherence_min"`
	StopLeadMs     int          `json:"stop_lead_ms"` // "stop" state: from this long before motion ends to stop_window_ms after
}

func defaultParams() Params {
	// LagCredibleR2 0.8: tracking joints fit at R2 0.896-0.995 on one Rosie 1430 arm's
	// dumps of 2026-10-06/07 (13:06 J5 at 5 deg/s was the lowest); a joint
	// that only rings fits at 0.007-0.02 (J4 at 17:40 and 21:04).
	return Params{LagMinSpeedDeg: 2, MedianMinSpeedDeg: 5, FitMinAccDeg: 5, MaxShiftMs: 60, LagCredibleR2: 0.8,
		StopMinSpeedDeg: 3, StopHoldMs: 300, StopWindowMs: 600,
		RingTriggerFactor: 3, RingQuietMs: 100, RingMergeMs: 300, SpectrumMinHz: 1, SpectrumMaxHz: 30,
		PlateauSpeedFrac: 0.25, PlateauMs: 100, OvershootWindowMs: 300,
		ActualAccSmoothCyc: 20, ActualAccDiffCyc: 10, JerkWindowCyc: 10, HoldTailMs: 800,
		Servo: defaultServoParams()}
}

func defaultServoParams() ServoParams {
	// Bands: 1-30 Hz holds the arm's rings (one Rosie 1430 arm's J4: 5.6-8 Hz) and the torque
	// that accelerates the arm; 30-80 Hz is above every measured mode;
	// 80-300 Hz holds both speed-loop oscillations seen on that arm (J4 228 Hz
	// at C00.06 = 700 %, 2026-10-07 22:01; J2 ~100 Hz with an 80-250 Hz
	// hump, 2026-10-08 11:28); 300-500 Hz runs to Nyquist of the 1 kHz dump.
	//
	// Buzz thresholds, loop band 80-500 Hz, 256-cycle windows every 100 ms:
	// joint commanded still, the floor is 2.3-4.0 permille RMS on every
	// joint (150-500 Hz, 2026-10-07 20:56-22:29 and 2026-10-08 11:27), and
	// J4's 228 Hz line at 700 % reached 10-20; on = 8. Joint moving, quiet
	// holds reach 39 permille (J2 at 31 deg/s, rev 14, 2026-10-08
	// 11:28:16) and 48 (J2 at 42 deg/s, 11:28:10); J2's oscillation read
	// 140 in the first window of rev 17 and 100-1100 after; on = 80.
	// Episodes shorter than 200 ms are dropped; gaps up to 2 s (one stop
	// between holds) merge.
	return ServoParams{BandsHz: [][2]float64{{1, 30}, {30, 80}, {80, 300}, {300, 500}}, LoopBandHz: [2]float64{80, 500},
		SegmentCycles: 512, TimelineMs: 100, TimelineCycles: 256,
		BuzzStillOn: 8, BuzzMovingOn: 80, BuzzMinMs: 200, BuzzMergeMs: 2000,
		LineMinHz: 30, LineOverFloor: 8, LineMinAmp: 1.0, CoherenceMin: 0.5, StopLeadMs: 200}
}

// win is one contiguous window in joint units.
type win struct {
	s                  *Series
	n                  int
	dt                 float64
	qc, qa, vc, va, fd [][]float64
	tq                 [][]float64 // 0x6077, per mille of rated torque
	vaOK, fdOK, tqOK   []bool      // per joint: the drive reported velocity_actual / 0x60F4 / 0x6077
	on                 [][]bool
	armed              []bool
}

type Analysis struct {
	cfg      *CellConfig
	p        Params
	tz       *time.Location
	wins     []*win
	lagShift []int // per joint, cycles, for the lag-compensated residual
}

func newAnalysis(cfg *CellConfig, p Params, tz *time.Location, series []*Series) *Analysis {
	a := &Analysis{cfg: cfg, p: p, tz: tz}
	nj := len(cfg.Joints)
	for _, s := range series {
		w := &win{s: s, n: s.Len(), dt: float64(s.PeriodNs) / 1e9, armed: make([]bool, s.Len())}
		for k := range w.armed {
			w.armed[k] = s.Armed[k] != 0
		}
		alloc := func() [][]float64 {
			m := make([][]float64, nj)
			for j := range m {
				m[j] = make([]float64, w.n)
			}
			return m
		}
		w.qc, w.qa, w.vc, w.va, w.fd, w.tq = alloc(), alloc(), alloc(), alloc(), alloc(), alloc()
		w.on = make([][]bool, nj)
		w.vaOK, w.fdOK, w.tqOK = make([]bool, nj), make([]bool, nj), make([]bool, nj)
		for j := 0; j < nj && j < s.Axes; j++ {
			jc := cfg.Joints[j]
			w.on[j] = make([]bool, w.n)
			for k := 0; k < w.n; k++ {
				w.qc[j][k] = jc.q(s.Target[j][k], s.HomeOff[j][k])
				w.qa[j][k] = jc.q(s.Pos[j][k], s.HomeOff[j][k])
				w.vc[j][k] = jc.v(s.TV[j][k])
				w.va[j][k] = jc.v(s.VA[j][k])
				w.fd[j][k] = float64(s.FE[j][k]) / jc.CountsPerRad
				w.on[j][k] = w.armed[k] && s.Enable[k]&(1<<j) != 0
				w.tq[j][k] = float64(s.Torque[j][k])
				w.vaOK[j] = w.vaOK[j] || s.VAValid[j][k] != 0
				w.fdOK[j] = w.fdOK[j] || s.FEValid[j][k] != 0
				w.tqOK[j] = w.tqOK[j] || s.TorqueValid[j][k] != 0
			}
		}
		a.wins = append(a.wins, w)
	}
	return a
}

func (a *Analysis) wall(w *win, k int) time.Time {
	return time.Unix(0, w.s.TNs[k]+w.s.ClockOffsetNs).In(a.tz)
}

func (a *Analysis) stamp(w *win, k int) string {
	if k < 0 || k >= w.n {
		return ""
	}
	return a.wall(w, k).Format("15:04:05.000")
}

func (a *Analysis) hasRange(w *win, j, from, to int) bool {
	if from < 0 || to >= w.n {
		return false
	}
	for k := from; k <= to; k++ {
		if !w.on[j][k] {
			return false
		}
	}
	return true
}

// JointMetrics is one joint's tracking, limits use and dynamics. Angles in
// degrees, times in ms, jerk in rad/s^3 (the unit jog_jerk is configured in).
type JointMetrics struct {
	Joint string `json:"joint"`
	// Lag: the shift minimising |actual(t) - commanded(t - shift)|, the
	// median of following error / commanded speed, and the least-squares
	// slope of following error on commanded speed (the published "lag fit").
	BestShiftMs   float64 `json:"lag_best_shift_ms"`
	ShiftResidDeg float64 `json:"lag_shift_residual_deg"`
	MedianLagMs   float64 `json:"lag_median_fe_over_v_ms"`
	FitLagMs      float64 `json:"lag_fit_ms"`
	FitR2         float64 `json:"lag_fit_r2"`
	FitSamples    int     `json:"lag_fit_samples"`
	// Following error = commanded[t-1] - actual[t] against the scaled bound
	// following_error + |v_cmd| x tracking_lag_allowance.
	PeakFEDeg       float64 `json:"fe_peak_deg"`
	PeakFEAtVDeg    float64 `json:"fe_peak_at_v_deg_s"`
	FERatioMax      float64 `json:"fe_over_bound_max"`
	FERatioMaxAt    string  `json:"fe_over_bound_max_at"`
	FEOverLongestMs float64 `json:"fe_over_bound_longest_ms"`
	FEOverTotalMs   float64 `json:"fe_over_bound_total_ms"`
	DriveFEPeakDeg  float64 `json:"drive_fe_peak_deg"`
	// Target lead = commanded[t] - actual[t] (native's check) against
	// max_target_lead + |v_cmd| x allowance; excursions merge across gaps
	// of <= 2 cycles, as the native warning does.
	LeadExcursions int     `json:"lead_excursions"`
	LeadPeakDeg    float64 `json:"lead_excursion_peak_deg"`
	LeadLongestMs  float64 `json:"lead_excursion_longest_ms"`
	// Acceleration: commanded from 1-cycle velocity differences, actual from
	// velocity_actual smoothed and differenced (no per-cycle quantisation).
	CmdAccPeakDeg    float64     `json:"acc_cmd_peak_deg_s2"`
	ActAccPeakDeg    float64     `json:"acc_actual_peak_deg_s2"`
	AccRatio         float64     `json:"acc_actual_over_cmd"`
	AccLimitUse      float64     `json:"acc_cmd_over_limit"`
	JerkCycleMax     float64     `json:"jerk_cmd_1cycle_max_rad_s3"`
	Jerk20Max        float64     `json:"jerk_cmd_20ms_max_rad_s3"`
	JerkOverCycles   int         `json:"jerk_cmd_1cycle_over_limit_cycles"`
	Jerk20Over       float64     `json:"jerk_cmd_20ms_over_limit"`
	Plateaus         int         `json:"start_plateaus"`
	VelOvershootPct  float64     `json:"start_velocity_overshoot_pct"`
	VelOvershootDeg  float64     `json:"start_velocity_overshoot_deg_s"`
	VelOvershootAt   string      `json:"start_velocity_overshoot_at,omitempty"`
	Stops            []StopEvent `json:"stops"`
	StopWorstOverDeg float64     `json:"stop_overshoot_worst_deg"`
	StopWorstSettle  float64     `json:"stop_settle_worst_ms"`
	Ring             RingMetrics `json:"ring"`
	MovingSamples    int         `json:"moving_samples"`
}

type StopEvent struct {
	At       string  `json:"at"`
	Kind     string  `json:"kind"`
	VPeakDeg float64 `json:"v_peak_deg_s"`
	ResidDeg float64 `json:"residual_at_stop_deg"`
	OverDeg  float64 `json:"overshoot_deg"`
	SettleMs float64 `json:"settle_ms"`
	Cross    int     `json:"crossings"`
	trace    []float64
}

func nan() float64 { return math.NaN() }

func (a *Analysis) jointMetrics(j int) JointMetrics {
	jc := a.cfg.Joints[j]
	p := a.p
	m := JointMetrics{Joint: jc.Name}
	allow := a.cfg.LagAllowanceS

	// --- lag: best shift
	best, bestSh, bestN := math.Inf(1), -1, 0
	for sh := 0; sh <= p.MaxShiftMs; sh++ {
		s, n := 0.0, 0
		for _, w := range a.wins {
			shc := int(math.Round(float64(sh) * 1e-3 / w.dt))
			for t := shc + 1; t < w.n; t++ {
				if !w.on[j][t] || !w.on[j][t-shc] || math.Abs(w.vc[j][t])*deg < p.LagMinSpeedDeg {
					continue
				}
				d := w.qa[j][t] - w.qc[j][t-shc]
				s += d * d
				n++
			}
		}
		if n > 0 && s/float64(n) < best {
			best, bestSh, bestN = s/float64(n), sh, n
		}
	}
	m.BestShiftMs, m.ShiftResidDeg = nan(), nan()
	if bestN > 0 {
		m.BestShiftMs, m.ShiftResidDeg = float64(bestSh), math.Sqrt(best)*deg
	}
	// --- median FE / v and the v-only fit on 5-cycle commanded differences
	var ratios []float64
	var sv2, sve, se, se2 float64
	nfit := 0
	type sample struct{ v, e float64 }
	var fitS []sample
	for _, w := range a.wins {
		for t := 1; t < w.n; t++ {
			if w.on[j][t] && w.on[j][t-1] && math.Abs(w.vc[j][t])*deg >= p.MedianMinSpeedDeg {
				ratios = append(ratios, (w.qc[j][t-1]-w.qa[j][t])/w.vc[j][t]*1e3)
			}
		}
		for k := 10; k < w.n-10; k++ {
			if !w.on[j][k] || !w.on[j][k-10] || !w.on[j][k+9] {
				continue
			}
			v := (w.qc[j][k+2] - w.qc[j][k-3]) / (5 * w.dt)
			vp := (w.qc[j][k+7] - w.qc[j][k+2]) / (5 * w.dt)
			vm := (w.qc[j][k-3] - w.qc[j][k-8]) / (5 * w.dt)
			acc := (vp - vm) / (10 * w.dt)
			if math.Abs(v)*deg < p.LagMinSpeedDeg && math.Abs(acc)*deg < p.FitMinAccDeg {
				continue
			}
			e := w.qc[j][k-1] - w.qa[j][k]
			fitS = append(fitS, sample{v, e})
			sv2 += v * v
			sve += v * e
			se += e
			se2 += e * e
			nfit++
		}
	}
	m.MedianLagMs = median(ratios)
	m.FitLagMs, m.FitR2 = nan(), nan()
	m.FitSamples = nfit
	if nfit >= 200 && sv2 > 0 {
		sl := sve / sv2
		mean := se / float64(nfit)
		var rss, tss float64
		for _, s := range fitS {
			r := s.e - sl*s.v
			rss += r * r
			tss += (s.e - mean) * (s.e - mean)
		}
		m.FitLagMs, m.FitR2 = sl*1e3, 1-rss/tss
	}

	// --- following error vs scaled bound, drive FE, target lead
	m.DriveFEPeakDeg = nan()
	runMax, total := 0, 0
	for _, w := range a.wins {
		run := 0
		inLead, gap, leadRun := false, 0, 0
		for k := 1; k < w.n; k++ {
			if !w.on[j][k] || !w.on[j][k-1] {
				run, inLead = 0, false
				continue
			}
			m.MovingSamples++
			vc := math.Abs(w.vc[j][k])
			fe := math.Abs(w.qc[j][k-1] - w.qa[j][k])
			if fe*deg > m.PeakFEDeg {
				m.PeakFEDeg, m.PeakFEAtVDeg = fe*deg, vc*deg
			}
			r := fe / (jc.FollowingError + vc*allow)
			if r > m.FERatioMax {
				m.FERatioMax, m.FERatioMaxAt = r, a.stamp(w, k)
			}
			if r > 1 {
				run++
				total++
				if run > runMax {
					runMax = run
				}
			} else {
				run = 0
			}
			if w.fdOK[j] {
				m.DriveFEPeakDeg = nanMax(m.DriveFEPeakDeg, math.Abs(w.fd[j][k])*deg)
			}
			lead := math.Abs(w.qc[j][k] - w.qa[j][k])
			if lead > jc.MaxTargetLead+vc*allow {
				if !inLead || gap > 2 {
					m.LeadExcursions++
					leadRun = 0
				}
				inLead, gap = true, 0
				leadRun++
				m.LeadPeakDeg = math.Max(m.LeadPeakDeg, lead*deg)
				m.LeadLongestMs = math.Max(m.LeadLongestMs, float64(leadRun)*w.dt*1e3)
			} else if inLead {
				gap++
				if gap <= 2 {
					leadRun++
				}
			}
		}
		_ = w
	}
	dt0 := 1e-3
	if len(a.wins) > 0 {
		dt0 = a.wins[0].dt
	}
	m.FEOverLongestMs, m.FEOverTotalMs = float64(runMax)*dt0*1e3, float64(total)*dt0*1e3

	// --- acceleration, jerk, velocity overshoot at starts
	m.ActAccPeakDeg, m.AccRatio = nan(), nan()
	W, D, J := p.ActualAccSmoothCyc, p.ActualAccDiffCyc, p.JerkWindowCyc
	jerkLim := a.cfg.JogJerk
	for _, w := range a.wins {
		for k := 1; k < w.n; k++ {
			if w.on[j][k] && w.on[j][k-1] {
				m.CmdAccPeakDeg = math.Max(m.CmdAccPeakDeg, math.Abs(w.vc[j][k]-w.vc[j][k-1])/w.dt*deg)
			}
			if a.hasRange(w, j, k-1, k+1) {
				jk := math.Abs(w.vc[j][k+1]-2*w.vc[j][k]+w.vc[j][k-1]) / (w.dt * w.dt)
				m.JerkCycleMax = math.Max(m.JerkCycleMax, jk)
				if jerkLim > 0 && jk > jerkLim*1.0001 {
					m.JerkOverCycles++
				}
			}
			if a.hasRange(w, j, k-J, k+J) {
				h := float64(J) * w.dt
				m.Jerk20Max = math.Max(m.Jerk20Max, math.Abs(w.vc[j][k+J]-2*w.vc[j][k]+w.vc[j][k-J])/(h*h))
			}
		}
		if !w.vaOK[j] {
			continue
		}
		sm := smooth(w.va[j], W)
		half := D / 2
		for k := W + D; k < w.n-W-D; k++ {
			if !a.hasRange(w, j, k-W-half, k+W+half) {
				continue
			}
			acc := math.Abs(sm[k+half]-sm[k-half]) / (float64(D) * w.dt) * deg
			m.ActAccPeakDeg = nanMax(m.ActAccPeakDeg, acc)
		}
		// plateaus: commanded speed constant for PlateauMs at >= a fraction
		// of the joint's velocity limit; overshoot = smoothed actual beyond
		// commanded, in the direction of motion, within OvershootWindowMs.
		thr := p.PlateauSpeedFrac * jc.VelocityLimit
		tol := 0.05 / deg
		pl, ow := int(float64(p.PlateauMs)*1e-3/w.dt), int(float64(p.OvershootWindowMs)*1e-3/w.dt)
		for k := W; k < w.n-ow-W; k++ {
			if !w.on[j][k] || math.Abs(w.vc[j][k]) < thr {
				continue
			}
			flat := true
			for q := k; q < k+pl; q++ {
				if !w.on[j][q] || math.Abs(w.vc[j][q]-w.vc[j][k]) >= tol {
					flat = false
					break
				}
			}
			if !flat {
				continue
			}
			s := math.Copysign(1, w.vc[j][k])
			o := math.Inf(-1)
			for q := k; q < k+ow; q++ {
				if w.on[j][q] && math.Abs(w.vc[j][q]-w.vc[j][k]) < tol {
					o = math.Max(o, (sm[q]-w.vc[j][q])*s)
				}
			}
			m.Plateaus++
			pct := o / math.Abs(w.vc[j][k]) * 100
			if m.Plateaus == 1 || pct > m.VelOvershootPct {
				m.VelOvershootPct, m.VelOvershootDeg, m.VelOvershootAt = pct, o*deg, a.stamp(w, k)
			}
			for k < w.n-1 && math.Abs(w.vc[j][k+1]-w.vc[j][k]) < tol {
				k++
			}
		}
	}
	if m.Plateaus == 0 {
		m.VelOvershootPct, m.VelOvershootDeg = nan(), nan()
	}
	// The ratio says how far the drive overshoots a commanded acceleration;
	// on a joint commanded below a tenth of its limit (J4 held still while
	// it rings) it would divide ringing by nothing and read in thousands.
	if !math.IsNaN(m.ActAccPeakDeg) && m.CmdAccPeakDeg/deg >= 0.1*jc.AccelerationLim {
		m.AccRatio = m.ActAccPeakDeg / m.CmdAccPeakDeg
	}
	if jc.AccelerationLim > 0 {
		m.AccLimitUse = m.CmdAccPeakDeg / deg / jc.AccelerationLim
	}
	if jerkLim > 0 {
		m.Jerk20Over = m.Jerk20Max / jerkLim
	}
	m.Stops = a.stops(j)
	m.StopWorstOverDeg, m.StopWorstSettle = nan(), nan()
	for _, s := range m.Stops {
		m.StopWorstOverDeg = nanMax(m.StopWorstOverDeg, s.OverDeg)
		m.StopWorstSettle = nanMax(m.StopWorstSettle, s.SettleMs)
	}
	return m
}

// stops finds commanded stops: v_cmd reaches 0 and stays 0 for StopHoldMs
// after at least StopMinSpeedDeg in the preceding StopHoldMs. e = (actual -
// final target) x direction: negative still lagging, positive past target.
// Settle is the last cycle outside completion_tolerance.
func (a *Analysis) stops(j int) []StopEvent {
	p := a.p
	tol := a.cfg.Joints[j].CompletionTol
	var out []StopEvent
	for _, w := range a.wins {
		hold := int(float64(p.StopHoldMs) * 1e-3 / w.dt)
		win := int(float64(p.StopWindowMs) * 1e-3 / w.dt)
		for k := hold + 1; k < w.n-1; k++ {
			if !(w.s.TV[j][k] == 0 && w.s.TV[j][k-1] != 0 && w.on[j][k]) {
				continue
			}
			ok := k+hold <= w.n
			for q := k; ok && q < k+hold; q++ {
				ok = w.s.TV[j][q] == 0
			}
			if !ok {
				continue
			}
			vpk, dir := 0.0, 0.0
			for q := k - hold; q < k; q++ {
				v := w.vc[j][q] * deg
				vpk = math.Max(vpk, math.Abs(v))
				if math.Abs(v) > 0.5 {
					dir = math.Copysign(1, v)
				}
			}
			if vpk < p.StopMinSpeedDeg || dir == 0 {
				continue
			}
			tf := w.qc[j][k]
			ev := StopEvent{At: a.stamp(w, k), VPeakDeg: vpk, OverDeg: math.Inf(-1), Kind: "jog"}
			if w.s.Mode[k] == 2 || w.s.Mode[k-1] == 2 {
				ev.Kind = "trajectory"
			}
			sign := 0.0
			for q := k; q < k+win && q < w.n; q++ {
				if !w.on[j][q] || w.s.TV[j][q] != 0 {
					break
				}
				e := (w.qa[j][q] - tf) * dir
				ev.trace = append(ev.trace, e*deg)
				if q == k {
					ev.ResidDeg = e * deg
				}
				ev.OverDeg = math.Max(ev.OverDeg, e*deg)
				if math.Abs(e) > tol {
					ev.SettleMs = float64(q-k) * w.dt * 1e3
					s := math.Copysign(1, e)
					if sign != 0 && s != sign {
						ev.Cross++
					}
					sign = s
				}
			}
			out = append(out, ev)
		}
	}
	return out
}

func smooth(x []float64, W int) []float64 {
	out := make([]float64, len(x))
	c := make([]float64, len(x)+1)
	for i, v := range x {
		c[i+1] = c[i] + v
	}
	h := W / 2
	for k := range x {
		if k-h < 0 || k-h+W > len(x) {
			out[k] = math.NaN()
			continue
		}
		out[k] = (c[k-h+W] - c[k-h]) / float64(W)
	}
	return out
}

func median(x []float64) float64 {
	if len(x) == 0 {
		return math.NaN()
	}
	y := append([]float64(nil), x...)
	sort.Float64s(y)
	return y[len(y)/2]
}

func percentile(x []float64, p float64) float64 {
	if len(x) == 0 {
		return math.NaN()
	}
	y := append([]float64(nil), x...)
	sort.Float64s(y)
	pos := p / 100 * float64(len(y)-1)
	lo := int(math.Floor(pos))
	hi := int(math.Ceil(pos))
	return y[lo] + (y[hi]-y[lo])*(pos-float64(lo))
}

func nanMax(a, b float64) float64 {
	if math.IsNaN(a) {
		return b
	}
	if math.IsNaN(b) {
		return a
	}
	return math.Max(a, b)
}
