package main

import (
	"math"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"time"

	"rosieos/rt-core/cartesian"
)

// OLPHold is one `cartesian path hold: begin` line joined with its `end`.
type OLPHold struct {
	Rev    int
	Begin  time.Time // naive journal time (see JournalLine.Local)
	Frame  string
	Axis   int // 0..5 = X Y Z Rx Ry Rz
	Sign   float64
	Speed  float64 // box x fraction, mm/s or deg/s
	Unit   string
	StartQ []float64
	End    map[string]string
	HasEnd bool
}

var axisNames = []string{"X", "Y", "Z", "Rx", "Ry", "Rz"}

var holdLine = regexp.MustCompile(`cartesian path hold: (begin|end) rev=(\d+)(.*)$`)

func parseOLPHolds(lines []JournalLine) []OLPHold {
	byRev := map[int]*OLPHold{}
	var order []int
	for _, l := range lines {
		if l.Source != "olp" {
			continue
		}
		m := holdLine.FindStringSubmatch(l.Text)
		if m == nil {
			continue
		}
		rev, _ := strconv.Atoi(m[2])
		f := kv(m[3])
		if m[1] == "begin" {
			h := &OLPHold{Rev: rev, Begin: l.Local, Frame: f["frame"], Unit: "mm/s"}
			ax := f["axis"]
			if len(ax) > 1 {
				h.Sign = 1
				if ax[0] == '-' {
					h.Sign = -1
				}
				h.Axis = -1
				for i, n := range axisNames {
					if ax[1:] == n {
						h.Axis = i
					}
				}
			}
			box := kvFloat(f, "box")
			// "box=100.0 mm/s": the unit is the token after box.
			if i := strings.Index(m[3], "box="); i >= 0 {
				rest := strings.Fields(m[3][i:])
				if len(rest) > 1 {
					h.Unit = rest[1]
				}
			}
			h.Speed = box * kvFloat(f, "fraction")
			for _, v := range strings.Split(strings.Trim(f["start_q_rad"], "[]"), ",") {
				x, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
				if err == nil {
					h.StartQ = append(h.StartQ, x)
				}
			}
			if h.Axis >= 0 {
				if _, ok := byRev[rev]; !ok {
					order = append(order, rev)
				}
				byRev[rev] = h
			}
		} else if h, ok := byRev[rev]; ok {
			h.End, h.HasEnd = f, true
		}
	}
	var out []OLPHold
	for _, r := range order {
		out = append(out, *byRev[r])
	}
	return out
}

// HoldMetrics is one Cartesian path hold measured on the commanded joint
// targets (what native sent the drives) and the actual feedback, through the
// resolver's kinematics. Linear holds: off = TCP distance from the commanded
// line (mm), orient = rotation from the start orientation (deg). Rotational
// holds: off = TCP drift from its start (mm), orient = rotation not about the
// commanded axis (deg); travel in deg. Stop: from the cycle native left the
// lane open (End received or input expired) to the last nonzero commanded
// joint velocity.
type HoldMetrics struct {
	Source        string    `json:"source"` // olp | inferred
	Rev           int       `json:"rev"`
	At            string    `json:"at"`
	Frame         string    `json:"frame"`
	Axis          string    `json:"axis"`
	Linear        bool      `json:"linear"`
	SpeedReq      float64   `json:"speed_requested"`
	Unit          string    `json:"unit"`
	Reason        string    `json:"end_reason,omitempty"`
	Travel        float64   `json:"travel"`
	VMax          float64   `json:"speed_peak"`
	CmdOffMax     float64   `json:"cmd_off_max_mm"`
	CmdOffRMS     float64   `json:"cmd_off_rms_mm"`
	CmdOriMax     float64   `json:"cmd_orient_max_deg"`
	ActOffSteady  float64   `json:"act_off_steady_max_mm"`
	ActOffMax     float64   `json:"act_off_max_mm"`
	ActOriMax     float64   `json:"act_orient_max_deg"`
	VStop         float64   `json:"stop_speed"`
	StopDist      float64   `json:"stop_distance"`
	StopMs        float64   `json:"stop_ms"`
	Tripped       bool      `json:"tripped"`
	StopSpreadMs  float64   `json:"joint_stop_spread_ms"`
	AccMax        float64   `json:"acc_max_rad_s2"`
	AccJoint      string    `json:"acc_max_joint"`
	JerkMax       float64   `json:"jerk_1cycle_max_rad_s3"`
	Jerk20Max     float64   `json:"jerk_20ms_max_rad_s3"`
	Jerk20Joint   string    `json:"jerk_20ms_max_joint"`
	FERatioMax    []float64 `json:"fe_over_bound_max"`
	FEOverMs      []float64 `json:"fe_over_bound_ms"`
	StartQCheck   float64   `json:"start_q_vs_olp_rad,omitempty"`
	Governing     string    `json:"governing_ms,omitempty"`
	Underruns     string    `json:"underruns,omitempty"`
	ResolverP99   string    `json:"resolver_p99_ms,omitempty"`
	OLPMonitorOff string    `json:"olp_monitor_offpath_mm,omitempty"`
	actOffTrace   []float64
	window        int
	start         int
	end           int // first cycle after the commanded motion (exclusive)
}

func (a *Analysis) holds(olp []OLPHold, olpShiftH float64) []HoldMetrics {
	var out []HoldMetrics
	for wi, w := range a.wins {
		used := map[int64]bool{}
		for _, h := range olp {
			bt := h.Begin.UnixNano() - int64(olpShiftH*3600e9) - w.s.ClockOffsetNs
			if bt < w.s.TNs[0] || bt > w.s.TNs[w.n-1] {
				continue
			}
			// The begin line is logged after native opened the lane: take
			// the lane open nearest the line within -500..+200 ms.
			bestK, bestD := -1, int64(math.MaxInt64)
			for k := 1; k < w.n; k++ {
				if w.s.JogOpen[k] == 1 && w.s.JogOpen[k-1] == 0 {
					d := bt - w.s.TNs[k]
					if d > -200e6 && d < 500e6 && absI(d) < bestD {
						bestK, bestD = k, absI(d)
					}
				}
			}
			if bestK < 0 {
				continue
			}
			gen := w.s.JogGen[bestK]
			used[gen] = true
			hm, ok := a.hold(w, gen, h.Frame, h.Axis, h.Sign, h.Axis < 3)
			if !ok {
				continue
			}
			hm.Source, hm.Rev, hm.SpeedReq, hm.Unit = "olp", h.Rev, h.Speed, h.Unit
			hm.window = wi
			if h.HasEnd {
				hm.Reason, hm.Governing, hm.Underruns = h.End["reason"], strings.Trim(h.End["governing_ms"], "{}"), h.End["underruns"]
				hm.ResolverP99, hm.OLPMonitorOff = h.End["resolver_p99_ms"], h.End["monitor_max_offpath_mm"]
			}
			if len(h.StartQ) == len(a.cfg.Joints) {
				s := a.firstOpen(w, gen)
				for j := range h.StartQ {
					hm.StartQCheck = math.Max(hm.StartQCheck, math.Abs(w.qc[j][s-1]-h.StartQ[j]))
				}
			}
			out = append(out, hm)
		}
		// Lanes without an OLP line: a Cartesian hold moves several joints;
		// a joint jog (one joint) is left to the per-joint metrics.
		for k := 1; k < w.n; k++ {
			if !(w.s.JogOpen[k] == 1 && w.s.JogOpen[k-1] == 0 && w.s.Mode[k] == 3) || used[w.s.JogGen[k]] {
				continue
			}
			gen := w.s.JogGen[k]
			used[gen] = true
			hm, ok := a.hold(w, gen, "inferred", -1, 1, true)
			if ok {
				hm.Source, hm.window = "inferred", wi
				out = append(out, hm)
			}
		}
	}
	sort.SliceStable(out, func(i, j int) bool {
		if out[i].window != out[j].window {
			return out[i].window < out[j].window
		}
		return out[i].start < out[j].start
	})
	return out
}

func (a *Analysis) firstOpen(w *win, gen int64) int {
	for k := 1; k < w.n; k++ {
		if w.s.JogGen[k] == gen && w.s.JogOpen[k] == 1 {
			return k
		}
	}
	return -1
}

func (a *Analysis) anyMoving(w *win, k int) bool {
	for j := range a.cfg.Joints {
		if w.s.TV[j][k] != 0 {
			return true
		}
	}
	return false
}

// hold measures one jog generation. axis < 0 infers the axis from the
// commanded motion: the start-to-stop chord (linear) or rotation vector
// (rotational), so an inferred "off-axis" is off the chord, a lower bound.
func (a *Analysis) hold(w *win, gen int64, frame string, axis int, sign float64, linear bool) (HoldMetrics, bool) {
	nj := len(a.cfg.Joints)
	s := a.firstOpen(w, gen)
	if s < 1 {
		return HoldMetrics{}, false
	}
	ks := s
	for ks < w.n && w.s.JogGen[ks] == gen && w.s.JogOpen[ks] == 1 {
		ks++
	}
	e := ks
	for e < w.n && a.anyMoving(w, e) && w.armed[e] {
		e++
	}
	if e >= w.n {
		return HoldMetrics{}, false
	}
	tripped := !w.armed[e]
	if tripped {
		e--
	}
	tail := e + 1
	for tail < w.n && tail < e+int(float64(a.p.HoldTailMs)*1e-3/w.dt) && !a.anyMoving(w, tail) && w.armed[tail] {
		tail++
	}
	q := make([]float64, nj)
	fk := func(src [][]float64, k int) []float64 {
		for j := range q {
			q[j] = src[j][k]
		}
		t := a.cfg.Kin.FK(q)
		return t[:]
	}
	i0 := s - 1
	origin := fk(w.qc, i0)
	moved := 0
	for j := 0; j < nj; j++ {
		if math.Abs(w.qc[j][e]-w.qc[j][i0]) > 1e-3 {
			moved++
		}
	}
	if axis < 0 {
		if moved < 2 {
			return HoldMetrics{}, false
		}
	}
	// Deviation from the line is rt-core/cartesian's definition, the one the
	// OLP hold's monitor and the offline harness use: a logged hold is judged
	// against its frame and axis, an inferred one against its chord.
	var metrics func(pose []float64) (off, ori, along float64)
	if axis >= 0 {
		linear = axis < 3
		direction := 1
		if sign < 0 {
			direction = -1
		}
		metrics = func(pose []float64) (float64, float64, float64) {
			off, ori, along, _ := cartesian.LineDeviation(origin, pose, frame, axis, direction)
			return off, ori, along
		}
	} else {
		dp, rv, _ := cartesian.Motion(origin, fk(w.qc, ks))
		norm := func(v [3]float64) float64 { return math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]) }
		linear = norm(dp)*1000 > norm(rv)*deg*10
		d := rv
		if linear {
			d = dp
		}
		inv := 1 / math.Max(norm(d), 1e-12)
		d = [3]float64{d[0] * inv, d[1] * inv, d[2] * inv}
		metrics = func(pose []float64) (float64, float64, float64) {
			off, ori, along, _ := cartesian.DeviationAlong(origin, pose, d, linear, sign)
			return off, ori, along
		}
	}
	hm := HoldMetrics{Frame: frame, Linear: linear, Tripped: tripped, At: a.stamp(w, s), FERatioMax: make([]float64, nj), FEOverMs: make([]float64, nj), start: s, end: e}
	if axis >= 0 {
		hm.Axis = map[bool]string{true: "+", false: "-"}[sign > 0] + axisNames[axis]
	} else if linear {
		hm.Axis = "chord"
	} else {
		hm.Axis = "rotation"
	}
	hm.Unit = map[bool]string{true: "mm/s", false: "deg/s"}[linear]
	var alc []float64
	var ss float64
	for k := i0; k <= e; k++ {
		off, ori, al := metrics(fk(w.qc, k))
		hm.CmdOffMax, hm.CmdOriMax = math.Max(hm.CmdOffMax, off), math.Max(hm.CmdOriMax, ori)
		ss += off * off
		alc = append(alc, al)
	}
	hm.CmdOffRMS = math.Sqrt(ss / float64(len(alc)))
	spd := make([]float64, len(alc))
	for i := 1; i < len(alc); i++ {
		spd[i] = (alc[i] - alc[i-1]) / w.dt
	}
	for _, v := range spd {
		hm.VMax = math.Max(hm.VMax, v)
	}
	st0, st1 := -1, -1
	for i, v := range spd {
		if v >= 0.97*hm.VMax {
			if st0 < 0 {
				st0 = i
			}
			st1 = i
		}
	}
	hm.ActOffSteady = math.NaN()
	for k := i0; k < tail; k++ {
		off, ori, _ := metrics(fk(w.qa, k))
		hm.ActOffMax, hm.ActOriMax = math.Max(hm.ActOffMax, off), math.Max(hm.ActOriMax, ori)
		hm.actOffTrace = append(hm.actOffTrace, off)
		if i := k - i0; st0 >= 0 && i >= st0+20 && i < st1 {
			hm.ActOffSteady = nanMax(hm.ActOffSteady, off)
		}
	}
	kr := ks - i0
	hm.Travel = alc[len(alc)-1]
	if kr < len(spd) {
		hm.VStop, hm.StopDist = spd[kr], alc[len(alc)-1]-alc[kr]
	}
	hm.StopMs = float64(e-ks) * w.dt * 1e3
	// joints that moved: last nonzero commanded velocity, spread across them
	first, last := math.MaxInt, -1
	for j := 0; j < nj; j++ {
		mx, lnz := 0.0, -1
		for k := i0; k <= e; k++ {
			mx = math.Max(mx, math.Abs(w.vc[j][k]))
			if w.s.TV[j][k] != 0 {
				lnz = k
			}
		}
		if mx > 0.01 && lnz >= 0 {
			first, last = min(first, lnz), max(last, lnz)
		}
	}
	if last >= 0 {
		hm.StopSpreadMs = float64(last-first) * w.dt * 1e3
	}
	J := a.p.JerkWindowCyc
	for j := 0; j < nj; j++ {
		for k := i0 + 1; k <= e; k++ {
			if acc := math.Abs(w.vc[j][k]-w.vc[j][k-1]) / w.dt; acc > hm.AccMax {
				hm.AccMax, hm.AccJoint = acc, a.cfg.Joints[j].Name
			}
			if k+1 <= e {
				hm.JerkMax = math.Max(hm.JerkMax, math.Abs(w.vc[j][k+1]-2*w.vc[j][k]+w.vc[j][k-1])/(w.dt*w.dt))
			}
		}
		for k := i0 - J; k <= e+J; k++ {
			if k-J < 0 || k+J >= w.n || !a.hasRange(w, j, k-J, k+J) {
				continue
			}
			h := float64(J) * w.dt
			if jk := math.Abs(w.vc[j][k+J]-2*w.vc[j][k]+w.vc[j][k-J]) / (h * h); jk > hm.Jerk20Max {
				hm.Jerk20Max, hm.Jerk20Joint = jk, a.cfg.Joints[j].Name
			}
		}
		jc := a.cfg.Joints[j]
		for k := max(i0-20, 1); k < tail; k++ {
			if !w.on[j][k] || !w.on[j][k-1] {
				continue
			}
			r := math.Abs(w.qc[j][k-1]-w.qa[j][k]) / (jc.FollowingError + math.Abs(w.vc[j][k])*a.cfg.LagAllowanceS)
			hm.FERatioMax[j] = math.Max(hm.FERatioMax[j], r)
			if r > 1 {
				hm.FEOverMs[j] += w.dt * 1e3
			}
		}
	}
	return hm, true
}

// TrajMetrics is one Move-to/trajectory (active mode 2): its joint-space
// straightness (the commanded joint vector against the straight segment from
// start to end, progress taken from the joint that moves most), peak speed,
// acceleration and jerk per joint, and what each joint did around it.
type TrajMetrics struct {
	Traj          int64     `json:"traj"`
	At            string    `json:"at"`
	DurMs         float64   `json:"duration_ms"`
	DqDeg         []float64 `json:"dq_deg"`
	PeakVelDeg    []float64 `json:"peak_vel_deg_s"`
	PeakAccDeg    []float64 `json:"peak_acc_deg_s2"`
	Jerk1         []float64 `json:"jerk_1cycle_rad_s3"`
	Jerk20        []float64 `json:"jerk_20ms_rad_s3"`
	StraightDeg   float64   `json:"straightness_max_dev_deg"`
	StraightCount float64   `json:"straightness_max_dev_counts"`
	ResidPeakDeg  []float64 `json:"residual_peak_deg"`
	FERatioMax    []float64 `json:"fe_over_bound_max"`
	FEOverMs      []float64 `json:"fe_over_bound_ms"`
	SettleMs      float64   `json:"settle_after_end_ms"`
	AccLimitUse   float64   `json:"acc_over_limit"`
	JerkLimitUse  float64   `json:"jerk20_over_jog_jerk"`
}

func (a *Analysis) trajectories(shifts []int) []TrajMetrics {
	var out []TrajMetrics
	nj := len(a.cfg.Joints)
	for _, w := range a.wins {
		for k := 1; k < w.n; k++ {
			if !(w.s.Mode[k] == 2 && w.armed[k] && a.anyMoving(w, k) && !(w.s.Mode[k-1] == 2 && a.anyMoving(w, k-1) && w.s.Traj[k-1] == w.s.Traj[k])) {
				continue
			}
			s, id := k, w.s.Traj[k]
			e := s
			for e < w.n && w.s.Mode[e] == 2 && w.s.Traj[e] == id && w.armed[e] && a.anyMoving(w, e) {
				e++
			}
			k = e
			tm := TrajMetrics{Traj: id, At: a.stamp(w, s), DurMs: float64(e-s) * w.dt * 1e3}
			for _, p := range []*[]float64{&tm.DqDeg, &tm.PeakVelDeg, &tm.PeakAccDeg, &tm.Jerk1, &tm.Jerk20, &tm.ResidPeakDeg, &tm.FERatioMax, &tm.FEOverMs} {
				*p = make([]float64, nj)
			}
			i0, i1 := s-1, e // commanded rest before and after
			if i1 >= w.n {
				i1 = w.n - 1
			}
			lead, span := 0, 0.0
			for j := 0; j < nj; j++ {
				d := w.qc[j][i1] - w.qc[j][i0]
				tm.DqDeg[j] = d * deg
				if math.Abs(d) > span {
					lead, span = j, math.Abs(d)
				}
			}
			J := a.p.JerkWindowCyc
			for j := 0; j < nj; j++ {
				jc := a.cfg.Joints[j]
				for q := s; q <= i1; q++ {
					tm.PeakVelDeg[j] = math.Max(tm.PeakVelDeg[j], math.Abs(w.vc[j][q])*deg)
					tm.PeakAccDeg[j] = math.Max(tm.PeakAccDeg[j], math.Abs(w.vc[j][q]-w.vc[j][q-1])/w.dt*deg)
					if q+1 < w.n {
						tm.Jerk1[j] = math.Max(tm.Jerk1[j], math.Abs(w.vc[j][q+1]-2*w.vc[j][q]+w.vc[j][q-1])/(w.dt*w.dt))
					}
					if q-J >= 0 && q+J < w.n && a.hasRange(w, j, q-J, q+J) {
						h := float64(J) * w.dt
						tm.Jerk20[j] = math.Max(tm.Jerk20[j], math.Abs(w.vc[j][q+J]-2*w.vc[j][q]+w.vc[j][q-J])/(h*h))
					}
					if span > 0 {
						f := (w.qc[lead][q] - w.qc[lead][i0]) / (w.qc[lead][i1] - w.qc[lead][i0])
						dev := w.qc[j][q] - (w.qc[j][i0] + f*(w.qc[j][i1]-w.qc[j][i0]))
						tm.StraightDeg = math.Max(tm.StraightDeg, math.Abs(dev)*deg)
						tm.StraightCount = math.Max(tm.StraightCount, math.Abs(dev)*jc.CountsPerRad)
					}
				}
				if jc.AccelerationLim > 0 {
					tm.AccLimitUse = math.Max(tm.AccLimitUse, tm.PeakAccDeg[j]/deg/jc.AccelerationLim)
				}
				if a.cfg.JogJerk > 0 {
					tm.JerkLimitUse = math.Max(tm.JerkLimitUse, tm.Jerk20[j]/a.cfg.JogJerk)
				}
			}
			// around it: from start to the next motion, disarm, or 3 s after
			tail := e
			for tail < w.n && tail < e+int(3/w.dt) && w.armed[tail] && !a.anyMoving(w, tail) {
				tail++
			}
			lastOut := e
			for j := 0; j < nj; j++ {
				jc := a.cfg.Joints[j]
				sh := shifts[j]
				for q := s; q < tail; q++ {
					if !w.on[j][q] || !w.on[j][q-1] {
						continue
					}
					if q-sh >= 0 && w.on[j][q-sh] {
						tm.ResidPeakDeg[j] = math.Max(tm.ResidPeakDeg[j], math.Abs(w.qa[j][q]-w.qc[j][q-sh])*deg)
					}
					r := math.Abs(w.qc[j][q-1]-w.qa[j][q]) / (jc.FollowingError + math.Abs(w.vc[j][q])*a.cfg.LagAllowanceS)
					tm.FERatioMax[j] = math.Max(tm.FERatioMax[j], r)
					if r > 1 {
						tm.FEOverMs[j] += w.dt * 1e3
					}
					if q >= e && math.Abs(w.qa[j][q]-w.qc[j][q]) > jc.CompletionTol && q > lastOut {
						lastOut = q
					}
				}
			}
			tm.SettleMs = float64(lastOut-e) * w.dt * 1e3
			out = append(out, tm)
		}
	}
	return out
}
