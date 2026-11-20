package main

import (
	"fmt"
	"math"
	"sort"
)

// lineFloor is the median power of the +-25 bins around k, the line's own
// +-3 bins excluded, so a line is judged against the spectrum it stands in.
func lineFloor(p []float64, k int) float64 {
	var v []float64
	for i := k - 25; i <= k+25; i++ {
		if i < 1 || i >= len(p) || absInt(i-k) <= 3 {
			continue
		}
		v = append(v, p[i])
	}
	return median(v)
}

// findLines returns the sharp lines of one averaged power spectrum (signal^2
// per bin) at or above LineMinHz, strongest first, at most five, none within
// three bins of a stronger one.
func findLines(p []float64, df float64, sp ServoParams) []ServoLine {
	var cand []ServoLine
	kmin := max(int(math.Ceil(sp.LineMinHz/df)), 2)
	for k := kmin; k < len(p)-1; k++ {
		if !(p[k] >= p[k-1] && p[k] > p[k+1]) {
			continue
		}
		fl := lineFloor(p, k)
		if !(fl > 0) || p[k] < sp.LineOverFloor*fl {
			continue
		}
		pl := 0.0
		for i := k - 3; i <= k+3; i++ {
			if i >= 0 && i < len(p) {
				pl += p[i] - fl
			}
		}
		amp := math.Sqrt(2 * math.Max(pl, 0))
		if amp < sp.LineMinAmp {
			continue
		}
		f0 := float64(k) * df
		if p[k-1] > 0 && p[k+1] > 0 {
			l1, l2, l3 := math.Log(p[k-1]), math.Log(p[k]), math.Log(p[k+1])
			if d := l1 - 2*l2 + l3; d < 0 {
				f0 += 0.5 * (l1 - l3) / d * df
			}
		}
		half := (p[k] - fl) / 2
		lo, hi := float64(k), float64(k)
		for i := k; i > 0; i-- {
			if p[i-1]-fl <= half {
				lo = float64(i-1) + (half-(p[i-1]-fl))/((p[i]-fl)-(p[i-1]-fl))
				break
			}
		}
		for i := k; i < len(p)-1; i++ {
			if p[i+1]-fl <= half {
				hi = float64(i) + ((p[i]-fl)-half)/((p[i]-fl)-(p[i+1]-fl))
				break
			}
		}
		width := math.Max(hi-lo, 1e-9)
		ln := ServoLine{Hz: f0, Amp: amp, Q: f0 / (width * df), QLimited: width <= 2.0, OverFloor: p[k] / fl}
		ln.Mains = f0 >= 59 && math.Abs(f0-60*math.Round(f0/60)) <= 1.5
		cand = append(cand, ln)
	}
	sort.SliceStable(cand, func(x, y int) bool { return cand[x].Amp > cand[y].Amp })
	var out []ServoLine
	for _, c := range cand {
		near := false
		for _, o := range out {
			if math.Abs(o.Hz-c.Hz) <= 3*df {
				near = true
			}
		}
		if !near {
			out = append(out, c)
		}
		if len(out) == 5 {
			break
		}
	}
	return out
}

// dominant returns the strongest bin of band b and the band's spread: the
// width holding the middle half of its power (25th to 75th percentile
// frequency). A sharp line reads a few Hz; a broad loop oscillation, such
// as a Rosie 1430 arm's J2 80-250 Hz hump with lines on it (2026-10-08), reads tens.
func dominant(p []float64, df float64, b [2]float64, nyq float64) (float64, float64) {
	best := -1
	total := 0.0
	for k := range p {
		if inBand(float64(k)*df, b, nyq) {
			total += p[k]
			if best < 0 || p[k] > p[best] {
				best = k
			}
		}
	}
	if best < 0 || !(total > 0) {
		return nan(), nan()
	}
	quantile := func(q float64) float64 {
		acc := 0.0
		for k := range p {
			if !inBand(float64(k)*df, b, nyq) {
				continue
			}
			if acc+p[k] >= q*total {
				return (float64(k) - 0.5 + (q*total-acc)/p[k]) * df
			}
			acc += p[k]
		}
		return b[1]
	}
	return float64(best) * df, quantile(0.75) - quantile(0.25)
}

// timeline: every TimelineMs, a TimelineCycles Hann window centred on the
// step, where the arm is armed with every joint enabled throughout; per
// joint the torque band RMS, the loop band, the spectrum (dB) and the
// joint's mean |commanded speed|, position and torque level.
func (sq *ServoQuality) timeline(a *Analysis, states [][]int) {
	sp := a.p.Servo
	if len(a.wins) == 0 {
		return
	}
	n, h := sp.TimelineCycles, sp.TimelineCycles/2
	w0 := a.wins[0]
	sq.t0 = w0.s.TNs[0] + w0.s.ClockOffsetNs
	nj := len(a.cfg.Joints)
	for wi, w := range a.wins {
		step := max(int(math.Round(float64(sp.TimelineMs)*1e-3/w.dt)), 1)
		sq.step = float64(step) * w.dt
		df, nyq := 1/(w.dt*float64(n)), 0.5/w.dt
		on := make([]int, w.n+1)
		for k := 0; k < w.n; k++ {
			on[k+1] = on[k]
			if a.allOn(w, k) {
				on[k+1]++
			}
		}
		for k := h; k+h <= w.n; k += step {
			if on[k+h]-on[k-h] != n {
				continue
			}
			// the window's state is its most active one: Move-to, then
			// moving, then stop, then idle
			state := stIdle
			rank := [nStates]int{stIdle: 0, stStop: 1, stMoving: 2, stMoveTo: 3}
			for q := k - h; q < k+h; q++ {
				if s := states[wi][q]; s >= 0 && rank[s] > rank[state] {
					state = s
				}
			}
			p := tlPoint{t: float64(w.s.TNs[k]+w.s.ClockOffsetNs-sq.t0) / 1e9, w: wi, k: k, state: state, j: make([]tlJoint, nj)}
			for j := 0; j < nj; j++ {
				tj := tlJoint{hasTorque: w.tqOK[j], still: true, loop: nan()}
				for q := k - h; q < k+h; q++ {
					tj.vAbs += math.Abs(w.vc[j][q])
					tj.q += w.qc[j][q]
					tj.load += w.tq[j][q]
					if w.s.TV[j][q] != 0 {
						tj.still = false
					}
				}
				tj.vAbs, tj.q, tj.load = tj.vAbs/float64(n)*deg, tj.q/float64(n)*deg, tj.load/float64(n)
				if tj.hasTorque {
					s := newSpectrum(w.tq[j][k-h:k+h], n)
					pw := make([]float64, n/2+1)
					tj.spec = make([]float32, n/2+1)
					for b := range pw {
						pw[b] = s.power(b)
						tj.spec[b] = float32(10 * math.Log10(math.Max(pw[b], 1e-6)))
					}
					for _, b := range sp.BandsHz {
						tj.band = append(tj.band, bandRMS(pw, df, b, nyq))
					}
					tj.loop = bandRMS(pw, df, sp.LoopBandHz, nyq)
				}
				p.j[j] = tj
			}
			sq.tl = append(sq.tl, p)
		}
	}
}

// peakHz: the loop-band maximum of the timeline spectra averaged (in power)
// over the given points, refined by a parabola through log power.
func (sq *ServoQuality) peakHz(a *Analysis, idx []int, j int) float64 {
	if len(idx) == 0 || len(sq.tl[idx[0]].j[j].spec) == 0 {
		return nan()
	}
	nb := len(sq.tl[idx[0]].j[j].spec)
	df := sq.TimeBinHz
	avg := make([]float64, nb)
	for _, i := range idx {
		for b, v := range sq.tl[i].j[j].spec {
			avg[b] += math.Pow(10, float64(v)/10)
		}
	}
	best := -1
	for b := range avg {
		if inBand(float64(b)*df, sq.LoopBand, sq.NyquistHz) && (best < 0 || avg[b] > avg[best]) {
			best = b
		}
	}
	if best < 0 {
		return nan()
	}
	f := float64(best) * df
	if best > 0 && best < nb-1 {
		l1, l2, l3 := math.Log(avg[best-1]), math.Log(avg[best]), math.Log(avg[best+1])
		if d := l1 - 2*l2 + l3; d < 0 {
			f += 0.5 * (l1 - l3) / d * df
		}
	}
	return f
}

func (sq *ServoQuality) episodes(a *Analysis) {
	sp := a.p.Servo
	nj := len(a.cfg.Joints)
	merge := float64(sp.BuzzMergeMs) * 1e-3
	for j := 0; j < nj; j++ {
		js := &sq.Joints[j]
		pk := -1
		for i, p := range sq.tl {
			if p.j[j].hasTorque && (pk < 0 || p.j[j].loop > sq.tl[pk].j[j].loop) {
				pk = i
			}
		}
		if pk >= 0 {
			p := sq.tl[pk]
			js.LoopPeakRMS, js.LoopPeakAt, js.LoopPeakHz = p.j[j].loop, a.stamp(a.wins[p.w], p.k), sq.peakHz(a, []int{pk}, j)
		}
		var cur []int
		flush := func() {
			if len(cur) == 0 {
				return
			}
			i0, i1 := cur[0], cur[len(cur)-1]
			p0, p1 := sq.tl[i0], sq.tl[i1]
			dur := p1.t - p0.t + sq.step
			if dur*1e3 < float64(sp.BuzzMinMs) {
				cur = cur[:0]
				return
			}
			e := BuzzEpisode{Joint: js.Joint, Start: a.stamp(a.wins[p0.w], p0.k), End: a.stamp(a.wins[p1.w], p1.k), DurS: dur,
				OnsetState: stateNames[max(p0.state, 0)], OnsetSpeed: p0.j[j].vAbs, OnsetPosDeg: p0.j[j].q, FreqHz: sq.peakHz(a, cur, j),
				Others: make([]float64, nj), t0: p0.t, t1: p1.t}
			span := 0
			// strongest: the joint furthest over its own threshold (a
			// joint moving is louder than one held still for the same loop)
			over := make([]float64, nj)
			for i := i0; i <= i1; i++ {
				if sq.tl[i].j[j].hasTorque {
					span++
				}
				for o := 0; o < nj; o++ {
					to := sq.tl[i].j[o]
					if to.loop > e.Others[o] {
						e.Others[o] = to.loop
					}
					if x := to.loop / thresholdFor(sp, to.still); x > over[o] {
						over[o] = x
					}
				}
			}
			e.OnFraction = float64(len(cur)) / float64(max(span, 1))
			for _, i := range cur {
				if v := sq.tl[i].j[j].loop; v > e.PeakRMS {
					e.PeakRMS, e.PeakAt = v, a.stamp(a.wins[sq.tl[i].w], sq.tl[i].k)
				}
			}
			src := 0
			for o := range over {
				if over[o] > over[src] {
					src = o
				}
			}
			e.Source, e.SourceOver = a.cfg.Joints[src].Name, over[src]
			w := a.wins[p0.w]
			best := math.MaxInt
			for _, ev := range a.events(w) {
				if d := p0.k - ev.k; absInt(d) < absInt(best) {
					best, e.Near = d, fmt.Sprintf("%s (%+.0f ms)", ev.label, float64(d)*w.dt*1e3)
				}
			}
			sq.Episodes = append(sq.Episodes, e)
			js.Episodes++
			js.EpisodeS += dur
			cur = cur[:0]
		}
		lastT := math.Inf(-1)
		for i, p := range sq.tl {
			tj := p.j[j]
			if !tj.hasTorque || tj.loop < thresholdFor(sp, tj.still) {
				continue
			}
			if len(cur) > 0 && p.t-lastT > merge {
				flush()
			}
			cur = append(cur, i)
			lastT = p.t
		}
		flush()
	}
	sort.SliceStable(sq.Episodes, func(x, y int) bool { return sq.Episodes[x].t0 < sq.Episodes[y].t0 })
}

func thresholdFor(sp ServoParams, still bool) float64 {
	if still {
		return sp.BuzzStillOn
	}
	return sp.BuzzMovingOn
}

var speedBinEdges = []float64{5, 15, 30, 60}

func speedBinLabel(i int) string {
	switch {
	case i == 0:
		return "commanded still"
	case i == 1:
		return fmt.Sprintf("< %g deg/s", speedBinEdges[0])
	case i == len(speedBinEdges)+1:
		return fmt.Sprintf(">= %g deg/s", speedBinEdges[len(speedBinEdges)-1])
	}
	return fmt.Sprintf("%g-%g deg/s", speedBinEdges[i-2], speedBinEdges[i-1])
}

// correlate: how the loop band follows the joint's own speed, position and
// torque level over the timeline windows (Spearman rank correlation, ties
// averaged; NaN under 20 windows).
func (sq *ServoQuality) correlate(a *Analysis) {
	for j := range sq.Joints {
		js := &sq.Joints[j]
		nbins := len(speedBinEdges) + 2
		bins := make([][]float64, nbins)
		var loop, v, q, ld []float64
		for _, p := range sq.tl {
			tj := p.j[j]
			if !tj.hasTorque {
				continue
			}
			loop, v, q, ld = append(loop, tj.loop), append(v, tj.vAbs), append(q, tj.q), append(ld, math.Abs(tj.load))
			b := 0
			if !tj.still {
				b = 1
				for _, e := range speedBinEdges {
					if tj.vAbs >= e {
						b++
					}
				}
			}
			bins[b] = append(bins[b], tj.loop)
		}
		for i, b := range bins {
			if len(b) == 0 {
				continue
			}
			js.SpeedBins = append(js.SpeedBins, SpeedBin{Label: speedBinLabel(i), Windows: len(b), Median: median(b), P95: percentile(b, 95)})
		}
		js.CorrSpeed, js.CorrPos, js.CorrLoad = spearman(loop, v), spearman(loop, q), spearman(loop, ld)
	}
}

func ranks(x []float64) []float64 {
	idx := make([]int, len(x))
	for i := range idx {
		idx[i] = i
	}
	sort.SliceStable(idx, func(a, b int) bool { return x[idx[a]] < x[idx[b]] })
	r := make([]float64, len(x))
	for i := 0; i < len(idx); {
		e := i
		for e+1 < len(idx) && x[idx[e+1]] == x[idx[i]] {
			e++
		}
		avg := float64(i+e)/2 + 1
		for k := i; k <= e; k++ {
			r[idx[k]] = avg
		}
		i = e + 1
	}
	return r
}

func spearman(x, y []float64) float64 {
	if len(x) < 20 || len(x) != len(y) {
		return nan()
	}
	rx, ry := ranks(x), ranks(y)
	var mx, my float64
	for i := range rx {
		mx += rx[i]
		my += ry[i]
	}
	mx /= float64(len(rx))
	my /= float64(len(ry))
	var sxy, sxx, syy float64
	for i := range rx {
		sxy += (rx[i] - mx) * (ry[i] - my)
		sxx += (rx[i] - mx) * (rx[i] - mx)
		syy += (ry[i] - my) * (ry[i] - my)
	}
	if sxx == 0 || syy == 0 {
		return nan()
	}
	return sxy / math.Sqrt(sxx*syy)
}

// holds: torque band energies per joint over each Cartesian path hold's
// commanded motion (lane open to the end of commanded motion).
func (sq *ServoQuality) holds(a *Analysis, holds []HoldMetrics) {
	sp := a.p.Servo
	m := sp.SegmentCycles
	nj := len(a.cfg.Joints)
	for _, h := range holds {
		if h.window >= len(a.wins) {
			continue
		}
		w := a.wins[h.window]
		var segs []segment
		st := make([]int, h.end-h.start)
		for k := h.start; k < h.end; k++ {
			if !a.allOn(w, k) {
				st[k-h.start] = -1
			}
		}
		for _, sg := range segmentsOf(st, 0, h.window, m) {
			sg.a += h.start
			segs = append(segs, sg)
		}
		if len(segs) == 0 {
			continue
		}
		df, nyq := 1/(w.dt*float64(m)), 0.5/w.dt
		sh := ServoHold{Source: h.Source, Rev: h.Rev, At: h.At, Axis: h.Axis, Speed: h.SpeedReq, Unit: h.Unit,
			TorqueRMS: make([][]float64, nj), LoopRMS: make([]float64, nj), PeakHz: make([]float64, nj), psd: make([][]float64, nj)}
		if h.Source != "olp" {
			sh.Speed = h.VMax
		}
		for j := 0; j < nj; j++ {
			sh.LoopRMS[j], sh.PeakHz[j] = nan(), nan()
			for range sp.BandsHz {
				sh.TorqueRMS[j] = append(sh.TorqueRMS[j], nan())
			}
			if !w.tqOK[j] {
				continue
			}
			p := make([]float64, m/2+1)
			for _, sg := range segs {
				s := newSpectrum(w.tq[j][sg.a:sg.a+sg.n], m)
				for k := range p {
					p[k] += s.power(k) / float64(len(segs))
				}
			}
			for bi, b := range sp.BandsHz {
				sh.TorqueRMS[j][bi] = bandRMS(p, df, b, nyq)
			}
			sh.LoopRMS[j] = bandRMS(p, df, sp.LoopBandHz, nyq)
			best := -1
			for k := range p {
				if inBand(float64(k)*df, sp.LoopBandHz, nyq) && (best < 0 || p[k] > p[best]) {
					best = k
				}
			}
			if best >= 0 {
				sh.PeakHz[j] = float64(best) * df
			}
			sh.psd[j] = p
		}
		sq.Holds = append(sq.Holds, sh)
	}
}
