package main

import (
	"fmt"
	"math"
	"math/cmplx"
)

// RingMetrics describes a joint moving on its own: the residual
// actual(t) - commanded(t - lag) with the joint's own best-shift lag removed,
// so what is left is ringing whether the joint is commanded still (the J4
// wrist under J1/J6 S-curves) or moving.
type RingMetrics struct {
	ShiftMs        float64       `json:"residual_shift_ms"`
	SpectrumPeakHz float64       `json:"spectrum_peak_hz"`
	SpectrumAmpDeg float64       `json:"spectrum_peak_amplitude_deg"`
	Episodes       []RingEpisode `json:"episodes"`
	WorstPeakDeg   float64       `json:"worst_peak_deg"`
	WorstAt        string        `json:"worst_at,omitempty"`
	WorstFreqHz    float64       `json:"worst_freq_hz"`
	WorstDamping   float64       `json:"worst_damping_ratio"`
	WorstSettleMs  float64       `json:"worst_settle_after_peak_ms"`
	MedianFreqHz   float64       `json:"major_episode_freq_hz"`
	MedianDamping  float64       `json:"major_episode_damping_ratio"`
	spectrumF      []float64
	spectrumA      []float64
}

type RingEpisode struct {
	At             string  `json:"at"`
	PeakAt         string  `json:"peak_at"`
	PeakDeg        float64 `json:"peak_deg"`
	DurMs          float64 `json:"duration_ms"`
	SettleMs       float64 `json:"settle_after_peak_ms"`
	FreqZCHz       float64 `json:"freq_zero_crossing_hz"`
	FreqDFTHz      float64 `json:"freq_dft_hz"`
	Damping        float64 `json:"damping_ratio"`
	DampingPeaks   int     `json:"damping_half_cycles"`
	CmdVelDeg      float64 `json:"joint_cmd_speed_peak_deg_s"`
	FERatioPeak    float64 `json:"fe_over_bound_peak"`
	Near           string  `json:"nearest_event,omitempty"`
	ExciteJoint    string  `json:"excited_by,omitempty"`
	ExciteAccDeg   float64 `json:"excitation_cmd_acc_deg_s2"`
	SensitivityDeg float64 `json:"peak_per_excitation_deg_per_deg_s2"`
	trace          []float64
}

// residual returns actual(t) - commanded(t - sh) in rad, NaN where either
// sample is not armed and enabled.
func residual(w *win, j, sh int) []float64 {
	r := make([]float64, w.n)
	for k := range r {
		if k < sh || !w.on[j][k] || !w.on[j][k-sh] {
			r[k] = math.NaN()
			continue
		}
		r[k] = w.qa[j][k] - w.qc[j][k-sh]
	}
	return r
}

type event struct {
	k     int
	label string
}

func (a *Analysis) events(w *win) []event {
	var ev []event
	for k := 1; k < w.n; k++ {
		s := w.s
		if s.JogOpen[k] == 1 && s.JogOpen[k-1] == 0 {
			ev = append(ev, event{k, fmt.Sprintf("jog lane %d open", s.JogGen[k])})
		}
		if s.JogOpen[k] == 0 && s.JogOpen[k-1] == 1 {
			ev = append(ev, event{k, fmt.Sprintf("jog lane %d stop", s.JogGen[k-1])})
		}
		if s.Mode[k] == 2 && s.Mode[k-1] != 2 {
			ev = append(ev, event{k, fmt.Sprintf("Move-to traj %d start", s.Traj[k])})
		}
		if s.Mode[k] != 2 && s.Mode[k-1] == 2 {
			ev = append(ev, event{k, fmt.Sprintf("Move-to traj %d end", s.Traj[k-1])})
		}
	}
	return ev
}

func (a *Analysis) ring(j int, shiftMs float64) RingMetrics {
	jc := a.cfg.Joints[j]
	rm := RingMetrics{ShiftMs: shiftMs}
	trig, quiet := a.p.RingTriggerFactor*jc.CompletionTol, jc.CompletionTol
	var all []RingEpisode
	var welch welchAcc
	for _, w := range a.wins {
		sh := int(math.Round(shiftMs * 1e-3 / w.dt))
		r := residual(w, j, sh)
		welch.add(r, w.dt)
		evs := a.events(w)
		merge := int(float64(a.p.RingMergeMs) * 1e-3 / w.dt)
		quietN := int(float64(a.p.RingQuietMs) * 1e-3 / w.dt)
		type span struct{ s, e int }
		var spans []span
		for k := 0; k < w.n; k++ {
			if math.IsNaN(r[k]) || math.Abs(r[k]) <= trig {
				continue
			}
			e := k
			for e < w.n && !math.IsNaN(r[e]) && math.Abs(r[e]) > trig {
				e++
			}
			if n := len(spans); n > 0 && k-spans[n-1].e < merge {
				spans[n-1].e = e
			} else {
				spans = append(spans, span{k, e})
			}
			k = e
		}
		for _, sp := range spans {
			kp, peak := sp.s, 0.0
			for k := sp.s; k < sp.e; k++ {
				if !math.IsNaN(r[k]) && math.Abs(r[k]) > math.Abs(peak) {
					kp, peak = k, r[k]
				}
			}
			// settle: last sample outside completion_tolerance before the
			// residual stays inside it for RingQuietMs (or the data ends).
			last, in := kp, 0
			for k := kp; k < w.n && !math.IsNaN(r[k]); k++ {
				if math.Abs(r[k]) > quiet {
					last, in = k, 0
				} else if in++; in >= quietN {
					break
				}
			}
			ep := RingEpisode{At: a.stamp(w, sp.s), PeakAt: a.stamp(w, kp), PeakDeg: peak * deg,
				DurMs: float64(last-sp.s) * w.dt * 1e3, SettleMs: float64(last-kp) * w.dt * 1e3}
			lo, hi := sp.s-200, last+200
			if lo < 0 {
				lo = 0
			}
			if hi > w.n {
				hi = w.n
			}
			seg := finiteDemeaned(r[lo:hi])
			ep.FreqDFTHz, _ = dftPeak(seg, w.dt, a.p.SpectrumMinHz, a.p.SpectrumMaxHz)
			ep.Damping, ep.DampingPeaks, ep.FreqZCHz = dampingFromPeak(r, kp, last+quietN, quiet/2, w.dt)
			for k := sp.s; k < last && k < w.n; k++ {
				ep.CmdVelDeg = math.Max(ep.CmdVelDeg, math.Abs(w.vc[j][k])*deg)
				if k > 0 && w.on[j][k] && w.on[j][k-1] {
					b := jc.FollowingError + math.Abs(w.vc[j][k])*a.cfg.LagAllowanceS
					ep.FERatioPeak = math.Max(ep.FERatioPeak, math.Abs(w.qc[j][k-1]-w.qa[j][k])/b)
				}
			}
			// excitation: the other joints' peak commanded acceleration in
			// the 300 ms before the episode starts up to its peak.
			from := sp.s - int(0.3/w.dt)
			if from < 1 {
				from = 1
			}
			for o := range a.cfg.Joints {
				if o == j {
					continue
				}
				for k := from; k <= kp; k++ {
					if w.on[o][k] && w.on[o][k-1] {
						acc := math.Abs(w.vc[o][k]-w.vc[o][k-1]) / w.dt * deg
						if acc > ep.ExciteAccDeg {
							ep.ExciteAccDeg, ep.ExciteJoint = acc, a.cfg.Joints[o].Name
						}
					}
				}
			}
			if ep.ExciteAccDeg > 0 {
				ep.SensitivityDeg = math.Abs(ep.PeakDeg) / ep.ExciteAccDeg
			}
			best := math.MaxInt
			for _, e := range evs {
				if d := sp.s - e.k; absInt(d) < absInt(best) {
					best, ep.Near = d, fmt.Sprintf("%s (%+.0f ms)", e.label, float64(d)*w.dt*1e3)
				}
			}
			for k := kp - 200; k < kp+800 && k < w.n; k++ {
				if k >= 0 {
					ep.trace = append(ep.trace, r[k]*deg)
				}
			}
			all = append(all, ep)
		}
	}
	rm.Episodes = all
	rm.spectrumF, rm.spectrumA = welch.amplitude(a.p.SpectrumMinHz, a.p.SpectrumMaxHz)
	rm.SpectrumPeakHz, rm.SpectrumAmpDeg = math.NaN(), math.NaN()
	if len(rm.spectrumF) > 0 {
		bi := 0
		for i := range rm.spectrumA {
			if rm.spectrumA[i] > rm.spectrumA[bi] {
				bi = i
			}
		}
		rm.SpectrumPeakHz, rm.SpectrumAmpDeg = parabolicPeak(rm.spectrumF, rm.spectrumA, bi), rm.spectrumA[bi]*deg
	}
	rm.WorstFreqHz, rm.WorstDamping, rm.WorstSettleMs = math.NaN(), math.NaN(), math.NaN()
	for i, e := range all {
		if i == 0 || math.Abs(e.PeakDeg) > rm.WorstPeakDeg {
			rm.WorstPeakDeg, rm.WorstAt, rm.WorstFreqHz, rm.WorstDamping, rm.WorstSettleMs = math.Abs(e.PeakDeg), e.PeakAt, e.FreqZCHz, e.Damping, e.SettleMs
		}
	}
	// Major episodes (>= half the worst peak) carry the mode; small ones are
	// mostly tracking noise whose crossings say little about it.
	var fs, zs []float64
	for _, e := range all {
		if math.Abs(e.PeakDeg) < rm.WorstPeakDeg/2 {
			continue
		}
		if !math.IsNaN(e.FreqZCHz) {
			fs = append(fs, e.FreqZCHz)
		}
		if !math.IsNaN(e.Damping) {
			zs = append(zs, e.Damping)
		}
	}
	rm.MedianFreqHz, rm.MedianDamping = median(fs), median(zs)
	return rm
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func finiteDemeaned(x []float64) []float64 {
	var out []float64
	s := 0.0
	for _, v := range x {
		if !math.IsNaN(v) {
			out = append(out, v)
			s += v
		}
	}
	if len(out) == 0 {
		return nil
	}
	m := s / float64(len(out))
	for i := range out {
		out[i] -= m
	}
	return out
}

// dftPeak scans a Hann-windowed DFT at 0.02 Hz steps (finer than one bin,
// so a short episode still yields a usable peak) and returns its frequency.
func dftPeak(x []float64, dt, lo, hi float64) (float64, float64) {
	n := len(x)
	if n < 16 {
		return math.NaN(), math.NaN()
	}
	wsum := 0.0
	xw := make([]float64, n)
	for i := range x {
		h := 0.5 - 0.5*math.Cos(2*math.Pi*float64(i)/float64(n-1))
		xw[i] = x[i] * h
		wsum += h
	}
	bestF, bestA := math.NaN(), -1.0
	for f := lo; f <= hi; f += 0.02 {
		// Goertzel
		c := 2 * math.Cos(2*math.Pi*f*dt)
		s1, s2 := 0.0, 0.0
		for _, v := range xw {
			s0 := v + c*s1 - s2
			s2, s1 = s1, s0
		}
		p := s1*s1 + s2*s2 - c*s1*s2
		if p > bestA {
			bestF, bestA = f, p
		}
	}
	return bestF, 2 * math.Sqrt(bestA) / wsum
}

// dampingFromPeak estimates the ring after the peak: the largest |r| of each
// half cycle (between zero crossings of the residual) while it stays above
// the noise floor; the crossing spacing gives the damped frequency
// f = 1 / (2 x mean half period), and ln(amplitude) fitted linearly in time
// the decay rate sigma, so zeta = sigma / sqrt(sigma^2 + (2 pi f)^2). Only
// half cycles above the floor count, so noise crossings after the ring has
// died do not raise the frequency. Fewer than three such half cycles give
// NaN: the decay is not measurable, which is not "zero damping".
func dampingFromPeak(r []float64, kp, end int, floor, dt float64) (zeta float64, halves int, freq float64) {
	if end > len(r) {
		end = len(r)
	}
	var amps, times []float64
	var crossings []int
	curMax, curK := 0.0, kp
	sign := r[kp] >= 0
	for k := kp; k < end; k++ {
		if math.IsNaN(r[k]) {
			break
		}
		if (r[k] >= 0) != sign {
			if curMax < floor {
				break
			}
			amps, times = append(amps, curMax), append(times, float64(curK)*dt)
			crossings = append(crossings, k)
			sign, curMax = r[k] >= 0, 0
		}
		if math.Abs(r[k]) > curMax {
			curMax, curK = math.Abs(r[k]), k
		}
	}
	if len(amps) < 3 || len(crossings) < 3 {
		return math.NaN(), len(amps), math.NaN()
	}
	// Median half period: one half cycle stretched by a new kick (the
	// Move-to's deceleration re-exciting J4 at 21:04) must not move it.
	var halfs []float64
	for i := 1; i < len(crossings); i++ {
		halfs = append(halfs, float64(crossings[i]-crossings[i-1])*dt)
	}
	wd := math.Pi / median(halfs)
	// Free decay only: a ring never grows on its own, so start after the
	// last half cycle that grew (re-excitation) when >= 3 remain.
	start := 0
	for i := len(amps) - 1; i > 0; i-- {
		if amps[i] >= amps[i-1] {
			if len(amps)-i >= 3 {
				start = i
			}
			break
		}
	}
	amps, times = amps[start:], times[start:]
	n := float64(len(amps))
	var st, sl, stt, stl float64
	for i := range amps {
		l := math.Log(amps[i])
		st += times[i]
		sl += l
		stt += times[i] * times[i]
		stl += times[i] * l
	}
	sigma := -(n*stl - st*sl) / (n*stt - st*st)
	return sigma / math.Sqrt(sigma*sigma+wd*wd), len(amps), wd / (2 * math.Pi)
}

// welchAcc averages Hann-windowed power spectra of 8192-sample segments
// (50 % overlap) over every armed run; a shorter run of at least 1024
// samples is zero-padded to one segment. 8192 cycles at 1 kHz resolve
// 0.12 Hz, refined by a parabolic fit at the peak.
type welchAcc struct {
	sum   []float64
	n     int
	dt    float64
	wsum2 float64
}

const welchLen = 8192

func (wa *welchAcc) add(r []float64, dt float64) {
	wa.dt = dt
	run := []float64{}
	flush := func() {
		if len(run) >= 1024 {
			if len(run) < welchLen {
				wa.segment(run)
			} else {
				for s := 0; s+welchLen <= len(run); s += welchLen / 2 {
					wa.segment(run[s : s+welchLen])
				}
			}
		}
		run = run[:0]
	}
	for _, v := range r {
		if math.IsNaN(v) {
			flush()
			continue
		}
		run = append(run, v)
	}
	flush()
}

func (wa *welchAcc) segment(x []float64) {
	m := 0.0
	for _, v := range x {
		m += v
	}
	m /= float64(len(x))
	buf := make([]complex128, welchLen)
	w2 := 0.0
	for i, v := range x {
		h := 0.5 - 0.5*math.Cos(2*math.Pi*float64(i)/float64(len(x)-1))
		buf[i] = complex((v-m)*h, 0)
		w2 += h
	}
	fft(buf)
	if wa.sum == nil {
		wa.sum = make([]float64, welchLen/2+1)
	}
	for i := range wa.sum {
		a := 2 * cmplx.Abs(buf[i]) / w2
		wa.sum[i] += a * a
	}
	wa.n++
}

// amplitude returns frequencies and RMS-averaged sinusoid amplitudes (rad).
func (wa *welchAcc) amplitude(lo, hi float64) ([]float64, []float64) {
	if wa.n == 0 {
		return nil, nil
	}
	var f, a []float64
	df := 1 / (wa.dt * welchLen)
	for i := range wa.sum {
		fi := float64(i) * df
		if fi < lo || fi > hi {
			continue
		}
		f = append(f, fi)
		a = append(a, math.Sqrt(wa.sum[i]/float64(wa.n)))
	}
	return f, a
}

func parabolicPeak(f, a []float64, i int) float64 {
	if i <= 0 || i >= len(a)-1 {
		return f[i]
	}
	d := a[i-1] - 2*a[i] + a[i+1]
	if d == 0 {
		return f[i]
	}
	off := 0.5 * (a[i-1] - a[i+1]) / d
	return f[i] + off*(f[1]-f[0])
}

// fft is an in-place radix-2 Cooley-Tukey transform; len(x) must be 2^k.
func fft(x []complex128) {
	n := len(x)
	for i, j := 1, 0; i < n; i++ {
		bit := n >> 1
		for ; j&bit != 0; bit >>= 1 {
			j ^= bit
		}
		j ^= bit
		if i < j {
			x[i], x[j] = x[j], x[i]
		}
	}
	for size := 2; size <= n; size <<= 1 {
		w := cmplx.Exp(complex(0, -2*math.Pi/float64(size)))
		for start := 0; start < n; start += size {
			wk := complex(1, 0)
			for k := 0; k < size/2; k++ {
				u, v := x[start+k], x[start+k+size/2]*wk
				x[start+k], x[start+k+size/2] = u+v, u-v
				wk *= w
			}
		}
	}
}
