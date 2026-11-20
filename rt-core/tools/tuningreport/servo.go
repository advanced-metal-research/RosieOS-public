package main

import (
	"fmt"
	"math"
	"math/cmplx"
	"sort"
	"strings"
)

// Servo-loop quality: what each drive's loops do above the arm's own modes,
// read from the 1 kHz dump. The ring section (ring.go) stops at 30 Hz and
// reads position only, so it cannot see a speed loop oscillating: on a Rosie 1430 arm, J4
// at C00.06 = 700 % ran a 228 Hz torque line for ~9 s after arming
// (2026-10-07 22:01) and J2 at its drive-held 1143 % broke into a ~100 Hz
// oscillation of +-2000 permille while moving (2026-10-08 11:28:21), both
// found only by scratch spectra of 0x6077. This section makes that check
// part of every report: band energies per joint per state and per hold,
// sharp lines with their Q and whether other joints share them, buzz
// episodes with onset and offset, torque ripple at rest, and how the loop
// band follows speed, position and load.
//
// The dump samples the drives' PDOs at 1 kHz with no anti-alias filter:
// content above 500 Hz (current loop, PWM) folds to 1000 - f, so a line near
// 400 Hz may be a 600 Hz one, and lines near 500 Hz are ambiguous.

const (
	stIdle = iota
	stMoving
	stMoveTo
	stStop
	nStates
)

var stateNames = [nStates]string{"armed idle", "jog/hold moving", "Move-to", "stop"}

type ServoQuality struct {
	SampleHz  float64       `json:"sample_hz"`
	NyquistHz float64       `json:"nyquist_hz"`
	BinHz     float64       `json:"spectrum_bin_hz"`
	TimeBinHz float64       `json:"timeline_bin_hz"`
	Bands     [][2]float64  `json:"bands_hz"`
	LoopBand  [2]float64    `json:"loop_band_hz"`
	Joints    []ServoJoint  `json:"joints"`
	Episodes  []BuzzEpisode `json:"buzz_episodes"`
	Holds     []ServoHold   `json:"holds"`
	Notes     []string      `json:"notes,omitempty"`
	tl        []tlPoint
	t0        int64
	step      float64
}

// ServoJoint is one drive's loop quality. Torque in per mille of rated
// (0x6077), velocity in deg/s (0x606C), residual = actual(t) -
// commanded(t - lag) in mdeg, every band value an RMS.
type ServoJoint struct {
	Joint          string       `json:"joint"`
	TorqueReported bool         `json:"torque_reported"`
	States         []ServoState `json:"states"`
	Lines          []ServoLine  `json:"lines"`
	RestRippleRMS  float64      `json:"rest_torque_ripple_rms_permille"`
	RestRippleHF   float64      `json:"rest_torque_ripple_above_30hz_rms_permille"`
	RestTorqueMean float64      `json:"rest_torque_mean_permille"`
	LoopPeakRMS    float64      `json:"loop_band_peak_rms_permille"`
	LoopPeakAt     string       `json:"loop_band_peak_at,omitempty"`
	LoopPeakHz     float64      `json:"loop_band_peak_hz"`
	Episodes       int          `json:"buzz_episodes"`
	EpisodeS       float64      `json:"buzz_total_s"`
	SpeedBins      []SpeedBin   `json:"loop_band_by_speed"`
	CorrSpeed      float64      `json:"spearman_loop_vs_speed"`
	CorrPos        float64      `json:"spearman_loop_vs_position"`
	CorrLoad       float64      `json:"spearman_loop_vs_torque_level"`
	psd            [nStates][]float64
}

type ServoState struct {
	State      string    `json:"state"`
	Seconds    float64   `json:"seconds"`
	Segments   int       `json:"segments"`
	TorqueRMS  []float64 `json:"torque_band_rms_permille"`
	LoopRMS    float64   `json:"torque_loop_band_rms_permille"`
	LoopPeakHz float64   `json:"torque_loop_band_peak_hz"`
	LoopSpread float64   `json:"torque_loop_band_iqr_hz"`
	VelRMS     []float64 `json:"velocity_band_rms_deg_s"`
	ResidRMS   []float64 `json:"residual_band_rms_mdeg"`
	TorquePeak float64   `json:"torque_abs_peak_permille"`
}

// ServoLine is a sharp torque line: a local power maximum LineOverFloor
// times the median of the +-25 bins around it. Amplitude is the sinusoid's
// (peak, not RMS) from the power within +-3 bins above that floor; Q =
// f / half-power width, limited by the Hann window to about f / (1.44 bins).
// Coupled joints carry the same line with magnitude-squared coherence >=
// CoherenceMin over the same segments: a shared line has a common source
// (structure, supply), a line on one joint alone is that joint's loop.
type ServoLine struct {
	State     string    `json:"state"`
	Hz        float64   `json:"hz"`
	Amp       float64   `json:"amplitude_permille"`
	Q         float64   `json:"q"`
	QLimited  bool      `json:"q_resolution_limited"`
	OverFloor float64   `json:"peak_over_floor_x"`
	Coherence []float64 `json:"coherence_with_joints"`
	Coupled   []string  `json:"coupled_joints"`
	Strongest string    `json:"strongest_joint"`
	Verdict   string    `json:"verdict"`
	Mains     bool      `json:"near_60hz_harmonic"`
	Segments  int       `json:"segments"`
}

type SpeedBin struct {
	Label   string  `json:"own_speed"`
	Windows int     `json:"windows"`
	Median  float64 `json:"median_permille"`
	P95     float64 `json:"p95_permille"`
}

// BuzzEpisode is a run of timeline windows whose loop-band torque RMS is
// over the threshold for the joint's own state (commanded still or moving),
// gaps up to BuzzMergeMs merged. Times are window centres (+- half a
// window, 128 ms at the defaults).
type BuzzEpisode struct {
	Joint       string    `json:"joint"`
	Start       string    `json:"start"`
	End         string    `json:"end"`
	DurS        float64   `json:"duration_s"`
	OnFraction  float64   `json:"on_fraction"`
	PeakRMS     float64   `json:"peak_rms_permille"`
	PeakAt      string    `json:"peak_at"`
	FreqHz      float64   `json:"dominant_hz"`
	OnsetState  string    `json:"onset_state"`
	OnsetSpeed  float64   `json:"onset_own_speed_deg_s"`
	OnsetPosDeg float64   `json:"onset_position_deg"`
	Others      []float64 `json:"joints_peak_rms_permille"`
	Source      string    `json:"strongest_joint"`
	SourceOver  float64   `json:"strongest_x_own_threshold"`
	Near        string    `json:"nearest_event,omitempty"`
	t0, t1      float64
}

type ServoHold struct {
	Source    string      `json:"source"`
	Rev       int         `json:"rev"`
	At        string      `json:"at"`
	Axis      string      `json:"axis"`
	Speed     float64     `json:"speed"`
	Unit      string      `json:"unit"`
	TorqueRMS [][]float64 `json:"torque_band_rms_permille"` // [joint][band]
	LoopRMS   []float64   `json:"torque_loop_band_rms_permille"`
	PeakHz    []float64   `json:"loop_band_peak_hz"`
	psd       [][]float64
}

type tlPoint struct {
	t     float64
	w, k  int
	state int
	j     []tlJoint
}

type tlJoint struct {
	band             []float64
	loop             float64
	spec             []float32 // dB re 1 permille^2 per bin
	vAbs, q, load    float64
	still, hasTorque bool
}

// allOn: armed with every configured joint enabled. Spectra need every
// joint in the same state over a segment, and coherence needs them together.
func (a *Analysis) allOn(w *win, k int) bool {
	if !w.armed[k] {
		return false
	}
	for j := range a.cfg.Joints {
		if !w.on[j][k] {
			return false
		}
	}
	return true
}

// servoStates labels every cycle: -1 not armed (or a joint disabled), else
// idle, moving (a joint commanded to move outside Move-to), Move-to, or
// stop: StopLeadMs before the last commanded motion until StopWindowMs after
// it or the next motion, whichever comes first.
func (a *Analysis) servoStates(w *win) []int {
	st := make([]int, w.n)
	moving := make([]bool, w.n)
	for k := 0; k < w.n; k++ {
		moving[k] = a.anyMoving(w, k)
		switch {
		case !a.allOn(w, k):
			st[k] = -1
		case moving[k] && w.s.Mode[k] == 2:
			st[k] = stMoveTo
		case moving[k]:
			st[k] = stMoving
		default:
			st[k] = stIdle
		}
	}
	lead := int(float64(a.p.Servo.StopLeadMs) * 1e-3 / w.dt)
	post := int(float64(a.p.StopWindowMs) * 1e-3 / w.dt)
	for k := 1; k < w.n; k++ {
		if !(moving[k-1] && !moving[k]) {
			continue
		}
		for q := max(k-lead, 0); q < k; q++ {
			if st[q] >= 0 {
				st[q] = stStop
			}
		}
		for q := k; q < min(k+post, w.n) && !moving[q]; q++ {
			if st[q] >= 0 {
				st[q] = stStop
			}
		}
	}
	return st
}

// spectrum is one segment's transform: linear trend removed (the slow motion
// and the gravity torque change must not leak into the upper bins), Hann
// window over the samples, zero-padded to m. power returns signal^2 per bin,
// so the sum over a band is the band's mean square (Parseval with the
// window's power corrected): a sinusoid of amplitude A gives A^2 / 2.
type spectrum struct {
	x    []complex128
	m    int
	wss  float64
	keep bool
}

func newSpectrum(x []float64, m int) spectrum {
	n := len(x)
	var mean, st, stt, sxy float64
	for i, v := range x {
		mean += v
		t := float64(i)
		st += t
		stt += t * t
		sxy += t * v
	}
	mean /= float64(n)
	tm := st / float64(n)
	slope := (sxy - float64(n)*tm*mean) / (stt - float64(n)*tm*tm)
	buf := make([]complex128, m)
	wss := 0.0
	for i, v := range x {
		h := 0.5 - 0.5*math.Cos(2*math.Pi*float64(i)/float64(n-1))
		buf[i] = complex((v-mean-slope*(float64(i)-tm))*h, 0)
		wss += h * h
	}
	fft(buf)
	return spectrum{x: buf, m: m, wss: wss, keep: true}
}

func (s spectrum) power(k int) float64 {
	c := 2.0
	if k == 0 || k == s.m/2 {
		c = 1
	}
	a := cmplx.Abs(s.x[k])
	return c * a * a / (float64(s.m) * s.wss)
}

// inBand: lo <= f < hi, and hi itself when hi is at or above Nyquist.
func inBand(f float64, b [2]float64, nyq float64) bool {
	return f >= b[0] && (f < b[1] || (b[1] >= nyq && f <= b[1]+1e-9))
}

func bandRMS(p []float64, df float64, b [2]float64, nyq float64) float64 {
	s := 0.0
	for k, v := range p {
		if inBand(float64(k)*df, b, nyq) {
			s += v
		}
	}
	return math.Sqrt(s)
}

// Lines need four segments (about 1.3 s of one state): fewer are dominated
// by the transient of one motion. Coherence needs eight: its estimate is
// biased up by about 1 / segments.
const (
	minLineSegments      = 4
	minCoherenceSegments = 8
)

type segment struct{ w, a, n int }

// segments cuts runs of one state into Welch segments of m cycles, 50 %
// overlap; a run of m/2..m cycles is one zero-padded segment.
func segmentsOf(st []int, s, wi, m int) []segment {
	var out []segment
	for k := 0; k < len(st); {
		if st[k] != s {
			k++
			continue
		}
		e := k
		for e < len(st) && st[e] == s {
			e++
		}
		switch n := e - k; {
		case n >= m:
			for a := k; a+m <= e; a += m / 2 {
				out = append(out, segment{wi, a, m})
			}
		case n >= m/2:
			out = append(out, segment{wi, k, n})
		}
		k = e
	}
	return out
}

func finite(x []float64) bool {
	for _, v := range x {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	return true
}

// servo computes the section. shifts are the joints' residual lags in
// cycles (as the ring section uses).
func (a *Analysis) servo(shifts []int, holds []HoldMetrics) ServoQuality {
	sp := a.p.Servo
	nj := len(a.cfg.Joints)
	dt := 1e-3
	if len(a.wins) > 0 {
		dt = a.wins[0].dt
	}
	fs := 1 / dt
	m := sp.SegmentCycles
	sq := ServoQuality{SampleHz: fs, NyquistHz: fs / 2, BinHz: fs / float64(m), TimeBinHz: fs / float64(sp.TimelineCycles),
		Bands: sp.BandsHz, LoopBand: sp.LoopBandHz}
	nyq := fs / 2
	df := fs / float64(m)
	nb := m/2 + 1
	states := make([][]int, len(a.wins))
	resid := make([][][]float64, len(a.wins))
	for wi, w := range a.wins {
		states[wi] = a.servoStates(w)
		resid[wi] = make([][]float64, nj)
		for j := 0; j < nj; j++ {
			r := residual(w, j, shifts[j])
			for k := range r {
				r[k] *= deg * 1e3
			}
			resid[wi][j] = r
		}
	}
	anyTorque := make([]bool, nj)
	for _, w := range a.wins {
		for j := 0; j < nj; j++ {
			anyTorque[j] = anyTorque[j] || w.tqOK[j]
		}
	}
	torqueOf := func(w *win, j int) []float64 { return w.tq[j] }
	velOf := func(w *win, j int) []float64 {
		v := make([]float64, w.n)
		for k := range v {
			v[k] = w.va[j][k] * deg
		}
		return v
	}
	vel := make([][][]float64, len(a.wins))
	for wi, w := range a.wins {
		vel[wi] = make([][]float64, nj)
		for j := 0; j < nj; j++ {
			vel[wi][j] = velOf(w, j)
		}
	}
	for j := 0; j < nj; j++ {
		sq.Joints = append(sq.Joints, ServoJoint{Joint: a.cfg.Joints[j].Name, TorqueReported: anyTorque[j],
			RestRippleRMS: nan(), RestRippleHF: nan(), RestTorqueMean: nan(), LoopPeakHz: nan(),
			CorrSpeed: nan(), CorrPos: nan(), CorrLoad: nan()})
	}

	// --- per state: Welch power of torque, velocity and residual per joint,
	// torque cross spectra for coherence
	for s := 0; s < nStates; s++ {
		var segs []segment
		cycles := 0
		for wi := range a.wins {
			segs = append(segs, segmentsOf(states[wi], s, wi, m)...)
			for _, v := range states[wi] {
				if v == s {
					cycles++
				}
			}
		}
		pt := make([][]float64, nj)
		pv := make([][]float64, nj)
		pr := make([][]float64, nj)
		nr := make([]int, nj)
		for j := range pt {
			pt[j], pv[j], pr[j] = make([]float64, nb), make([]float64, nb), make([]float64, nb)
		}
		cross := make([][][]complex128, nj)
		auto := make([][]float64, nj)
		for j := range cross {
			cross[j] = make([][]complex128, nj)
			auto[j] = make([]float64, nb)
			for o := range cross[j] {
				cross[j][o] = make([]complex128, nb)
			}
		}
		for _, sg := range segs {
			w := a.wins[sg.w]
			specs := make([]spectrum, nj)
			for j := 0; j < nj; j++ {
				if w.tqOK[j] {
					specs[j] = newSpectrum(torqueOf(w, j)[sg.a:sg.a+sg.n], m)
					for k := 0; k < nb; k++ {
						pt[j][k] += specs[j].power(k)
						mag := cmplx.Abs(specs[j].x[k])
						auto[j][k] += mag * mag
					}
				}
				if w.vaOK[j] {
					sv := newSpectrum(vel[sg.w][j][sg.a:sg.a+sg.n], m)
					for k := 0; k < nb; k++ {
						pv[j][k] += sv.power(k)
					}
				}
				if x := resid[sg.w][j][sg.a : sg.a+sg.n]; finite(x) {
					sr := newSpectrum(x, m)
					for k := 0; k < nb; k++ {
						pr[j][k] += sr.power(k)
					}
					nr[j]++
				}
			}
			for j := 0; j < nj; j++ {
				for o := j + 1; o < nj; o++ {
					if !specs[j].keep || !specs[o].keep {
						continue
					}
					for k := 0; k < nb; k++ {
						cross[j][o][k] += specs[j].x[k] * cmplx.Conj(specs[o].x[k])
					}
				}
			}
		}
		// every joint's averages first: the coupling check reads the others'
		if n := float64(len(segs)); n > 0 {
			for j := 0; j < nj; j++ {
				for k := 0; k < nb; k++ {
					pt[j][k] /= n
					pv[j][k] /= n
					if nr[j] > 0 {
						pr[j][k] /= float64(nr[j])
					}
				}
			}
		}
		for j := 0; j < nj; j++ {
			js := &sq.Joints[j]
			ss := ServoState{State: stateNames[s], Seconds: float64(cycles) * dt, Segments: len(segs), LoopRMS: nan(), LoopPeakHz: nan(), LoopSpread: nan()}
			for range sp.BandsHz {
				ss.TorqueRMS, ss.VelRMS, ss.ResidRMS = append(ss.TorqueRMS, nan()), append(ss.VelRMS, nan()), append(ss.ResidRMS, nan())
			}
			if len(segs) > 0 {
				for bi, b := range sp.BandsHz {
					if anyTorque[j] {
						ss.TorqueRMS[bi] = bandRMS(pt[j], df, b, nyq)
					}
					ss.VelRMS[bi] = bandRMS(pv[j], df, b, nyq)
					if nr[j] > 0 {
						ss.ResidRMS[bi] = bandRMS(pr[j], df, b, nyq)
					}
				}
				if anyTorque[j] {
					ss.LoopRMS = bandRMS(pt[j], df, sp.LoopBandHz, nyq)
					js.psd[s] = pt[j]
				}
			}
			for wi, w := range a.wins {
				for k, v := range states[wi] {
					if v == s && w.tqOK[j] {
						ss.TorquePeak = math.Max(ss.TorquePeak, math.Abs(w.tq[j][k]))
					}
				}
			}
			js.States = append(js.States, ss)
			if len(segs) == 0 || !anyTorque[j] {
				continue
			}
			js.States[s].LoopPeakHz, js.States[s].LoopSpread = dominant(pt[j], df, sp.LoopBandHz, nyq)
			if len(segs) < minLineSegments {
				continue
			}
			for _, ln := range findLines(pt[j], df, sp) {
				ln.State, ln.Segments = stateNames[s], len(segs)
				ln.Coherence = make([]float64, nj)
				k0 := int(math.Round(ln.Hz / df))
				ampAt := func(p []float64) float64 {
					sum := 0.0
					for k := max(k0-3, 0); k <= min(k0+3, nb-1); k++ {
						sum += p[k]
					}
					return math.Sqrt(2 * sum)
				}
				strongest, strongAmp := j, ampAt(pt[j])
				for o := 0; o < nj; o++ {
					ln.Coherence[o] = nan()
					if o == j || len(segs) < minCoherenceSegments {
						continue
					}
					x, y := j, o
					if y < x {
						x, y = y, x
					}
					// magnitude-squared coherence over the line's +-1 bins
					var sxy complex128
					var sxx, syy float64
					for k := max(k0-1, 0); k <= min(k0+1, nb-1); k++ {
						sxy += cross[x][y][k]
						sxx += auto[x][k]
						syy += auto[y][k]
					}
					if sxx > 0 && syy > 0 {
						c := cmplx.Abs(sxy)
						ln.Coherence[o] = c * c / (sxx * syy)
					}
					if ln.Coherence[o] >= sp.CoherenceMin {
						ln.Coupled = append(ln.Coupled, a.cfg.Joints[o].Name)
						if v := ampAt(pt[o]); v > strongAmp {
							strongest, strongAmp = o, v
						}
					}
				}
				ln.Strongest = a.cfg.Joints[strongest].Name
				switch {
				case len(ln.Coupled) > 0 && strongest != j:
					ln.Verdict = fmt.Sprintf("coherent with %s; strongest on %s (%.1f permille): carried from %s", strings.Join(ln.Coupled, ", "), ln.Strongest, strongAmp, ln.Strongest)
				case len(ln.Coupled) > 0:
					ln.Verdict = fmt.Sprintf("coherent with %s; strongest here: a source shared through the structure or supply, or this joint driving them", strings.Join(ln.Coupled, ", "))
				case len(segs) < minCoherenceSegments:
					ln.Verdict = fmt.Sprintf("coupling not tested (%d segments, need %d)", len(segs), minCoherenceSegments)
				default:
					ln.Verdict = "this joint only (its own loop)"
				}
				js.Lines = append(js.Lines, ln)
			}
		}
	}
	for j := range sq.Joints {
		js := &sq.Joints[j]
		sort.SliceStable(js.Lines, func(x, y int) bool { return js.Lines[x].Amp > js.Lines[y].Amp })
		if p := js.psd[stIdle]; p != nil {
			js.RestRippleRMS = bandRMS(p, df, [2]float64{sp.BandsHz[0][0], nyq}, nyq)
			js.RestRippleHF = bandRMS(p, df, [2]float64{30, nyq}, nyq)
		}
		sum, n := 0.0, 0
		for wi, w := range a.wins {
			if !w.tqOK[j] {
				continue
			}
			for k, v := range states[wi] {
				if v == stIdle {
					sum += w.tq[j][k]
					n++
				}
			}
		}
		if n > 0 {
			js.RestTorqueMean = sum / float64(n)
		}
	}

	sq.timeline(a, states)
	sq.episodes(a)
	sq.correlate(a)
	sq.holds(a, holds)
	if len(sq.tl) == 0 {
		sq.Notes = append(sq.Notes, "no armed window long enough for the timeline")
	}
	for j := range sq.Joints {
		if !sq.Joints[j].TorqueReported {
			sq.Notes = append(sq.Notes, sq.Joints[j].Joint+": no valid 0x6077 torque in the input (a fixture written before 2026-10-08, or the drive did not report it); torque figures are absent")
		}
	}
	return sq
}
