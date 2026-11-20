package main

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"strings"
)

var stateKeys = [nStates]string{"idle", "moving", "moveto", "stop"}

func bandKey(b [2]float64) string { return fmt.Sprintf("%g_%g_hz", b[0], b[1]) }

func bandLabel(b [2]float64) string { return fmt.Sprintf("%g-%g Hz", b[0], b[1]) }

// servoScalars adds the servo-loop numbers a tuning iteration is judged by
// to the baseline delta: loop-band torque per state, every band while
// moving, ripple at rest, the worst window, buzz episodes, the strongest
// line, velocity above 80 Hz and the residual's low band.
func (r *Report) servoScalars(add func(key, label, unit string, v float64, better string)) {
	sq := r.Servo
	if sq == nil {
		return
	}
	loop := bandLabel(sq.LoopBand)
	for _, js := range sq.Joints {
		p := "servo." + js.Joint + "."
		for s, st := range js.States {
			add(p+"torque_loop_rms."+stateKeys[s], js.Joint+" torque "+loop+" RMS, "+st.State, "permille", st.LoopRMS, "lower")
		}
		if len(js.States) > stMoving {
			mv := js.States[stMoving]
			for bi, b := range sq.Bands {
				add(p+"torque_"+bandKey(b)+"_rms.moving", js.Joint+" torque "+bandLabel(b)+" RMS, moving", "permille", mv.TorqueRMS[bi], "lower")
				if b[0] >= 80 {
					add(p+"velocity_"+bandKey(b)+"_rms.moving", js.Joint+" velocity "+bandLabel(b)+" RMS, moving", "deg/s", mv.VelRMS[bi], "lower")
				}
			}
			add(p+"residual_"+bandKey(sq.Bands[0])+"_rms.moving", js.Joint+" residual "+bandLabel(sq.Bands[0])+" RMS, moving", "mdeg", mv.ResidRMS[0], "lower")
		}
		add(p+"rest_ripple_rms", js.Joint+" torque ripple at rest", "permille", js.RestRippleRMS, "lower")
		add(p+"rest_ripple_above_30hz_rms", js.Joint+" torque ripple at rest above 30 Hz", "permille", js.RestRippleHF, "lower")
		add(p+"loop_peak_rms", js.Joint+" worst "+loop+" torque window", "permille", js.LoopPeakRMS, "lower")
		add(p+"buzz_episodes", js.Joint+" buzz episodes", "", float64(js.Episodes), "lower")
		add(p+"buzz_total_s", js.Joint+" buzz time", "s", js.EpisodeS, "lower")
		amp, hz := 0.0, nan()
		for _, l := range js.Lines {
			if l.Amp > amp {
				amp, hz = l.Amp, l.Hz
			}
		}
		add(p+"top_line_amplitude", js.Joint+" strongest torque line amplitude", "permille", amp, "lower")
		add(p+"top_line_hz", js.Joint+" strongest torque line frequency", "Hz", hz, "")
	}
}

func (r *Report) servoMarkdown(w func(string, ...any), plots []string) {
	sq := r.Servo
	if sq == nil {
		return
	}
	sp := r.Params.Servo
	loop := bandLabel(sq.LoopBand)
	w("\n## Servo loop quality\n\n")
	w("Every joint's drive loops above the arm's modes, from the %g Hz dump: torque 0x6077 in per mille of rated, velocity 0x606C in deg/s, residual = actual(t) - commanded(t - lag) in mdeg; band values are RMS. ", sq.SampleHz)
	w("Spectra reach Nyquist (%g Hz); the drives' PDOs are sampled with no anti-alias filter, so content above %g Hz folds to %g - f (a line near 400 Hz may be a 600 Hz one). ", sq.NyquistHz, sq.NyquistHz, sq.SampleHz)
	w("States (armed, every joint enabled): armed idle (nothing commanded), jog/hold moving, Move-to, stop (%d ms before commanded motion ends to %d ms after, or the next motion). ", sp.StopLeadMs, r.Params.StopWindowMs)
	w("State and hold spectra: Welch, %d-cycle Hann segments (%.2f Hz bins), linear trend removed. Timeline: %d-cycle Hann windows every %d ms (%.2f Hz bins). ", sp.SegmentCycles, sq.BinHz, sp.TimelineCycles, sp.TimelineMs, sq.TimeBinHz)
	w("Buzz episode: %s torque RMS of a timeline window >= %g permille with the joint commanded still, >= %g moving; >= %d ms, gaps <= %d ms merged (detection settings from a Rosie 1430 arm's measurements, UNVERIFIED as limits; `params.servo`). ",
		loop, sp.BuzzStillOn, sp.BuzzMovingOn, sp.BuzzMinMs, sp.BuzzMergeMs)
	w("The 1-30 Hz torque band holds the torque that accelerates the arm: judge rings on the residual and the ring section.\n\n")
	for _, n := range sq.Notes {
		w("- **Note:** %s.\n", n)
	}
	if len(sq.Notes) > 0 {
		w("\n")
	}

	w("| joint | ripple at rest RMS (> 30 Hz) | mean torque at rest | %s RMS idle / moving / Move-to / stop | moving: peak Hz (spread Hz) | worst window (at, Hz) | buzz episodes (s) | strongest line Hz, amplitude, Q, state | coupling | loop vs speed / position / load (Spearman) |\n|---|---|---|---|---|---|---|---|---|---|\n", loop)
	for _, js := range sq.Joints {
		var ls []string
		for _, st := range js.States {
			ls = append(ls, f1(st.LoopRMS, 1))
		}
		mvPeak := "-"
		if len(js.States) > stMoving {
			mvPeak = fmt.Sprintf("%s (%s)", f1(js.States[stMoving].LoopPeakHz, 0), f1(js.States[stMoving].LoopSpread, 0))
		}
		line, coup := "-", "-"
		if len(js.Lines) > 0 {
			l := js.Lines[0]
			line = fmt.Sprintf("%s, %s, %s, %s", f1(l.Hz, 1), f1(l.Amp, 1), qText(l), l.State)
			coup = l.Verdict
			if l.Mains {
				coup += "; 60 Hz harmonic"
			}
		}
		w("| %s | %s (%s) | %s | %s | %s | %s (%s, %s) | %d (%s) | %s | %s | %s / %s / %s |\n", js.Joint, f1(js.RestRippleRMS, 2), f1(js.RestRippleHF, 2), f1(js.RestTorqueMean, 0),
			strings.Join(ls, " / "), mvPeak, f1(js.LoopPeakRMS, 1), js.LoopPeakAt, f1(js.LoopPeakHz, 0), js.Episodes, f1(js.EpisodeS, 1), line, coup,
			f1(js.CorrSpeed, 2), f1(js.CorrPos, 2), f1(js.CorrLoad, 2))
	}

	w("\nSpread = the width holding the middle half of the loop band's power: a few Hz for one sharp line, tens to hundreds for a broad loop oscillation or flat noise; read it with the band RMS.\n")
	w("\nSharp torque lines >= %g Hz (peak >= %g x the local median power, amplitude >= %g permille, states with >= %d segments; amplitude is the sinusoid's peak; Q limited by the %.2f Hz bins is marked >=; coherence needs >= %d segments):\n\n",
		sp.LineMinHz, sp.LineOverFloor, sp.LineMinAmp, minLineSegments, sq.BinHz, minCoherenceSegments)
	w("| joint | state | Hz | amplitude permille | x floor | Q | coherence with other joints | verdict |\n|---|---|---|---|---|---|---|---|\n")
	for _, js := range sq.Joints {
		for i, l := range js.Lines {
			if i >= 6 {
				break
			}
			var cs []string
			for o, c := range l.Coherence {
				if !math.IsNaN(c) {
					cs = append(cs, fmt.Sprintf("%s %.2f", r.Config.Joints[o].Name, c))
				}
			}
			v := l.Verdict
			if l.Mains {
				v += "; 60 Hz harmonic"
			}
			w("| %s | %s | %s | %s | %s | %s | %s | %s |\n", js.Joint, l.State, f1(l.Hz, 1), f1(l.Amp, 2), f1(l.OverFloor, 0), qText(l), strings.Join(cs, ", "), v)
		}
	}

	if len(sq.Episodes) > 0 {
		w("\nBuzz episodes (%s torque over the threshold; per joint peak RMS across the episode shows who carries it):\n\n", loop)
		w("| joint | start | end | s | on fraction | peak RMS at | dominant Hz | onset state | own speed deg/s | position deg | every joint's peak RMS | strongest (x its own threshold) | nearest event |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
		for _, e := range sq.Episodes {
			var os []string
			for o, v := range e.Others {
				os = append(os, fmt.Sprintf("%s %s", r.Config.Joints[o].Name, f1(v, 0)))
			}
			w("| %s | %s | %s | %s | %s | %s %s | %s | %s | %s | %s | %s | %s | %s |\n", e.Joint, e.Start, e.End, f1(e.DurS, 1), f1(e.OnFraction, 2), f1(e.PeakRMS, 0), e.PeakAt,
				f1(e.FreqHz, 0), e.OnsetState, f1(e.OnsetSpeed, 1), f1(e.OnsetPosDeg, 1), strings.Join(os, ", "), e.Source+" ("+f1(e.SourceOver, 1)+")", e.Near)
		}
	} else {
		w("\nNo buzz episodes.\n")
	}

	w("\nTorque band RMS (permille) per state:\n\n| joint | state | s | segments |")
	for _, b := range sq.Bands {
		w(" %s |", bandLabel(b))
	}
	w(" %s | its peak Hz (spread Hz) | abs peak |\n|---|---|---|---|%s---|---|---|\n", loop, strings.Repeat("---|", len(sq.Bands)))
	for _, js := range sq.Joints {
		for _, st := range js.States {
			if st.Segments == 0 {
				continue
			}
			w("| %s | %s | %s | %d |", js.Joint, st.State, f1(st.Seconds, 1), st.Segments)
			for _, v := range st.TorqueRMS {
				w(" %s |", f1(v, 2))
			}
			w(" %s | %s (%s) | %s |\n", f1(st.LoopRMS, 2), f1(st.LoopPeakHz, 0), f1(st.LoopSpread, 0), f1(st.TorquePeak, 0))
		}
	}
	w("\nVelocity (deg/s) and residual (mdeg) band RMS per state:\n\n| joint | state |")
	for _, b := range sq.Bands {
		w(" vel %s |", bandLabel(b))
	}
	for _, b := range sq.Bands {
		w(" resid %s |", bandLabel(b))
	}
	w("\n|---|---|%s\n", strings.Repeat("---|", 2*len(sq.Bands)))
	for _, js := range sq.Joints {
		for _, st := range js.States {
			if st.Segments == 0 {
				continue
			}
			w("| %s | %s |", js.Joint, st.State)
			for _, v := range st.VelRMS {
				w(" %s |", f1(v, 4))
			}
			for _, v := range st.ResidRMS {
				w(" %s |", f1(v, 3))
			}
			w("\n")
		}
	}

	w("\n%s torque RMS by the joint's own commanded speed (timeline windows; median / p95 permille):\n\n| joint |", loop)
	labels := []string{speedBinLabel(0)}
	for i := 1; i <= len(speedBinEdges)+1; i++ {
		labels = append(labels, speedBinLabel(i))
	}
	for _, l := range labels {
		w(" %s |", l)
	}
	w("\n|---|%s\n", strings.Repeat("---|", len(labels)))
	for _, js := range sq.Joints {
		w("| %s |", js.Joint)
		for _, l := range labels {
			cell := "-"
			for _, b := range js.SpeedBins {
				if b.Label == l {
					cell = fmt.Sprintf("%s / %s (%d)", f1(b.Median, 1), f1(b.P95, 1), b.Windows)
				}
			}
			w(" %s |", cell)
		}
		w("\n")
	}

	if len(sq.Holds) > 0 {
		w("\nPer path hold, %s torque RMS (permille) per joint over the commanded motion, and the loudest joint's bands:\n\n| src | rev | at | axis | speed |", loop)
		for _, j := range r.Config.Joints {
			w(" %s |", j.Name)
		}
		w(" loudest (peak Hz) | its %s |\n|---|---|---|---|---|%s---|---|\n", strings.Join(bandLabels(sq.Bands), " / "), strings.Repeat("---|", len(r.Config.Joints)))
		for _, h := range sq.Holds {
			rev := fmt.Sprint(h.Rev)
			if h.Source != "olp" {
				rev = "-"
			}
			w("| %s | %s | %s | %s | %s %s |", h.Source, rev, h.At, h.Axis, f1(h.Speed, 0), h.Unit)
			worst := -1
			for j, v := range h.LoopRMS {
				w(" %s |", f1(v, 1))
				if !math.IsNaN(v) && (worst < 0 || v > h.LoopRMS[worst]) {
					worst = j
				}
			}
			if worst >= 0 {
				var bs []string
				for _, v := range h.TorqueRMS[worst] {
					bs = append(bs, f1(v, 1))
				}
				w(" %s (%s) | %s |\n", r.Config.Joints[worst].Name, f1(h.PeakHz[worst], 0), strings.Join(bs, " / "))
			} else {
				w(" - | - |\n")
			}
		}
	}
	for _, p := range plots {
		if strings.HasPrefix(p, "servo-") {
			w("\n![%s](%s)\n", p, p)
		}
	}
	pj, _ := json.Marshal(sp)
	w("\nServo detection settings: `%s`.\n", pj)
}

// writeServoTimeline writes the timeline as CSV, one row per window, so an
// episode can be followed window by window without decoding the dump again.
func (a *Analysis) writeServoTimeline(path string, sq *ServoQuality) error {
	var b strings.Builder
	b.WriteString("time,s,state")
	for _, j := range a.cfg.Joints {
		fmt.Fprintf(&b, ",%s_speed_deg_s,%s_position_deg,%s_torque_mean_permille", j.Name, j.Name, j.Name)
		for _, bd := range sq.Bands {
			fmt.Fprintf(&b, ",%s_torque_%s_rms", j.Name, bandKey(bd))
		}
		fmt.Fprintf(&b, ",%s_torque_loop_%s_rms", j.Name, bandKey(sq.LoopBand))
	}
	b.WriteString("\n")
	for _, p := range sq.tl {
		fmt.Fprintf(&b, "%s,%.3f,%s", a.stamp(a.wins[p.w], p.k), p.t, stateKeys[max(p.state, 0)])
		for _, tj := range p.j {
			fmt.Fprintf(&b, ",%.2f,%.3f,%.1f", tj.vAbs, tj.q, tj.load)
			for bi := range sq.Bands {
				v := nan()
				if bi < len(tj.band) {
					v = tj.band[bi]
				}
				fmt.Fprintf(&b, ",%s", f1(v, 2))
			}
			fmt.Fprintf(&b, ",%s", f1(tj.loop, 2))
		}
		b.WriteString("\n")
	}
	return os.WriteFile(path, []byte(b.String()), 0o644)
}

func bandLabels(bs [][2]float64) []string {
	var out []string
	for _, b := range bs {
		out = append(out, bandLabel(b))
	}
	return out
}

func qText(l ServoLine) string {
	if l.QLimited {
		return ">= " + f1(l.Q, 0)
	}
	return f1(l.Q, 0)
}
