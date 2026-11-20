package main

import (
	"fmt"
	"math"
	"path/filepath"
	"strings"
)

func pm(v, ci float64, prec int) string {
	if math.IsNaN(ci) {
		return fmt.Sprintf("%.*f (one estimate, no interval)", prec, v)
	}
	return fmt.Sprintf("%.*f +- %.*f", prec, v, prec, ci)
}

func (r *InertiaReport) markdown(plots []string) string {
	var b strings.Builder
	w := func(f string, a ...any) { fmt.Fprintf(&b, f, a...) }
	title := "designed test"
	if r.Opportunistic {
		title = "OPPORTUNISTIC (ordinary motion, not the designed test)"
	}
	w("# %s load inertia: %s\n\n", r.Joint, title)
	if r.Label != "" {
		w("%s. ", r.Label)
	}
	w("Generated %s by `tools/tuningreport inertia`. Configuration `%.12s`", r.Generated, r.Config.ConfigurationSHA256)
	if r.Config.PinnedConfiguration != "" {
		w(" (cell pin `%.12s`)", r.Config.PinnedConfiguration)
	}
	w(".\n\n")
	m := r.Motor.Motor
	verified := "UNVERIFIED assignment"
	if r.Motor.Verified {
		verified = "verified"
	}
	w("Motor %s (%s, `%s`): rated torque %.3g N.m (0x6077 = 1000 permille), rotor inertia %.3g kg.m^2 (C00.06 = 0 %%). %s\n\n", m.Model, verified, r.Motor.File, m.RatedTorqueNm, m.RotorInertiaKgM2, m.Source)
	for _, n := range r.Notes {
		w("> NOTE: %s\n\n", n)
	}
	if r.Plan != nil {
		w("Program `%s` (%s): window %.2f..%.2f deg, levels", r.Plan.Program, r.Plan.TrajectoryDigest, r.Plan.WindowDeg[0], r.Plan.WindowDeg[1])
		for _, l := range r.Plan.Levels {
			w(" %.0f", l.AccelDeg)
		}
		w(" deg/s^2, peak %.1f deg/s, jerk %.0f rad/s^3, %d repeats.\n\n", r.Plan.PeakSpeedDeg, r.Plan.JerkRadS3, r.Plan.Repeats)
	}
	w("## Result\n\n")
	w("| method | J_total motor side (kg.m^2) | load, motor side (kg.m^2) | load, joint side (kg.m^2) | C00.06 (%%) | n |\n|---|---|---|---|---|---|\n")
	for _, e := range []*InertiaEstimate{r.Fit, r.Vendor, r.UpDown} {
		if e == nil {
			continue
		}
		w("| %s | %s | %.4g | %.4g | **%s** | %d |\n", e.Method, pm(e.JTotal*1e4, e.JTotalCI95*1e4, 3)+" e-4", e.JLoad, e.JLoadJointSide, pm(e.C0006, e.C0006CI95, 0), e.N)
	}
	w("\nIntervals are 95 %%: the fit's from its residuals (t, %d dof), the differentials' from the spread of their pairs. They exclude the motor data's own uncertainty (rated torque and rotor inertia are nameplate values).\n\n", r.FitDOF)
	if r.ConfiguredPct != nil {
		w("Configured C00.06 on %s: %.0f %% (this configuration writes it). ", r.Joint, *r.ConfiguredPct)
	} else {
		w("This configuration does not write C00.06 on %s (the drive keeps its own value). ", r.Joint)
	}
	w("The torque measures the arm, not the setting: every session's estimate should agree whatever C00.06 was.\n\n")
	w("## Fit\n\n")
	mode := "constant-acceleration and cruise segments (the designed test: two or more levels)"
	if r.Mode == "ramp" {
		mode = "whole speed changes (momentum balance: mean torque = J dv/T + mean friction + mean gravity, any ramp shape) and cruise segments, because fewer than two constant-acceleration levels were found"
	}
	w("Weighted least squares over %d %s (weight = duration), torque = J' alpha_motor + terms below; R^2 %.4f, residual RMS %.2f permille. Position terms are about the fitted segments' mean position %.2f deg; gravity of one joint with the others still is A cos q + B sin q, which c0 + g1 dq + g2 dq^2 follows to third order in the window.\n\n",
		r.FitN, mode, r.FitR2, r.FitRMS, r.qRef*deg)
	w("| term | value | 95 %% | unit |\n|---|---|---|---|\n")
	for _, t := range r.Terms {
		w("| %s | %.5g | %.2g | %s |\n", t.Name, t.Value, t.CI95, t.Unit)
	}
	w("\n## Differential pairs\n\n")
	if len(r.Pairs) == 0 {
		w("None: no two groups in one direction share a speed band of %.0f deg/s or more.\n\n", r.Params.MinBandDeg)
	} else {
		w("Same direction, common speed band: friction cancels. `vendor` = one phase at two levels (STEPPERONLINE's method); `up/down` = one level's speed-up against its slow-down (a whole move apart, so the gravity correction carries it). J in 1e-4 kg.m^2.\n\n")
		w("| type | dir | A | B | band deg/s | n A/B | torque A/B permille | motor accel A/B rad/s^2 | J raw | J gravity-corrected |\n|---|---|---|---|---|---|---|---|---|---|\n")
		for _, p := range r.Pairs {
			w("| %s | %+d | %s | %s | %.1f-%.1f | %d/%d | %.1f / %.1f | %.1f / %.1f | %.3f | %.3f |\n", p.Type, p.Dir, p.A, p.B, p.BandDeg[0], p.BandDeg[1], p.SegmentsA, p.SegmentsB,
				p.TorqueA, p.TorqueB, p.MotorAccA, p.MotorAccB, p.JRaw*1e4, p.JGravityCorr*1e4)
		}
		w("\n")
	}
	w("## Segments\n\n")
	w("Levels (|commanded| deg/s^2):")
	for _, l := range r.Levels {
		w(" %.1f", l)
	}
	w(" `ramp` acceleration is the mean, dv/T. Unused segments carry the fit's prediction residual.\n\n| # | at | kind | used | phase | dir | level | ms | cmd acc deg/s^2 | actual acc | speed deg/s | q mean deg | torque permille (sd, |peak|) | residual |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
	for i, s := range r.Segments {
		lvl := "-"
		if s.Level >= 0 {
			lvl = fmt.Sprintf("L%d", s.Level+1)
		}
		used := ""
		if s.Used {
			used = "yes"
		}
		w("| %d | %s | %s | %s | %s | %+d | %s | %.0f | %.1f | %.1f | %.1f-%.1f | %.2f | %.1f (%.1f, %.0f) | %+.2f |\n", i+1, s.At, s.Kind, used, s.Phase, s.Dir, lvl, s.DurMs,
			s.CmdAccDeg, s.ActAccDeg, s.VMinDeg, s.VMaxDeg, s.QMeanDeg, s.TorqueMean, s.TorqueSD, s.TorquePeak, s.ResidualPm)
	}
	w("\n")
	for _, p := range plots {
		w("![%s](%s)\n\n", p, p)
	}
	w("## Method notes\n\n")
	pr := r.Params
	w("- Segments: the joint's commanded acceleration (centred difference of the 1 kHz target velocity) constant within %.1f deg/s^2 or %.0f %% for >= %d ms, every other joint commanded under %.2f deg/s, all joints enabled, 0x6077 valid; the first %d ms after each jerk phase dropped (>= %d ms kept); acceleration segments >= %.0f deg/s^2, cruise segments at >= %.0f deg/s; every cycle >= %.0f deg/s so sign(v) is defined.\n",
		pr.AccTolAbsDeg, pr.AccTolRel*100, pr.MinRunMs, pr.StillDeg, pr.TrimLeadMs, pr.MinKeptMs, pr.MinAccDeg, pr.CruiseMinDeg, pr.SignMinDeg)
	w("- The regressor is the actual acceleration, (v_end - v_start) / T from 0x606C with each end averaged over +-%d cycles: the mean acceleration over the span without differentiation noise. Motor side = joint x gear x sign (the drive's count direction).\n", pr.EndAvgCyc)
	w("- C00.06 = (J_total / J_rotor - 1) x 100 %% (A6-EC manual PDF p.234). J_total = J' x rated torque / 1000.\n")
	w("- Sources: %d dump(s)", len(r.Sources))
	for _, s := range r.Sources {
		w("; `%s` (%.12s)", s.File, s.SHA256)
	}
	w(".\n")
	return b.String()
}

func meanQ(s []InertiaSegment) float64 {
	var q []float64
	for _, x := range s {
		q = append(q, x.QMeanDeg)
	}
	return mean(q)
}

// plots writes inertia-fit.png (inertial torque against motor acceleration,
// and residuals) and inertia-timeline.png (torque and model over each
// window holding segments).
func (r *InertiaReport) plots(a *Analysis, ji int, dir string) ([]string, error) {
	var names []string
	{
		W, H := 1000, 360
		c := newCanvas(W, 2*H)
		var xs [4][]float64
		var ys [4][]float64
		var rx, ry []float64
		for i, s := range r.Segments {
			if !s.Used {
				continue
			}
			nonInertial := s.TorqueMean - s.ResidualPm - r.term("inertia")*s.MotorAcc
			g := 0
			if s.Dir < 0 {
				g = 1
			}
			if s.Kind == "cruise" {
				g += 2
			}
			xs[g] = append(xs[g], s.MotorAcc)
			ys[g] = append(ys[g], s.TorqueMean-nonInertial)
			rx, ry = append(rx, float64(i+1)), append(ry, s.ResidualPm)
		}
		lo, hi := math.Inf(1), math.Inf(-1)
		for _, s := range r.Segments {
			if s.Used {
				lo, hi = math.Min(lo, s.MotorAcc), math.Max(hi, s.MotorAcc)
			}
		}
		jp := r.term("inertia")
		ss := []series{{name: "+dir " + r.Mode, x: xs[0], y: ys[0], col: 0, mark: true}, {name: "-dir " + r.Mode, x: xs[1], y: ys[1], col: 1, mark: true},
			{name: "+dir cruise", x: xs[2], y: ys[2], col: 2, mark: true}, {name: "-dir cruise", x: xs[3], y: ys[3], col: 3, mark: true},
			{name: "fit", x: []float64{lo, hi}, y: []float64{jp * lo, jp * hi}, col: 7}}
		e := r.Fit
		c.panel(0, 0, W, H, panelSpec{title: fmt.Sprintf("%s: torque minus fitted friction, gravity and offset, per segment; slope J' = %.4g permille/(rad/s^2): J_total %.3g kg.m^2, C00.06 %.0f +- %.0f %%",
			r.Joint, jp, e.JTotal, e.C0006, e.C0006CI95), xl: "motor accel rad/s^2", yl: "permille", ss: ss})
		c.panel(0, H, W, H, panelSpec{title: fmt.Sprintf("fit residual per segment (RMS %.2f permille)", r.FitRMS), xl: "segment #", yl: "permille",
			ss: []series{{x: rx, y: ry, col: 4, mark: true}}, hlines: []float64{0}})
		names = append(names, "inertia-fit.png")
		if err := savePNG(filepath.Join(dir, "inertia-fit.png"), c); err != nil {
			return names, err
		}
	}
	// timeline: smoothed torque against the fitted model, per window with segments
	byWin := map[int][2]int{}
	for _, s := range r.Segments {
		span, ok := byWin[s.Window]
		if !ok {
			span = [2]int{s.from, s.to}
		}
		span[0], span[1] = min(span[0], s.from), max(span[1], s.to)
		byWin[s.Window] = span
	}
	var wins []int
	for wi := range byWin {
		wins = append(wins, wi)
	}
	if len(wins) == 0 {
		return names, nil
	}
	for i := 1; i < len(wins); i++ {
		for k := i; k > 0 && wins[k] < wins[k-1]; k-- {
			wins[k], wins[k-1] = wins[k-1], wins[k]
		}
	}
	if len(wins) > 6 {
		wins = wins[:6]
	}
	W, H := 1600, 260
	c := newCanvas(W, 3*H*len(wins))
	jc := r.Config.Joints[ji]
	motor := jc.GearRatio * jc.Sign
	flip := 1.0
	if r.TorqueFlipped {
		flip = -1
	}
	qRef := r.qRef
	for pi, wi := range wins {
		w := a.wins[wi]
		span := byWin[wi]
		lo, hi := max(0, span[0]-1000), min(w.n, span[1]+1000)
		sm, sd := r.Params.SmoothCyc[0], r.Params.SmoothCyc[1]
		tq := movingMean(w.tq[ji][lo:hi], sm)
		tq = movingMean(tq, sd)
		vs := movingMean(w.va[ji][lo:hi], sm)
		var x, tqs, pred, acmd, aact, vel, inSeg []float64
		step := decimate(hi-lo, 4000)
		for k := 0; k < hi-lo; k += step {
			q := lo + k
			x = append(x, float64(q-span[0])*w.dt)
			tqs = append(tqs, flip*tq[k])
			aa := math.NaN()
			if k-sd/2 >= 0 && k+sd/2 < len(vs) {
				aa = (vs[k+sd/2] - vs[k-sd/2]) / (float64(sd) * w.dt)
			}
			v := w.va[ji][q] * motor
			sg := 1.0
			if v < 0 {
				sg = -1
			}
			dq := w.qa[ji][q] - qRef
			pr := r.term("inertia")*aa*motor + r.term("coulomb")*sg + r.term("viscous")*v + r.term("offset") + r.term("gravity_1")*dq + r.term("gravity_2")*dq*dq
			pred = append(pred, pr)
			ac := 0.0
			if q > 0 && q+1 < w.n {
				ac = (w.vc[ji][q+1] - w.vc[ji][q-1]) / (2 * w.dt) * deg
			}
			acmd, aact, vel = append(acmd, ac), append(aact, aa*deg), append(vel, w.vc[ji][q]*deg)
			in := math.NaN()
			for _, s := range r.Segments {
				if s.Used && s.Window == wi && q >= s.from && q < s.to {
					in = 0
				}
			}
			inSeg = append(inSeg, in)
		}
		y0 := pi * 3 * H
		c.panel(0, y0, W, H, panelSpec{title: fmt.Sprintf("%s window %d from %s: torque (%d+%d-cycle means) and the fitted model; s from the first segment", r.Joint, wi, a.stamp(w, span[0]), sm, sd),
			xl: "s", yl: "permille", ss: []series{{name: "0x6077", x: x, y: tqs, col: 0}, {name: "model", x: x, y: pred, col: 1}}})
		c.panel(0, y0+H, W, H, panelSpec{title: "acceleration: commanded and actual (smoothed 0x606C); black = segments used", xl: "s", yl: "deg/s^2",
			ss: []series{{name: "commanded", x: x, y: acmd, col: 2}, {name: "actual", x: x, y: aact, col: 3}, {name: "used", x: x, y: inSeg, col: 7}}})
		c.panel(0, y0+2*H, W, H, panelSpec{title: "commanded speed", xl: "s", yl: "deg/s", ss: []series{{x: x, y: vel, col: 4}}})
	}
	names = append(names, "inertia-timeline.png")
	return names, savePNG(filepath.Join(dir, "inertia-timeline.png"), c)
}

// movingMean is a centred moving mean of n samples (shorter at the ends).
func movingMean(x []float64, n int) []float64 {
	out := make([]float64, len(x))
	if n < 2 {
		copy(out, x)
		return out
	}
	h := n / 2
	pre := make([]float64, len(x)+1)
	for i, v := range x {
		pre[i+1] = pre[i] + v
	}
	for i := range x {
		a, b := max(0, i-h), min(len(x), i-h+n)
		out[i] = (pre[b] - pre[a]) / float64(b-a)
	}
	return out
}
