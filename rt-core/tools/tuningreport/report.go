package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"reflect"
	"sort"
	"strconv"
	"strings"
)

const reportSchema = "rosie.tuning-report.v1"

type WindowInfo struct {
	First    string   `json:"first"`
	Last     string   `json:"last"`
	FirstSeq int64    `json:"first_sequence"`
	LastSeq  int64    `json:"last_sequence"`
	Records  int      `json:"records"`
	ArmedS   float64  `json:"armed_s"`
	Sources  []string `json:"sources"`
	Clock    Clock    `json:"clock"`
}

type Report struct {
	Schema       string           `json:"schema"`
	Generated    string           `json:"generated_utc"`
	TimeZone     string           `json:"time_zone"`
	Label        string           `json:"label,omitempty"`
	Sources      []SourceInfo     `json:"sources"`
	Journals     []string         `json:"journals"`
	Windows      []WindowInfo     `json:"windows"`
	Config       *CellConfig      `json:"config"`
	Params       Params           `json:"params"`
	Joints       []JointMetrics   `json:"joints"`
	Holds        []HoldMetrics    `json:"holds"`
	Trajectories []TrajMetrics    `json:"trajectories"`
	Servo        *ServoQuality    `json:"servo_quality"`
	Cycle        CycleMetrics     `json:"cycle"`
	LeadJournal  []TargetLeadLine `json:"journal_target_lead"`
	Observed     []ObserveLine    `json:"journal_drive_observe"`
	Scalars      []Scalar         `json:"scalars"`
	Baseline     *Baseline        `json:"baseline,omitempty"`
}

// Scalar is one comparable number. Better says which way is an improvement
// ("lower", "higher", "toward 1") so a tuning iteration's delta table can
// call it; "" means only "changed".
type Scalar struct {
	Key    string  `json:"key"`
	Label  string  `json:"label"`
	Unit   string  `json:"unit"`
	Value  float64 `json:"value"`
	Better string  `json:"better,omitempty"`
}

func (r *Report) buildScalars() {
	add := func(key, label, unit string, v float64, better string) {
		r.Scalars = append(r.Scalars, Scalar{key, label, unit, v, better})
	}
	for _, m := range r.Joints {
		p := "joint." + m.Joint + "."
		add(p+"lag_fit_ms", m.Joint+" lag (fit)", "ms", m.FitLagMs, "lower")
		add(p+"lag_best_shift_ms", m.Joint+" lag (best shift)", "ms", m.BestShiftMs, "lower")
		add(p+"fe_over_bound_max", m.Joint+" FE / scaled bound, max", "", m.FERatioMax, "lower")
		add(p+"fe_over_bound_longest_ms", m.Joint+" FE over bound, longest", "ms", m.FEOverLongestMs, "lower")
		add(p+"fe_peak_deg", m.Joint+" FE peak", "deg", m.PeakFEDeg, "lower")
		add(p+"lead_excursions", m.Joint+" target-lead excursions", "", float64(m.LeadExcursions), "lower")
		add(p+"lead_excursion_peak_deg", m.Joint+" target-lead excursion peak", "deg", m.LeadPeakDeg, "lower")
		add(p+"acc_actual_over_cmd", m.Joint+" actual / commanded peak acceleration", "", m.AccRatio, "toward 1")
		add(p+"jerk_cmd_20ms_max", m.Joint+" commanded jerk (20 ms)", "rad/s^3", m.Jerk20Max, "lower")
		add(p+"start_velocity_overshoot_pct", m.Joint+" velocity overshoot at starts", "%", m.VelOvershootPct, "lower")
		add(p+"stop_overshoot_worst_deg", m.Joint+" stop overshoot, worst", "deg", m.StopWorstOverDeg, "lower")
		add(p+"stop_settle_worst_ms", m.Joint+" stop settle, worst", "ms", m.StopWorstSettle, "lower")
		add(p+"ring_worst_peak_deg", m.Joint+" ring peak", "deg", m.Ring.WorstPeakDeg, "lower")
		add(p+"ring_episodes", m.Joint+" ring episodes", "", float64(len(m.Ring.Episodes)), "lower")
		add(p+"ring_spectrum_peak_hz", m.Joint+" ring frequency (spectrum)", "Hz", m.Ring.SpectrumPeakHz, "")
		add(p+"ring_median_damping", m.Joint+" ring damping ratio (major episodes)", "", m.Ring.MedianDamping, "higher")
		add(p+"ring_worst_settle_ms", m.Joint+" ring settle after worst peak", "ms", m.Ring.WorstSettleMs, "lower")
	}
	if len(r.Holds) > 0 {
		var off, ori, act, spread, n float64
		for _, h := range r.Holds {
			if h.Linear {
				off = math.Max(off, h.CmdOffMax)
				act = nanMax(act, h.ActOffSteady)
			}
			ori = math.Max(ori, h.CmdOriMax)
			spread = math.Max(spread, h.StopSpreadMs)
			n++
		}
		add("hold.count", "path holds", "", n, "")
		add("hold.cmd_off_max_mm", "commanded TCP off line, worst linear hold", "mm", off, "lower")
		add("hold.act_off_steady_max_mm", "actual TCP off line at steady speed, worst", "mm", act, "lower")
		add("hold.joint_stop_spread_max_ms", "joint stop spread, worst hold", "ms", spread, "lower")
	}
	for i, j := range r.Config.Joints {
		resid, ratio := 0.0, 0.0
		for _, t := range r.Trajectories {
			resid = math.Max(resid, t.ResidPeakDeg[i])
			ratio = math.Max(ratio, t.FERatioMax[i])
		}
		if len(r.Trajectories) > 0 {
			add("traj."+j.Name+".residual_peak_deg", j.Name+" Move-to residual peak", "deg", resid, "lower")
			add("traj."+j.Name+".fe_over_bound_max", j.Name+" Move-to FE / scaled bound", "", ratio, "lower")
		}
	}
	r.servoScalars(add)
	for _, c := range r.Cycle.Classes {
		add("cycle."+strings.ReplaceAll(c.Name, " ", "_")+".work_p99_us", c.Name+" work p99", "us", c.WorkP99, "lower")
	}
	add("cycle.overruns", "cycles with work over the period", "", float64(r.Cycle.Overruns), "lower")
	add("cycle.wkc_short", "cycles with short WKC", "", float64(r.Cycle.WkcShort), "lower")
}

// --- baseline delta

type BaselineRow struct {
	Key     string  `json:"key"`
	Label   string  `json:"label"`
	Unit    string  `json:"unit"`
	Base    float64 `json:"baseline"`
	Cur     float64 `json:"current"`
	Delta   float64 `json:"delta"`
	Verdict string  `json:"verdict"`
}

type Baseline struct {
	File    string        `json:"file"`
	Label   string        `json:"label,omitempty"`
	Machine string        `json:"machine_config_sha256"`
	Rows    []BaselineRow `json:"rows"`
	Better  int           `json:"better"`
	Worse   int           `json:"worse"`
}

// sameTolerance: a change smaller than 2 % of the larger magnitude is noise
// between two runs of the same test program, not a tuning effect.
const sameTolerance = 0.02

func loadBaseline(path string, cur []Scalar) (*Baseline, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var prev struct {
		Schema string `json:"schema"`
		Label  string `json:"label"`
		Config struct {
			MachineSHA256       string `json:"machine_config_sha256"`
			ConfigurationSHA256 string `json:"configuration_sha256"`
		} `json:"config"`
		Scalars []struct {
			Key   string   `json:"key"`
			Value *float64 `json:"value"`
		} `json:"scalars"`
	}
	if err = json.Unmarshal(b, &prev); err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	if prev.Schema != reportSchema {
		return nil, fmt.Errorf("%s: schema %q, want %q", path, prev.Schema, reportSchema)
	}
	base := map[string]float64{}
	for _, s := range prev.Scalars {
		v := math.NaN()
		if s.Value != nil {
			v = *s.Value
		}
		base[s.Key] = v
	}
	// Reports before the compiled configuration name their machine file.
	bl := &Baseline{File: path, Label: prev.Label, Machine: prev.Config.ConfigurationSHA256}
	if bl.Machine == "" {
		bl.Machine = prev.Config.MachineSHA256
	}
	for _, s := range cur {
		bv, ok := base[s.Key]
		if !ok {
			continue
		}
		row := BaselineRow{Key: s.Key, Label: s.Label, Unit: s.Unit, Base: bv, Cur: s.Value, Delta: s.Value - bv}
		row.Verdict = verdict(bv, s.Value, s.Better)
		switch row.Verdict {
		case "better":
			bl.Better++
		case "worse":
			bl.Worse++
		}
		bl.Rows = append(bl.Rows, row)
	}
	return bl, nil
}

func verdict(base, cur float64, better string) string {
	if math.IsNaN(base) || math.IsNaN(cur) {
		if math.IsNaN(base) && math.IsNaN(cur) {
			return "n/a"
		}
		return "not comparable"
	}
	d := cur - base
	if math.Abs(d) <= sameTolerance*math.Max(math.Abs(base), math.Abs(cur)) || d == 0 {
		return "same"
	}
	switch better {
	case "lower":
		if d < 0 {
			return "better"
		}
		return "worse"
	case "higher":
		if d > 0 {
			return "better"
		}
		return "worse"
	case "toward 1":
		// a ratio: below 1 the drive trails the command, above it overshoots
		if math.Abs(cur-1) < math.Abs(base-1) {
			return "better"
		}
		return "worse"
	}
	return "changed"
}

// --- JSON: field order kept, NaN/Inf written as null ("not measurable"),
// never as 0, so a missing measurement cannot read as a perfect one.

func writeJSON(path string, v any) error {
	var b bytes.Buffer
	encodeValue(&b, reflect.ValueOf(v), 0)
	b.WriteByte('\n')
	return os.WriteFile(path, b.Bytes(), 0o644)
}

func encodeValue(b *bytes.Buffer, v reflect.Value, ind int) {
	pad := func(n int) string { return "\n" + strings.Repeat("  ", n) }
	switch v.Kind() {
	case reflect.Pointer, reflect.Interface:
		if v.IsNil() {
			b.WriteString("null")
			return
		}
		encodeValue(b, v.Elem(), ind)
	case reflect.Struct:
		t := v.Type()
		b.WriteByte('{')
		first := true
		for i := 0; i < t.NumField(); i++ {
			f := t.Field(i)
			if !f.IsExported() {
				continue
			}
			tag := f.Tag.Get("json")
			name, opts, _ := strings.Cut(tag, ",")
			if name == "-" {
				continue
			}
			if name == "" {
				name = f.Name
			}
			fv := v.Field(i)
			if strings.Contains(opts, "omitempty") && fv.IsZero() {
				continue
			}
			if (fv.Kind() == reflect.Slice || fv.Kind() == reflect.Map) && strings.Contains(opts, "omitempty") && fv.Len() == 0 {
				continue
			}
			if !first {
				b.WriteByte(',')
			}
			first = false
			b.WriteString(pad(ind + 1))
			nb, _ := json.Marshal(name)
			b.Write(nb)
			b.WriteString(": ")
			encodeValue(b, fv, ind+1)
		}
		if !first {
			b.WriteString(pad(ind))
		}
		b.WriteByte('}')
	case reflect.Slice, reflect.Array:
		if v.Kind() == reflect.Slice && v.IsNil() {
			b.WriteString("[]")
			return
		}
		simple := v.Len() == 0 || isScalarKind(v.Index(0).Kind())
		b.WriteByte('[')
		for i := 0; i < v.Len(); i++ {
			if i > 0 {
				b.WriteByte(',')
				if simple {
					b.WriteByte(' ')
				}
			}
			if !simple {
				b.WriteString(pad(ind + 1))
			}
			encodeValue(b, v.Index(i), ind+1)
		}
		if !simple && v.Len() > 0 {
			b.WriteString(pad(ind))
		}
		b.WriteByte(']')
	case reflect.Map:
		keys := v.MapKeys()
		sort.Slice(keys, func(i, j int) bool { return keys[i].String() < keys[j].String() })
		b.WriteByte('{')
		for i, k := range keys {
			if i > 0 {
				b.WriteByte(',')
			}
			b.WriteString(pad(ind + 1))
			nb, _ := json.Marshal(k.String())
			b.Write(nb)
			b.WriteString(": ")
			encodeValue(b, v.MapIndex(k), ind+1)
		}
		if len(keys) > 0 {
			b.WriteString(pad(ind))
		}
		b.WriteByte('}')
	case reflect.Float32, reflect.Float64:
		f := v.Float()
		if math.IsNaN(f) || math.IsInf(f, 0) {
			b.WriteString("null")
			return
		}
		b.WriteString(strconv.FormatFloat(f, 'g', 7, 64))
	default:
		nb, _ := json.Marshal(v.Interface())
		b.Write(nb)
	}
}

func isScalarKind(k reflect.Kind) bool {
	switch k {
	case reflect.Struct, reflect.Slice, reflect.Array, reflect.Map, reflect.Pointer, reflect.Interface:
		return false
	}
	return true
}

// --- markdown

func f1(v float64, prec int) string {
	if math.IsNaN(v) || math.IsInf(v, 0) {
		return "-"
	}
	return strconv.FormatFloat(v, 'f', prec, 64)
}

func (r *Report) markdown(plots []string) string {
	var b strings.Builder
	w := func(format string, args ...any) { fmt.Fprintf(&b, format, args...) }
	title := "Joint tuning report"
	if r.Label != "" {
		title += ": " + r.Label
	}
	w("# %s\n\n", title)
	w("Generated %s by `rt-core/tools/tuningreport` (schema `%s`). Times are %s. Angles in degrees, jerk in rad/s^3.\n\n", r.Generated, r.Schema, r.TimeZone)
	w("## Inputs\n\n")
	w("| dump | kind | protocol | records | missing | trigger | sha256 |\n|---|---|---|---|---|---|---|\n")
	for _, s := range r.Sources {
		w("| `%s` | %s | %d.%d | %d | %d | %s | `%.16s` |\n", s.File, s.Kind, s.ProtocolMajor, s.ProtocolMinor, s.Records, s.Missing, s.Trigger, s.SHA256)
	}
	for _, s := range r.Sources {
		if s.Note != "" {
			w("\n- `%s`: %s", s.File, s.Note)
		}
	}
	w("\n\n| window | from | to | records | armed s | clock | uncertainty |\n|---|---|---|---|---|---|---|\n")
	for i, win := range r.Windows {
		w("| %d | %s | %s | %d | %s | %s | %s |\n", i+1, win.First, win.Last, win.Records, f1(win.ArmedS, 1), win.Clock.Method, win.Clock.Uncertainty)
	}
	if len(r.Journals) > 0 {
		w("\nJournals: %s.\n", "`"+strings.Join(r.Journals, "`, `")+"`")
	}
	c := r.Config
	w("\n## Configuration (every bound below is from it)\n\n")
	source := fmt.Sprintf("machine config `%s` (sha256 `%.16s`) compiled by rtctl", c.MachinePath, c.MachineSHA256)
	if c.CompiledPath != "" {
		source = fmt.Sprintf("compiled directory `%s`", c.CompiledPath)
	}
	w("- Compiled configuration `%.16s` from %s; cell pin `%.16s`; robot description `%s` `%.23s`.\n", c.ConfigurationSHA256, source, c.PinnedConfiguration, c.DescriptionPath, c.DescriptionSHA256)
	for _, n := range c.Notes {
		w("- **Note:** %s.\n", n)
	}
	w("- Tracking lag allowance %s ms (following-error and target-lead bounds grow by |v_cmd| x allowance); jog_jerk %s rad/s^3.\n\n", f1(c.LagAllowanceS*1e3, 1), f1(c.JogJerk, 1))
	w("| joint | gear | FE static deg | target lead static deg | completion tol deg | accel limit deg/s^2 | velocity limit deg/s | drive settings |\n|---|---|---|---|---|---|---|---|\n")
	for _, j := range c.Joints {
		settings, _ := json.Marshal(j.DriveSettings)
		w("| %s | %s | %s | %s | %s | %s | %s | `%s` |\n", j.Name, f1(j.GearRatio, 2), f1(j.FollowingError*deg, 4), f1(j.MaxTargetLead*deg, 4), f1(j.CompletionTol*deg, 4), f1(j.AccelerationLim*deg, 0), f1(j.VelocityLimit*deg, 1), settings)
	}
	if len(r.Observed) > 0 {
		w("\nDrive values read back at startup (RevPi journal `startup observe`):\n\n| axis | key | value |\n|---|---|---|\n")
		for _, o := range r.Observed {
			w("| %d | %s | %s |\n", o.Axis, o.Key, o.Value)
		}
	}
	if r.Baseline != nil {
		bl := r.Baseline
		w("\n## Change against baseline\n\nBaseline `%s`%s (configuration `%.16s`): **%d better, %d worse**; changes within %.0f %% are \"same\". Compare runs of the same test program only.\n\n",
			bl.File, map[bool]string{true: " (" + bl.Label + ")", false: ""}[bl.Label != ""], bl.Machine, bl.Better, bl.Worse, sameTolerance*100)
		w("| metric | unit | baseline | current | delta | verdict |\n|---|---|---|---|---|---|\n")
		for _, row := range bl.Rows {
			if row.Verdict == "n/a" {
				continue
			}
			v := row.Verdict
			if v == "better" || v == "worse" {
				v = "**" + v + "**"
			}
			w("| %s | %s | %s | %s | %s | %s |\n", row.Label, row.Unit, f1(row.Base, 4), f1(row.Cur, 4), f1(row.Delta, 4), v)
		}
	}
	w("\n## Per joint\n\n")
	w("Lag: fit = least-squares slope of following error on commanded speed; shift = delay minimising |actual - commanded(t - shift)|; median = median FE / speed above %g deg/s. ", r.Params.MedianMinSpeedDeg)
	w("FE = commanded[t-1] - actual[t] against following_error + |v| x allowance; lead = commanded[t] - actual[t] against max_target_lead + |v| x allowance (native's check, a warning).\n\n")
	w("| joint | lag fit ms (R2) | shift ms | median ms | FE peak deg | FE/bound max | over bound longest / total ms | drive FE peak deg | lead excursions (peak deg) |\n|---|---|---|---|---|---|---|---|---|\n")
	for _, m := range r.Joints {
		w("| %s | %s (%s) | %s | %s | %s @%s deg/s | %s at %s | %s / %s | %s | %d (%s) |\n", m.Joint, f1(m.FitLagMs, 2), f1(m.FitR2, 3), f1(m.BestShiftMs, 0), f1(m.MedianLagMs, 1),
			f1(m.PeakFEDeg, 3), f1(m.PeakFEAtVDeg, 1), f1(m.FERatioMax, 3), m.FERatioMaxAt, f1(m.FEOverLongestMs, 0), f1(m.FEOverTotalMs, 0), f1(m.DriveFEPeakDeg, 3), m.LeadExcursions, f1(m.LeadPeakDeg, 3))
	}
	w("\n| joint | cmd acc peak deg/s^2 (x limit) | actual acc peak deg/s^2 (x cmd) | jerk 1-cycle / 20 ms rad/s^3 (20 ms x jog_jerk) | 1-cycle over jog_jerk | start overshoot %% (deg/s, plateaus) | stops | stop overshoot worst deg | stop settle worst ms |\n|---|---|---|---|---|---|---|---|---|\n")
	for _, m := range r.Joints {
		w("| %s | %s (%s) | %s (%s) | %s / %s (%s) | %d cycles | %s (%s, %d) | %d | %s | %s |\n", m.Joint, f1(m.CmdAccPeakDeg, 0), f1(m.AccLimitUse, 2), f1(m.ActAccPeakDeg, 0), f1(m.AccRatio, 2),
			f1(m.JerkCycleMax, 0), f1(m.Jerk20Max, 0), f1(m.Jerk20Over, 2), m.JerkOverCycles, f1(m.VelOvershootPct, 1), f1(m.VelOvershootDeg, 2), m.Plateaus, len(m.Stops), f1(m.StopWorstOverDeg, 4), f1(m.StopWorstSettle, 0))
	}
	w("\n## Resonance\n\nResidual = actual(t) - commanded(t - shift) with the joint's own lag removed. Episodes (ringing, or transient tracking error when no frequency is measurable) start above %g x completion_tolerance and settle when inside completion_tolerance for %d ms. Damping ratio from the log decrement of half-cycle peaks after the episode peak (needs 3 half cycles above half the tolerance).\n\n",
		r.Params.RingTriggerFactor, r.Params.RingQuietMs)
	w("| joint | shift ms | spectrum peak Hz (amp deg) | episodes | worst peak deg at | worst f Hz | worst zeta | worst settle ms | major episodes f Hz | major episodes zeta |\n|---|---|---|---|---|---|---|---|---|---|\n")
	for _, m := range r.Joints {
		g := m.Ring
		w("| %s | %s | %s (%s) | %d | %s %s | %s | %s | %s | %s | %s |\n", m.Joint, f1(g.ShiftMs, 0), f1(g.SpectrumPeakHz, 2), f1(g.SpectrumAmpDeg, 4), len(g.Episodes), f1(g.WorstPeakDeg, 3), g.WorstAt,
			f1(g.WorstFreqHz, 2), f1(g.WorstDamping, 3), f1(g.WorstSettleMs, 0), f1(g.MedianFreqHz, 2), f1(g.MedianDamping, 3))
	}
	for _, m := range r.Joints {
		eps := append([]RingEpisode(nil), m.Ring.Episodes...)
		if len(eps) == 0 {
			continue
		}
		sort.Slice(eps, func(i, j int) bool { return math.Abs(eps[i].PeakDeg) > math.Abs(eps[j].PeakDeg) })
		if len(eps) > 8 {
			eps = eps[:8]
		}
		w("\n%s largest episodes:\n\n| peak at | peak deg | settle ms | f zc / dft Hz | zeta | joint speed deg/s | FE/bound | excited by (deg/s^2) | deg per deg/s^2 | nearest event |\n|---|---|---|---|---|---|---|---|---|---|\n", m.Joint)
		for _, e := range eps {
			w("| %s | %+.3f | %s | %s / %s | %s | %s | %s | %s %s | %s | %s |\n", e.PeakAt, e.PeakDeg, f1(e.SettleMs, 0), f1(e.FreqZCHz, 2), f1(e.FreqDFTHz, 2), f1(e.Damping, 3), f1(e.CmdVelDeg, 1),
				f1(e.FERatioPeak, 2), e.ExciteJoint, f1(e.ExciteAccDeg, 0), strconv.FormatFloat(e.SensitivityDeg, 'e', 2, 64), e.Near)
		}
	}
	if len(r.Holds) > 0 {
		w("\n## Cartesian path holds\n\nCommanded = native's joint targets through the resolver's kinematics; actual = feedback. Linear: off = TCP distance from the commanded line, orient = rotation from start. Rotational: off = TCP drift, orient = rotation off the commanded axis. Inferred holds (no OLP line) measure off the start-to-stop chord.\n\n")
		w("| src | rev | at | axis | speed | travel | cmd off max / RMS mm | cmd orient deg | act off steady / max mm | stop v, dist, ms | joint stop spread ms | acc max rad/s^2 | jerk 1-cycle / 20 ms | worst FE/bound | governing |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
		for _, h := range r.Holds {
			worst, wj := 0.0, ""
			for j, v := range h.FERatioMax {
				if v > worst {
					worst, wj = v, r.Config.Joints[j].Name
				}
			}
			trip := ""
			if h.Tripped {
				trip = " **disarmed**"
			}
			rev, speed := strconv.Itoa(h.Rev), f1(h.SpeedReq, 0)
			if h.Source != "olp" {
				rev, speed = "-", "peak "+f1(h.VMax, 0)
			}
			w("| %s | %s | %s | %s %s | %s %s | %s | %s / %s | %s | %s / %s | %s, %s, %s%s | %s | %s (%s) | %s / %s (%s) | %s %s | %s |\n", h.Source, rev, h.At, h.Frame, h.Axis, speed, h.Unit, f1(h.Travel, 1),
				f1(h.CmdOffMax, 4), f1(h.CmdOffRMS, 4), f1(h.CmdOriMax, 4), f1(h.ActOffSteady, 2), f1(h.ActOffMax, 2), f1(h.VStop, 0), f1(h.StopDist, 1), f1(h.StopMs, 0), trip,
				f1(h.StopSpreadMs, 0), f1(h.AccMax, 2), h.AccJoint, f1(h.JerkMax, 0), f1(h.Jerk20Max, 0), h.Jerk20Joint, wj, f1(worst, 2), h.Governing)
		}
	}
	if len(r.Trajectories) > 0 {
		w("\n## Move-to / trajectories\n\n| traj | at | ms | straightness deg (counts) | acc x limit | jerk20 x jog_jerk | settle after end ms |\n|---|---|---|---|---|---|---|\n")
		for _, t := range r.Trajectories {
			w("| %d | %s | %s | %s (%s) | %s | %s | %s |\n", t.Traj, t.At, f1(t.DurMs, 0), f1(t.StraightDeg, 6), f1(t.StraightCount, 1), f1(t.AccLimitUse, 2), f1(t.JerkLimitUse, 2), f1(t.SettleMs, 0))
		}
		w("\n| traj | joint | dq deg | peak vel deg/s | peak acc deg/s^2 | jerk 1-cycle / 20 ms | residual peak deg | FE/bound max | over bound ms |\n|---|---|---|---|---|---|---|---|---|\n")
		for _, t := range r.Trajectories {
			for j, jc := range r.Config.Joints {
				w("| %d | %s | %s | %s | %s | %s / %s | %s | %s | %s |\n", t.Traj, jc.Name, f1(t.DqDeg[j], 2), f1(t.PeakVelDeg[j], 1), f1(t.PeakAccDeg[j], 0), f1(t.Jerk1[j], 0), f1(t.Jerk20[j], 0), f1(t.ResidPeakDeg[j], 3), f1(t.FERatioMax[j], 3), f1(t.FEOverMs[j], 0))
			}
		}
	}
	r.servoMarkdown(w, plots)
	cy := r.Cycle
	w("\n## Cycle cost (servo host)\n\n| state | cycles | work p50 / p99 / p99.9 / max us | jitter p50 / p99 / max us |\n|---|---|---|---|\n")
	for _, k := range cy.Classes {
		w("| %s | %d | %s / %s / %s / %s | %s / %s / %s |\n", k.Name, k.Cycles, f1(k.WorkP50, 1), f1(k.WorkP99, 1), f1(k.WorkP999, 1), f1(k.WorkMax, 1), f1(k.JitterP50, 1), f1(k.JitterP99, 1), f1(k.JitterMax, 1))
	}
	w("\nOverruns (work > period) %d; WKC short %d (values %v, expected %v); windows split by sequence gaps %d; missing records in dumps %d; safety fault masks %v; execution fault reasons %v.\n",
		cy.Overruns, cy.WkcShort, cy.WkcValues, cy.WkcExpect, cy.SeqGaps, cy.Missing, cy.Safety, cy.Execution)
	if len(r.LeadJournal) > 0 {
		w("\n## Target-lead warnings in the RevPi journal\n\n| utc | axis | kind | lead deg | limit deg | v deg/s | duration ms |\n|---|---|---|---|---|---|---|\n")
		for _, l := range r.LeadJournal {
			kind := "warning"
			if l.Cleared {
				kind = "cleared (peak)"
			}
			w("| %s | %d | %s | %s | %s | %s | %s |\n", l.UTC, l.Axis, kind, f1(l.LeadDeg, 4), f1(l.LimitDeg, 4), f1(l.VelDegS, 3), f1(l.DurMs, 0))
		}
	}
	if len(plots) > 0 {
		w("\n## Plots\n\n")
		for _, p := range plots {
			if !strings.HasPrefix(p, "servo-") {
				w("![%s](%s)\n\n", p, p)
			}
		}
	}
	pj, _ := json.Marshal(r.Params)
	w("## Method notes\n\n- Decoded on the analysis host from copied dumps; nothing runs on the servo host.\n- Analysis parameters (data selection, not bounds): `%s`.\n- Metrics are only comparable between iterations that ran the same test program; the delta table cannot tell a gain change from a different motion.\n", pj)
	return b.String()
}
