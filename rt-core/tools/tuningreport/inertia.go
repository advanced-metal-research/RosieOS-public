package main

import (
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"time"
)

// Load inertia from torque against acceleration (STEPPERONLINE's
// dual-acceleration differential, 2026-10-09, and a least-squares fit over
// every level). In a constant-acceleration phase of one joint, with the
// others still, the drive's torque is
//
//	T = J_total * alpha_motor + F(v) + G(q)
//
// J_total the motor-side inertia (rotor plus the load reflected through the
// gear), F friction (Coulomb sign(v) plus viscous v), G gravity. With the
// other joints fixed, the gravity torque of a single joint is exactly
// A cos q + B sin q; over the test window it is fitted as c0 + g1 dq + g2 dq^2.
// Two levels sweeping the same speed band in the same direction cancel F
// (vendor's method); the fit separates F and G with every segment.
// C00.06 = (J_total / J_rotor - 1) x 100 % (A6-EC manual PDF p.234: 0 = no
// load, 100 % = load inertia equal to the motor's).

// InertiaParams select data. They are detection settings, not bounds, and
// are printed in every report.
type InertiaParams struct {
	StillDeg     float64 `json:"other_joints_still_deg_s"`         // every other joint commanded slower than this
	AccTolAbsDeg float64 `json:"accel_run_tolerance_deg_s2"`       // a constant-acceleration run stays within this ...
	AccTolRel    float64 `json:"accel_run_tolerance_relative"`     // ... or this fraction of its acceleration
	MinAccDeg    float64 `json:"min_accel_deg_s2"`                 // an acceleration segment
	CruiseMinDeg float64 `json:"cruise_min_speed_deg_s"`           // a cruise segment (acceleration within tolerance of 0)
	SignMinDeg   float64 `json:"friction_sign_min_speed_deg_s"`    // every cycle this fast, so sign(v) is defined
	MinRunMs     int     `json:"min_run_ms"`                       // before trimming
	TrimLeadMs   int     `json:"trim_lead_ms"`                     // dropped after each commanded jerk phase
	MinKeptMs    int     `json:"min_kept_ms"`                      // after trimming
	EndAvgCyc    int     `json:"velocity_endpoint_average_cycles"` // actual acceleration = (v_end - v_start) / T, ends averaged
	LevelTolRel  float64 `json:"level_cluster_tolerance_relative"` // segments within this of a level belong to it
	MinBandDeg   float64 `json:"differential_min_band_deg_s"`      // shortest common speed band for a pair
	SmoothCyc    [2]int  `json:"timeline_smoothing_cycles"`        // plots: moving means for torque and actual acceleration
	// Coupled admits speed changes while other joints move and fits their
	// mean accelerations and positions as nuisance terms (inertial coupling
	// and gravity, held constant over the data): opportunistic only.
	Coupled bool `json:"coupled"`
}

func defaultInertiaParams() InertiaParams {
	// TrimLeadMs 30: about twice the position-loop lag with speed
	// feedforward on (14-16 ms, a Rosie 1430 arm's 2026-10-07 dumps, README "Lag"), so the
	// drive has caught the new acceleration before a segment is used; the
	// regressor is the actual acceleration either way. Tolerances fit the
	// commanded acceleration of native jogs and Hermite-interpolated dense
	// programs, which is exactly constant in a constant phase (the 1 kHz
	// target velocity's count quantisation is ~0.1 deg/s^2 at J4 over 2 ms).
	return InertiaParams{StillDeg: 0.01, AccTolAbsDeg: 1, AccTolRel: 0.01, MinAccDeg: 20, CruiseMinDeg: 2, SignMinDeg: 1,
		MinRunMs: 60, TrimLeadMs: 30, MinKeptMs: 30, EndAvgCyc: 5, LevelTolRel: 0.05, MinBandDeg: 3, SmoothCyc: [2]int{20, 10}}
}

// InertiaSegment is one constant-acceleration or cruise span of the joint
// under test, after trimming. Motor-side values are in the drive's count
// direction (joint x gear x sign); torque is 0x6077 per mille of rated.
type InertiaSegment struct {
	Window     int     `json:"window"`
	At         string  `json:"at"`
	DurMs      float64 `json:"duration_ms"`
	Kind       string  `json:"kind"`  // accel (constant) | ramp (whole speed change) | cruise
	Phase      string  `json:"phase"` // speed-up | slow-down | cruise
	Used       bool    `json:"used_in_fit"`
	Dir        int     `json:"direction"`
	Level      int     `json:"level"` // index into Levels, -1 for cruise
	CmdAccDeg  float64 `json:"commanded_accel_deg_s2"`
	ActAccDeg  float64 `json:"actual_accel_deg_s2"`
	MotorAcc   float64 `json:"motor_accel_rad_s2"`
	VMinDeg    float64 `json:"speed_min_deg_s"`
	VMaxDeg    float64 `json:"speed_max_deg_s"`
	QMeanDeg   float64 `json:"position_mean_deg"`
	TorqueMean float64 `json:"torque_mean_permille"`
	TorqueSD   float64 `json:"torque_sd_permille"`
	TorquePeak float64 `json:"torque_abs_peak_permille"`
	ResidualPm float64 `json:"fit_residual_permille"`
	from, to   int
	q1, q2     float64 // mean q and mean q^2 over the span, rad
	reg        map[string]float64
}

// InertiaTerm is one fitted coefficient with its 95 % interval.
type InertiaTerm struct {
	Name  string  `json:"name"`
	Unit  string  `json:"unit"`
	Value float64 `json:"value"`
	CI95  float64 `json:"ci95"`
}

// InertiaPair is one differential estimate: two groups of segments in the
// same direction at different accelerations over one common speed band.
type InertiaPair struct {
	Type         string     `json:"type"` // vendor: same phase, two levels | up/down: one level
	Dir          int        `json:"direction"`
	A            string     `json:"a"`
	B            string     `json:"b"`
	BandDeg      [2]float64 `json:"band_deg_s"`
	SegmentsA    int        `json:"segments_a"`
	SegmentsB    int        `json:"segments_b"`
	TorqueA      float64    `json:"torque_a_permille"`
	TorqueB      float64    `json:"torque_b_permille"`
	MotorAccA    float64    `json:"motor_accel_a_rad_s2"`
	MotorAccB    float64    `json:"motor_accel_b_rad_s2"`
	JRaw         float64    `json:"j_total_raw_kg_m2"`
	JGravityCorr float64    `json:"j_total_gravity_corrected_kg_m2"`
}

// InertiaEstimate is J_total, the load and C00.06 with a 95 % interval.
type InertiaEstimate struct {
	Method         string  `json:"method"`
	JTotal         float64 `json:"j_total_motor_side_kg_m2"`
	JTotalCI95     float64 `json:"j_total_ci95_kg_m2"`
	JLoad          float64 `json:"j_load_motor_side_kg_m2"`
	JLoadJointSide float64 `json:"j_load_joint_side_kg_m2"`
	C0006          float64 `json:"c00_06_percent"`
	C0006CI95      float64 `json:"c00_06_ci95_percent"`
	N              int     `json:"n"`
}

// InertiaReport is inertia.json.
type InertiaReport struct {
	Schema        string           `json:"schema"`
	Generated     string           `json:"generated"`
	Label         string           `json:"label,omitempty"`
	Opportunistic bool             `json:"opportunistic"`
	Joint         string           `json:"joint"`
	Config        *CellConfig      `json:"config"`
	Motor         JointMotor       `json:"motor"`
	Plan          *InertiaPlan     `json:"plan,omitempty"`
	Sources       []SourceInfo     `json:"sources"`
	Params        InertiaParams    `json:"params"`
	ConfiguredPct *float64         `json:"configured_c00_06_percent,omitempty"`
	TorqueFlipped bool             `json:"torque_sign_flipped"`
	Mode          string           `json:"fit_mode"` // constant | ramp
	FitN          int              `json:"fit_segments"`
	Levels        []float64        `json:"levels_deg_s2"`
	Segments      []InertiaSegment `json:"segments"`
	Terms         []InertiaTerm    `json:"fit_terms"`
	FitR2         float64          `json:"fit_r2"`
	FitRMS        float64          `json:"fit_residual_rms_permille"`
	FitDOF        int              `json:"fit_dof"`
	Fit           *InertiaEstimate `json:"fit_estimate"`
	Pairs         []InertiaPair    `json:"differential_pairs"`
	Vendor        *InertiaEstimate `json:"vendor_differential_estimate,omitempty"`
	UpDown        *InertiaEstimate `json:"up_down_differential_estimate,omitempty"`
	Notes         []string         `json:"notes,omitempty"`
	qRef          float64          // rad, the fit's position reference
}

type inertiaOptions struct {
	dumps, manifest, machine, compiled, repo, out, joint, motors, plan, label, tz, since string
	band                                                                                 *[2]float64
	plots                                                                                bool
	params                                                                               InertiaParams
}

func runInertia(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("inertia", flag.ContinueOnError)
	fs.SetOutput(stderr)
	o := inertiaOptions{params: defaultInertiaParams()}
	var band string
	noPlots := false
	fs.StringVar(&o.dumps, "dumps", "", "directory of copied Stop dumps (*.bin, *.trfx.gz)")
	fs.StringVar(&o.manifest, "manifest", "", "cell manifest (rt-core/config/cells/<cell>.json)")
	fs.StringVar(&o.machine, "machine", "", "machine config, instead of the manifest's")
	fs.StringVar(&o.compiled, "compiled", "", "compiled configuration directory, instead of compiling")
	fs.StringVar(&o.repo, "repo", "", "RosieOS checkout (default: found above the manifest)")
	fs.StringVar(&o.out, "out", "", "output directory")
	fs.StringVar(&o.joint, "joint", "", "joint under test, e.g. J4")
	fs.StringVar(&o.motors, "motors", "", "joint-motor file (rt-core/config/motors/<robot>.json): rated torque and rotor inertia")
	fs.StringVar(&o.plan, "plan", "", "the inertia-plan's plan.json; without it the report is labelled opportunistic")
	fs.StringVar(&o.label, "label", "", "label, e.g. 'J4 C00.06=550 session'")
	fs.StringVar(&o.tz, "tz", "Local", "time zone for printed times")
	fs.StringVar(&o.since, "since", "", "only dumps named after this: a duration back from now (30m) or a time in --tz (2026-10-08T10:30)")
	fs.StringVar(&band, "band-deg-s", "", "differential speed band LO,HI deg/s (default: each pair's common band)")
	fs.IntVar(&o.params.TrimLeadMs, "trim-lead-ms", o.params.TrimLeadMs, "drop this much of each segment after a jerk phase")
	fs.BoolVar(&o.params.Coupled, "coupled", false, "opportunistic: use the joint's speed changes while other joints move, fitting their accelerations and positions as nuisance terms")
	fs.Float64Var(&o.params.StillDeg, "still-deg-s", o.params.StillDeg, "other joints count as still under this commanded speed, deg/s (raise only for opportunistic data; their coupling torque is then unmodelled)")
	fs.BoolVar(&noPlots, "no-plots", false, "skip PNG plots")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if o.dumps == "" || o.out == "" || o.joint == "" || o.motors == "" || (o.manifest == "" && o.machine == "" && o.compiled == "") {
		usage(stderr)
		return errors.New("--dumps, --out, --joint, --motors and --manifest (or --machine or --compiled) are required")
	}
	if band != "" {
		v, err := parseFloats(band)
		if err != nil || len(v) != 2 || !(v[1] > v[0]) || v[0] < 0 {
			return fmt.Errorf("--band-deg-s %q: want LO,HI with 0 <= LO < HI", band)
		}
		o.band = &[2]float64{v[0], v[1]}
	}
	o.plots = !noPlots
	r, err := buildInertiaReport(o)
	if err != nil {
		return err
	}
	fmt.Fprintf(stdout, "wrote %s (%d segments)\n", filepath.Join(o.out, "README.md"), len(r.Segments))
	if e := r.Fit; e != nil {
		fmt.Fprintf(stdout, "%s fit: J_total %.4g +- %.2g kg.m^2 (motor side), C00.06 %.0f +- %.0f %%\n", r.Joint, e.JTotal, e.JTotalCI95, e.C0006, e.C0006CI95)
	}
	if e := r.Vendor; e != nil {
		fmt.Fprintf(stdout, "%s vendor differential: C00.06 %.0f +- %.0f %% (%d pairs)\n", r.Joint, e.C0006, e.C0006CI95, e.N)
	}
	return nil
}

// loadDumpWindows reads every dump and fixture in dir (named after since, as
// report does) and merges them into contiguous windows on the file-name clock.
func loadDumpWindows(dir, since string, tz *time.Location, axes int) ([]*Series, []SourceInfo, error) {
	var cut time.Time
	if since != "" {
		if d, err := time.ParseDuration(since); err == nil {
			cut = time.Now().Add(-d)
		} else if c, err := time.ParseInLocation("2006-01-02T15:04", since, tz); err == nil {
			cut = c
		} else {
			return nil, nil, fmt.Errorf("--since %q: want a duration (30m) or 2006-01-02T15:04", since)
		}
	}
	entries, err := os.ReadDir(dir)
	if err != nil {
		return nil, nil, err
	}
	var inputs []*Series
	var sources []SourceInfo
	for _, e := range entries {
		name := e.Name()
		if !strings.HasSuffix(name, ".bin") && !strings.HasSuffix(name, ".trfx.gz") {
			continue
		}
		if ft, ok := parseDumpFileTime(name); ok && !cut.IsZero() && ft.Before(cut) {
			continue
		}
		s, err := loadInput(filepath.Join(dir, name))
		if err != nil {
			return nil, nil, err
		}
		if s.Axes < axes {
			return nil, nil, fmt.Errorf("%s has %d axes, the configuration %d joints", name, s.Axes, axes)
		}
		inputs = append(inputs, s)
		sources = append(sources, s.Sources...)
	}
	windows, err := mergeSources(inputs)
	if err != nil {
		return nil, nil, err
	}
	settleClock(windows, nil, nil)
	return windows, sources, nil
}

func buildInertiaReport(o inertiaOptions) (*InertiaReport, error) {
	cfg, err := loadCellConfig(o.manifest, o.machine, o.compiled, o.repo)
	if err != nil {
		return nil, err
	}
	ji, err := jointIndex(cfg, o.joint)
	if err != nil {
		return nil, err
	}
	motor, err := loadJointMotor(o.motors, cfg.Joints[ji].Name)
	if err != nil {
		return nil, err
	}
	tz, err := time.LoadLocation(o.tz)
	if err != nil {
		return nil, err
	}
	windows, sources, err := loadDumpWindows(o.dumps, o.since, tz, len(cfg.Joints))
	if err != nil {
		return nil, err
	}
	r := &InertiaReport{Schema: "rosie.tuningreport.inertia.v1", Generated: time.Now().UTC().Format(time.RFC3339), Label: o.label,
		Joint: cfg.Joints[ji].Name, Config: cfg, Motor: motor, Sources: sources, Params: o.params, Opportunistic: o.plan == ""}
	if o.plan != "" {
		var p InertiaPlan
		if _, err := readJSON(o.plan, &p); err != nil {
			return nil, err
		}
		if !strings.EqualFold(p.Joint, r.Joint) {
			return nil, fmt.Errorf("%s is a plan for %s, not %s", o.plan, p.Joint, r.Joint)
		}
		p.Config = nil
		r.Plan = &p
	}
	a := newAnalysis(cfg, defaultParams(), tz, windows)
	if err := r.analyse(a, ji, o.band); err != nil {
		return nil, err
	}
	if err := os.MkdirAll(o.out, 0o755); err != nil {
		return nil, err
	}
	var plots []string
	if o.plots {
		if plots, err = r.plots(a, ji, o.out); err != nil {
			return nil, err
		}
	}
	if err := writeJSON(filepath.Join(o.out, "inertia.json"), r); err != nil {
		return nil, err
	}
	return r, os.WriteFile(filepath.Join(o.out, "README.md"), []byte(r.markdown(plots)), 0o644)
}

// analyse finds the segments, fits, pairs and estimates. It is the whole
// method; buildInertiaReport only loads and writes.
func (r *InertiaReport) analyse(a *Analysis, ji int, band *[2]float64) error {
	jc := r.Config.Joints[ji]
	p := r.Params
	if v, ok := jc.DriveSettings["load_inertia_ratio"]; ok {
		if f, err := strconv.ParseFloat(v, 64); err == nil {
			r.ConfiguredPct = &f
		}
	}
	for wi, w := range a.wins {
		if !w.tqOK[ji] {
			continue
		}
		r.Segments = append(r.Segments, findInertiaSegments(a, wi, ji, p)...)
	}
	if len(r.Segments) == 0 {
		return errors.New("no constant-acceleration or cruise segment of " + jc.Name + " alone with valid torque in these dumps")
	}
	r.clusterLevels()
	nAcc := 0
	for _, s := range r.Segments {
		if s.Kind == "accel" {
			nAcc++
		}
	}
	// Constant-acceleration segments at two or more levels (the designed
	// test) carry the fit; otherwise whole speed changes do (ordinary jogs:
	// a Rosie 1430 arm's 720 deg/s^2 at 150 rad/s^3 has no constant phase below 60 deg/s).
	r.Mode = "constant"
	if nAcc < 4 || len(r.Levels) < 2 {
		r.Mode = "ramp"
	}
	nUse := 0
	for i := range r.Segments {
		s := &r.Segments[i]
		s.Used = s.Kind == "cruise" || (r.Mode == "constant" && s.Kind == "accel") || (r.Mode == "ramp" && s.Kind == "ramp")
		if s.Used && s.Kind != "cruise" {
			nUse++
		}
	}
	if nUse < 2 {
		return fmt.Errorf("%d %s segment(s) of %s alone: the fit needs at least two", nUse, r.Mode, jc.Name)
	}
	if err := r.fit(); err != nil {
		return err
	}
	// 0x6077's sign follows the drive's torque direction; a physical
	// inertia is positive, so a confidently negative slope means the torque
	// reads opposite to the position counts on this joint.
	if t := r.Terms[0]; t.Value < 0 {
		if t.Value+t.CI95 >= 0 {
			return fmt.Errorf("%s: the inertia slope %.4g +- %.2g permille/(rad/s^2) is not distinguishable from zero", jc.Name, t.Value, t.CI95)
		}
		r.TorqueFlipped = true
		r.Notes = append(r.Notes, "0x6077 reads opposite to the position counts on this joint: torque sign flipped for the fit")
		for i := range r.Segments {
			r.Segments[i].TorqueMean = -r.Segments[i].TorqueMean
		}
		if err := r.fit(); err != nil {
			return err
		}
	}
	method := "least squares, constant-acceleration and cruise segments"
	if r.Mode == "ramp" {
		method = "least squares, whole speed changes and cruise segments"
	}
	r.Fit = r.estimate(method, r.Terms[0].Value, r.Terms[0].CI95, r.FitN)
	r.pairs(a, ji, band)
	var vendor, updown []float64
	for _, pr := range r.Pairs {
		if pr.Type == "vendor" {
			vendor = append(vendor, pr.JGravityCorr)
		} else {
			updown = append(updown, pr.JGravityCorr)
		}
	}
	scale := r.Motor.Motor.RatedTorqueNm / 1000
	if m, ci, ok := meanCI(vendor); ok {
		r.Vendor = r.estimate("vendor differential (same phase, two levels, gravity-corrected), mean of pairs", m/scale, ci/scale, len(vendor))
	}
	if m, ci, ok := meanCI(updown); ok {
		r.UpDown = r.estimate("speed-up vs slow-down at one level (gravity-corrected), mean of pairs", m/scale, ci/scale, len(updown))
	}
	if !r.Motor.Verified {
		r.Notes = append(r.Notes, fmt.Sprintf("motor %s for %s is an UNVERIFIED assignment (%s): rated torque %.2f N.m and rotor inertia %.3g kg.m^2 scale every estimate",
			r.Motor.Motor.Model, jc.Name, r.Motor.File, r.Motor.Motor.RatedTorqueNm, r.Motor.Motor.RotorInertiaKgM2))
	}
	if p.Coupled {
		r.Notes = append(r.Notes, "COUPLED: the other joints moved during these segments; their inertial coupling and gravity are fitted as constant linear terms (mean accelerations and positions), which holds only near one pose and ignores velocity-product torques. Trust the number only if it repeats across sessions.")
	}
	if r.Opportunistic {
		r.Notes = append(r.Notes, fmt.Sprintf("OPPORTUNISTIC: segments come from ordinary motion (jogs, Move-tos), not the designed test program (%d acceleration level(s)); the fit leans on its gravity and friction terms more than a designed test would", len(r.Levels)))
	}
	return nil
}

// findInertiaSegments walks one window: runs of constant commanded
// acceleration of joint ji while every other joint is commanded still, all
// joints enabled and torque valid, then trims and measures each.
func findInertiaSegments(a *Analysis, wi, ji int, p InertiaParams) []InertiaSegment {
	w := a.wins[wi]
	jc := a.cfg.Joints[ji]
	motor := jc.GearRatio * jc.Sign
	n := w.n
	acc := make([]float64, n)
	for k := 1; k+1 < n; k++ {
		acc[k] = (w.vc[ji][k+1] - w.vc[ji][k-1]) / (2 * w.dt)
	}
	ok := func(k int) bool {
		if k < 1 || k+1 >= n || w.s.TorqueValid[ji][k] == 0 {
			return false
		}
		for j := range a.cfg.Joints {
			if !w.on[j][k] {
				return false
			}
			if j != ji && !p.Coupled && math.Abs(w.vc[j][k])*deg > p.StillDeg {
				return false
			}
		}
		return true
	}
	if p.Coupled {
		return findRamps(a, wi, ji, p, acc, ok)
	}
	tolAbs := p.AccTolAbsDeg / deg
	var out []InertiaSegment
	k := 1
	for k+1 < n {
		if !ok(k) {
			k++
			continue
		}
		start, sum := k, acc[k]
		k++
		for k+1 < n && ok(k) {
			mean := sum / float64(k-start)
			if math.Abs(acc[k]-mean) > math.Max(tolAbs, p.AccTolRel*math.Abs(mean)) {
				break
			}
			sum += acc[k]
			k++
		}
		end := k // exclusive
		mean := sum / float64(end-start)
		if float64(end-start)*w.dt*1e3 < float64(p.MinRunMs) {
			continue
		}
		kind := "accel"
		if math.Abs(mean)*deg < p.MinAccDeg {
			if math.Abs(mean) > tolAbs {
				continue
			}
			kind = "cruise"
		}
		from := start + int(math.Round(float64(p.TrimLeadMs)/1e3/w.dt))
		if float64(end-from)*w.dt*1e3 < float64(p.MinKeptMs) {
			continue
		}
		vmin, vmax := math.Inf(1), 0.0
		dir := 0
		good := true
		for q := from; q < end; q++ {
			v := w.vc[ji][q]
			sv := 1
			if v < 0 {
				sv = -1
			}
			if dir == 0 {
				dir = sv
			}
			if sv != dir || math.Abs(v)*deg < p.SignMinDeg || math.Abs(w.va[ji][q])*deg < p.SignMinDeg*0.5 {
				good = false
				break
			}
			vmin, vmax = math.Min(vmin, math.Abs(v)), math.Max(vmax, math.Abs(v))
		}
		if !good || (kind == "cruise" && vmin*deg < p.CruiseMinDeg) {
			continue
		}
		s := measureSegment(w, ji, from, end, p, motor, false)
		s.Window, s.Kind, s.Dir, s.CmdAccDeg = wi, kind, dir, mean*deg
		s.VMinDeg, s.VMaxDeg = vmin*deg, vmax*deg
		s.At = a.stamp(w, from)
		switch {
		case kind == "cruise":
			s.Phase, s.Level = "cruise", -1
		case (mean > 0) == (dir > 0):
			s.Phase = "speed-up"
		default:
			s.Phase = "slow-down"
		}
		out = append(out, s)
	}
	return append(out, findRamps(a, wi, ji, p, acc, ok)...)
}

// slopeVA is the least-squares slope of velocity_actual over cycles
// [from, to): the acceleration of a constant phase from every sample, with
// no endpoint window to clip at the span's edges.
func slopeVA(w *win, ji, from, to int) float64 {
	n := float64(to - from)
	tm := (n - 1) / 2
	var vm, stv, stt float64
	for q := from; q < to; q++ {
		vm += w.va[ji][q]
	}
	vm /= n
	for q := from; q < to; q++ {
		t := float64(q-from) - tm
		stv += t * (w.va[ji][q] - vm)
		stt += t * t
	}
	return stv / stt / w.dt
}

// measureSegment averages over cycles [from, to): torque and regressors. The
// acceleration of a constant phase is velocity_actual's least-squares slope;
// a ramp's is (v_end - v_start) / T with each end averaged over steady
// cycles, the momentum balance that holds whatever the ramp's shape.
func measureSegment(w *win, ji, from, to int, p InertiaParams, motor float64, ramp bool) InertiaSegment {
	s := InertiaSegment{from: from, to: to, reg: map[string]float64{}}
	s.DurMs = float64(to-from) * w.dt * 1e3
	actAcc := slopeVA(w, ji, from, to)
	if ramp {
		m := p.EndAvgCyc
		endV := func(c int) float64 {
			sum, cnt := 0.0, 0
			for q := c - m; q <= c+m; q++ {
				if q >= 0 && q < w.n {
					sum += w.va[ji][q]
					cnt++
				}
			}
			return sum / float64(cnt)
		}
		actAcc = (endV(to-1) - endV(from)) / (float64(to-1-from) * w.dt)
	}
	s.ActAccDeg = actAcc * deg
	s.MotorAcc = actAcc * motor
	var tq, tq2, qm, qm2, vm, sg float64
	nq := float64(to - from)
	for q := from; q < to; q++ {
		t := w.tq[ji][q]
		tq += t
		tq2 += t * t
		qm += w.qa[ji][q]
		qm2 += w.qa[ji][q] * w.qa[ji][q]
		vm += w.va[ji][q]
		// Coulomb regressor: sign of the motor speed, 0 in the deadband
		// where a ramp starts from rest
		if v := w.va[ji][q]; math.Abs(v)*deg >= p.SignMinDeg/2 {
			sg += math.Copysign(1, v*motor)
		}
		s.TorquePeak = math.Max(s.TorquePeak, math.Abs(t))
	}
	s.TorqueMean = tq / nq
	s.TorqueSD = math.Sqrt(math.Max(0, tq2/nq-s.TorqueMean*s.TorqueMean))
	s.q1, s.q2 = qm/nq, qm2/nq
	s.QMeanDeg = s.q1 * deg
	s.reg["inertia"] = s.MotorAcc
	s.reg["viscous"] = vm / nq * motor
	s.reg["coulomb"] = sg / nq
	return s
}

// findRamps finds whole speed changes of joint ji: a run of commanded
// acceleration of one sign (any shape: jerk ramps with or without a constant
// phase), from its first cycle, while the actual speed is still steady, to
// TrimLeadMs after its last, when the drive has caught up, with the
// command steady over that extension. Torque over the span balances the
// momentum change: mean T = J dv/T + mean F + mean G.
func findRamps(a *Analysis, wi, ji int, p InertiaParams, acc []float64, ok func(int) bool) []InertiaSegment {
	w := a.wins[wi]
	jc := a.cfg.Joints[ji]
	motor := jc.GearRatio * jc.Sign
	tol := p.AccTolAbsDeg / deg
	ext := int(math.Round(float64(p.TrimLeadMs) / 1e3 / w.dt))
	var out []InertiaSegment
	k := 1
	for k+1 < w.n {
		if !ok(k) || math.Abs(acc[k]) <= tol {
			k++
			continue
		}
		start, sign := k, math.Signbit(acc[k])
		for k+1 < w.n && ok(k) && math.Abs(acc[k]) > tol && math.Signbit(acc[k]) == sign {
			k++
		}
		end := k + ext
		if end >= w.n {
			break
		}
		steady := true
		for q := k; q < end && !p.Coupled; q++ {
			if !ok(q) || math.Abs(acc[q]) > tol {
				steady = false
				break
			}
		}
		// the command must also be steady just before the ramp
		for q := start - ext/2; q < start && !p.Coupled; q++ {
			if q < 1 || !ok(q) || math.Abs(acc[q]) > tol {
				steady = false
				break
			}
		}
		for q := start; q < end && p.Coupled; q++ {
			if !ok(q) {
				steady = false
				break
			}
		}
		v0, v1 := w.vc[ji][start-1], w.vc[ji][k]
		dv := math.Abs(v1-v0) * deg
		if !steady || dv < p.CruiseMinDeg*2 || float64(end-start)*w.dt*1e3 < float64(p.MinRunMs) {
			continue
		}
		s := measureSegment(w, ji, start, end, p, motor, true)
		s.Window, s.Kind, s.Level = wi, "ramp", -1
		if p.Coupled {
			// the other joints' mean (joint-side) accelerations and
			// positions over the same span
			for i, oc := range a.cfg.Joints {
				if i == ji {
					continue
				}
				avg := func(c int) float64 {
					sum, cnt := 0.0, 0
					for q := c - p.EndAvgCyc; q <= c+p.EndAvgCyc; q++ {
						if q >= 0 && q < w.n {
							sum += w.va[i][q]
							cnt++
						}
					}
					return sum / float64(cnt)
				}
				qm := 0.0
				for q := start; q < end; q++ {
					qm += w.qa[i][q]
				}
				s.reg["couple_accel_"+oc.Name] = (avg(end-1) - avg(start)) / (float64(end-1-start) * w.dt)
				s.reg["couple_pos_"+oc.Name] = qm / float64(end-start)
			}
		}
		s.CmdAccDeg = (v1 - v0) / (float64(k-start) * w.dt) * deg
		s.VMinDeg, s.VMaxDeg = math.Min(math.Abs(v0), math.Abs(v1))*deg, math.Max(math.Abs(v0), math.Abs(v1))*deg
		s.At = a.stamp(w, start)
		vbig := v1
		if math.Abs(v0) > math.Abs(v1) {
			vbig = v0
		}
		s.Dir = 1
		if vbig < 0 {
			s.Dir = -1
		}
		s.Phase = "speed-up"
		if math.Abs(v1) < math.Abs(v0) {
			s.Phase = "slow-down"
		}
		if (v0 > 0 && v1 < 0) || (v0 < 0 && v1 > 0) {
			s.Phase = "reversal"
		}
		out = append(out, s)
	}
	return out
}

// clusterLevels groups acceleration segments by |commanded acceleration|
// into levels (deg/s^2), slowest first.
func (r *InertiaReport) clusterLevels() {
	var mags []float64
	for _, s := range r.Segments {
		if s.Kind == "accel" {
			mags = append(mags, math.Abs(s.CmdAccDeg))
		}
	}
	sort.Float64s(mags)
	var levels [][]float64
	for _, m := range mags {
		if n := len(levels); n > 0 && m <= mean(levels[n-1])*(1+r.Params.LevelTolRel) {
			levels[n-1] = append(levels[n-1], m)
			continue
		}
		levels = append(levels, []float64{m})
	}
	r.Levels = nil
	for _, l := range levels {
		r.Levels = append(r.Levels, mean(l))
	}
	for i := range r.Segments {
		s := &r.Segments[i]
		if s.Kind != "accel" {
			continue
		}
		best := 0
		for li, l := range r.Levels {
			if math.Abs(math.Abs(s.CmdAccDeg)-l) < math.Abs(math.Abs(s.CmdAccDeg)-r.Levels[best]) {
				best = li
			}
		}
		s.Level = best
	}
}

func mean(x []float64) float64 {
	if len(x) == 0 {
		return math.NaN()
	}
	s := 0.0
	for _, v := range x {
		s += v
	}
	return s / float64(len(x))
}

// fit is the weighted least squares over segments (weight = duration):
// torque = J' alpha_m + Fc sign(v) + Bv v_m + c0 + g1 dq + g2 dq^2. Terms the
// data cannot separate are left out (one direction: no Coulomb term; a
// window under 1 deg: no gravity slope; under 5 deg: no curvature) and named
// in the report. The interval is t(0.975, n - p) x the coefficient's
// standard error from the residual variance.
func (r *InertiaReport) fit() error {
	var segs []*InertiaSegment
	for i := range r.Segments {
		if r.Segments[i].Used {
			segs = append(segs, &r.Segments[i])
		}
	}
	qRef := 0.0
	for _, s := range segs {
		qRef += s.QMeanDeg
	}
	qRef = qRef / float64(len(segs)) / deg
	r.qRef = qRef
	dirs := map[int]bool{}
	qMin, qMax := math.Inf(1), math.Inf(-1)
	var vs []float64
	for _, s := range segs {
		dirs[s.Dir] = true
		// the span's mean of dq and dq^2 (not (mean dq)^2): a long segment
		// sweeps degrees of the window
		s.reg["offset"] = 1
		s.reg["gravity_1"] = s.q1 - qRef
		s.reg["gravity_2"] = s.q2 - 2*qRef*s.q1 + qRef*qRef
		qMin, qMax = math.Min(qMin, s.QMeanDeg), math.Max(qMax, s.QMeanDeg)
		vs = append(vs, math.Abs(s.reg["viscous"]))
	}
	terms := []struct{ name, unit string }{{"inertia", "permille/(rad/s^2) motor"}}
	if len(dirs) == 2 {
		terms = append(terms, struct{ name, unit string }{"coulomb", "permille"})
	}
	// a viscous slope needs segments at distinctly different mean speeds
	if sd(vs) >= 0.1*maxF(vs) {
		terms = append(terms, struct{ name, unit string }{"viscous", "permille/(rad/s) motor"})
	}
	terms = append(terms, struct{ name, unit string }{"offset", "permille"})
	if qMax-qMin >= 1 {
		terms = append(terms, struct{ name, unit string }{"gravity_1", "permille/rad"})
	}
	if qMax-qMin >= 5 {
		terms = append(terms, struct{ name, unit string }{"gravity_2", "permille/rad^2"})
	}
	if r.Params.Coupled {
		for _, oc := range r.Config.Joints {
			if oc.Name == r.Joint {
				continue
			}
			var ac, ps []float64
			for _, s := range segs {
				ac, ps = append(ac, s.reg["couple_accel_"+oc.Name]), append(ps, s.reg["couple_pos_"+oc.Name])
			}
			// 1 deg/s^2 and 0.5 deg of spread: below that the term is a
			// copy of the offset
			if sd(ac)*deg >= 1 {
				terms = append(terms, struct{ name, unit string }{"couple_accel_" + oc.Name, "permille/(rad/s^2) joint"})
			}
			if sd(ps)*deg >= 0.5 {
				terms = append(terms, struct{ name, unit string }{"couple_pos_" + oc.Name, "permille/rad"})
			}
		}
	}
	np := len(terms)
	if len(segs) <= np {
		return fmt.Errorf("%d segments for %d fitted terms: need more", len(segs), np)
	}
	X := make([][]float64, len(segs))
	y := make([]float64, len(segs))
	wts := make([]float64, len(segs))
	wsum := 0.0
	for i, s := range segs {
		for _, t := range terms {
			X[i] = append(X[i], s.reg[t.name])
		}
		y[i] = s.TorqueMean
		wts[i] = s.DurMs
		wsum += s.DurMs
	}
	for i := range wts {
		wts[i] *= float64(len(segs)) / wsum
	}
	beta, cov, rss, err := weightedLSQ(X, y, wts)
	if err != nil {
		return err
	}
	dof := len(segs) - np
	s2 := rss / float64(dof)
	tq := studentT975(dof)
	r.Terms = nil
	for i, t := range terms {
		r.Terms = append(r.Terms, InertiaTerm{Name: t.name, Unit: t.unit, Value: beta[i], CI95: tq * math.Sqrt(s2*cov[i][i])})
	}
	var ybar, sst float64
	for i := range y {
		ybar += wts[i] * y[i]
	}
	ybar /= float64(len(y))
	for i := range y {
		sst += wts[i] * (y[i] - ybar) * (y[i] - ybar)
	}
	// every segment gets a residual, used or not: an unused one shows how
	// well the fit predicts data it did not see
	for i := range r.Segments {
		s := &r.Segments[i]
		s.reg["offset"] = 1
		s.reg["gravity_1"] = s.q1 - qRef
		s.reg["gravity_2"] = s.q2 - 2*qRef*s.q1 + qRef*qRef
		pred := 0.0
		for c, t := range terms {
			pred += beta[c] * s.reg[t.name]
		}
		s.ResidualPm = s.TorqueMean - pred
	}
	r.FitR2, r.FitRMS, r.FitDOF, r.FitN = 1-rss/sst, math.Sqrt(rss/float64(len(y))), dof, len(segs)
	return nil
}

func maxF(x []float64) float64 {
	m := 0.0
	for _, v := range x {
		m = math.Max(m, v)
	}
	return m
}

func sd(x []float64) float64 {
	if len(x) < 2 {
		return 0
	}
	m := mean(x)
	s := 0.0
	for _, v := range x {
		s += (v - m) * (v - m)
	}
	return math.Sqrt(s / float64(len(x)-1))
}

// term returns a fitted coefficient, 0 when the term was not fitted.
func (r *InertiaReport) term(name string) float64 {
	for _, t := range r.Terms {
		if t.Name == name {
			return t.Value
		}
	}
	return 0
}

// estimate converts a slope in permille/(rad/s^2) to kg.m^2 and C00.06.
func (r *InertiaReport) estimate(method string, slope, ci float64, n int) *InertiaEstimate {
	m := r.Motor.Motor
	scale := m.RatedTorqueNm / 1000
	jt, jci := slope*scale, ci*scale
	gear := r.Config.Joints[0].GearRatio
	for _, j := range r.Config.Joints {
		if j.Name == r.Joint {
			gear = j.GearRatio
		}
	}
	return &InertiaEstimate{Method: method, JTotal: jt, JTotalCI95: jci, JLoad: jt - m.RotorInertiaKgM2, JLoadJointSide: (jt - m.RotorInertiaKgM2) * gear * gear,
		C0006: (jt/m.RotorInertiaKgM2 - 1) * 100, C0006CI95: jci / m.RotorInertiaKgM2 * 100, N: n}
}

// pairs forms the differential estimates. Within one direction, two groups
// of acceleration segments at different commanded accelerations are
// compared over the speed band both cover: the vendor pair is one phase
// (speed-up or slow-down) at two levels; the up/down pair is one level's
// speed-up against its slow-down. Same direction and same band cancel
// friction; gravity is removed with the fit's position terms (raw values
// are reported too: an up/down pair's two halves sit a whole move apart).
func (r *InertiaReport) pairs(a *Analysis, ji int, band *[2]float64) {
	type group struct {
		key  string
		dir  int
		lvl  int
		ph   string
		segs []int
	}
	groups := map[string]*group{}
	var keys []string
	for i, s := range r.Segments {
		if s.Kind != "accel" {
			continue
		}
		k := fmt.Sprintf("%+d %s L%d", s.Dir, s.Phase, s.Level+1)
		g := groups[k]
		if g == nil {
			g = &group{key: k, dir: s.Dir, lvl: s.Level, ph: s.Phase}
			groups[k] = g
			keys = append(keys, k)
		}
		g.segs = append(g.segs, i)
	}
	sort.Strings(keys)
	motor := r.Config.Joints[ji].GearRatio * r.Config.Joints[ji].Sign
	g1, g2 := r.term("gravity_1"), r.term("gravity_2")
	qRef := r.qRef
	flip := 1.0
	if r.TorqueFlipped {
		flip = -1
	}
	// bandMean averages one segment's cycles whose commanded speed is in
	// [lo, hi]: torque raw and gravity-corrected, and the actual motor
	// acceleration from the band's velocity endpoints.
	bandMean := func(s InertiaSegment, lo, hi float64) (raw, corr, acc float64, ok bool) {
		w := a.wins[s.Window]
		first, last := -1, -1
		var sr, sc float64
		n := 0
		for q := s.from; q < s.to; q++ {
			v := math.Abs(w.vc[ji][q]) * deg
			if v < lo || v > hi {
				continue
			}
			if first < 0 {
				first = q
			}
			last = q
			t := flip * w.tq[ji][q]
			dq := w.qa[ji][q] - qRef
			sr += t
			sc += t - g1*dq - g2*dq*dq
			n++
		}
		if n < 10 || last <= first {
			return 0, 0, 0, false
		}
		return sr / float64(n), sc / float64(n), slopeVA(w, ji, first, last+1) * motor, true
	}
	addPair := func(typ string, ga, gb *group) {
		lo, hi := 0.0, math.Inf(1)
		for _, gi := range append(append([]int(nil), ga.segs...), gb.segs...) {
			s := r.Segments[gi]
			lo, hi = math.Max(lo, s.VMinDeg), math.Min(hi, s.VMaxDeg)
		}
		if band != nil {
			lo, hi = math.Max(lo, band[0]), math.Min(hi, band[1])
		}
		if hi-lo < r.Params.MinBandDeg {
			return
		}
		pr := InertiaPair{Type: typ, Dir: ga.dir, A: ga.key, B: gb.key, BandDeg: [2]float64{lo, hi}}
		var ra, ca, aa, rb, cb, ab []float64
		for _, gi := range ga.segs {
			if x, y, z, ok := bandMean(r.Segments[gi], lo, hi); ok {
				ra, ca, aa = append(ra, x), append(ca, y), append(aa, z)
			}
		}
		for _, gi := range gb.segs {
			if x, y, z, ok := bandMean(r.Segments[gi], lo, hi); ok {
				rb, cb, ab = append(rb, x), append(cb, y), append(ab, z)
			}
		}
		if len(ra) == 0 || len(rb) == 0 || math.Abs(mean(aa)-mean(ab)) < 1e-9 {
			return
		}
		scale := r.Motor.Motor.RatedTorqueNm / 1000
		pr.SegmentsA, pr.SegmentsB = len(ra), len(rb)
		pr.TorqueA, pr.TorqueB, pr.MotorAccA, pr.MotorAccB = mean(ra), mean(rb), mean(aa), mean(ab)
		pr.JRaw = (mean(ra) - mean(rb)) / (mean(aa) - mean(ab)) * scale
		pr.JGravityCorr = (mean(ca) - mean(cb)) / (mean(aa) - mean(ab)) * scale
		r.Pairs = append(r.Pairs, pr)
	}
	for i, ka := range keys {
		for _, kb := range keys[i+1:] {
			ga, gb := groups[ka], groups[kb]
			if ga.dir != gb.dir {
				continue
			}
			switch {
			case ga.ph == gb.ph && ga.lvl != gb.lvl:
				addPair("vendor", ga, gb)
			case ga.ph != gb.ph && ga.lvl == gb.lvl:
				addPair("up/down", ga, gb)
			}
		}
	}
}

// meanCI is the mean and its 95 % interval (t, n - 1) over estimates.
func meanCI(x []float64) (float64, float64, bool) {
	if len(x) == 0 {
		return 0, 0, false
	}
	if len(x) == 1 {
		return x[0], math.NaN(), true
	}
	return mean(x), studentT975(len(x)-1) * sd(x) / math.Sqrt(float64(len(x))), true
}

// weightedLSQ solves min sum w_i (y_i - X_i beta)^2 by the normal equations
// on column-scaled regressors (Gauss-Jordan with partial pivoting) and
// returns beta, (X'WX)^-1 in the original units and the weighted RSS.
func weightedLSQ(X [][]float64, y, w []float64) ([]float64, [][]float64, float64, error) {
	n, p := len(X), len(X[0])
	scale := make([]float64, p)
	for c := 0; c < p; c++ {
		for i := 0; i < n; i++ {
			scale[c] = math.Max(scale[c], math.Abs(X[i][c]))
		}
		if scale[c] == 0 {
			return nil, nil, 0, fmt.Errorf("regressor %d is zero on every segment", c)
		}
	}
	A := make([][]float64, p)
	for r := range A {
		A[r] = make([]float64, 2*p+1)
	}
	for i := 0; i < n; i++ {
		for r := 0; r < p; r++ {
			xr := X[i][r] / scale[r]
			for c := 0; c < p; c++ {
				A[r][c] += w[i] * xr * X[i][c] / scale[c]
			}
			A[r][2*p] += w[i] * xr * y[i]
		}
	}
	for r := 0; r < p; r++ {
		A[r][p+r] = 1
	}
	for c := 0; c < p; c++ {
		piv := c
		for r := c + 1; r < p; r++ {
			if math.Abs(A[r][c]) > math.Abs(A[piv][c]) {
				piv = r
			}
		}
		if math.Abs(A[piv][c]) < 1e-12 {
			return nil, nil, 0, errors.New("the segments cannot separate the fitted terms (singular normal equations)")
		}
		A[c], A[piv] = A[piv], A[c]
		d := A[c][c]
		for k := range A[c] {
			A[c][k] /= d
		}
		for r := 0; r < p; r++ {
			if r == c {
				continue
			}
			f := A[r][c]
			for k := range A[r] {
				A[r][k] -= f * A[c][k]
			}
		}
	}
	beta := make([]float64, p)
	cov := make([][]float64, p)
	for r := 0; r < p; r++ {
		beta[r] = A[r][2*p] / scale[r]
		cov[r] = make([]float64, p)
		for c := 0; c < p; c++ {
			cov[r][c] = A[r][p+c] / (scale[r] * scale[c])
		}
	}
	rss := 0.0
	for i := 0; i < n; i++ {
		pred := 0.0
		for c := 0; c < p; c++ {
			pred += beta[c] * X[i][c]
		}
		rss += w[i] * (y[i] - pred) * (y[i] - pred)
	}
	return beta, cov, rss, nil
}

// studentT975 is the 0.975 quantile of Student's t with dof degrees of
// freedom, by bisection on the regularised incomplete beta function.
func studentT975(dof int) float64 {
	if dof < 1 {
		return math.NaN()
	}
	v := float64(dof)
	cdf := func(t float64) float64 { return 1 - 0.5*incBeta(v/2, 0.5, v/(v+t*t)) }
	lo, hi := 0.0, 1000.0
	for i := 0; i < 200; i++ {
		mid := (lo + hi) / 2
		if cdf(mid) < 0.975 {
			lo = mid
		} else {
			hi = mid
		}
	}
	return (lo + hi) / 2
}

// incBeta is the regularised incomplete beta I_x(a, b) (continued fraction,
// modified Lentz).
func incBeta(a, b, x float64) float64 {
	if x <= 0 {
		return 0
	}
	if x >= 1 {
		return 1
	}
	la, _ := math.Lgamma(a + b)
	lb, _ := math.Lgamma(a)
	lc, _ := math.Lgamma(b)
	front := math.Exp(la - lb - lc + a*math.Log(x) + b*math.Log(1-x))
	if x > (a+1)/(a+b+2) {
		return 1 - incBeta(b, a, 1-x)
	}
	const tiny = 1e-300
	f, c, d := 1.0, 1.0, 0.0
	for i := 0; i <= 400; i++ {
		m := float64(i / 2)
		var num float64
		switch {
		case i == 0:
			num = 1
		case i%2 == 0:
			num = m * (b - m) * x / ((a + 2*m - 1) * (a + 2*m))
		default:
			num = -((a + m) * (a + b + m) * x) / ((a + 2*m) * (a + 2*m + 1))
		}
		d = 1 + num*d
		if math.Abs(d) < tiny {
			d = tiny
		}
		d = 1 / d
		c = 1 + num/c
		if math.Abs(c) < tiny {
			c = tiny
		}
		cd := c * d
		f *= cd
		if math.Abs(1-cd) < 1e-14 {
			break
		}
	}
	return front * (f - 1) / a
}
