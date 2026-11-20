package main

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
)

// The inertia test program: one joint, the others held, constant-acceleration
// phases at two or more levels in both directions over one position window,
// for the dual-acceleration differential (STEPPERONLINE, 2026-10-09) and the
// least-squares fit in inertia.go. It is a robot.v4.dense-joint-trajectory.v1
// program, so it runs through native program admission and execution like
// any planner program: every native limit, following-error fault and Stop
// applies. F30.10 cannot measure J4 in the arm (it needs >= 3000 rpm/s, about
// 1000 deg/s^2 at 18:1, above the 720 deg/s^2 limit).

// planDtS is the dense program grid: the planner's WIRE_DT (0.01 s), the only
// grid native program execution has run on a Rosie 1430 arm.
const planDtS = 0.01

// sMove is one rest-to-rest jerk-limited move of one joint: jerk ramps of
// +-jerk, a constant-acceleration phase at accel when the peak speed allows
// it, a cruise at vPeak, and the mirror image down. dist is signed.
type sMove struct {
	dist, accel, jerk, vPeak, cruise, dur float64
	pieces                                []jerkPiece
}

type jerkPiece struct{ t, j float64 }

// gridRamp is the jerk ramp in whole grid steps for accel a under jerk j.
func gridRamp(a, j, dt float64) int { return int(math.Ceil(a/(j*dt) - 1e-9)) }

// planMove plans a move of |dist| with every jerk piece a whole number of
// grid steps: ramps of ntj steps, a constant phase of ntc, a cruise of ncr.
// Native execution interpolates the knots with cubic Hermites, which
// reproduce a cubic exactly, so on the grid the 1 kHz command is this profile
// with no overshoot (off the grid a 400 deg/s^2 phase read 412.7 between
// knots). Rounding only ever lowers the acceleration (a' <= a) and the jerk
// (j' = a'/(ntj dt) <= j); the shortest move wins, then the higher a'.
func planMove(dist, vMax, a, j, dt float64) sMove {
	D := math.Abs(dist)
	m := sMove{dist: dist, accel: a, jerk: j}
	if D == 0 {
		return m
	}
	// distance = vp (tacc + ncr) dt, duration = (2 tacc + ncr) dt; a short
	// ramp (ntj below a/(j dt)) is the triangular profile of a slow move,
	// its acceleration capped by the jerk: a' <= j ntj dt
	bestN, bestA, bestJ, bestC, bestR := math.MaxInt, 0.0, 0, 0, 0
	for ntj := gridRamp(a, j, dt); ntj >= 1; ntj-- {
		aj := math.Min(a, j*float64(ntj)*dt)
		ntcMax := max(0, int(math.Floor(vMax/(aj*dt)-float64(ntj)+1e-9)))
		for ntc := ntcMax; ntc >= 0; ntc-- {
			capV := math.Min(vMax, aj*float64(ntj+ntc)*dt)
			tacc := 2*ntj + ntc
			ncr := max(0, int(math.Ceil(D/(capV*dt)-float64(tacc)-1e-9)))
			n := 2*tacc + ncr
			ap := D / (float64(tacc+ncr) * dt) / (float64(ntj+ntc) * dt)
			if n < bestN || (n == bestN && ap > bestA) {
				bestN, bestA, bestJ, bestC, bestR = n, ap, ntj, ntc, ncr
			}
		}
	}
	ntj, ap := bestJ, bestA
	jp := ap / (float64(ntj) * dt)
	m.accel, m.jerk = ap, jp
	m.vPeak, m.cruise, m.dur = ap*float64(ntj+bestC)*dt, float64(bestR)*dt, float64(bestN)*dt
	s := math.Copysign(1, dist)
	up := []jerkPiece{{float64(ntj) * dt, jp}, {float64(bestC) * dt, 0}, {float64(ntj) * dt, -jp}}
	for _, p := range up {
		m.pieces = append(m.pieces, jerkPiece{p.t, s * p.j})
	}
	m.pieces = append(m.pieces, jerkPiece{m.cruise, 0})
	for _, p := range up {
		m.pieces = append(m.pieces, jerkPiece{p.t, -s * p.j})
	}
	return m
}

// at integrates the jerk pieces exactly: displacement, velocity and
// acceleration t seconds into the move.
func (m sMove) at(t float64) (q, v, acc float64) {
	for _, p := range m.pieces {
		if t <= 0 {
			break
		}
		h := math.Min(t, p.t)
		q += v*h + acc*h*h/2 + p.j*h*h*h/6
		v += acc*h + p.j*h*h/2
		acc += p.j * h
		t -= p.t
	}
	return q, v, acc
}

// constantAccelS is the duration of one constant-acceleration phase.
func (m sMove) constantAccelS() float64 {
	if len(m.pieces) < 2 {
		return 0
	}
	return m.pieces[1].t
}

// PlanSegment is one program segment as the operator and the analysis read it.
type PlanSegment struct {
	Index        int     `json:"index"`
	Kind         string  `json:"kind"`
	Label        string  `json:"label"`
	StartS       float64 `json:"start_s"`
	DurationS    float64 `json:"duration_s"`
	FromDeg      float64 `json:"from_deg"`
	ToDeg        float64 `json:"to_deg"`
	AccelDeg     float64 `json:"accel_deg_s2,omitempty"`
	PeakDeg      float64 `json:"peak_speed_deg_s,omitempty"`
	ConstAccelMs float64 `json:"constant_accel_ms_each,omitempty"`
}

// PlanLevel is one acceleration level's feasibility on this joint.
type PlanLevel struct {
	AccelDeg     float64    `json:"accel_deg_s2"`
	ActualDeg    float64    `json:"accel_on_grid_deg_s2"` // the measurement moves' level after grid rounding (<= requested)
	MotorAccel   float64    `json:"motor_accel_rad_s2"`
	JerkRampMs   float64    `json:"jerk_ramp_ms"`
	ConstAccelMs float64    `json:"constant_accel_ms"`
	BandDeg      [2]float64 `json:"constant_accel_speed_band_deg_s"`
	TravelDeg    float64    `json:"travel_needed_deg"`
	// ExpectedPermille is J_total x motor acceleration at the configured
	// C00.06 (when the configuration writes it and --motors is given): a
	// headroom check, not a measurement.
	ExpectedPermille float64 `json:"expected_inertial_torque_permille,omitempty"`
}

// InertiaPlan is plan.json: everything needed to review, run and later
// analyse the program, with the configuration it was checked against.
type InertiaPlan struct {
	Schema           string        `json:"schema"`
	Generated        string        `json:"generated"`
	Joint            string        `json:"joint"`
	JointIndex       int           `json:"joint_index"`
	Config           *CellConfig   `json:"config"`
	Motor            *JointMotor   `json:"motor,omitempty"`
	StartDeg         []float64     `json:"start_pose_deg"`
	CenterDeg        float64       `json:"window_center_deg"`
	WindowDeg        [2]float64    `json:"window_deg"`
	PeakSpeedDeg     float64       `json:"peak_speed_deg_s"`
	PeakSpeedFrac    float64       `json:"peak_speed_x_velocity_limit"`
	JerkRadS3        float64       `json:"jerk_rad_s3"`
	Repeats          int           `json:"repeats"`
	DwellS           float64       `json:"dwell_s"`
	Levels           []PlanLevel   `json:"levels"`
	Segments         []PlanSegment `json:"segments"`
	DurationS        float64       `json:"duration_s"`
	Samples          int           `json:"samples"`
	Program          string        `json:"program_file"`
	TrajectoryDigest string        `json:"trajectory_digest"`
	ProgramDigest    string        `json:"program_digest"`
	PlanID           string        `json:"plan_id"`
	ProgramID        string        `json:"program_id"`
	Checks           []string      `json:"checks"`
	Notes            []string      `json:"notes,omitempty"`
}

type inertiaPlanOptions struct {
	manifest, machine, compiled, repo, out, joint, motors string
	accels                                                []float64
	start                                                 []float64
	center                                                *float64
	peakFrac, jerk, dwellS, leadS, cruiseS, minConstS     float64
	marginDeg, maxTotalS                                  float64
	repeats                                               int
	manifestRevision                                      uint64
}

func parseFloats(s string) ([]float64, error) {
	var out []float64
	for _, f := range strings.Split(s, ",") {
		if f = strings.TrimSpace(f); f == "" {
			continue
		}
		v, err := strconv.ParseFloat(f, 64)
		if err != nil || math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, fmt.Errorf("%q is not a number", f)
		}
		out = append(out, v)
	}
	return out, nil
}

func runInertiaPlan(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("inertia-plan", flag.ContinueOnError)
	fs.SetOutput(stderr)
	var o inertiaPlanOptions
	var accels, start, center string
	fs.StringVar(&o.manifest, "manifest", "", "cell manifest (rt-core/config/cells/<cell>.json)")
	fs.StringVar(&o.machine, "machine", "", "machine config, instead of the manifest's")
	fs.StringVar(&o.compiled, "compiled", "", "compiled configuration directory, instead of compiling")
	fs.StringVar(&o.repo, "repo", "", "RosieOS checkout (default: found above the manifest)")
	fs.StringVar(&o.out, "out", "", "output directory (program .rdt, plan.json, CHECKLIST.md)")
	fs.StringVar(&o.joint, "joint", "", "joint under test, e.g. J4")
	fs.StringVar(&o.motors, "motors", "", "joint-motor file (rt-core/config/motors/<robot>.json): adds the expected torque per level")
	fs.StringVar(&accels, "accels", "", "joint-side acceleration levels, deg/s^2, comma separated (at least two), e.g. 200,400")
	fs.StringVar(&start, "start-deg", "", "the held command of every joint, deg, comma separated, as OLP POS shows it (to 0.001 deg)")
	fs.StringVar(&center, "center-deg", "", "centre of the test window for the joint under test, deg (default: its start)")
	// 0.75: OLP Move-to's measured drive reserve (A6-EC velocity feedback
	// peaked 12.7 % above plan, 2026-09-14; jointMoveVelocityReserve).
	fs.Float64Var(&o.peakFrac, "peak-speed-frac", 0.75, "peak speed as a fraction of the compiled velocity limit")
	fs.Float64Var(&o.jerk, "jerk", 0, "jerk, rad/s^3 (default and ceiling: the compiled jog_jerk)")
	fs.IntVar(&o.repeats, "repeats", 4, "there-and-back moves per level")
	fs.Float64Var(&o.dwellS, "dwell-s", 0.5, "still time between moves, s")
	fs.Float64Var(&o.leadS, "lead-s", 1.0, "still time at the start and the end, s")
	fs.Float64Var(&o.cruiseS, "cruise-s", 0.2, "minimum cruise of the slowest level's moves, s (friction without acceleration)")
	fs.Float64Var(&o.minConstS, "min-constant-accel-s", 0.1, "refuse a level whose constant-acceleration phase is shorter, s")
	fs.Float64Var(&o.marginDeg, "limit-margin-deg", 2, "keep the window this far inside the compiled position limits, deg")
	fs.Float64Var(&o.maxTotalS, "max-duration-s", 90, "refuse a longer program, s (a Stop dump holds the last 100 s)")
	fs.Uint64Var(&o.manifestRevision, "manifest-revision", 0, "the cell's control binding revision, which native preparation requires as the program's manifest_revision (default: the manifest's rt_core.pair_revision; required with --machine or --compiled)")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if o.out == "" || o.joint == "" || accels == "" || start == "" || (o.manifest == "" && o.machine == "" && o.compiled == "") {
		usage(stderr)
		return errors.New("--out, --joint, --accels, --start-deg and --manifest (or --machine or --compiled) are required")
	}
	var err error
	if o.accels, err = parseFloats(accels); err != nil {
		return fmt.Errorf("--accels: %w", err)
	}
	if o.start, err = parseFloats(start); err != nil {
		return fmt.Errorf("--start-deg: %w", err)
	}
	if center != "" {
		c, err := strconv.ParseFloat(center, 64)
		if err != nil {
			return fmt.Errorf("--center-deg: %w", err)
		}
		o.center = &c
	}
	p, blob, err := buildInertiaPlan(o)
	if err != nil {
		return err
	}
	if err = writeInertiaPlan(o.out, p, blob); err != nil {
		return err
	}
	fmt.Fprintf(stdout, "wrote %s (%s, %.1f s, %d samples, %s)\n", filepath.Join(o.out, p.Program), p.Joint, p.DurationS, p.Samples, p.TrajectoryDigest)
	for _, l := range p.Levels {
		fmt.Fprintf(stdout, "  %.0f deg/s^2 (%.1f on the grid): constant acceleration %.0f ms over %.1f-%.1f deg/s\n", l.AccelDeg, l.ActualDeg, l.ConstAccelMs, l.BandDeg[0], l.BandDeg[1])
	}
	fmt.Fprintf(stdout, "checklist %s\n", filepath.Join(o.out, "CHECKLIST.md"))
	return nil
}

func jointIndex(cfg *CellConfig, name string) (int, error) {
	for i, j := range cfg.Joints {
		if strings.EqualFold(j.Name, name) {
			return i, nil
		}
	}
	var names []string
	for _, j := range cfg.Joints {
		names = append(names, j.Name)
	}
	return 0, fmt.Errorf("no joint %q in the configuration (%s)", name, strings.Join(names, ", "))
}

// buildInertiaPlan plans, encodes and checks the program. Every bound is the
// compiled configuration's; a level, window or duration the configuration
// does not allow is refused with the number that failed, never clipped.
func buildInertiaPlan(o inertiaPlanOptions) (*InertiaPlan, []byte, error) {
	cfg, err := loadCellConfig(o.manifest, o.machine, o.compiled, o.repo)
	if err != nil {
		return nil, nil, err
	}
	ji, err := jointIndex(cfg, o.joint)
	if err != nil {
		return nil, nil, err
	}
	jc := cfg.Joints[ji]
	nj := len(cfg.Joints)
	if nj > densejointtraj.AxisCount {
		return nil, nil, fmt.Errorf("%d joints; the dense format carries %d", nj, densejointtraj.AxisCount)
	}
	if len(o.start) != nj {
		return nil, nil, fmt.Errorf("--start-deg has %d values, the configuration %d joints", len(o.start), nj)
	}
	if len(o.accels) < 2 {
		return nil, nil, errors.New("--accels needs at least two levels: the differential is a difference")
	}
	if o.repeats < 1 || o.dwellS < planDtS || o.leadS < planDtS || o.cruiseS < 0 || !(o.peakFrac > 0 && o.peakFrac <= 1) {
		return nil, nil, errors.New("--repeats >= 1, --dwell-s and --lead-s >= 0.01, --cruise-s >= 0 and 0 < --peak-speed-frac <= 1")
	}
	if cfg.JogJerk <= 0 {
		return nil, nil, errors.New("the configuration has no jog_jerk: there is no compiled jerk to respect")
	}
	jerk := cfg.JogJerk
	if o.jerk != 0 {
		if o.jerk < 0 || o.jerk > cfg.JogJerk {
			return nil, nil, fmt.Errorf("--jerk %.1f rad/s^3: must be > 0 and <= the compiled jog_jerk %.1f", o.jerk, cfg.JogJerk)
		}
		jerk = o.jerk
	}
	aLim := jc.AccelerationLim
	if aLim <= 0 {
		return nil, nil, fmt.Errorf("%s has no compiled max_acceleration", jc.Name)
	}
	vp := o.peakFrac * jc.VelocityLimit
	start := make([]float64, nj)
	for i, d := range o.start {
		start[i] = d / deg
		if start[i] < cfg.Joints[i].LowerRad || start[i] > cfg.Joints[i].UpperRad {
			return nil, nil, fmt.Errorf("%s start %.3f deg is outside its limits %.1f..%.1f deg", cfg.Joints[i].Name, d, cfg.Joints[i].LowerRad*deg, cfg.Joints[i].UpperRad*deg)
		}
	}
	q0 := start[ji]
	qc := q0
	if o.center != nil {
		qc = *o.center / deg
	}
	levels := append([]float64(nil), o.accels...)
	sort.Float64s(levels)
	gear := jc.GearRatio
	p := &InertiaPlan{Schema: "rosie.tuningreport.inertia-plan.v1", Generated: time.Now().UTC().Format(time.RFC3339), Joint: jc.Name, JointIndex: ji,
		Config: cfg, StartDeg: o.start, CenterDeg: qc * deg, PeakSpeedDeg: vp * deg, PeakSpeedFrac: o.peakFrac, JerkRadS3: jerk,
		Repeats: o.repeats, DwellS: o.dwellS}
	var motor *JointMotor
	if o.motors != "" {
		m, err := loadJointMotor(o.motors, jc.Name)
		if err != nil {
			return nil, nil, err
		}
		motor, p.Motor = &m, &m
	}
	travel := 0.0
	for i, aDeg := range levels {
		if aDeg <= 0 || (i > 0 && aDeg == levels[i-1]) {
			return nil, nil, fmt.Errorf("--accels: levels must be positive and distinct (%v)", levels)
		}
		a := aDeg / deg
		if a > aLim*(1+1e-12) {
			return nil, nil, fmt.Errorf("%.0f deg/s^2 is above %s's compiled max_acceleration %.1f deg/s^2", aDeg, jc.Name, aLim*deg)
		}
		// on the grid: ramps of ntj steps, the longest constant phase that
		// keeps the peak at or under vp
		gridConst := func(a float64) (ntj, ntc int) {
			ntj = gridRamp(a, jerk, planDtS)
			return ntj, int(math.Floor(vp/(a*planDtS) - float64(ntj) + 1e-9))
		}
		ntj, ntc := gridConst(a)
		if float64(ntc)*planDtS < o.minConstS-1e-9 {
			best := 0.0
			for b := math.Floor(aDeg); b > 0; b-- {
				if _, c := gridConst(b / deg); float64(c)*planDtS >= o.minConstS-1e-9 {
					best = b
					break
				}
			}
			return nil, nil, fmt.Errorf("%.0f deg/s^2: the constant-acceleration phase is %d ms on the %.0f ms grid (peak %.1f deg/s, jerk %.0f rad/s^3, ramps %.0f ms each), under %.0f ms; the highest level that keeps %.0f ms is %.0f deg/s^2",
				aDeg, max(ntc, 0)*int(planDtS*1e3), planDtS*1e3, vp*deg, jerk, float64(ntj)*planDtS*1e3, o.minConstS*1e3, o.minConstS*1e3, best)
		}
		vl := a * float64(ntj+ntc) * planDtS
		dv := a * float64(ntj) * planDtS / 2
		need := vl * float64(2*ntj+ntc) * planDtS
		lv := PlanLevel{AccelDeg: aDeg, MotorAccel: a * gear, JerkRampMs: float64(ntj) * planDtS * 1e3, ConstAccelMs: float64(ntc) * planDtS * 1e3,
			BandDeg: [2]float64{dv * deg, (vl - dv) * deg}, TravelDeg: need * deg}
		if motor != nil {
			if r, ok := jc.DriveSettings["load_inertia_ratio"]; ok {
				if ratio, err := strconv.ParseFloat(r, 64); err == nil {
					jt := motor.Motor.RotorInertiaKgM2 * (1 + ratio/100)
					lv.ExpectedPermille = jt * a * gear / motor.Motor.RatedTorqueNm * 1000
				}
			}
		}
		p.Levels = append(p.Levels, lv)
		travel = math.Max(travel, need+vp*o.cruiseS)
	}
	lo, hi := qc-travel/2, qc+travel/2
	margin := o.marginDeg / deg
	if lo < jc.LowerRad+margin || hi > jc.UpperRad-margin || q0 < jc.LowerRad+margin || q0 > jc.UpperRad-margin {
		return nil, nil, fmt.Errorf("%s window %.2f..%.2f deg (start %.2f) is not %.1f deg inside its limits %.2f..%.2f deg: move the centre",
			jc.Name, lo*deg, hi*deg, q0*deg, o.marginDeg, jc.LowerRad*deg, jc.UpperRad*deg)
	}
	p.WindowDeg = [2]float64{lo * deg, hi * deg}

	// Segments: lead dwell, position to the window's low end, the levels
	// (slowest first) there and back, return to the start, tail dwell.
	var segs []densejointtraj.Segment
	var metas []densejointtraj.SegmentMeta
	q := q0
	clock := 0.0
	sample := func(qj, vj float64, t float64) densejointtraj.Sample {
		var s densejointtraj.Sample
		s.TimeS = t
		for i := 0; i < nj; i++ {
			s.QRad[i] = start[i]
		}
		s.QRad[ji], s.QdRadS[ji] = qj, vj
		return s
	}
	addDwell := func(label string, dur float64) {
		n := int(math.Round(dur/planDtS)) + 1
		seg := densejointtraj.Segment{Kind: "dwell", SourceID: label}
		for k := 0; k < n; k++ {
			seg.Samples = append(seg.Samples, sample(q, 0, float64(k)*planDtS))
		}
		d := float64(n-1) * planDtS
		p.Segments = append(p.Segments, PlanSegment{Index: len(segs), Kind: "dwell", Label: label, StartS: clock, DurationS: d, FromDeg: q * deg, ToDeg: q * deg})
		segs = append(segs, seg)
		metas = append(metas, densejointtraj.SegmentMeta{Index: len(metas), Kind: "dwell", SourceID: label, SampleCount: n, DurationS: d})
		clock += d
	}
	addMove := func(label string, to, a float64) {
		m := planMove(to-q, vp, a, jerk, planDtS)
		n := int(math.Round(m.dur/planDtS)) + 1
		seg := densejointtraj.Segment{Kind: "freespace", SourceID: label}
		for k := 0; k < n; k++ {
			t := float64(k) * planDtS
			dq, v, _ := m.at(t)
			if k == n-1 {
				dq, v = to-q, 0
			}
			if k == 0 {
				dq, v = 0, 0
			}
			seg.Samples = append(seg.Samples, sample(q+dq, v, t))
		}
		d := float64(n-1) * planDtS
		p.Segments = append(p.Segments, PlanSegment{Index: len(segs), Kind: "freespace", Label: label, StartS: clock, DurationS: d, FromDeg: q * deg, ToDeg: to * deg,
			AccelDeg: a * deg, PeakDeg: m.vPeak * deg, ConstAccelMs: m.constantAccelS() * 1e3})
		segs = append(segs, seg)
		metas = append(metas, densejointtraj.SegmentMeta{Index: len(metas), Kind: "freespace", SourceID: label, SampleCount: n, DurationS: d})
		clock += d
		q = to
	}
	addDwell("lead", o.leadS)
	if math.Abs(q0-lo) > 1e-9 {
		addMove("position to window", lo, levels[0]/deg)
		addDwell("dwell", o.dwellS)
	}
	for li, aDeg := range levels {
		m := planMove(hi-lo, vp, aDeg/deg, jerk, planDtS)
		lv := &p.Levels[li]
		lv.ActualDeg, lv.ConstAccelMs = m.accel*deg, m.constantAccelS()*1e3
		lv.MotorAccel = m.accel * gear
		// 2 %: the grid may lower a level by one cruise step's share
		// (<= 10 ms of a >= 0.6 s move); more means it planned another level
		if lv.ConstAccelMs < o.minConstS*1e3-1e-6 || lv.ActualDeg < 0.98*aDeg {
			return nil, nil, fmt.Errorf("%.0f deg/s^2: the window's moves run at %.1f deg/s^2 with %.0f ms of constant acceleration", aDeg, lv.ActualDeg, lv.ConstAccelMs)
		}
		if lv.ExpectedPermille != 0 {
			lv.ExpectedPermille *= m.accel / (aDeg / deg)
		}
		for r := 1; r <= o.repeats; r++ {
			addMove(fmt.Sprintf("L%d %.0f deg/s^2 rep %d +", li+1, aDeg, r), hi, aDeg/deg)
			addDwell("dwell", o.dwellS)
			addMove(fmt.Sprintf("L%d %.0f deg/s^2 rep %d -", li+1, aDeg, r), lo, aDeg/deg)
			addDwell("dwell", o.dwellS)
		}
	}
	if math.Abs(q0-lo) > 1e-9 {
		addMove("return to start", q0, levels[0]/deg)
	}
	addDwell("tail", o.leadS)
	p.DurationS = clock
	if clock > o.maxTotalS {
		return nil, nil, fmt.Errorf("the program lasts %.1f s, over --max-duration-s %.0f (a Stop dump holds the last 100 s): fewer repeats or levels", clock, o.maxTotalS)
	}
	total := 0
	for _, s := range segs {
		total += len(s.Samples)
	}
	p.Samples = total

	limits := make([]float64, densejointtraj.AxisCount)
	for i := range limits {
		limits[i] = jc.VelocityLimit
		if i < nj {
			limits[i] = cfg.Joints[i].VelocityLimit
		}
	}
	identityText, _ := json.Marshal(struct {
		Joint  string
		Start  []float64
		Levels []float64
		Window [2]float64
		Config string
		Peak   float64
		Jerk   float64
		Rep    int
	}{jc.Name, o.start, levels, p.WindowDeg, cfg.ConfigurationSHA256, vp, jerk, o.repeats})
	revision := o.manifestRevision
	if revision == 0 {
		revision = cfg.PairRevision
	}
	switch {
	case revision == 0:
		return nil, nil, errors.New("no control binding revision: pass --manifest (its rt_core.pair_revision) or --manifest-revision; native preparation refuses any other manifest_revision")
	case cfg.PairRevision != 0 && revision != cfg.PairRevision:
		return nil, nil, fmt.Errorf("--manifest-revision %d is not the manifest's rt_core.pair_revision %d", revision, cfg.PairRevision)
	}
	sum := sha256.Sum256(identityText)
	p.ProgramDigest = "sha256:" + hex.EncodeToString(sum[:])
	p.ProgramID = "tuningreport-inertia-" + jc.Name
	p.PlanID = p.ProgramID + ":" + hex.EncodeToString(sum[:6])
	header := densejointtraj.Header{Schema: densejointtraj.Schema, PlanID: p.PlanID, ProgramID: p.ProgramID, ProgramDigest: p.ProgramDigest,
		ManifestRevision: revision, PlanRevision: 1, CreatedAt: p.Generated,
		Axes:     densejointtraj.HeaderAxes{AxisCount: densejointtraj.AxisCount, AxisMask: uint32(1)<<uint(nj) - 1, PositionUnit: "rad", VelocityUnit: "rad_s"},
		Sampling: densejointtraj.Sampling{DtS: planDtS, TotalSampleCount: total, TotalDurationS: clock},
		Limits:   densejointtraj.Limits{MaxAbsQdRadS: limits}, Segments: metas,
		SampleEncoding: densejointtraj.EncodingMeta{Encoding: densejointtraj.SampleEncoding, RecordBytes: densejointtraj.RecordBytes}}
	blob, err := densejointtraj.Encode(header, segs)
	if err != nil {
		return nil, nil, err
	}
	if blob, err = stampRobotCell(blob, cfg); err != nil {
		return nil, nil, err
	}
	checks, err := checkInertiaProgram(cfg, blob)
	if err != nil {
		return nil, nil, err
	}
	checks = append(checks, fmt.Sprintf("robot_cell: model %s, description %.19s, calibration %.19s, configuration %.12s (OLP refuses the program on a cell that is not exactly this)",
		cfg.ModelID, cfg.DescriptionSHA256, cfg.CalibrationSHA256, cfg.ConfigurationSHA256))
	plan, _ := densejointtraj.Decode(blob)
	p.TrajectoryDigest = plan.Header.TrajectoryDigest
	p.Program = fmt.Sprintf("inertia-%s-%s.rdt", strings.ToLower(jc.Name), strings.TrimPrefix(p.TrajectoryDigest, "sha256:")[:12])
	p.Checks = checks
	p.Notes = append(p.Notes, cfg.Notes...)
	if motor != nil && !motor.Verified {
		p.Notes = append(p.Notes, fmt.Sprintf("%s's motor %s is an UNVERIFIED assignment (%s)", jc.Name, motor.Motor.Model, motor.File))
	}
	return p, blob, nil
}

// stampRobotCell writes the header's robot_cell: the robot and machine
// configuration the program was checked against, which OLP's test-program
// Load compares with the cell's own and refuses on any difference. rt-core's
// codec does not carry the block (it is planner identity outside the
// trajectory digest, ignored by native admission), so it is added to the
// encoded header JSON; segment blocks are addressed from the end of the
// header, and the digest covers motion only, so neither moves.
func stampRobotCell(blob []byte, cfg *CellConfig) ([]byte, error) {
	hexDigest := regexp.MustCompile(`^sha256:[0-9a-f]{64}$`)
	configuration := "sha256:" + cfg.ConfigurationSHA256
	for name, v := range map[string]string{"robot description": cfg.DescriptionSHA256, "machine_planning_calibration.json": cfg.CalibrationSHA256, "configuration": configuration} {
		if !hexDigest.MatchString(v) {
			return nil, fmt.Errorf("the configuration's %s identity %q is not sha256:<64 hex>: the program could not name its robot", name, v)
		}
	}
	if cfg.ModelID == "" {
		return nil, errors.New("the configuration's robot binding names no model: the program could not name its robot")
	}
	if len(blob) < 4 {
		return nil, errors.New("encoded program has no header")
	}
	n := int(binary.BigEndian.Uint32(blob[:4]))
	if 4+n > len(blob) {
		return nil, errors.New("encoded program header overruns the blob")
	}
	var header map[string]json.RawMessage
	if err := json.Unmarshal(blob[4:4+n], &header); err != nil {
		return nil, err
	}
	robot, err := json.Marshal(map[string]string{
		"model_id":                            cfg.ModelID,
		"robot_description_sha256":            cfg.DescriptionSHA256,
		"machine_planning_calibration_sha256": cfg.CalibrationSHA256,
		// The compiled configuration's calibration: the bytes the cell
		// serves under that configuration, not a model's empty document.
		"machine_planning_calibration_source": "target",
		"machine_configuration_sha256":        configuration,
	})
	if err != nil {
		return nil, err
	}
	header["robot_cell"] = robot
	raw, err := json.Marshal(header)
	if err != nil {
		return nil, err
	}
	out := make([]byte, 4, 4+len(raw)+len(blob)-4-n)
	binary.BigEndian.PutUint32(out, uint32(len(raw)))
	out = append(append(out, raw...), blob[4+n:]...)
	return out, nil
}

// checkInertiaProgram decodes the blob with the format validator the
// daemon's twin uses, then repeats native program admission
// (adapters/rosie/control DecodeProgram, linux-only) on the Mac: per-sample
// speed against max_velocity, per-sample acceleration (dqd/dt on the
// rounded clock) against max_acceleration, and every Hermite interval's
// position extremum and peak speed through densejointtraj.AdmitHermiteSegment
// in the zero-Home frame offline producers use. Live preparation repeats all
// of it with the cell's Home offsets.
func checkInertiaProgram(cfg *CellConfig, blob []byte) ([]string, error) {
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		return nil, fmt.Errorf("format validation: %w", err)
	}
	segments, err := densejointtraj.NormaliseStationary(plan)
	if err != nil {
		return nil, fmt.Errorf("stationary normalisation: %w", err)
	}
	peakV, peakA := make([]float64, len(cfg.Joints)), make([]float64, len(cfg.Joints))
	for si, seg := range segments {
		for k, s := range seg.Samples {
			for j, jc := range cfg.Joints {
				frame := densejointtraj.CommandFrame{CountsPerUnit: jc.CountsPerRad, Sign: int(jc.Sign)}
				q, qd := s.QRad[j], s.QdRadS[j]
				if k == 0 {
					if err := densejointtraj.AdmitHermiteKnot(frame, densejointtraj.HermiteKnot{Position: q, Velocity: qd}, jc.LowerRad, jc.UpperRad); err != nil {
						return nil, fmt.Errorf("segment %d sample 0 %s: %w", si, jc.Name, err)
					}
				}
				if math.Abs(qd) > jc.VelocityLimit {
					return nil, fmt.Errorf("segment %d sample %d %s: %.4f rad/s over max_velocity %.4f", si, k, jc.Name, qd, jc.VelocityLimit)
				}
				peakV[j] = math.Max(peakV[j], math.Abs(qd))
				if k == 0 {
					continue
				}
				prev := seg.Samples[k-1]
				t0, t1 := int64(math.Round(prev.TimeS*1e9)), int64(math.Round(s.TimeS*1e9))
				b, err := densejointtraj.AdmitHermiteSegment(frame, densejointtraj.HermiteKnot{TimeNS: t0, Position: prev.QRad[j], Velocity: prev.QdRadS[j]},
					densejointtraj.HermiteKnot{TimeNS: t1, Position: q, Velocity: qd}, jc.LowerRad, jc.UpperRad)
				if err != nil {
					return nil, fmt.Errorf("segment %d sample %d %s: %w", si, k, jc.Name, err)
				}
				if b.PeakVelocity > jc.VelocityLimit*jc.CountsPerRad {
					return nil, fmt.Errorf("segment %d sample %d %s: interpolated peak %.4f rad/s over max_velocity", si, k, jc.Name, b.PeakVelocity/jc.CountsPerRad)
				}
				acc := (qd - prev.QdRadS[j]) / (float64(t1-t0) / 1e9)
				if math.Abs(acc) > jc.AccelerationLim {
					return nil, fmt.Errorf("segment %d sample %d %s: %.3f rad/s^2 over max_acceleration %.3f", si, k, jc.Name, acc, jc.AccelerationLim)
				}
				peakA[j] = math.Max(peakA[j], math.Abs(acc))
			}
		}
	}
	checks := []string{"format: robot.v4.dense-joint-trajectory.v1 decode and validation (densejointtraj.Decode), digest " + plan.Header.TrajectoryDigest,
		"stationary seams normalise (densejointtraj.NormaliseStationary)"}
	for j, jc := range cfg.Joints {
		checks = append(checks, fmt.Sprintf("%s: peak %.2f deg/s of max_velocity %.2f, peak sampled acceleration %.1f deg/s^2 of max_acceleration %.1f, every Hermite interval inside %.2f..%.2f deg",
			jc.Name, peakV[j]*deg, jc.VelocityLimit*deg, peakA[j]*deg, jc.AccelerationLim*deg, jc.LowerRad*deg, jc.UpperRad*deg))
	}
	return checks, nil
}

func writeInertiaPlan(dir string, p *InertiaPlan, blob []byte) error {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(dir, p.Program), blob, 0o644); err != nil {
		return err
	}
	if err := writeJSON(filepath.Join(dir, "plan.json"), p); err != nil {
		return err
	}
	return os.WriteFile(filepath.Join(dir, "CHECKLIST.md"), []byte(p.checklist()), 0o644)
}

// checklist is the operator's page for the one session the program needs.
func (p *InertiaPlan) checklist() string {
	var b strings.Builder
	jc := p.Config.Joints[p.JointIndex]
	cell := strings.TrimSuffix(filepath.Base(p.Config.ManifestPath), ".json")
	if cell == "" || cell == "." {
		cell = "<cell>"
	}
	w := func(f string, a ...any) { fmt.Fprintf(&b, f, a...) }
	w("# %s load inertia test (dual-acceleration differential)\n\n", p.Joint)
	w("Program `%s` (%s), %.1f s, generated %s against configuration `%.12s`", p.Program, p.TrajectoryDigest, p.DurationS, p.Generated, p.Config.ConfigurationSHA256)
	if p.Config.PinnedConfiguration != "" {
		w(" (cell pin `%.12s`)", p.Config.PinnedConfiguration)
	}
	w(".\n\nOnly %s moves; every other joint holds its start angle. Levels (joint side): ", p.Joint)
	for i, l := range p.Levels {
		if i > 0 {
			w(", ")
		}
		w("%.0f deg/s^2 (%.1f on the 10 ms grid, %.0f ms constant, %.1f-%.1f deg/s)", l.AccelDeg, l.ActualDeg, l.ConstAccelMs, l.BandDeg[0], l.BandDeg[1])
	}
	w("; peak %.1f deg/s (%.2f x the %.1f deg/s limit), jerk %.0f rad/s^3 (compiled jog_jerk %.0f), %d there-and-back moves per level.\n\n",
		p.PeakSpeedDeg, p.PeakSpeedFrac, jc.VelocityLimit*deg, p.JerkRadS3, p.Config.JogJerk, p.Repeats)
	for _, n := range p.Notes {
		w("> NOTE: %s\n\n", n)
	}
	w("## Before the session (owner)\n\n")
	w("- [ ] The cell runs configuration `%.12s` (the program was checked against its limits; regenerate after any configuration change).\n", p.Config.ConfigurationSHA256)
	w("- [ ] Owner approval for this motion. The cell's OLP has test programs on (cell manifest `cell_deploy.olp.test_programs`; otherwise the OLP refuses the upload with `test_programs_disabled`).\n")
	w("- [ ] No build, deploy or test is running on the cell's hosts (no maintenance notice).\n\n")
	w("## Pose and clearance\n\n")
	w("- [ ] Home valid on every joint (Home 63). Never re-home for this test.\n")
	w("- [ ] Start pose, every joint's held command (OLP POS), deg:\n\n")
	w("  | joint | start |\n  |---|---|\n")
	for i, d := range p.StartDeg {
		w("  | %s | %.3f |\n", p.Config.Joints[i].Name, d)
	}
	w("\n  Every joint must match within %.3f deg (completion_tolerance; native settles a smaller offset before the program and refuses a larger one with `start_discontinuity`). Otherwise regenerate with the POS values.\n", jc.CompletionTol*deg)
	w("- [ ] %s sweeps **%.2f to %.2f deg** (window centre %.2f deg). Walk the whole sweep of the arm distal to %s at the start pose: cables, hoses, fixtures, people. Nothing within reach.\n",
		p.Joint, p.WindowDeg[0], p.WindowDeg[1], p.CenterDeg, p.Joint)
	w("- [ ] Gravity: for a joint whose axis is not vertical (on a Rosie 1430 arm: J2, J3, J5 always; J4/J6 when the wrist is not straight), centre the window where the distal link is horizontal, so gravity torque is at its extremum and changes least over the window (`--center-deg`). The analysis also fits gravity as a quadratic in position.\n")
	w("- [ ] Brakes: J2/J3 are gravity-loaded; a Stop or fault disables the drives and the drive-managed brake holds. Stand clear of the sweep below the arm.\n")
	w("- [ ] Hand on the hardware E-stop for the whole program. OLP STOP is software, not an emergency stop.\n\n")
	w("## Run\n\n")
	w("- [ ] Arm in OLP. Machine panel, Run: **Load test program (.rdt)…**, choose `%s` and this directory's `plan.json` together. The OLP checks the robot, the configuration (`%.12s`) and that every joint is at the start within its completion tolerance, then loads it, labelled TEST PROGRAM. Any mismatch is refused with the reason; fix the cause, never the check.\n", p.Program, p.Config.ConfigurationSHA256)
	w("- [ ] Press **▶ Play test program**. Watch: following-error faults stop the arm by themselves; STOP and the heartbeat deadman work as for any program.\n")
	w("- [ ] When it ends (back at the start pose, execution completed), press **STOP within 10 s**: the Stop dump holds the last 100 s and the program lasts %.1f s. Then Disarm.\n", p.DurationS)
	w("- [ ] Note the time of the Stop (local) and the configured C00.06 of %s", p.Joint)
	if r, ok := jc.DriveSettings["load_inertia_ratio"]; ok {
		w(" (this configuration writes %s %%)", r)
	}
	w(".\n\n## After\n\n")
	motors := "config/motors/<robot>.json"
	if p.Motor != nil {
		motors = p.Motor.File
	}
	w("```sh\ncd steamdeck/real/v4\ngo run . realtime-core copy-dumps --manifest rt-core/config/cells/%s.json --to mac\ncd ../../../rt-core\n", cell)
	w("go run ./tools/tuningreport inertia --dumps ~/.rosie/stop-dumps/%s --since 30m \\\n  --manifest config/cells/%s.json --joint %s \\\n  --motors %s --plan <this directory>/plan.json --out ~/.rosie/tuning-reports/<date>-%s-inertia\n```\n\n",
		cell, cell, p.Joint, motors, strings.ToLower(p.Joint))
	w("The report gives J_total (motor side), the load inertia and the C00.06 estimate with a 95 %% interval. Setting C00.06 is a separate configuration change (`rt-core/docs/drives/a6ec/README.md`, \"Tuning a joint\").\n\n")
	w("## Offline checks passed\n\n")
	for _, c := range p.Checks {
		w("- %s\n", c)
	}
	return b.String()
}
