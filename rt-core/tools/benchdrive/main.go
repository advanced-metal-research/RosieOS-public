//go:build linux

package main

import (
	"context"
	"encoding/hex"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"syscall"
	"time"
	"unicode/utf8"

	"rosieos/rt-core/sdk/control"
)

// Unverified operator defaults and hard command bounds, in nanoseconds.
const (
	defaultDeadline = 10_000_000_000
	maxJogDuration  = 2_000_000_000
	defaultJog      = 1_000_000_000
	defaultHold     = 2_000_000_000
	defaultFraction = 0.1
	maxFraction     = 0.5
)

// Unverified operator travel bound for halt, whose cruise profile has no
// described-position check: 3 rad. Move and program are bounded instead by the
// axis's described min/max position (mini bench 2026-10-28: the multi-turn
// qualification needs single 719 degree moves inside a +/-720 degree range).
const maxHaltRadians = 3.0

type options struct {
	socket, controller, evidence, phase    string
	binding                                control.Binding
	mask                                   uint64
	axis                                   int
	fraction                               float64
	jogNS, deadlineNS, holdNS              int64
	confirm                                bool
	delta, velocityFraction, accelFraction float64
	keepArmed                              bool
	label                                  string
	deltas                                 string
	moves                                  []moveSpec
	direction                              int
	motorRevsPerAxisRev                    float64
	measure                                bool
	cruiseVelocity                         float64
	tuneCollision                          bool
	collisionMargin                        float64
}

// moveSpec is one axis displacement of a simultaneous move; every axis in a
// move shares one time profile so the group starts and settles together.
type moveSpec struct {
	axis  int
	delta float64
}

// parseMoveSpecs reads "axis:delta,axis:delta" pairs. Duplicate axes, out of
// range indices and unbounded displacements are refused before any request.
func parseMoveSpecs(text string) ([]moveSpec, error) {
	var moves []moveSpec
	seen := map[int]bool{}
	for _, item := range strings.Split(text, ",") {
		item = strings.TrimSpace(item)
		if item == "" {
			continue
		}
		axisText, deltaText, ok := strings.Cut(item, ":")
		if !ok {
			return nil, errors.New("deltas-rad entries must be axis:delta")
		}
		axis, err := strconv.Atoi(strings.TrimSpace(axisText))
		if err != nil || axis < 0 || axis >= 16 || seen[axis] {
			return nil, errors.New("deltas-rad axis must be a unique index in 0..15")
		}
		delta, err := strconv.ParseFloat(strings.TrimSpace(deltaText), 64)
		if err != nil || !finite(delta) || delta == 0 {
			return nil, errors.New("deltas-rad displacement must be finite and nonzero")
		}
		seen[axis] = true
		moves = append(moves, moveSpec{axis: axis, delta: delta})
	}
	if len(moves) == 0 {
		return nil, errors.New("deltas-rad selects no axis")
	}
	return moves, nil
}

// parseOptions rejects ambiguous masks, identities and unbounded motion before
// opening a connection, preventing invalid flags from acquiring authority.
func parseOptions(args []string, out io.Writer) (options, error) {
	var o options
	f := flag.NewFlagSet("benchdrive", flag.ContinueOnError)
	f.SetOutput(out)
	f.Usage = func() {
		fmt.Fprintln(out, "Usage: benchdrive [flags]")
		f.PrintDefaults()
		fmt.Fprintln(out, `
Exit classes:
  0 expectations_met
  1 expectation_failed
  2 usage
  3 motion_confirmation_required
  4 transport_or_protocol_loss
  5 observation_deadline_expired
  6 interrupted (SIGINT/SIGTERM or caller cancellation)
Transport/protocol loss takes precedence over deadline expiry and interruption.
Deadline expiry takes precedence over interruption.`)
	}
	f.StringVar(&o.socket, "socket", "control.sock", "application control socket")
	f.StringVar(&o.binding.PairID, "pair-id", "", "required pair identity")
	f.Uint64Var(&o.binding.Revision, "pair-revision", 0, "required pair revision")
	f.StringVar(&o.binding.ConfigurationSHA256, "configuration-sha256", "", "expected configuration digest")
	f.StringVar(&o.binding.MachineSHA256, "machine-sha256", "", "expected machine digest")
	f.StringVar(&o.controller, "controller", "benchdrive", "controller identity")
	f.StringVar(&o.evidence, "evidence", "", "required new JSONL file (never overwritten)")
	f.StringVar(&o.phase, "phase", "", "inhibited, home, enable, jog, move, halt, program, discard, recovery, telemetry, sequence or session")
	f.BoolVar(&o.measure, "measure", false, "move: require complete cyclic drive telemetry and report tracking/settling measurements; halt always measures")
	f.BoolVar(&o.tuneCollision, "tune-collision", false, "move/program: record native per-cycle torque and following-error peaks since arm and suggest unverified thresholds")
	f.Float64Var(&o.collisionMargin, "collision-margin", defaultCollisionMargin, "collision threshold multiplier, finite and greater than 1, at most 10")
	f.Float64Var(&o.cruiseVelocity, "cruise-velocity-rad-s", 0, "halt: required positive planned cruise velocity within the existing fraction/reserve cap")
	f.Uint64Var(&o.mask, "axis-mask", 0, "nonzero axis mask; defaults to all described axes; jog/move/halt select 1<<axis; program enables all described axes")
	f.IntVar(&o.axis, "axis", -1, "single Describe axis index for jog, move or halt")
	f.Float64Var(&o.fraction, "jog-fraction", defaultFraction, "fraction of max_velocity, greater than 0 and at most 0.5")
	f.IntVar(&o.direction, "direction", 0, "jog direction: +1 or -1; 0 preserves out-and-back")
	f.Float64Var(&o.motorRevsPerAxisRev, "motor-revs-per-axis-rev", 0, "verified motor/axis revolution ratio for jog evidence; required for directional jog and session")
	f.Float64Var(&o.delta, "delta-rad", 0, "move/halt/program displacement in rad, nonzero; move/program stay inside the described position range, halt within +/-3.0; program defaults to 0.2")
	f.StringVar(&o.deltas, "deltas-rad", "", "move: simultaneous multi-axis displacements as axis:rad pairs, e.g. 0:0.7854,8:-0.5236 (replaces axis and delta-rad)")
	f.Float64Var(&o.velocityFraction, "velocity-fraction", defaultFraction, "move/program fraction of max_velocity in (0, 0.5]")
	f.Float64Var(&o.accelFraction, "accel-fraction", defaultFraction, "move/program fraction of described acceleration in (0, 0.5]")
	f.BoolVar(&o.keepArmed, "keep-armed", false, "move: skip explicit Stop after completion; mandatory Release still inhibits outputs")
	f.StringVar(&o.label, "label", "benchdrive", "telemetry mark label, 1..128 UTF-8 bytes")
	f.Int64Var(&o.jogNS, "jog-duration-ns", defaultJog, "positive jog duration, at most 2000000000 ns")
	f.Int64Var(&o.deadlineNS, "deadline-ns", defaultDeadline, "positive per-observation deadline in ns")
	f.Int64Var(&o.holdNS, "hold-ns", defaultHold, "positive enabled hold duration in ns")
	f.BoolVar(&o.confirm, "confirm-motion", false, "required for energising phases and the inhibited Enable probe; confirm drive energisation")
	if err := f.Parse(args); err != nil {
		return o, err
	}
	if f.NArg() != 0 || o.socket == "" || o.controller == "" || o.evidence == "" || o.binding.PairID == "" || o.binding.Revision == 0 {
		return o, errors.New("socket, controller, evidence, pair-id and nonzero pair-revision are required; positional arguments are forbidden")
	}
	if !finite(o.collisionMargin) || o.collisionMargin <= 1 || o.collisionMargin > 10 {
		return o, errors.New("collision-margin must be finite and in (1,10]")
	}
	if o.tuneCollision && o.phase != "move" && o.phase != "program" {
		return o, errors.New("tune-collision requires phase move or program")
	}
	if (o.binding.ConfigurationSHA256 == "") == (o.binding.MachineSHA256 == "") {
		return o, errors.New("provide exactly one of configuration-sha256 or machine-sha256")
	}
	digest := o.binding.ConfigurationSHA256 + o.binding.MachineSHA256
	decoded, err := hex.DecodeString(digest)
	if err != nil || len(decoded) != 32 || hex.EncodeToString(decoded) != digest {
		return o, errors.New("digest must be 64 lowercase hexadecimal characters")
	}
	maskSet := false
	f.Visit(func(flag *flag.Flag) { maskSet = maskSet || flag.Name == "axis-mask" })
	if maskSet && o.mask == 0 {
		return o, errors.New("axis-mask must be in 1..65535")
	}
	switch o.phase {
	case "program":
		if o.axis != -1 || o.deltas != "" || o.mask > uint64(programAxisMask) {
			return o, errors.New("program requires axis-mask in 1..511, without axis or deltas-rad")
		}
		deltaSet := false
		f.Visit(func(flag *flag.Flag) { deltaSet = deltaSet || flag.Name == "delta-rad" })
		if !deltaSet {
			o.delta = defaultProgramDelta
		}
	case "inhibited", "home", "enable", "discard", "recovery", "telemetry", "sequence", "session":
		if o.axis != -1 {
			return o, errors.New("axis is only valid for jog or move")
		}
	case "jog", "move", "halt":
		if o.phase == "move" && o.deltas != "" {
			if o.axis != -1 || o.delta != 0 {
				return o, errors.New("deltas-rad replaces axis and delta-rad")
			}
			moves, err := parseMoveSpecs(o.deltas)
			if err != nil {
				return o, err
			}
			var union uint64
			for _, m := range moves {
				union |= 1 << m.axis
			}
			if o.mask != 0 && o.mask != union {
				return o, errors.New("move axis-mask must equal the deltas-rad axes")
			}
			o.mask, o.moves = union, moves
			break
		}
		if o.axis < 0 || o.axis >= 16 {
			return o, errors.New("jog, move or halt requires axis in 0..15")
		}
		if o.mask == 0 {
			o.mask = 1 << o.axis
		}
		if o.mask != 1<<o.axis {
			return o, errors.New("jog, move or halt axis-mask must select only axis")
		}
		if o.phase == "move" || o.phase == "halt" {
			o.moves = []moveSpec{{axis: o.axis, delta: o.delta}}
		}
	default:
		return o, errors.New("unknown phase")
	}
	if o.mask > 65535 {
		return o, errors.New("axis-mask must be in 1..65535")
	}
	if o.direction < -1 || o.direction > 1 || (o.direction != 0 && o.phase != "jog") {
		return o, errors.New("direction must be -1, 0 or +1 and is only valid for jog")
	}
	if !finite(o.motorRevsPerAxisRev) || o.motorRevsPerAxisRev < 0 || ((o.direction != 0 || o.phase == "session") && o.motorRevsPerAxisRev == 0) {
		return o, errors.New("directional jog and session require a positive finite motor-revs-per-axis-rev for the selected axes")
	}
	if !finite(o.fraction) || o.fraction <= 0 || o.fraction > maxFraction {
		return o, errors.New("jog-fraction must be finite and in (0, 0.5]")
	}
	if !finite(o.velocityFraction) || o.velocityFraction <= 0 || o.velocityFraction > maxFraction || !finite(o.accelFraction) || o.accelFraction <= 0 || o.accelFraction > maxFraction {
		return o, errors.New("velocity-fraction and accel-fraction must be finite and in (0, 0.5]")
	}
	if (o.phase == "move" || o.phase == "halt" || o.phase == "program") && o.deltas == "" && (!finite(o.delta) || o.delta == 0) {
		return o, errors.New("move/halt/program requires finite nonzero delta-rad")
	}
	if o.phase == "halt" && math.Abs(o.delta) > maxHaltRadians {
		return o, errors.New("halt requires |delta-rad| <= 3.0")
	}
	if o.measure && o.phase != "move" && o.phase != "halt" {
		return o, errors.New("measure is only valid for move or halt")
	}
	if !finite(o.cruiseVelocity) || (o.phase == "halt" && o.cruiseVelocity <= 0) || (o.phase != "halt" && o.cruiseVelocity != 0) {
		return o, errors.New("halt requires positive finite cruise-velocity-rad-s; other phases forbid it")
	}
	if o.phase == "halt" {
		if o.deltas != "" {
			return o, errors.New("halt requires one axis and delta-rad; deltas-rad is only valid for move")
		}
		o.measure = true
	}
	if o.keepArmed && o.phase != "move" {
		return o, errors.New("keep-armed is only valid for move")
	}
	if len(o.label) == 0 || len(o.label) > 128 || !utf8.ValidString(o.label) {
		return o, errors.New("label must contain 1..128 UTF-8 bytes")
	}
	if o.jogNS <= 0 || o.jogNS > maxJogDuration || o.deadlineNS <= 0 || o.holdNS <= 0 {
		return o, errors.New("durations must be positive; jog-duration-ns must be at most 2000000000")
	}
	return o, nil
}

func finite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

// outcome gives scripts and evidence the same classification. Transport failures,
// including unsent requests, precede joined cleanup deadlines so connection
// failures cannot be mistaken for observation timeouts.
func outcome(err error) (int, string) {
	switch {
	case err == nil:
		return 0, "expectations_met"
	case errors.Is(err, control.ErrTransport), errors.Is(err, control.ErrConnectionUnsent), errors.Is(err, control.ErrProtocol):
		return 4, "transport_or_protocol_loss"
	case errors.Is(err, context.DeadlineExceeded):
		return 5, "observation_deadline_expired"
	case errors.Is(err, context.Canceled):
		return 6, "interrupted"
	default:
		return 1, "expectation_failed"
	}
}

// run is shared by the executable and integration tests. Motion phases require
// confirmation before Dial; inhibited gates only its Enable probe, preventing
// unconfirmed drive energisation while retaining the other inhibited checks.
func run(ctx context.Context, args []string, out io.Writer) int {
	return runInput(ctx, args, os.Stdin, out)
}

// runInput owns session input until exit, so cancellation can unblock stdin and
// join its reader before Stop and Release complete.
func runInput(ctx context.Context, args []string, in io.ReadCloser, out io.Writer) int {
	o, err := parseOptions(args, out)
	if errors.Is(err, flag.ErrHelp) {
		return 0
	}
	if err != nil {
		fmt.Fprintln(out, err)
		if o.evidence != "" {
			if e, openErr := openEvidence(o.evidence); openErr == nil {
				e.write(record{Kind: "rejection", Operation: "flags", Request: args, Reason: "invalid_flags", Error: err.Error()})
				e.write(record{Kind: "summary", Class: "usage", ExitCode: 2, NotMet: []string{err.Error()}})
				if closeErr := e.close(); closeErr != nil {
					fmt.Fprintln(out, closeErr)
				}
			} else {
				fmt.Fprintln(out, openErr)
			}
		}
		return 2
	}
	e, err := openEvidence(o.evidence)
	if err != nil {
		fmt.Fprintln(out, err)
		return 1
	}
	code := executeInput(ctx, o, e, in, out)
	if err := e.close(); err != nil {
		fmt.Fprintln(out, err)
		return 1
	}
	return code
}

func execute(ctx context.Context, o options, e *evidence, out io.Writer) int {
	return executeInput(ctx, o, e, os.Stdin, out)
}

func executeInput(ctx context.Context, o options, e *evidence, in io.ReadCloser, out io.Writer) int {
	if (o.phase == "home" || o.phase == "enable" || o.phase == "jog" || o.phase == "move" || o.phase == "halt" || o.phase == "program" || o.phase == "sequence" || o.phase == "session") && !o.confirm {
		plan := "Acquire with renewal, Enable and observe the selected mask, Arm, observe Operation Enabled and readiness, hold and record drift, Stop, Release."
		if o.phase == "home" {
			plan = "Acquire with renewal, Home the selected mask, observe Home validity and disabled outputs, Release. Home energises the drive; a6ec native Home enables it and runs method 35, setting the current position as home without commanded motion."
		} else if o.phase == "jog" {
			plan = fmt.Sprintf("Acquire with renewal, Enable, Arm, bounded single-axis jog (direction=%d; 0 means out-and-back), End, Stop, Release.", o.direction)
		}
		if o.phase == "move" {
			plan = "Acquire with renewal, check Home, Enable, Arm, execute a bounded relative trajectory, observe endpoint and terminal handle, Stop unless --keep-armed, Release."
		}
		if o.phase == "halt" {
			plan = "Acquire with renewal, check Home, Enable, Arm, start a bounded trajectory, observe cruise in cyclic telemetry, Halt with the same session/generation, measure stopping and armed hold, Stop, Release."
		}
		if o.phase == "program" {
			plan = "Acquire with renewal, check Home and Enable all described axes, Arm, upload a supported dense program from held targets with only axis-mask moving, Start with the complete returned identity, observe Done, terminal handle, endpoints and velocity bounds, Stop, Release."
		}
		if o.phase == "sequence" {
			plan = "Acquire with renewal, inhibited probes, Home selected axes, Enable, Arm, hold, Stop, Release."
		}
		if o.phase == "session" {
			plan = "Acquire with renewal, Enable and Arm once, read operator commands until quit or EOF, Stop and Release on every exit."
		}
		fmt.Fprintf(out, "Would run %s, axis-mask=%d: %s\nPass --confirm-motion to confirm drive energisation and this sequence.\n", o.phase, o.mask, plan)
		e.write(record{Kind: "plan", Operation: o.phase, Response: plan})
		e.write(record{Kind: "summary", Class: "motion_confirmation_required", ExitCode: 3, Met: []string{}, NotMet: []string{"explicit motion confirmation required"}})
		return 3
	}
	c, err := control.Dial(control.UnixPath(o.socket))
	if err != nil {
		code, class := outcome(err)
		e.write(record{Kind: "summary", Class: class, ExitCode: code, NotMet: []string{err.Error()}})
		fmt.Fprintln(out, err)
		return code
	}
	defer c.Close()
	r := newRunner(o, c, e, out)
	r.in = in
	err = r.run(ctx)
	err = r.summary(err)
	if err != nil {
		fmt.Fprintln(out, err)
		code, _ := outcome(err)
		return code
	}
	fmt.Fprintf(out, "%s expectations met; evidence: %s\n", o.phase, o.evidence)
	return 0
}

func main() {
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()
	os.Exit(run(ctx, os.Args[1:], os.Stdout))
}

func timeout(ctx context.Context, ns int64) (context.Context, context.CancelFunc) {
	return context.WithTimeout(ctx, time.Duration(ns))
}
