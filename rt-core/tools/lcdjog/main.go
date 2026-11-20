//go:build linux

// lcdjog: hold-to-jog for the LCD shield panel (tools/lcddash).
//
// One process owns one grant for as long as the panel's JOG page is armed and
// turns stdin velocity lines into the adapter's local jog datagrams at 5 ms
// cadence, the same producer contract benchdrive's jog phase uses. It exists
// because the bounded `jog` phase pre-checks v*t + v^2/2a of travel against the
// described span, which on a 314 rad/s axis with a +-3 rad span refuses every
// useful speed, and because a button is a velocity source, not an angle.
//
//	stdin:  "v <fraction>"  signed fraction of max_velocity, |f| <= --max-fraction;
//	                       applies until the next line or --stale-ms without one
//	        "q"             end jog, Stop, Release, exit
//	stdout: "READY"         grant held, axis Operation Enabled, jog lane open
//	        "POS <rad> <statusword>"  every 50 ms while running
//	        "LIMIT"         native limit approach clamped the input
//	        "REJECTED <reason>" / "ERR <text>"  then Stop, Release and exit
//	        "STOPPED"       outputs inhibited and authority released
//
// Three independent things stop the motor when the panel goes quiet: this
// process zeroes velocity after --stale-ms without a line; every nonzero frame
// carries a 100 ms deadline the native cell enforces; and stdin EOF (the panel
// process dying) runs the full End/Stop/Release path. Nothing here decides
// where the axis may go: limit approach and following error stay native.
package main

import (
	"bufio"
	"context"
	"errors"
	"flag"
	"fmt"
	"math"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Cadences and bounds shared with benchdrive's jog phase (unverified there too):
// 5 ms producer, 50 ms observer, 100 ms renewal, 100 ms per-frame input lifetime.
const (
	jogCadence    = 5 * time.Millisecond
	pollCadence   = 50 * time.Millisecond
	renewCadence  = 100 * time.Millisecond
	inputNS       = 100_000_000
	operationMask = 0x006f
	operationWord = 0x0027
)

type options struct {
	socket, controller string
	binding            control.Binding
	axis               int
	maxFraction        float64
	staleMS            int
	deadlineNS         int64
	confirm            bool
}

func finite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

func parse(args []string) (options, error) {
	var o options
	f := flag.NewFlagSet("lcdjog", flag.ContinueOnError)
	f.StringVar(&o.socket, "socket", "control.sock", "application control socket")
	f.StringVar(&o.binding.PairID, "pair-id", "", "required pair identity")
	f.Uint64Var(&o.binding.Revision, "pair-revision", 0, "required pair revision")
	f.StringVar(&o.binding.ConfigurationSHA256, "configuration-sha256", "", "expected configuration digest")
	f.StringVar(&o.controller, "controller", "lcddash", "controller identity")
	f.IntVar(&o.axis, "axis", 0, "Describe axis index")
	f.Float64Var(&o.maxFraction, "max-fraction", 0.5, "ceiling on |v| as a fraction of the described max_velocity, at most 1.0 (the native cell refuses above the described limit regardless)")
	f.IntVar(&o.staleMS, "stale-ms", 350, "zero the velocity after this long without a v line")
	f.Int64Var(&o.deadlineNS, "deadline-ns", 10_000_000_000, "per-observation deadline")
	f.BoolVar(&o.confirm, "confirm-motion", false, "required: confirms drive energisation and operator-held jog")
	if err := f.Parse(args); err != nil {
		return o, err
	}
	if f.NArg() != 0 || o.binding.PairID == "" || o.binding.Revision == 0 || len(o.binding.ConfigurationSHA256) != 64 {
		return o, errors.New("socket, pair-id, nonzero pair-revision and a 64-hex configuration-sha256 are required")
	}
	if o.axis < 0 || o.axis >= 16 || !finite(o.maxFraction) || o.maxFraction <= 0 || o.maxFraction > 1.0 || o.staleMS < 50 || o.staleMS > 2000 || o.deadlineNS <= 0 {
		return o, errors.New("axis 0..15, max-fraction in (0,1], stale-ms 50..2000, positive deadline")
	}
	if !o.confirm {
		return o, errors.New("--confirm-motion is required: this energises the drive and jogs it while the operator holds a button")
	}
	return o, nil
}

type jogger struct {
	o    options
	c    *control.Client
	desc control.Description
	out  *bufio.Writer
	mu   sync.Mutex
	last control.Status

	velMu     sync.Mutex
	velocity  float64 // rad/s, signed
	velocityT time.Time
	homeEpoch uint64
	cfgEpoch  uint64

	// soft limit: newest observed position and its estimated velocity, from the
	// 50 ms status poll, extrapolated by the producer between polls
	posMu    sync.Mutex
	pos      float64
	posVel   float64
	posT     time.Time
	posValid bool
	limited  bool
}

// Unverified latency allowance for the soft limit: one status poll plus the
// adapter's publication delay. Measured 2026-09-17: at the +3 rad span end the
// lane refused outward input and the daemon faulted target_lead instead of
// holding, so lcdjog must stop the request before the boundary is reached.
const softLimitLatency = 0.06

// softLimitBlocks reports whether jogging at velocity v (rad/s, signed) from
// the extrapolated position would reach the span end before it could stop:
// margin = lead allowance + v^2/(2 a_jog) + |v| * latency.
func (j *jogger) softLimitBlocks(v float64, ax ipcclient.AxisDescription) bool {
	if v == 0 {
		return false
	}
	j.posMu.Lock()
	valid, p, pv, t := j.posValid, j.pos, j.posVel, j.posT
	j.posMu.Unlock()
	if !valid {
		return false
	}
	dt := time.Since(t).Seconds()
	if dt > 0.5 {
		dt = 0.5
	}
	est := p + pv*dt
	margin := ax.MaxTargetLead + v*v/(2*ax.JogAcceleration) + math.Abs(v)*softLimitLatency
	if v > 0 {
		return est >= ax.MaxPosition-margin
	}
	return est <= ax.MinPosition+margin
}

func (j *jogger) say(format string, a ...any) {
	j.mu.Lock()
	fmt.Fprintf(j.out, format+"\n", a...)
	j.out.Flush()
	j.mu.Unlock()
}

func (j *jogger) ctxT(ctx context.Context) (context.Context, context.CancelFunc) {
	return context.WithTimeout(ctx, time.Duration(j.o.deadlineNS))
}

func (j *jogger) command(ctx context.Context, op string, mask uint32) (control.Response, error) {
	cctx, cancel := j.ctxT(ctx)
	defer cancel()
	req := control.Request{Operation: op, AxisMask: mask}
	if op == "acquire" {
		req.Controller, req.Binding = j.o.controller, j.o.binding
	}
	return j.c.Command(cctx, &req)
}

func (j *jogger) status(ctx context.Context) (control.Status, error) {
	cctx, cancel := j.ctxT(ctx)
	defer cancel()
	s, err := j.c.Status(cctx)
	if err != nil {
		return s, err
	}
	if s.Core.TimeNS == 0 || len(s.Core.Axes) <= j.o.axis {
		return s, errors.New("missing native status")
	}
	j.last = s
	return s, nil
}

func (j *jogger) observe(ctx context.Context, pred func(control.Status) (bool, error)) error {
	cctx, cancel := j.ctxT(ctx)
	defer cancel()
	t := time.NewTicker(pollCadence)
	defer t.Stop()
	for {
		s, err := j.status(cctx)
		if err != nil {
			return err
		}
		ok, err := pred(s)
		if err != nil || ok {
			return err
		}
		select {
		case <-cctx.Done():
			return cctx.Err()
		case <-t.C:
		}
	}
}

// fault mirrors benchdrive's sessionFault: recovery names, masks and the
// CiA402 fault bit, so a refusal names the fault that caused it.
func fault(s control.Status) error {
	if s.Core.Recovery != nil && len(s.Core.Recovery.Faults) != 0 {
		var names []string
		for _, f := range s.Core.Recovery.Faults {
			names = append(names, f.Name)
		}
		return fmt.Errorf("fault %s", strings.Join(names, ","))
	}
	if s.Core.SafetyFaultMask != 0 || s.Core.ExecutionFaultReasons != 0 {
		return fmt.Errorf("fault mask=%d exec=%d", s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons)
	}
	for i, a := range s.Core.Axes {
		if a.Statusword&0x0008 != 0 || strings.Contains(a.Readiness, "fault") {
			return fmt.Errorf("axis %d fault %s sw=%04x", i, a.Readiness, a.Statusword)
		}
	}
	return nil
}

// jogReasonName renders ipcclient's JogReason enumeration for the panel; an
// unknown code is shown as its number rather than guessed.
func jogReasonName(code uint32) string {
	names := map[uint32]string{
		ipcclient.JogReasonClosed: "closed", ipcclient.JogReasonModeConflict: "mode_conflict",
		ipcclient.JogReasonNotReady: "not_ready", ipcclient.JogReasonWrongIdentity: "wrong_identity",
		ipcclient.JogReasonInvalidGeneration: "invalid_generation", ipcclient.JogReasonStaleSequence: "stale_sequence",
		ipcclient.JogReasonInvalidVector: "invalid_vector", ipcclient.JogReasonInvalidTiming: "invalid_timing",
		ipcclient.JogReasonExpired: "expired", ipcclient.JogReasonAuthorityLost: "authority_lost(fault)",
		ipcclient.JogReasonClockReset: "clock_reset", ipcclient.JogReasonGenerationExhausted: "generation_exhausted",
		ipcclient.JogReasonRamping: "ramping", ipcclient.JogReasonLimited: "limited",
		ipcclient.JogReasonOutsideLimitsOutward: "outside_limits",
	}
	if n, ok := names[code]; ok {
		return n
	}
	return fmt.Sprintf("reason_%d", code)
}

func inhibited(s control.Status) bool {
	return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.CommissioningPhase == 0
}

func (j *jogger) ready(s control.Status, mask uint32) error {
	if err := fault(s); err != nil {
		return err
	}
	if s.Core.HomeEpoch != j.homeEpoch || s.Core.ConfigurationEpoch != j.cfgEpoch {
		return errors.New("home or configuration epoch changed")
	}
	if j.c.Grant().Stopping || s.Core.LeaseValid != 1 || s.Core.CommissioningPhase != 0 || s.Core.AxisEnableMask != mask || s.Core.Armed != 1 {
		return errors.New("group not ready")
	}
	a := s.Core.Axes[j.o.axis]
	if a.Readiness != "ready" || a.Statusword&operationMask != operationWord || !a.HomeValid || !a.LogicalValid {
		return fmt.Errorf("axis readiness %s sw=%04x", a.Readiness, a.Statusword)
	}
	return nil
}

func (j *jogger) setVelocity(v float64) {
	j.velMu.Lock()
	j.velocity, j.velocityT = v, time.Now()
	j.velMu.Unlock()
}

func (j *jogger) currentVelocity() float64 {
	j.velMu.Lock()
	defer j.velMu.Unlock()
	if time.Since(j.velocityT) > time.Duration(j.o.staleMS)*time.Millisecond {
		return 0
	}
	return j.velocity
}

// run: describe, check, acquire, renew, enable, arm, open the jog lane, then
// produce until quit or a refusal. Cleanup always runs with fresh contexts so a
// cancelled parent cannot skip Stop and Release.
func (j *jogger) run(parent context.Context, lines <-chan string) (err error) {
	ctx, cancel := context.WithCancelCause(parent)
	defer cancel(nil)
	mask := uint32(1) << j.o.axis
	owned := false
	var session *control.JogSession
	defer func() {
		if session != nil {
			// zero the lane for a few frames, then End before Stop as benchdrive does
			for i := 0; i < 6; i++ {
				now := ipcclient.HostMonotonicNS()
				vec := make([]float64, len(j.desc.Axes))
				_ = session.UpdateAt(vec, now, now+inputNS)
				time.Sleep(jogCadence)
			}
			ectx, ecancel := context.WithTimeout(context.Background(), time.Duration(j.o.deadlineNS))
			err = errors.Join(err, session.End(ectx))
			ecancel()
		}
		j.c.StopRenewal()
		if owned {
			sctx, scancel := context.WithTimeout(context.Background(), time.Duration(j.o.deadlineNS))
			defer scancel()
			if _, e := j.c.Stop(sctx); e != nil {
				err = errors.Join(err, e)
			}
			if e := j.observe(sctx, func(s control.Status) (bool, error) { return inhibited(s), nil }); e != nil {
				err = errors.Join(err, e)
			}
			if _, e := j.c.Release(sctx); e != nil {
				err = errors.Join(err, e)
			}
			if e := j.observe(sctx, func(s control.Status) (bool, error) { return inhibited(s) && s.Core.LeaseValid == 0, nil }); e != nil {
				err = errors.Join(err, e)
			} else {
				j.say("STOPPED")
			}
		}
	}()

	dctx, dcancel := j.ctxT(ctx)
	j.desc, err = j.c.Describe(dctx)
	dcancel()
	if err != nil {
		return err
	}
	if j.o.axis >= len(j.desc.Axes) || j.desc.ConfigurationSHA256 != j.o.binding.ConfigurationSHA256 {
		return errors.New("axis outside Describe or configuration digest mismatch")
	}
	ax := j.desc.Axes[j.o.axis]
	if !finite(ax.MaxVelocity) || ax.MaxVelocity <= 0 || !finite(ax.JogAcceleration) || ax.JogAcceleration <= 0 || ax.PositionUnit != "rad" {
		return errors.New("axis lacks positive finite max_velocity, jog_acceleration or rad units")
	}
	s, err := j.status(ctx)
	if err != nil {
		return err
	}
	if !inhibited(s) || fault(s) != nil {
		return errors.New("stack is not inhibited and fault-free")
	}
	a := s.Core.Axes[j.o.axis]
	if !a.HomeValid || !a.LogicalValid || s.Core.HomeValidMask&mask == 0 {
		return fmt.Errorf("axis %d: Home or coordinates missing (%s)", j.o.axis, a.Readiness)
	}
	j.homeEpoch, j.cfgEpoch = s.Core.HomeEpoch, s.Core.ConfigurationEpoch

	if _, err = j.command(ctx, "acquire", 0); err != nil {
		return err
	}
	owned = true
	if j.c.Grant().Session == "" || j.c.Grant().Generation == 0 {
		return errors.New("acquire returned no grant")
	}
	updates, err := j.c.StartRenewal(ctx, renewCadence)
	if err != nil {
		return err
	}
	go func() {
		for u := range updates {
			if u.Err != nil {
				cancel(fmt.Errorf("renewal: %w", u.Err))
				return
			}
			if u.Grant.Stopping {
				cancel(errors.New("renewal reports stopping"))
				return
			}
		}
	}()
	if _, err = j.command(ctx, "enable", mask); err != nil {
		return err
	}
	if err = j.observe(ctx, func(s control.Status) (bool, error) {
		if s.Core.SafetyFaultMask != 0 {
			return false, errors.New("enable safety fault")
		}
		return s.Core.AxisEnableMask == mask && s.Core.CommissioningPhase == 0, nil
	}); err != nil {
		return err
	}
	if _, err = j.command(ctx, "arm", 0); err != nil {
		return err
	}
	if err = j.observe(ctx, func(s control.Status) (bool, error) {
		if f := fault(s); f != nil {
			return false, f
		}
		return j.ready(s, mask) == nil, nil
	}); err != nil {
		return err
	}

	begin := ipcclient.HostMonotonicNS()
	sctx, scancel := j.ctxT(ctx)
	session, err = j.c.NewJogSession(sctx, mask, begin+inputNS)
	scancel()
	if err != nil {
		return err
	}
	jctx, jcancel := j.ctxT(ctx)
	initial, err := j.c.JogStatus(jctx)
	jcancel()
	if err != nil {
		return err
	}
	if initial.JogGeneration == 0 || initial.Open != 1 || initial.ControlAccepted != 1 {
		return errors.New("begin_jog lacks an accepted open observation")
	}
	j.say("READY %g %g", ax.MaxVelocity, ax.JogAcceleration)

	// producer: one frame every 5 ms with whatever the panel last asked for
	prodDone := make(chan error, 1)
	go func() {
		t := time.NewTicker(jogCadence)
		defer t.Stop()
		cap := j.o.maxFraction * ax.MaxVelocity
		for {
			select {
			case <-ctx.Done():
				prodDone <- nil
				return
			case <-t.C:
			}
			v := j.currentVelocity()
			if !finite(v) || math.Abs(v) > cap {
				prodDone <- errors.New("velocity exceeds the fraction bound")
				return
			}
			if j.softLimitBlocks(v, ax) {
				v = 0
				if !j.limited {
					j.limited = true
					j.say("LIMIT")
				}
			} else if j.limited && v != 0 {
				j.limited = false
			}
			vec := make([]float64, len(j.desc.Axes))
			vec[j.o.axis] = v
			now := ipcclient.HostMonotonicNS()
			if e := session.Update(vec, now+inputNS); e != nil && !errors.Is(e, control.ErrJogWouldBlock) {
				prodDone <- e
				return
			}
		}
	}()

	// observer and stdin: refusals, faults and identity changes end the run
	poll := time.NewTicker(pollCadence)
	defer poll.Stop()
	limited := false
	for {
		select {
		case <-ctx.Done():
			return context.Cause(ctx)
		case e := <-prodDone:
			return e
		case line, ok := <-lines:
			if !ok {
				return nil // stdin closed: the panel went away
			}
			fields := strings.Fields(line)
			if len(fields) == 0 {
				continue
			}
			switch fields[0] {
			case "q":
				return nil
			case "v":
				if len(fields) != 2 {
					continue
				}
				f, e := strconv.ParseFloat(fields[1], 64)
				if e != nil || !finite(f) || math.Abs(f) > j.o.maxFraction {
					j.say("ERR bad v %q", fields[1])
					j.setVelocity(0)
					continue
				}
				j.setVelocity(f * ax.MaxVelocity)
			}
		case <-poll.C:
			s, e := j.status(ctx)
			if e != nil {
				return e
			}
			if e = j.ready(s, mask); e != nil {
				return e
			}
			jo := s.Jog
			if jo == nil || jo.JogGeneration != initial.JogGeneration || jo.AppGrantGeneration != initial.AppGrantGeneration {
				return errors.New("jog observation identity changed")
			}
			if jo.UpdateReason != ipcclient.JogReasonNone || (jo.StateReason != ipcclient.JogReasonNone && jo.StateReason != ipcclient.JogReasonLimited) || jo.Open != 1 {
				reason := jo.StateReason
				if jo.UpdateReason != ipcclient.JogReasonNone {
					reason = jo.UpdateReason
				}
				j.say("REJECTED %s", jogReasonName(reason))
				return errors.New("native jog rejected")
			}
			if (jo.StateReason == ipcclient.JogReasonLimited) != limited {
				limited = !limited
				if limited {
					j.say("LIMIT")
				}
			}
			ax := s.Core.Axes[j.o.axis]
			now := time.Now()
			j.posMu.Lock()
			if j.posValid {
				if dt := now.Sub(j.posT).Seconds(); dt > 0.01 {
					j.posVel = (ax.LogicalPosition - j.pos) / dt
				}
			}
			j.pos, j.posT, j.posValid = ax.LogicalPosition, now, ax.LogicalValid
			j.posMu.Unlock()
			j.say("POS %.6f %d", ax.LogicalPosition, ax.Statusword)
		}
	}
}

func main() {
	o, err := parse(os.Args[1:])
	if errors.Is(err, flag.ErrHelp) {
		os.Exit(0)
	}
	out := bufio.NewWriter(os.Stdout)
	if err != nil {
		fmt.Fprintln(out, "ERR", err)
		out.Flush()
		os.Exit(2)
	}
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()
	c, err := control.Dial(control.UnixPath(o.socket))
	if err != nil {
		fmt.Fprintln(out, "ERR", err)
		out.Flush()
		os.Exit(4)
	}
	defer c.Close()
	lines := make(chan string)
	go func() {
		sc := bufio.NewScanner(os.Stdin)
		for sc.Scan() {
			lines <- sc.Text()
		}
		close(lines)
	}()
	j := &jogger{o: o, c: c, out: out}
	if err := j.run(ctx, lines); err != nil {
		j.say("ERR %v", err)
		os.Exit(1)
	}
}
