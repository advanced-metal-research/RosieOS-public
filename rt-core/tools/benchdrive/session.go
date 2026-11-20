//go:build linux

package main

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"io"
	"os"
	"strconv"
	"strings"
	"syscall"
	"time"
	"unicode/utf8"

	"rosieos/rt-core/sdk/control"
)

// Unverified operator input ceiling: 4096 bytes per command, including newline.
const sessionLineBytes = 4096

// sessionFault preserves the native recovery names and bit masks, preventing
// a generic authority failure from hiding the fault that inhibited the session.
func sessionFault(s control.Status) error {
	if s.Core.Recovery != nil && len(s.Core.Recovery.Faults) != 0 {
		var faults []string
		for _, fault := range s.Core.Recovery.Faults {
			faults = append(faults, fmt.Sprintf("%s (bit=%d axis_mask=%d)", fault.Name, fault.Bit, fault.AxisMask))
		}
		return fmt.Errorf("session faults: %s", strings.Join(faults, ", "))
	}
	if s.Core.SafetyFaultMask != 0 || s.Core.ExecutionFaultReasons != 0 {
		return fmt.Errorf("session fault: safety_fault_mask=%d execution_fault_reasons=%d", s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons)
	}
	for i, a := range s.Core.Axes {
		// CiA402 statusword bit 3 denotes Fault; unverified on other profiles.
		if a.Statusword&0x0008 != 0 || strings.Contains(a.Readiness, "fault") {
			return fmt.Errorf("session axis %d fault: readiness=%s statusword=0x%04x", i, a.Readiness, a.Statusword)
		}
	}
	return nil
}

// sessionHealth rejects faults on any axis and unexpected native generations,
// preventing an external Stop from being silently followed by another move.
func (r *runner) sessionHealth(s control.Status) error {
	if err := sessionFault(s); err != nil {
		return err
	}
	if !r.sessionTransition && (s.Core.ControlGeneration != r.sessionGeneration || r.c.Grant().Stopping || s.Core.LeaseValid != 1) {
		return errors.New("session authority changed or external Stop observed")
	}
	return nil
}

type sessionLine struct {
	text string
	err  error
}

// session owns one renewed grant and one cancellable input reader. Idle input
// still polls native status; all failures return to run's Stop/Release cleanup.
func (r *runner) session(ctx context.Context, cancelRun context.CancelCauseFunc) error {
	if r.in == nil {
		return errors.New("session has no input")
	}
	r.sessionActive, r.sessionGeneration = true, r.last.Core.ControlGeneration
	defer func() { r.sessionActive = false }()
	if _, err := fmt.Fprintln(r.out, "Session commands (axis indices must belong to --axis-mask):\nmove axis:rad[,axis:rad...] [velocity-fraction] [accel-fraction]\njog axis direction fraction duration_ns\nstop\nstatus\nhome mask\nquit"); err != nil {
		return err
	}
	input, err := cancellableSessionInput(r.in)
	if err != nil {
		return err
	}
	r.in = input
	ctx, cancelInput := context.WithCancel(ctx)
	lines, readerDone := make(chan sessionLine), make(chan struct{})
	go func() {
		defer close(readerDone)
		scanner := bufio.NewScanner(r.in)
		scanner.Buffer(make([]byte, sessionLineBytes), sessionLineBytes)
		for scanner.Scan() {
			select {
			case lines <- sessionLine{text: scanner.Text()}:
			case <-ctx.Done():
				return
			}
		}
		err := scanner.Err()
		if err == nil {
			err = io.EOF
		}
		select {
		case lines <- sessionLine{err: err}:
		case <-ctx.Done():
		}
	}()
	defer func() {
		cancelInput()
		r.in.Close()
		<-readerDone
	}()
	ticker := time.NewTicker(pollCadence)
	defer ticker.Stop()
	armed := true
	for {
		select {
		case <-ctx.Done():
			return context.Cause(ctx)
		case <-ticker.C:
			if ctx.Err() != nil {
				return context.Cause(ctx)
			}
			if _, err := r.status(ctx); err != nil {
				if ctx.Err() != nil {
					return context.Cause(ctx)
				}
				return err
			}
		case line := <-lines:
			if ctx.Err() != nil {
				return context.Cause(ctx)
			}
			var err error
			quit := false
			if line.err != nil {
				quit = errors.Is(line.err, io.EOF)
				if !quit {
					err = fmt.Errorf("unreadable session line: %w", line.err)
				}
			} else if _, err = r.status(ctx); err == nil {
				quit, err = r.sessionCommand(ctx, cancelRun, line.text, &armed)
			}
			_, class := outcome(err)
			rec := record{Kind: "session_command", Operation: "session", Request: map[string]any{"line": line.text, "eof": errors.Is(line.err, io.EOF)}, Response: map[string]any{"class": class}}
			if err != nil {
				rec.Error = err.Error()
			}
			err = errors.Join(err, r.e.write(rec))
			if err != nil || quit {
				return err
			}
			if _, err := fmt.Fprintf(r.out, "ok %s\n", line.text); err != nil {
				return err
			}
		}
	}
}

// cancellableSessionInput reopens inherited file descriptors in Go's poller.
// Close then interrupts an idle terminal/pipe Read, preventing reader shutdown
// from blocking Stop/Release. A separate file description preserves shell flags.
func cancellableSessionInput(in io.ReadCloser) (io.ReadCloser, error) {
	file, ok := in.(*os.File)
	if !ok {
		return in, nil
	}
	offset, seekErr := file.Seek(0, io.SeekCurrent)
	reader, err := os.OpenFile(fmt.Sprintf("/proc/self/fd/%d", file.Fd()), os.O_RDONLY|syscall.O_NONBLOCK, 0)
	if err != nil {
		return nil, err
	}
	if seekErr == nil {
		if _, err := reader.Seek(offset, io.SeekStart); err != nil {
			reader.Close()
			return nil, err
		}
	}
	if err := file.Close(); err != nil {
		reader.Close()
		return nil, err
	}
	return reader, nil
}

// sessionStop joins renewal before the owned Stop changes the fence. Restarting
// renewal afterward avoids treating our own transient stopping grant as a loss.
func (r *runner) sessionStop(ctx context.Context, cancelRun context.CancelCauseFunc) error {
	r.stopRenewal()
	r.sessionTransition = true
	err := r.stop(ctx)
	r.sessionTransition = false
	if err != nil {
		return err
	}
	r.sessionGeneration = r.last.Core.ControlGeneration
	return r.startRenewal(ctx, cancelRun)
}

// sessionCommand validates a whole line and mask before motion. It reuses the
// standalone profiles and observations and restores defaults after each command.
func (r *runner) sessionCommand(ctx context.Context, cancelRun context.CancelCauseFunc, line string, armed *bool) (bool, error) {
	if !utf8.ValidString(line) || strings.ContainsAny(line, "\x00\r") {
		return false, errors.New("unreadable session command")
	}
	fields := strings.Fields(line)
	if len(fields) == 0 {
		return false, errors.New("empty session command")
	}
	saved := r.o
	defer func() { r.o = saved }()
	rearm := func() error {
		if !*armed {
			if err := r.preEnable(r.last); err != nil {
				return err
			}
			if err := r.enable(ctx); err != nil {
				return err
			}
			*armed = true
		}
		return nil
	}
	switch fields[0] {
	case "quit", "status", "stop":
		if len(fields) != 1 {
			return false, errors.New("command takes no arguments")
		}
		switch fields[0] {
		case "quit":
			return true, nil
		case "stop":
			err := r.sessionStop(ctx, cancelRun)
			*armed = false
			return false, err
		default:
			for i, a := range r.last.Core.Axes {
				if _, err := fmt.Fprintf(r.out, "axis %d readiness=%s logical_position=%.9g home_valid=%t statusword=0x%04x\n", i, a.Readiness, a.LogicalPosition, a.HomeValid, a.Statusword); err != nil {
					return false, err
				}
			}
			return false, nil
		}
	case "move":
		if len(fields) < 2 || len(fields) > 4 {
			return false, errors.New("expected move axis:rad[,axis:rad...] [velocity-fraction] [accel-fraction]")
		}
		moves, err := parseMoveSpecs(fields[1])
		if err != nil {
			return false, err
		}
		for _, m := range moves {
			if r.o.mask&(1<<m.axis) == 0 {
				return false, errors.New("move axis outside session mask")
			}
		}
		for i, target := range []*float64{&r.o.velocityFraction, &r.o.accelFraction} {
			if len(fields) > i+2 {
				*target, err = strconv.ParseFloat(fields[i+2], 64)
				if err != nil || !finite(*target) || *target <= 0 || *target > maxFraction {
					return false, errors.New("move fractions must be in (0, 0.5]")
				}
			}
		}
		r.o.moves = moves
		if err := rearm(); err != nil {
			return false, err
		}
		return false, r.move(ctx)
	case "jog":
		if len(fields) != 5 {
			return false, errors.New("expected jog axis direction fraction duration_ns")
		}
		axis, e1 := strconv.Atoi(fields[1])
		direction, e2 := strconv.Atoi(fields[2])
		fraction, e3 := strconv.ParseFloat(fields[3], 64)
		duration, e4 := strconv.ParseInt(fields[4], 10, 64)
		if errors.Join(e1, e2, e3, e4) != nil || axis < 0 || axis >= len(r.description.Axes) || r.o.mask&(1<<axis) == 0 ||
			(direction != 1 && direction != -1) || !finite(fraction) || fraction <= 0 || fraction > maxFraction || duration <= 0 || duration > maxJogDuration {
			return false, errors.New("invalid or unbounded session jog")
		}
		r.o.axis, r.o.direction, r.o.fraction, r.o.jogNS = axis, direction, fraction, duration
		if err := r.jogLimits(r.last); err != nil {
			return false, err
		}
		if err := rearm(); err != nil {
			return false, err
		}
		r.expectations["jog_input_observed"] = false
		return false, r.jog(ctx, cancelRun)
	case "home":
		if len(fields) != 2 {
			return false, errors.New("expected home mask")
		}
		mask, err := strconv.ParseUint(fields[1], 10, 64)
		if err != nil || mask == 0 || mask & ^r.o.mask != 0 {
			return false, errors.New("home mask outside session mask")
		}
		if err := r.sessionStop(ctx, cancelRun); err != nil {
			return false, err
		}
		*armed = false
		r.o.mask = mask
		if err := r.home(ctx); err != nil {
			return false, err
		}
		r.o.mask = saved.mask
		return false, rearm()
	default:
		return false, errors.New("unknown session command")
	}
}
