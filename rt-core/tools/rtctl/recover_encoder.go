package rtctl

import (
	"context"
	"flag"
	"fmt"
	"io"
	"strconv"
	"strings"
	"time"
)

// Unverified hardware bounds: 500 ms encoder settle, 100 ms poll cadence,
// 10 s reset convergence. Successful readbacks, never elapsed time, admit steps.
const (
	encoderSettle       = 500 * time.Millisecond
	encoderPoll         = 100 * time.Millisecond
	encoderResetTimeout = 10 * time.Second
)

type encoderRecovery struct {
	slave string
	out   io.Writer
	run   func(context.Context, ...string) (string, error)
	wait  func(context.Context, time.Duration) error
}

// RunRecoverEncoderCLI performs only the explicit stopped-master A6 recovery.
// It never starts a daemon or Home, and refuses unknown alarms before more writes.
func RunRecoverEncoderCLI(args []string, stdout, stderr io.Writer) int {
	flags := flag.NewFlagSet("recover-encoder", flag.ContinueOnError)
	flags.SetOutput(stderr)
	slave := flags.Int("slave", -1, "absolute slave position on master 0")
	backend := flags.String("backend", "", "ethercat")
	if err := flags.Parse(args); err != nil {
		return 2
	}
	if flags.NArg() != 0 || *slave < 0 || *slave > 65535 || *backend != "ethercat" {
		fmt.Fprintln(stderr, "recover-encoder requires --slave 0..65535 --backend ethercat")
		return 2
	}
	r := encoderRecovery{slave: strconv.Itoa(*slave), out: stdout, wait: encoderWait}
	r.run = func(ctx context.Context, args ...string) (string, error) {
		read := runInventoryRead(ctx, "ethercat", args...)
		if read.ExitCode != 0 || read.Error != "" || read.TimedOut {
			return "", fmt.Errorf("ethercat %s: exit=%d %s %s", strings.Join(args, " "), read.ExitCode, read.Error, strings.TrimSpace(read.Stderr))
		}
		return read.Stdout, nil
	}
	if err := r.recover(context.Background()); err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	return 0
}

func encoderWait(ctx context.Context, interval time.Duration) error {
	timer := time.NewTimer(interval)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}

// idle checks the kernel master phase before every mutation; a daemon's requested
// master is Operation even before activation. Unknown/duplicate fields fail closed.
func (r encoderRecovery) idle(ctx context.Context) error {
	output, err := r.run(ctx, "master", "--master", "0")
	if err != nil {
		return err
	}
	counts := map[string]int{}
	for _, line := range strings.Split(output, "\n") {
		line = strings.TrimSpace(line)
		if line == "Master0" {
			counts["master"]++
		}
		if strings.HasPrefix(line, "Phase:") {
			if strings.TrimSpace(strings.TrimPrefix(line, "Phase:")) != "Idle" {
				return fmt.Errorf("master 0 is owned or unavailable; stop the daemon before encoder recovery")
			}
			counts["phase"]++
		}
		if strings.HasPrefix(line, "Active:") {
			if strings.TrimSpace(strings.TrimPrefix(line, "Active:")) != "no" {
				return fmt.Errorf("master 0 remains active")
			}
			counts["active"]++
		}
	}
	if counts["master"] != 1 || counts["phase"] != 1 || counts["active"] != 1 {
		return fmt.Errorf("cannot establish idle master 0 from ethercat output")
	}
	return nil
}

func (r encoderRecovery) upload(ctx context.Context, kind, index, sub string) (int64, error) {
	output, err := r.run(ctx, "upload", "--master", "0", "--position", r.slave, "--type", kind, index, sub)
	if err != nil {
		return 0, err
	}
	return inventoryNumber(output, kind)
}

func (r encoderRecovery) alarms(ctx context.Context, step string) ([2]int64, error) {
	var pair [2]int64
	var err error
	pair[0], err = r.upload(ctx, "uint16", "0x603f", "0")
	if err != nil {
		return pair, err
	}
	// The manual documents 0x203F as UInt32, but the mini bench AS715N
	// (revision 0x2ef8) answers 2 bytes (measured 2026-10-28), so a typed
	// uint32 upload refused before any write. Read the octet string and accept
	// exactly 2 or 4 little-endian bytes.
	raw, err := r.run(ctx, "upload", "--master", "0", "--position", r.slave, "--type", "octet_string", "0x203f", "0")
	if err != nil {
		return pair, err
	}
	if len(raw) != 2 && len(raw) != 4 {
		return pair, fmt.Errorf("0x203F returned %d bytes; expected 2 or 4", len(raw))
	}
	for i := len(raw) - 1; i >= 0; i-- {
		pair[1] = pair[1]<<8 | int64(raw[i])
	}
	if err == nil {
		fmt.Fprintf(r.out, "%s: 0x603F=0x%04X 0x203F=0x%04X\n", step, pair[0], pair[1])
	}
	return pair, err
}

// preop verifies exactly the addressed A6 identity and a non-operational bus
// state; recovery must not reset an unrelated family or any enabled slave.
func (r encoderRecovery) preop(ctx context.Context) (bool, error) {
	output, err := r.run(ctx, "slaves", "--master", "0", "--position", r.slave, "--verbose")
	if err != nil {
		return false, err
	}
	identities := inventorySIIIdentities(output)
	n, _ := strconv.ParseUint(r.slave, 10, 16)
	id, ok := identities[uint32(n)]
	if len(identities) != 1 || !ok || id.VendorID == nil || id.ProductCode == nil || *id.VendorID != 4194304 || *id.ProductCode != 1813 {
		return false, fmt.Errorf("slave %s is not an identified A6-EC drive", r.slave)
	}
	states := []string{}
	for _, line := range strings.Split(output, "\n") {
		if strings.HasPrefix(strings.TrimSpace(line), "State:") {
			states = append(states, strings.TrimSpace(strings.TrimPrefix(strings.TrimSpace(line), "State:")))
		}
	}
	if len(states) != 1 {
		return false, fmt.Errorf("ambiguous slave state")
	}
	switch states[0] {
	case "PREOP":
		return true, nil
	case "INIT":
		return false, nil
	default:
		return false, fmt.Errorf("unexpected slave state %s; require PREOP", states[0])
	}
}

func (r encoderRecovery) write(ctx context.Context, sub, value string) error {
	if err := r.idle(ctx); err != nil {
		return err
	}
	ready, err := r.preop(ctx)
	if err != nil {
		return err
	}
	if !ready {
		return fmt.Errorf("slave must be PREOP before encoder recovery write")
	}
	sw, err := r.upload(ctx, "uint16", "0x6041", "0")
	if err != nil {
		return err
	}
	// CiA402 status mask: switched-on/operation-enabled/quick-stop and fault
	// reaction states may still apply torque. Only disabled/ready/fault are admitted.
	state := sw & 0x6f
	if state != 0x40 && state != 0x21 && state != 0x08 && state != 0 {
		return fmt.Errorf("slave is not disabled: statusword 0x%04X", sw)
	}
	_, err = r.run(ctx, "download", "--master", "0", "--position", r.slave, "--type", "uint16", "0x2031", sub, value)
	return err
}

func (r encoderRecovery) recover(ctx context.Context) error {
	if err := r.idle(ctx); err != nil {
		return err
	}
	ready, err := r.preop(ctx)
	if err != nil {
		return err
	}
	if !ready {
		return fmt.Errorf("initial slave state must be PREOP")
	}
	pair, err := r.alarms(ctx, "initial")
	if err != nil {
		return err
	}
	// Measured mini bench 2026-09-15: Er20.8 -> Er20.8/ALF9.0 -> ALF9.0 -> clear.
	er208, alf90, clear := [2]int64{0x7305, 0x0208}, [2]int64{0x7305, 0x0f90}, [2]int64{}
	if pair != er208 {
		return fmt.Errorf("encoder recovery requires Er20.8 (0x7305/0x0208); observed %X", pair)
	}
	// Reset changes the coordinate origin even if a later step fails.
	fmt.Fprintln(r.out, "Encoder reset invalidates saved Home anchors. Home is required before Enable; do not restore an old anchor.")
	if err := r.write(ctx, "0x11", "4"); err != nil {
		return err
	}
	if err := r.wait(ctx, encoderSettle); err != nil {
		return err
	}
	pair, err = r.alarms(ctx, "F31.10")
	if err != nil {
		return err
	}
	if pair != er208 && pair != alf90 {
		return fmt.Errorf("unexpected alarm after F31.10: %X", pair)
	}
	result, err := r.upload(ctx, "uint16", "0x2031", "0x11")
	if err != nil {
		return err
	}
	if result != 0 && result != 4 {
		return fmt.Errorf("encoder reset unsuccessful: F31.10=%d", result)
	}
	if err := r.write(ctx, "0x02", "1"); err != nil {
		return err
	}
	resetCtx, cancel := context.WithTimeout(ctx, encoderResetTimeout)
	defer cancel()
	// F31.01 restarts the drive: measured mini bench 2026-10-28, the link went
	// down and the master saw 0 slaves for ~3.5 s before PREOP returned. Slave
	// and SDO read failures inside this window are retried until the bound;
	// a master owner or an unexpected alarm pair still stops at once.
	var lost error
	for {
		if err := r.wait(resetCtx, encoderPoll); err != nil {
			return fmt.Errorf("software reset did not converge: %w (last: %v)", err, lost)
		}
		if err := r.idle(resetCtx); err != nil {
			return err
		}
		ready, lost = r.preop(resetCtx)
		if lost != nil || !ready {
			continue
		}
		pair, lost = r.alarms(resetCtx, "F31.01 PREOP")
		if lost != nil {
			continue
		}
		if pair == alf90 || pair == clear {
			break
		}
		if pair != er208 {
			return fmt.Errorf("unexpected alarm after F31.01: %X", pair)
		}
	}
	if err := r.write(ctx, "0x01", "1"); err != nil {
		return err
	}
	clearCtx, clearCancel := context.WithTimeout(ctx, encoderResetTimeout)
	defer clearCancel()
	for {
		pair, err = r.alarms(clearCtx, "F31.00")
		if err != nil {
			return err
		}
		if pair == clear {
			break
		}
		if pair != alf90 {
			return fmt.Errorf("unexpected alarm after F31.00: %X", pair)
		}
		if err := r.wait(clearCtx, encoderPoll); err != nil {
			return fmt.Errorf("fault reset did not clear: %w", err)
		}
	}
	fmt.Fprintln(r.out, "Encoder recovery complete. Home is required before Enable; do not restore an old anchor.")
	return nil
}
