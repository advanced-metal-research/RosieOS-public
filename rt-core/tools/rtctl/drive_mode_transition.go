package rtctl

import (
	"context"
	"flag"
	"fmt"
	"io"
	"os"
	"strconv"
	"strings"
)

// driveModeTransition changes only the A6-EC C00.07 tracking mode while
// the controller and EtherCAT master are stopped. It cannot power-cycle a
// drive, start a controller, or validate a Home reference.
type driveModeTransition struct {
	slave     int
	phase     string
	direction string
	run       func(context.Context, string, ...string) (string, error)
	effective func() int
}

// RunDriveModeTransitionCLI applies or verifies one reviewed A6-EC mode change.
func RunDriveModeTransitionCLI(args []string, stdout, stderr io.Writer) int {
	fs := flag.NewFlagSet("drive-mode-transition", flag.ContinueOnError)
	fs.SetOutput(stderr)
	slave := fs.Int("slave", -1, "absolute EtherCAT slave position")
	phase := fs.String("phase", "", "apply or verify")
	direction := fs.String("direction", "", "to-linear (4 to 1) or restore-rotary (1 to 4)")
	if err := fs.Parse(args); err != nil {
		return 2
	}
	t := driveModeTransition{slave: *slave, phase: *phase, direction: *direction, effective: os.Geteuid}
	t.run = func(ctx context.Context, command string, argv ...string) (string, error) {
		if command != "ethercat" && command != "systemctl" {
			return "", fmt.Errorf("unsupported command %s", command)
		}
		read := runInventoryRead(ctx, command, argv...)
		if read.ExitCode != 0 || read.Error != "" || read.TimedOut {
			return "", fmt.Errorf("%s %s: exit=%d %s %s", command, strings.Join(argv, " "), read.ExitCode, read.Error, strings.TrimSpace(read.Stderr))
		}
		return read.Stdout, nil
	}
	if fs.NArg() != 0 {
		fmt.Fprintln(stderr, "drive-mode-transition rejects positional arguments")
		return 2
	}
	if err := t.execute(context.Background(), stdout); err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	return 0
}

func (t driveModeTransition) execute(ctx context.Context, out io.Writer) error {
	if t.slave < 0 || t.slave > 65535 || (t.phase != "apply" && t.phase != "verify") || (t.direction != "to-linear" && t.direction != "restore-rotary") {
		return fmt.Errorf("require --slave 0..65535 --phase apply|verify --direction to-linear|restore-rotary")
	}
	if t.effective() != 0 {
		return fmt.Errorf("drive-mode-transition requires root")
	}
	// Never rely on a stale earlier check. Repeat both unit and bus checks
	// immediately before the only possible write.
	check := func() error {
		for _, unit := range []string{"rosie-rt-control.service", "rosie-rt-core.service"} {
			load, err := t.run(ctx, "systemctl", "show", unit, "-p", "LoadState", "--value")
			if err != nil || strings.TrimSpace(load) != "loaded" {
				return fmt.Errorf("%s is not proven installed: %q: %v", unit, strings.TrimSpace(load), err)
			}
			state, err := t.run(ctx, "systemctl", "show", unit, "-p", "ActiveState", "--value")
			if err != nil || strings.TrimSpace(state) != "inactive" {
				return fmt.Errorf("%s is not proven inactive: %q: %v", unit, strings.TrimSpace(state), err)
			}
		}
		r := encoderRecovery{slave: strconv.Itoa(t.slave), run: func(ctx context.Context, argv ...string) (string, error) {
			return t.run(ctx, "ethercat", argv...)
		}}
		if err := r.idle(ctx); err != nil {
			return err
		}
		ready, err := r.preop(ctx)
		if err != nil || !ready {
			return fmt.Errorf("A6-EC slave %d is not proven PREOP: %v", t.slave, err)
		}
		status, err := r.upload(ctx, "uint16", "0x6041", "0")
		if err != nil {
			return err
		}
		switch status & 0x6f {
		case 0, 0x08, 0x21, 0x40:
		default:
			return fmt.Errorf("slave %d is not disabled: statusword 0x%04x", t.slave, status)
		}
		return nil
	}
	if err := check(); err != nil {
		return err
	}
	from, to := int64(4), int64(1)
	if t.direction == "restore-rotary" {
		from, to = to, from
	}
	r := encoderRecovery{slave: strconv.Itoa(t.slave), run: func(ctx context.Context, argv ...string) (string, error) {
		return t.run(ctx, "ethercat", argv...)
	}}
	current, err := r.upload(ctx, "uint16", "0x2000", "0x08")
	if err != nil {
		return err
	}
	if t.phase == "verify" {
		if current != to {
			return fmt.Errorf("slave %d C00.07 is %d, expected %d", t.slave, current, to)
		}
		fmt.Fprintf(out, "drive_mode slave=%d mode=%d phase=verify readback=1\n", t.slave, current)
		return nil
	}
	if current != from && current != to {
		return fmt.Errorf("slave %d C00.07 is %d, expected %d or %d; no write", t.slave, current, from, to)
	}
	if current == from {
		if err := check(); err != nil {
			return err
		}
		if _, err := t.run(ctx, "ethercat", "download", "--master", "0", "--position", strconv.Itoa(t.slave), "--type", "uint16", "0x2000", "0x08", strconv.FormatInt(to, 10)); err != nil {
			return err
		}
	}
	observed, err := r.upload(ctx, "uint16", "0x2000", "0x08")
	if err != nil {
		return err
	}
	if observed != to {
		return fmt.Errorf("slave %d C00.07 readback %d, expected %d", t.slave, observed, to)
	}
	fmt.Fprintf(out, "drive_mode slave=%d old=%d new=%d phase=apply readback=1 power_cycle_required=%t\n", t.slave, current, observed, current != to)
	return nil
}
