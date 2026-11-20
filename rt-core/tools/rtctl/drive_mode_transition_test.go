package rtctl

import (
	"bytes"
	"context"
	"fmt"
	"strings"
	"testing"
)

// These command responses are a simulated stopped A6-EC drive, not evidence
// that a real drive has changed mode or been power-cycled.
func TestDriveModeTransitionApplyAndVerify(t *testing.T) {
	mode, writes := int64(4), 0
	fake := func(_ context.Context, command string, args ...string) (string, error) {
		line := command + " " + strings.Join(args, " ")
		switch {
		case strings.Contains(line, "-p LoadState"):
			return "loaded\n", nil
		case strings.HasPrefix(line, "systemctl show "):
			return "inactive\n", nil
		case strings.HasPrefix(line, "ethercat master "):
			return "Master0\n  Phase: Idle\n  Active: no\n", nil
		case strings.HasPrefix(line, "ethercat slaves "):
			return "=== Master 0, Slave 0 ===\nState: PREOP\nVendor Id: 0x00400000\nProduct code: 0x00000715\n", nil
		case strings.Contains(line, "upload") && strings.HasSuffix(line, "0x6041 0"):
			return "0x0040 64\n", nil
		case strings.Contains(line, "upload") && strings.HasSuffix(line, "0x2000 0x08"):
			return fmt.Sprintf("0x%04x %d\n", mode, mode), nil
		case strings.Contains(line, "download") && strings.HasSuffix(line, "0x2000 0x08 1"):
			writes++
			mode = 1
			return "", nil
		default:
			return "", fmt.Errorf("unexpected %s", line)
		}
	}
	task := driveModeTransition{slave: 0, phase: "apply", direction: "to-linear", run: fake, effective: func() int { return 0 }}
	var out bytes.Buffer
	if err := task.execute(context.Background(), &out); err != nil || writes != 1 || !strings.Contains(out.String(), "power_cycle_required=true") {
		t.Fatalf("apply err=%v writes=%d output=%s", err, writes, &out)
	}
	out.Reset()
	if err := task.execute(context.Background(), &out); err != nil || writes != 1 || !strings.Contains(out.String(), "power_cycle_required=false") {
		t.Fatalf("repeat err=%v writes=%d output=%s", err, writes, &out)
	}
	task.phase = "verify"
	if err := task.execute(context.Background(), &out); err != nil || writes != 1 {
		t.Fatalf("verify err=%v writes=%d", err, writes)
	}
}

func TestDriveModeTransitionRefusesUnsafeState(t *testing.T) {
	for _, test := range []struct{ name, state, phase, status string }{
		{"active unit", "active", "Idle", "0x0040 64\n"},
		{"owned master", "inactive", "Operation", "0x0040 64\n"},
		{"enabled drive", "inactive", "Idle", "0x0027 39\n"},
	} {
		t.Run(test.name, func(t *testing.T) {
			writes := 0
			task := driveModeTransition{slave: 0, phase: "apply", direction: "to-linear", effective: func() int { return 0 }}
			task.run = func(_ context.Context, command string, args ...string) (string, error) {
				line := command + " " + strings.Join(args, " ")
				switch {
				case strings.Contains(line, "-p LoadState"):
					return "loaded\n", nil
				case strings.HasPrefix(line, "systemctl show "):
					return test.state, nil
				case strings.HasPrefix(line, "ethercat master "):
					return "Master0\n  Phase: " + test.phase + "\n  Active: no\n", nil
				case strings.HasPrefix(line, "ethercat slaves "):
					return "=== Master 0, Slave 0 ===\nState: PREOP\nVendor Id: 0x00400000\nProduct code: 0x00000715\n", nil
				case strings.Contains(line, "upload") && strings.HasSuffix(line, "0x6041 0"):
					return test.status, nil
				case strings.Contains(line, "download"):
					writes++
				}
				return "0x0004 4\n", nil
			}
			if err := task.execute(context.Background(), &bytes.Buffer{}); err == nil || writes != 0 {
				t.Fatalf("unsafe state not refused: err=%v writes=%d", err, writes)
			}
		})
	}
}
