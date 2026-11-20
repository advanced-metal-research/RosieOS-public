//go:build linux

package rtctl

import (
	"context"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"regexp"
	"strings"
	"testing"
	"time"
)

// Unverified process watchdog; only actual validation/help exit can pass a test.
const bringupDaemonWatchdog = 10 * time.Second

func runBringupDaemon(t *testing.T, args ...string) (string, error) {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), bringupDaemonWatchdog)
	defer cancel()
	output, err := exec.CommandContext(ctx, filepath.Join(testRoot(t), "build/rosie-rt-core-sim"), args...).CombinedOutput()
	if ctx.Err() != nil {
		t.Fatalf("daemon watchdog expired: %s", output)
	}
	return string(output), err
}

// TestDaemonUsageFlagCoverage compares the parser's accepted flag table against
// actual --help output, so additions and obsolete documentation both fail CI.
func TestDaemonUsageFlagCoverage(t *testing.T) {
	source := string(readTestFile(t, "include/daemon_options.hpp"))
	_, parser, ok := strings.Cut(source, "OptionsParseResult parse_options(")
	if !ok {
		t.Fatal("parser definition not found")
	}
	parsed := map[string]bool{}
	for _, match := range regexp.MustCompile(`\barg\s*==\s*"(-[a-zA-Z0-9-]+)"`).FindAllStringSubmatch(parser, -1) {
		parsed[match[1]] = true
	}
	if len(parsed) == 0 {
		t.Fatal("parsed flag table is empty")
	}
	help, err := runBringupDaemon(t, "--help")
	if err != nil {
		t.Fatalf("help: %v %s", err, help)
	}
	documented := map[string]bool{}
	flags := regexp.MustCompile(`--[a-z][a-z0-9-]*|-h\b`)
	for _, line := range strings.Split(help, "\n") {
		if !strings.HasPrefix(line, "  --") {
			continue
		}
		if !strings.Contains(line, "default:") {
			t.Errorf("flag has no default: %s", line)
		}
		for _, flag := range flags.FindAllString(line, -1) {
			documented[flag] = true
			for _, unit := range []string{"ms", "ns"} {
				if strings.HasSuffix(flag, "-"+unit) && !strings.Contains(line, " "+unit) {
					t.Errorf("flag has no explicit units: %s", line)
				}
			}
		}
	}
	if !reflect.DeepEqual(parsed, documented) {
		for flag := range parsed {
			if !documented[flag] {
				t.Errorf("parsed but undocumented: %s", flag)
			}
		}
		for flag := range documented {
			if !parsed[flag] {
				t.Errorf("documented but not parsed: %s", flag)
			}
		}
	}
}

// TestBusBringupDaemonIdentity validates published argv against native identity
// recomputation and rejects flag tampering for every newly configurable policy.
func TestBusBringupDaemonIdentity(t *testing.T) {
	baseline, err := fresh().compile()
	if err != nil {
		t.Fatal(err)
	}
	_, baselineArgs, err := writeCompiled(baseline, buildTemp(t))
	if err != nil {
		t.Fatal(err)
	}
	all := fresh()
	all.put("machine.bus_bringup", "{}")
	for _, tc := range bringupCases {
		all.put("machine.bus_bringup."+tc.key, tc.value)
		t.Run(tc.key, func(t *testing.T) {
			f := fresh()
			f.put("machine.bus_bringup", `{"`+tc.key+`":`+tc.value+`}`)
			compiled, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			validatePublishedBringup(t, compiled)
			tampered := append(append([]string{}, baselineArgs...), "--validate-config", tc.flag)
			if tc.value != "true" {
				tampered = append(tampered, tc.value)
			}
			output, err := runBringupDaemon(t, tampered...)
			exit, ok := err.(*exec.ExitError)
			if !ok || exit.ExitCode() != 2 || !strings.Contains(output, "machine identity mismatch: "+tc.field) {
				t.Fatalf("want exit 2 with %s mismatch, got %v %s", tc.field, err, output)
			}
		})
	}
	for _, cycle := range []string{"1000000", "1000001"} {
		all.put("machine.cycle_ns", cycle)
		compiled, err := all.compile()
		if err != nil {
			t.Fatal(err)
		}
		validatePublishedBringup(t, compiled)
	}
}

func validatePublishedBringup(t *testing.T, compiled Compiled) {
	t.Helper()
	dir, args, err := writeCompiled(compiled, buildTemp(t))
	if err != nil {
		t.Fatal(err)
	}
	data, err := os.ReadFile(filepath.Join(dir, "argv.json"))
	if err != nil {
		t.Fatal(err)
	}
	var published []string
	if err := json.Unmarshal(data, &published); err != nil || !reflect.DeepEqual(published, args) {
		t.Fatalf("published argv changed: %v %s", err, data)
	}
	output, err := runBringupDaemon(t, append(published, "--validate-config")...)
	if err != nil || !strings.Contains(output, "Machine identity verified: machine_sha256="+compiled.MachineSHA256) || !strings.Contains(output, "Configuration valid: 1 axes") {
		t.Fatalf("native identity verification: %v %s", err, output)
	}
}
