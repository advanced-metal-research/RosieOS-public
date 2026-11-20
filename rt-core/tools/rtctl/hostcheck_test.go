package rtctl

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"testing"
)

func hostcheckWrite(t *testing.T, root, path, value string) {
	t.Helper()
	name := filepath.Join(root, path)
	if err := os.MkdirAll(filepath.Dir(name), 0755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(name, []byte(value), 0644); err != nil {
		t.Fatal(err)
	}
}

// A complete filesystem fixture keeps every positive assertion independent of
// the test machine's kernel, NICs, service accounts and systemd installation.
func hostcheckFixture(t *testing.T) (string, string) {
	t.Helper()
	// Go's testing directory can live under GOTMPDIR on drvfs; ownership and
	// mode assertions require the configured Linux runtime filesystem instead.
	root, err := os.MkdirTemp(os.Getenv("TMPDIR"), "hostcheck-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := os.RemoveAll(root); err != nil {
			t.Error(err)
		}
	})
	t.Setenv("ROSIE_RT_HOSTCHECK_ROOT", root)
	t.Setenv("ROSIE_RT_ETHERCAT_GROUP", "ethercat")
	t.Setenv("ROSIE_RT_FOUNDATION_ETHERCAT_INTERFACE", "")
	uid, gid := os.Getuid(), os.Getgid()
	if uid == 0 {
		uid = 1000
	}
	files := map[string]string{
		"machine.json":                                                      `{"runtime":{"cpu":2},"ethercat":{"interface":"eth0","mac":"02:00:00:00:00:01"}}`,
		"proc/sys/kernel/osrelease":                                         "6.12.0-rt1\n",
		"proc/sys/kernel/version":                                           "#1 SMP PREEMPT_RT test\n",
		"sys/kernel/realtime":                                               "1\n",
		"sys/module/ec_master/version":                                      "1.6.9\n",
		"dev/EtherCAT0":                                                     "",
		"etc/ethercat.conf":                                                 "MASTER0_DEVICE=\"02:00:00:00:00:01\"\nDEVICE_MODULES=\"generic\"\n",
		"sys/class/net/eth0/address":                                        "02:00:00:00:00:01\n",
		"sys/class/net/eth0/device/msi_irqs/42":                             "msi\n",
		"sys/devices/system/cpu/isolated":                                   "2-3\n",
		"sys/devices/system/cpu/nohz_full":                                  "2-3\n",
		"sys/devices/system/cpu/cpu2/cpufreq/scaling_governor":              "performance\n",
		"proc/interrupts":                                                   " CPU0 CPU1 CPU2 CPU3\n 42: 10 0 0 0 PCI-MSI eth0-TxRx-0\n 43: 0 0 0 0 PCI-MSI eth01-TxRx-0\n",
		"proc/irq/42/smp_affinity_list":                                     "0-1\n",
		"proc/irq/43/smp_affinity_list":                                     "2\n",
		"hostcheck/timer_slack_ns":                                          "50000\n",
		"hostcheck/clock_monotonic_resolution_ns":                           "1\n",
		"sys/devices/system/clocksource/clocksource0/current_clocksource":   "tsc\n",
		"sys/devices/system/clocksource/clocksource0/available_clocksource": "tsc hpet acpi_pm\n",
		"etc/group":                     fmt.Sprintf("rosie-rt:x:%d:\nrosie-ctl:x:%d:rosie-rt\nrosie-rt-clients:x:%d:rosie-ctl\nethercat:x:%d:rosie-rt\n", gid, gid, gid, gid),
		"etc/passwd":                    fmt.Sprintf("rosie-rt:x:%d:%d::/nonexistent:/usr/sbin/nologin\nrosie-ctl:x:%d:%d::/nonexistent:/usr/sbin/nologin\n", uid, gid, uid, gid),
		"host/rosie-rt-core.service":    "[Service]\nUser=rosie-rt\nGroup=rosie-rt-clients\nLimitMEMLOCK=infinity\n",
		"host/rosie-rt-control.service": "[Service]\nUser=rosie-ctl\nGroup=rosie-rt-clients\n",
	}
	for path, value := range files {
		hostcheckWrite(t, root, path, value)
	}
	if err := os.Chmod(filepath.Join(root, "dev/EtherCAT0"), 0660); err != nil {
		t.Fatal(err)
	}
	for _, path := range []string{"run/rosie-rt-core", "run/rosie-rt-core/native", "run/rosie-rt-core/public"} {
		name := filepath.Join(root, path)
		if err := os.MkdirAll(name, 0755); err != nil {
			t.Fatal(err)
		}
		if os.Getuid() == 0 {
			if err := os.Chown(name, uid, gid); err != nil {
				t.Fatal(err)
			}
		}
		mode := os.FileMode(0750) | os.ModeSetgid
		if path == "run/rosie-rt-core" {
			mode = 0770 | os.ModeSetgid
		}
		if err := os.Chmod(name, mode); err != nil {
			t.Fatal(err)
		}
	}
	return root, filepath.Join(root, "machine.json")
}

func hostcheckRun(t *testing.T, config string, extra ...string) (hostcheckReport, string, int) {
	t.Helper()
	var out, stderr bytes.Buffer
	args := []string{"--json"}
	if config != "" {
		args = append(args, "--config", config)
	}
	args = append(args, extra...)
	code := RunHostcheckCLI(args, "../..", &out, &stderr)
	var report hostcheckReport
	if err := json.Unmarshal(out.Bytes(), &report); err != nil {
		t.Fatalf("exit=%d stderr=%s output=%s: %v", code, stderr.String(), out.String(), err)
	}
	if code != report.ExitCode {
		t.Fatalf("exit=%d report=%d", code, report.ExitCode)
	}
	seen := map[string]bool{}
	for _, check := range report.Checks {
		if seen[check.ID] || check.Expected == "" {
			t.Fatalf("duplicate or incomplete check: %+v", check)
		}
		seen[check.ID] = true
		if check.Status != "pass" && check.Status != "fail" && check.Status != "not_applicable" {
			t.Fatalf("invalid status: %+v", check)
		}
	}
	return report, out.String(), code
}

func hostcheckFind(t *testing.T, report hostcheckReport, id, status string) hostCheck {
	t.Helper()
	for _, check := range report.Checks {
		if check.ID == id {
			if check.Status != status {
				t.Fatalf("%s: wanted %s, got %+v", id, status, check)
			}
			return check
		}
	}
	t.Fatalf("missing check %s: %+v", id, report)
	return hostCheck{}
}

func TestHostcheckPassGolden(t *testing.T) {
	_, config := hostcheckFixture(t)
	report, output, code := hostcheckRun(t, config)
	if code != 0 {
		t.Fatalf("exit=%d: %s", code, output)
	}
	if len(report.Checks) != 26 {
		t.Fatalf("expected 26 prerequisite checks, got %d", len(report.Checks))
	}
	golden := filepath.Join("testdata", "hostcheck", "pass.json")
	if os.Getenv("UPDATE_HOSTCHECK_GOLDEN") == "1" {
		hostcheckWrite(t, ".", golden, output)
	}
	want, err := os.ReadFile(golden)
	if err != nil {
		t.Fatal(err)
	}
	if output != string(want) {
		t.Fatalf("hostcheck JSON differs from golden:\n%s", output)
	}
	var out, stderr bytes.Buffer
	if code := RunHostcheckCLI([]string{"--config", config}, "../..", &out, &stderr); code != 0 || !strings.Contains(out.String(), "pass preempt_rt:") {
		t.Fatalf("text output: code=%d %s %s", code, &out, &stderr)
	}
}

func TestHostcheckFailures(t *testing.T) {
	cases := []struct{ name, path, value, id string }{
		{"kernel_release", "proc/sys/kernel/osrelease", "", "kernel_version"},
		{"kernel_build", "proc/sys/kernel/version", "", "kernel_build"},
		{"cpu", "machine.json", `{"runtime":{"cpu":-1}}`, "rt_cpu"},
		{"module_version", "sys/module/ec_master/version", "1.5.3", "igh_master"},
		{"mac", "sys/class/net/eth0/address", "02:00:00:00:00:02", "master_nic"},
		{"invalid_mac", "etc/ethercat.conf", "MASTER0_DEVICE=$(touch /must-not-exist)", "master_nic"},
		{"duplicate_mac", "etc/ethercat.conf", "MASTER0_DEVICE=02:00:00:00:00:01\nMASTER0_DEVICE=02:00:00:00:00:01", "master_nic"},
		{"conflicting_sysconfig", "etc/sysconfig/ethercat", `MASTER0_DEVICE="02:00:00:00:00:02"`, "master_nic"},
		{"duplicate_sysconfig", "etc/sysconfig/ethercat", "MASTER0_DEVICE=02:00:00:00:00:01\nMASTER0_DEVICE=02:00:00:00:00:01", "master_nic"},
		{"unset_conf", "etc/ethercat.conf", `MASTER0_DEVICE=""`, "master_nic"},
		{"isolated", "sys/devices/system/cpu/isolated", "0-1", "isolated"},
		{"nohz", "sys/devices/system/cpu/nohz_full", "", "nohz_full"},
		{"governor", "sys/devices/system/cpu/cpu2/cpufreq/scaling_governor", "powersave", "cpufreq_governor"},
		{"irq_rt_cpu", "proc/irq/42/smp_affinity_list", "0-2", "nic_irq_42"},
		{"irq_invalid", "proc/irq/42/smp_affinity_list", "oops", "nic_irq_42"},
		{"irq_empty", "proc/irq/42/smp_affinity_list", "", "nic_irq_42"},
		{"irq_absent_interrupts", "proc/interrupts", "", "nic_irq_42"},
		{"slack", "hostcheck/timer_slack_ns", "50001", "timer_slack"},
		{"slack_invalid", "hostcheck/timer_slack_ns", "-1", "timer_slack"},
		{"resolution", "hostcheck/clock_monotonic_resolution_ns", "1001", "timer_resolution"},
		{"resolution_zero", "hostcheck/clock_monotonic_resolution_ns", "0", "timer_resolution"},
		{"clocksource", "sys/devices/system/clocksource/clocksource0/current_clocksource", "unknown", "clocksource"},
		{"memlock", "host/rosie-rt-core.service", "LimitMEMLOCK=65536", "memlock_rosie-rt-core"},
		{"groups", "etc/group", "", "group_rosie-rt"},
		{"user_missing", "etc/passwd", "", "user_rosie-rt"},
		{"user_root", "etc/passwd", "rosie-rt:x:0:1000::/nonexistent:/usr/sbin/nologin\n", "user_rosie-rt"},
		{"user_shell", "etc/passwd", "rosie-rt:x:1000:1000::/nonexistent:/bin/bash\n", "user_rosie-rt"},
	}
	for _, test := range cases {
		t.Run(test.name, func(t *testing.T) {
			root, config := hostcheckFixture(t)
			hostcheckWrite(t, root, test.path, test.value)
			report, output, code := hostcheckRun(t, config)
			if code != 2 {
				t.Fatalf("expected failed qualification exit 2, got %d: %s", code, output)
			}
			hostcheckFind(t, report, test.id, "fail")
		})
	}
}

func TestHostcheckRequiredReadErrors(t *testing.T) {
	for _, path := range []string{"machine.json", "proc/sys/kernel/osrelease", "proc/sys/kernel/version", "sys/module/ec_master/version", "etc/group", "etc/passwd", "sys/class/net/eth0/address", "sys/devices/system/cpu/isolated", "sys/devices/system/cpu/cpu2/cpufreq/scaling_governor", "proc/interrupts", "proc/irq/42/smp_affinity_list", "hostcheck/timer_slack_ns", "hostcheck/clock_monotonic_resolution_ns", "sys/devices/system/clocksource/clocksource0/available_clocksource", "host/rosie-rt-core.service"} {
		t.Run(path, func(t *testing.T) {
			root, config := hostcheckFixture(t)
			if err := os.Remove(filepath.Join(root, path)); err != nil {
				t.Fatal(err)
			}
			report, output, code := hostcheckRun(t, config)
			if code != 3 {
				t.Fatalf("required file missing: expected exit 3, got %d: %s", code, output)
			}
			readErrors := 0
			for _, check := range report.Checks {
				if check.Error != "" {
					readErrors++
				}
			}
			if readErrors != 1 {
				t.Fatalf("expected one read error, got %d: %s", readErrors, output)
			}
		})
	}
	t.Run("unreadable_optional", func(t *testing.T) {
		root, config := hostcheckFixture(t)
		path := filepath.Join(root, "sys/kernel/realtime")
		if err := os.Remove(path); err != nil {
			t.Fatal(err)
		}
		if err := os.Mkdir(path, 0755); err != nil {
			t.Fatal(err)
		}
		_, output, code := hostcheckRun(t, config)
		if code != 3 {
			t.Fatalf("unreadable present file exit=%d: %s", code, output)
		}
	})
}

func TestHostcheckMissingFacilitiesAndPermissions(t *testing.T) {
	for _, item := range []struct{ path, id string }{{"sys/module/ec_master", "igh_master"}, {"dev/EtherCAT0", "ethercat_device"}, {"run/rosie-rt-core/native", "native_directory"}, {"run/rosie-rt-core/public", "public_directory"}} {
		t.Run(item.id+"_missing", func(t *testing.T) {
			root, config := hostcheckFixture(t)
			if err := os.RemoveAll(filepath.Join(root, item.path)); err != nil {
				t.Fatal(err)
			}
			report, output, code := hostcheckRun(t, config)
			if code != 2 {
				t.Fatalf("exit=%d: %s", code, output)
			}
			hostcheckFind(t, report, item.id, "fail")
		})
	}
	for _, item := range []struct{ path, id string }{{"dev/EtherCAT0", "ethercat_device"}, {"run/rosie-rt-core", "runtime_directory"}, {"run/rosie-rt-core/native", "native_directory"}, {"run/rosie-rt-core/public", "public_directory"}} {
		t.Run(item.id+"_mode", func(t *testing.T) {
			root, config := hostcheckFixture(t)
			if err := os.Chmod(filepath.Join(root, item.path), 0777); err != nil {
				t.Fatal(err)
			}
			report, _, code := hostcheckRun(t, config)
			if code != 2 {
				t.Fatal(code)
			}
			hostcheckFind(t, report, item.id, "fail")
		})
	}
	t.Run("owner", func(t *testing.T) {
		root, config := hostcheckFixture(t)
		hostcheckWrite(t, root, "etc/passwd", fmt.Sprintf("rosie-rt:x:%d:%d::/nonexistent:/usr/sbin/nologin\n", os.Getuid()+1, os.Getgid()))
		report, _, code := hostcheckRun(t, config)
		if code != 2 {
			t.Fatal(code)
		}
		hostcheckFind(t, report, "runtime_directory", "fail")
	})
	t.Run("device_group", func(t *testing.T) {
		root, config := hostcheckFixture(t)
		hostcheckWrite(t, root, "etc/group", fmt.Sprintf("ethercat:x:%d:\n", os.Getgid()+1))
		report, _, code := hostcheckRun(t, config)
		if code != 2 {
			t.Fatal(code)
		}
		hostcheckFind(t, report, "ethercat_device", "fail")
	})
}

func TestHostcheckNonRtHost(t *testing.T) {
	root, config := hostcheckFixture(t)
	hostcheckWrite(t, root, "proc/sys/kernel/osrelease", "6.6.87.2-microsoft-standard-WSL2")
	hostcheckWrite(t, root, "proc/sys/kernel/version", "#1 SMP PREEMPT_DYNAMIC WSL2")
	for _, path := range []string{"sys/kernel/realtime", "sys/module/ec_master", "dev/EtherCAT0", "sys/devices/system/cpu/isolated", "sys/devices/system/cpu/nohz_full", "sys/devices/system/cpu/cpu2/cpufreq", "sys/devices/system/clocksource"} {
		if err := os.RemoveAll(filepath.Join(root, path)); err != nil {
			t.Fatal(err)
		}
	}
	report, output, code := hostcheckRun(t, config)
	if code != 2 {
		t.Fatalf("non-RT exit=%d: %s", code, output)
	}
	hostcheckFind(t, report, "preempt_rt", "fail")
	hostcheckFind(t, report, "igh_master", "fail")
	for _, id := range []string{"isolated", "nohz_full", "cpufreq_governor", "timer_slack", "timer_resolution", "clocksource", "timing_qualification"} {
		hostcheckFind(t, report, id, "not_applicable")
	}
}

func TestHostcheckNonRtTimingIsNotQualified(t *testing.T) {
	root, config := hostcheckFixture(t)
	hostcheckWrite(t, root, "proc/sys/kernel/version", "#1 SMP PREEMPT_DYNAMIC WSL2")
	hostcheckWrite(t, root, "sys/kernel/realtime", "0")
	report, _, code := hostcheckRun(t, config)
	if code != 2 {
		t.Fatalf("non-RT kernel exit=%d", code)
	}
	for _, id := range []string{"timer_slack", "timer_resolution", "clocksource", "timing_qualification"} {
		hostcheckFind(t, report, id, "not_applicable")
	}
}

// Exercise the calling-thread read on Linux as well as fixture I/O. This catches
// procfs paths that only work when a Go goroutine happens to run on its leader.
func TestHostcheckCallingThreadTimerRead(t *testing.T) {
	h := &hostInspector{root: "/"}
	h.timers(false)
	check := hostcheckFind(t, h.report, "timer_slack", "not_applicable")
	fields := strings.Fields(check.Observed)
	if len(fields) < 2 || fields[1] != "ns" {
		t.Fatalf("timer slack lacks ns: %+v", check)
	}
	if _, err := strconv.ParseUint(fields[0], 10, 64); err != nil {
		t.Fatalf("invalid timer slack: %+v", check)
	}
}

func TestHostcheckDiscoveryAndParsing(t *testing.T) {
	t.Run("deployment_and_sysconfig", func(t *testing.T) {
		root, _ := hostcheckFixture(t)
		hostcheckWrite(t, root, "etc/rosie-rt-core/control.env", `ROSIE_RT_CORE_ARGS="--num-axes 1 --rt-cpu 2 --rt-priority 90"`)
		if err := os.Rename(filepath.Join(root, "etc/ethercat.conf"), filepath.Join(root, "ethercat.saved")); err != nil {
			t.Fatal(err)
		}
		hostcheckWrite(t, root, "etc/sysconfig/ethercat", `MASTER0_DEVICE="02:00:00:00:00:01"`)
		_, output, code := hostcheckRun(t, "")
		if code != 0 {
			t.Fatalf("exit=%d: %s", code, output)
		}
	})
	// other-bench-servo (the second 1430 cell), read 2026-10-11: the master runs from
	// /etc/ethercat.conf and IgH's legacy template stays beside it unset.
	t.Run("unset_sysconfig_template", func(t *testing.T) {
		root, config := hostcheckFixture(t)
		hostcheckWrite(t, root, "etc/sysconfig/ethercat", "MASTER0_DEVICE=\"\"\nDEVICE_MODULES=\"\"\nUPDOWN_INTERFACES=\"\"\n")
		report, output, code := hostcheckRun(t, config)
		if code != 0 {
			t.Fatalf("exit=%d: %s", code, output)
		}
		hostcheckFind(t, report, "master_nic", "pass")
	})
	t.Run("duplicate_then_named", func(t *testing.T) {
		root, config := hostcheckFixture(t)
		hostcheckWrite(t, root, "etc/ethercat.conf", "MASTER0_DEVICE=02:00:00:00:00:01\nMASTER0_DEVICE=02:00:00:00:00:02\n")
		hostcheckWrite(t, root, "etc/sysconfig/ethercat", `MASTER0_DEVICE="02:00:00:00:00:01"`)
		report, _, code := hostcheckRun(t, config)
		if code != 2 {
			t.Fatal(code)
		}
		hostcheckFind(t, report, "master_nic", "fail")
	})
	t.Run("uname_fallback", func(t *testing.T) {
		root, config := hostcheckFixture(t)
		if err := os.Remove(filepath.Join(root, "sys/kernel/realtime")); err != nil {
			t.Fatal(err)
		}
		report, _, code := hostcheckRun(t, config)
		if code != 0 {
			t.Fatal(code)
		}
		hostcheckFind(t, report, "preempt_rt", "pass")
	})
	t.Run("no_cpu", func(t *testing.T) {
		hostcheckFixture(t)
		report, _, code := hostcheckRun(t, "")
		if code != 2 {
			t.Fatal(code)
		}
		hostcheckFind(t, report, "rt_cpu", "fail")
	})
	for _, list := range []string{"-1", "1-0", "0,", ",0", "0,,1", "2:4", "0-2:1/2", "x", "1048576", "0-999999999999"} {
		if _, err := hostCPUList(list); err == nil {
			t.Fatalf("accepted invalid CPU list %q", list)
		}
	}
	cpus, err := hostCPUList("0-1,4,4")
	if err != nil || !reflect.DeepEqual(cpus, map[int]bool{0: true, 1: true, 4: true}) {
		t.Fatalf("CPU list: %v %v", cpus, err)
	}
}

func TestHostcheckReadOnly(t *testing.T) {
	root, config := hostcheckFixture(t)
	snapshot := func() map[string]string {
		result := map[string]string{}
		err := filepath.WalkDir(root, func(path string, entry os.DirEntry, err error) error {
			if err != nil {
				return err
			}
			info, err := entry.Info()
			if err != nil {
				return err
			}
			value := info.Mode().String() + " " + strconv.FormatInt(info.ModTime().UnixNano(), 10)
			if !entry.IsDir() {
				raw, err := os.ReadFile(path)
				if err != nil {
					return err
				}
				value += fmt.Sprintf(" %x", sha256.Sum256(raw))
			}
			result[path] = value
			return nil
		})
		if err != nil {
			t.Fatal(err)
		}
		return result
	}
	before := snapshot()
	_, _, code := hostcheckRun(t, config)
	if code != 0 {
		t.Fatal(code)
	}
	if !reflect.DeepEqual(before, snapshot()) {
		t.Fatal("hostcheck changed fixture contents, modes or modification times")
	}
}

func TestHostcheckCLIRejectsInvalidArguments(t *testing.T) {
	for _, args := range [][]string{{"--unknown"}, {"unexpected"}, {"--config"}} {
		var out, stderr bytes.Buffer
		if code := RunHostcheckCLI(args, "../..", &out, &stderr); code != 2 {
			t.Fatalf("args=%v exit=%d", args, code)
		}
	}
}
