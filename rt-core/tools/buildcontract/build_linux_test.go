//go:build linux

package buildcontract

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

func TestLiveBuildRequiresPinnedIgHPackage(t *testing.T) {
	root, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	for _, version := range []string{"1.6.9", "1.6.8", "1.5.3", "2.0.0", "missing"} {
		t.Run(version, func(t *testing.T) {
			directory := t.TempDir()
			if version != "missing" {
				pc := fmt.Sprintf("Name: libethercat\nDescription: build contract fixture\nVersion: %s\nLibs: -lethercat\nCflags:\n", version)
				if err := os.WriteFile(filepath.Join(directory, "libethercat.pc"), []byte(pc), 0600); err != nil {
					t.Fatal(err)
				}
			}
			t.Setenv("PKG_CONFIG_PATH", directory)
			t.Setenv("PKG_CONFIG_LIBDIR", directory)
			// This target checks metadata only; it cannot invoke a runtime/master.
			cmd := exec.Command("make", "--no-print-directory", "-s", "-f", filepath.Join(root, "Makefile"), "check-ecrt", "PKG_CONFIG=pkg-config")
			cmd.Dir = root
			out, err := cmd.CombinedOutput()
			if version == "1.6.9" {
				if err != nil {
					t.Fatalf("pinned package rejected: %v\n%s", err, out)
				}
			} else if err == nil || !strings.Contains(string(out), "Live builds require IgH libethercat 1.6.9") {
				t.Fatalf("unqualified package was not explicitly rejected: %v\n%s", err, out)
			}
		})
	}
}

func TestIncludedIgHHeaderRequiresCheckedReturnAPI(t *testing.T) {
	root, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	functions := []struct{ name, args string }{
		{"ecrt_master_application_time", "void*, unsigned long long"},
		{"ecrt_master_receive", "void*"}, {"ecrt_domain_process", "void*"},
		{"ecrt_domain_queue", "void*"}, {"ecrt_master_send", "void*"},
		{"ecrt_master_sync_reference_clock", "void*"}, {"ecrt_master_sync_slave_clocks", "void*"},
		{"ecrt_domain_state", "void*, void*"}, {"ecrt_master_state", "void*, void*"},
		{"ecrt_slave_config_state", "void*, void*"},
	}
	for _, tc := range []struct{ name, version, voidFunction, diagnostic string }{
		{"checked-1.6", "1 6", "", ""},
		{"old-version", "1 5", "", "requires the IgH 1.6 API"},
		{"future-api", "2 0", "", "requires the IgH 1.6 API"},
		{"void-receive", "1 6", "ecrt_master_receive", "transport operations must return checked int"},
		{"void-clock", "1 6", "ecrt_master_sync_slave_clocks", "clock operations must return checked int"},
		{"void-state", "1 6", "ecrt_domain_state", "state operations must return checked int"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			directory := t.TempDir()
			version := strings.Fields(tc.version)
			header := fmt.Sprintf("#define ECRT_VER_MAJOR %s\n#define ECRT_VER_MINOR %s\n", version[0], version[1])
			for _, f := range functions {
				result := "int"
				if f.name == tc.voidFunction {
					result = "void"
				}
				header += fmt.Sprintf("%s %s(%s);\n", result, f.name, f.args)
			}
			if err := os.WriteFile(filepath.Join(directory, "ecrt.h"), []byte(header), 0600); err != nil {
				t.Fatal(err)
			}
			probe := filepath.Join(directory, "probe.cpp")
			if err := os.WriteFile(probe, []byte("#define GRADIENT_ECRT_ENABLED 1\n#include \"ecrt_detect.hpp\"\n"), 0600); err != nil {
				t.Fatal(err)
			}
			cmd := exec.Command("g++", "-std=c++17", "-fsyntax-only", "-I"+directory, "-I"+filepath.Join(root, "include"), probe)
			out, err := cmd.CombinedOutput()
			if tc.diagnostic == "" {
				if err != nil {
					t.Fatalf("supported header rejected: %v\n%s", err, out)
				}
			} else if err == nil || !strings.Contains(string(out), tc.diagnostic) {
				t.Fatalf("incompatible header escaped the API guard: %v\n%s", err, out)
			}
		})
	}
}
