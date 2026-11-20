package rtctl

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strconv"
	"strings"
	"testing"
)

func identityFixtureSHA(raw string) string { return fmt.Sprintf("%x", sha256.Sum256([]byte(raw))) }

func identityFixtureJSON(t *testing.T, root, path string, value any) {
	t.Helper()
	raw, err := json.MarshalIndent(value, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	hostcheckWrite(t, root, path, string(raw)+"\n")
}

func identityFixtureLink(t *testing.T, root, path, target string) {
	t.Helper()
	full := filepath.Join(root, path)
	if err := os.MkdirAll(filepath.Dir(full), 0755); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(target, full); err != nil {
		t.Fatal(err)
	}
}

func identityFixtureStat(pid int, ticks string) string {
	return fmt.Sprintf("%d (fixture (with spaces)) S 1 1 1 0 -1 0 0 0 0 0 0 0 0 0 20 0 1 0 %s 0 0\n", pid, ticks)
}

func identityFixtureEnv(env map[string]string, nul bool) string {
	keys := []string{}
	for key := range env {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	var raw strings.Builder
	for _, key := range keys {
		if nul {
			fmt.Fprintf(&raw, "%s=%s\x00", key, env[key])
		} else {
			fmt.Fprintf(&raw, "%s=%q\n", key, env[key])
		}
	}
	return raw.String()
}

// identityFixture uses all three shipping unit templates, an installed drop-in,
// typed systemd replies and a fake proc tree. Loaded images are separate files so
// replacing an installed pathname cannot silently replace the observed process.
func identityFixture(t *testing.T) (string, string, string, hostRelease) {
	t.Helper()
	root, config := hostcheckFixture(t)
	release := hostRelease{Schema: "rosie-rt-core.host-identity.v1", Services: map[string]hostReleaseService{}}
	coreEnv := map[string]string{
		"ROSIE_RT_CORE_ARGS":            "--rt-cpu 2 --configuration-hash configuration-a",
		"ROSIE_RT_CONFIGURATION_SHA256": "configuration-a", "ROSIE_RT_PAIR_ID": "bench",
		"ROSIE_RT_PAIR_REVISION": "7", "ROSIE_RT_REMOTE_ARGS": "",
		"ROSIE_RT_ANCHOR_DIR":                          "/var/lib/rosie-rt-core/anchors",
		"ROSIE_RT_COMPILED_CONFIG":                     "/etc/rosie-rt-core/compiled/configuration-a",
		"GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED": "1", "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED": "1", "GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED": "1",
	}
	pubEnv := map[string]string{"ROSIE_RT_NATS_URL": "nats://bench:4222", "ROSIE_RT_NATS_HOST": "bench", "ROSIE_RT_NATS_CADENCE": "100ms", "ROSIE_RT_NATS_TELEMETRY_CADENCE": "10ms", "ROSIE_RT_NATS_TELEMETRY_MAX_BYTES": "100000"}
	hostcheckWrite(t, root, "etc/rosie-rt-core/control.env", identityFixtureEnv(coreEnv, false))
	hostcheckWrite(t, root, "etc/rosie-rt-core/natspublisher.env", identityFixtureEnv(pubEnv, false))
	hostcheckWrite(t, root, "proc/sys/kernel/random/boot_id", "01234567-89ab-cdef-0123-456789abcdef\n")
	hostcheckWrite(t, root, "proc/mounts", "proc /proc proc rw,nosuid,nodev,noexec,relatime 0 0\n")
	commands := [][]string{
		{"/opt/rosie/rt-core/build/rosie-rt-core", "$ROSIE_RT_CORE_ARGS", "--socket-path", "/run/rosie-rt-core/native/ipc.sock", "--ipc-group", "rosie-ctl", "--adapter-user", "rosie-ctl"},
		{"/opt/rosie/rt-core/build/rt-control", "--core-socket", "/run/rosie-rt-core/native/ipc.sock", "--socket", "/run/rosie-rt-core/public/control.sock", "--configuration-sha256", "${ROSIE_RT_CONFIGURATION_SHA256}", "--pair-id", "${ROSIE_RT_PAIR_ID}", "--pair-revision", "${ROSIE_RT_PAIR_REVISION}", "$ROSIE_RT_REMOTE_ARGS"},
		{"/opt/rosie/rt-core/build/rt-natspublisher", "--socket", "/run/rosie-rt-core/public/control.sock"},
	}
	expanded := [][]string{
		{"/opt/rosie/rt-core/build/rosie-rt-core", "--rt-cpu", "2", "--configuration-hash", "configuration-a", "--socket-path", "/run/rosie-rt-core/native/ipc.sock", "--ipc-group", "rosie-ctl", "--adapter-user", "rosie-ctl"},
		{"/opt/rosie/rt-core/build/rt-control", "--core-socket", "/run/rosie-rt-core/native/ipc.sock", "--socket", "/run/rosie-rt-core/public/control.sock", "--configuration-sha256", "configuration-a", "--pair-id", "bench", "--pair-revision", "7"},
		{"/opt/rosie/rt-core/build/rt-natspublisher", "--socket", "/run/rosie-rt-core/public/control.sock"},
	}
	for i, name := range hostIdentityServices {
		unit, err := os.ReadFile(filepath.Join("../..", "host", name+".service"))
		if err != nil {
			t.Fatal(err)
		}
		unitPath := "/etc/systemd/system/" + name + ".service"
		dropPath := "/etc/systemd/system/" + name + ".service.d/10-deployment.conf"
		drop := "[Service]\nEnvironment=HOSTCHECK_FIXTURE=1\n"
		hostcheckWrite(t, root, "host/"+name+".service", string(unit))
		hostcheckWrite(t, root, unitPath, string(unit))
		hostcheckWrite(t, root, dropPath, drop)
		env, envPath := coreEnv, "/etc/rosie-rt-core/control.env"
		if i == 2 {
			env, envPath = pubEnv, "/etc/rosie-rt-core/natspublisher.env"
		}
		image := "release image for " + name + "\n"
		s := hostReleaseService{
			TemplateSHA256: identityFixtureSHA(string(unit)), UnitFiles: []hostReleaseFile{{unitPath, identityFixtureSHA(string(unit))}, {dropPath, identityFixtureSHA(drop)}},
			Argv: expanded[i], ExecutableSHA256: identityFixtureSHA(image), EnvironmentFiles: []string{envPath}, Environment: env,
		}
		release.Services[name] = s
		hostcheckWrite(t, root, s.Argv[0], image)
		hostcheckWrite(t, root, "hostcheck/images/"+name, image)
		pid := 101 + i
		base := fmt.Sprintf("proc/%d/", pid)
		hostcheckWrite(t, root, base+"stat", identityFixtureStat(pid, strconv.Itoa(10001+i)))
		hostcheckWrite(t, root, base+"cmdline", strings.Join(s.Argv, "\x00")+"\x00")
		hostcheckWrite(t, root, base+"environ", identityFixtureEnv(env, true)+"HOSTCHECK_FIXTURE=1\x00IGNORED_SECRET=do-not-print-this-secret\x00")
		hostcheckWrite(t, root, base+"cgroup", "0::/system.slice/"+name+".service\n")
		identityFixtureLink(t, root, base+"exe", "../../hostcheck/images/"+name)
		if err := os.MkdirAll(filepath.Join(root, base, "fd"), 0755); err != nil {
			t.Fatal(err)
		}
		identityFixtureLink(t, root, fmt.Sprintf("%stask/%d/fd", base, pid), "../../fd")
		unitProps := map[string]any{
			"FragmentPath": map[string]any{"type": "s", "data": unitPath}, "DropInPaths": map[string]any{"type": "as", "data": []string{dropPath}},
			"LoadState": map[string]any{"type": "s", "data": "loaded"}, "ActiveState": map[string]any{"type": "s", "data": "active"}, "NeedDaemonReload": map[string]any{"type": "b", "data": false},
		}
		serviceProps := map[string]any{
			"MainPID": map[string]any{"type": "u", "data": pid}, "ExecMainStartTimestampMonotonic": map[string]any{"type": "t", "data": 100010000 + i},
			"ControlGroup": map[string]any{"type": "s", "data": "/system.slice/" + name + ".service"},
			"ExecStartEx":  map[string]any{"type": "a(sasasttttuii)", "data": [][]any{{commands[i][0], commands[i], []string{}, 0, 100010000 + i, 0, 0, pid, 0, 0}}},
			"Environment":  map[string]any{"type": "as", "data": []string{"HOSTCHECK_FIXTURE=1"}}, "EnvironmentFiles": map[string]any{"type": "a(sb)", "data": [][]any{{envPath, false}}},
			"PassEnvironment": map[string]any{"type": "as", "data": []string{}}, "UnsetEnvironment": map[string]any{"type": "as", "data": []string{}},
			"RootDirectory": map[string]any{"type": "s", "data": ""}, "RootImage": map[string]any{"type": "s", "data": ""},
		}
		for iface, props := range map[string]map[string]any{"Unit": unitProps, "Service": serviceProps} {
			identityFixtureJSON(t, root, "hostcheck/systemd/"+name+"."+iface+".json", map[string]any{"type": "a{sv}", "data": []any{props}})
		}
	}
	identityFixtureLink(t, root, "proc/101/fd/7", "../../../dev/EtherCAT0")
	manifest := filepath.Join(root, "release.json")
	identityFixtureJSON(t, root, "release.json", release)
	return root, config, manifest, release
}

func identityFixtureProperty(t *testing.T, root, service, iface, key string, data any) {
	t.Helper()
	path := "hostcheck/systemd/" + service + "." + iface + ".json"
	raw, err := os.ReadFile(filepath.Join(root, path))
	if err != nil {
		t.Fatal(err)
	}
	var reply struct {
		Type string                    `json:"type"`
		Data []map[string]hostBusValue `json:"data"`
	}
	if err := json.Unmarshal(raw, &reply); err != nil {
		t.Fatal(err)
	}
	v := reply.Data[0][key]
	v.Data, err = json.Marshal(data)
	if err != nil {
		t.Fatal(err)
	}
	reply.Data[0][key] = v
	identityFixtureJSON(t, root, path, reply)
}

func identityRun(t *testing.T, config, manifest string) (hostIdentityReport, string, int) {
	t.Helper()
	r, output, code := hostcheckRun(t, config, "--expected-release", manifest)
	if r.Identity == nil {
		t.Fatal("missing release identity")
	}
	if strings.Contains(output, "do-not-print-this-secret") {
		t.Fatal("secret disclosed")
	}
	return *r.Identity, output, code
}

func identityAssertUnqualified(t *testing.T, r hostIdentityReport, code, wantCode int) {
	t.Helper()
	if (wantCode != 2 && wantCode != 3) || code != wantCode || r.Conflicts+r.Unknowns < 1 || r.QualifiedMatches != 0 || r.Status == "match" || len(r.Services) != 3 {
		t.Fatalf("identity incorrectly qualified: code=%d want=%d %+v", code, wantCode, r)
	}
	for _, s := range r.Services {
		if s.QualifiedMatch {
			t.Fatalf("qualified service despite failed deployment: %+v", s)
		}
	}
}

func identityMutate(t *testing.T, root, name, which string, i int) {
	t.Helper()
	pid := 101 + i
	base := fmt.Sprintf("proc/%d/", pid)
	switch which {
	case "dropin_command":
		hostcheckWrite(t, root, "/etc/systemd/system/"+name+".service.d/10-deployment.conf", "[Service]\nExecStart=\nExecStart=/opt/old/runtime --old\n")
		identityFixtureProperty(t, root, name, "Service", "ExecStartEx", [][]any{{"/opt/old/runtime", []string{"/opt/old/runtime", "--old"}, []string{}, 0, 100010000 + i, 0, 0, pid, 0, 0}})
	case "arguments":
		executable := "/opt/rosie/rt-core/build/" + map[string]string{"rosie-rt-core": "rosie-rt-core", "rosie-rt-control": "rt-control", "rosie-rt-natspublisher": "rt-natspublisher"}[name]
		identityFixtureProperty(t, root, name, "Service", "ExecStartEx", [][]any{{executable, []string{executable, "--configuration", "other"}, []string{}, 0, 100010000 + i, 0, 0, pid, 0, 0}})
	case "replaced_executable":
		image := filepath.Join(root, "hostcheck/images/"+name)
		old := image + " (deleted)"
		if err := os.Rename(image, old); err != nil {
			t.Fatal(err)
		}
		hostcheckWrite(t, root, "hostcheck/images/"+name+" (deleted)", "old v4 image still loaded\n")
		if err := os.Remove(filepath.Join(root, base+"exe")); err != nil {
			t.Fatal(err)
		}
		identityFixtureLink(t, root, base+"exe", "../../hostcheck/images/"+name+" (deleted)")
	case "unreadable_image":
		if err := os.Remove(filepath.Join(root, "hostcheck/images/"+name)); err != nil {
			t.Fatal(err)
		}
		if err := os.Mkdir(filepath.Join(root, "hostcheck/images/"+name), 0755); err != nil {
			t.Fatal(err)
		}
	case "absent_mainpid":
		identityFixtureProperty(t, root, name, "Service", "MainPID", 0)
	case "environment":
		// Unit EnvironmentFile values take precedence; changing the deployment
		// file also differs from the old process's still-loaded environment.
		path, key := "etc/rosie-rt-core/control.env", "ROSIE_RT_PAIR_ID"
		if i == 2 {
			path, key = "etc/rosie-rt-core/natspublisher.env", "ROSIE_RT_NATS_HOST"
		}
		raw, err := os.ReadFile(filepath.Join(root, path))
		if err != nil {
			t.Fatal(err)
		}
		hostcheckWrite(t, root, path, strings.ReplaceAll(string(raw), key+"=\"bench\"", key+"=\"other\""))
	}
}

func TestHostIdentityGolden(t *testing.T) {
	for which, wantCode := range map[string]int{"matching": 0, "dropin_command": 2, "arguments": 2, "replaced_executable": 2, "unreadable_image": 3, "absent_mainpid": 3, "environment": 2, "foreign_master": 2} {
		t.Run(which, func(t *testing.T) {
			root, config, manifest, _ := identityFixture(t)
			for i, name := range hostIdentityServices {
				identityMutate(t, root, name, which, i)
			}
			if which == "foreign_master" {
				hostcheckWrite(t, root, "proc/404/stat", identityFixtureStat(404, "777"))
				identityFixtureLink(t, root, "proc/404/fd/9", "../../../dev/EtherCAT0")
				identityFixtureLink(t, root, "proc/404/task/404/fd", "../../fd")
			}
			r, output, code := identityRun(t, config, manifest)
			if which == "matching" {
				if code != 0 || r.Conflicts != 0 || r.Unknowns != 0 || r.QualifiedMatches != 3 || r.Status != "match" {
					t.Fatalf("matching: code=%d %s", code, output)
				}
				for _, s := range r.Services {
					if !s.QualifiedMatch || len(s.Layers) != 14 {
						t.Fatalf("missing identity layers: %+v", s)
					}
				}
			} else {
				identityAssertUnqualified(t, r, code, wantCode)
			}
			if which == "foreign_master" && (r.MasterOwner.Status != "conflict" || !strings.Contains(r.MasterOwner.Observed, "404")) {
				t.Fatalf("foreign owner: %+v", r.MasterOwner)
			}
			if which == "replaced_executable" {
				for _, s := range r.Services {
					if s.Layers["installed_executable"].Status != "match" || s.Layers["loaded_image"].Status != "conflict" {
						t.Fatalf("conflated installed and loaded bytes: %+v", s)
					}
				}
			}
			golden := filepath.Join("testdata", "hostcheck", "identity", which+".json")
			raw, err := json.MarshalIndent(struct {
				hostIdentityReport
				ExitCode int `json:"exit_code"`
			}{r, code}, "", "  ")
			if err != nil {
				t.Fatal(err)
			}
			raw = append(raw, '\n')
			if os.Getenv("UPDATE_HOSTCHECK_GOLDEN") == "1" {
				hostcheckWrite(t, ".", golden, string(raw))
				if which == "matching" {
					release, err := os.ReadFile(manifest)
					if err != nil {
						t.Fatal(err)
					}
					hostcheckWrite(t, ".", "testdata/hostcheck/identity/release.json", string(release))
				}
			}
			want, err := os.ReadFile(golden)
			if err != nil {
				t.Fatal(err)
			}
			if !bytes.Equal(want, raw) {
				t.Fatalf("identity golden differs: %s\n%s", golden, raw)
			}
		})
	}
}

func TestHostIdentityEachServiceFailsClosed(t *testing.T) {
	for i, name := range hostIdentityServices {
		for which, wantCode := range map[string]int{"dropin_command": 2, "arguments": 2, "replaced_executable": 2, "unreadable_image": 3, "absent_mainpid": 3, "environment": 2} {
			t.Run(name+"/"+which, func(t *testing.T) {
				root, config, manifest, _ := identityFixture(t)
				identityMutate(t, root, name, which, i)
				r, _, code := identityRun(t, config, manifest)
				identityAssertUnqualified(t, r, code, wantCode)
				if identityAllMatch(r.Services[i].Layers) {
					t.Fatalf("failed service lacks explicit conflict or unknown: %+v", r.Services[i])
				}
			})
		}
	}
}

func TestHostIdentityReadOnlyAndText(t *testing.T) {
	for _, tc := range []struct {
		status  string
		code    int
		matches int
	}{{"match", 0, 3}, {"conflict", 2, 0}, {"unknown", 3, 0}} {
		t.Run(tc.status, func(t *testing.T) {
			root, config, manifest, _ := identityFixture(t)
			if tc.status == "conflict" {
				identityMutate(t, root, "rosie-rt-core", "arguments", 0)
			} else if tc.status == "unknown" {
				if err := os.Remove(filepath.Join(root, "proc/101/fd/7")); err != nil {
					t.Fatal(err)
				}
			}
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
					value := fmt.Sprintf("%s %d", info.Mode(), info.ModTime().UnixNano())
					if entry.Type()&os.ModeSymlink != 0 {
						target, err := os.Readlink(path)
						if err != nil {
							return err
						}
						value += target
					} else if !entry.IsDir() {
						raw, err := os.ReadFile(path)
						if err != nil {
							return err
						}
						value += identityFixtureSHA(string(raw))
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
			var out, stderr bytes.Buffer
			if code := RunHostcheckCLI([]string{"--config", config, "--expected-release", manifest}, "../..", &out, &stderr); code != tc.code || !strings.Contains(out.String(), fmt.Sprintf("qualified_matches=%d", tc.matches)) || !strings.Contains(out.String(), "\n"+tc.status+" release_identity:") {
				t.Fatalf("text exit=%d: %s %s", code, &out, &stderr)
			}
			if strings.Contains(out.String(), "do-not-print-this-secret") {
				t.Fatal("text exposed secret")
			}
			if !reflect.DeepEqual(before, snapshot()) {
				t.Fatal("identity check mutated host files")
			}
		})
	}
}
