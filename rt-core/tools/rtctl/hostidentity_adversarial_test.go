package rtctl

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"syscall"
	"testing"
)

func TestHostIdentityUnknownEvidence(t *testing.T) {
	for which, wantCode := range map[string]int{"missing_manager": 3, "malformed_manager": 3, "missing_manifest": 3, "duplicate_manifest": 3, "partial_manifest": 3, "unknown_manifest_field": 3, "invalid_digest": 3, "unknown_service": 3, "pending_reload": 2, "changed_unit_path": 3, "unlisted_dropin": 2, "missing_environment": 3, "duplicate_binding": 3, "process_environment": 2, "process_arguments": 2, "process_start": 3, "process_cgroup": 3, "exec_flags": 3, "nul_argv": 3, "hidden_processes": 3, "unreadable_fds": 3, "no_master_holder": 3, "fixture_escape": 3, "inherited_environment": 3, "loader_environment": 3} {
		t.Run(which, func(t *testing.T) {
			root, config, manifest, release := identityFixture(t)
			core := "rosie-rt-core"
			remove := func(path string) {
				t.Helper()
				if err := os.Remove(filepath.Join(root, path)); err != nil {
					t.Fatal(err)
				}
			}
			switch which {
			case "missing_manager":
				remove("hostcheck/systemd/" + core + ".Unit.json")
			case "malformed_manager":
				hostcheckWrite(t, root, "hostcheck/systemd/"+core+".Service.json", `{"type":"a{sv}","data":[]}`)
			case "missing_manifest":
				remove("release.json")
			case "duplicate_manifest":
				raw, err := os.ReadFile(manifest)
				if err != nil {
					t.Fatal(err)
				}
				hostcheckWrite(t, root, "release.json", strings.Replace(string(raw), "{", `{"schema":"ignored",`, 1))
			case "partial_manifest":
				delete(release.Services, core)
				identityFixtureJSON(t, root, "release.json", release)
			case "unknown_manifest_field":
				raw, err := os.ReadFile(manifest)
				if err != nil {
					t.Fatal(err)
				}
				hostcheckWrite(t, root, "release.json", strings.Replace(string(raw), "{", `{"ignored":true,`, 1))
			case "invalid_digest":
				s := release.Services[core]
				s.ExecutableSHA256 = "bad"
				release.Services[core] = s
				identityFixtureJSON(t, root, "release.json", release)
			case "unknown_service":
				release.Services["foreign"] = release.Services[core]
				delete(release.Services, core)
				identityFixtureJSON(t, root, "release.json", release)
			case "pending_reload":
				identityFixtureProperty(t, root, core, "Unit", "NeedDaemonReload", true)
			case "changed_unit_path":
				identityFixtureProperty(t, root, core, "Unit", "FragmentPath", "/run/systemd/system/rosie-rt-core.service")
			case "unlisted_dropin":
				hostcheckWrite(t, root, "run/systemd/system/rosie-rt-core.service.d/90-foreign.conf", "[Service]\nEnvironment=ROSIE_RT_PAIR_ID=foreign\n")
				identityFixtureProperty(t, root, core, "Unit", "DropInPaths", []string{release.Services[core].UnitFiles[1].Path, "/run/systemd/system/rosie-rt-core.service.d/90-foreign.conf"})
			case "missing_environment":
				remove("etc/rosie-rt-core/control.env")
			case "duplicate_binding":
				hostcheckWrite(t, root, "etc/rosie-rt-core/control.env", "ROSIE_RT_PAIR_ID=bench\nROSIE_RT_PAIR_ID=bench\n")
			case "process_environment":
				hostcheckWrite(t, root, "proc/101/environ", "ROSIE_RT_PAIR_ID=other\x00")
			case "process_arguments":
				hostcheckWrite(t, root, "proc/101/cmdline", "/opt/old/runtime\x00--old\x00")
			case "process_start":
				hostcheckWrite(t, root, "proc/101/stat", identityFixtureStat(101, "0"))
			case "process_cgroup":
				hostcheckWrite(t, root, "proc/101/cgroup", "0::/user.slice/old-runtime.service\n")
			case "exec_flags":
				identityFixtureProperty(t, root, core, "Service", "ExecStartEx", [][]any{{release.Services[core].Argv[0], release.Services[core].Argv, []string{"no-env-expand"}, 0, 1, 0, 0, 101, 0, 0}})
			case "nul_argv":
				hostcheckWrite(t, root, "proc/101/cmdline", "unterminated")
			case "hidden_processes":
				hostcheckWrite(t, root, "proc/mounts", "proc /proc proc rw,hidepid=2 0 0\n")
			case "unreadable_fds":
				hostcheckWrite(t, root, "proc/404/fd", "not a descriptor directory")
			case "no_master_holder":
				remove("proc/101/fd/7")
			case "fixture_escape":
				remove("proc/101/exe")
				identityFixtureLink(t, root, "proc/101/exe", "/proc/self/exe")
			case "inherited_environment":
				identityFixtureProperty(t, root, core, "Service", "PassEnvironment", []string{"ROSIE_RT_PAIR_ID"})
			case "loader_environment":
				identityFixtureProperty(t, root, core, "Service", "Environment", []string{"LD_PRELOAD=/unverified.so"})
			}
			r, _, code := identityRun(t, config, manifest)
			identityAssertUnqualified(t, r, code, wantCode)
		})
	}
}

func TestHostIdentityRestartDuringObservation(t *testing.T) {
	for _, which := range []string{"manager_restart", "pid_reuse", "environment_change", "unit_change", "reexec", "process_arguments", "process_environment", "installed_image"} {
		t.Run(which, func(t *testing.T) {
			root, _, _, release := identityFixture(t)
			h := &hostInspector{root: root}
			s := hostServiceIdentity{Service: "rosie-rt-core", Layers: map[string]hostIdentityCheck{}}
			calls := 0
			snapshot := func(name string) (hostManager, error) {
				calls++
				m, err := h.managerSnapshot(name)
				if calls == 2 {
					switch which {
					case "manager_restart":
						m.MainPID = 404
						m.StartUS = 777
					case "pid_reuse":
						hostcheckWrite(t, root, "proc/101/stat", identityFixtureStat(101, "20002"))
					case "environment_change":
						hostcheckWrite(t, root, "etc/rosie-rt-core/control.env", "ROSIE_RT_PAIR_ID=other\n")
					case "unit_change":
						hostcheckWrite(t, root, "etc/systemd/system/rosie-rt-core.service", "[Service]\nExecStart=/old/runtime\n")
					case "reexec":
						hostcheckWrite(t, root, "hostcheck/images/rosie-rt-core", "different loaded bytes\n")
					case "process_arguments":
						hostcheckWrite(t, root, "proc/101/cmdline", "/old/runtime\x00")
					case "process_environment":
						hostcheckWrite(t, root, "proc/101/environ", "ROSIE_RT_PAIR_ID=other\x00")
					case "installed_image":
						hostcheckWrite(t, root, "opt/rosie/rt-core/build/rosie-rt-core", "different installed bytes\n")
					}
				}
				return m, err
			}
			h.inspectServiceSnapshots(&s, release.Services[s.Service], "../..", snapshot)
			if calls != 3 || s.Layers["stable_observation"].Status != "unknown" || identityAllMatch(s.Layers) {
				t.Fatalf("mixed observation qualified: calls=%d %+v", calls, s)
			}
		})
	}
}

// TestHostIdentityClosingIncarnation changes identity after the final command
// read opens; matching bytes cannot qualify a reused PID or a changed manager.
func TestHostIdentityClosingIncarnation(t *testing.T) {
	for _, which := range []string{"pid_reuse", "process_inode", "image_inode", "closing_manager"} {
		t.Run(which, func(t *testing.T) {
			root, _, _, release := identityFixture(t)
			h := &hostInspector{root: root}
			s := hostServiceIdentity{Service: "rosie-rt-control", Layers: map[string]hostIdentityCheck{}}
			command := filepath.Join(root, "proc/102/cmdline")
			raw, err := os.ReadFile(command)
			if err != nil {
				t.Fatal(err)
			}
			calls := 0
			opened := make(chan *os.File)
			done := make(chan struct{})
			go func() {
				defer close(done)
				h.inspectServiceSnapshots(&s, release.Services[s.Service], "../..", func(name string) (hostManager, error) {
					calls++
					m, err := h.managerSnapshot(name)
					if calls == 2 {
						if err := os.Remove(command); err != nil {
							t.Error(err)
							return m, err
						}
						if err := syscall.Mkfifo(command, 0600); err != nil {
							t.Error(err)
							return m, err
						}
						go func() {
							f, err := os.OpenFile(command, os.O_WRONLY, 0)
							if err != nil {
								t.Error(err)
							}
							opened <- f
						}()
					}
					return m, err
				})
			}()
			var writer *os.File
			select {
			case writer = <-opened:
			case <-done:
				t.Fatal("inspection returned before the final command read")
			}
			if writer == nil {
				t.Fatal("command pipe unavailable")
			}
			t.Cleanup(func() { _ = writer.Close(); <-done })
			switch which {
			case "pid_reuse":
				hostcheckWrite(t, root, "proc/102/stat", identityFixtureStat(102, "20002"))
			case "process_inode":
				base := filepath.Join(root, "proc/102")
				old := filepath.Join(root, "old-process")
				entries, err := os.ReadDir(base)
				if err != nil {
					t.Fatal(err)
				}
				if err := os.Rename(base, old); err != nil {
					t.Fatal(err)
				}
				if err := os.Mkdir(base, 0755); err != nil {
					t.Fatal(err)
				}
				for _, entry := range entries {
					if err := os.Rename(filepath.Join(old, entry.Name()), filepath.Join(base, entry.Name())); err != nil {
						t.Fatal(err)
					}
				}
			case "image_inode":
				path := filepath.Join(root, "hostcheck/images/rosie-rt-control")
				image, err := os.ReadFile(path)
				if err != nil {
					t.Fatal(err)
				}
				if err := os.Rename(path, path+".old"); err != nil {
					t.Fatal(err)
				}
				hostcheckWrite(t, root, "hostcheck/images/rosie-rt-control", string(image))
			case "closing_manager":
				identityFixtureProperty(t, root, s.Service, "Service", "ExecMainStartTimestampMonotonic", 200020000)
			}
			if _, err := writer.Write(raw); err != nil {
				t.Fatal(err)
			}
			if err := writer.Close(); err != nil {
				t.Fatal(err)
			}
			<-done
			if calls != 3 || s.start != "10002" || s.Layers["process_identity"].Status != "match" || s.Layers["stable_observation"].Status != "unknown" || identityAllMatch(s.Layers) {
				t.Fatalf("late incarnation change qualified: calls=%d %+v", calls, s)
			}
		})
	}
}

func TestHostIdentityMasterAliasAndForeignOnly(t *testing.T) {
	root, config, manifest, _ := identityFixture(t)
	if err := os.Link(filepath.Join(root, "dev/EtherCAT0"), filepath.Join(root, "dev/master-alias")); err != nil {
		t.Fatal(err)
	}
	if err := os.Remove(filepath.Join(root, "proc/101/fd/7")); err != nil {
		t.Fatal(err)
	}
	hostcheckWrite(t, root, "proc/404/stat", identityFixtureStat(404, "777"))
	identityFixtureLink(t, root, "proc/404/fd/9", "../../../dev/master-alias")
	identityFixtureLink(t, root, "proc/404/task/404/fd", "../../fd")
	r, _, code := identityRun(t, config, manifest)
	identityAssertUnqualified(t, r, code, 2)
	if r.MasterOwner.Status != "conflict" || !strings.Contains(r.MasterOwner.Observed, "404") {
		t.Fatalf("foreign sole owner through device alias missed: %+v", r.MasterOwner)
	}
}

// TestHostIdentityTaskDescriptorTables requires all thread tables to be visible;
// CLONE_FILES unsharing must not conceal a holder or misattribute its process.
func TestHostIdentityTaskDescriptorTables(t *testing.T) {
	for _, which := range []string{"core_task_only", "foreign_task_only", "missing_tasks", "empty_tasks", "missing_task_fds", "unreadable_task_fds"} {
		t.Run(which, func(t *testing.T) {
			root, config, manifest, _ := identityFixture(t)
			wantCode, wantStatus := 3, "unknown"
			switch which {
			case "core_task_only":
				if err := os.Remove(filepath.Join(root, "proc/101/fd/7")); err != nil {
					t.Fatal(err)
				}
				identityFixtureLink(t, root, "proc/101/task/111/fd/7", "../../../../../dev/EtherCAT0")
				wantCode, wantStatus = 0, "match"
			case "foreign_task_only":
				hostcheckWrite(t, root, "proc/404/stat", identityFixtureStat(404, "777"))
				if err := os.MkdirAll(filepath.Join(root, "proc/404/task/404/fd"), 0755); err != nil {
					t.Fatal(err)
				}
				identityFixtureLink(t, root, "proc/404/task/405/fd/9", "../../../../../dev/EtherCAT0")
				wantCode, wantStatus = 2, "conflict"
			case "missing_tasks":
				if err := os.Rename(filepath.Join(root, "proc/102/task"), filepath.Join(root, "proc/102/hidden-task")); err != nil {
					t.Fatal(err)
				}
			case "empty_tasks":
				if err := os.Rename(filepath.Join(root, "proc/102/task/102"), filepath.Join(root, "proc/102/hidden-task")); err != nil {
					t.Fatal(err)
				}
			case "missing_task_fds":
				if err := os.MkdirAll(filepath.Join(root, "proc/102/task/112"), 0755); err != nil {
					t.Fatal(err)
				}
			case "unreadable_task_fds":
				hostcheckWrite(t, root, "proc/102/task/112/fd", "not a descriptor directory")
			}
			r, _, code := identityRun(t, config, manifest)
			if code != wantCode || r.MasterOwner.Status != wantStatus {
				t.Fatalf("exit=%d want=%d master=%+v", code, wantCode, r.MasterOwner)
			}
			if which == "foreign_task_only" && r.MasterOwner.Observed != "foreign or unqualified master holder PIDs=[404]" {
				t.Fatalf("task holder not attributed to process 404: %+v", r.MasterOwner)
			}
			if wantCode != 0 {
				identityAssertUnqualified(t, r, code, wantCode)
			} else if r.QualifiedMatches != 3 {
				t.Fatalf("visible core task did not qualify: %+v", r)
			}
		})
	}
}

func TestHostIdentityExpansionAndBoundaries(t *testing.T) {
	env := map[string]string{"SPLIT": `one "two three" ''`, "LITERAL": "literal space $OTHER", "EMPTY": ""}
	got, err := identityExpand([]string{"/exe", "$SPLIT", "--key=${LITERAL}", "${EMPTY}", "$EMPTY"}, env)
	want := []string{"/exe", "one", "two three", "", "--key=literal space $OTHER", ""}
	if err != nil || !reflect.DeepEqual(got, want) {
		t.Fatalf("expansion: %q %v", got, err)
	}
	if identityArgv([]string{"/exe", "one two"}) == identityArgv([]string{"/exe", "one", "two"}) {
		t.Fatal("argument boundaries lost")
	}
	for _, raw := range []string{`/exe $BROKEN-tail`, `/exe %i`, "/exe ${BROKEN", `/exe $$`} {
		if _, err := identityExpand(strings.Fields(raw), env); err == nil {
			t.Fatalf("accepted ambiguous expansion %q", raw)
		}
	}
	for _, raw := range []string{`"unclosed`, `a\x20b`, `a"b"`, `"a"b`, "multi\nline"} {
		if _, err := identityWords(raw); err == nil {
			t.Fatalf("accepted ambiguous syntax %q", raw)
		}
	}
}

// The helper blocks on an explicit pipe, not a sleep. Its loaded executable stays
// alive until the parent finishes inspecting a replacement of its disk pathname.
func TestHostIdentityLoadedImageHelper(t *testing.T) {
	if os.Getenv("HOSTIDENTITY_IMAGE_HELPER") != "1" {
		return
	}
	fmt.Fprintln(os.Stdout, "ready")
	_, _ = io.Copy(io.Discard, os.Stdin)
	os.Exit(0)
}

func TestHostIdentityRealDeletedLoadedImage(t *testing.T) {
	root, _, _, _ := identityFixture(t)
	image := filepath.Join(root, "running-test")
	self, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(self)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(image, raw, 0755); err != nil {
		t.Fatal(err)
	}
	cmd := exec.Command(image, "-test.run=^TestHostIdentityLoadedImageHelper$")
	cmd.Env = append(os.Environ(), "HOSTIDENTITY_IMAGE_HELPER=1")
	stdin, err := cmd.StdinPipe()
	if err != nil {
		t.Fatal(err)
	}
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		t.Fatal(err)
	}
	if err := cmd.Start(); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = stdin.Close(); _ = cmd.Process.Kill(); _ = cmd.Wait() })
	line, err := bufio.NewReader(stdout).ReadString('\n')
	if err != nil || line != "ready\n" {
		t.Fatalf("helper readiness: %q %v", line, err)
	}
	h := &hostInspector{root: "/"}
	proc := fmt.Sprintf("/proc/%d/exe", cmd.Process.Pid)
	before, _, err := h.identityDigest(proc)
	if err != nil {
		t.Fatal(err)
	}
	if before != identityFixtureSHA(string(raw)) {
		t.Fatal("loaded executable did not match initial release")
	}
	if err := os.WriteFile(image+".new", []byte("replacement release bytes\n"), 0755); err != nil {
		t.Fatal(err)
	}
	if err := os.Rename(image+".new", image); err != nil {
		t.Fatal(err)
	}
	loaded, _, err := h.identityDigest(proc)
	if err != nil {
		t.Fatal(err)
	}
	installed, _, err := h.identityDigest(image)
	if err != nil {
		t.Fatal(err)
	}
	target, err := os.Readlink(proc)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.HasSuffix(target, " (deleted)") || loaded != before || installed != identityFixtureSHA("replacement release bytes\n") || loaded == installed {
		t.Fatalf("loaded bytes confused with replacement: target=%q loaded=%s installed=%s", target, loaded, installed)
	}
}
