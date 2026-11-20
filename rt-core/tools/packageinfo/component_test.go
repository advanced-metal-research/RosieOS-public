package main

import (
	"archive/tar"
	"compress/gzip"
	"encoding/json"
	"errors"
	"io"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strings"
	"testing"
)

// TestComponentManifestVersion refuses relabeling and pre-versioning manifests
// while accepting packages explicitly stamped with the emitter's version.
func TestComponentManifestVersion(t *testing.T) {
	for _, tc := range []struct {
		name, versionField, emitterVersion, wantError string
	}{
		{"mismatch", `"component_version":"0.1.0",`, "0.2.0", `component version mismatch: manifest="0.1.0" emitter="0.2.0"`},
		{"matching-current", `"component_version":"0.1.0",`, "0.1.0", ""},
		{"matching-next", `"component_version":"0.2.0",`, "0.2.0", ""},
		{"pre-versioning", "", "0.2.0", `pre-versioning manifest: component version missing; emitter="0.2.0"`},
	} {
		t.Run(tc.name, func(t *testing.T) {
			root := runtimeDir(t)
			raw := `{"schema":"rosie.rt-core.package.v1","format_version":1,` + tc.versionField +
				`"backend":"simulation","digest_excludes":["rt_package.json"],"build_inputs":{"git_sha":"` + strings.Repeat("a", 40) +
				`"},"files":[{"path":"bin/rosie-rt-core","elf":{"machine":"EM_X86_64"}}]}`
			if err := os.WriteFile(filepath.Join(root, "rt_package.json"), []byte(raw), 0644); err != nil {
				t.Fatal(err)
			}
			m, err := readManifest(root)
			if err != nil {
				t.Fatal(err)
			}
			d, archive, err := describeComponent(m, tc.emitterVersion)
			if tc.wantError != "" {
				if err == nil || err.Error() != tc.wantError {
					t.Fatalf("want %s, got descriptor=%+v error=%v", tc.wantError, d, err)
				}
				var versionErr *componentVersionError
				if !errors.As(err, &versionErr) || versionErr.ManifestVersion != m.ComponentVersion || versionErr.EmitterVersion != tc.emitterVersion {
					t.Fatalf("missing typed version identities: %v", err)
				}
				if !reflect.DeepEqual(d, componentDescriptor{}) || archive != "" {
					t.Fatal("refused version produced release output")
				}
				return
			}
			if err != nil || d.Version != tc.emitterVersion || d.ReleaseTag != "rt-core-v"+tc.emitterVersion || archive != "rosie-rt-core-"+tc.emitterVersion+"-linux-amd64.tar.gz" {
				t.Fatalf("matching version refused or mislabeled: %+v %s %v", d, archive, err)
			}
		})
	}
}

// TestEmitComponentRefusesPreVersioningBeforeOutput prevents legacy manifests
// from reaching payload verification or creating a release directory.
func TestEmitComponentRefusesPreVersioningBeforeOutput(t *testing.T) {
	root := runtimeDir(t)
	raw := `{"schema":"rosie.rt-core.package.v1","format_version":1,"backend":"simulation","digest_excludes":["rt_package.json"],"build_inputs":{"git_sha":"` + strings.Repeat("a", 40) + `"}}`
	if err := os.WriteFile(filepath.Join(root, "rt_package.json"), []byte(raw), 0644); err != nil {
		t.Fatal(err)
	}
	output := filepath.Join(root, "release")
	var versionErr *componentVersionError
	if err := emitComponent(root, output, ""); !errors.As(err, &versionErr) || versionErr.ManifestVersion != "" || versionErr.EmitterVersion != componentVersion {
		t.Fatalf("legacy package was not refused with its version identity: %v", err)
	}
	if _, err := os.Stat(output); !os.IsNotExist(err) {
		t.Fatalf("refused package created output: %v", err)
	}
}

// TestComponentDescriptorContract checks the release fields and measured dependency
// union, including an adapter requirement absent from the primary binary.
func TestComponentDescriptorContract(t *testing.T) {
	m := manifest{ComponentVersion: componentVersion, Backend: "simulation", Inputs: inputs{GitSHA: strings.Repeat("a", 40), Go: "go fixture", Compiler: "compiler fixture"}, Files: []artifact{
		{Path: "bin/rosie-rt-core", SHA256: strings.Repeat("b", 64), ELF: &elfIdentity{Machine: "EM_X86_64", Needed: []string{"libc.so.6"}, Versions: map[string][]string{"libc.so.6": {"GLIBC_2.34"}}}},
		{Path: "bin/rt-control", ELF: &elfIdentity{Machine: "EM_X86_64", Interpreter: "/lib64/ld-linux-x86-64.so.2", Needed: []string{"libc.so.6"}, Versions: map[string][]string{"libc.so.6": {"GLIBC_2.35"}}}},
	}}
	d, archive, err := describeComponent(m, componentVersion)
	if err != nil {
		t.Fatal(err)
	}
	if d.Name != "rosie-rt-core" || d.Role != "realtime-core" || d.Version != "0.1.0" || d.ReleaseTag != "rt-core-v0.1.0" || d.Artifact != "bin/rosie-rt-core" || d.SHA256 != strings.Repeat("b", 64) || d.Source != strings.Repeat("a", 40) || d.Toolchain != "go fixture; compiler fixture" || archive != "rosie-rt-core-0.1.0-linux-amd64.tar.gz" {
		t.Fatalf("wrong release identity: %+v %s", d, archive)
	}
	for _, required := range []string{"Linux amd64 ELF", "backend=simulation", "GLIBC_2.34", "GLIBC_2.35", "/lib64/ld-linux-x86-64.so.2"} {
		if !strings.Contains(d.RuntimeRequirements, required) {
			t.Fatalf("missing %s: %s", required, d.RuntimeRequirements)
		}
	}
	if strings.Contains(d.WireFamily, "optional observer emits") {
		t.Fatal("absent publisher advertised")
	}
	if !reflect.DeepEqual(d.Receipts, []string{"build.log", "rt_package.json", "verify-package.log", "rosie-rt-core-0.1.0-linux-amd64.tar.gz"}) {
		t.Fatal(d.Receipts)
	}
	raw, err := json.Marshal(d)
	if err != nil {
		t.Fatal(err)
	}
	var fields map[string]any
	if err = json.Unmarshal(raw, &fields); err != nil {
		t.Fatal(err)
	}
	keys := []string{}
	for k := range fields {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	if !reflect.DeepEqual(keys, []string{"artifact", "name", "receipts", "release_tag", "role", "runtime_requirements", "sha256", "source", "source_files", "toolchain", "version", "wire_family"}) {
		t.Fatal(keys)
	}
	m.Files = append(m.Files, artifact{Path: "bin/rt-natspublisher", ELF: &elfIdentity{Machine: "EM_X86_64"}})
	d, _, err = describeComponent(m, componentVersion)
	if err != nil || !strings.Contains(d.WireFamily, "robot/v4/rtcore.<host>.status") {
		t.Fatalf("publisher contract missing: %+v %v", d, err)
	}
	m.Files[1].ELF.Machine = "EM_AARCH64"
	if _, _, err = describeComponent(m, componentVersion); err == nil {
		t.Fatal("mixed architecture accepted")
	}
	for i := range m.Files {
		m.Files[i].ELF.Machine = "EM_AARCH64"
	}
	_, archive, err = describeComponent(m, componentVersion)
	if err != nil || archive != "rosie-rt-core-0.1.0-linux-arm64.tar.gz" {
		t.Fatalf("arm64 naming: %s %v", archive, err)
	}
	m.Files[0].ELF = nil
	if _, _, err = describeComponent(m, componentVersion); err == nil {
		t.Fatal("missing primary ELF accepted")
	}
}

// TestRuntimeArchiveMetadataAndNoOverwrite verifies portable executable permissions,
// fixed timestamps, byte identity and retention of an existing output.
func TestRuntimeArchiveMetadataAndNoOverwrite(t *testing.T) {
	root := runtimeDir(t)
	if err := os.Mkdir(filepath.Join(root, "bin"), 0755); err != nil {
		t.Fatal(err)
	}
	for _, file := range []string{"rt_package.json", "bin/rosie-rt-core"} {
		if err := os.WriteFile(filepath.Join(root, file), []byte("abc"), 0600); err != nil {
			t.Fatal(err)
		}
	}
	m := manifest{Files: []artifact{{Path: "bin/rosie-rt-core", InstallMode: "0755"}}}
	first, second := filepath.Join(root, "first.tar.gz"), filepath.Join(root, "second.tar.gz")
	for _, path := range []string{first, second} {
		if err := runtimeArchive(root, path, m, 1); err != nil {
			t.Fatal(err)
		}
	}
	a, err := os.ReadFile(first)
	if err != nil {
		t.Fatal(err)
	}
	b, err := os.ReadFile(second)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(a, b) {
		t.Fatal("archive depends on output name")
	}
	if err = runtimeArchive(root, first, m, 1); !os.IsExist(err) {
		t.Fatalf("overwritten archive: %v", err)
	}
	f, err := os.Open(first)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	z, err := gzip.NewReader(f)
	if err != nil {
		t.Fatal(err)
	}
	defer z.Close()
	tr := tar.NewReader(z)
	for i, name := range []string{"rt_package.json", "bin/rosie-rt-core"} {
		h, err := tr.Next()
		if err != nil {
			t.Fatal(err)
		}
		mode := int64(0644)
		if i == 1 {
			mode = 0755
		}
		if h.Name != name || h.Mode != mode || h.ModTime.Unix() != 1 || h.Uid != 0 || h.Gid != 0 || h.Size != 3 {
			t.Fatalf("unsafe header: %+v", h)
		}
		raw, err := io.ReadAll(tr)
		if err != nil || string(raw) != "abc" {
			t.Fatalf("payload differs: %s %v", raw, err)
		}
	}
	if _, err = tr.Next(); err != io.EOF {
		t.Fatalf("extra archive data: %v", err)
	}
	if err = exclusiveWrite(second, []byte("replace")); !os.IsExist(err) {
		t.Fatal("receipt overwritten")
	}
}
