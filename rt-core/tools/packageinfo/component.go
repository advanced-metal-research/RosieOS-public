package main

import (
	"archive/tar"
	"compress/gzip"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

// Unverified release proposal: one version selects the complete runtime payload.
const componentVersion = "0.1.0"

// componentVersionError identifies incompatible or pre-versioning packages so
// callers cannot silently relabel an existing payload with the emitter's version.
type componentVersionError struct {
	ManifestVersion string
	EmitterVersion  string
}

func (e *componentVersionError) Error() string {
	if e.ManifestVersion == "" {
		return fmt.Sprintf("pre-versioning manifest: component version missing; emitter=%q", e.EmitterVersion)
	}
	return fmt.Sprintf("component version mismatch: manifest=%q emitter=%q", e.ManifestVersion, e.EmitterVersion)
}

// componentDescriptor preserves the robot-stack release.v1 primary-binary contract.
// The retained archive receipt carries the companion files without changing that contract.
type componentDescriptor struct {
	Name                string   `json:"name"`
	Role                string   `json:"role"`
	Version             string   `json:"version"`
	ReleaseTag          string   `json:"release_tag"`
	WireFamily          string   `json:"wire_family"`
	RuntimeRequirements string   `json:"runtime_requirements"`
	Artifact            string   `json:"artifact"`
	SHA256              string   `json:"sha256"`
	Source              string   `json:"source"`
	Toolchain           string   `json:"toolchain"`
	SourceFiles         []string `json:"source_files"`
	Receipts            []string `json:"receipts"`
}

// describeComponent requires the producer's version to match the emitter, preventing
// relabeling, and derives target requirements from every shipped ELF.
func describeComponent(m manifest, emitterVersion string) (componentDescriptor, string, error) {
	if m.ComponentVersion == "" || m.ComponentVersion != emitterVersion {
		return componentDescriptor{}, "", &componentVersionError{ManifestVersion: m.ComponentVersion, EmitterVersion: emitterVersion}
	}
	d := componentDescriptor{
		Name: "rosie-rt-core", Role: "realtime-core", Version: m.ComponentVersion,
		ReleaseTag: "rt-core-v" + m.ComponentVersion, Artifact: "bin/rosie-rt-core",
		Source: m.Inputs.GitSHA, Toolchain: m.Inputs.Go + "; " + m.Inputs.Compiler,
		WireFamily:  "control.sock HTTP/JSON application v1 (contract_version=2, rosie.rt-control.request.v1/response.v1); jog.sock datagram and WSS /v1/jog local_jog_update; private native IPC 1.7; EtherCAT PDO/SDO; daemon and adapter have no native NATS subjects",
		SourceFiles: []string{"rt-core/Makefile", "rt-core/tools/build-igh-userlib.sh", "rt-core/go.mod", "rt-core/go.sum", "rt-core/tools/package-runtime.sh", "rt-core/tools/packageinfo/component.go", "rt-core/tools/packageinfo/main.go", "rt-core/tools/packageinfo/git.sh", "rt-core/tools/verify-package.sh", "rt-core/host/rosie-rt-core.service", "rt-core/host/rosie-rt-control.service", "rt-core/host/rosie-rt-natspublisher.service"},
	}
	arch, machine := "", ""
	for _, a := range m.Files {
		if a.Path == d.Artifact && a.ELF != nil {
			d.SHA256, machine = a.SHA256, a.ELF.Machine
		}
	}
	switch machine {
	case "EM_X86_64":
		arch = "amd64"
	case "EM_AARCH64":
		arch = "arm64"
	default:
		return d, "", fmt.Errorf("unsupported or missing primary ELF machine: %s", machine)
	}
	requirements := []string{"Linux " + arch + " ELF (" + machine + "); measured packaged artifacts; backend=" + m.Backend}
	for _, a := range m.Files {
		if a.ELF == nil {
			continue
		}
		id := a.ELF
		if id.Machine != machine {
			return d, "", fmt.Errorf("mixed ELF architectures: %s", a.Path)
		}
		parts := []string{a.Path}
		if id.Interpreter != "" {
			parts = append(parts, "interpreter="+id.Interpreter)
		}
		if len(id.Needed) == 0 {
			parts = append(parts, "no dynamic dependencies")
		} else {
			parts = append(parts, "NEEDED="+strings.Join(id.Needed, ","))
		}
		versions := []string{}
		for _, tags := range id.Versions {
			versions = append(versions, tags...)
		}
		if len(versions) != 0 {
			parts = append(parts, "required symbol versions="+strings.Join(unique(versions), ","))
		}
		requirements = append(requirements, strings.Join(parts, " "))
		if a.Path == "bin/rt-natspublisher" {
			d.WireFamily += "; optional observer emits compatibility v4-shape robot.v4.rtcore.status.v1 and robot.v4.rtcore.telemetry.batch.v1 on robot/v4/rtcore.<host>.status and robot/v4/rtcore.<host>.telemetry.batch; no NATS command ingress"
		}
	}
	requirements = append(requirements, "live operation requires external matching IgH kernel master, PREEMPT_RT and qualified cell wiring; optional publisher requires external NATS authorization/endpoints; target loader and hardware qualification not established")
	d.RuntimeRequirements = strings.Join(requirements, "; ")
	archive := d.Name + "-" + d.Version + "-linux-" + arch + ".tar.gz"
	d.Receipts = []string{"build.log", "rt_package.json", "verify-package.log", archive}
	return d, archive, nil
}

// exclusiveWrite preserves previous and interrupted outputs instead of replacing them.
func exclusiveWrite(path string, raw []byte) error {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0644)
	if err != nil {
		return err
	}
	_, writeErr := f.Write(raw)
	closeErr := f.Close()
	if writeErr != nil {
		return writeErr
	}
	return closeErr
}

// runtimeArchive retains the complete verified install tree with normalized metadata.
// Neither source mtimes nor destination names can change the compressed output.
func runtimeArchive(root, path string, m manifest, epoch int64) (err error) {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0644)
	if err != nil {
		return err
	}
	defer func() {
		if e := f.Close(); err == nil {
			err = e
		}
	}()
	z := gzip.NewWriter(f)
	defer func() {
		if e := z.Close(); err == nil {
			err = e
		}
	}()
	tw := tar.NewWriter(z)
	defer func() {
		if e := tw.Close(); err == nil {
			err = e
		}
	}()
	files := append([]artifact{{Path: "rt_package.json", InstallMode: "0644"}}, m.Files...)
	for _, a := range files {
		mode, err := strconv.ParseInt(a.InstallMode, 8, 64)
		if err != nil {
			return err
		}
		raw, err := os.ReadFile(filepath.Join(root, a.Path))
		if err != nil {
			return err
		}
		if err := tw.WriteHeader(&tar.Header{Name: a.Path, Mode: mode, Size: int64(len(raw)), ModTime: time.Unix(epoch, 0), Typeflag: tar.TypeReg}); err != nil {
			return err
		}
		if _, err := tw.Write(raw); err != nil {
			return err
		}
	}
	return nil
}

// emitComponent verifies the install before creating a new release asset directory.
// Missing producer logs are explicit; inspection output never impersonates a build log.
func emitComponent(root, output, buildLog string) error {
	m, err := readManifest(root)
	if err != nil {
		return err
	}
	d, archive, err := describeComponent(m, componentVersion)
	if err != nil {
		return err
	}
	if err = integrity(root, m); err != nil {
		return err
	}
	epoch := int64(0)
	if m.Inputs.SourceDateEpoch != "" {
		epoch, err = strconv.ParseInt(m.Inputs.SourceDateEpoch, 10, 64)
		if err != nil || epoch < 0 {
			return fmt.Errorf("invalid SOURCE_DATE_EPOCH")
		}
	}
	log := []byte("Producer invocation unavailable: prebuilt inputs; compiler/Go banners and ELF measurements are inspection only. Supply ROSIE_RT_PACKAGE_BUILD_LOG for a captured producer log.\n")
	if buildLog != "" {
		log, err = os.ReadFile(buildLog)
		if err != nil {
			return err
		}
		if len(log) == 0 {
			return fmt.Errorf("empty producer build log")
		}
	}
	verified, err := exec.Command("bash", filepath.Join(root, "tools/verify-package.sh"), root).CombinedOutput()
	if err != nil {
		return fmt.Errorf("package verification refused: %w\n%s", err, verified)
	}
	manifestRaw, err := os.ReadFile(filepath.Join(root, "rt_package.json"))
	if err != nil {
		return err
	}
	descriptor, err := json.MarshalIndent(d, "", "  ")
	if err != nil {
		return err
	}
	if err = os.Mkdir(output, 0755); err != nil {
		return err
	}
	for _, file := range []struct {
		name string
		raw  []byte
	}{
		{"build.log", log}, {"rt_package.json", manifestRaw},
		{"verify-package.log", verified}, {d.Name + ".json", append(descriptor, '\n')},
	} {
		if err = exclusiveWrite(filepath.Join(output, file.name), file.raw); err != nil {
			return err
		}
	}
	if err = runtimeArchive(root, filepath.Join(output, archive), m, epoch); err != nil {
		return err
	}
	files, err := inventory(output)
	if err != nil {
		return err
	}
	// inventory excludes rt_package.json for install verification; asset sums include it.
	manifestSum, err := digest(filepath.Join(output, "rt_package.json"))
	if err != nil {
		return err
	}
	lines := []string{manifestSum + "  rt_package.json\n"}
	for _, a := range files {
		lines = append(lines, a.SHA256+"  "+a.Path+"\n")
	}
	return exclusiveWrite(filepath.Join(output, "SHA256SUMS"), []byte(strings.Join(unique(lines), "")))
}
