// rt-package records and verifies relocatable artifacts without executing their code.
package main

import (
	"crypto/sha256"
	"debug/elf"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"io/fs"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"regexp"
	"rosieos/robotdesc"
	"sort"
	"strconv"
	"strings"
	"syscall"
)

// Measured package wire contract: version 1 excludes its own digest to avoid recursion.
const schema = "rosie.rt-core.package.v1"

type elfIdentity struct {
	Machine     string              `json:"machine"`
	Needed      []string            `json:"needed"`
	Versions    map[string][]string `json:"version_needs"`
	Defined     []string            `json:"version_definitions"`
	SONAME      string              `json:"soname,omitempty"`
	Interpreter string              `json:"interpreter,omitempty"`
	GLIBC       []string            `json:"glibc_tags"`
	ECRTSymbols []string            `json:"ecrt_symbols"`
}
type artifact struct {
	Path         string       `json:"path"`
	Mode         string       `json:"mode"`
	InstallMode  string       `json:"install_mode"`
	SHA256       string       `json:"sha256"`
	Reproducible bool         `json:"reproducible"`
	Reason       string       `json:"reason,omitempty"`
	ELF          *elfIdentity `json:"elf,omitempty"`
}
type inputs struct {
	GitSHA           string `json:"git_sha"`
	Dirty            bool   `json:"dirty"`
	Compiler         string `json:"compiler"`
	Go               string `json:"go"`
	IgHVersion       string `json:"igh_version"`
	IgHArchiveSHA256 string `json:"igh_archive_sha256"`
	Architecture     string `json:"architecture"`
	Kernel           string `json:"build_kernel"`
	KernelContract   string `json:"kernel_requirement"`
	DaemonVersion    string `json:"daemon_version"`
	SourceDateEpoch  string `json:"source_date_epoch,omitempty"`
}
type manifest struct {
	Schema           string   `json:"schema"`
	FormatVersion    int      `json:"format_version"`
	ComponentVersion string   `json:"component_version"`
	Backend          string   `json:"backend"`
	Excluded         []string `json:"digest_excludes"`
	Inputs           inputs   `json:"build_inputs"`
	// RobotDescriptions is every robot description the package carries under
	// robot_description/robots/, by its manifest identity; verify recomputes
	// each so a tampered or missing description cannot compile.
	RobotDescriptions []robotDescription `json:"robot_descriptions"`
	Files             []artifact         `json:"files"`
}

type robotDescription struct {
	ModelID                string `json:"model_id"`
	Path                   string `json:"path"`
	RobotDescriptionSHA256 string `json:"robot_description_sha256"`
}

// robotDescriptions lists the package's descriptions with their identities.
func robotDescriptions(root string) ([]robotDescription, error) {
	store := filepath.Join(root, "robot_description", "robots")
	entries, err := os.ReadDir(store)
	if os.IsNotExist(err) {
		return []robotDescription{}, nil
	}
	if err != nil {
		return nil, err
	}
	out := []robotDescription{}
	for _, entry := range entries {
		if !entry.IsDir() {
			continue
		}
		identity, err := robotdesc.RobotDescriptionSHA256(filepath.Join(store, entry.Name()))
		if err != nil {
			return nil, err
		}
		out = append(out, robotDescription{ModelID: entry.Name(), Path: "robot_description/robots/" + entry.Name(), RobotDescriptionSHA256: identity})
	}
	return out, nil
}

func checkRobotDescriptions(root string, recorded []robotDescription) error {
	found, err := robotDescriptions(root)
	if err != nil {
		return err
	}
	if !reflect.DeepEqual(found, recorded) {
		return fmt.Errorf("robot descriptions differ from the manifest")
	}
	return nil
}

func digest(path string) (string, error) {
	f, err := os.Open(path)
	if err != nil {
		return "", err
	}
	defer f.Close()
	h := sha256.New()
	if _, err = io.Copy(h, f); err != nil {
		return "", err
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}
func command(dir, name string, args ...string) (string, error) {
	c := exec.Command(name, args...)
	if name == "git" {
		c = exec.Command("bash", append([]string{filepath.Join(dir, "tools/packageinfo/git.sh")}, args...)...)
	}
	c.Dir = dir
	b, err := c.Output()
	if err != nil {
		return "", fmt.Errorf("%s: %w", name, err)
	}
	return strings.TrimSpace(string(b)), nil
}
func unique(a []string) []string {
	sort.Strings(a)
	out := []string{}
	for _, s := range a {
		if len(out) == 0 || s != out[len(out)-1] {
			out = append(out, s)
		}
	}
	return out
}

// inspect reads ELF tables directly; prerequisite checks never launch packaged binaries.
func inspect(path string) (*elfIdentity, error) {
	f, err := elf.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	id := &elfIdentity{Machine: f.Machine.String(), Versions: map[string][]string{}, Defined: []string{}, GLIBC: []string{}}
	id.ECRTSymbols = []string{}
	if symbols, err := f.DynamicSymbols(); err == nil {
		for _, symbol := range symbols {
			if strings.HasPrefix(symbol.Name, "ecrt_") {
				id.ECRTSymbols = append(id.ECRTSymbols, symbol.Name)
			}
		}
		id.ECRTSymbols = unique(id.ECRTSymbols)
	}
	id.Needed, err = f.ImportedLibraries()
	if err != nil {
		return nil, err
	}
	id.Needed = unique(id.Needed)
	if f.Section(".gnu.version_r") != nil {
		needs, err := f.DynamicVersionNeeds()
		if err != nil {
			return nil, err
		}
		for _, lib := range needs {
			for _, dep := range lib.Needs {
				id.Versions[lib.Name] = append(id.Versions[lib.Name], dep.Dep)
				if strings.HasPrefix(dep.Dep, "GLIBC_") {
					id.GLIBC = append(id.GLIBC, dep.Dep)
				}
			}
		}
	}
	for lib, versions := range id.Versions {
		id.Versions[lib] = unique(versions)
	}
	id.GLIBC = unique(id.GLIBC)
	if f.Section(".gnu.version_d") != nil {
		versions, err := f.DynamicVersions()
		if err != nil {
			return nil, err
		}
		for _, v := range versions {
			id.Defined = append(id.Defined, v.Name)
		}
		id.Defined = unique(id.Defined)
	}
	if names, err := f.DynString(elf.DT_SONAME); err == nil && len(names) == 1 {
		id.SONAME = names[0]
	}
	for _, p := range f.Progs {
		if p.Type == elf.PT_INTERP {
			b, err := io.ReadAll(p.Open())
			if err != nil {
				return nil, err
			}
			id.Interpreter = strings.TrimRight(string(b), "\x00")
		}
	}
	return id, nil
}

// inventory rejects links and special files, preventing hashing or installing outside the tree.
func inventory(root string) ([]artifact, error) {
	files := []artifact{}
	err := filepath.WalkDir(root, func(path string, entry fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		info, err := entry.Info()
		if err != nil {
			return err
		}
		if info.IsDir() {
			return nil
		}
		if !info.Mode().IsRegular() || info.Mode()&(os.ModeSetuid|os.ModeSetgid|os.ModeSticky) != 0 {
			return fmt.Errorf("nonregular or privileged file: %s", path)
		}
		rel, err := filepath.Rel(root, path)
		if err != nil {
			return err
		}
		if rel == "rt_package.json" {
			return nil
		}
		sum, err := digest(path)
		if err != nil {
			return err
		}
		files = append(files, artifact{Path: filepath.ToSlash(rel), Mode: fmt.Sprintf("%04o", info.Mode().Perm()), SHA256: sum, Reproducible: true})
		return nil
	})
	return files, err
}

// installedMode keeps root-owned payloads non-writable by service identities.
// Host scripts are executable; systemd unit files (.service, .timer) are not,
// matching the modes package-runtime.sh installs them with.
func installedMode(path string) string {
	unit := strings.HasSuffix(path, ".service") || strings.HasSuffix(path, ".timer")
	if strings.HasPrefix(path, "bin/") || strings.HasPrefix(path, "tools/") || strings.HasPrefix(path, "host/") && !unit {
		return "0755"
	}
	return "0644"
}

func windowsModes(root string) bool {
	var stat syscall.Statfs_t
	// Measured Linux UAPI: V9FS_MAGIC is 0x01021997; WSL's drvfs exposes synthetic modes through 9p.
	return syscall.Statfs(root, &stat) == nil && stat.Type == 0x01021997
}

// generate records the producer's component version with its measured payload,
// preventing a later emitter from assigning its own version to an older package.
func generate(root, component, backend string) error {
	m := manifest{Schema: schema, FormatVersion: 1, ComponentVersion: componentVersion, Backend: backend, Excluded: []string{"rt_package.json"}}
	if backend != "live" && backend != "simulation" {
		return fmt.Errorf("unknown backend")
	}
	var err error
	for _, c := range []struct {
		dst  *string
		name string
		args []string
	}{
		{&m.Inputs.GitSHA, "git", []string{"rev-parse", "HEAD"}},
		{&m.Inputs.Compiler, "g++", []string{"--version"}},
		{&m.Inputs.Go, "go", []string{"version"}},
		{&m.Inputs.Architecture, "uname", []string{"-m"}},
		{&m.Inputs.Kernel, "uname", []string{"-r"}},
		{&m.Inputs.DaemonVersion, filepath.Join(root, "bin/rosie-rt-core"), []string{"--version"}},
	} {
		*c.dst, err = command(component, c.name, c.args...)
		if err != nil {
			return err
		}
	}
	m.Inputs.Compiler = strings.Split(m.Inputs.Compiler, "\n")[0]
	if m.Inputs.DaemonVersion == "" {
		return fmt.Errorf("daemon has no compiled source identity; rebuild it")
	}
	version, err := command(component, "git", "describe", "--always", "--dirty")
	if err != nil {
		return err
	}
	if version != m.Inputs.DaemonVersion {
		return fmt.Errorf("daemon source identity is stale: built=%s source=%s; rebuild it", m.Inputs.DaemonVersion, version)
	}
	status, err := command(component, "git", "status", "--porcelain", "--untracked-files=normal")
	if err != nil {
		return err
	}
	m.Inputs.Dirty = status != ""
	m.Inputs.SourceDateEpoch = os.Getenv("SOURCE_DATE_EPOCH")
	m.Inputs.KernelContract = "Linux; live operation requires a matching host IgH master module and PREEMPT_RT; neither is installed or qualified by this package"
	pin, err := command(component, "bash", "tools/build-igh-userlib.sh", "--print-pin")
	if err != nil {
		return err
	}
	parts := strings.SplitN(pin, " ", 3)
	if len(parts) != 3 || !regexp.MustCompile(`^[0-9a-f]{64}$`).MatchString(parts[1]) {
		return fmt.Errorf("missing IgH pin")
	}
	m.Inputs.IgHVersion, m.Inputs.IgHArchiveSHA256 = parts[0], parts[1]
	archive := parts[2]
	if _, err := os.Stat(archive); err == nil {
		sum, err := digest(archive)
		if err != nil {
			return err
		}
		if sum != m.Inputs.IgHArchiveSHA256 {
			return fmt.Errorf("IgH archive does not match pin")
		}
	}
	m.RobotDescriptions, err = robotDescriptions(root)
	if err != nil {
		return err
	}
	m.Files, err = inventory(root)
	if err != nil {
		return err
	}
	for i := range m.Files {
		a := &m.Files[i]
		a.InstallMode = installedMode(a.Path)
		if !strings.HasPrefix(a.Path, "bin/") && a.Path != "lib/libethercat.so.1" {
			continue
		}
		a.ELF, err = inspect(filepath.Join(root, a.Path))
		if err != nil {
			return err
		}
		if a.Path == "lib/libethercat.so.1" {
			if a.ELF.SONAME != "libethercat.so.1" || !includes(a.ELF.ECRTSymbols, "ecrt_version_magic") {
				return fmt.Errorf("unexpected IgH SONAME")
			}
			// unverified upstream reproducibility: its existing builder retains debug compilation paths.
			a.Reproducible = false
			a.Reason = "upstream IgH build retains absolute debug compilation paths; independent rebuild not qualified"
		}
		if a.Path == "bin/rosie-rt-core" && backend == "live" {
			// unverified relocation reproducibility: preserve the existing build-tree RUNPATH flag.
			a.Reproducible = false
			a.Reason = "live linker retains the absolute build-tree RUNPATH in addition to $ORIGIN/../lib"
		}
	}
	b, err := json.MarshalIndent(m, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(filepath.Join(root, "rt_package.json"), append(b, '\n'), 0644)
}

func readManifest(root string) (manifest, error) {
	var m manifest
	f, err := os.Open(filepath.Join(root, "rt_package.json"))
	if err != nil {
		return m, err
	}
	defer f.Close()
	d := json.NewDecoder(f)
	d.DisallowUnknownFields()
	if err = d.Decode(&m); err != nil {
		return m, err
	}
	var extra any
	if err = d.Decode(&extra); err != io.EOF {
		return m, fmt.Errorf("trailing manifest data")
	}
	if m.Schema != schema || m.FormatVersion != 1 || (m.Backend != "live" && m.Backend != "simulation") || !reflect.DeepEqual(m.Excluded, []string{"rt_package.json"}) || !regexp.MustCompile(`^[0-9a-f]{40}$`).MatchString(m.Inputs.GitSHA) {
		return m, fmt.Errorf("invalid manifest identity")
	}
	return m, nil
}

func integrity(root string, m manifest) error { return checkFiles(root, m, false) }

func checkFiles(root string, m manifest, restoring bool) error {
	actual, err := inventory(root)
	if err != nil {
		return err
	}
	if len(actual) != len(m.Files) {
		return fmt.Errorf("file count mismatch: expected %d, observed %d", len(m.Files), len(actual))
	}
	required := map[string]bool{}
	for _, p := range []string{"bin/rosie-rt-core", "bin/rt-control", "bin/rtctl", "bin/rt-package", "lib/libethercat.so.1", "licenses/igh/COPYING", "licenses/igh/COPYING.LESSER", "tools/verify-package.sh", "host/install.sh", "host/run-core", "host/generate-control-env.sh", "host/remote-pki.sh", "host/ethercat-foundation.sh", "host/rosie-rt-core.service", "host/rosie-rt-control.service"} {
		required[p] = false
	}
	profiles, configs := 0, 0
	for i, a := range actual {
		expected := m.Files[i]
		mode := expected.InstallMode
		if windowsModes(root) {
			mode = expected.Mode
		}
		if expected.InstallMode != installedMode(expected.Path) {
			return fmt.Errorf("unsafe installation mode: %s", expected.Path)
		}
		if a.Path != expected.Path || (!restoring && a.Mode != mode) || a.SHA256 != expected.SHA256 {
			return fmt.Errorf("file integrity mismatch: %s", a.Path)
		}
		if !expected.Reproducible && expected.Reason == "" {
			return fmt.Errorf("unexplained reproducibility exception: %s", a.Path)
		}
		if _, ok := required[a.Path]; ok {
			required[a.Path] = true
		}
		if strings.HasPrefix(a.Path, "config/drives/") {
			profiles++
		}
		if strings.HasPrefix(a.Path, "config/") {
			configs++
		}
		binary := strings.HasPrefix(a.Path, "bin/") || a.Path == "lib/libethercat.so.1"
		if binary != (expected.ELF != nil) {
			return fmt.Errorf("missing/unexpected ELF identity: %s", a.Path)
		}
		if binary {
			id, err := inspect(filepath.Join(root, a.Path))
			if err != nil {
				return err
			}
			if !reflect.DeepEqual(id, expected.ELF) {
				return fmt.Errorf("ELF identity mismatch: %s", a.Path)
			}
		}
	}
	for p, found := range required {
		if !found {
			return fmt.Errorf("required artifact missing: %s", p)
		}
	}
	if profiles == 0 || configs == 0 {
		return fmt.Errorf("config/drives/configuration identities missing")
	}
	if err := checkRobotDescriptions(root, m.RobotDescriptions); err != nil {
		return err
	}
	var core, library *elfIdentity
	for _, a := range m.Files {
		if a.Path == "bin/rosie-rt-core" {
			core = a.ELF
		}
		if a.Path == "lib/libethercat.so.1" {
			library = a.ELF
		}
	}
	if library.SONAME != "libethercat.so.1" || !includes(library.ECRTSymbols, "ecrt_version_magic") {
		return fmt.Errorf("IgH userspace ABI identity missing")
	}
	if includes(core.Needed, "libethercat.so.1") != (m.Backend == "live") {
		return fmt.Errorf("backend and daemon dependencies differ")
	}
	for _, symbol := range core.ECRTSymbols {
		if !includes(library.ECRTSymbols, symbol) {
			return fmt.Errorf("IgH symbol missing: %s", symbol)
		}
	}
	return nil
}

// restoreModes validates the entire copied payload before normalizing permissions on a POSIX installation.
func restoreModes(root string) error {
	m, err := readManifest(root)
	if err != nil {
		return err
	}
	if err = checkFiles(root, m, true); err != nil {
		return err
	}
	for _, a := range m.Files {
		mode, err := strconv.ParseUint(a.InstallMode, 8, 32)
		if err != nil {
			return err
		}
		if err = os.Chmod(filepath.Join(root, a.Path), os.FileMode(mode)); err != nil {
			return err
		}
	}
	if err = os.Chmod(filepath.Join(root, "rt_package.json"), 0644); err != nil {
		return err
	}
	return filepath.WalkDir(root, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if d.IsDir() {
			return os.Chmod(path, 0755)
		}
		return nil
	})
}

func includes(values []string, value string) bool {
	for _, v := range values {
		if v == value {
			return true
		}
	}
	return false
}

// prerequisites checks host ELF providers read-only; RT and module observations are advisory.
func prerequisites(root string, m manifest) error {
	arch, err := command("", "uname", "-m")
	if err != nil {
		return err
	}
	if arch != m.Inputs.Architecture {
		return fmt.Errorf("architecture required=%s host=%s", m.Inputs.Architecture, arch)
	}
	host, err := inspect("/proc/self/exe")
	if err != nil {
		return err
	}
	cache, err := command("", "/sbin/ldconfig", "-p")
	if err != nil {
		return err
	}
	libraries := map[string][]string{"libethercat.so.1": {filepath.Join(root, "lib/libethercat.so.1")}}
	for _, line := range strings.Split(cache, "\n") {
		fields := strings.Fields(line)
		if len(fields) >= 4 && fields[len(fields)-2] == "=>" && fields[0] != "libethercat.so.1" {
			libraries[fields[0]] = append(libraries[fields[0]], fields[len(fields)-1])
		}
	}
	for _, a := range m.Files {
		if a.ELF == nil {
			continue
		}
		id := a.ELF
		if id.Machine != host.Machine {
			return fmt.Errorf("%s ELF architecture differs from host", a.Path)
		}
		if id.Interpreter != "" {
			loader, err := inspect(id.Interpreter)
			if err != nil || loader.Machine != id.Machine {
				return fmt.Errorf("missing matching ELF interpreter: %s", id.Interpreter)
			}
		}
		for _, needed := range id.Needed {
			matched := false
			for _, path := range libraries[needed] {
				provider, err := inspect(path)
				if err != nil || provider.Machine != id.Machine {
					continue
				}
				matched = true
				for _, version := range id.Versions[needed] {
					if !includes(provider.Defined, version) {
						matched = false
					}
				}
				if matched {
					break
				}
			}
			if !matched {
				return fmt.Errorf("%s requires %s symbol versions %v", a.Path, needed, id.Versions[needed])
			}
		}
	}
	fmt.Println("prerequisites status=passed architecture,ELF-interpreters,NEEDED,GLIBC-tags,IgH-symbol-versions")
	return nil
}

func verify(root string) int {
	m, err := readManifest(root)
	if err == nil {
		err = integrity(root, m)
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "integrity:", err)
		return 3
	}
	sum, err := digest(filepath.Join(root, "rt_package.json"))
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		return 3
	}
	b, _ := json.MarshalIndent(struct {
		ManifestSHA256 string `json:"manifest_sha256"`
		Backend        string `json:"backend"`
		Inputs         inputs `json:"build_inputs"`
	}{sum, m.Backend, m.Inputs}, "", "  ")
	fmt.Println(string(b))
	for _, a := range m.Files {
		if !a.Reproducible {
			fmt.Printf("reproducible=false path=%s reason=%s\n", a.Path, a.Reason)
		}
	}
	_, moduleErr := command("", "modinfo", "ec_master")
	fmt.Printf("igh_master_module_present=%t (advisory)\n", moduleErr == nil)
	rt, rtErr := os.ReadFile("/sys/kernel/realtime")
	fmt.Printf("preempt_rt=%t (advisory)\n", rtErr == nil && strings.TrimSpace(string(rt)) == "1")
	if err := prerequisites(root, m); err != nil {
		fmt.Fprintln(os.Stderr, "prerequisite:", err)
		return 2
	}
	fmt.Println("package integrity=passed")
	return 0
}

func main() {
	if len(os.Args) == 5 && os.Args[1] == "component" {
		if err := emitComponent(os.Args[2], os.Args[3], os.Args[4]); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if len(os.Args) == 5 && os.Args[1] == "manifest" {
		if err := generate(os.Args[2], os.Args[3], os.Args[4]); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if len(os.Args) == 3 {
		switch os.Args[1] {
		case "verify":
			os.Exit(verify(os.Args[2]))
		case "install-modes":
			if err := restoreModes(os.Args[2]); err != nil {
				fmt.Fprintln(os.Stderr, err)
				os.Exit(3)
			}
			return
		case "git-sha", "backend":
			m, err := readManifest(os.Args[2])
			if err != nil {
				fmt.Fprintln(os.Stderr, err)
				os.Exit(3)
			}
			if os.Args[1] == "git-sha" {
				fmt.Println(m.Inputs.GitSHA)
			} else {
				fmt.Println(m.Backend)
			}
			return
		}
	}
	fmt.Fprintln(os.Stderr, "usage: rt-package verify|git-sha|backend PACKAGE | manifest PACKAGE COMPONENT live|simulation | component PACKAGE OUTPUT BUILD_LOG")
	os.Exit(2)
}
