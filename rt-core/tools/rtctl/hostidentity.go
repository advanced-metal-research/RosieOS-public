package rtctl

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"reflect"
	"regexp"
	"sort"
	"strings"
)

// Unverified deployment contract: these three services must qualify together.
var hostIdentityServices = []string{"rosie-rt-core", "rosie-rt-control", "rosie-rt-natspublisher"}

type hostReleaseFile struct {
	Path   string `json:"path"`
	SHA256 string `json:"sha256"`
}

// hostReleaseService binds independently supplied release bytes to the installed
// unit chain, expanded argv and nonsecret environment; paths alone prove nothing.
type hostReleaseService struct {
	TemplateSHA256   string            `json:"template_sha256"`
	UnitFiles        []hostReleaseFile `json:"unit_files"`
	Argv             []string          `json:"argv"`
	ExecutableSHA256 string            `json:"executable_sha256"`
	EnvironmentFiles []string          `json:"environment_files"`
	Environment      map[string]string `json:"environment"`
}

type hostRelease struct {
	Schema   string                        `json:"schema"`
	Services map[string]hostReleaseService `json:"services"`
}

type hostIdentityCheck struct {
	Status   string `json:"status"`
	Observed string `json:"observed"`
	Expected string `json:"expected"`
}

type hostServiceIdentity struct {
	Service        string                       `json:"service"`
	QualifiedMatch bool                         `json:"qualified_match"`
	Layers         map[string]hostIdentityCheck `json:"layers"`
	pid            int
	start          string
	commandSHA     string
	environmentSHA string
	processInfo    os.FileInfo
	imageInfo      os.FileInfo
}

type hostIdentityReport struct {
	ExpectedRelease  hostIdentityCheck     `json:"expected_release"`
	Services         []hostServiceIdentity `json:"services"`
	MasterOwner      hostIdentityCheck     `json:"ethercat_master_owner"`
	Status           string                `json:"status"`
	Conflicts        int                   `json:"conflicts"`
	Unknowns         int                   `json:"unknowns"`
	QualifiedMatches int                   `json:"qualified_matches"`
}

func identityComparison(observed, expected string) hostIdentityCheck {
	status := "conflict"
	if observed == expected {
		status = "match"
	}
	return hostIdentityCheck{status, observed, expected}
}

func identityUnknown(reason, expected string) hostIdentityCheck {
	return hostIdentityCheck{"unknown", reason, expected}
}

func identityJSON(value any) string {
	raw, _ := json.Marshal(value)
	return string(raw)
}

func identitySHA(raw []byte) string { return fmt.Sprintf("%x", sha256.Sum256(raw)) }

func identityArgv(argv []string) string {
	// Hash arguments, including unexpected arguments, so a mistaken secret binding
	// cannot be copied into a hostcheck report. Exact token boundaries are retained.
	return fmt.Sprintf("argc=%d argv_sha256=%s", len(argv), identitySHA([]byte(identityJSON(argv))))
}

// identityNoDuplicateKeys rejects ambiguous JSON before decoding a release or
// manager snapshot; encoding/json's last-key-wins rule cannot qualify identity.
func identityNoDuplicateKeys(raw []byte) error {
	d := json.NewDecoder(bytes.NewReader(raw))
	d.UseNumber()
	var value func() error
	value = func() error {
		token, err := d.Token()
		if err != nil {
			return err
		}
		if delim, ok := token.(json.Delim); ok {
			keys := map[string]bool{}
			for d.More() {
				if delim == '{' {
					key, err := d.Token()
					if err != nil || keys[fmt.Sprint(key)] {
						return fmt.Errorf("duplicate or invalid JSON key")
					}
					keys[fmt.Sprint(key)] = true
				}
				if err := value(); err != nil {
					return err
				}
			}
			_, err = d.Token()
		}
		return err
	}
	if err := value(); err != nil {
		return err
	}
	if _, err := d.Token(); err != io.EOF {
		return fmt.Errorf("trailing JSON data")
	}
	return nil
}

func identityAbsolute(path string) bool {
	return strings.HasPrefix(path, "/") && filepath.Clean(path) == path && !strings.ContainsAny(path, "\x00\n\r")
}

// identityEnvironmentKeys binds service expansion and runtime environment reads;
// source-audit tests prevent new consumers from silently escaping qualification.
func identityEnvironmentKeys(service string) []string {
	if service == "rosie-rt-natspublisher" {
		// Unverified conservative binding: publisher imports the control package,
		// so cover its anchor consumer even if a release does not call it.
		return []string{"ROSIE_RT_NATS_URL", "ROSIE_RT_NATS_HOST", "ROSIE_RT_NATS_CADENCE", "ROSIE_RT_NATS_TELEMETRY_CADENCE", "ROSIE_RT_NATS_TELEMETRY_MAX_BYTES", "ROSIE_RT_ANCHOR_DIR"}
	}
	// Unverified deployment contract: core and control share control.env, so both
	// manifests bind its runtime controls even when only one service consumes them.
	return []string{"ROSIE_RT_CORE_ARGS", "ROSIE_RT_CONFIGURATION_SHA256", "ROSIE_RT_COMPILED_CONFIG", "ROSIE_RT_PAIR_ID", "ROSIE_RT_PAIR_REVISION", "ROSIE_RT_REMOTE_ARGS",
		"ROSIE_RT_ANCHOR_DIR", "GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED", "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED"}
}

func readHostRelease(path string) (hostRelease, string, error) {
	var release hostRelease
	raw, err := os.ReadFile(path)
	if err != nil {
		return release, "", fmt.Errorf("expected release is unreadable")
	}
	if err := identityNoDuplicateKeys(raw); err != nil {
		return release, "", fmt.Errorf("ambiguous expected release JSON")
	}
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	if err := d.Decode(&release); err != nil || release.Schema != "rosie-rt-core.host-identity.v1" || len(release.Services) != 3 {
		return release, "", fmt.Errorf("expected release must use rosie-rt-core.host-identity.v1 and name exactly the three services")
	}
	digest := regexp.MustCompile(`^[0-9a-f]{64}$`)
	for _, name := range hostIdentityServices {
		s, ok := release.Services[name]
		if !ok || !digest.MatchString(s.TemplateSHA256) || !digest.MatchString(s.ExecutableSHA256) || len(s.UnitFiles) == 0 || len(s.Argv) == 0 || !identityAbsolute(s.Argv[0]) || s.Environment == nil || s.EnvironmentFiles == nil {
			return release, "", fmt.Errorf("incomplete expected identity for %s", name)
		}
		seen := map[string]bool{}
		for _, file := range s.UnitFiles {
			if !identityAbsolute(file.Path) || !digest.MatchString(file.SHA256) || seen[file.Path] {
				return release, "", fmt.Errorf("invalid unit file identity for %s", name)
			}
			seen[file.Path] = true
		}
		seen = map[string]bool{}
		for _, path := range s.EnvironmentFiles {
			if !identityAbsolute(path) || seen[path] {
				return release, "", fmt.Errorf("invalid environment file identity for %s", name)
			}
			seen[path] = true
		}
		allowed := map[string]bool{}
		for _, key := range identityEnvironmentKeys(name) {
			allowed[key] = true
		}
		for key, value := range s.Environment {
			if !allowed[key] || strings.ContainsAny(value, "\x00\r\n") {
				return release, "", fmt.Errorf("unsupported environment binding for %s", name)
			}
		}
		for _, arg := range s.Argv {
			if strings.ContainsRune(arg, 0) {
				return release, "", fmt.Errorf("invalid argv for %s", name)
			}
		}
	}
	return release, identitySHA(raw), nil
}

// identityPath confines fixture symlinks to the fake root; absolute live procfs
// executable links are followed only during a real host inspection.
func (h *hostInspector) identityPath(path string) (string, error) {
	if !identityAbsolute(path) {
		return "", fmt.Errorf("invalid absolute path")
	}
	full := h.path(path)
	if h.root != "/" {
		resolved, err := filepath.EvalSymlinks(full)
		if err != nil {
			return "", err
		}
		rel, err := filepath.Rel(h.root, resolved)
		if err != nil || rel == ".." || strings.HasPrefix(rel, "../") || filepath.IsAbs(rel) {
			return "", fmt.Errorf("fixture link escapes root")
		}
		full = resolved
	}
	return full, nil
}

func (h *hostInspector) identityRead(path string) ([]byte, error) {
	full, err := h.identityPath(path)
	if err != nil {
		return nil, err
	}
	return os.ReadFile(full)
}

func (h *hostInspector) identityDigest(path string) (string, os.FileInfo, error) {
	full, err := h.identityPath(path)
	if err != nil {
		return "", nil, err
	}
	f, err := os.Open(full)
	if err != nil {
		return "", nil, err
	}
	defer f.Close()
	before, err := f.Stat()
	if err != nil || !before.Mode().IsRegular() {
		return "", nil, fmt.Errorf("not a readable regular image")
	}
	hash := sha256.New()
	if _, err := io.Copy(hash, f); err != nil {
		return "", nil, err
	}
	after, err := f.Stat()
	current, pathErr := os.Stat(full)
	if err != nil || pathErr != nil || !os.SameFile(before, current) || before.Size() != after.Size() || !before.ModTime().Equal(after.ModTime()) {
		return "", nil, fmt.Errorf("image changed during inspection")
	}
	return fmt.Sprintf("%x", hash.Sum(nil)), after, nil
}

func (h *hostInspector) identityFile(path, expected string) hostIdentityCheck {
	digest, _, err := h.identityDigest(path)
	if err != nil {
		return identityUnknown("unreadable or unstable file: "+path, expected)
	}
	return identityComparison(digest, expected)
}

func (h *hostInspector) releaseIdentity(manifest, component string) {
	r := &hostIdentityReport{Services: []hostServiceIdentity{}}
	h.report.Identity = r
	release, sum, err := readHostRelease(manifest)
	if err != nil {
		r.ExpectedRelease = identityUnknown(err.Error(), "explicit complete release manifest")
	} else {
		r.ExpectedRelease = identityComparison(sum, sum)
	}
	for _, name := range hostIdentityServices {
		s := hostServiceIdentity{Service: name, Layers: map[string]hostIdentityCheck{}}
		if err != nil {
			s.Layers["expected_release"] = identityUnknown("release identity unavailable", "complete expected service identity")
		} else {
			h.inspectService(&s, release.Services[name], component)
		}
		r.Services = append(r.Services, s)
	}
	r.MasterOwner = h.identityMasterOwner(r.Services[0])
	count := func(c hostIdentityCheck) {
		if c.Status == "conflict" {
			r.Conflicts++
		} else if c.Status != "match" {
			r.Unknowns++
		}
	}
	count(r.ExpectedRelease)
	count(r.MasterOwner)
	for _, s := range r.Services {
		for _, c := range s.Layers {
			count(c)
		}
	}
	r.Status = "match"
	if r.Conflicts != 0 {
		r.Status = "conflict"
	} else if r.Unknowns != 0 {
		r.Status = "unknown"
	}
	if r.Status == "match" {
		r.QualifiedMatches = 3
		for i := range r.Services {
			r.Services[i].QualifiedMatch = true
		}
	} else if r.Unknowns != 0 {
		h.report.ExitCode = 3
	} else if h.report.ExitCode == 0 {
		h.report.ExitCode = 2
	}
}

func (r *hostIdentityReport) print(out io.Writer) {
	fmt.Fprintf(out, "%s release_identity: conflicts=%d unknowns=%d qualified_matches=%d\n", r.Status, r.Conflicts, r.Unknowns, r.QualifiedMatches)
	print := func(name string, c hostIdentityCheck) {
		fmt.Fprintf(out, "%s %s: observed=%q expected=%q\n", c.Status, name, c.Observed, c.Expected)
	}
	print("expected_release", r.ExpectedRelease)
	for _, s := range r.Services {
		keys := []string{}
		for key := range s.Layers {
			keys = append(keys, key)
		}
		sort.Strings(keys)
		for _, key := range keys {
			print(s.Service+"."+key, s.Layers[key])
		}
	}
	print("ethercat_master_owner", r.MasterOwner)
}

func identityAllMatch(layers map[string]hostIdentityCheck) bool {
	for _, c := range layers {
		if c.Status != "match" {
			return false
		}
	}
	return len(layers) != 0
}

func identityEnvironment(service string, env map[string]string) map[string]string {
	selected := map[string]string{}
	for _, key := range identityEnvironmentKeys(service) {
		if value, ok := env[key]; ok {
			selected[key] = value
		}
	}
	return selected
}

func identityEnvironmentComparison(observed, expected map[string]string) hostIdentityCheck {
	changed := []string{}
	for key, value := range expected {
		if got, ok := observed[key]; !ok || got != value {
			changed = append(changed, key)
		}
	}
	for key := range observed {
		if _, ok := expected[key]; !ok {
			changed = append(changed, key)
		}
	}
	sort.Strings(changed)
	check := identityComparison(identitySHA([]byte(identityJSON(observed))), identitySHA([]byte(identityJSON(expected))))
	if !reflect.DeepEqual(observed, expected) {
		check.Observed += "; changed_bindings=" + strings.Join(changed, ",")
	}
	return check
}
