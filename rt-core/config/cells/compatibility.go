// Package cells validates repository cell bindings against the current compiler,
// preventing stale configuration pins or incompatible dense execution mappings.
package cells

import (
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"

	"rosieos/rt-core/tools/rtctl"
)

type Binding struct {
	MachineConfig       string `json:"machine_config"`
	ConfigurationSHA256 string `json:"configuration_sha256"`
	PairID              string `json:"pair_id"`
	PairRevision        uint64 `json:"pair_revision"`
	RemoteListen        string `json:"remote_listen"`
}

type Result struct {
	Manifest  string
	Binding   Binding
	SHA256    string
	AxisCount int
	Warnings  []string
}

// ErrCompilerWarnings marks an incompatible compile even when its digest matches;
// callers may report the returned results but must not label them warning-free.
var ErrCompilerWarnings = errors.New("compiler warnings")

// CheckRepository checks every native binding, including newly added manifests;
// missing directories and malformed JSON fail instead of producing an empty pass.
// It reads the cells and the deploy-only aliases under cells/aliases (bench
// aliases OLP must not offer as cells), so an alias's pin is checked too.
func CheckRepository(root string) ([]Result, error) {
	dir := filepath.Join(root, "rt-core/config/cells")
	files, err := filepath.Glob(filepath.Join(dir, "*.json"))
	if err != nil || len(files) == 0 {
		return nil, fmt.Errorf("cell configs missing: %v", err)
	}
	aliases, err := filepath.Glob(filepath.Join(dir, "aliases", "*.json"))
	if err != nil {
		return nil, fmt.Errorf("cell aliases unreadable: %v", err)
	}
	files = append(files, aliases...)
	var results []Result
	var findings []error
	for _, path := range files {
		raw, err := os.ReadFile(path)
		if err != nil {
			return nil, err
		}
		bindings, err := CheckManifest(root, raw)
		if err != nil && !errors.Is(err, ErrCompilerWarnings) {
			return nil, fmt.Errorf("%s: %w", manifestName(dir, path), err)
		}
		if err != nil {
			findings = append(findings, fmt.Errorf("%s: %w", manifestName(dir, path), err))
		}
		for _, result := range bindings {
			result.Manifest = manifestName(dir, path)
			results = append(results, result)
		}
	}
	return results, errors.Join(findings...)
}

// manifestName is path relative to the cells directory: "<cell>.json", or
// "aliases/<alias>.json" for an alias.
func manifestName(dir, path string) string {
	if rel, err := filepath.Rel(dir, path); err == nil {
		return filepath.ToSlash(rel)
	}
	return filepath.Base(path)
}

// CheckManifest compiles each rt_core block and checks its endpoint, identity,
// pin and axis mapping. An absent block is a v4-only/fleet document, not a binding.
func CheckManifest(root string, raw []byte) ([]Result, error) {
	var manifest struct {
		SchemaVersion string `json:"schema_version"`
		Nodes         []struct {
			Core json.RawMessage `json:"rt_core"`
		} `json:"nodes"`
	}
	if err := json.Unmarshal(raw, &manifest); err != nil {
		return nil, err
	}
	var results []Result
	var findings []error
	for i, node := range manifest.Nodes {
		if len(node.Core) == 0 {
			continue
		}
		if manifest.SchemaVersion != "robot_v4_deployment_manifest_v1" {
			return nil, fmt.Errorf("node %d: unsupported schema_version %q for rt_core binding", i, manifest.SchemaVersion)
		}
		var binding Binding
		if err := json.Unmarshal(node.Core, &binding); err != nil {
			return nil, err
		}
		var fields map[string]json.RawMessage
		if err := json.Unmarshal(node.Core, &fields); err != nil {
			return nil, err
		}
		if _, present := fields["configuration_sha256"]; present && binding.ConfigurationSHA256 == "" {
			return nil, fmt.Errorf("node %d: present configuration_sha256 must not be empty", i)
		}
		result, err := checkBinding(root, binding)
		if err != nil && !errors.Is(err, ErrCompilerWarnings) {
			return nil, fmt.Errorf("node %d: %w", i, err)
		}
		if err != nil {
			findings = append(findings, fmt.Errorf("node %d: %w", i, err))
		}
		results = append(results, result)
	}
	return results, errors.Join(findings...)
}

// checkBinding applies unverified manifest syntax policy: positive pair revision,
// unique axis tokens, ports 1..65535 and DNS names/labels of at most 253/63 bytes.
// Compiler identity and axis-count checks prevent a syntactically valid misbinding.
func checkBinding(root string, binding Binding) (Result, error) {
	result := Result{Binding: binding}
	if !regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]*$`).MatchString(binding.PairID) || binding.PairRevision == 0 {
		return result, fmt.Errorf("pair_id and positive pair_revision required")
	}
	host, port, err := net.SplitHostPort(binding.RemoteListen)
	portNumber, portErr := strconv.Atoi(port)
	if err != nil || host == "" || strings.ContainsAny(host, " /\\\t\r\n?#@") || portErr != nil || portNumber < 1 || portNumber > 65535 {
		return result, fmt.Errorf("remote_listen must be host:port with port 1..65535")
	}
	if net.ParseIP(strings.Split(host, "%")[0]) == nil {
		if len(host) > 253 {
			return result, fmt.Errorf("remote_listen hostname is too long")
		}
		for _, label := range strings.Split(strings.TrimSuffix(host, "."), ".") {
			if !regexp.MustCompile(`^[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?$`).MatchString(label) {
				return result, fmt.Errorf("remote_listen has an invalid hostname")
			}
		}
	}
	if !filepath.IsLocal(binding.MachineConfig) || strings.Contains(binding.MachineConfig, "\\") {
		return result, fmt.Errorf("machine_config must be repository relative")
	}
	path, err := filepath.EvalSymlinks(filepath.Join(root, binding.MachineConfig))
	if err != nil {
		return result, err
	}
	absRoot, err := filepath.EvalSymlinks(root)
	if err != nil {
		return result, err
	}
	relative, err := filepath.Rel(absRoot, path)
	if err != nil || !filepath.IsLocal(relative) {
		return result, fmt.Errorf("machine_config escapes repository")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		return result, err
	}
	compiled, err := rtctl.Compile(raw, filepath.Join(root, "rt-core/config/drives"))
	if err != nil {
		return result, err
	}
	result.SHA256 = compiled.SHA256
	result.Warnings = compiled.Warnings
	for i, arg := range compiled.Argv {
		if arg == "--num-axes" && i+1 < len(compiled.Argv) {
			result.AxisCount, err = strconv.Atoi(compiled.Argv[i+1])
		}
	}
	// The axis count is the compiled configuration's alone; the adapter derives
	// its dense axis map from the same description, so nothing here restates it.
	if err != nil || result.AxisCount == 0 {
		return result, fmt.Errorf("compiled configuration names no axes")
	}
	if binding.ConfigurationSHA256 != "" && binding.ConfigurationSHA256 != compiled.SHA256 {
		return result, fmt.Errorf("configuration_sha256 %s differs from compiled %s", binding.ConfigurationSHA256, compiled.SHA256)
	}
	if len(compiled.Warnings) != 0 {
		return result, fmt.Errorf("%w: %v", ErrCompilerWarnings, compiled.Warnings)
	}
	return result, nil
}
