//go:build linux

package targets

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"io/fs"
	"net/url"
	"os"
	"path/filepath"
	"regexp"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"strings"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/rt-core/tools/rtctl"
)

// Unverified catalogue version; selection still requires authenticated Describe.
const CellCatalogueSchema = "offline-programming.cell-catalogue.v1"

type CellExecution struct {
	Local bool `json:"local,omitempty"`
	denseexec.RTCoreConfig
}

type Cell struct {
	CellID            string        `json:"cell_id"`
	Label             string        `json:"label"`
	Role              string        `json:"role"`
	Models            []string      `json:"models"`
	Execution         CellExecution `json:"execution"`
	RequestedMode     string        `json:"requested_mode"`
	unavailableReason string
}

type CellCatalogue struct {
	Schema string `json:"schema"`
	Cells  []Cell `json:"cells"`
}

type CellConfiguration struct {
	ModelID, SHA256, Address, PairID string
	Revision                         uint64
	// JogJerk is the compiled machine jog_jerk (rad/s^3, zero when absent),
	// bound to SHA256 like the model.
	JogJerk float64
	// UnattendedLoopAllowed is the cell manifest's
	// cell_deploy.olp.unattended_loop_allowed (owner decision; false when
	// absent; two Rosie 1430 cells set it, 2026-10-11 23:32 PDT). Only OLP reads
	// it, so it lives with the cell's other OLP settings and an OLP-tier
	// release carries it: in the machine configuration rt-core's compiler put
	// it in the deployment identity, which the runtime ignores and only a
	// full redeploy may change.
	UnattendedLoopAllowed bool
}
type CellResolver func(string) (CellConfiguration, error)

// ReadCellCatalogue admits manifest-bound execution and retains unconfigured
// cells as unavailable. Metadata grants no authority; credentials stay private.
func ReadCellCatalogue(path, root string) (CellCatalogue, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return CellCatalogue{}, fmt.Errorf("cell_catalogue_unreadable")
	}
	return DecodeCellCatalogue(raw, filepath.Dir(path), func(id string) (CellConfiguration, error) {
		return ResolveCellConfiguration(root, id)
	})
}

// DecodeCellCatalogue validates execution identity before selection. Missing
// native configuration disables that cell without aborting usable catalogue entries.
func DecodeCellCatalogue(raw []byte, base string, resolve CellResolver) (CellCatalogue, error) {
	var catalogue CellCatalogue
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	if d.Decode(&catalogue) != nil || d.Decode(new(any)) != io.EOF || catalogue.Schema != CellCatalogueSchema || len(catalogue.Cells) == 0 {
		return catalogue, fmt.Errorf("cell_catalogue_invalid")
	}
	seen := map[string]bool{}
	for i := range catalogue.Cells {
		cell := &catalogue.Cells[i]
		if seen[cell.CellID] {
			return catalogue, fmt.Errorf("cell_id_duplicate")
		}
		seen[cell.CellID] = true
		if cell.Label == "" || cell.Role == "" {
			return catalogue, fmt.Errorf("cell_label_invalid")
		}
		if cell.RequestedMode != "simulation" && cell.RequestedMode != "real" {
			return catalogue, fmt.Errorf("cell_mode_invalid")
		}
		for _, model := range cell.Models {
			if robotmodel.Class(model) == "" {
				return catalogue, fmt.Errorf("cell_model_unknown")
			}
		}
		if len(cell.Models) != 1 {
			return catalogue, fmt.Errorf("cell_model_binding_ambiguous")
		}
		e := &cell.Execution
		if e.Local {
			if e.RTCoreConfig != (denseexec.RTCoreConfig{}) {
				return catalogue, fmt.Errorf("cell_local_binding_conflict")
			}
			if cell.CellID != "local-simulation" || cell.RequestedMode != "simulation" || robotmodel.Class(cell.Models[0]) != "arm" {
				return catalogue, fmt.Errorf("cell_local_identity_invalid")
			}
			continue
		}
		// The resolver reads the id's manifest; only the id's shape is checked here.
		if !cellIDPattern.MatchString(cell.CellID) {
			return catalogue, fmt.Errorf("cell_id_unknown")
		}
		if resolve == nil {
			return catalogue, refusal("cell_configuration_missing")
		}
		binding, err := resolve(cell.CellID)
		if err != nil {
			if err.Error() == "cell_configuration_missing" {
				cell.unavailableReason = "cell_configuration_missing"
				continue
			}
			return catalogue, err
		}
		if binding.SHA256 == "" {
			cell.unavailableReason = "cell_configuration_missing"
			continue
		}
		if e.CAFile == "" || e.CertificateFile == "" || e.KeyFile == "" {
			return catalogue, fmt.Errorf("cell_credentials_missing")
		}
		u, err := url.Parse(e.Address)
		if err != nil || u.Scheme != "https" || u.Hostname() == "" || u.User != nil || (u.Path != "" && u.Path != "/") || u.RawQuery != "" || u.Fragment != "" || e.Socket != "" {
			return catalogue, fmt.Errorf("cell_address_invalid")
		}
		if e.Binding.PairID == "" || strings.ContainsAny(e.Binding.PairID, " /\t\r\n") || e.Binding.Revision == 0 || e.AxisMask == 0 || e.AxisMask > 511 {
			return catalogue, fmt.Errorf("cell_execution_invalid")
		}
		if e.ExpectedBackend != "simulation" && e.ExpectedBackend != "ethercat" {
			return catalogue, fmt.Errorf("cell_backend_invalid")
		}
		for _, path := range []*string{&e.CAFile, &e.CertificateFile, &e.KeyFile} {
			if !filepath.IsAbs(*path) {
				*path = filepath.Join(base, *path)
			}
			if st, err := os.Stat(*path); err != nil || !st.Mode().IsRegular() {
				return catalogue, fmt.Errorf("cell_credentials_missing")
			}
		}
		if binding.ModelID != cell.Models[0] {
			return catalogue, fmt.Errorf("cell_model_mismatch")
		}
		if binding.Address == "" || binding.PairID == "" || binding.Revision == 0 {
			return catalogue, refusal("cell_manifest_binding_missing")
		}
		if strings.TrimSuffix(e.Address, "/") != binding.Address {
			return catalogue, refusal("cell_endpoint_mismatch")
		}
		if e.Binding.PairID != binding.PairID {
			return catalogue, refusal("cell_pair_mismatch")
		}
		if e.Binding.Revision != binding.Revision {
			return catalogue, refusal("cell_pair_revision_mismatch")
		}
		if e.Binding.ConfigurationSHA256 != "" && e.Binding.ConfigurationSHA256 != binding.SHA256 {
			return catalogue, refusal("cell_configuration_mismatch")
		}
		e.Binding.ConfigurationSHA256 = binding.SHA256
		e.MoveJerk = binding.JogJerk
		e.UnattendedLoopAllowed = binding.UnattendedLoopAllowed
	}
	return catalogue, nil
}

// ResolveCellConfiguration compiles the manifest's configured robot definition
// without hardware contact. A role label or URDF renderer cannot supply a model.
func ResolveCellConfiguration(root, id string) (CellConfiguration, error) {
	_, raw, err := cellManifestName(root, id)
	if err != nil {
		return CellConfiguration{}, err
	}
	var manifest robotV4Manifest
	if json.Unmarshal(raw, &manifest) != nil {
		return CellConfiguration{}, fmt.Errorf("cell_manifest_invalid")
	}
	allowed, err := unattendedLoopAllowed(raw)
	if err != nil {
		return CellConfiguration{}, err
	}
	var config, declaredSHA256, driveProfiles string
	binding := CellConfiguration{UnattendedLoopAllowed: allowed}
	found := false
	for _, node := range manifest.Nodes {
		if node.RTCore != nil {
			if found {
				return CellConfiguration{}, fmt.Errorf("cell_configuration_ambiguous")
			}
			found = true
			config = node.RTCore.MachineConfig
			driveProfiles = node.RTCore.DriveProfiles
			binding.Address = "https://" + node.RTCore.RemoteListen
			binding.PairID, binding.Revision = node.RTCore.PairID, node.RTCore.PairRevision
			declaredSHA256 = node.RTCore.ConfigurationSHA256
		}
	}
	if !filepath.IsLocal(config) {
		return CellConfiguration{}, fmt.Errorf("cell_configuration_missing")
	}
	u, err := url.Parse(binding.Address)
	if err != nil || u.Hostname() == "" || u.Port() == "" || u.User != nil || u.Path != "" || u.RawQuery != "" || u.Fragment != "" || binding.PairID == "" || strings.ContainsAny(binding.PairID, " /\t\r\n") || binding.Revision == 0 {
		return CellConfiguration{}, refusal("cell_manifest_binding_missing")
	}
	raw, err = os.ReadFile(filepath.Join(root, config))
	if err != nil {
		return CellConfiguration{}, fmt.Errorf("cell_configuration_missing")
	}
	profilesDir := filepath.Join(root, "rt-core/config/drives")
	if driveProfiles != "" {
		profilesDir, err = boundDriveProfiles(root, driveProfiles)
		if err != nil {
			return CellConfiguration{}, err
		}
		if declaredSHA256 == "" {
			return CellConfiguration{}, refusal("cell_configuration_mismatch")
		}
	}
	compiled, err := rtctl.Compile(raw, profilesDir)
	if err != nil || compiled.Robot == nil {
		return CellConfiguration{}, fmt.Errorf("cell_robot_definition_unverified")
	}
	if declaredSHA256 != "" && declaredSHA256 != compiled.SHA256 {
		return CellConfiguration{}, refusal("cell_configuration_mismatch")
	}
	binding.ModelID, binding.SHA256, binding.JogJerk = compiled.Robot.ModelID, compiled.SHA256, compiled.JogJerk
	return binding, nil
}

// unattendedLoopAllowed reads the cell manifest's
// cell_deploy.olp.unattended_loop_allowed: false when absent, refused when
// not a boolean (a quoted "true" must not pass as true, nor as false).
func unattendedLoopAllowed(manifest []byte) (bool, error) {
	var top struct {
		CellDeploy struct {
			OLP struct {
				Allowed *json.RawMessage `json:"unattended_loop_allowed"`
			} `json:"olp"`
		} `json:"cell_deploy"`
	}
	if err := json.Unmarshal(manifest, &top); err != nil || top.CellDeploy.OLP.Allowed == nil {
		return false, nil
	}
	var allowed bool
	if err := json.Unmarshal(*top.CellDeploy.OLP.Allowed, &allowed); err != nil {
		return false, refusal("cell_manifest_invalid: cell_deploy.olp.unattended_loop_allowed must be true or false")
	}
	return allowed, nil
}

// cellIDPattern is the shape of a cell id: its manifest's file name under
// rt-core/config/cells, without ".json". Catalogue files and network discovery
// supply ids, so only this shape may reach that path (no "/", "..", or empty).
var cellIDPattern = regexp.MustCompile(`^[a-z0-9]+(?:-[a-z0-9]+)*$`)

// cellManifestName reads cell id's manifest and returns the host name it
// records, with the manifest's bytes. The manifest is the one record of a
// cell: adding a cell is adding its manifest, and no cell id is written into
// code, so the public tree, which holds only placeholder cells, names none.
// Each catalogue identity (a bench alias, an arm cell on the same host) is its
// own file. It refuses cell_id_unknown when id does not have the cell id shape
// or no manifest exists for it, and cell_manifest_invalid when the manifest
// does not parse or names no host.
func cellManifestName(root, id string) (string, []byte, error) {
	if len(id) > 128 || !cellIDPattern.MatchString(id) {
		return "", nil, fmt.Errorf("cell_id_unknown")
	}
	raw, err := os.ReadFile(filepath.Join(root, "rt-core/config/cells", id+".json"))
	if errors.Is(err, fs.ErrNotExist) {
		return "", nil, fmt.Errorf("cell_id_unknown")
	}
	var manifest struct {
		Name string `json:"name"`
	}
	if err != nil || json.Unmarshal(raw, &manifest) != nil || manifest.Name == "" {
		return "", nil, fmt.Errorf("cell_manifest_invalid")
	}
	return manifest.Name, raw, nil
}

// An explicit drive variant is repository-local, including its symlink target.
// Its compiled digest must still match the cell manifest and catalogue pins.
func boundDriveProfiles(root, reference string) (string, error) {
	invalid := fmt.Errorf("cell_drive_profiles_invalid")
	if !filepath.IsLocal(reference) {
		return "", invalid
	}
	base, err := filepath.Abs(root)
	if err != nil {
		return "", invalid
	}
	base, err = filepath.EvalSymlinks(base)
	if err != nil {
		return "", invalid
	}
	resolved, err := filepath.EvalSymlinks(filepath.Join(base, reference))
	if err != nil {
		return "", invalid
	}
	rel, err := filepath.Rel(base, resolved)
	if err != nil || !filepath.IsLocal(rel) {
		return "", invalid
	}
	info, err := os.Stat(resolved)
	if err != nil || !info.IsDir() {
		return "", invalid
	}
	return resolved, nil
}
