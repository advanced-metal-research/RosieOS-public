package targets

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"rosieos/robotdesc"
)

// Resolve only the selected configuration's robot definition. Missing model
// bindings remain absent in read-only descriptors; a broken explicit binding fails
// closed instead of silently substituting legacy gearing or another robot model.
func resolveManifestRatios(manifestRef string, manifest *robotV4Manifest, repositoryRoot ...string) error {
	var configuration string
	for _, node := range manifest.Nodes {
		if node.Role == "realtime-core" && node.RTCore != nil {
			if configuration != "" {
				return fmt.Errorf("targets: ambiguous rt_core configuration")
			}
			configuration = node.RTCore.MachineConfig
			if configuration == "" {
				return fmt.Errorf("targets: missing rt_core machine_config")
			}
		}
	}
	if configuration == "" {
		return nil
	}
	root, err := os.Getwd()
	if err != nil {
		return err
	}
	if len(repositoryRoot) > 1 {
		return fmt.Errorf("targets: ambiguous repository root")
	}
	if len(repositoryRoot) == 1 {
		root, err = filepath.Abs(repositoryRoot[0])
		if err != nil {
			return err
		}
	} else {
		// Logical references retain compatibility for callers within the repository.
		for {
			if _, err := os.Stat(filepath.Join(root, filepath.FromSlash(manifestRef))); err == nil {
				break
			}
			parent := filepath.Dir(root)
			if parent == root {
				return fmt.Errorf("targets: cannot resolve manifest reference %q", manifestRef)
			}
			root = parent
		}
	}
	ratios, err := robotDefinitionRatios(root, configuration)
	if err != nil {
		return err
	}
	for i := range manifest.RobotAxes.Axes {
		axis := &manifest.RobotAxes.Axes[i]
		ratio, ok := ratios[axis.URDFJoint]
		if !ok {
			return fmt.Errorf("targets: robot definition missing joint %s", axis.URDFJoint)
		}
		axis.GearRatio = &ratio
	}
	return nil
}

// Follow release and configuration references, verify the pinned definition hash,
// and preserve rational gearing; malformed references cannot authorize a descriptor.
func robotDefinitionRatios(root, configuration string) (map[string]float64, error) {
	read := func(base, ref string) ([]byte, string, error) {
		if ref == "" || filepath.IsAbs(ref) {
			return nil, "", fmt.Errorf("targets: invalid configuration reference %q", ref)
		}
		path := filepath.Join(base, filepath.FromSlash(ref))
		rel, err := filepath.Rel(root, path)
		if err != nil || !filepath.IsLocal(rel) {
			return nil, "", fmt.Errorf("targets: reference escapes repository")
		}
		raw, err := os.ReadFile(path)
		return raw, path, err
	}
	raw, _, err := read(root, configuration)
	if err != nil {
		return nil, err
	}
	var release struct {
		Schema   string `json:"schema"`
		External struct {
			Configuration struct {
				Path string `json:"configuration"`
			} `json:"rt_core_configuration_digest"`
		} `json:"external_references"`
	}
	if err := json.Unmarshal(raw, &release); err != nil {
		return nil, err
	}
	if release.Schema == "rosie.robot-stack.release.v1" {
		raw, _, err = read(root, release.External.Configuration.Path)
		if err != nil {
			return nil, err
		}
	} else if release.Schema != "" {
		return nil, fmt.Errorf("targets: unsupported release schema %q", release.Schema)
	}
	var machine struct {
		SchemaVersion int `json:"schema_version"`
		Robot         struct {
			RobotDescription struct {
				Path   string `json:"path"`
				SHA256 string `json:"sha256"`
			} `json:"robot_description"`
		} `json:"robot"`
	}
	if err := json.Unmarshal(raw, &machine); err != nil {
		return nil, err
	}
	if machine.SchemaVersion != 1 {
		return nil, fmt.Errorf("targets: unsupported machine configuration")
	}
	// The robot is a description directory at the repository root, pinned by
	// its manifest identity (docs/ROBOT-DESCRIPTION-CONTRACT.md); rt-core's
	// definition of it is one of the registered files.
	if machine.Robot.RobotDescription.Path == "" || filepath.IsAbs(machine.Robot.RobotDescription.Path) {
		return nil, fmt.Errorf("targets: invalid configuration reference %q", machine.Robot.RobotDescription.Path)
	}
	dir := filepath.Join(root, filepath.FromSlash(machine.Robot.RobotDescription.Path))
	if rel, err := filepath.Rel(root, dir); err != nil || !filepath.IsLocal(rel) {
		return nil, fmt.Errorf("targets: reference escapes repository")
	}
	identity, err := robotdesc.RobotDescriptionSHA256(dir)
	if err != nil {
		return nil, fmt.Errorf("targets: robot description unavailable: %w", err)
	}
	if identity != machine.Robot.RobotDescription.SHA256 {
		return nil, fmt.Errorf("targets: robot description identity mismatch")
	}
	raw, _, err = read(root, machine.Robot.RobotDescription.Path+"/"+robotdesc.Definition)
	if err != nil {
		return nil, err
	}
	var definition struct {
		Schema string `json:"schema"`
		Joints []struct {
			Name      string                                   `json:"name"`
			URDFJoint string                                   `json:"urdf_joint"`
			GearRatio struct{ Numerator, Denominator float64 } `json:"gear_ratio"`
		} `json:"joints"`
	}
	if err := json.Unmarshal(raw, &definition); err != nil {
		return nil, err
	}
	if definition.Schema != "rosie.robot-definition.v1" {
		return nil, fmt.Errorf("targets: unsupported robot definition")
	}
	ratios := make(map[string]float64, len(definition.Joints))
	for _, joint := range definition.Joints {
		ratio := joint.GearRatio.Numerator / joint.GearRatio.Denominator
		if joint.Name == "" || joint.Name != joint.URDFJoint || joint.GearRatio.Numerator <= 0 || joint.GearRatio.Denominator <= 0 || math.IsNaN(ratio) || math.IsInf(ratio, 0) {
			return nil, fmt.Errorf("targets: invalid robot definition joint %q", joint.Name)
		}
		if _, exists := ratios[joint.URDFJoint]; exists {
			return nil, fmt.Errorf("targets: duplicate robot definition joint %s", joint.URDFJoint)
		}
		ratios[joint.URDFJoint] = ratio
	}
	return ratios, nil
}
