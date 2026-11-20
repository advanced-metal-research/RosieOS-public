package rtctl

import (
	"bytes"
	"encoding/json"
	"encoding/xml"
	"io"
	"os"
	"path"
	"path/filepath"
	"strings"

	"rosieos/robotdesc"
)

// compileResources captures immutable model bytes and canonical drive content.
// Native configuration identity remains independent of the artefact directory.
func compileResources(result *Compiled, machine object, profiles []any, robot *RobotBinding, profilesDir string) {
	manifest := resourceManifest{Drives: []driveResource{}}
	for i, value := range asArray(machine.get("axes")) {
		axis := asObject(value)
		manifest.Drives = append(manifest.Drives, driveResource{Axis: token(axis.get("name")), ConfigName: token(axis.get("profile")),
			ConfigSHA256: identityDigest(canonical(profiles[i])), SlavePosition: uint16(finite(axis.get("slave_position"), "slave position"))})
	}
	result.resourceData = map[string][]byte{}
	if robot != nil {
		// The description, file for file: the manifest and everything it
		// registers, by description-relative path, plus this machine's
		// machine_planning_calibration.json (docs/ROBOT-DESCRIPTION-CONTRACT.md). The definition
		// rt-core read and the URDF it bounded the axes by are among them.
		manifest.Robot = &robotResources{ModelID: robot.ModelID, RobotDescriptionSHA256: robot.Description.RobotDescriptionSHA256, MachinePlanningCalibrationSHA256: robot.MachinePlanningCalibrationSHA256, Resources: []resourceInfo{}}
		add := func(reference, media string, data []byte) {
			sha := identityDigest(string(data))
			manifest.Robot.Resources = append(manifest.Robot.Resources, resourceInfo{Path: reference, SHA256: sha, Bytes: uint64(len(data)), MediaType: media})
			result.resourceData[sha] = data
		}
		registered, err := robotdesc.DescriptionFiles(boundPath(robot.repository, robot.Description.Path))
		if err != nil {
			reject("robot description: %v", err)
		}
		for _, rel := range append([]string{robotdesc.Manifest}, registered...) {
			add(rel, resourceMedia(rel), readBoundedResource(boundPath(robot.repository, robot.Description.Path+"/"+rel)))
		}
		add("machine_planning_calibration.json", "application/json", robot.machinePlanningCalibration)
	}
	var total int
	for _, data := range result.resourceData {
		if len(data) > maxResourceBytes {
			reject("compiled resource exceeds byte bound")
		}
		total += len(data)
	}
	if total > maxResourceSetBytes || manifest.Robot != nil && len(manifest.Robot.Resources) > maxResourceEntries {
		reject("compiled resource set exceeds bound")
	}
	result.resources = manifest
}

// meshReferences keeps exact URDF references, deduplicating visual/collision
// reuse. Unsupported locations fail closed instead of returning an incomplete set.
func meshReferences(urdf []byte) []string {
	decoder := xml.NewDecoder(bytes.NewReader(urdf))
	seen := map[string]bool{}
	var references []string
	for {
		token, err := decoder.Token()
		if err == io.EOF {
			return references
		}
		if err != nil {
			reject("mesh URDF: %v", err)
		}
		start, ok := token.(xml.StartElement)
		if !ok || start.Name.Local != "mesh" {
			continue
		}
		var reference string
		for _, attr := range start.Attr {
			if attr.Name.Local == "filename" {
				reference = attr.Value
			}
		}
		// Package references are the contract; file and HTTP URIs are refused.
		validateRobotReference(strings.TrimPrefix(reference, "package://"))
		if !seen[reference] {
			references = append(references, reference)
			seen[reference] = true
		}
		if len(references) > maxResourceEntries {
			reject("too many mesh resources")
		}
	}
}

// resourceMedia is the media type a description file is served as, by its
// extension: the planner's readers key on it, and the adapter admits only
// the types it names.
func resourceMedia(rel string) string {
	switch strings.ToLower(path.Ext(rel)) {
	case ".dae":
		return "model/vnd.collada+xml"
	case ".stl":
		return "model/stl"
	case ".json":
		return "application/json"
	case ".urdf", ".srdf", ".xml":
		return "application/xml"
	}
	return "application/octet-stream"
}

func readBoundedResource(file string) []byte {
	f, err := os.Open(file)
	if err != nil {
		reject("robot resource unavailable: %v", err)
	}
	defer f.Close()
	info, err := f.Stat()
	if err != nil || !info.Mode().IsRegular() || info.Size() > maxResourceBytes {
		reject("invalid robot resource size or type: %s", file)
	}
	data, err := io.ReadAll(io.LimitReader(f, maxResourceBytes+1))
	if err != nil || len(data) > maxResourceBytes {
		reject("robot resource read: %s: %v", file, err)
	}
	return data
}

func writeCompiledResources(compiled Compiled, directory string) error {
	manifest := compiled.resources
	manifest.ConfigurationSHA256, manifest.MachineSHA256 = compiled.SHA256, compiled.MachineSHA256
	if err := os.MkdirAll(filepath.Join(directory, "resources"), 0755); err != nil {
		return err
	}
	for digest, data := range compiled.resourceData {
		if err := os.WriteFile(filepath.Join(directory, "resources", digest), data, 0644); err != nil {
			return err
		}
	}
	if err := os.WriteFile(filepath.Join(directory, "configuration.identity"), []byte(compiled.configurationIdentity), 0644); err != nil {
		return err
	}
	if compiled.baseConfigurationIdentity != "" {
		if err := os.WriteFile(filepath.Join(directory, "configuration.base.identity"), []byte(compiled.baseConfigurationIdentity), 0644); err != nil {
			return err
		}
	}
	data, err := json.MarshalIndent(manifest, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(filepath.Join(directory, "resources.json"), append(data, '\n'), 0644)
}
