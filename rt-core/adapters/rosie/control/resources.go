//go:build linux

package control

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"encoding/xml"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"path"
	"strconv"
	"strings"
	"syscall"

	"rosieos/robotdesc"
	"rosieos/rt-core/ipcclient"
)

// Unverified resource admission bounds: 32 MiB per file, 128 MiB per set,
// 1024 references and 8 MiB for compiled metadata. Admission precedes serving.
const (
	MaxResourceBytes         = 32 << 20
	MaxResourceSetBytes      = 128 << 20
	MaxResourceEntries       = 1024
	maxResourceMetadataBytes = 8 << 20
)

// ResourceManifest binds packaged observations to the native configuration;
// loading a different deployment cannot silently supply a different model.
type ResourceManifest struct {
	ConfigurationSHA256 string             `json:"configuration_sha256"`
	MachineSHA256       string             `json:"machine_sha256"`
	Robot               *RobotDescription  `json:"robot"`
	Drives              []DriveDescription `json:"drives"`
}

type compiledResources struct {
	manifest ResourceManifest
	data     map[string][]byte
	media    map[string]string
}

// ValidResourceDigest accepts only a canonical SHA-256 path component, preventing
// callers from turning resource lookup into traversal or arbitrary file access.
func ValidResourceDigest(digest string) bool {
	b, err := hex.DecodeString(digest)
	return err == nil && len(b) == sha256.Size && digest == strings.ToLower(digest)
}

func resourceDigest(data []byte) string { return fmt.Sprintf("%x", sha256.Sum256(data)) }

func readArtefact(root *os.Root, name string, limit int64) ([]byte, error) {
	f, err := root.OpenFile(name, os.O_RDONLY|syscall.O_NONBLOCK, 0)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	info, err := f.Stat()
	if err != nil {
		return nil, err
	}
	if !info.Mode().IsRegular() || info.Size() > limit {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	b, err := io.ReadAll(io.LimitReader(f, limit+1))
	if err == nil && int64(len(b)) > limit {
		err = errors.New(ReasonResourceArtifactInvalid)
	}
	return b, err
}

// loadCompiledResources validates and snapshots the complete set at startup.
// Requests never consult repository files or race edits to the artefact on disk.
func loadCompiledResources(directory string, native ipcclient.Description) (*compiledResources, error) {
	root, err := os.OpenRoot(directory)
	if err != nil {
		return nil, err
	}
	defer root.Close()
	b, err := readArtefact(root, "resources.json", maxResourceMetadataBytes)
	if err != nil {
		return nil, err
	}
	set := &compiledResources{data: map[string][]byte{}, media: map[string]string{}}
	if err = json.Unmarshal(b, &set.manifest); err != nil {
		return nil, err
	}
	m := &set.manifest
	if m.ConfigurationSHA256 != native.ConfigurationSHA256 || m.MachineSHA256 != native.MachineSHA256 || len(m.Drives) != len(native.Axes) {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	identity, err := readArtefact(root, "configuration.identity", maxResourceMetadataBytes)
	if err != nil {
		return nil, err
	}
	if resourceDigest(identity) != native.ConfigurationSHA256 {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	var envelope struct {
		BaseSHA256 string `json:"base_sha256"`
	}
	if json.Unmarshal(identity, &envelope) != nil {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	if envelope.BaseSHA256 != "" {
		identity, err = readArtefact(root, "configuration.base.identity", maxResourceMetadataBytes)
		if err != nil {
			return nil, err
		}
		if resourceDigest(identity) != envelope.BaseSHA256 {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
	}
	var configuration struct {
		Machine struct {
			Robot *struct {
				RobotDescription struct {
					Path   string `json:"path"`
					SHA256 string `json:"sha256"`
				} `json:"robot_description"`
				BenchMeasurementProfile string `json:"bench_measurement_profile"`
			} `json:"robot"`
			Axes []struct {
				Name          string  `json:"name"`
				Profile       string  `json:"profile"`
				RobotJoint    string  `json:"robot_joint"`
				SlavePosition *uint16 `json:"slave_position"`
			} `json:"axes"`
		} `json:"machine"`
		Profiles []json.RawMessage `json:"profiles"`
	}
	if err = json.Unmarshal(identity, &configuration); err != nil {
		return nil, err
	}
	if len(configuration.Profiles) != len(m.Drives) || len(configuration.Machine.Axes) != len(m.Drives) {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	for i, drive := range m.Drives {
		axis := configuration.Machine.Axes[i]
		name := axis.Name
		if configuration.Machine.Robot != nil {
			name = axis.RobotJoint
		}
		if drive.Axis != native.Axes[i].ID || drive.Axis != name || drive.ConfigName == "" || drive.ConfigSHA256 != resourceDigest(configuration.Profiles[i]) || drive.VerifiedIdentity != nil ||
			axis.SlavePosition == nil || drive.SlavePosition != *axis.SlavePosition || configuration.Machine.Robot == nil && drive.ConfigName != axis.Profile {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
	}
	if m.Robot == nil {
		if configuration.Machine.Robot != nil {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
		return set, nil
	}
	// The robot is served whole: the description's manifest and every file
	// it registers, by description-relative path, plus this machine's
	// machine_planning_calibration.json (docs/ROBOT-DESCRIPTION-CONTRACT.md). Admission recomputes the
	// identity over the served files and requires it to be the one the
	// machine was compiled against; a set that hashes to anything else, or
	// carries a file the manifest does not name, is refused whole.
	r := m.Robot
	machineRobot := configuration.Machine.Robot
	if machineRobot == nil || r.ModelID == "" || path.Base(machineRobot.RobotDescription.Path) != r.ModelID || machineRobot.RobotDescription.SHA256 != r.RobotDescriptionSHA256 ||
		len(r.Resources) == 0 || len(r.Resources) > MaxResourceEntries {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	byPath := map[string]ResourceInfo{}
	var total uint64
	for _, resource := range r.Resources {
		if _, duplicate := byPath[resource.Path]; duplicate || !ValidResourceDigest(resource.SHA256) || resource.Path == "" || resource.Bytes > MaxResourceBytes || !validResourceMedia(resource.MediaType) {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
		byPath[resource.Path] = resource
		if previous, ok := set.data[resource.SHA256]; ok {
			if uint64(len(previous)) != resource.Bytes || set.media[resource.SHA256] != resource.MediaType {
				return nil, errors.New(ReasonResourceArtifactInvalid)
			}
			continue
		}
		total += resource.Bytes
		if total > MaxResourceSetBytes {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
		data, err := readArtefact(root, "resources/"+resource.SHA256, int64(resource.Bytes))
		if err != nil {
			return nil, err
		}
		if uint64(len(data)) != resource.Bytes || resourceDigest(data) != resource.SHA256 {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
		set.data[resource.SHA256], set.media[resource.SHA256] = data, resource.MediaType
	}
	manifest, ok := byPath[robotdesc.Manifest]
	if !ok || set.media[manifest.SHA256] != "application/json" {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	var registry struct {
		Schema  string   `json:"schema"`
		ModelID string   `json:"model_id"`
		Files   []string `json:"files"`
	}
	if json.Unmarshal(set.data[manifest.SHA256], &registry) != nil || registry.Schema != robotdesc.ManifestSchema || registry.ModelID != r.ModelID || len(registry.Files) == 0 {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	references := map[string]string{robotdesc.Manifest: manifest.SHA256}
	entries := []robotdesc.Entry{{Path: robotdesc.Manifest, SHA256: digestBytes(manifest.SHA256)}}
	for _, rel := range registry.Files {
		resource, ok := byPath[rel]
		if !ok || rel == robotdesc.Manifest || rel == "machine_planning_calibration.json" {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
		references[rel] = resource.SHA256
		entries = append(entries, robotdesc.Entry{Path: rel, SHA256: digestBytes(resource.SHA256)})
	}
	if robotdesc.HashEntries(entries) != r.RobotDescriptionSHA256 {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	cell, ok := byPath["machine_planning_calibration.json"]
	if !ok || set.media[cell.SHA256] != "application/json" || "sha256:"+cell.SHA256 != r.MachinePlanningCalibrationSHA256 {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	references["machine_planning_calibration.json"] = cell.SHA256
	// rt-core's definition of the robot is one of the registered files.
	// Resolve the same profile selection from hash-verified inputs; a
	// manifest cannot rename an approved drive or attach it to a different
	// bus position.
	definition, ok := references[robotdesc.Definition]
	if !ok || set.media[definition] != "application/json" {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	var model struct {
		ID       string `json:"robot_id"`
		Defaults struct {
			DriveProfile string `json:"drive_profile"`
		} `json:"defaults"`
		Joints []struct {
			Name         string `json:"name"`
			DriveProfile string `json:"drive_profile"`
		} `json:"joints"`
	}
	if json.Unmarshal(set.data[definition], &model) != nil || model.ID != r.ModelID {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	profiles := map[string]string{}
	for _, joint := range model.Joints {
		if _, duplicate := profiles[joint.Name]; duplicate || joint.Name == "" {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
		profile := joint.DriveProfile
		if profile == "" {
			profile = model.Defaults.DriveProfile
		}
		profiles[joint.Name] = profile
	}
	for i, drive := range m.Drives {
		profile := profiles[configuration.Machine.Axes[i].RobotJoint]
		if override := machineRobot.BenchMeasurementProfile; override != "" {
			if model.ID != "bench_nine_motors_1to1" || profile != "a6ec_bench_1to1" || override != "a6ec_bench_1to1_fast" {
				return nil, errors.New(ReasonResourceArtifactInvalid)
			}
			profile = override
		}
		if profile == "" || drive.ConfigName != profile {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
	}
	// A described robot's meshes are the description's own files: every
	// mesh its URDF names must be registered.
	if urdf, ok := references["robot.urdf"]; ok {
		if set.media[urdf] != "application/xml" {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
		decoder := xml.NewDecoder(bytes.NewReader(set.data[urdf]))
		for {
			token, err := decoder.Token()
			if err == io.EOF {
				break
			}
			if err != nil {
				return nil, err
			}
			start, ok := token.(xml.StartElement)
			if !ok || start.Name.Local != "mesh" {
				continue
			}
			var filename string
			for _, attr := range start.Attr {
				if attr.Name.Local == "filename" {
					filename = attr.Value
				}
			}
			rel, valid := robotdesc.MeshResourcePath(filename)
			if !valid {
				return nil, errors.New(ReasonResourceArtifactInvalid)
			}
			if _, exists := references[rel]; !exists {
				return nil, errors.New(ReasonResourceArtifactInvalid)
			}
		}
	}
	if len(references) != len(r.Resources) {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	for _, resource := range r.Resources {
		if pin, exists := references[resource.Path]; !exists || pin != resource.SHA256 {
			return nil, errors.New(ReasonResourceArtifactInvalid)
		}
	}
	return set, nil
}

// digestBytes is a validated lowercase hex sha256 as bytes.
func digestBytes(digest string) [32]byte {
	var out [32]byte
	decoded, _ := hex.DecodeString(digest)
	copy(out[:], decoded)
	return out
}

func validResourceMedia(media string) bool {
	switch media {
	case "application/json", "application/xml", "model/vnd.collada+xml", "model/stl", "application/octet-stream":
		return true
	}
	return false
}

func (c *Controller) describeResources(state *ipcclient.Status) (*RobotDescription, []DriveDescription) {
	if c.resources == nil {
		return nil, []DriveDescription{}
	}
	m := c.resources.manifest
	var robot *RobotDescription
	if m.Robot != nil {
		copy := *m.Robot
		copy.Resources = append([]ResourceInfo{}, copy.Resources...)
		robot = &copy
	}
	drives := append([]DriveDescription{}, m.Drives...)
	for i := range drives {
		if state == nil || i >= len(state.Axes) || state.ConfigVerifiedMask&(1<<i) == 0 {
			continue
		}
		a := state.Axes[i]
		if a.RevisionValid != 0 && a.SlavePosition == drives[i].SlavePosition {
			drives[i].VerifiedIdentity = &DriveIdentity{ExpectedVendorID: a.VendorID, ExpectedProductCode: a.ProductCode, ObservedCoERevision: a.Revision}
		}
	}
	return robot, drives
}

func (h *Handler) serveResource(w http.ResponseWriter, r *http.Request) {
	digest := strings.TrimPrefix(r.URL.Path, "/v1/resources/")
	set := h.Controller.resources
	if !ValidResourceDigest(digest) || set == nil || set.data[digest] == nil {
		w.WriteHeader(http.StatusNotFound)
		_ = json.NewEncoder(w).Encode(Response{Schema: ResponseSchema, Operation: OperationResource, Error: ReasonResourceUnknown})
		return
	}
	data := set.data[digest]
	w.Header().Set("Content-Type", set.media[digest])
	w.Header().Set("Content-Length", strconv.Itoa(len(data)))
	w.Header().Set("ETag", `"`+digest+`"`)
	w.Header().Set("X-Content-Type-Options", "nosniff")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(data)
}
