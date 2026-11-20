// Package robotdesc is the robot description contract in Go, shared by the
// two sides that must agree on it: rtctl, which pins a description into a
// cell's compiled configuration and serves it, and the OLP, which reads a
// description and the cell's settings to compile the planning context
// (docs/ROBOT-DESCRIPTION-CONTRACT.md, docs/PLANNING-CONTEXT.md). One
// implementation, so "which robot is this" and "may this cell deviate by
// this much" have one answer in Go; the Python twin is held to the same
// vectors.
//
// This file is the identity: a sha256 over the files the description's own
// manifest names, read from disk.
//
// Manifest rather than a hash of the directory: a description directory also
// carries CAD exports, the scripts that made them and measurement reports, and
// none of those are the robot. Hashing the directory would put a re-exported
// STEP into the robot's identity and the cell would refuse every plan made
// before it.
package robotdesc

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

const (
	// Manifest is the file in a description directory that says which files
	// the robot is made of.
	Manifest = "robot_description_manifest.json"
	// Definition is the file in a description directory that says what
	// rt-core needs of the robot -- gearing, encoders, drive profiles, which
	// URDF joint each drive turns -- and nothing the planner reads.
	Definition = "rtcore_definition.json"
	// MachinePlanningCalibration is the name the cell serves its own
	// deviation from the description under, beside the description's files
	// (rtctl compileResources, the adapter's admission). A description may
	// not register a file of that name: the two would collide in the served
	// set, rtctl would compile it and the adapter would then refuse the whole
	// set at start with resource_artifact_invalid, a cell down for a manifest
	// typo. Refused here, where the manifest is read.
	MachinePlanningCalibration = "machine_planning_calibration.json"
	// ManifestSchema is the only schema Manifest may carry.
	ManifestSchema = "rosie.robot-manifest.v1"
	// identityDomain separates this hash from every other sha256 in the stack.
	identityDomain = "rosie.robot-description.identity.v1\x00"
)

type manifest struct {
	Schema  string   `json:"schema"`
	ModelID string   `json:"model_id"`
	Files   []string `json:"files"`
}

// DescriptionFiles is the sorted list of paths the manifest names, validated,
// without the manifest itself. A registered file that is missing or is not a
// regular file is an error: the identity is then unknown, never something
// else's.
func DescriptionFiles(dir string) ([]string, error) {
	path := filepath.Join(dir, Manifest)
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("%s: a description says which files it is made of: %w", path, err)
	}
	var document manifest
	if err := json.Unmarshal(raw, &document); err != nil {
		return nil, fmt.Errorf("%s is not JSON: %w", path, err)
	}
	if document.Schema != ManifestSchema {
		return nil, fmt.Errorf("%s schema %q is not %q", path, document.Schema, ManifestSchema)
	}
	if model := filepath.Base(dir); document.ModelID != model {
		return nil, fmt.Errorf("%s is for model %q, the directory is %q", path, document.ModelID, model)
	}
	if len(document.Files) == 0 {
		return nil, fmt.Errorf("%s: files must be a non-empty list", path)
	}
	seen := make(map[string]bool, len(document.Files))
	files := make([]string, 0, len(document.Files))
	for _, entry := range document.Files {
		if entry == "" || strings.HasPrefix(entry, "/") || strings.Contains(entry, `\`) ||
			seen[entry] || entry == Manifest {
			return nil, fmt.Errorf("%s: %q is not a description-relative path", path, entry)
		}
		if entry == MachinePlanningCalibration {
			return nil, fmt.Errorf("%s: %q is the cell's own file, served beside the description; a description may not register one",
				path, entry)
		}
		for _, part := range strings.Split(entry, "/") {
			if part == "" || part == "." || part == ".." {
				return nil, fmt.Errorf("%s: %q escapes the description", path, entry)
			}
		}
		info, err := os.Stat(filepath.Join(dir, filepath.FromSlash(entry)))
		if err != nil || !info.Mode().IsRegular() {
			return nil, fmt.Errorf("%s registers %q, which is not a file here", path, entry)
		}
		seen[entry] = true
		files = append(files, entry)
	}
	sort.Strings(files)
	return files, nil
}

// RobotDescriptionSHA256 is the description's identity, as "sha256:" + 64 hex
// characters.
func RobotDescriptionSHA256(dir string) (string, error) {
	files, err := DescriptionFiles(dir)
	if err != nil {
		return "", err
	}
	return HashFiles(dir, append([]string{Manifest}, files...))
}

// HashFiles is the identity over files on disk: `entries` by path under
// `dir`, each read and hashed. The manifest is hashed as an entry of its
// own, so dropping a name from it is itself a change of identity.
func HashFiles(dir string, entries []string) (string, error) {
	hashed := make([]Entry, 0, len(entries))
	for _, entry := range entries {
		data, err := os.ReadFile(filepath.Join(dir, filepath.FromSlash(entry)))
		if err != nil {
			return "", err
		}
		hashed = append(hashed, Entry{Path: entry, SHA256: sha256.Sum256(data)})
	}
	return HashEntries(hashed), nil
}

// Entry is one registered file as the identity sees it: its path and the
// sha256 of its bytes. A reader that already holds the files' digests -- a
// cell serving them as content-addressed resources -- hashes these without
// the bytes.
type Entry struct {
	Path   string
	SHA256 [32]byte
}

// HashEntries is the identity layout: the domain, the entry count, then each
// path and the sha256 of its bytes, in path order.
func HashEntries(entries []Entry) string {
	ordered := append([]Entry(nil), entries...)
	sort.Slice(ordered, func(i, j int) bool { return ordered[i].Path < ordered[j].Path })
	digest := sha256.New()
	digest.Write([]byte(identityDomain))
	_ = binary.Write(digest, binary.BigEndian, uint64(len(ordered)))
	for _, entry := range ordered {
		_ = binary.Write(digest, binary.BigEndian, uint64(len(entry.Path)))
		digest.Write([]byte(entry.Path))
		digest.Write(entry.SHA256[:])
	}
	return "sha256:" + hex.EncodeToString(digest.Sum(nil))
}

// MachinePlanningCalibrationSHA256 is a machine_planning_calibration.json's identity: the sha256 of the bytes the cell
// served, rendered the way the daemon renders it. Of the BYTES, not of a
// re-serialisation: the cell hashed what it holds, and a plan that hashed a
// tidied copy of it would be refused for a difference nobody made.
func MachinePlanningCalibrationSHA256(bytes []byte) string {
	sum := sha256.Sum256(bytes)
	return "sha256:" + hex.EncodeToString(sum[:])
}
