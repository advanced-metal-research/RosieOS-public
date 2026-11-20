package robotconfig

// A description fetched from the cell that holds it, kept by its identity.
//
// The cell advertises WHICH description it has (`robot_description_sha256`) and, since
// docs/PLANNING-CONTEXT.md, serves the files themselves (`kind:
// "description"`). What arrives is not trusted: the manifest identity is
// recomputed over the bytes and the directory is kept only when it equals
// the advertised hash. A content-addressed name cannot be answered with
// something else, which is what makes fetching from the cell safe -- and
// what makes `robot_cell_mismatch` unreachable while a cell is connected,
// since the planner then plans the description the cell holds.
//
// The cache is a cache: `<root>/<hex of robot_description_sha256>/<model_id>/`, one
// store per identity, rebuilt from the cell whenever it is missing. Nothing
// in it is a source of truth, and deleting it costs one fetch.

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
)

// Cached is the fetched description with this identity, when the cache
// holds one that still hashes to it.
func Cached(cacheRoot, modelID, robotSHA256 string) (Description, bool) {
	dir := cacheDir(cacheRoot, robotSHA256)
	if dir == "" {
		return Description{}, false
	}
	found, err := At(dir).Load(modelID)
	if err != nil || found.Identity != robotSHA256 {
		return Description{}, false
	}
	return found, true
}

// Adopt writes the files a cell served into the cache under the identity it
// advertised, verifies them, and loads the result. The files are the
// manifest and everything it registers, by description-relative path. A
// set that hashes to anything else is removed again and refused: the cell
// said one robot and sent another.
func Adopt(cacheRoot, modelID, robotSHA256 string, files map[string][]byte) (Description, error) {
	if modelID == "" || strings.ContainsAny(modelID, `/\.`) {
		return Description{}, fmt.Errorf("model id %q is not a description name", modelID)
	}
	root := cacheDir(cacheRoot, robotSHA256)
	if root == "" {
		return Description{}, fmt.Errorf("robot_description_sha256 %q is not an identity", robotSHA256)
	}
	dir := filepath.Join(root, modelID)
	// Written beside the final place and renamed in, so a half-written
	// description is never what `Cached` finds.
	if err := os.MkdirAll(cacheRoot, 0o755); err != nil {
		return Description{}, err
	}
	staging, err := os.MkdirTemp(cacheRoot, "fetching-")
	if err != nil {
		return Description{}, err
	}
	defer os.RemoveAll(staging)
	staged := filepath.Join(staging, modelID)
	for path, data := range files {
		if path == "" || strings.HasPrefix(path, "/") || strings.Contains(path, `\`) {
			return Description{}, fmt.Errorf("the cell served %q, which is not a description-relative path", path)
		}
		for _, part := range strings.Split(path, "/") {
			if part == "" || part == "." || part == ".." {
				return Description{}, fmt.Errorf("the cell served %q, which escapes the description", path)
			}
		}
		target := filepath.Join(staged, filepath.FromSlash(path))
		if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
			return Description{}, err
		}
		if err := os.WriteFile(target, data, 0o644); err != nil {
			return Description{}, err
		}
	}
	identity, err := RobotDescriptionSHA256(staged)
	if err != nil {
		return Description{}, fmt.Errorf("the cell served a description without an identity: %w", err)
	}
	if identity != robotSHA256 {
		return Description{}, fmt.Errorf(
			"the cell advertised description %s and served files that hash to %s; refused",
			robotSHA256, identity)
	}
	if err := os.MkdirAll(root, 0o755); err != nil {
		return Description{}, err
	}
	_ = os.RemoveAll(dir)
	if err := os.Rename(staged, dir); err != nil {
		return Description{}, err
	}
	return At(root).Load(modelID)
}

// cacheDir is the store for one identity, or "" for a string that is not
// one: the hex alone names the directory, so a path can never be smuggled
// in through the hash.
func cacheDir(cacheRoot, robotSHA256 string) string {
	hex, ok := strings.CutPrefix(robotSHA256, "sha256:")
	if !ok || len(hex) != 64 || cacheRoot == "" {
		return ""
	}
	for _, r := range hex {
		if !(r >= '0' && r <= '9' || r >= 'a' && r <= 'f') {
			return ""
		}
	}
	return filepath.Join(cacheRoot, hex)
}
