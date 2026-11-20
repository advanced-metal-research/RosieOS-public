// Package robotmodel answers which robot models this tree knows. The record
// of a model is its description directory, robot_description/robots/<id>/:
// one with a robot.urdf is an arm, one with only a description manifest is a
// bench (bare motors, a rail). No model id is written into code, so a model
// is added by adding its description, and a tree without a description (the
// public mirror leaves the real robots out) does not know that model at all.
//
// The answer is fail-closed: an id that is not a plain model id, or a tree
// whose root cannot be found, knows no model, and every check refuses it.
package robotmodel

import (
	"os"
	"path/filepath"
	"regexp"
	"sync"
)

// Class values.
const (
	Arm   = "arm"
	Bench = "bench"
)

// idPattern is the shape of a model id: its description's directory name.
// Ids arrive in requests, catalogues and saved programs, so nothing else may
// reach a path (no "/", "..", or upper case).
var idPattern = regexp.MustCompile(`^[a-z0-9]+(?:_[a-z0-9]+)*$`)

var (
	mu   sync.Mutex
	root string
	set  bool
)

// SetRoot names the repository whose robot_description/robots holds the
// descriptions. The server calls it once at start; until then the root is
// found by walking up from the working directory.
func SetRoot(repo string) {
	mu.Lock()
	defer mu.Unlock()
	root, set = repo, true
}

func repoRoot() string {
	mu.Lock()
	defer mu.Unlock()
	if !set {
		root, set = discover(), true
	}
	return root
}

// discover walks up from the working directory to the first directory that
// holds robot_description/robots; "" when there is none.
func discover() string {
	dir, err := os.Getwd()
	if err != nil {
		return ""
	}
	for {
		if info, err := os.Stat(filepath.Join(dir, "robot_description", "robots")); err == nil && info.IsDir() {
			return dir
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			return ""
		}
		dir = parent
	}
}

// Class is Arm, Bench or "" (unknown) for model id.
func Class(id string) string {
	repo := repoRoot()
	if repo == "" || len(id) > 64 || !idPattern.MatchString(id) {
		return ""
	}
	dir := filepath.Join(repo, "robot_description", "robots", id)
	if !regular(filepath.Join(dir, "robot_description_manifest.json")) {
		return ""
	}
	if regular(filepath.Join(dir, "robot.urdf")) {
		return Arm
	}
	return Bench
}

// IsArm reports whether id is an arm model this tree describes.
func IsArm(id string) bool { return Class(id) == Arm }

func regular(path string) bool {
	info, err := os.Stat(path)
	return err == nil && info.Mode().IsRegular()
}
