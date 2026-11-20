package simulator

import (
	"fmt"
	"os"
	"os/user"
	"path/filepath"
	"runtime"
	"strings"
)

const defaultJogBinaryCacheRel = "motion-server-bin/robot-v4-cartesiand"

// CanonicalJogBinaryCandidates is the search path for the isolated Motion
// Server jog runtime (robot-v4-cartesiand). Tesseract is not on this list.
func CanonicalJogBinaryCandidates(repoRoot string) []string {
	repoRoot = filepath.Clean(strings.TrimSpace(repoRoot))
	candidates := make([]string, 0, 6)
	add := func(path string) {
		path = filepath.Clean(strings.TrimSpace(path))
		if path == "" || path == "." {
			return
		}
		for _, existing := range candidates {
			if existing == path {
				return
			}
		}
		candidates = append(candidates, path)
	}
	if repoRoot != "" && repoRoot != "." {
		add(filepath.Join(repoRoot, filepath.FromSlash(defaultCanonicalBinary)))
	}
	// Never search robot/v4/motion-server: that tree is gone. Cache ELF copies
	// are fallbacks of the same motion-server/v1 binary, not a second product.
	if cache := strings.TrimSpace(os.Getenv("ROSIEOS_OLP_CACHE")); cache != "" {
		add(filepath.Join(cache, filepath.FromSlash(defaultJogBinaryCacheRel)))
	}
	if home, err := os.UserHomeDir(); err == nil {
		add(filepath.Join(home, ".cache", "rosieos-olp", filepath.FromSlash(defaultJogBinaryCacheRel)))
	}
	// The login's Linux home, for a process whose HOME points elsewhere (WSL).
	if login, err := user.Current(); err == nil && login.Username != "" {
		add(filepath.Join("/home", login.Username, ".cache", "rosieos-olp", filepath.FromSlash(defaultJogBinaryCacheRel)))
	}
	return candidates
}

// ResolveCanonicalJogBinary returns the first runnable robot-v4-cartesiand.
func ResolveCanonicalJogBinary(repoRoot string) (string, error) {
	if path := os.Getenv("OFFLINE_PROGRAMMING_JOG_BINARY"); path != "" {
		if !jogBinaryRunnable(path) {
			return "", fmt.Errorf("configured robot-v4-cartesiand is not executable: %s", path)
		}
		return path, nil
	}
	candidates := CanonicalJogBinaryCandidates(repoRoot)
	tried := make([]string, 0, len(candidates))
	for _, candidate := range candidates {
		tried = append(tried, candidate)
		resolved := candidate
		if linked, err := filepath.EvalSymlinks(candidate); err == nil && linked != "" {
			resolved = linked
		}
		if jogBinaryRunnable(resolved) {
			return resolved, nil
		}
		if resolved != candidate && jogBinaryRunnable(candidate) {
			return candidate, nil
		}
	}
	return "", fmt.Errorf(
		"jog runtime unavailable: robot-v4-cartesiand binary is not built (looked in %s)",
		strings.Join(tried, ", "),
	)
}

func jogBinaryRunnable(path string) bool {
	if strings.TrimSpace(path) == "" || !isELFExecutable(path) {
		return false
	}
	if runtime.GOOS == "windows" {
		return true
	}
	info, err := os.Stat(path)
	return err == nil && info.Mode().Perm()&0o111 != 0
}
