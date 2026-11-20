package main

import (
	"bytes"
	_ "embed"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"regexp"
	"strings"
)

//go:embed retired-runtime-allowlist.txt
var retiredRuntimeAllowlist string

// Interpreter tokens, source suffixes and environment assignments that mean a
// Python requirement survived. The scan's own sources are allowlisted, and the
// Make target that runs it is stripped from each line before matching, so the
// tokens can be written plainly everywhere else.
var retiredReference = regexp.MustCompile(`\bpython3?\b|\.py\b|PYTHON(?:PATH|[A-Z]*=)`)

const retiredScanTarget = "check-python-absent"

// checkRetiredRuntime scans the component's tracked sources, failing closed on
// unreadable files or git failure so missing inputs cannot establish absence.
func checkRetiredRuntime(output io.Writer) error {
	command := exec.Command("bash", "tools/packageinfo/git.sh", "ls-files", "-z", "--cached", "--", ".")
	data, err := command.Output()
	if err != nil {
		return fmt.Errorf("tracked source inventory: %w", err)
	}
	if len(data) == 0 || data[len(data)-1] != 0 {
		return fmt.Errorf("tracked source inventory is empty or incomplete")
	}
	return scanRetiredRuntime(".", strings.Split(string(data[:len(data)-1]), "\x00"), output)
}

func scanRetiredRuntime(root string, files []string, output io.Writer) error {
	var patterns []string
	for _, line := range strings.Split(strings.TrimSpace(retiredRuntimeAllowlist), "\n") {
		pattern, reason, ok := strings.Cut(line, " # ")
		if !ok || strings.TrimSpace(reason) == "" || pattern == "" {
			return fmt.Errorf("absence allowlist requires a reason for each path")
		}
		patterns = append(patterns, pattern)
	}
	hits := 0
	for _, file := range files {
		if path.IsAbs(file) || path.Clean(file) != file || file == ".." || strings.HasPrefix(file, "../") {
			return fmt.Errorf("invalid tracked component path: %q", file)
		}
		if strings.HasSuffix(file, ".py") {
			fmt.Fprintf(output, "%s: retired source file\n", file)
			hits++
		}
		allowed := false
		for _, pattern := range patterns {
			if strings.HasSuffix(pattern, "/**") {
				allowed = allowed || strings.HasPrefix(file, strings.TrimSuffix(pattern, "**"))
			} else {
				match, err := path.Match(pattern, file)
				if err != nil {
					return err
				}
				allowed = allowed || match
			}
		}
		if allowed {
			continue
		}
		data, err := os.ReadFile(filepath.Join(root, filepath.FromSlash(file)))
		if err != nil {
			return fmt.Errorf("tracked source %s: %w", file, err)
		}
		for number, line := range bytes.Split(data, []byte{'\n'}) {
			if retiredReference.Match(bytes.ReplaceAll(line, []byte(retiredScanTarget), nil)) {
				fmt.Fprintf(output, "%s:%d:%s\n", file, number+1, line)
				hits++
			}
		}
	}
	if hits != 0 {
		return fmt.Errorf("retired-runtime absence: failed (%d hits)", hits)
	}
	fmt.Fprintln(output, "retired-runtime absence: passed")
	return nil
}
