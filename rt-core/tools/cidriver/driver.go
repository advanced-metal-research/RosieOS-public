// cidriver fails closed on missing coverage and preserves the first failure in a measured CI record.
package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	_ "embed"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strings"
	"syscall"
	"time"
)

// Measured wire identity: version 1 stores command exit codes separately from gate outcomes.
const recordSchema = "rosie.rt-core.ci.v1"

//go:embed allowed_skips.json
var allowedSkips []byte

type allowance struct {
	Test      string `json:"test"`
	Note      string `json:"note"`
	Reason    string `json:"reason"`
	TrackerID string `json:"tracker_id"`
}
type skip struct {
	Line      string `json:"line"`
	Reason    string `json:"reason"`
	TrackerID string `json:"tracker_id"`
}
type command struct {
	Name string            `json:"name"`
	Argv []string          `json:"command"`
	Dir  string            `json:"directory"`
	Env  map[string]string `json:"environment,omitempty"`
}
type result struct {
	Code           int
	Stdout, Stderr string
	Error          string
}
type unsupportedHost struct{ text string }

func (e unsupportedHost) Error() string { return e.text }

type runner func(context.Context, command) result
type stepRecord struct {
	command
	Code         int    `json:"exit_code"`
	WallNS       int64  `json:"wall_ns"`
	StdoutDigest string `json:"stdout_sha256"`
	StderrDigest string `json:"stderr_sha256"`
	Log          string `json:"log"`
	Status       string `json:"status"`
	Reason       string `json:"reason,omitempty"`
	Skips        []skip `json:"allowed_skips,omitempty"`
}
type qualification struct {
	Status       string `json:"status"`
	Architecture string `json:"architecture,omitempty"`
	Error        string `json:"error,omitempty"`
}
type record struct {
	Schema         string          `json:"schema"`
	GitSHA         string          `json:"git_sha"`
	Inputs         json.RawMessage `json:"build_inputs"`
	Status         string          `json:"status"`
	Error          string          `json:"error,omitempty"`
	Steps          []stepRecord    `json:"steps"`
	TSan           qualification   `json:"tsan"`
	Atomics        qualification   `json:"cross_process_atomics"`
	OptionalAbsent []string        `json:"optional_targets_absent,omitempty"`
}
type driver struct {
	root, path, logDir string
	env                map[string]string
	run                runner
	out                io.Writer
	allow              []allowance
	record             record
}

func digest(text string) string {
	sum := sha256.Sum256([]byte(text))
	return hex.EncodeToString(sum[:])
}

// execute captures both streams and kills the subprocess group on cancellation, preventing orphan test daemons.
func execute(ctx context.Context, request command) result {
	cmd := exec.CommandContext(ctx, request.Argv[0], request.Argv[1:]...)
	cmd.Dir = request.Dir
	cmd.Env = os.Environ()
	for key, value := range request.Env {
		cmd.Env = append(cmd.Env, key+"="+value)
	}
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	cmd.Cancel = func() error { return syscall.Kill(-cmd.Process.Pid, syscall.SIGKILL) }
	var stdout, stderr bytes.Buffer
	cmd.Stdout, cmd.Stderr = &stdout, &stderr
	err := cmd.Run()
	r := result{Stdout: stdout.String(), Stderr: stderr.String()}
	if err != nil {
		r.Code = -1
		var exit *exec.ExitError
		if errors.As(err, &exit) {
			r.Code = exit.ExitCode()
		}
		r.Error = err.Error()
	}
	return r
}

// scanSkips requires both the test identity and its explanatory note, preventing broad name-only exemptions.
func scanSkips(log string, allow []allowance) ([]skip, error) {
	goSkip := regexp.MustCompile(`--- SKIP: ([^ ]+)`)
	goContext := regexp.MustCompile(`=== (RUN|CONT|NAME)\s+(\S+)`)
	note := regexp.MustCompile(`(?i)\b(SKIP|skipped|not_applicable)\b`)
	lines := strings.Split(log, "\n")
	contexts := map[string][]string{}
	active := ""
	var found []skip
	for _, line := range lines {
		if marker := goContext.FindStringSubmatch(line); len(marker) != 0 {
			active = marker[2]
			if marker[1] == "RUN" {
				contexts[active] = nil
			}
		}
		if active != "" {
			contexts[active] = append(contexts[active], line)
		}
		match := goSkip.FindStringSubmatch(line)
		if len(match) == 0 && !note.MatchString(line) {
			continue
		}
		name, context := "", strings.TrimSpace(line)
		if len(match) != 0 {
			name = match[1]
			context = strings.Join(contexts[name], "\n")
		}
		accepted := false
		for _, a := range allow {
			if (name == "") != (a.Test == "") {
				continue
			}
			if a.Test != "" && !regexp.MustCompile(a.Test).MatchString(name) {
				continue
			}
			if regexp.MustCompile(a.Note).MatchString(context) {
				found = append(found, skip{Line: strings.TrimSpace(line), Reason: a.Reason, TrackerID: a.TrackerID})
				accepted = true
				break
			}
		}
		if !accepted {
			return found, fmt.Errorf("unlisted_skip: %s", strings.TrimSpace(line))
		}
	}
	return found, nil
}

func loadAllowlist(data []byte) ([]allowance, error) {
	var entries []allowance
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&entries); err != nil {
		return nil, err
	}
	if err := decoder.Decode(new(any)); err != io.EOF {
		return nil, errors.New("allowlist trailing data")
	}
	for _, a := range entries {
		if a.Reason == "" || !regexp.MustCompile(`^P[0-9]+-[0-9]+$`).MatchString(a.TrackerID) || a.Note == "" {
			return nil, errors.New("allowlist requires reason, note and tracker_id")
		}
		for _, pattern := range []string{a.Test, a.Note} {
			if _, err := regexp.Compile(pattern); err != nil {
				return nil, err
			}
		}
	}
	return entries, nil
}

func (d *driver) write() error {
	data, err := json.MarshalIndent(d.record, "", "  ")
	if err != nil {
		return err
	}
	if err = os.MkdirAll(filepath.Dir(d.path), 0755); err != nil {
		return err
	}
	tmp := d.path + ".tmp"
	if err = os.WriteFile(tmp, append(data, '\n'), 0644); err != nil {
		return err
	}
	return os.Rename(tmp, d.path)
}

// step records actual exit status even for a required negative probe; a probe expectation never rewrites it to zero.
func (d *driver) step(ctx context.Context, name string, argv []string, check func(result) error) (result, error) {
	request := command{Name: name, Argv: argv, Dir: d.root, Env: d.env}
	request.Env = make(map[string]string, len(d.env))
	for key, value := range d.env {
		request.Env[key] = value
	}
	start := time.Now()
	r := d.run(ctx, request)
	entry := stepRecord{command: request, Code: r.Code, WallNS: time.Since(start).Nanoseconds(), StdoutDigest: digest(r.Stdout), StderrDigest: digest(r.Stderr), Status: "passed"}
	err := ctx.Err()
	if err == nil && check != nil {
		err = check(r)
	} else if err == nil && r.Code != 0 {
		err = fmt.Errorf("command_failed: exit_code=%d", r.Code)
	}
	if err != nil && r.Error != "" {
		err = fmt.Errorf("%w: %s", err, r.Error)
	}
	var unsupported unsupportedHost
	if errors.As(err, &unsupported) {
		entry.Status, entry.Reason = "unsupported_on_host", unsupported.text
		d.record.TSan = qualification{Status: "unsupported_on_host", Error: unsupported.text}
		err = nil
	}
	var skipErr error
	entry.Skips, skipErr = scanSkips(r.Stdout+"\n"+r.Stderr, d.allow)
	err = errors.Join(err, skipErr)
	entry.Log = filepath.Join(d.logDir, fmt.Sprintf("%02d-%s.log", len(d.record.Steps)+1, name))
	if logErr := os.MkdirAll(d.logDir, 0755); logErr != nil {
		err = errors.Join(err, logErr)
	} else {
		err = errors.Join(err, os.WriteFile(entry.Log, []byte(r.Stdout+"\n"+r.Stderr), 0644))
	}
	if err != nil {
		entry.Status, entry.Reason = "failed", err.Error()
	}
	d.record.Steps = append(d.record.Steps, entry)
	if err != nil {
		d.record.Status, d.record.Error = "failed", name+": "+err.Error()
	}
	err = errors.Join(err, d.write())
	fmt.Fprintf(d.out, "%s: %s exit=%d wall_ns=%d\n", name, entry.Status, r.Code, entry.WallNS)
	if err != nil {
		return r, fmt.Errorf("%s: %w", name, err)
	}
	return r, nil
}

// finish persists setup failures as well as suite failures, so an incomplete run cannot become a pass.
func (d *driver) finish(work func() error) (err error) {
	defer func() {
		if err != nil {
			d.record.Status, d.record.Error = "failed", err.Error()
		} else {
			d.record.Status = "passed"
		}
		err = errors.Join(err, d.write())
	}()
	return work()
}
