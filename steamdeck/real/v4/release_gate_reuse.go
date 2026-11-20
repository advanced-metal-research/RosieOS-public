package main

import (
	"bufio"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"
)

// A test gate that passed is not run again for the same tests of the same
// code. The first live release (one cell, 2026-10-07) re-ran a 12 min full gate
// on every retry of bc423a86, although the gate had already passed three
// times on that commit and only later steps had failed (DEVLOG 20:17).
//
// A pass is keyed by the cell, the commit, its git tree, the level, the
// suites it ran and the gate definition (gateDefinitionDigest: every suite's
// directory, command and ordering, and the scripts that run and parse them,
// i.e. the deploy tool's version of the gate). A later build of the same
// commit reuses it when every suite it would run now was run by that pass
// (a full pass covers every level) and the definition is unchanged. The
// recorded per-suite results are classified again against today's known
// failures, so removing a known-failure line can turn a reused pass into a
// failure, never the reverse. --regate ignores the record.

type gatePassRecord struct {
	Cell       string               `json:"cell"`
	Commit     string               `json:"commit"`
	Tree       string               `json:"tree"`
	Level      string               `json:"level"`
	Gate       string               `json:"gate_definition"`
	Suites     []string             `json:"suites"`
	Steps      []cellDeployGateStep `json:"steps"`
	Deployment string               `json:"deployment"`
	At         string               `json:"at"`
	// Partial retains only independently successful suites from a failed gate.
	// It is never evidence that the whole gate passed.
	Partial bool `json:"partial,omitempty"`
	// Keep Level distinct from "full": older readers do not know Partial and
	// would otherwise treat a failed full attempt as covering every suite.
	AttemptLevel string `json:"attempt_level,omitempty"`
	// Per suite that ran: its definition digest (gateSuiteDefinitionDigest)
	// and the hash of its inputs at Commit (gateInputHash, every input
	// counted), for fast selection against the last pass of each suite
	// (release_gate_history.go). Records before 2026-10-08 have neither and
	// count only under an unchanged whole-gate definition.
	SuiteDefinitions map[string]string `json:"suite_definitions,omitempty"`
	SuiteInputs      map[string]string `json:"suite_inputs,omitempty"`
}

// gateDefinitionDigest identifies the gate definition: a change to any
// suite, the scheduler or the log parser makes earlier passes unusable.
func gateDefinitionDigest() string {
	h := sha256.New()
	for _, s := range gateSuites {
		fmt.Fprintf(h, "%s\x1f%s\x1f%s\x1f%s\x1f%s\x1f%t\n", s.Name, s.Dir, s.Cmd, strings.Join(s.Needs, ","), strings.Join(s.After, ","), s.Alone)
	}
	for _, part := range []string{gateSchedulerShell, cellDeployGatePy, cellDeployGateTSH} {
		h.Write([]byte(part))
		h.Write([]byte{0})
	}
	return hex.EncodeToString(h.Sum(nil))
}

// gateSuiteDefinitionDigest is one suite's part of the gate definition: a
// change to its command or ordering, the scheduler or the log parser makes
// its earlier passes unusable, and leaves the other suites' passes standing.
func gateSuiteDefinitionDigest(s gateSuite) string {
	h := sha256.New()
	fmt.Fprintf(h, "%s\x1f%s\x1f%s\x1f%s\x1f%s\x1f%t\n", s.Name, s.Dir, s.Cmd, strings.Join(s.Needs, ","), strings.Join(s.After, ","), s.Alone)
	for _, part := range []string{gateSchedulerShell, cellDeployGatePy, cellDeployGateTSH} {
		h.Write([]byte(part))
		h.Write([]byte{0})
	}
	return hex.EncodeToString(h.Sum(nil))
}

func gatePassFile() (string, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	return filepath.Join(home, ".rosie", "deploy-journal", "gate-passes.jsonl"), nil
}

// gateTreeOf is the commit's git tree: two commits with one tree test the
// same code.
var gateTreeOf = func(repo, commit string) (string, error) {
	out, err := robotV4GitOutput(repo, "rev-parse", commit+"^{tree}")
	return strings.TrimSpace(out), err
}

// gatePassCovers: the record ran every suite the selection would run now.
func gatePassCovers(rec gatePassRecord, sel gateSelection) bool {
	if rec.Partial {
		return false
	}
	if rec.Level == "full" {
		return true
	}
	ran := map[string]bool{}
	for _, s := range rec.Suites {
		ran[s] = true
	}
	for _, s := range sel.Suites {
		if !ran[s.Name] {
			return false
		}
	}
	return true
}

// gateFindPass returns the newest matching passed gate, if any.
func gateFindPass(cell, commit, tree string, sel gateSelection) (gatePassRecord, bool) {
	file, err := gatePassFile()
	if err != nil {
		return gatePassRecord{}, false
	}
	f, err := os.Open(file)
	if err != nil {
		return gatePassRecord{}, false
	}
	defer f.Close()
	digest := gateDefinitionDigest()
	var found gatePassRecord
	ok := false
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 0, 1<<20), 16<<20)
	for sc.Scan() {
		var rec gatePassRecord
		if json.Unmarshal(sc.Bytes(), &rec) != nil {
			continue
		}
		if rec.Cell == cell && rec.Commit == commit && rec.Tree == tree && rec.Gate == digest && gatePassCovers(rec, sel) {
			found, ok = rec, true
		}
	}
	return found, ok
}

// gateSuccessfulSubset accepts only explicit, completed results for selected
// suites. It deliberately excludes known-but-failing suites as well as missing,
// duplicate, unparsed and unselected results. A truncated/cancelled session is
// not a completed gate and contributes no new coverage.
func gateSuccessfulSubset(sel gateSelection, result []byte) (gateSelection, []cellDeployGateStep) {
	subset := gateSelection{Level: sel.Level}
	var gate struct {
		Status string            `json:"status"`
		Steps  []json.RawMessage `json:"steps"`
	}
	if json.Unmarshal(result, &gate) != nil || gate.Status != "done" {
		return subset, nil
	}
	byName := map[string][]json.RawMessage{}
	for _, raw := range gate.Steps {
		var name struct {
			Step string `json:"step"`
		}
		if json.Unmarshal(raw, &name) != nil {
			return subset, nil
		}
		byName[name.Step] = append(byName[name.Step], raw)
	}
	var steps []cellDeployGateStep
	for _, suite := range sel.Suites {
		raw := byName[suite.Name]
		if len(raw) != 1 {
			continue
		}
		var explicit struct {
			RC       *int      `json:"rc"`
			Failures *[]string `json:"failures"`
		}
		var step cellDeployGateStep
		if json.Unmarshal(raw[0], &explicit) != nil || explicit.RC == nil || *explicit.RC != 0 ||
			explicit.Failures == nil || len(*explicit.Failures) != 0 || json.Unmarshal(raw[0], &step) != nil {
			continue
		}
		subset.Suites = append(subset.Suites, suite)
		steps = append(steps, step)
	}
	return subset, steps
}

// gateRecordPass appends a passed gate, or an explicitly partial set of suite
// passes. The caller must leave a failed gate failed and preserve its journal.
func gateRecordPass(rec gatePassRecord) error {
	if rec.Partial && rec.Level != "partial" {
		rec.AttemptLevel, rec.Level = rec.Level, "partial"
	}
	file, err := gatePassFile()
	if err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(file), 0o700); err != nil {
		return err
	}
	if rec.At == "" {
		rec.At = time.Now().UTC().Format(time.RFC3339)
	}
	if rec.Gate == "" {
		rec.Gate = gateDefinitionDigest()
	}
	if rec.SuiteDefinitions == nil {
		rec.SuiteDefinitions = map[string]string{}
		for _, name := range rec.Suites {
			if s, ok := gateSuiteByName(name); ok {
				rec.SuiteDefinitions[name] = gateSuiteDefinitionDigest(s)
			}
		}
	}
	raw, err := json.Marshal(rec)
	if err != nil {
		return err
	}
	f, err := os.OpenFile(file, os.O_CREATE|os.O_APPEND|os.O_WRONLY, 0o600)
	if err != nil {
		return err
	}
	defer f.Close()
	_, err = f.Write(append(raw, '\n'))
	return err
}
