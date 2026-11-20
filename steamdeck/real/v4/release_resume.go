package main

import (
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path"
	"path/filepath"
	"time"
)

// Resuming a full release. The first live release (one cell, 2026-10-07) stopped
// three times after its package was installed: "installed, units stopped"
// (the health gate refused igh_master), "started, not verified" (a stalled
// bring-up) and "verified, OLP not installed"; each time the cell was
// finished by hand with the documented steps. A release of the same commit
// now reads the cell and continues from the step it stopped at:
//
//   - the commit's rt-core release is current and both units are inactive:
//     start, verify, then olp install if the OLP tree is not the commit;
//   - it is current and both units run: verify (read-only), then olp
//     install if needed;
//   - anything else (one unit up, a unit failed): refused, nothing changed.
//
// Automatic rollback still needs the configuration that ran before the
// release. A full release records it on this machine before its first step
// (~/.rosie/deploy-journal/release-state/<cell>.json); a resume uses that
// record when its commit matches and the servo host's previous release is the
// one recorded, else --rollback-to-configuration, else it refuses unless
// --rollback never.

type releaseState struct {
	Cell                         string `json:"cell"`
	Commit                       string `json:"commit"`
	FromRTCommit                 string `json:"from_rt_commit"`
	FromConfiguration            string `json:"from_configuration"`
	FromPairID                   string `json:"from_pair_id,omitempty"`
	FromAxes                     int    `json:"from_axes,omitempty"`
	FromCapabilities             string `json:"from_capabilities"`
	FromOLPCommit                string `json:"from_olp_commit"`
	DriveTransitionPhase         string `json:"drive_transition_phase,omitempty"`
	DriveTransitionRunID         string `json:"drive_transition_run_id,omitempty"`
	DriveTransitionSlaves        []int  `json:"drive_transition_slaves,omitempty"`
	DriveTransitionPackageSHA256 string `json:"drive_transition_package_sha256,omitempty"`
	At                           string `json:"at"`
}

func releaseStateFile(cell string) (string, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	return filepath.Join(home, ".rosie", "deploy-journal", "release-state", cell+".json"), nil
}

func releaseSaveState(s releaseState) error {
	file, err := releaseStateFile(s.Cell)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(file), 0o700); err != nil {
		return err
	}
	if s.At == "" {
		s.At = time.Now().UTC().Format(time.RFC3339)
	}
	raw, err := json.MarshalIndent(s, "", "  ")
	if err != nil {
		return err
	}
	if err := os.WriteFile(file+".tmp", append(raw, '\n'), 0o600); err != nil {
		return err
	}
	return os.Rename(file+".tmp", file)
}

func releaseLoadState(cell string) (releaseState, bool) {
	file, err := releaseStateFile(cell)
	if err != nil {
		return releaseState{}, false
	}
	raw, err := os.ReadFile(file)
	if err != nil {
		return releaseState{}, false
	}
	var s releaseState
	if json.Unmarshal(raw, &s) != nil {
		return releaseState{}, false
	}
	return s, true
}

// releaseResumePoint reads where a full release of commit stopped: "start",
// "verify", or "" when the commit's rt-core release is not current (not a
// resume). refuse is set when the units are in neither resumable state.
func releaseResumePoint(f releaseProbeFacts, commit string) (from, why, refuse string) {
	if f.Current == "" || path.Base(f.Current) != commit {
		return "", "", ""
	}
	core, control := f.unitActive("rosie-rt-core.service"), f.unitActive("rosie-rt-control.service")
	switch {
	case core == "inactive" && control == "inactive":
		return "start", fmt.Sprintf("rt-core %s is installed with both units stopped (a release stopped after deploy)", cellDeployShort(commit)), ""
	case core == "active" && control == "active":
		return "verify", fmt.Sprintf("rt-core %s is installed and running; verifying it before anything else", cellDeployShort(commit)), ""
	}
	return "", "", fmt.Sprintf("rt-core %s is installed but rosie-rt-core is %q and rosie-rt-control %q: neither stopped nor running. Read journalctl -u rosie-rt-core -u rosie-rt-control on the servo host first; never SIGKILL rosie-rt-core", cellDeployShort(commit), core, control)
}

// releaseResumeSteps continues a full release from its resume point.
func releaseResumeSteps(from string, ctx cellDeployContext, commit, caps, test, transport string, olp bool, o releaseOptions) []releaseStep {
	byName := map[string]releaseStep{}
	for _, s := range releaseSteps(releaseTierFull, releasePlan{OLP: true}, ctx, commit, "", caps, test, transport, nil, o) {
		byName[s.Name] = s
	}
	// The package is installed already: build without --fetch-package.
	for _, s := range releaseSteps(releaseTierOLP, releasePlan{OLP: true}, ctx, commit, "", caps, test, transport, nil, o) {
		if s.Name == "build+test" {
			byName[s.Name] = s
		}
	}
	var steps []releaseStep
	if from == "start" {
		steps = append(steps, byName["start"])
	}
	steps = append(steps, byName["verify"])
	if olp {
		// olp install takes the relay's built tree of the commit: build skips
		// an existing one and reuses its recorded gate pass. It runs after
		// start and verify, so the cell is not down while it does.
		steps = append(steps, byName["build+test"], byName["olp install"])
	}
	return steps
}

// releaseResumeETA estimates a resume's steps. Its build+test runs the olp
// tier's build of an already built commit (releaseResumeSteps); a gate
// selection the records cannot give counts every suite.
func releaseResumeETA(ctx cellDeployContext, commit, test string, regate bool, steps []releaseStep) releaseETA {
	h := releaseJournalHistory(ctx.CellID)
	gate := releaseGatePlan{Level: test}
	if repo, err := repoRoot(); err == nil {
		if sel, err := releaseGateSelection(repo, ctx, commit, releaseTierOLP, test, nil, regate, h.Means); err == nil {
			gate = releaseGatePlanOf(repo, ctx, commit, sel, regate)
		} else if all, err := gateSelect("full", nil, true, h.Means); err == nil {
			gate = releaseGatePlanOf(repo, ctx, commit, all, regate)
			gate.Level = test
		}
	}
	return releaseEstimateSteps(h, steps, gate)
}

// releaseResumeRun finishes a full release from where it stopped, under one
// cell lock, with automatic rollback when the configuration that ran before
// is known.
func releaseResumeRun(run *cellDeployRun, ctx cellDeployContext, facts releaseProbeFacts, from, why, commit, caps string, common cellDeployCommonFlags, o releaseOptions, stdout, stderr io.Writer) error {
	in := releaseInstalled{RTCoreCommit: path.Base(facts.Previous), OLPCommit: facts.OLPHead}
	var source string
	var transition releaseState
	if o.DriveTransition == "resume" {
		var ok bool
		transition, ok = releaseLoadState(ctx.CellID)
		if !ok || transition.Commit != commit || transition.DriveTransitionPhase != "awaiting_power_cycle" || len(transition.DriveTransitionSlaves) == 0 || from != "start" ||
			transition.DriveTransitionRunID == "" || transition.DriveTransitionRunID != facts.MaintenanceRunID ||
			!rtCoreHexSHA256.MatchString(transition.DriveTransitionPackageSHA256) || facts.PackageManifestSHA256 != transition.DriveTransitionPackageSHA256 {
			return cellDeployExit(cellDeployExitRefused, "drive transition resume requires a matching prepared record, the new release installed, and both units stopped")
		}
		if o.AcceptConfigDrift != transition.FromConfiguration {
			return cellDeployExit(cellDeployExitRefused, "--accept-config-drift must name the previous running configuration %s", transition.FromConfiguration)
		}
		run.p.Step("power cycle", "operator asserted that the drives were physically power-cycled after the recorded mode writes; direct readback and controller startup still have to verify the result")
	}
	if s, ok := releaseLoadState(ctx.CellID); ok && s.Commit == commit && s.FromRTCommit == in.RTCoreCommit && rtCoreHexSHA256.MatchString(s.FromConfiguration) {
		in.Configuration, source = s.FromConfiguration, fmt.Sprintf("this machine's record of the release started at %s", s.At)
		in.Axes = s.FromAxes
		if s.FromPairID != "" {
			if o.PairMigrationFrom != "" && o.PairMigrationFrom != s.FromPairID {
				return cellDeployExit(cellDeployExitRefused, "resume pair migration disagrees with the saved previous pair %s", s.FromPairID)
			}
			o.PairMigrationFrom = s.FromPairID
		}
	}
	if o.RollbackTo != "" {
		in.Configuration, source = o.RollbackTo, "--rollback-to-configuration"
		if o.RollbackToAxes != 0 {
			in.Axes = o.RollbackToAxes
		}
	}
	if o.PairMigrationFrom != "" && (in.Axes < 1 || in.Axes > 63) {
		return cellDeployExit(cellDeployExitRefused, "paired resume needs the previous Describe axis count; use the saved release record or --rollback-to-configuration with --rollback-to-axes")
	}
	run.p.Step("resume", why)
	if o.Rollback == "auto" {
		if in.Configuration == "" {
			return cellDeployExit(cellDeployExitRefused, "resume refused: the configuration that ran before %s is unknown on this machine (no matching record in ~/.rosie/deploy-journal/release-state/%s.json for previous release %s), so an automatic rollback could not run. Pass --rollback-to-configuration <the previous Describe configuration_sha256> or --rollback never",
				cellDeployShort(commit), ctx.CellID, cellDeployShort(in.RTCoreCommit))
		}
		run.p.Step("resume", fmt.Sprintf("automatic rollback target: release %s with configuration %.12s (%s)", cellDeployShort(in.RTCoreCommit), in.Configuration, source))
	}
	test := firstNonEmpty(o.Test, releaseDefaultTestLevel(releaseTierFull))
	olp := facts.OLPHead != commit && o.CandidatePackage == ""
	steps := releaseResumeSteps(from, ctx, commit, caps, test, common.Transport, olp, o)
	if o.DriveTransition == "resume" {
		for i := range steps {
			if steps[i].Name != "verify" {
				continue
			}
			steps[i].Args = []string{"<internal drive-transition verify: Home invalidated>"}
			steps[i].run = func(_ []string, _, _ io.Writer) error {
				zero := uint64(0)
				_, err := cellDeployVerifySessionExpect(run, ctx, 0, cellDeployVerifyExpect{
					Capabilities: caps, HomeMask: &zero, RequireHomeInvalidated: true, AllowAnchorRemoval: true,
				}, false)
				return err
			}
		}
	}
	for i, s := range steps {
		run.p.Step("plan", fmt.Sprintf("%d. %s", i+1, s.line()))
	}
	eta := releaseResumeETA(ctx, commit, test, o.Regate, steps)
	for _, line := range eta.lines() {
		run.p.Step("eta", line)
	}
	if o.Plan {
		run.p.Step("done", "--plan: nothing changed")
		run.finish("ok", 0, "plan resume from "+from)
		return nil
	}
	unlock, err := releaseLock(run, ctx, common.Transport, commit, releaseTierFull, eta.remaining(0), o.Wait)
	if err != nil {
		run.finish("refused", deployExitCode(err), err.Error())
		return err
	}
	if o.CandidatePackage != "" {
		if err := releaseConfirmLockedProbe(run, releaseProbeScript(ctx), facts, false); err != nil {
			unlock()
			run.finish("refused", deployExitCode(err), err.Error())
			return err
		}
	}
	if o.DriveTransition == "resume" {
		if err := releaseConfirmLockedProbe(run, releaseProbeScript(ctx), facts, false); err != nil {
			unlock()
			run.finish("refused", deployExitCode(err), err.Error())
			return err
		}
		code, _, result, refused, verifyErr := run.session("drive mode verify", releaseDriveModeScript(ctx, commit, ctx.RTCore.ConfigurationSHA256, transition.DriveTransitionSlaves, "verify"))
		if err := sessionOutcome("drive mode verify", code, result, refused, verifyErr); err != nil {
			unlock()
			run.finish("refused", deployExitCode(err), err.Error())
			return err
		}
	}
	err = releaseRunSteps(run, ctx, steps, releasePlan{Tier: releaseTierFull, OLP: olp}, in, o, eta, stdout, stderr)
	unlock()
	if err != nil {
		run.finish("failed", deployExitCode(err), err.Error())
		return err
	}
	if o.DriveTransition == "resume" {
		transition.DriveTransitionPhase = "verified"
		if err := releaseSaveState(transition); err != nil {
			run.finish("failed", deployExitCode(err), err.Error())
			return fmt.Errorf("release verified, but transition record could not be closed; maintenance stays up: %w", err)
		}
		run.p.Step("done", "paired release verified disarmed; operator maintenance stays up until a separate checked clear and OLP select. Home must be requalified before motion.")
		run.finish("ok", 0, "drive transition verified, maintenance retained")
		return nil
	}
	if releaseHasStep(steps, "olp install") {
		err := releaseSelect(run, ctx, ctx.DeploymentDir(commit))
		if err != nil {
			if o.PairMigrationFrom != "" {
				err = fmt.Errorf("paired release resumed, but post-notice OLP selection failed (%v); do not use the cell until controller and OLP identities are checked", err)
				run.finish("failed", deployExitCode(err), err.Error())
				return err
			}
			err = fmt.Errorf("resumed %s, but the new OLP's Describe-only select failed after the notice cleared (%v); olp status, then olp rollback if it persists", cellDeployShort(commit), err)
			run.finish("failed", deployExitCode(err), err.Error())
			return err
		}
	}
	run.p.Step("done", fmt.Sprintf("resumed and released %s on %s (from %s); operators re-select the cell", cellDeployShort(commit), ctx.CellID, from))
	run.finish("ok", 0, "resume "+from)
	return nil
}
