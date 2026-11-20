package main

import (
	"strings"
	"testing"
)

func TestReleaseDriveSettingsBindTargetToVerifiedReadbacks(t *testing.T) {
	target := rtCoreCompiledFiles{Files: map[string]string{
		"axes.conf": "axis=0\nstartup_sdos=mode|u16|8192|8|1|power_cycle\naxis=1\nstartup_sdos=mode|u16|8192|8|1|power_cycle\n",
		"argv.json": `["--slave-positions","0,1"]`,
	}}
	facts := releaseProbeFacts{cellDeployFacts: cellDeployFacts{DriveReadbacks: []cellDeployDriveReadback{
		{Axis: 0, SlavePosition: 0, Key: "mode", Type: "u16", Index: 8192, Subindex: 8, Readback: 4, ReadbackValid: true, Verified: true},
		{Axis: 1, SlavePosition: 1, Key: "mode", Type: "u16", Index: 8192, Subindex: 8, Readback: 1, ReadbackValid: true, Verified: true},
	}}}
	settings, err := releaseDriveSettings(target, facts)
	if err != nil {
		t.Fatal(err)
	}
	if len(settings) != 2 || settings[0].Before != 4 || settings[0].After != 1 || settings[1].Before != 1 || settings[1].After != 1 {
		t.Fatalf("settings = %+v", settings)
	}
	facts.DriveReadbacks = append(facts.DriveReadbacks, facts.DriveReadbacks[0])
	if _, err := releaseDriveSettings(target, facts); err == nil || !strings.Contains(err.Error(), "ambiguous readbacks") {
		t.Fatalf("duplicate evidence = %v", err)
	}
}

func TestReleaseDriveSettingsRejectAmbiguousTarget(t *testing.T) {
	target := rtCoreCompiledFiles{Files: map[string]string{
		"axes.conf": "axis=0\nstartup_sdos=mode|u16|8192|8|1|power_cycle\naxis=1\nstartup_sdos=mode|u16|8192|8|1|power_cycle\n",
		"argv.json": `["--slave-positions","0,0"]`,
	}}
	facts := releaseProbeFacts{cellDeployFacts: cellDeployFacts{DriveReadbacks: []cellDeployDriveReadback{
		{Axis: 0, SlavePosition: 0, Key: "mode", Type: "u16", Index: 8192, Subindex: 8, Readback: 4, ReadbackValid: true, Verified: true},
		{Axis: 1, SlavePosition: 0, Key: "mode", Type: "u16", Index: 8192, Subindex: 8, Readback: 4, ReadbackValid: true, Verified: true},
	}}}
	if _, err := releaseDriveSettings(target, facts); err == nil || !strings.Contains(err.Error(), "repeats object") {
		t.Fatalf("duplicate target = %v", err)
	}
	facts.DriveReadbacks[0].Verified = false
	target.Files["argv.json"] = `["--slave-positions","0,1"]`
	if _, err := releaseDriveSettings(target, facts); err == nil || !strings.Contains(err.Error(), "no verified readback") {
		t.Fatalf("unverified evidence = %v", err)
	}
}

func TestReleaseDriveModeTransitionSettingsRejectsOtherChanges(t *testing.T) {
	settings := []releaseDriveSetting{{Axis: 0, SlavePosition: 2, Key: "a6ec_encoder_position_tracking_mode", Type: "u16", Index: 0x2000, Subindex: 8, Before: 4, After: 1}}
	if slaves, err := releaseDriveModeTransitionSettings(settings); err != nil || len(slaves) != 1 || slaves[0] != 2 {
		t.Fatalf("supported transition: slaves=%v err=%v", slaves, err)
	}
	settings = append(settings, releaseDriveSetting{Axis: 1, SlavePosition: 3, Key: "a6ec_encoder_position_tracking_mode", Type: "u16", Index: 0x2000, Subindex: 8, Before: 1, After: 4})
	if _, err := releaseDriveModeTransitionSettings(settings); err == nil || !strings.Contains(err.Error(), "does not support") {
		t.Fatalf("unreviewed reversal accepted: %v", err)
	}
}

func TestReleaseDriveModeScriptBindsInstalledIdentity(t *testing.T) {
	ctx := cellDeployContext{CellID: "cell-test", RelayHost: "relay", ServoTarget: "operator@servo"}
	script := releaseDriveModeScript(ctx, strings.Repeat("a", 40), strings.Repeat("b", 64), []int{0, 5}, "apply")
	for _, want := range []string{"/opt/rosie-rt-core/" + strings.Repeat("a", 40), "ROSIE_RT_CONFIGURATION_SHA256=" + strings.Repeat("b", 64), "--slave 0 --phase apply --direction to-linear", "--slave 5 --phase apply --direction to-linear"} {
		if !strings.Contains(script, want) {
			t.Errorf("script missing %q", want)
		}
	}
	if strings.Count(script, "drive-mode-transition --slave") != 2 {
		t.Errorf("unexpected drive count: %s", script)
	}
}

func TestReleaseDrivePreInstallRetryRequiresOldLiveState(t *testing.T) {
	const oldCommit = "0123456789012345678901234567890123456789"
	const nextCommit = "abcdefabcdefabcdefabcdefabcdefabcdefabcd"
	const oldPin = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	saved := releaseState{Commit: nextCommit, FromRTCommit: oldCommit, FromConfiguration: oldPin, FromOLPCommit: oldCommit,
		DriveTransitionPhase: "applying", DriveTransitionRunID: "drive-transition-cell-test", DriveTransitionSlaves: []int{0, 1}}
	facts := releaseProbeFacts{cellDeployFacts: cellDeployFacts{Current: "/opt/rosie-rt-core/" + oldCommit, ControlEnvPin: oldPin,
		Units: map[string]cellDeployUnitFacts{"rosie-rt-core.service": {Active: "active"}, "rosie-rt-control.service": {Active: "active"}},
		DriveReadbacks: []cellDeployDriveReadback{
			{SlavePosition: 0, Key: "a6ec_encoder_position_tracking_mode", Type: "u16", Index: 0x2000, Subindex: 8, Readback: 4, ReadbackValid: true, Verified: true},
			{SlavePosition: 1, Key: "a6ec_encoder_position_tracking_mode", Type: "u16", Index: 0x2000, Subindex: 8, Readback: 4, ReadbackValid: true, Verified: true},
		}}, OLPHead: oldCommit}
	if !releaseDrivePreInstallRetry(saved, facts, nextCommit) {
		t.Fatal("unchanged pre-install state must permit a fresh prepare")
	}
	saved.DriveTransitionPhase = "preparing"
	if !releaseDrivePreInstallRetry(saved, facts, nextCommit) {
		t.Fatal("new preparing phase must permit a fresh prepare")
	}
	for _, mutate := range []func(*releaseState, *releaseProbeFacts){
		func(s *releaseState, _ *releaseProbeFacts) { s.DriveTransitionPackageSHA256 = "installed" },
		func(_ *releaseState, f *releaseProbeFacts) { f.MaintenanceRunID = "other" },
		func(_ *releaseState, f *releaseProbeFacts) {
			f.Units["rosie-rt-core.service"] = cellDeployUnitFacts{Active: "inactive"}
		},
		func(_ *releaseState, f *releaseProbeFacts) { f.DriveReadbacks[0].Readback = 1 },
		func(_ *releaseState, f *releaseProbeFacts) { f.DriveReadbacks[0].Verified = false },
		func(_ *releaseState, f *releaseProbeFacts) { f.Current = "/opt/rosie-rt-core/" + nextCommit },
	} {
		s := saved
		f := facts
		f.Units = map[string]cellDeployUnitFacts{"rosie-rt-core.service": facts.Units["rosie-rt-core.service"], "rosie-rt-control.service": facts.Units["rosie-rt-control.service"]}
		f.DriveReadbacks = append([]cellDeployDriveReadback(nil), facts.DriveReadbacks...)
		mutate(&s, &f)
		if releaseDrivePreInstallRetry(s, f, nextCommit) {
			t.Fatalf("changed or partly installed state accepted: state=%+v facts=%+v", s, f)
		}
	}
}
