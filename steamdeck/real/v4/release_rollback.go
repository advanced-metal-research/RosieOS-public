package main

import (
	"flag"
	"fmt"
	"io"
	"regexp"
	"strings"
	"time"
)

// `realtime-core rollback`: tier 3 rollback, previously a hand procedure
// (README "Tier 3"): stop both units (systemctl, never a signal by hand),
// `install.sh --rollback` to the previous release slot, restore the
// control.env backup that full deploy took of the configuration that ran
// before, start disarmed, verify against that configuration. It refuses an
// armed, enabled or leased controller and Stop dumps without a copy (the
// restart deletes /run/rosie-rt-core). Drive parameters the new release
// wrote and the old configuration omits keep their new values (A6-EC C13.10):
// revert those with explicit values, never by rolling back alone.

func cellDeployRollbackServoScript(toConfiguration string) string {
	return cellDeployRollbackServoScriptPair(toConfiguration, "", "")
}

func cellDeployRollbackServoScriptPair(toConfiguration, fromPair, toPair string) string {
	prefix := ""
	pairCheck := ""
	backupSelect := `backup=$(sudo -n ls -1t "$H" 2> /dev/null | grep -E -- "-$PIN\.env$" | head -n 1)
[ -n "$backup" ] || { printf 'rollback: no control.env backup for configuration %s in %s; the previous release is selected and both units are stopped\n' "$PIN" "$H" >&2; exit 1; }
backup="$H/$backup"`
	if fromPair != "" {
		backup := releasePairPKIBackup(fromPair, toPair)
		prefix = "PAIR_BACKUP=" + shellQuote(backup) + "\n" +
			`test "$(sudo -n cat "$PAIR_BACKUP/pki/pair-id" 2>/dev/null)" = ` + shellQuote(fromPair) + ` || { echo 'rollback refused: old pair PKI backup is missing'; exit 1; }` + "\n" +
			`sudo -n grep -Fxq ` + shellQuote("ROSIE_RT_CONFIGURATION_SHA256="+toConfiguration) + ` "$PAIR_BACKUP/control.env" || { echo 'rollback refused: exact old control.env backup is missing'; exit 1; }` + "\n" +
			`sudo -n grep -Fxq ` + shellQuote("ROSIE_RT_PAIR_ID="+fromPair) + ` "$PAIR_BACKUP/control.env" || { echo 'rollback refused: old control.env pair differs'; exit 1; }` + "\n"
		backupSelect = `backup="$PAIR_BACKUP/control.env"`
		pairCheck = `sudo -n grep -Fxq ` + shellQuote("ROSIE_RT_PAIR_ID="+fromPair) + ` /etc/rosie-rt-core/control.env`
	}
	body := rtCoreRollbackCommand() + "\n"
	if fromPair != "" {
		body += releasePairPKIRestoreScript(fromPair, toPair) + "\n"
	}
	return prefix + body + rtCoreConfigReplace(`PIN=@PIN@; H=@HISTORY@/control-env
@BACKUP_SELECT@
sudo -n install -m 0640 -o root -g rosie-ctl "$backup" /etc/rosie-rt-core/control.env
sudo -n grep -Fxq "ROSIE_RT_CONFIGURATION_SHA256=$PIN" /etc/rosie-rt-core/control.env
@PAIR_CHECK@
printf 'rollback: release %s selected; control.env restored from %s\n' "$(readlink -f /opt/rosie-rt-core/current)" "$backup"
`, "@PIN@", shellQuote(toConfiguration), "@HISTORY@", rtCoreConfigHistoryDir, "@BACKUP_SELECT@", backupSelect, "@PAIR_CHECK@", pairCheck)
}

// cellDeployRollbackScript is the relay session: the servo rollback, then the
// start script for the configuration rolled back to.
func cellDeployRollbackScript(ctx cellDeployContext, toConfiguration string) (string, error) {
	return cellDeployRollbackScriptPair(ctx, toConfiguration, "")
}

func cellDeployRollbackScriptPair(ctx cellDeployContext, toConfiguration, fromPair string) (string, error) {
	c := ctx.RTCore
	c.ConfigurationSHA256 = toConfiguration
	if fromPair != "" {
		c.PairID = fromPair
	}
	start, err := rtCoreStartCommand(c)
	if err != nil {
		return "", err
	}
	return cellDeployRelayPrologue(ctx) + "trap '' HUP PIPE\n" +
		"maint_phase 'rolling rt-core back to the previous release'\nstep \"rollback servo host $SERVO to configuration " + toConfiguration[:12] + "\"\n" +
		"servos <<'ROSIE_RT_CORE_ROLLBACK' || { rc=$?; step \"rollback exit $rc: see the lines above; never SIGKILL rosie-rt-core\"; exit $rc; }\n" + cellDeployRollbackServoScriptPair(toConfiguration, fromPair, ctx.RTCore.PairID) + "\nROSIE_RT_CORE_ROLLBACK\n" +
		"maint_phase 'starting the previous release (stays disarmed)'\nstep \"rollback starting the previous release\"\n" +
		"servos <<'ROSIE_RT_CORE_START'\n" + start + "\nROSIE_RT_CORE_START\nrc=$?\nstep \"rollback start exit $rc\"\nexit $rc\n", nil
}

func runCellDeployRollback(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var to, restorePairFrom string
	var toAxes int
	var homeWait time.Duration
	var maint rtCoreMaintenanceFlags
	ctx, restore, err := cellDeploySetup("realtime-core rollback", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&to, "to-configuration", "", "the configuration sha256 that ran before the full deploy (its control.env backup is restored; required)")
		fs.StringVar(&restorePairFrom, "restore-pair-pki-from", "", "old pair ID whose root-owned PKI backup is restored before restart")
		fs.IntVar(&toAxes, "to-axes", 0, "prior controller's Describe axis count; required with --restore-pair-pki-from")
		fs.DurationVar(&homeWait, "home-wait", 20*time.Second, "wait this long for home_valid_mask to settle after the start")
		maint.register(fs, false)
	}, &common, true, "--to-configuration", "--restore-pair-pki-from", "--to-axes", "--home-wait", "--maintenance-eta", "--keep-maintenance")
	if err != nil {
		return err
	}
	defer restore()
	if err := maint.validate(); err != nil {
		return err
	}
	if !rtCoreHexSHA256.MatchString(to) {
		return fmt.Errorf("--to-configuration must be the lowercase 64-hex configuration that ran before (the previous Describe configuration_sha256)")
	}
	if restorePairFrom != "" && (!regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]*$`).MatchString(restorePairFrom) || restorePairFrom == ctx.RTCore.PairID) {
		return fmt.Errorf("--restore-pair-pki-from must name a valid old pair distinct from the manifest pair")
	}
	if (restorePairFrom != "" && (toAxes <= 0 || toAxes >= 64)) || (restorePairFrom == "" && toAxes != 0) {
		return fmt.Errorf("paired rollback requires --to-axes between 1 and 63 with --restore-pair-pki-from")
	}
	w, err := cellDeploySeconds("--home-wait", homeWait, 0, 10*time.Minute)
	if err != nil {
		return err
	}
	body, err := cellDeployRollbackScriptPair(ctx, to, restorePairFrom)
	if err != nil {
		return err
	}
	m := cellDeployNotice(ctx, "rollback", "rt-core rollback to the previous release", maint, rtCoreMaintenanceStartEstimate)
	m.IdleWait = 0
	script := rtCoreMaintenanceWrap(m, ctx.ServoTarget, "rolling rt-core back to the previous release; the controller restarts disarmed", body)
	back := ctx
	back.RTCore.ConfigurationSHA256 = to
	if restorePairFrom != "" {
		back.RTCore.PairID = restorePairFrom
		back.MachineAxes = toAxes
	}
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core rollback cell=%s servo=%s to_configuration=%s (dry run: nothing contacted)\n# 1. facts (read-only)\n%s\n# 2. rollback and start\n%s\n# 3. verify (read-only)\n%s\n",
			ctx.CellID, ctx.ServoTarget, to, cellDeployFactsScript(ctx, false), script, cellDeployVerifyScript(back, w))
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core rollback", common.Transport, stdout)
	if err != nil {
		return err
	}
	code, probe, result, refused, err := run.session("rollback facts", cellDeployFactsScript(ctx, false))
	if e := sessionOutcome("rollback facts", code, result, refused, err); e != nil {
		run.finish("refused", cellDeployExitRefused, e.Error())
		return e
	}
	facts, err := decodeCellDeployFacts(probe)
	if err != nil {
		run.finish("refused", cellDeployExitRefused, err.Error())
		return cellDeployExit(cellDeployExitRefused, "rollback facts unreadable: %v", err)
	}
	facts.markMacCopies(ctx.CellID)
	var refuse []string
	if facts.busy() {
		refuse = append(refuse, "controller armed, enabled or leased: disarm and release first")
	}
	if missing := facts.uncopiedDumps(); len(missing) > 0 {
		refuse = append(refuse, "stop dumps without a copy: "+strings.Join(missing, ", ")+"; run realtime-core copy-dumps first (the restart deletes them)")
	}
	if facts.Previous == "" {
		refuse = append(refuse, "no /opt/rosie-rt-core/previous release to roll back to")
	}
	if len(facts.DState) > 0 {
		refuse = append(refuse, fmt.Sprintf("rosie processes in D/Z state %v: a stop may hang in an IgH call; ask the owner (never SIGKILL rosie-rt-core)", facts.DState))
	}
	if len(refuse) > 0 {
		run.report(refuse, nil)
		run.finish("refused", cellDeployExitRefused, strings.Join(refuse, "; "))
		return cellDeployExit(cellDeployExitRefused, "rollback refused (nothing was changed):\n  - %s", strings.Join(refuse, "\n  - "))
	}
	run.p.Step("rollback", fmt.Sprintf("current %s -> previous %s, configuration %.12s; drive parameters the new configuration wrote and the old one omits keep their values (revert explicitly)", facts.Current, facts.Previous, to))
	code, _, result, refused, err = run.session("rollback", script)
	if e := sessionOutcome("rollback", code, result, refused, err); e != nil {
		run.finish("failed", code, e.Error())
		return e
	}
	return cellDeployVerifySession(run, back, w, "")
}
