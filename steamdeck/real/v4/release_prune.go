package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"regexp"
	"strings"
)

// `realtime-core prune`: the servo host keeps every release ever installed
// under /opt/rosie-rt-core (each ~60 MB; one relay held six rt-core packages
// on 2026-10-07). The installed release's own `install.sh --prune` deletes all
// but the current and previous releases and the pinned known-good ones
// (/opt/rosie-rt-core/pinned; --pin adds one). It never touches units or /etc,
// so it runs with the controller up. A full release runs it after the new
// release verified; a release whose install.sh predates --prune reports
// "unsupported" and changes nothing. A failed prune never fails a release.

var releasePruneDone = regexp.MustCompile(`prune status=done removed=(\d+) kept=(\d+) freed_kb=(\d+)`)

func cellDeployPruneScript(ctx cellDeployContext, pins []string) string {
	args := "--prune"
	for _, p := range pins {
		args += " --pin " + p
	}
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`step "prune servo host $SERVO: install.sh @ARGS@"
out=$(servo_once 'I=/opt/rosie-rt-core/current/host/install.sh
if ! sudo -n grep -q -- "--prune" "$I" 2> /dev/null; then echo "prune unsupported by the installed release"; exit 0; fi
sudo -n bash "$I" @ARGS@' 2>&1); rc=$?
printf '%s\n' "$out" | sed 's/^/  /'
case "$out" in
  *"prune unsupported"*) printf '@@result {"status":"unsupported"}\n'; exit 0 ;;
esac
[ "$rc" = 0 ] || { printf '@@result {"status":"failed","rc":%s}\n' "$rc"; exit 1; }
line=$(printf '%s\n' "$out" | grep -E 'prune status=done' | tail -n 1)
printf '@@result {"status":"ok","summary":"%s"}\n' "$line"
`, "@ARGS@", args)
}

func runCellDeployPrune(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var pins string
	var wait cellLockWait
	ctx, restore, err := cellDeploySetup("realtime-core prune", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&pins, "pin", "", "comma-separated 40-hex releases to keep as known-good (added to /opt/rosie-rt-core/pinned)")
		wait.register(fs)
	}, &common, true, "--pin", "--idle-wait", "--quiet", "--interval")
	if err != nil {
		return err
	}
	defer restore()
	if err := wait.validate(); err != nil {
		return err
	}
	var list []string
	for _, p := range strings.Split(pins, ",") {
		if p = strings.TrimSpace(p); p == "" {
			continue
		}
		if !cellDeployHexCommit.MatchString(p) {
			return fmt.Errorf("--pin takes 40-hex release revisions, got %q", p)
		}
		list = append(list, p)
	}
	script := cellDeployPruneScript(ctx, list)
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core prune cell=%s servo=%s pins=%v (dry run: nothing contacted)\n# under the cell lock (realtime-core lock)\n%s\n", ctx.CellID, ctx.ServoTarget, list, script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core prune", common.Transport, stdout)
	if err != nil {
		return err
	}
	estimate := releaseJournalHistory(ctx.CellID).stepEstimate("prune", releaseGatePlan{})
	unlock, err := cellLockForRun(run.p, ctx, common.Transport, "prune", "pruning old rt-core releases on the servo host", releaseETAWithMargin(estimate.Expected), wait)
	if err != nil {
		run.finish("refused", deployExitCode(err), err.Error())
		return err
	}
	_, stopProgress := noticeRunProgress(run.p, ctx.CellID, "prune")
	err = cellDeployPruneSession(run, ctx, list)
	stopProgress()
	unlock()
	if err != nil {
		run.finish("failed", deployExitCode(err), err.Error())
		return err
	}
	run.finish("ok", 0, "pruned")
	return nil
}

// cellDeployPruneSession runs the prune and reports what it freed.
func cellDeployPruneSession(run *cellDeployRun, ctx cellDeployContext, pins []string) error {
	code, _, result, refused, err := run.session("prune", cellDeployPruneScript(ctx, pins))
	if e := sessionOutcome("prune", code, result, refused, err); e != nil {
		return e
	}
	var r struct {
		Status, Summary string
	}
	_ = json.Unmarshal(result, &r)
	switch {
	case r.Status == "unsupported":
		run.p.Step("prune", "the installed release's install.sh has no --prune (a release before ffc7a644): nothing removed")
	case releasePruneDone.MatchString(r.Summary):
		m := releasePruneDone.FindStringSubmatch(r.Summary)
		run.p.Step("prune", fmt.Sprintf("removed %s release(s), kept %s, freed %s KiB", m[1], m[2], m[3]))
	default:
		run.p.Step("prune", "done: "+firstNonEmpty(r.Summary, string(result)))
	}
	return nil
}
