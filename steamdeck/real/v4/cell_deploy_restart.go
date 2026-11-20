package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"os"
	"path"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"time"
)

// `realtime-core restart`: a controlled restart of the controller a cell
// already runs, release and configuration unchanged. On 2026-10-11 it was
// done twice by hand to clear a latched A6-EC Er41.1 after a locked-rotor
// stall (one arm's J5 14:48, another's J2 20:43; DEVLOG 14:50 and 20:47): copy-dumps,
// `lock --on servo -- systemctl stop rosie-rt-control && systemctl stop
// rosie-rt-core`, start, verify. On the second arm `start` refused because the
// installed pin (66c7300b, which no commit pins) was not the manifest's, and
// the agent ran start's script body by hand with the installed pin. This
// command is that sequence with its checks:
//
//  1. the cell lock for the whole run (its idle wait; both maintenance
//     notices up, arming refused from every client),
//  2. copy-dumps (a restart deletes /run/rosie-rt-core and its Stop dumps),
//  3. read-only facts (the precheck script, which also records the anchor
//     files verify compares) and the refusals: armed, enabled or leased; OLP
//     session or operator activity; uncopied dumps; rosie threads stuck in D
//     state; a running configuration that is not the installed one; any
//     fault or drive alarm unless --clear-drive-alarms covers it,
//  4. one servo-host session under the configuration deploy's flock: the
//     same checks again just before the stop, `systemctl stop` of
//     rosie-rt-control then rosie-rt-core (the units' SIGTERM drain; never a
//     signal by hand), proof that both stopped with Result=success, then
//     `realtime-core start`'s own script with the INSTALLED pin (it still
//     refuses another pair, pair revision or a release that fails `rt-package
//     verify`),
//  5. verify against the configuration that ran before and the Home that
//     was valid before (restart never homes), reporting drive alarms and Home
//     before and after.
//
// Restart never changes the release or the configuration and never requires
// the manifest's pin: a cell whose installed pin no commit holds restarts on
// that pin, and the manifest pin is only reported. `realtime-core config` or
// `release` change configuration, from a commit.

// cellDeployExitStopUnclean: the stop did not end with Result=success on both
// units; nothing was started.
const cellDeployExitStopUnclean = 6

// cellDeployFaultDriveReadiness is execution_fault_reasons bit 7,
// drive_readiness (rt-core include/execution_policy.hpp FaultDriveReadiness):
// the reason a latched drive alarm raises, beside the axis's bit in
// safety_fault_mask (an arm's J5 Er41.1 2026-10-11: safety 16, execution 128).
const cellDeployFaultDriveReadiness = 1 << 7

// cellDeployRestartReasonMax bounds --reason: it is one line on the overlays
// and in the journals.
const cellDeployRestartReasonMax = 300

var cellDeployDriveProfileName = regexp.MustCompile(`^[a-z0-9_]+$`)

// cellDeployAxisAlarms is one axis's joint, drive profile and the alarms that
// profile's fault_reset list names resettable, keyed by 0x603F<<32 | 0x203F:
// the list rt-core's operator Reset uses (rt-core/docs/drives/a6ec/README.md
// "Operator reset of resettable drive alarms").
type cellDeployAxisAlarms struct {
	Joint, Profile string
	Resettable     map[uint64]string
}

// cellDeployResettableAlarms reads this checkout's machine config, robot
// definition and drive profiles for the cell. Nothing about alarms is coded
// here: a profile without fault_reset makes every alarm not resettable.
func cellDeployResettableAlarms(repo string, ctx cellDeployContext) ([]cellDeployAxisAlarms, error) {
	read := func(rel string, v any) error {
		raw, err := os.ReadFile(filepath.Join(repo, filepath.FromSlash(rel)))
		if err != nil {
			return err
		}
		if err := json.Unmarshal(raw, v); err != nil {
			return fmt.Errorf("%s: %w", rel, err)
		}
		return nil
	}
	var machine struct {
		Robot struct {
			RobotDescription struct {
				Path string `json:"path"`
			} `json:"robot_description"`
		} `json:"robot"`
		Axes []struct {
			RobotJoint string `json:"robot_joint"`
		} `json:"axes"`
	}
	if err := read(ctx.RTCore.MachineConfig, &machine); err != nil {
		return nil, err
	}
	var definition struct {
		Defaults struct {
			DriveProfile string `json:"drive_profile"`
		} `json:"defaults"`
		Joints []struct {
			Name         string  `json:"name"`
			DriveProfile *string `json:"drive_profile"`
		} `json:"joints"`
	}
	if err := read(path.Join(machine.Robot.RobotDescription.Path, "rtcore_definition.json"), &definition); err != nil {
		return nil, err
	}
	profiles := map[string]map[uint64]string{}
	out := make([]cellDeployAxisAlarms, len(machine.Axes))
	for i, axis := range machine.Axes {
		name := definition.Defaults.DriveProfile
		for _, j := range definition.Joints {
			if j.Name == axis.RobotJoint && j.DriveProfile != nil {
				name = *j.DriveProfile
			}
		}
		if !cellDeployDriveProfileName.MatchString(name) {
			return nil, fmt.Errorf("axis %d (%s): drive profile %q is not a profile name", i, axis.RobotJoint, name)
		}
		if _, ok := profiles[name]; !ok {
			var raw struct {
				FaultReset struct {
					Resettable []struct {
						Code      string `json:"code"`
						ErrorCode string `json:"error_code"`
						AuxCode   string `json:"aux_code"`
					} `json:"resettable_alarms"`
				} `json:"fault_reset"`
			}
			if err := read("rt-core/config/drives/"+name+".json", &raw); err != nil {
				return nil, err
			}
			set := map[uint64]string{}
			for _, a := range raw.FaultReset.Resettable {
				e, err1 := strconv.ParseUint(a.ErrorCode, 0, 32)
				x, err2 := strconv.ParseUint(a.AuxCode, 0, 32)
				if err1 != nil || err2 != nil {
					return nil, fmt.Errorf("drive profile %s: fault_reset alarm %s has codes %q/%q", name, a.Code, a.ErrorCode, a.AuxCode)
				}
				set[e<<32|x] = a.Code
			}
			profiles[name] = set
		}
		out[i] = cellDeployAxisAlarms{Joint: axis.RobotJoint, Profile: name, Resettable: profiles[name]}
	}
	return out, nil
}

// cellDeployRestartPlan is what the facts decided.
type cellDeployRestartPlan struct {
	// Resume: both units were already stopped (a restart whose start failed,
	// or a hand stop); restart then only starts them.
	Resume bool
	// Pin and Release are the installed configuration (control.env) and the
	// release /opt/rosie-rt-core/current resolves to; both are started again.
	Pin, Release string
	// HomeBefore is home_valid_mask before the stop (nil when resuming: then
	// verify expects every axis).
	HomeBefore *uint64
	// Alarms are the drive alarms before the stop; Cleared describes those
	// the owner's --clear-drive-alarms covers.
	Alarms  []cellDeployDriveAlarm
	Cleared []string
}

func (a cellDeployAxisAlarms) label(axis int) string {
	if a.Joint == "" {
		return fmt.Sprintf("axis %d", axis)
	}
	return fmt.Sprintf("%s (axis %d)", a.Joint, axis)
}

func cellDeployAlarmLine(axes []cellDeployAxisAlarms, a cellDeployDriveAlarm) string {
	if a.Axis >= 0 && a.Axis < len(axes) {
		return axes[a.Axis].Joint + " " + a.String()
	}
	return a.String()
}

// cellDeployClassifyAlarm: resettable only by the exact 0x603F/0x203F pair in
// the axis's profile; an unread 0x203F is not resettable (rt-core's rule).
func cellDeployClassifyAlarm(axes []cellDeployAxisAlarms, a cellDeployDriveAlarm) (bool, string) {
	if a.Axis < 0 || a.Axis >= len(axes) {
		return false, "an axis the machine config does not have"
	}
	if !a.AuxValid {
		return false, "its 0x203F detail is unread, so its class is unknown"
	}
	if code, ok := axes[a.Axis].Resettable[uint64(a.ErrorCode)<<32|uint64(a.AuxCode)]; ok {
		return true, "resettable " + code + " in drive profile " + axes[a.Axis].Profile + " (fault_reset)"
	}
	return false, "not in drive profile " + axes[a.Axis].Profile + "'s fault_reset list (the manual: power-cycle and service the drive)"
}

// cellDeployEvaluateRestart decides from the read-only facts: refuse (nothing
// changed), or the plan. A restart can clear a latched drive alarm at
// bring-up, which hides a hardware failure unless the owner inspected its
// cause (CLAUDE.md: never route around a drive or hardware failure): any
// alarm refuses without --clear-drive-alarms, an alarm the drive profile
// does not list resettable refuses even with it, and a fault no such alarm
// explains always refuses (the operator Reset clears latches; restart is not
// a fault-clearing path). The one exception is --clear-rehome-faults: faults
// whose recovery is a new Home on every axis they latched, on axes whose Home
// is already lost, with no drive alarm. Reset and the OLP's Home refuse that
// combination (one arm, 2026-11-13: coordinate_reference with drive_readiness on
// all six axes after a bulk SDO scan disturbed the coordinate sampling), and
// a restart cannot lose a Home that is already gone: startup re-checks the
// saved anchors against the drives, and an operator homes every axis they do
// not restore.
func cellDeployEvaluateRestart(ctx cellDeployContext, f cellDeployFacts, axes []cellDeployAxisAlarms, consent cellDeployRestartConsent) (plan cellDeployRestartPlan, refuse, notes []string) {
	clearAlarms := consent.DriveAlarms
	for _, e := range f.Errors {
		refuse = append(refuse, "servo host read failed: "+e)
	}
	core, control := f.unitActive("rosie-rt-core.service"), f.unitActive("rosie-rt-control.service")
	switch {
	case core == "active" && control == "active":
	case core == "inactive" && control == "inactive":
		plan.Resume = true
		notes = append(notes, "both units are stopped: restart only starts the installed release (resume); the drive alarms before that stop cannot be read, the report after the start shows them")
	default:
		refuse = append(refuse, fmt.Sprintf("rosie-rt-core is %q and rosie-rt-control %q: restart needs both running or both stopped; read their journals first (never SIGKILL rosie-rt-core)", core, control))
	}
	plan.Pin, plan.Release = f.ControlEnvPin, f.Current
	switch {
	case !rtCoreHexSHA256.MatchString(plan.Pin):
		refuse = append(refuse, fmt.Sprintf("control.env pins %q: no installed configuration to restart", plan.Pin))
	case plan.Pin != ctx.RTCore.ConfigurationSHA256:
		notes = append(notes, fmt.Sprintf("restart keeps the installed configuration %.12s (control.env); the manifest pins %.12s. Restart never changes the release or the configuration: release or config does, from a commit", plan.Pin, ctx.RTCore.ConfigurationSHA256))
	}
	if plan.Release == "" {
		refuse = append(refuse, "no /opt/rosie-rt-core/current release")
	}
	if len(f.DState) > 0 {
		refuse = append(refuse, fmt.Sprintf("rosie processes in D/Z state %v: a stop may hang in an IgH call; read /proc/<pid>/task/*/stack and ask the owner (never SIGKILL rosie-rt-core)", f.DState))
	}
	if missing := f.uncopiedDumps(); len(missing) > 0 {
		refuse = append(refuse, fmt.Sprintf("stop dump(s) without a hash-verified copy: %s; the restart deletes %s (copy-dumps ran first: read its lines)", strings.Join(missing, ", "), rtCoreConfigTelemetryDir))
	}
	if o := f.OLP; o != nil {
		if o.Session == "playing" || o.Session == "paused" || o.Held {
			refuse = append(refuse, fmt.Sprintf("OLP session %q held=%t: an operator is running the cell", o.Session, o.Held))
		}
		if o.RecentActivity != 0 {
			refuse = append(refuse, fmt.Sprintf("OLP journal shows %d operator action line(s) in the quiet window (or could not be read: -1)", o.RecentActivity))
		}
	}
	if clearAlarms && plan.Resume {
		notes = append(notes, "--clear-drive-alarms: the units are stopped, so no alarm can be read before the start")
	}
	if plan.Resume {
		return plan, refuse, notes
	}
	s := f.Status
	if s == nil {
		return plan, append(refuse, "controller status unavailable"), notes
	}
	full := ctx.FullAxisMask()
	if f.busy() {
		refuse = append(refuse, fmt.Sprintf("controller armed=%d axis_enable_mask=%d lease_valid=%d: an operator holds the cell; finish, Stop, disarm and release", s.Armed, s.AxisEnableMask, s.LeaseValid))
	}
	home := s.HomeValidMask
	plan.HomeBefore = &home
	notes = append(notes, fmt.Sprintf("home_valid_mask=%d of %d before the restart (restart restores it from the anchors, never homes)", home, full))
	if s.ConfigVerifiedMask != full {
		notes = append(notes, fmt.Sprintf("config_verified_mask=%d of %d before the restart", s.ConfigVerifiedMask, full))
	}
	switch d := f.Describe; {
	case d == nil:
		refuse = append(refuse, "Describe unavailable")
	case d.ConfigurationSHA256 != plan.Pin:
		refuse = append(refuse, fmt.Sprintf("the controller runs configuration %.12s but control.env pins %.12s: a configuration change is half-applied, and a restart would start one that is not running; use realtime-core config or rollback", d.ConfigurationSHA256, plan.Pin))
	case d.Backend != "ethercat":
		refuse = append(refuse, fmt.Sprintf("Describe backend %q, not ethercat", d.Backend))
	}
	plan.Alarms = s.DriveAlarms
	if len(s.Alarms) > 0 && len(s.DriveAlarms) == 0 {
		refuse = append(refuse, fmt.Sprintf("drive alarms %v without their 0x603F/0x203F codes: cannot classify them", s.Alarms))
	}
	var resettable uint32
	var lines, notResettable []string
	for _, a := range s.DriveAlarms {
		ok, why := cellDeployClassifyAlarm(axes, a)
		line := cellDeployAlarmLine(axes, a) + ": " + why
		lines = append(lines, line)
		if !ok {
			notResettable = append(notResettable, line)
			continue
		}
		resettable |= 1 << uint(a.Axis)
		plan.Cleared = append(plan.Cleared, line)
	}
	switch {
	case len(lines) > 0 && !clearAlarms:
		refuse = append(refuse, "drive alarm(s) present: "+strings.Join(lines, "; ")+". A restart can clear a latched drive alarm at bring-up, which hides a hardware failure: find and fix the cause at the drive first; when the owner has inspected it, run again with --clear-drive-alarms")
	case len(notResettable) > 0:
		refuse = append(refuse, "drive alarm(s) restart must not clear even with --clear-drive-alarms: "+strings.Join(notResettable, "; "))
	}
	var explainedSafety, explainedExecution uint32
	if len(notResettable) == 0 && resettable != 0 {
		explainedSafety, explainedExecution = resettable, cellDeployFaultDriveReadiness
	}
	rehomeSafety, rehomeExecution, rehomeNames := cellDeployRehomeBoundFaults(s)
	rehomeHint := ""
	if rehomeSafety != 0 {
		lost := cellDeployAxisNames(axes, rehomeSafety)
		switch {
		case !consent.RehomeFaults:
			rehomeHint = fmt.Sprintf("; %s need a new Home on %s (Home already lost, or still valid with positions trusted and audit-verified): when the owner has found their cause, run again with --clear-rehome-faults", strings.Join(rehomeNames, ", "), lost)
		case !consent.DriveAlarms && (len(s.Alarms) > 0 || len(s.DriveAlarms) > 0):
			// A bus drop-out latches both at once (one arm, 2026-11-13: a drive
			// power cycle raised ErC1.0/ErC2.0 on the others): resettable
			// alarms are --clear-drive-alarms' to accept, the rest refuse above.
			refuse = append(refuse, "--clear-rehome-faults: a drive alarm is present; when the owner has inspected it, add --clear-drive-alarms (resettable alarms only)")
		case uint32(s.HomeValidMask)&rehomeSafety&^s.VerifiedReferenceMask != 0:
			refuse = append(refuse, fmt.Sprintf("--clear-rehome-faults: Home is still valid on %s but its position is not trusted and audit-verified; restart would put a valid Home at risk",
				cellDeployAxisNames(axes, uint32(s.HomeValidMask)&rehomeSafety&^s.VerifiedReferenceMask)))
		default:
			explainedSafety |= rehomeSafety
			explainedExecution |= rehomeExecution
			if stale := uint32(s.HomeValidMask) & rehomeSafety; stale != 0 {
				// A one-cycle frame loss latches coordinate_reference with the
				// axes recorded untrusted, and that provenance never clears
				// although fresh evidence re-verifies them (issue #177; one arm
				// on 2026-11-13 after 6.12.91 kernel latency spikes). The fresh
				// process re-checks the saved anchors; verify then requires the
				// Home that was valid before, so a real reference loss fails.
				explainedExecution |= cellDeployConditionFaults(s, stale)
				plan.Cleared = append(plan.Cleared, fmt.Sprintf("%s on %s, whose Home is valid and whose positions are trusted and audit-verified (a stale latch, issue #177): the restart must restore that Home from the saved anchors or it fails",
					strings.Join(rehomeNames, ", "), cellDeployAxisNames(axes, stale)))
			}
			if lost := rehomeSafety &^ uint32(s.HomeValidMask); lost != 0 {
				plan.Cleared = append(plan.Cleared, fmt.Sprintf("%s on %s, whose Home is already lost: startup re-checks the saved anchors; an operator homes every joint they do not restore",
					strings.Join(rehomeNames, ", "), cellDeployAxisNames(axes, lost)))
			}
		}
	} else if consent.RehomeFaults {
		notes = append(notes, "--clear-rehome-faults: no fault that needs a new Home is latched; nothing to clear")
	}
	if s.SafetyFaultMask&^explainedSafety != 0 || s.ExecutionFaultReasons&^explainedExecution != 0 {
		refuse = append(refuse, fmt.Sprintf("faults safety_fault_mask=%d execution_fault_reasons=%d that no resettable drive alarm explains: restart is not a fault-clearing path; report the cause (the operator Reset clears a latched fault and never re-homes)%s", s.SafetyFaultMask, s.ExecutionFaultReasons, rehomeHint))
	}
	if clearAlarms && len(lines) == 0 {
		notes = append(notes, "--clear-drive-alarms: no drive alarm is present; nothing to clear")
	}
	return plan, refuse, notes
}

// cellDeployRestartServoScript runs on the servo host under the
// configuration deploy's flock (one controller change at a time). It ignores
// SIGHUP/SIGPIPE, so a dropped connection cannot leave it between stop and
// start. Restart mode re-checks release, pin, units, disarmed and the running
// configuration just before the stop, stops rosie-rt-control then
// rosie-rt-core with systemctl and requires Result=success on both: systemd
// sends SIGKILL only when rosie-rt-core's 15 s TimeoutStopSec runs out
// (Result=timeout), and a kill during a mailbox call leaves the EtherCAT
// master reserved until a reboot, so nothing is started then (exit 6). Then
// it runs `realtime-core start`'s script for the installed pin. Every step
// is a line in the configuration deploy's journal on the servo host.
func cellDeployRestartServoScript(ctx cellDeployContext, plan cellDeployRestartPlan, reason string) (string, error) {
	c := ctx.RTCore
	c.ConfigurationSHA256 = plan.Pin
	start, err := rtCoreStartCommand(c)
	if err != nil {
		return "", err
	}
	mode := "restart"
	if plan.Resume {
		mode = "resume"
	}
	return rtCoreConfigReplace(`set -uo pipefail
trap '' PIPE HUP
export LC_ALL=C NO_REAL_MOVEMENT=1
@STEP@
MODE=@MODE@; PIN=@PIN@; RELEASE=@RELEASE@; REASON=@REASON@
R=/opt/rosie-rt-core/current; SOCK=/run/rosie-rt-core/control.sock; ENV=/etc/rosie-rt-core/control.env
H=@HISTORY@; LOCK=@LOCK@
UNITS="rosie-rt-control.service rosie-rt-core.service"
refuse(){ step "restart refused: $*"; printf '@@refused %s\n' "$*"; exit 2; }
unit(){ systemctl show "$1" -p "$2" --value; }
journal(){ # $1 event, $2 detail; the configuration deploy's journal fields
  sudo -n install -d -m 0750 -o root -g root "$H" 2> /dev/null || return 0
  sudo -n test -e "$H/journal.jsonl" || sudo -n install -m 0640 -o root -g root /dev/null "$H/journal.jsonl"
  python3 -c 'import json, sys, time; e, d, rel, pin, why = sys.argv[1:6]; print(json.dumps({"time": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "event": e, "release": rel, "old": pin, "new": pin, "label": why, "git_head": "", "bytes_sha256": "", "detail": d}))' \
    "$1" "${2:-}" "$RELEASE" "$PIN" "$REASON" | sudo -n tee -a "$H/journal.jsonl" > /dev/null
}
[ -e "$LOCK" ] || : > "$LOCK" 2> /dev/null
exec 9< "$LOCK" || refuse "cannot open $LOCK"
flock -n 9 || refuse "a configuration deploy holds $LOCK on $(hostname)"
test "$(readlink -f "$R")" = "$RELEASE" || refuse "the installed release is $(readlink -f "$R") now, not $RELEASE"
sudo -n grep -qFx "ROSIE_RT_CONFIGURATION_SHA256=$PIN" "$ENV" || refuse "control.env no longer pins $PIN"
if [ "$MODE" = restart ]; then
  for u in $UNITS; do s=$(unit "$u" ActiveState); [ "$s" = active ] || refuse "$u is $s now, not active"; done
  st=$(sudo -n "$R/bin/rtctl" status --socket "$SOCK" --json 2> /dev/null) || refuse "controller status unavailable just before the stop"
  printf '%s' "$st" | python3 -c '@BUSY@import json, sys
c = json.load(sys.stdin); c = c.get("data", c)["core"]
sys.exit(1 if rosie_busy(c) else 0)' || refuse "controller armed, enabled or leased just before the stop: an operator holds the cell"
  d=$(sudo -n "$R/bin/rtctl" describe --socket "$SOCK" --json 2> /dev/null) || refuse "Describe unavailable just before the stop"
  printf '%s' "$d" | python3 -c 'import json, sys
d = json.load(sys.stdin); d = d.get("data", d)
sys.exit(0 if d.get("configuration_sha256") == sys.argv[1] else 1)' "$PIN" || refuse "the controller does not run $PIN"
  since=$(date '+%Y-%m-%d %H:%M:%S')
  journal restart-stop
  step "restart stop rosie-rt-control (systemctl stop: SIGTERM within the unit's own bound)"
  sudo -n systemctl stop rosie-rt-control.service; rc_control=$?
  step "restart stop rosie-rt-core (systemctl stop: SIGTERM, outputs inhibited, mailbox drain, master released; never a signal by hand)"
  sudo -n systemctl stop rosie-rt-core.service; rc_core=$?
  sudo -n journalctl -u rosie-rt-core --since "$since" --no-pager -o cat 2> /dev/null | grep -i 'shutdown' | tail -n 8 | while IFS= read -r l; do step "restart journal $l"; done
  bad=""
  for u in $UNITS; do
    l="$u ActiveState=$(unit "$u" ActiveState) Result=$(unit "$u" Result) ExecMainCode=$(unit "$u" ExecMainCode) ExecMainStatus=$(unit "$u" ExecMainStatus)"
    step "restart stopped $l"
    case "$l" in *" ActiveState=inactive Result=success "*) ;; *) bad="$bad; $l" ;; esac
  done
  if [ "$rc_control" != 0 ] || [ "$rc_core" != 0 ] || [ -n "$bad" ]; then
    journal restart-stop-unclean "systemctl stop exit $rc_control/$rc_core$bad"
    printf '%s\n' "STOP NOT CLEAN on $(hostname): systemctl stop exit rosie-rt-control=$rc_control rosie-rt-core=$rc_core$bad" \
      "Nothing was started. Result=timeout means systemd ended rosie-rt-core with SIGKILL after its stop bound; a kill during a mailbox call leaves the EtherCAT master reserved until a servo host reboot (owner approval)." \
      "Read the shutdown lines first: sudo -n journalctl -u rosie-rt-core --since '$since'. Never SIGKILL rosie-rt-core by hand." >&2
    exit 6
  fi
  journal restart-stopped
else
  for u in $UNITS; do s=$(unit "$u" ActiveState); [ "$s" = inactive ] || refuse "$u is $s now, not inactive"; done
  journal restart-resume "both units were stopped before this run"
fi
step "restart start the installed release with configuration ${PIN:0:12} (start refuses another pair or a release that fails verification)"
bash -s <<'ROSIE_RT_CORE_START'
@START@
ROSIE_RT_CORE_START
rc=$?
if [ "$rc" = 0 ]; then journal restart-started; else journal restart-start-failed "start exit $rc: both units stopped"; fi
exit "$rc"
`, "@STEP@", rtCoreConfigStepFunc, "@MODE@", mode, "@PIN@", shellQuote(plan.Pin), "@RELEASE@", shellQuote(plan.Release),
		"@REASON@", shellQuote(reason), "@HISTORY@", rtCoreConfigHistoryDir, "@LOCK@", rtCoreConfigLockPath, "@BUSY@", rtCoreBusyPy,
		"@START@", start), nil
}

// cellDeployRestartScript is the relay session around the servo script,
// under the run's maintenance notice (its phase; the lock's run owns it).
func cellDeployRestartScript(ctx cellDeployContext, m rtCoreMaintenance, plan cellDeployRestartPlan, reason string) (string, error) {
	servo, err := cellDeployRestartServoScript(ctx, plan, reason)
	if err != nil {
		return "", err
	}
	phase := fmt.Sprintf("restarting the controller: same release, configuration %.12s; it stays disarmed", plan.Pin)
	m.IdleWait = 0
	return rtCoreMaintenanceWrap(m, ctx.ServoTarget, phase, cellDeployRelayPrologue(ctx)+"trap '' HUP PIPE\n"+
		"step \"restart servo host $SERVO\"\n"+
		"servos <<'ROSIE_RT_CORE_RESTART'\n"+servo+"\nROSIE_RT_CORE_RESTART\nrc=$?\nstep \"restart exit $rc\"\nexit $rc\n"), nil
}

// cellDeployRestartNotice is the restart session's notice; built after the
// run lock is held, it updates that lock's notice and never clears it.
func cellDeployRestartNotice(ctx cellDeployContext, reason string) rtCoreMaintenance {
	return cellDeployNotice(ctx, "restart", "controlled rt-core restart ("+reason+")", rtCoreMaintenanceFlags{}, releaseStepDefaults["restart"])
}

func runCellDeployRestart(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var reason string
	var consent cellDeployRestartConsent
	var homeWait time.Duration
	var wait cellLockWait
	ctx, restore, err := cellDeploySetup("realtime-core restart", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&reason, "reason", "", "why the controller is restarted (required; shown on the overlays and kept in both journals)")
		fs.BoolVar(&consent.DriveAlarms, "clear-drive-alarms", false, "the owner inspected the cause of the drive alarm(s) present and accepts that the restart may clear them; only alarms the drive profile lists resettable")
		fs.BoolVar(&consent.RehomeFaults, "clear-rehome-faults", false, "the owner found the cause of latched faults that need a new Home on joints whose Home is already lost, and accepts that the restart may clear them; an operator then homes every joint the anchors do not restore")
		fs.DurationVar(&homeWait, "home-wait", rtCoreConfigHomeTimeout, "after the start, wait this long for the Home that was valid before to be restored")
		wait.register(fs)
	}, &common, true, "--reason", "--clear-drive-alarms", "--clear-rehome-faults", "--home-wait", "--idle-wait", "--quiet", "--interval")
	if err != nil {
		return err
	}
	defer restore()
	reason = strings.TrimSpace(reason)
	switch {
	case reason == "":
		return cellDeployExit(cellDeployExitRefused, "realtime-core restart requires --reason (why; kept in the journals and shown to operators); nothing was contacted")
	case strings.ContainsAny(reason, "\r\n") || len(reason) > cellDeployRestartReasonMax:
		return cellDeployExit(cellDeployExitRefused, "--reason must be one line of at most %d characters", cellDeployRestartReasonMax)
	}
	if err := wait.validate(); err != nil {
		return err
	}
	w, err := cellDeploySeconds("--home-wait", homeWait, 0, 10*time.Minute)
	if err != nil {
		return err
	}
	repo, err := repoRoot()
	if err != nil {
		return err
	}
	axes, err := cellDeployResettableAlarms(repo, ctx)
	if err != nil {
		return fmt.Errorf("drive alarm classes for %s: %w", ctx.CellID, err)
	}
	quiet := int(wait.Quiet / time.Second)
	if common.DryRun {
		const placeholderPin = "0000000000000000000000000000000000000000000000000000000000000000"
		m := cellDeployRestartNotice(ctx, reason)
		m.LockInherit = "<this run's cell lock>"
		script, err := cellDeployRestartScript(ctx, m, cellDeployRestartPlan{Pin: placeholderPin, Release: "<current release>"}, reason)
		if err != nil {
			return err
		}
		script = strings.ReplaceAll(script, placeholderPin, "<installed pin>")
		fmt.Fprintf(stdout, "realtime-core restart cell=%s servo=%s clear_drive_alarms=%t clear_rehome_faults=%t reason=%q (dry run: nothing contacted)\n"+
			"# 0. cell lock for the whole run: idle wait up to %s (quiet %s), then both maintenance notices (arming refused)\n"+
			"# 1. copy-dumps\n%s\n# 2. facts (read-only; the installed pin and release come from here)\n%s\n"+
			"# 3. restart on the servo host (installed release and pin; start's refusals)\n%s\n"+
			"# 4. verify (read-only) against the configuration and Home before the restart\n%s\n",
			ctx.CellID, ctx.ServoTarget, consent.DriveAlarms, consent.RehomeFaults, reason, wait.Limit, wait.Quiet, cellDeployCopyDumpsScript(ctx), cellDeployPrecheckScript(ctx, quiet), script, cellDeployVerifyScript(ctx, w))
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core restart", common.Transport, stdout)
	if err != nil {
		return err
	}
	run.p.Step("restart", "reason: "+reason)
	if consent.RehomeFaults {
		run.p.Step("restart", "--clear-rehome-faults: the owner found the cause of the latched faults that need a new Home; accepted only on joints whose Home is already lost, with no drive alarm")
	}
	if consent.DriveAlarms {
		run.p.Step("restart", "--clear-drive-alarms: the owner inspected the cause of the drive alarm(s) present; only alarms the drive profile lists resettable are accepted")
	}
	estimate := releaseJournalHistory(ctx.CellID).stepEstimate("restart", releaseGatePlan{})
	unlock, err := cellLockForRun(run.p, ctx, common.Transport, "restart", "controlled rt-core restart ("+reason+")", releaseETAWithMargin(estimate.Expected), wait)
	if err != nil {
		run.finish("refused", deployExitCode(err), err.Error())
		return err
	}
	_, stopProgress := noticeRunProgress(run.p, ctx.CellID, "restart")
	detail, err := cellDeployRestartSteps(run, ctx, common.Transport, reason, consent, w, quiet, axes)
	stopProgress()
	unlock()
	if err != nil {
		result := "failed"
		if deployExitCode(err) == cellDeployExitRefused {
			result = "refused"
		}
		run.finish(result, deployExitCode(err), err.Error())
		return err
	}
	run.finish("ok", 0, detail)
	return nil
}

// cellDeployRestartSteps runs inside the run lock: copy-dumps, facts and
// refusals, the servo-host restart, verify and the before/after report.
func cellDeployRestartSteps(run *cellDeployRun, ctx cellDeployContext, transport, reason string, consent cellDeployRestartConsent, homeWait, quiet int, axes []cellDeployAxisAlarms) (string, error) {
	if _, _, err := cellDeployCopyDumpsSession(run, ctx, transport, false); err != nil {
		return "", err
	}
	code, probe, result, refused, err := run.session("facts", cellDeployPrecheckScript(ctx, quiet))
	if e := sessionOutcome("restart facts", code, result, refused, err); e != nil {
		return "", e
	}
	facts, err := decodeCellDeployFacts(probe)
	if err != nil {
		return "", cellDeployExit(cellDeployExitRefused, "restart facts unreadable: %v", err)
	}
	facts.markMacCopies(ctx.CellID)
	plan, refuse, notes := cellDeployEvaluateRestart(ctx, facts, axes, consent)
	run.report(nil, notes)
	run.p.Step("before", cellDeployRestartAlarmSummary(axes, facts, plan.Resume))
	if len(refuse) > 0 {
		run.report(refuse, nil)
		return "", cellDeployExit(cellDeployExitRefused, "RESTART REFUSED (nothing was changed):\n  - %s", strings.Join(refuse, "\n  - "))
	}
	for _, line := range plan.Cleared {
		run.p.Step("cleared", "owner-approved, present before the restart: "+line)
	}
	mode := "stop and start"
	if plan.Resume {
		mode = "start only (both units were stopped)"
	}
	run.p.Step("restart", fmt.Sprintf("%s: release %s, configuration %s (installed; unchanged)", mode, path.Base(plan.Release), plan.Pin))
	if err := cellLockLost(); err != nil {
		return "", cellDeployExit(cellDeployExitRefused, "restart stopped before the servo host was touched: %v", err)
	}
	script, err := cellDeployRestartScript(ctx, cellDeployRestartNotice(ctx, reason), plan, reason)
	if err != nil {
		return "", err
	}
	code, _, result, refused, err = run.session("restart", script)
	switch {
	case err != nil:
		return "", cellDeployExit(cellDeployExitFailed, "restart: relay session failed: %v. The servo-host script ignores hangups and finishes on its own; read the state with realtime-core verify --expect-installed --manifest %s before anything else", err, ctx.Manifest)
	case code == cellDeployExitRefused:
		return "", cellDeployExit(cellDeployExitRefused, "restart refused on the servo host (nothing was changed): %s", strings.Join(refused, "; "))
	case code == cellDeployExitStopUnclean:
		return "", cellDeployExit(code, "STOP NOT CLEAN: nothing was started and both units are stopped (lines above). Ask the owner before anything else; never SIGKILL rosie-rt-core, and a servo host reboot needs the owner's approval")
	case code != 0:
		return "", cellDeployExit(cellDeployExitFailed, "START FAILED (exit %d): both units are stopped, disarmed, bus owner none (lines above). Fix the cause, then run restart again: with both units stopped it only starts them", code)
	}
	after, verr := cellDeployVerifySessionExpect(run, ctx, homeWait, cellDeployVerifyExpect{Pin: plan.Pin, HomeMask: plan.HomeBefore}, false)
	run.p.Step("after", cellDeployRestartAlarmSummary(axes, after, false))
	for _, line := range cellDeployRestartHomeReport(ctx, axes, plan.HomeBefore, after) {
		run.p.Step("HOME", line)
	}
	if verr != nil {
		if after.Status != nil && len(after.Status.Alarms) > 0 {
			run.p.Step("after", "a drive alarm is still present after the restart: its cause persists; fix it at the drive (never re-home, never widen a bound)")
		}
		return "", verr
	}
	home := "Home restored"
	if after.Status != nil && plan.HomeBefore != nil && *plan.HomeBefore != ctx.FullAxisMask() {
		home = fmt.Sprintf("Home was lost before the restart; home_valid_mask=%d now", after.Status.HomeValidMask)
		if missing := uint32(ctx.FullAxisMask() &^ after.Status.HomeValidMask); missing != 0 {
			home += "; an operator homes " + cellDeployAxisNames(axes, missing) + " at the mechanical home pose (restart never homes)"
		}
	}
	run.p.Step("done", fmt.Sprintf("RESTARTED: release %s, configuration %.12s unchanged; every axis verified; %s; disarmed; no faults", path.Base(plan.Release), plan.Pin, home))
	return "restarted " + plan.Pin, nil
}

// cellDeployRestartConsent is what the owner accepts that a restart may
// clear; the zero value accepts nothing.
type cellDeployRestartConsent struct {
	// DriveAlarms: --clear-drive-alarms, resettable drive alarms whose cause
	// the owner inspected.
	DriveAlarms bool
	// RehomeFaults: --clear-rehome-faults, faults that need a new Home on axes
	// whose Home is already lost.
	RehomeFaults bool
}

// cellDeployRehomeBoundFaults returns the latched faults whose recovery
// re-homes every axis they latched: their axes, their
// execution_fault_reasons bits and their names. A fault with any axis a new
// Home would not recover is not included, so it still refuses.
func cellDeployRehomeBoundFaults(s *cellDeployStatusFacts) (safety, execution uint32, names []string) {
	for _, fault := range s.RecoveryFaults {
		if fault.AxisMask == 0 || fault.Bit >= 32 || fault.RehomeAxisMask&fault.AxisMask != fault.AxisMask {
			continue
		}
		safety |= fault.AxisMask
		execution |= 1 << fault.Bit
		names = append(names, fault.Name)
	}
	return safety, execution, names
}

// cellDeployConditionFaults returns the execution_fault_reasons bits of the
// drive_readiness and bus_transport latches (reset_after_condition_clears)
// confined to the stale axes: a stale coordinate latch leaves readiness
// faulted on the same joints. Drive alarms are refused before this is asked;
// a condition that persists latches again in the fresh process and fails verify.
func cellDeployConditionFaults(s *cellDeployStatusFacts, stale uint32) uint32 {
	var bits uint32
	for _, fault := range s.RecoveryFaults {
		if fault.Bit >= 32 || fault.Recovery != "reset_after_condition_clears" || fault.AxisMask&^stale != 0 {
			continue
		}
		if fault.Name == "drive_readiness" || fault.Name == "bus_transport" {
			bits |= 1 << fault.Bit
		}
	}
	return bits
}

// cellDeployAxisNames names the joints in mask ("J1, J2"), by the machine's
// joint names when known.
func cellDeployAxisNames(axes []cellDeployAxisAlarms, mask uint32) string {
	var names []string
	for i := 0; i < 32; i++ {
		if mask&(1<<uint(i)) == 0 {
			continue
		}
		if i < len(axes) && axes[i].Joint != "" {
			names = append(names, axes[i].Joint)
		} else {
			names = append(names, fmt.Sprintf("axis %d", i))
		}
	}
	return strings.Join(names, ", ")
}

// cellDeployRestartAlarmSummary is the one line of faults and drive alarms.
func cellDeployRestartAlarmSummary(axes []cellDeployAxisAlarms, f cellDeployFacts, stopped bool) string {
	s := f.Status
	switch {
	case s == nil && stopped:
		return "controller stopped: faults and drive alarms not readable"
	case s == nil:
		return "controller status unavailable: faults and drive alarms not readable"
	}
	alarms := "none"
	if len(s.DriveAlarms) > 0 {
		lines := make([]string, len(s.DriveAlarms))
		for i, a := range s.DriveAlarms {
			lines[i] = cellDeployAlarmLine(axes, a)
		}
		alarms = strings.Join(lines, "; ")
	} else if len(s.Alarms) > 0 {
		alarms = strings.Join(s.Alarms, "; ")
	}
	return fmt.Sprintf("home_valid_mask=%d safety_fault_mask=%d execution_fault_reasons=%d drive alarms: %s", s.HomeValidMask, s.SafetyFaultMask, s.ExecutionFaultReasons, alarms)
}

// cellDeployRestartHomeReport names every axis whose Home was valid before
// the restart and is not after it. Restart never homes: report it.
func cellDeployRestartHomeReport(ctx cellDeployContext, axes []cellDeployAxisAlarms, before *uint64, after cellDeployFacts) []string {
	want := ctx.FullAxisMask()
	if before != nil {
		want = *before
	}
	if after.Status == nil {
		return []string{"Home after the restart not readable: report it; never re-home to find out"}
	}
	lost := want &^ after.Status.HomeValidMask
	var out []string
	for i := 0; i < ctx.MachineAxes; i++ {
		if lost&(1<<uint(i)) == 0 {
			continue
		}
		label := fmt.Sprintf("axis %d", i)
		if i < len(axes) {
			label = axes[i].label(i)
		}
		out = append(out, fmt.Sprintf("%s: Home NOT restored after the restart (home_valid_mask %d before, %d after). Report it to the owner; never re-home (HM35 only by an operator at the mechanical home pose) and never restore anchors to clear it", label, want, after.Status.HomeValidMask))
	}
	return out
}
