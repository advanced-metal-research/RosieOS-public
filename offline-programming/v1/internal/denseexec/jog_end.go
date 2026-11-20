//go:build linux

package denseexec

import (
	"fmt"
	"log"
	"math"
	"strings"
	"time"

	"rosieos/rt-core/sdk/control"
)

// Jog end reasons (owner request 2026-10-10). A hold that ends without the
// operator releasing -- the robot stops, stays armed and the operator must
// pull the trigger again -- used to leave only an internal code
// (jog_input_deadline_expired, path_buffer_underrun, a resolver gate, a
// native stop number) in JointJogState.reason, which only the client that
// sent the failing intent ever saw, and the journal said reason=<code> with
// nothing to judge it by. Every hold end is now classified once, here, into
// one operator-meaningful code with the words to show, the technical detail,
// whether pulling the trigger again can work, and the evidence; it is
// journalled on the hold's end line and published as last_jog_end on the
// status body every client polls, attributed to the client that owned it.
//
// The classification only reads causes OLP already has: its own refusal
// codes, the native path observation (PathStopReason, PathGoverning, samples
// received/consumed) and the resolver's path-end reason. No native field or
// protocol value is added (the capabilities digest is unchanged).

// Operator end codes.
const (
	JogEndReleased             = "released"
	JogEndInputDeadlineExpired = "input_deadline_expired"
	JogEndPathUnderrun         = "path_underrun"
	JogEndPlayableEnd          = "playable_end"
	JogEndSingularityGate      = "singularity_gate"
	JogEndIKDiscontinuity      = "ik_discontinuity"
	JogEndJointLimit           = "joint_limit"
	JogEndWorkspaceLimit       = "workspace_limit"
	JogEndSpeedGovernedToZero  = "speed_governed_to_zero"
	JogEndSessionStale         = "session_stale"
	JogEndTransportLost        = "transport_lost"
	JogEndHalted               = "halted"
	JogEndInhibited            = "inhibited"
	JogEndGeneratorFault       = "generator_fault"
	JogEndStartMismatch        = "start_mismatch"
	JogEndCommandRefused       = "command_refused"
	JogEndDriveFault           = "drive_fault"
	// Client end causes (an End's end_cause).
	JogEndStickDirectionChange   = "stick_direction_change"
	JogEndPointerLeft            = "pointer_left"
	JogEndFocusLost              = "focus_lost"
	JogEndControllerDisconnected = "controller_disconnected"
	JogEndModeSwitch             = "mode_switch"
	JogEndUINavigation           = "ui_navigation"
	JogEndJogDisabled            = "jog_disabled"
	JogEndClientEnded            = "client_ended"
	JogEndRefusedAtStart         = "refused_at_start"
	JogEndUnknown                = "unknown"
)

// Whether pulling the trigger again (same axis, same direction) can work.
const (
	RepullYes        = "yes"
	RepullNo         = "no"          // the same direction meets the same cause
	RepullAfterRearm = "after_rearm" // the robot is disarmed: Arm first
	// Release the trigger and centre the stick first (the Deck's
	// neutral-first rule for a new direction).
	RepullAfterNeutral = "after_neutral"
	RepullUnknown      = "unknown"
)

// jogEndCauses maps every cause OLP can retire a hold with -- its own codes,
// the SDK's and the native jog reasons as nativeJointJogReason names them,
// the native path stop names (pathStopNames) and the resolver's path-end
// reasons (pathEndCodes) -- to one operator code. A cause missing here is
// JogEndUnknown: shown with its raw code, never hidden.
var jogEndCauses = map[string]string{
	"":               JogEndReleased,
	"trace_complete": JogEndReleased,
	// Operator input stopped arriving in time (OLP's own deadman, native's
	// speed deadline, or an intent native refused for its timing).
	"jog_input_deadline_expired": JogEndInputDeadlineExpired, "jog_expired": JogEndInputDeadlineExpired,
	"expired": JogEndInputDeadlineExpired, "jog_invalid_timing": JogEndInputDeadlineExpired,
	"jog_clock_moved_backwards": JogEndInputDeadlineExpired, "jog_input_too_old": JogEndInputDeadlineExpired,
	// Path geometry did not arrive ahead of consumption, or the line ended.
	"path_buffer_underrun": JogEndPathUnderrun, "path_end": JogEndPlayableEnd, "path_ended": JogEndPlayableEnd,
	// Resolver gates and native path checks.
	"jacobian_gate": JogEndSingularityGate, "ik_discontinuity": JogEndIKDiscontinuity,
	"joint_limit": JogEndJointLimit, "outside_limits_outward": JogEndJointLimit,
	"ik_no_solution": JogEndWorkspaceLimit, "cartesian_reach": JogEndWorkspaceLimit,
	"jog_limited": JogEndSpeedGovernedToZero,
	// The session OLP was feeding is not the one native has any more.
	"jog_session_stale": JogEndSessionStale, "jog_wrong_identity": JogEndSessionStale,
	"jog_invalid_generation": JogEndSessionStale, "jog_stale_sequence": JogEndSessionStale,
	"jog_session_or_sequence_stale": JogEndSessionStale, "control_session_stale": JogEndSessionStale,
	"jog_closed": JogEndSessionStale, "path_closed": JogEndSessionStale, "release": JogEndSessionStale,
	"jog_generation_exhausted": JogEndSessionStale, "cartesian_resolver_changed": JogEndSessionStale,
	"server_shutdown": JogEndSessionStale, "jog_mode_conflict": JogEndSessionStale, "jog_ramping": JogEndSessionStale,
	// The link to rt-control.
	"rt_core_transport_lost": JogEndTransportLost, "rt_core_closed": JogEndTransportLost,
	"rt_core_protocol_error": JogEndTransportLost, "rt_core_connection_reopened": JogEndTransportLost,
	"rt_core_request_slow": JogEndTransportLost, "rt_core_timeout": JogEndTransportLost,
	"rt_core_cancelled": JogEndTransportLost, "jog_transport_congested": JogEndTransportLost,
	"rt_core_status_slow": JogEndTransportLost, "rt_core_status_stale": JogEndTransportLost,
	// rtFailure's catch-all: an SDK error of no typed kind (a jog stream's EOF).
	"rt_core_failure":    JogEndTransportLost,
	"jog_publisher_busy": JogEndCommandRefused, "jog_stream_idle": JogEndInputDeadlineExpired,
	"jog_input_entirely_future": JogEndInputDeadlineExpired,
	// The link from the operator's client to this server.
	"ui_heartbeat_lost": JogEndTransportLost, "client_disconnected": JogEndTransportLost,
	"rt_core_identity_mismatch": JogEndSessionStale, "rt_core_incarnation_changed": JogEndSessionStale,
	"path_chunk_stale": JogEndSessionStale, "mode_conflict": JogEndSessionStale,
	"control_renewal_lost": JogEndInhibited, "rt_core_readiness_lost": JogEndInhibited,
	"rt_core_fault": JogEndInhibited, "rt_core_event": JogEndInhibited, "not_ready": JogEndInhibited,
	// Stop, Disarm, Halt.
	"operator_stop": JogEndHalted, "stop": JogEndHalted, "disarm": JogEndHalted,
	"selection_changed": JogEndHalted, "halt": JogEndHalted,
	// Authority, readiness or identity withdrawn: native inhibits (disarms).
	"jog_not_ready": JogEndInhibited, "no_grant": JogEndInhibited, "jog_authority_lost": JogEndInhibited,
	"jog_clock_reset": JogEndInhibited, "inhibited": JogEndInhibited,
	// The path generator: native fail-closed, or OLP's resolver/streaming.
	"path_fault": JogEndGeneratorFault, "cartesian_resolver_unavailable": JogEndGeneratorFault,
	"cartesian_resolver_invalid": JogEndGeneratorFault, "path_capacity": JogEndGeneratorFault,
	"path_out_of_order": JogEndGeneratorFault, "path_invalid_sample": JogEndGeneratorFault,
	"path_invalid_config": JogEndGeneratorFault, "path_queue_full": JogEndCommandRefused,
	"path_start_mismatch": JogEndStartMismatch,
	// Refusals while the hold was still setting up.
	"cartesian_hold_unavailable": JogEndRefusedAtStart, "cartesian_input_invalid": JogEndRefusedAtStart,
	"cartesian_axes_invalid": JogEndRefusedAtStart, "cartesian_pose_stale": JogEndRefusedAtStart,
	"cartesian_resolver_too_slow": JogEndRefusedAtStart, "dense_target_unavailable": JogEndRefusedAtStart,
	"capability_unimplemented": JogEndRefusedAtStart, "invalid_axis_mask": JogEndRefusedAtStart,
	"jog_invalid_vector": JogEndRefusedAtStart, "unknown_operation": JogEndRefusedAtStart,
	"path_jog_unavailable": JogEndRefusedAtStart,
}

// Unverified operator policy: a path hold whose native speed sat at zero
// under a geometric governor (per-sample cap, joint velocity, curvature,
// jerk) for this long before release had stalled while the operator held
// the trigger -- the "singularity jam". Shorter zero-speed spells are the
// ordinary approach to a cap and are not reported.
const jogStallReportAfter = 300 * time.Millisecond

// jogEndFacts is everything the classification reads. It is filled from the
// retiring hold under the controller mutex; classifyJogEnd is pure.
type jogEndFacts struct {
	cause   string // OLP retirement code; "" is an End from a client
	atStart bool   // the hold had not finished setting up
	// clientCause: the End's end_cause ("" from an older client: a release).
	clientCause string
	// endedByOther: a client other than the owner ended it (its label).
	endedByOther string
	lifetime     time.Duration
	intentAge    time.Duration
	// Path holds.
	path        bool
	stopReason  uint32
	governing   uint32
	ahead       uint64
	underruns   uint64
	resolverEnd string
	stall       time.Duration
	stallBy     string
	// The last joint position the hold reached or streamed, with limits, for
	// naming a joint: Cartesian holds the last sample sent, joint jogs the
	// measured position of the jogged axis.
	q            []float64
	lower, upper []float64
	jogAxis      int // joint jog axis, -1 for Cartesian
	// driveFault names the drives native holds faulted (driveFaultWords).
	driveFault string
}

// driveFaultWords names every axis whose drive native holds faulted, with the
// drive's alarm code and the manual's words when native decoded them (rt-core
// publishes drive_alarm from docs/drives/a6ec/a6ec_manual_codes.json), e.g.
// "J5 drive alarm Er41.1 (Motor over-temperature due to locked-rotor)". An axis
// counts only while it is faulted (readiness faulted, its safety-fault bit or a
// drive_readiness recovery fault) and the drive itself reports it (an alarm, a
// 0x603F error code or that recovery fault): a drive warning on a ready axis
// and a fault rt-core raised on its own (following error) are not drive
// faults. "" when no drive faulted.
func driveFaultWords(s *control.Status, ids []string) string {
	if s == nil {
		return ""
	}
	var driveReadiness uint32
	if s.Core.Recovery != nil {
		for _, fault := range s.Core.Recovery.Faults {
			if fault.Name == "drive_readiness" {
				driveReadiness |= fault.AxisMask
			}
		}
	}
	var words []string
	for i, a := range s.Axes {
		bit := uint32(1) << uint(i)
		alarm, text, code := a.DriveAlarm, a.DriveAlarmText, uint16(0)
		if i < len(s.Core.Axes) {
			c := s.Core.Axes[i]
			code = c.ErrorCode
			if alarm == "" {
				alarm, text = c.DriveAlarm, c.DriveAlarmText
			}
		}
		faulted := a.Readiness == "faulted" || s.Core.SafetyFaultMask&bit != 0 || driveReadiness&bit != 0
		if !faulted || alarm == "" && code == 0 && driveReadiness&bit == 0 {
			continue
		}
		name := fmt.Sprintf("J%d", i+1)
		if i < len(ids) && ids[i] != "" {
			name = ids[i]
		}
		switch {
		case alarm != "" && strings.TrimSpace(text) != "":
			words = append(words, fmt.Sprintf("%s drive alarm %s (%s)", name, alarm, strings.Join(strings.Fields(text), " ")))
		case alarm != "":
			words = append(words, fmt.Sprintf("%s drive alarm %s", name, alarm))
		case code != 0:
			words = append(words, fmt.Sprintf("%s drive fault 0x%04X", name, code))
		default:
			words = append(words, name+" drive not ready")
		}
	}
	return strings.Join(words, "; ")
}

// driveFaultMayExplain: the end codes a drive fault outranks. Native ends the
// hold and disarms when a drive faults; the lane then reports a stale session,
// an inhibit, a closed stream or no code it knows. An operator's release,
// Stop or Halt, a limit or a resolver gate keeps its own words.
func driveFaultMayExplain(code string) bool {
	switch code {
	case JogEndSessionStale, JogEndInhibited, JogEndTransportLost, JogEndUnknown,
		JogEndCommandRefused, JogEndGeneratorFault, JogEndInputDeadlineExpired, JogEndPathUnderrun, JogEndRefusedAtStart:
		return true
	}
	return false
}

func driveFaultText(words string, atStart bool) string {
	stopped := "Jog stopped"
	if atStart {
		stopped = "Jog could not start"
	}
	return fmt.Sprintf("%s and the robot disarmed: %s. Check that joint (a blockage, a cable or hose binding, a brake that did not release); once the drive clears the alarm, Clear faults, then Arm.", stopped, words)
}

type jogEndClass struct {
	code, text, detail, repull string
	popup, disarmed            bool
}

func degrees(rad float64) float64 { return rad * 180 / math.Pi }

// limitJoint names the joint nearest one of its limits.
func (f jogEndFacts) limitJoint() string {
	best, margin, side, limit := -1, math.Inf(1), "", 0.0
	for i := range f.q {
		if i >= len(f.lower) || i >= len(f.upper) || f.jogAxis >= 0 && i != f.jogAxis {
			continue
		}
		if m := f.q[i] - f.lower[i]; m < margin {
			best, margin, side, limit = i, m, "lower", f.lower[i]
		}
		if m := f.upper[i] - f.q[i]; m < margin {
			best, margin, side, limit = i, m, "upper", f.upper[i]
		}
	}
	if best < 0 {
		return "a joint reached its limit"
	}
	return fmt.Sprintf("J%d reached its %s limit (%.1f°)", best+1, side, degrees(limit))
}

// singularPose gives the wrist bend and elbow at the last streamed sample.
// It does not say which singularity it is: the resolver reports none, and
// where the wrist one lies is the robot's geometry (a Rosie 1430 arm: J5 = +-90 deg, not
// 0; rt-core/docs/history/2026-10-10-singularity-and-jog-dropouts.md).
func (f jogEndFacts) singularPose() string {
	if len(f.q) >= 5 {
		return fmt.Sprintf(" (J5 = %.1f°, J3 = %.1f°)", degrees(f.q[4]), degrees(f.q[2]))
	}
	return ""
}

var governorWords = map[string]string{
	"cap":       "the per-sample speed cap near a singularity",
	"velocity":  "a joint speed limit",
	"curvature": "a joint acceleration limit at the path's curvature",
	"jerk":      "a joint jerk limit",
}

var nativeInhibitWords = map[string]string{
	"jog_not_ready":          "the machine is no longer ready (disarmed, a drive not ready, or Home lost)",
	"no_grant":               "this server no longer holds the robot's control grant",
	"jog_authority_lost":     "rt-control withdrew this server's authority",
	"jog_clock_reset":        "the controller's clock mapping changed",
	"inhibited":              "rt-control inhibited the outputs (authority, readiness, identity or Home)",
	"not_ready":              "the machine is no longer ready",
	"control_renewal_lost":   "the control lease could not be renewed",
	"rt_core_readiness_lost": "the machine is no longer ready (a drive or Home)",
	"rt_core_fault":          "the robot controller reported a fault",
	"rt_core_event":          "the robot controller reported a fault event",
}

// classifyJogEnd turns one hold end into the operator's words.
func classifyJogEnd(f jogEndFacts) jogEndClass {
	if f.cause == "" && f.clientCause != "" {
		return withClientCause(classifyClientEnd(f), f.clientCause)
	}
	return classifyServerEnd(f)
}

func withClientCause(c jogEndClass, cause string) jogEndClass {
	if cause != "" && !strings.Contains(c.detail, "client_cause=") {
		c.detail += " client_cause=" + cause
	}
	return c
}

// classifyClientEnd words an End the client sent with its reason (fix B2 of
// rt-core/docs/history/2026-10-10-singularity-and-jog-dropouts.md: 107 of
// that night's 112 holds were client ends, all logged "release"). A plain
// release stays released; a client end that the operator did not choose
// (stick direction change, lost focus, a pointer that slid off) pops up; one
// the operator chose (a page or mode change, Stop) does not. A client
// reporting a refusal it met is classified by that refusal.
func classifyClientEnd(f jogEndFacts) jogEndClass {
	cc := f.clientCause
	for _, prefix := range []string{"intent_refused:", "server_reason:"} {
		if inner := strings.TrimPrefix(cc, prefix); inner != cc && inner != "" {
			f.cause = inner
			return classifyServerEnd(f)
		}
	}
	code, known := clientEndCauses[cc]
	if !known {
		code = JogEndClientEnded
	}
	stopped := "Jog stopped"
	if f.atStart {
		stopped = "Jog could not start"
	}
	c := jogEndClass{code: code, popup: true, repull: RepullYes, detail: "client ended the hold: " + cc}
	switch code {
	case JogEndReleased:
		f.clientCause = ""
		c = classifyServerEnd(f)
	case JogEndHalted, JogEndSessionStale, JogEndTransportLost:
		// The client's words for a cause the server already has.
		f.cause = map[string]string{"operator_stop": "operator_stop", "operator_disarm": "disarm", "selection_changed": "selection_changed",
			"session_changed": "jog_session_stale", "state_poll_failed": "client_disconnected"}[cc]
		c = classifyServerEnd(f)
		if cc == "operator_stop" || cc == "operator_disarm" {
			// This client's own Stop or Disarm: the operator pressed it.
			c.popup = f.endedByOther != ""
		}
	case JogEndStickDirectionChange:
		c.text, c.repull = "Jog stopped: the stick changed direction while the trigger was held. Release RT and centre the stick to jog a new direction.", RepullAfterNeutral
	case JogEndPointerLeft:
		c.text = fmt.Sprintf("%s: the pointer left the jog button (or the browser took it). Press and hold again.", stopped)
	case JogEndFocusLost:
		c.text = fmt.Sprintf("%s: this window lost focus (another window, an overlay or a hidden tab). Jog again once it is back in front.", stopped)
	case JogEndControllerDisconnected:
		c.text = fmt.Sprintf("%s: the game controller disconnected. Reconnect it, then jog again.", stopped)
	case JogEndModeSwitch, JogEndUINavigation:
		// The operator changed page, mode, frame or speed: their own choice.
		c.text, c.popup = fmt.Sprintf("Jog ended: %s.", clientEndWords[cc]), false
	case JogEndJogDisabled:
		c.text, c.repull = fmt.Sprintf("%s: jogging became unavailable on this client (no fresh status, not armed, or no control session).", stopped), RepullUnknown
	default:
		c.text, c.repull = fmt.Sprintf("%s by this client (%s).", stopped, cc), RepullUnknown
	}
	if f.endedByOther != "" && !strings.Contains(c.detail, "ended_by=") {
		c.detail += " ended_by=" + f.endedByOther
	}
	return c
}

// Client end causes (the End body's end_cause), each to one operator code.
// An older client sends none: a release, as before.
var clientEndCauses = map[string]string{
	"trigger_released": JogEndReleased, "pointer_up": JogEndReleased, "stick_neutral": JogEndReleased,
	"button_released": JogEndReleased, "key_up": JogEndReleased,
	"stick_direction_change": JogEndStickDirectionChange,
	"pointer_leave":          JogEndPointerLeft, "pointer_cancel": JogEndPointerLeft, "pointer_capture_lost": JogEndPointerLeft,
	"focus_lost": JogEndFocusLost, "visibility_hidden": JogEndFocusLost, "page_hidden": JogEndFocusLost,
	"controller_disconnected": JogEndControllerDisconnected,
	"mode_switch":             JogEndModeSwitch, "speed_change": JogEndModeSwitch, "frame_change": JogEndModeSwitch,
	"ui_navigation": JogEndUINavigation, "cell_change": JogEndUINavigation, "app_closed": JogEndUINavigation,
	"maintenance":   JogEndUINavigation,
	"operator_stop": JogEndHalted, "operator_disarm": JogEndHalted, "selection_changed": JogEndHalted,
	"session_changed": JogEndSessionStale, "state_poll_failed": JogEndTransportLost,
	"jog_disabled": JogEndJogDisabled,
}

var clientEndWords = map[string]string{
	"mode_switch": "the jog mode changed", "speed_change": "the jog speed changed", "frame_change": "the jog frame changed",
	"ui_navigation": "the page changed", "cell_change": "the cell connection changed", "app_closed": "the client closed",
	"maintenance": "the maintenance notice came up",
}

// sanitizeEndCause keeps a client's end cause to what the journal and the
// classification can carry: lower-case words, digits and _:.- up to 80
// characters; anything else is reported as such, never dropped.
func sanitizeEndCause(cause string) string {
	cause = strings.TrimSpace(cause)
	if cause == "" {
		return ""
	}
	if len(cause) > 80 {
		return "invalid_end_cause"
	}
	for _, c := range cause {
		if !(c >= 'a' && c <= 'z' || c >= '0' && c <= '9' || c == '_' || c == ':' || c == '.' || c == '-') {
			return "invalid_end_cause"
		}
	}
	return cause
}

func classifyServerEnd(f jogEndFacts) jogEndClass {
	cause := strings.SplitN(f.cause, ":", 2)[0]
	code, known := jogEndCauses[cause]
	switch {
	case known:
	case strings.HasPrefix(cause, "jog_clock_"):
		// The remote lane's clock calibration (WSS): the link's timing.
		code = JogEndTransportLost
	case strings.HasPrefix(cause, "jog_native_rejected_"):
		// The SDK's name for an adapter refusal it has no name for (a Rosie 1430
		// arm, 2026-10-10 22:01:13: reason 3 on one speed frame, armed and ready).
		code = JogEndCommandRefused
	default:
		code = JogEndUnknown
	}
	// A native close the path lane reports as plain "closed" carries the
	// native stop reason: a public Halt (another rt-control client) is a halt.
	if f.path && cause == "jog_closed" && pathName(pathStopNames, f.stopReason) == "halt" {
		code = JogEndHalted
	}
	// Released while native had held the path at zero speed: the stall the
	// operator saw, not a release.
	if code == JogEndReleased && f.path && f.stall >= jogStallReportAfter {
		code = JogEndSpeedGovernedToZero
	}
	if f.atStart {
		switch code {
		case JogEndSessionStale, JogEndTransportLost, JogEndUnknown, JogEndPlayableEnd, JogEndPathUnderrun:
			code = JogEndRefusedAtStart
		}
	}
	if f.driveFault != "" && driveFaultMayExplain(code) {
		code = JogEndDriveFault
	}
	stopped := "Jog stopped"
	if f.atStart {
		stopped = "Jog could not start"
	}
	c := jogEndClass{code: code, popup: true, repull: RepullYes}
	nativeStop := ""
	if f.path {
		nativeStop = fmt.Sprintf(" native_stop=%s governing=%s samples_ahead=%d underruns=%d",
			pathName(pathStopNames, f.stopReason), pathName(pathGoverningNames, f.governing), f.ahead, f.underruns)
	}
	switch code {
	case JogEndReleased:
		c.text, c.popup = "Jog released.", f.endedByOther != ""
		if f.endedByOther != "" {
			c.text = fmt.Sprintf("Jog stopped: %s ended this jog.", f.endedByOther)
		}
		c.detail = "operator release"
	case JogEndInputDeadlineExpired:
		c.text = fmt.Sprintf("%s: the jog commands arrived late (last one %d ms ago, limit %d ms). Pull the trigger again; if it repeats, check the pendant's network link.",
			stopped, f.intentAge.Milliseconds(), f.lifetime.Milliseconds())
		c.detail = fmt.Sprintf("cause=%s last_intent_age_ms=%d lifetime_ms=%d: no admitted operator intent renewed the deadman in time%s",
			f.cause, f.intentAge.Milliseconds(), f.lifetime.Milliseconds(), nativeStop)
	case JogEndPathUnderrun:
		c.text = fmt.Sprintf("%s: the path was not computed fast enough to keep up (%d samples ahead at the end). Pull the trigger again; lower the speed if it repeats.", stopped, f.ahead)
		c.detail = fmt.Sprintf("cause=%s: native stopped at the end of the samples received%s", f.cause, nativeStop)
	case JogEndPlayableEnd:
		c.text = fmt.Sprintf("%s at the end of the computed line. Pull the trigger again to continue.", stopped)
		c.detail = fmt.Sprintf("cause=%s resolver_end=%q: native stopped at the playable end%s", f.cause, f.resolverEnd, nativeStop)
	case JogEndSingularityGate:
		c.text, c.repull = fmt.Sprintf("%s: the robot reached a singularity%s; jog back the way you came, or reposition with joint jog.", stopped, f.singularPose()), RepullNo
		c.detail = fmt.Sprintf("cause=%s: the resolver's Jacobian gate (sigma_min <= 0.02) ended the line%s", f.cause, nativeStop)
	case JogEndIKDiscontinuity:
		c.text, c.repull = fmt.Sprintf("%s: this direction needs a joint to turn faster than it can (near a singularity or an arm flip%s); jog another way or reposition with joint jog.", stopped, f.singularPose()), RepullNo
		c.detail = fmt.Sprintf("cause=%s: a joint step beyond max velocity x max_sample_step (resolver or native step check) ended the line%s", f.cause, nativeStop)
	case JogEndCommandRefused:
		c.text = fmt.Sprintf("%s: the robot controller refused one jog command; the robot stayed armed. Pull the trigger again.", stopped)
		c.detail = fmt.Sprintf("cause=%s: rt-control refused one speed frame and closed the jog stream (adapter reason %s; its catch-all not_ready also covers a busy publisher)%s",
			f.cause, firstNonEmptyCause(strings.TrimPrefix(cause, "jog_native_rejected_")), nativeStop)
	case JogEndJointLimit:
		c.text, c.repull = fmt.Sprintf("%s: %s; jog the other way.", stopped, f.limitJoint()), RepullNo
		c.detail = fmt.Sprintf("cause=%s: the next sample would cross a joint limit (resolver or native position check)%s", f.cause, nativeStop)
	case JogEndWorkspaceLimit:
		c.text, c.repull = fmt.Sprintf("%s: this line leaves the robot's reach; jog the other way.", stopped), RepullNo
		c.detail = fmt.Sprintf("cause=%s: no inverse kinematics solution further along the line%s", f.cause, nativeStop)
	case JogEndSpeedGovernedToZero:
		by := governorWords[f.stallBy]
		if by == "" {
			by = "a joint limit"
		}
		c.text, c.repull = fmt.Sprintf("Jog stalled: the speed was held at zero by %s while the trigger was held; jog the other way or reposition with joint jog.", by), RepullNo
		c.detail = fmt.Sprintf("cause=%s stalled_ms=%d governor=%s%s", firstNonEmptyCause(f.cause), f.stall.Milliseconds(), f.stallBy, nativeStop)
	case JogEndSessionStale:
		c.text = fmt.Sprintf("%s: the jog session was replaced or went stale. Pull the trigger again.", stopped)
		c.detail = fmt.Sprintf("cause=%s%s", f.cause, nativeStop)
	case JogEndTransportLost:
		c.text = fmt.Sprintf("%s: the link to the robot controller dropped. Pull the trigger again once it is back.", stopped)
		if cause == "ui_heartbeat_lost" || cause == "client_disconnected" {
			c.text = fmt.Sprintf("%s: this pendant or browser lost its link to the OLP server. Pull the trigger again once it is back.", stopped)
		}
		c.detail = fmt.Sprintf("cause=%s%s", f.cause, nativeStop)
	case JogEndHalted:
		// Stop and Disarm disarm; a native public Halt stops along the path
		// and leaves the robot armed.
		c.repull, c.disarmed, c.popup = RepullAfterRearm, true, f.endedByOther != ""
		switch {
		case cause == "halt" || cause == "jog_closed":
			c.text, c.repull, c.disarmed, c.popup = "Jog stopped: another rt-control client halted the robot. Pull the trigger again.", RepullYes, false, true
		case cause == "selection_changed":
			c.text, c.popup = "Jog stopped: the selected cell changed, which stops the robot. Arm before jogging again.", true
		case cause == "disarm" && f.endedByOther != "":
			c.text = fmt.Sprintf("Jog stopped: %s disarmed the robot. Arm before jogging again.", f.endedByOther)
		case f.endedByOther != "":
			c.text = fmt.Sprintf("Jog stopped: %s pressed Stop. Arm before jogging again.", f.endedByOther)
		case cause == "disarm":
			c.text = "Jog stopped: the robot was disarmed."
		default:
			c.text = "Jog stopped: Stop was pressed."
		}
		c.detail = fmt.Sprintf("cause=%s%s", f.cause, nativeStop)
	case JogEndInhibited:
		why := nativeInhibitWords[cause]
		if why == "" {
			why = "rt-control inhibited the outputs"
		}
		c.text, c.repull, c.disarmed = fmt.Sprintf("%s and the robot disarmed: %s. Arm again before jogging.", stopped, why), RepullAfterRearm, true
		c.detail = fmt.Sprintf("cause=%s%s", f.cause, nativeStop)
	case JogEndGeneratorFault:
		if cause == "path_fault" {
			c.text, c.repull, c.disarmed = fmt.Sprintf("%s and the robot disarmed: the path generator faulted. Check the robot, then arm again.", stopped), RepullAfterRearm, true
			c.detail = fmt.Sprintf("cause=%s: native PathJog failed closed (stop bound overrun, playable end passed or an invalid target)%s", f.cause, nativeStop)
		} else {
			c.text = fmt.Sprintf("%s: the Cartesian path could not be computed or streamed. Pull the trigger again; if it repeats, restart the OLP server.", stopped)
			c.detail = fmt.Sprintf("cause=%s: OLP's resolver or path stream failed%s", f.cause, nativeStop)
		}
	case JogEndDriveFault:
		c.text, c.repull, c.disarmed = driveFaultText(f.driveFault, f.atStart), RepullAfterRearm, true
		c.detail = fmt.Sprintf("cause=%s: native ended the hold for a drive fault: %s;%s", f.cause, f.driveFault, nativeStop)
	case JogEndStartMismatch:
		c.text = "Jog could not start: the robot was still settling from the last stop. Pull the trigger again."
		c.detail = fmt.Sprintf("cause=%s: native refused sample 0, more than one count from the held target; nothing moved", f.cause)
	case JogEndRefusedAtStart:
		c.text, c.repull = fmt.Sprintf("Jog could not start (%s).", f.cause), RepullUnknown
		c.detail = fmt.Sprintf("cause=%s: refused while the hold was setting up%s", f.cause, nativeStop)
	default:
		c.text, c.repull = fmt.Sprintf("Jog stopped (%s). Pull the trigger again; report it if it repeats.", f.cause), RepullUnknown
		c.detail = fmt.Sprintf("cause=%s: no operator text for this cause%s", f.cause, nativeStop)
	}
	if f.endedByOther != "" && !strings.Contains(c.detail, "ended_by=") {
		c.detail += " ended_by=" + f.endedByOther
	}
	return c
}

// sameClient compares ids when both clients sent one, else their labels.
func sameClient(a, b Client) bool {
	if a.ID != "" && b.ID != "" {
		return a.ID == b.ID
	}
	return a.Label == b.Label
}

func firstNonEmptyCause(cause string) string {
	if cause == "" {
		return "release"
	}
	return cause
}

// recordJogEndLocked classifies the retiring hold, journals its end line
// and publishes it as the last jog end. Hold r.mu.
func (r *RTCoreController) recordJogEndLocked(h *jointJogHold, revision uint64, reason string) {
	now := time.Now()
	f := jogEndFacts{cause: reason, atStart: h.timer == nil, lifetime: h.lifetime, intentAge: now.Sub(h.last), jogAxis: h.intent.Axis}
	if by := r.jogEndBy; by.Label != "" && !sameClient(by, h.client) {
		f.endedByOther = by.Label
	}
	if reason == "" {
		f.clientCause = r.jogEndCause
	}
	r.jogEndBy, r.jogEndCause = Client{}, ""
	end := JogEnd{Kind: "joint", ClientID: h.client.ID, ClientLabel: h.client.Label, Revision: revision,
		At: now.UTC().Format(time.RFC3339Nano), at: now, Updates: h.sequence, LifetimeMS: h.lifetime.Milliseconds(),
		RTRoundTripMS: float64(r.roundTrip.Microseconds()) / 1e3, LastIntentAgeMS: f.intentAge.Milliseconds()}
	if !h.begun.IsZero() {
		end.DurationMS = now.Sub(h.begun).Milliseconds()
	}
	if h.lane != nil {
		end.Generation = h.lane.Generation()
	}
	if h.intent.trace != nil {
		f.jogAxis = -1
		end.Kind, end.Axis = "trace", "program"
	} else if c := h.intent.cartesian; c != nil {
		f.jogAxis = -1
		end.Kind, end.Frame = "cartesian", c.Frame
		if axis, direction, _, ok := singleAxis(c.Twist); ok {
			end.Axis = map[int]string{1: "+", -1: "-"}[direction] + axisNames[axis]
		}
	} else {
		end.Axis = fmt.Sprintf("%sJ%d", map[int]string{1: "+", -1: "-"}[h.intent.Direction], h.intent.Axis+1)
		end.SpeedUnit, end.DistanceUnit = "deg/s", "deg"
		if h.intent.Axis >= 0 && h.intent.Axis < len(r.description.Axes) {
			a := r.description.Axes[h.intent.Axis]
			end.RequestedSpeed = degrees(h.intent.Fraction * a.MaxVelocity * jointMoveVelocityReserve)
			f.lower, f.upper = make([]float64, len(r.description.Axes)), make([]float64, len(r.description.Axes))
			for i, x := range r.description.Axes {
				f.lower[i], f.upper[i] = x.MinPosition, x.MaxPosition
			}
			if s := r.observation; s != nil && h.intent.Axis < len(s.Axes) {
				f.q = make([]float64, len(s.Axes))
				for i := range s.Axes {
					f.q[i] = s.Axes[i].LogicalPosition
				}
				if h.startValid {
					end.Distance = degrees(math.Abs(s.Axes[h.intent.Axis].LogicalPosition - h.startQ))
				}
			}
		}
		end.AchievedSpeed = degrees(h.peakVelocity)
	}
	if p := h.path; p != nil {
		o := p.observation
		f.path, f.stopReason, f.governing, f.underruns = true, o.StopReason, o.Governing, o.Underruns
		p.geomMu.Lock()
		f.resolverEnd = p.endReason
		if p.sent > 0 {
			f.q = append([]float64(nil), p.last[:]...)
			f.lower, f.upper = append([]float64(nil), p.line.Lower[:]...), append([]float64(nil), p.line.Upper[:]...)
		}
		p.geomMu.Unlock()
		if o.Received > o.Consumed {
			f.ahead = o.Received - o.Consumed
		}
		if !p.stallSince.IsZero() {
			f.stall, f.stallBy = now.Sub(p.stallSince), p.stallBy
		}
		ahead := f.ahead
		end.Governing, end.SamplesAhead = pathName(pathGoverningNames, o.Governing), &ahead
		end.SpeedUnit, end.DistanceUnit = p.unit+"/s", p.unit
		end.RequestedSpeed, end.AchievedSpeed, end.Distance = p.lastSpeed*p.scale(), p.maxSpeed*p.scale(), o.PathS*p.scale()
	} else if end.Kind == "cartesian" {
		end.SpeedUnit, end.DistanceUnit = "mm/s", "mm"
	}
	f.driveFault = driveFaultWords(r.observation, r.axisIDsLocked())
	c := classifyJogEnd(f)
	end.Code, end.Cause, end.Text, end.Detail, end.Repull, end.Popup, end.Disarmed, end.AtStart =
		c.code, firstNonEmptyCause(reason), c.text, c.detail, c.repull, c.popup, c.disarmed, f.atStart
	end.EndedBy, end.ClientCause = f.endedByOther, f.clientCause
	r.jogEndSeq++
	end.Seq = r.jogEndSeq
	r.lastJogEnd = &end
	logJogHoldEnd(h, end)
}

// noteJogEndDisarmedLocked marks the last end as disarmed when its End was
// uncertain and the controller's Stop fence disarmed the robot. Hold r.mu.
func (r *RTCoreController) noteJogEndDisarmedLocked(seq uint64) {
	if e := r.lastJogEnd; e != nil && e.Seq == seq && !e.Disarmed {
		end := *e
		end.Disarmed, end.Popup = true, true
		if end.Repull == RepullYes || end.Repull == RepullUnknown {
			end.Repull = RepullAfterRearm
		}
		end.Text = strings.TrimSuffix(end.Text, ".") + ". The robot was disarmed: arm again before jogging."
		end.Detail += " end_uncertain=stop_fence_disarmed"
		r.lastJogEnd = &end
		log.Printf("jog hold: end seq=%d disarmed=true (the lane's End was uncertain; Stop disarmed the robot)", seq)
	}
}

// Unverified operator policy: a native fault reaches OLP's observation within
// one status poll and a round trip of the hold end it caused (a Rosie 1430
// arm, 2026-10-11 12:19:20: the lane reported jog_session_stale before the poll showed J5's
// Er41.1). An end older than this keeps its words.
const jogEndDriveFaultWindow = 3 * time.Second

// attributeJogEndToDriveFaultLocked rewords the last end when a fresh
// observation shows the drive fault that ended it: the operator saw "the jog
// session was replaced or went stale" for J5's locked-rotor alarm. Clients
// show a revised end of the same seq in place. Hold r.mu.
func (r *RTCoreController) attributeJogEndToDriveFaultLocked(s *control.Status) {
	e := r.lastJogEnd
	if e == nil || time.Since(e.at) > jogEndDriveFaultWindow || !driveFaultMayExplain(e.Code) {
		return
	}
	words := driveFaultWords(s, r.axisIDsLocked())
	if words == "" {
		return
	}
	end := *e
	end.Code, end.Text, end.Repull, end.Popup, end.Disarmed = JogEndDriveFault, driveFaultText(words, end.AtStart), RepullAfterRearm, true, true
	end.Detail += fmt.Sprintf(" reattributed_from=%s after_ms=%d: native ended the hold for a drive fault: %s", e.Code, time.Since(e.at).Milliseconds(), words)
	r.lastJogEnd = &end
	log.Printf("jog hold: end seq=%d code=%s (was %s, seen %d ms after the end): %s", end.Seq, JogEndDriveFault, e.Code, time.Since(e.at).Milliseconds(), words)
}

// axisIDsLocked: the described axis names (J1..Jn), by index. Hold r.mu.
func (r *RTCoreController) axisIDsLocked() []string {
	ids := make([]string, len(r.description.Axes))
	for i, a := range r.description.Axes {
		ids[i] = a.ID
	}
	return ids
}

// lastJogEndLocked is the published copy with its age. Hold r.mu.
func (r *RTCoreController) lastJogEndLocked() *JogEnd {
	if r.lastJogEnd == nil {
		return nil
	}
	end := *r.lastJogEnd
	end.AgeMS = time.Since(end.at).Milliseconds()
	return &end
}
