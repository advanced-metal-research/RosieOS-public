//go:build linux

package denseexec

import (
	"fmt"
	"log"
	"strings"
)

// Hold journal lines exist because the Cartesian analysis of a Rosie 1430 arm (2026-10-06)
// had to infer every hold's axis, frame and speed from stop dumps: nothing
// logged them. One begin and one end line per hold; Cartesian path holds add
// their own begin/speed/end evidence (cartesian_path_hold.go).

func formatRad(values []float64) string {
	parts := make([]string, len(values))
	for i, v := range values {
		parts[i] = fmt.Sprintf("%.6f", v)
	}
	return "[" + strings.Join(parts, ",") + "]"
}

// logJogHoldBeginLocked records the held command and the measured pose at the
// press. The measured pose is evidence only; nothing here feeds motion.
func (r *RTCoreController) logJogHoldBeginLocked(h *jointJogHold) {
	var target, measured []float64
	var ageMS []string
	if s := r.observation; s != nil {
		for _, a := range s.Axes {
			target = append(target, a.LogicalTarget)
			measured = append(measured, a.LogicalPosition)
			age := "na"
			if a.TimeNS != 0 && s.Core.TimeNS >= a.TimeNS {
				age = fmt.Sprintf("%.1f", float64(s.Core.TimeNS-a.TimeNS)/1e6)
			}
			ageMS = append(ageMS, age)
		}
	}
	in := h.intent
	kind := fmt.Sprintf("joint axis=%d direction=%d", in.Axis, in.Direction)
	if c := in.cartesian; c != nil {
		kind = fmt.Sprintf("cartesian frame=%s twist=%s", c.Frame, formatRad(c.Twist[:]))
	}
	log.Printf("jog hold: begin rev=%d session=%s target=%s %s fraction=%.3f lifetime_ms=%d remote=%v start_target_rad=%s start_measured_rad=%s pose_age_ms=[%s]",
		in.Revision, in.SessionID, in.TargetID, kind, in.Fraction, h.lifetime.Milliseconds(), r.cfg.Address != "",
		formatRad(target), formatRad(measured), strings.Join(ageMS, ","))
}

// logJogHoldEnd is the hold's one end line: the retirement cause (reason, an
// operator release is "release"), its operator end code and words (jog_end.go)
// and the evidence to judge it by. The leading fields keep their order for
// existing journal readers; detail and text never contain a double quote, so
// key="value" parses.
func logJogHoldEnd(h *jointJogHold, end JogEnd) {
	ahead := "na"
	if end.SamplesAhead != nil {
		ahead = fmt.Sprint(*end.SamplesAhead)
	}
	quote := func(s string) string { return `"` + strings.ReplaceAll(s, `"`, "'") + `"` }
	log.Printf("jog hold: end rev=%d kind=%s reason=%s duration_ms=%d updates=%d end_code=%s seq=%d client_id=%s client=%s ended_by=%s client_cause=%s generation=%d axis=%s frame=%s requested=%.1f achieved=%.1f speed_unit=%s distance=%.2f distance_unit=%s governing=%s samples_ahead=%s last_intent_age_ms=%d lifetime_ms=%d rt_rtt_ms=%.1f at_start=%v repull=%s popup=%v disarmed=%v text=%s detail=%s",
		end.Revision, end.Kind, end.Cause, end.DurationMS, h.sequence, end.Code, end.Seq, firstNonEmpty(end.ClientID, "none"),
		quote(end.ClientLabel), quote(end.EndedBy), firstNonEmpty(end.ClientCause, "none"), end.Generation, firstNonEmpty(end.Axis, "na"), firstNonEmpty(end.Frame, "na"),
		end.RequestedSpeed, end.AchievedSpeed, firstNonEmpty(end.SpeedUnit, "na"), end.Distance, firstNonEmpty(end.DistanceUnit, "na"),
		firstNonEmpty(end.Governing, "na"), ahead, end.LastIntentAgeMS, end.LifetimeMS, end.RTRoundTripMS, end.AtStart, end.Repull,
		end.Popup, end.Disarmed, quote(end.Text), quote(end.Detail))
}

func firstNonEmpty(values ...string) string {
	for _, v := range values {
		if v != "" {
			return v
		}
	}
	return ""
}
