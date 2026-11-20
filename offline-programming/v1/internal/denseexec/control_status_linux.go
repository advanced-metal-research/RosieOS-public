//go:build linux

package denseexec

import "rosieos/rt-core/sdk/control"

// ControlStatusBody bounds the handle history on the pendant's frequent read
// without changing the observation or its authority/freshness evidence. A loop
// prepares a new native handle every cycle; sending its entire retired history
// made an observed 275-handle status 131 KB and exceeded the Deck's freshness
// deadline. Diagnostics retain the full Status body on the ordinary route.
//
// Retain the native motion's handle, even after completion/refusal, and the
// newest prepared handle. Native handle IDs increase within an incarnation.
// These are at most two records; all current execution/session/fault fields and
// both raw and logical axis observations remain unchanged.
func ControlStatusBody(body *StatusBody) *StatusBody {
	if body == nil || body.RTCore == nil || body.RTCore.Status == nil {
		return body
	}
	view := *body
	rt := *body.RTCore
	status := *rt.Status
	var motion, prepared *control.HandleRecord
	for i := range status.Execution.Handles {
		h := &status.Execution.Handles[i]
		if h.Handle != 0 && h.Handle == status.Motion.TrajectoryID {
			motion = h
		}
		if h.State == "prepared" && (prepared == nil || h.Handle > prepared.Handle) {
			prepared = h
		}
	}
	// A separate slice is essential: Status may share this history with its
	// observer and other clients. Filtering in place would destroy diagnostics.
	status.Execution.Handles = make([]control.HandleRecord, 0, 2)
	if motion != nil {
		status.Execution.Handles = append(status.Execution.Handles, *motion)
	}
	if prepared != nil && (motion == nil || prepared.Handle != motion.Handle) {
		status.Execution.Handles = append(status.Execution.Handles, *prepared)
	}
	rt.Status = &status
	view.RTCore = &rt
	return &view
}
