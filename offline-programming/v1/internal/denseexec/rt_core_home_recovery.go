//go:build linux

package denseexec

import (
	"errors"
	"rosieos/rt-core/sdk/control"
)

// This observation allowance is not reset authority. Native Reset independently
// requires fresh, coherent encoder acquisition while stopped and inhibited.
// Keep it tied to the pre-existing fault, selected joints and native incarnation.
type homeReferenceRecovery struct {
	requested, affected uint32
	adapter, daemon     string
	timeNS              uint64
}

func referenceRecoveryForHome(s control.Status, requested uint32) *homeReferenceRecovery {
	if requested == 0 || s.TimeNS == 0 || s.Motion.TimeNS == 0 || s.AdapterIncarnation == "" || s.DaemonIncarnation == "" ||
		s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 || s.Core.CommissioningPhase != 0 || s.Core.NativeHomeActiveMask != 0 ||
		s.Execution.Error != "" || s.Core.ExecutionFaultReasons != 1<<6 || s.Core.Recovery == nil || len(s.Core.Recovery.Faults) != 1 {
		return nil
	}
	fault := s.Core.Recovery.Faults[0]
	if fault.Bit != 6 || fault.Name != "coordinate_reference" || fault.Recovery != "rehome_required" ||
		fault.AxisMask == 0 || s.Core.SafetyFaultMask != fault.AxisMask || fault.AxisMask & ^requested != 0 || fault.RehomeAxisMask != fault.AxisMask {
		return nil
	}
	return &homeReferenceRecovery{requested: requested, affected: fault.AxisMask, adapter: s.AdapterIncarnation, daemon: s.DaemonIncarnation, timeNS: s.TimeNS}
}
func (h *homeReferenceRecovery) allows(s control.Status) bool {
	if h == nil {
		return false
	}
	current := referenceRecoveryForHome(s, h.requested)
	return current != nil && current.adapter == h.adapter && current.daemon == h.daemon && current.timeNS >= h.timeNS && current.affected & ^h.affected == 0
}
func (h *homeReferenceRecovery) allowsAxis(s control.Status, axis int) bool {
	return axis >= 0 && axis < 32 && h.allows(s) && h.affected&(1<<axis) != 0
}

func retiredHomeResetGrant(err error) bool {
	var rejected *control.Rejected
	return errors.As(err, &rejected) && rejected.Reason == "control_session_stale"
}
func (h *homeReferenceRecovery) expectedClear(e control.Event) bool {
	return h != nil && e.Type == "fault_cleared" && e.Fault != nil && e.Fault.Bit == 1<<6 &&
		e.Fault.RecoveryClass == "rehome_required" && e.Fault.AxisMask != 0 && e.Fault.AxisMask & ^h.affected == 0 &&
		e.TimeNS >= h.timeNS && e.AdapterIncarnation == h.adapter && e.DaemonIncarnation == h.daemon
}

func historicalHomeExecution(execution control.Execution) bool {
	return execution.Error != "" && (execution.State == "faulted" || execution.State == "cancelled" || execution.State == "released")
}

var errObservationSuperseded = errors.New("observation belongs to previous authority")

type observedAuthority struct {
	epoch    uint64
	fence    control.Fence
	renewals <-chan control.Renewal
}

// Caller holds r.mu; channel identity distinguishes replacement keepers even
// when a server's session/generation representation is unchanged.
func (o observedAuthority) current(r *RTCoreController) bool {
	return r.held && r.epoch == o.epoch && r.fence == o.fence && r.renewals == o.renewals
}
