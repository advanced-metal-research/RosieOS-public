package rtctl

// trackingLagAllowanceMaximumMS is an unverified admission cap, not a
// measurement: an arm that trails its command by a second or more is not
// tracking, and an allowance that large would let the following-error fault
// ignore real loss of control. A Rosie 1430 arm's stop dumps 2026-10-06 measured ~27 ms.
const trackingLagAllowanceMaximumMS = 1000

// validateTrackingLagAllowance returns the machine-level tracking lag
// allowance in milliseconds, or "" when the machine omits the block.
//
// A loaded servo trails its commanded target by a near-constant time, so its
// tracking error is about |commanded velocity| x lag and a static bound is
// crossed by ordinary fast motion (a Rosie 1430 arm, 2026-10-06: J3's 0.35 degree
// max_target_lead at ~13 deg/s, the 0.02 rad following_error at ~42 deg/s).
// Natively the target-lead and following-error bounds become the configured
// static bound plus |commanded velocity| x this time. The value is one time for
// every axis: each axis's commanded velocity is already in its own counts/s
// (or rad/s), so no per-joint number or gear conversion is involved.
//
// Omission compiles no line: the static bounds apply and existing
// configuration identities are unchanged.
func validateTrackingLagAllowance(machine object) string {
	for _, member := range machine {
		if member.key == "tracking_lag_allowance" && member.value == nil {
			reject("tracking_lag_allowance must be an explicit object")
		}
	}
	value := machine.optional("tracking_lag_allowance", nil)
	if value == nil {
		return ""
	}
	block, ok := value.(object)
	if !ok {
		reject("tracking_lag_allowance must be an object")
	}
	knownFields(block, "lag_allowance_ms")
	return integer(block.get("lag_allowance_ms"), 1, trackingLagAllowanceMaximumMS, "tracking_lag_allowance lag_allowance_ms")
}
