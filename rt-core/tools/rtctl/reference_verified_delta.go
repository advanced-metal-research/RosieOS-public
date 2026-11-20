package rtctl

// Home is established only by drive homing (HM35). The removed current-pose
// zero key is refused by name, not read as a supported template field, so an
// old trial configuration cannot compile into a software-defined zero.
func rejectCurrentPoseReference(axis, profile object) {
	for _, members := range []object{axis, profile} {
		for _, m := range members {
			if m.key == "reference_capture" {
				reject("reference_capture is not supported: current-pose reference anchors are not accepted; establish Home with drive homing (HM35)")
			}
		}
	}
}

// This optional operation admission is separate from the immutable origin/scale
// identity. It re-establishes only an HM35-derived (v3) anchor and never alters
// the meaning of RestoreAnchor or defines a new zero. There is no delta cap:
// any restored position within the joint's hard limits is admissible, and
// native still requires encoder/PDO delta agreement, unchanged 607C, fresh
// inhibited acquisition and the hard-limit check. The removed
// maximum_delta_counts key is refused by name so an old capped configuration
// cannot be mistaken for the current policy.
func renderReferenceVerifiedDeltaRestore(axis, profile object) string {
	if profile.optional("reference_verified_delta_restore", nil) != nil {
		reject("reference_verified_delta_restore must be declared on the machine axis")
	}
	for _, m := range axis {
		if m.key == "reference_verified_delta_restore" && m.value == nil {
			reject("reference_verified_delta_restore must be an explicit object")
		}
	}
	value := axis.optional("reference_verified_delta_restore", nil)
	if value == nil {
		return ""
	}
	deltaRestore := asObject(value)
	for _, m := range deltaRestore {
		if m.key == "maximum_delta_counts" {
			reject("reference_verified_delta_restore.maximum_delta_counts was removed: verified-delta restore is bounded by the joint's hard limits, not a delta cap")
		}
	}
	knownFields(deltaRestore, "method")
	if deltaRestore.get("method") != "independent_absolute_delta_v1" {
		reject("unsupported verified-delta restore method")
	}
	coordinate := asObject(profile.get("coordinate_evidence"))
	if coordinate.optional("source", nil) != "a6ec_sdo" || axis.optional("encoder_battery", nil) != "present" || axis.optional("coordinate_evidence_mode", nil) != "multi_turn" || !truth(profile.optional("native_home", nil)) {
		reject("verified-delta restore requires battery-backed independent absolute native Home evidence")
	}
	return "independent_absolute_delta_v1"
}
