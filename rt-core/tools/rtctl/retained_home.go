package rtctl

// Linear multi-turn profiles use the native default retained-Home policy.
// The retired position_semantics block no longer alters it.
func renderRetainedHomeRestore(axis, profile object) string {
	coordinate := profile.optional("coordinate_evidence", nil)
	if coordinate == nil || asObject(coordinate).optional("source", "pdo") != "a6ec_sdo" || axis.optional("encoder_battery", nil) == "absent" {
		return ""
	}
	return ""
}
