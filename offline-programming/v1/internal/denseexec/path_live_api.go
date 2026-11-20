package denseexec

// PathLive is a Cartesian path hold as it runs (fixes A3/A5 of
// rt-core/docs/history/2026-10-10-singularity-and-jog-dropouts.md): the
// operator saw the speed collapse near the wrist singularity and read it as a
// jam, with nothing on screen saying why or how to get out. It reports what
// OLP already has -- the native path speed and PathGoverning, the resolver's
// per-sample singularity scale for the samples it streamed, the line end the
// resolver found ahead -- and the wrist bend against the singular angle the
// robot's URDF gives. It changes no motion. After the hold ends the last one
// stays (Active false) until the next hold or move, so a client can keep the
// proximity meter up at the stop.
type PathLive struct {
	Active   bool   `json:"active"`
	Revision uint64 `json:"revision"`
	// The hold: +X, -Rz; base or tool.
	Axis  string `json:"axis"`
	Frame string `json:"frame"`
	// Native path speed against the operator's requested speed, in SpeedUnit.
	SpeedUnit      string  `json:"speed_unit"`
	RequestedSpeed float64 `json:"requested_speed"`
	Speed          float64 `json:"speed"`
	// Governing: native PathGoverning. LimitKind refines a "cap" into
	// singularity or joint_reserve from the samples OLP streamed: command,
	// joint_velocity, curvature, jerk, singularity, joint_reserve,
	// lookahead, line_end, stopping or none. Limit is its operator words.
	Governing string `json:"governing"`
	LimitKind string `json:"limit_kind"`
	Limit     string `json:"limit"`
	// GateScale: the resolver's singularity scale at the current sample (1
	// clear, falling to 0 at its Jacobian gate, where the line ends);
	// MinGateScaleAhead over the samples streamed beyond it. Near: the
	// current or a streamed sample ahead is inside the resolver's slowing
	// band (scale < 1). The band is the resolver's own (sigma_min 0.05 ->
	// 0.02, marked unverified in cartesian_resolver.hpp); no threshold here.
	GateScale         *float64 `json:"gate_scale,omitempty"`
	MinGateScaleAhead *float64 `json:"min_gate_scale_ahead,omitempty"`
	Near              bool     `json:"near"`
	// J5 at the current sample, the nearest wrist-singular J5 from the
	// robot's URDF and the signed distance J5 - that angle (degrees).
	J5Deg            *float64 `json:"j5_deg,omitempty"`
	WristSingularDeg *float64 `json:"wrist_singular_deg,omitempty"`
	WristDistanceDeg *float64 `json:"wrist_distance_deg,omitempty"`
	// LineEnd: the resolver's reason the line ends ahead (jacobian_gate,
	// joint_limit, ...), once streamed; DistanceToEnd to the playable end in
	// DistanceUnit.
	LineEnd       string   `json:"line_end,omitempty"`
	DistanceUnit  string   `json:"distance_unit"`
	DistanceToEnd *float64 `json:"distance_to_end,omitempty"`
	// Escape: the reverse of the hold's direction (reversing out of the
	// 2026-10-10 jam worked); JointHint: the joint jog away from the
	// wrist-singular angle.
	Escape    string `json:"escape"`
	JointHint string `json:"joint_hint,omitempty"`
	// Text: the line a client shows, worded here.
	Text  string `json:"text"`
	AgeMS int64  `json:"age_ms"`
}
