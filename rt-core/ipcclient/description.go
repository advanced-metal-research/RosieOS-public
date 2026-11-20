//go:build linux

package ipcclient

import (
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
)

// ReferenceVerifiedDeltaRestorePolicy has no delta cap: native bounds the
// restored position by the joint's hard limits.
type ReferenceVerifiedDeltaRestorePolicy struct {
	Method string `json:"method"`
}

type AxisDescription struct {
	ReferenceVerifiedDeltaRestore *ReferenceVerifiedDeltaRestorePolicy `json:"reference_verified_delta_restore,omitempty"`
	BrakeOverrideReason           string                               `json:"brake_override_reason,omitempty"`
	Interpolation                 string                               `json:"interpolation"`
	Feedforward                   string                               `json:"feedforward"`
	Checks                        json.RawMessage                      `json:"checks"`
	Endpoint                      string                               `json:"endpoint"`
	MaxAcceleration               *float64                             `json:"max_acceleration,omitempty"`
	CompletionTolerance           float64                              `json:"completion_tolerance"`
	FeedbackFields                []string                             `json:"feedback_fields"`
	Index                         uint32                               `json:"index"`
	ID                            string                               `json:"id"`
	PositionUnit                  string                               `json:"position_unit"`
	CountsPerUnit                 float64                              `json:"counts_per_unit"`
	Sign                          int                                  `json:"sign"`
	VelocityCommandMapped         bool                                 `json:"velocity_command_mapped"`
	NativeHome                    bool                                 `json:"native_home"`
	RequireHome                   bool                                 `json:"require_home"`
	MinPosition                   float64                              `json:"min_position"`
	MaxPosition                   float64                              `json:"max_position"`
	MaxVelocity                   float64                              `json:"max_velocity"`
	JogAcceleration               float64                              `json:"jog_acceleration"`
	MaxTargetLead                 float64                              `json:"max_target_lead"`
	FollowingError                float64                              `json:"following_error"`
	FollowingErrorTimeoutNS       uint64                               `json:"following_error_timeout_ns"`
	CompletionTimeoutNS           uint64                               `json:"completion_timeout_ns"`
}

// PathJogCapability is advertised only when the machine configures jog_jerk:
// the path generator needs that one jerk bound, so a cell without it omits
// the capability and clients refuse path holds instead of falling back to a
// velocity jog that bends the path.
type PathJogCapability struct {
	MaxSamples     uint32  `json:"max_samples"`
	MaxChunk       uint32  `json:"max_chunk"`
	MaxSampleStepS float64 `json:"max_sample_step_s"`
}

type Description struct {
	PathJog             *PathJogCapability `json:"path_jog,omitempty"`
	IO                  json.RawMessage    `json:"io,omitempty"`
	MaxGrantLeaseNS     uint64             `json:"max_grant_lease_ns"`
	MaxJogInputAgeNS    uint64             `json:"max_jog_input_age_ns"`
	Bus                 json.RawMessage    `json:"bus"`
	Backend             string             `json:"backend"`
	Schema              string             `json:"schema"`
	ProtocolMajor       uint32             `json:"protocol_major"`
	ProtocolMinor       uint32             `json:"protocol_minor"`
	ConfigurationSHA256 string             `json:"configuration_sha256"`
	MachineSHA256       string             `json:"machine_sha256"`
	DeploymentSHA256    string             `json:"deployment_sha256"`
	MaxTrajectoryPoints uint32             `json:"max_trajectory_points"`
	ResidentPlans       uint32             `json:"resident_plans"`
	CycleNS             uint64             `json:"cycle_ns"`
	Interpolation       string             `json:"interpolation"`
	Stop                string             `json:"stop"`
	Axes                []AxisDescription  `json:"axes"`
}

func (c *Client) Describe() (Description, error) {
	var description Description
	if _, err := c.Status(); err != nil {
		return description, err
	}
	raw, err := os.ReadFile(filepath.Join(filepath.Dir(c.options.SocketPath), "description.json"))
	if err != nil {
		return description, err
	}
	if err := json.Unmarshal(raw, &description); err != nil {
		return description, err
	}
	if description.Schema != "rosie.rt-core.description.v1" || description.Backend != c.options.Backend || description.ProtocolMajor != VersionMajor ||
		description.ProtocolMinor != VersionMinor || description.ConfigurationSHA256 != c.options.ConfigurationSHA256 ||
		len(description.Axes) != int(c.axes) {
		return Description{}, errors.New("RTCore description identity/version mismatch")
	}
	if !validIdentityDigest(description.MachineSHA256) || !validIdentityDigest(description.DeploymentSHA256) ||
		(c.options.MachineSHA256 != "" && description.MachineSHA256 != c.options.MachineSHA256) {
		return Description{}, errors.New("RTCore machine/deployment identity mismatch")
	}
	return description, nil
}
