package targets

import (
	"context"
	"errors"
	"math"
	"strings"
	"time"
)

var ErrTeleopDisabled = errors.New("targets: tele-op capability is disabled")

type TeleopCapability struct {
	Schema              string `json:"schema"`
	Enabled             bool   `json:"enabled"`
	Reason              string `json:"reason"`
	ServerOwned         bool   `json:"server_owned"`
	RawBrowserTransport bool   `json:"raw_browser_transport"`
}

func DisabledTeleopCapability() TeleopCapability {
	return TeleopCapability{
		Schema:              "offline-programming.teleop-capability.v1",
		Enabled:             false,
		Reason:              "reserved_for_future_server_implementation",
		ServerOwned:         true,
		RawBrowserTransport: false,
	}
}

type HotCommand struct {
	TargetProfileID  string          `json:"target_profile_id"`
	ControllerID     string          `json:"controller_id"`
	CommandID        string          `json:"command_id"`
	InputSequence    uint64          `json:"input_sequence"`
	SampledAt        time.Time       `json:"sampled_at"`
	ExpiresAt        time.Time       `json:"expires_at"`
	DeadmanHeld      bool            `json:"deadman_held"`
	ReleaseZero      bool            `json:"release_zero"`
	CommandRateHz    float64         `json:"command_rate_hz"`
	AxisVelocityRadS [9]float64      `json:"axis_velocity_rad_s"`
	Credential       LeaseCredential `json:"credential"`
}

type HotCommandPolicy struct {
	MaximumInputAge    time.Duration `json:"maximum_input_age"`
	MaximumRateHz      float64       `json:"maximum_rate_hz"`
	MaximumAxisRadS    [9]float64    `json:"maximum_axis_rad_s"`
	RequireDeadman     bool          `json:"require_deadman"`
	RequireReleaseZero bool          `json:"require_release_zero"`
}

func ValidateHotCommand(
	command HotCommand,
	policy HotCommandPolicy,
	now time.Time,
	authority LeaseStatus,
) error {
	if strings.TrimSpace(command.CommandID) == "" || command.InputSequence == 0 {
		return errors.New("targets: hot command identity is incomplete")
	}
	if now.IsZero() || command.SampledAt.IsZero() || command.ExpiresAt.IsZero() ||
		now.Before(command.SampledAt) || !now.Before(command.ExpiresAt) ||
		policy.MaximumInputAge <= 0 || now.Sub(command.SampledAt) > policy.MaximumInputAge {
		return errors.New("targets: hot command input is stale")
	}
	if policy.MaximumRateHz <= 0 ||
		command.CommandRateHz <= 0 ||
		command.CommandRateHz > policy.MaximumRateHz {
		return errors.New("targets: hot command rate exceeds policy")
	}
	if policy.RequireDeadman && !command.DeadmanHeld && !command.ReleaseZero {
		return errors.New("targets: hot command deadman is not held")
	}
	if policy.RequireReleaseZero && !command.ReleaseZero && !command.DeadmanHeld {
		return errors.New("targets: deadman release requires a zero command")
	}
	for index, velocity := range command.AxisVelocityRadS {
		limit := policy.MaximumAxisRadS[index]
		if math.IsNaN(velocity) || math.IsInf(velocity, 0) ||
			limit <= 0 || math.Abs(velocity) > limit ||
			(command.ReleaseZero && velocity != 0) {
			return errors.New("targets: hot command axis velocity exceeds policy")
		}
	}
	if !authority.Authorizes(
		now,
		command.TargetProfileID,
		command.ControllerID,
		command.Credential,
	) {
		return errors.New("targets: hot command requires a fresh matching server lease and fence")
	}
	return nil
}

// HotCommandService is a future server-internal boundary. No implementation
// here opens UDP/NATS or accepts browser motion packets.
type HotCommandService interface {
	Submit(context.Context, HotCommand) error
	Release(context.Context, string, string, LeaseCredential) error
}

type DisabledHotCommandService struct{}

func (DisabledHotCommandService) Submit(context.Context, HotCommand) error {
	return ErrTeleopDisabled
}

func (DisabledHotCommandService) Release(context.Context, string, string, LeaseCredential) error {
	return ErrTeleopDisabled
}
