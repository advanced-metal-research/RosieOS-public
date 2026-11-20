package targets

import (
	"context"
	"errors"
	"strings"
	"time"
)

var (
	ErrFenceValidationDisabled   = errors.New("targets: fence validation is disabled")
	ErrProtectiveActionsDisabled = errors.New("targets: protective action gateway is disabled")
)

type ControllerIdentity struct {
	ControllerID     string `json:"controller_id"`
	ControllerBootID string `json:"controller_boot_id"`
}

func (identity ControllerIdentity) Valid() bool {
	return strings.TrimSpace(identity.ControllerID) != "" &&
		strings.TrimSpace(identity.ControllerBootID) != "" &&
		len(identity.ControllerID) <= 96 &&
		len(identity.ControllerBootID) <= 96
}

type FenceValidationRequest struct {
	TargetProfileID    string             `json:"target_profile_id"`
	Controller         ControllerIdentity `json:"controller"`
	CampaignGeneration uint64             `json:"campaign_generation"`
	Credential         LeaseCredential    `json:"credential"`
	ObservedStatus     LeaseStatus        `json:"observed_status"`
	CheckedAt          time.Time          `json:"checked_at"`
}

// FenceValidator is a future authenticated server-side boundary. Observing a
// lease in a status stream is not equivalent to validating it for a command.
type FenceValidator interface {
	ValidateFence(context.Context, FenceValidationRequest) error
}

type DisabledFenceValidator struct{}

func (DisabledFenceValidator) ValidateFence(context.Context, FenceValidationRequest) error {
	return ErrFenceValidationDisabled
}

// ProtectiveActionKind intentionally contains only the non-energizing safety
// tail requested for future server integration. It is not a browser command
// contract and has no transport implementation in Offline Programming V1.
type ProtectiveActionKind string

const (
	ProtectiveStop    ProtectiveActionKind = "stop"
	ProtectiveDisarm  ProtectiveActionKind = "disarm"
	ProtectiveWeldOff ProtectiveActionKind = "weld_off"
)

type ProtectiveActionRequest struct {
	Kind            ProtectiveActionKind `json:"kind"`
	TargetProfileID string               `json:"target_profile_id"`
	Controller      ControllerIdentity   `json:"controller"`
	RequestID       string               `json:"request_id"`
	RequestedAt     time.Time            `json:"requested_at"`
	Reason          string               `json:"reason"`
}

type ProtectiveActionReceipt struct {
	RequestID  string    `json:"request_id"`
	Accepted   bool      `json:"accepted"`
	RecordedAt time.Time `json:"recorded_at"`
	Result     string    `json:"result"`
}

type ProtectiveActionGateway interface {
	Stop(context.Context, ProtectiveActionRequest) (ProtectiveActionReceipt, error)
	Disarm(context.Context, ProtectiveActionRequest) (ProtectiveActionReceipt, error)
	WeldOff(context.Context, ProtectiveActionRequest) (ProtectiveActionReceipt, error)
}

type DisabledProtectiveActionGateway struct{}

func (DisabledProtectiveActionGateway) Stop(
	context.Context,
	ProtectiveActionRequest,
) (ProtectiveActionReceipt, error) {
	return ProtectiveActionReceipt{Accepted: false, Result: "disabled"}, ErrProtectiveActionsDisabled
}

func (DisabledProtectiveActionGateway) Disarm(
	context.Context,
	ProtectiveActionRequest,
) (ProtectiveActionReceipt, error) {
	return ProtectiveActionReceipt{Accepted: false, Result: "disabled"}, ErrProtectiveActionsDisabled
}

func (DisabledProtectiveActionGateway) WeldOff(
	context.Context,
	ProtectiveActionRequest,
) (ProtectiveActionReceipt, error) {
	return ProtectiveActionReceipt{Accepted: false, Result: "disabled"}, ErrProtectiveActionsDisabled
}
