package targets

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"time"
)

var ErrCommandExecutionDisabled = errors.New("targets: real command execution is disabled")

type CommandKind string

const (
	CommandUploadPlan CommandKind = "upload_plan"
	CommandPlay       CommandKind = "play"
	CommandZeroHold   CommandKind = "zero_hold"
	CommandStop       CommandKind = "stop"
	CommandArm        CommandKind = "arm"
	CommandDisarm     CommandKind = "disarm"
	CommandWeldOn     CommandKind = "weld_on"
	CommandWeldOff    CommandKind = "weld_off"
)

type CommandIdentity struct {
	CommandID        string    `json:"command_id"`
	IdempotencyKey   string    `json:"idempotency_key"`
	IntentSequence   uint64    `json:"intent_sequence"`
	ControllerBootID string    `json:"controller_boot_id"`
	CreatedAt        time.Time `json:"created_at"`
}

type CommandPlanIdentity struct {
	PlanID          string `json:"plan_id"`
	PlanRevision    uint64 `json:"plan_revision"`
	ProgramID       string `json:"program_id"`
	ProgramRevision uint64 `json:"program_revision"`
	ProgramHash     string `json:"program_hash"`
}

type CommandRequest struct {
	Kind            CommandKind         `json:"kind"`
	TargetProfileID string              `json:"target_profile_id"`
	Service         string              `json:"service"`
	ControllerID    string              `json:"controller_id"`
	Identity        CommandIdentity     `json:"identity"`
	Plan            CommandPlanIdentity `json:"plan"`
	Credential      *LeaseCredential    `json:"credential,omitempty"`
}

type CommandReceipt struct {
	CommandID      string    `json:"command_id"`
	IdempotencyKey string    `json:"idempotency_key"`
	Accepted       bool      `json:"accepted"`
	Duplicate      bool      `json:"duplicate"`
	RecordedAt     time.Time `json:"recorded_at"`
	Result         string    `json:"result"`
}

type CommandGateway interface {
	Submit(context.Context, CommandRequest) (CommandReceipt, error)
}

type IdempotencyStore interface {
	Reserve(context.Context, string) (alreadyCommitted bool, err error)
	Commit(context.Context, string) error
	Abort(context.Context, string) error
}

type DisabledCommandGateway struct{}

func (DisabledCommandGateway) Submit(context.Context, CommandRequest) (CommandReceipt, error) {
	return CommandReceipt{Accepted: false, Result: "disabled"}, ErrCommandExecutionDisabled
}

func CommandIdempotencyKey(targetProfileID, service, commandID string) string {
	return targetProfileID + "+" + service + "+" + commandID
}

func (request CommandRequest) Validate(now time.Time, authority LeaseStatus) error {
	if !supportedCommand(request.Kind) {
		return errors.New("targets: unsupported command kind")
	}
	if strings.TrimSpace(request.TargetProfileID) == "" ||
		strings.TrimSpace(request.Service) == "" ||
		strings.TrimSpace(request.ControllerID) == "" {
		return errors.New("targets: command target, service, and controller are required")
	}
	if strings.TrimSpace(request.Identity.CommandID) == "" ||
		request.Identity.IntentSequence == 0 ||
		strings.TrimSpace(request.Identity.ControllerBootID) == "" ||
		request.Identity.CreatedAt.IsZero() ||
		now.Before(request.Identity.CreatedAt) {
		return errors.New("targets: command identity is incomplete")
	}
	expectedKey := CommandIdempotencyKey(
		request.TargetProfileID,
		request.Service,
		request.Identity.CommandID,
	)
	if request.Identity.IdempotencyKey != expectedKey {
		return fmt.Errorf("targets: idempotency key must be %q", expectedKey)
	}
	if commandNeedsPlan(request.Kind) {
		if strings.TrimSpace(request.Plan.PlanID) == "" ||
			request.Plan.PlanRevision == 0 ||
			strings.TrimSpace(request.Plan.ProgramID) == "" ||
			request.Plan.ProgramRevision == 0 ||
			strings.TrimSpace(request.Plan.ProgramHash) == "" {
			return errors.New("targets: exact plan identity is required")
		}
	}
	if commandIsEnergizing(request.Kind) {
		if request.Credential == nil ||
			request.Credential.ControllerBootID != request.Identity.ControllerBootID ||
			!authority.Authorizes(now, request.TargetProfileID, request.ControllerID, *request.Credential) {
			return errors.New("targets: fresh matching server lease and fence are required")
		}
	}
	return nil
}

func commandNeedsPlan(kind CommandKind) bool {
	return kind == CommandUploadPlan || kind == CommandPlay
}

func commandIsEnergizing(kind CommandKind) bool {
	return kind == CommandUploadPlan ||
		kind == CommandPlay ||
		kind == CommandArm ||
		kind == CommandWeldOn
}

func supportedCommand(kind CommandKind) bool {
	switch kind {
	case CommandUploadPlan, CommandPlay, CommandZeroHold, CommandStop,
		CommandArm, CommandDisarm, CommandWeldOn, CommandWeldOff:
		return true
	default:
		return false
	}
}
