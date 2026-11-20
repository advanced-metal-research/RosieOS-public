package targets

import (
	"context"
	"errors"
	"strings"
	"time"
)

var ErrAuthorityDisabled = errors.New("targets: server authority adapter is disabled")

type LeaseCredential struct {
	ControllerBootID string `json:"controller_boot_id"`
	LeaseID          string `json:"lease_id"`
	FenceEpoch       uint64 `json:"fence_epoch"`
}

type LeaseAcquireRequest struct {
	TargetProfileID    string        `json:"target_profile_id"`
	ControllerID       string        `json:"controller_id"`
	ControllerBootID   string        `json:"controller_boot_id"`
	RequestID          string        `json:"request_id"`
	CampaignGeneration uint64        `json:"campaign_generation"`
	TTL                time.Duration `json:"ttl"`
}

type LeaseRenewRequest struct {
	TargetProfileID    string          `json:"target_profile_id"`
	ControllerID       string          `json:"controller_id"`
	RequestID          string          `json:"request_id"`
	CampaignGeneration uint64          `json:"campaign_generation"`
	TTL                time.Duration   `json:"ttl"`
	Credential         LeaseCredential `json:"credential"`
}

type LeaseReleaseRequest struct {
	TargetProfileID    string          `json:"target_profile_id"`
	ControllerID       string          `json:"controller_id"`
	RequestID          string          `json:"request_id"`
	CampaignGeneration uint64          `json:"campaign_generation"`
	Credential         LeaseCredential `json:"credential"`
}

type LeaseStatus struct {
	TargetProfileID    string          `json:"target_profile_id"`
	ControllerID       string          `json:"controller_id"`
	CampaignGeneration uint64          `json:"campaign_generation"`
	Credential         LeaseCredential `json:"credential"`
	StatusSequence     uint64          `json:"status_sequence"`
	FreshUntil         time.Time       `json:"fresh_until"`
	TransitionPending  bool            `json:"transition_pending"`
}

type AuthorityController interface {
	Acquire(context.Context, LeaseAcquireRequest) (LeaseStatus, error)
	Renew(context.Context, LeaseRenewRequest) (LeaseStatus, error)
	Release(context.Context, LeaseReleaseRequest) error
	Status(context.Context, string) (LeaseStatus, error)
}

// DisabledAuthorityController is the only authority implementation in this
// module. A future server adapter may implement AuthorityController, but the
// browser cannot mint credentials and this package cannot publish commands.
type DisabledAuthorityController struct{}

func (DisabledAuthorityController) Acquire(context.Context, LeaseAcquireRequest) (LeaseStatus, error) {
	return LeaseStatus{}, ErrAuthorityDisabled
}

func (DisabledAuthorityController) Renew(context.Context, LeaseRenewRequest) (LeaseStatus, error) {
	return LeaseStatus{}, ErrAuthorityDisabled
}

func (DisabledAuthorityController) Release(context.Context, LeaseReleaseRequest) error {
	return ErrAuthorityDisabled
}

func (DisabledAuthorityController) Status(context.Context, string) (LeaseStatus, error) {
	return LeaseStatus{}, ErrAuthorityDisabled
}

func (status LeaseStatus) Authorizes(
	now time.Time,
	targetProfileID string,
	controllerID string,
	credential LeaseCredential,
) bool {
	return !now.IsZero() &&
		now.Before(status.FreshUntil) &&
		!status.TransitionPending &&
		status.StatusSequence > 0 &&
		status.TargetProfileID == targetProfileID &&
		status.ControllerID == controllerID &&
		strings.TrimSpace(status.ControllerID) != "" &&
		status.Credential == credential &&
		strings.TrimSpace(credential.ControllerBootID) != "" &&
		strings.TrimSpace(credential.LeaseID) != "" &&
		credential.FenceEpoch > 0
}
