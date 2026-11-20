// Package targetplanning owns the server-only promotion boundary from a saved
// shared Robot V4 program to an immutable plan accepted by the selected
// physical Motion Server.
package targetplanning

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"reflect"
	"regexp"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/targets"
)

const (
	RequestSchema  = "offline-programming.target-plan-request.v1"
	EnvelopeSchema = "offline-programming.target-plan-envelope.v1"
	MaxPlanCount   = 256
)

var (
	ErrUnavailable       = errors.New("target planning: target adapter is unavailable")
	ErrTargetNotFound    = errors.New("target planning: selected target was not found")
	ErrInvalidRequest    = errors.New("target planning: request is invalid")
	ErrRejected          = errors.New("target planning: Motion Server rejected the program")
	ErrIdentityMismatch  = errors.New("target planning: accepted plan identity mismatch")
	ErrProgramNotSaved   = errors.New("target planning: saved program revision was not found")
	ErrProgramMismatch   = errors.New("target planning: submitted program differs from the saved revision")
	ErrIdempotencyReplay = errors.New("target planning: idempotency key was reused for different input")

	safeWireIdentity = regexp.MustCompile(`^[A-Za-z0-9._:/-]{8,160}$`)
)

type Clock interface {
	Now() time.Time
}

type systemClock struct{}

func (systemClock) Now() time.Time { return time.Now().UTC() }

// MutationIdentity never crosses to the browser as transport configuration.
// It binds retries and audit correlation to one exact saved program request.
type MutationIdentity struct {
	CorrelationID  string `json:"correlation_id"`
	IdempotencyKey string `json:"idempotency_key"`
}

type TargetSource interface {
	PlanningTarget(string) (targets.ConnectedManifestTarget, bool)
}

// ProgramAuthority resolves one immutable saved revision from the
// Motion Server-owned shared catalog. Physical planning never treats the
// browser request body as the authoritative copy of the program.
type ProgramAuthority interface {
	SavedProgram(context.Context, string, uint64) (json.RawMessage, error)
}

// Adapter is implemented by the cold, authenticated target transport. The
// selected Motion Server must return the final accepted-plan artifact; this
// service never mints a physical plan on its behalf.
type Adapter interface {
	Available(targets.ConnectedManifestTarget) bool
	Plan(
		context.Context,
		targets.ConnectedManifestTarget,
		planning.NormalizedPlanRequest,
		MutationIdentity,
	) (planning.RobotV4AcceptedPlan, error)
}

type Service struct {
	mu          sync.RWMutex
	targets     TargetSource
	programs    ProgramAuthority
	adapter     Adapter
	clock       Clock
	plans       map[string]planning.RobotV4AcceptedPlan
	requests    map[string]planning.NormalizedPlanRequest
	order       []string
	idempotency map[string]idempotencyRecord
}

type idempotencyRecord struct {
	Fingerprint string
	PlanID      string
}

type Options struct {
	Targets  TargetSource
	Programs ProgramAuthority
	Adapter  Adapter
	Clock    Clock
}

type PlanEnvelope struct {
	Schema string                       `json:"schema"`
	Plan   planning.RobotV4AcceptedPlan `json:"plan"`
}

type StoredPlan struct {
	Request planning.NormalizedPlanRequest
	Plan    planning.RobotV4AcceptedPlan
}

func NewService(options Options) *Service {
	clock := options.Clock
	if clock == nil {
		clock = systemClock{}
	}
	return &Service{
		targets:     options.Targets,
		programs:    options.Programs,
		adapter:     options.Adapter,
		clock:       clock,
		plans:       make(map[string]planning.RobotV4AcceptedPlan),
		requests:    make(map[string]planning.NormalizedPlanRequest),
		idempotency: make(map[string]idempotencyRecord),
	}
}

func (service *Service) Available(profileID string) bool {
	if service == nil || service.targets == nil ||
		service.programs == nil || service.adapter == nil {
		return false
	}
	target, ok := service.targets.PlanningTarget(strings.TrimSpace(profileID))
	return ok && service.adapter.Available(target)
}

func (service *Service) Submit(
	ctx context.Context,
	profileID string,
	request planning.NormalizedPlanRequest,
	mutation MutationIdentity,
) (planning.RobotV4AcceptedPlan, error) {
	if service == nil || service.targets == nil ||
		service.programs == nil || service.adapter == nil {
		return planning.RobotV4AcceptedPlan{}, ErrUnavailable
	}
	if err := validateMutation(mutation); err != nil {
		return planning.RobotV4AcceptedPlan{}, err
	}
	if err := request.Validate(); err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf("%w: %v", ErrInvalidRequest, err)
	}
	profileID = strings.TrimSpace(profileID)
	target, ok := service.targets.PlanningTarget(profileID)
	if !ok {
		return planning.RobotV4AcceptedPlan{}, ErrTargetNotFound
	}
	if request.Target.ProfileID != profileID ||
		request.Target.ManifestRef != target.Discovery.ManifestRef ||
		target.Discovery.Target.Kind != targets.TargetRealCell ||
		!target.Discovery.Target.Capabilities.Planning {
		return planning.RobotV4AcceptedPlan{}, ErrIdentityMismatch
	}
	if !productionProgram(request.Program) {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: physical target planning requires execution_mode production",
			ErrInvalidRequest,
		)
	}
	if !service.adapter.Available(target) {
		return planning.RobotV4AcceptedPlan{}, ErrUnavailable
	}
	savedProgram, err := service.programs.SavedProgram(
		ctx,
		request.ProgramID,
		request.ProgramRevision,
	)
	if err != nil {
		return planning.RobotV4AcceptedPlan{}, err
	}
	if !sameJSONDocument(savedProgram, request.Program) {
		return planning.RobotV4AcceptedPlan{}, ErrProgramMismatch
	}
	// From this point onwards, including the private NATS exchange and the
	// idempotency fingerprint, use the exact catalog-owned bytes.
	request.Program = append(json.RawMessage(nil), savedProgram...)
	if err := request.Validate(); err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: saved program revision is invalid: %v",
			ErrProgramMismatch,
			err,
		)
	}

	fingerprint := requestFingerprint(profileID, request, mutation.CorrelationID)
	service.mu.RLock()
	replay, replayed := service.idempotency[mutation.IdempotencyKey]
	if replayed && replay.Fingerprint == fingerprint {
		plan, present := service.plans[replay.PlanID]
		service.mu.RUnlock()
		if !present {
			return planning.RobotV4AcceptedPlan{}, ErrIdentityMismatch
		}
		return clonePlan(plan)
	}
	service.mu.RUnlock()
	if replayed {
		return planning.RobotV4AcceptedPlan{}, ErrIdempotencyReplay
	}

	plan, err := service.adapter.Plan(ctx, target, request, mutation)
	if err != nil {
		return planning.RobotV4AcceptedPlan{}, err
	}
	if err := plan.ValidateForRequest(
		request,
		planning.AcceptedPlanPhysicalTarget,
	); err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf("%w: %v", ErrIdentityMismatch, err)
	}
	if plan.Target.ManifestRevision != target.Binding.ManifestRevision {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: target manifest revision changed",
			ErrIdentityMismatch,
		)
	}
	if service.clock.Now().Before(plan.CreatedAt) {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: accepted plan creation time is in the future",
			ErrIdentityMismatch,
		)
	}

	service.mu.Lock()
	defer service.mu.Unlock()
	if replay, present := service.idempotency[mutation.IdempotencyKey]; present {
		if replay.Fingerprint != fingerprint {
			return planning.RobotV4AcceptedPlan{}, ErrIdempotencyReplay
		}
		stored, present := service.plans[replay.PlanID]
		if !present {
			return planning.RobotV4AcceptedPlan{}, ErrIdentityMismatch
		}
		return clonePlan(stored)
	}
	if previous, present := service.plans[plan.PlanID]; present &&
		(previous.TrajectoryDigest != plan.TrajectoryDigest ||
			previous.ProgramDigest != plan.ProgramDigest ||
			previous.PlanRevision != plan.PlanRevision) {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: target reused an immutable plan id",
			ErrIdentityMismatch,
		)
	}
	stored, err := clonePlan(plan)
	if err != nil {
		return planning.RobotV4AcceptedPlan{}, err
	}
	service.plans[stored.PlanID] = stored
	service.requests[stored.PlanID] = cloneRequest(request)
	service.order = append(service.order, stored.PlanID)
	service.idempotency[mutation.IdempotencyKey] = idempotencyRecord{
		Fingerprint: fingerprint,
		PlanID:      stored.PlanID,
	}
	service.pruneLocked()
	return clonePlan(stored)
}

func (service *Service) GetStored(planID string) (StoredPlan, bool) {
	if service == nil {
		return StoredPlan{}, false
	}
	planID = strings.TrimSpace(planID)
	service.mu.RLock()
	plan, planOK := service.plans[planID]
	request, requestOK := service.requests[planID]
	service.mu.RUnlock()
	if !planOK || !requestOK {
		return StoredPlan{}, false
	}
	planClone, err := clonePlan(plan)
	if err != nil {
		return StoredPlan{}, false
	}
	return StoredPlan{
		Request: cloneRequest(request),
		Plan:    planClone,
	}, true
}

func (service *Service) Get(planID string) (planning.RobotV4AcceptedPlan, bool) {
	if service == nil {
		return planning.RobotV4AcceptedPlan{}, false
	}
	planID = strings.TrimSpace(planID)
	service.mu.RLock()
	plan, ok := service.plans[planID]
	service.mu.RUnlock()
	if !ok {
		return planning.RobotV4AcceptedPlan{}, false
	}
	clone, err := clonePlan(plan)
	return clone, err == nil
}

func (service *Service) pruneLocked() {
	for len(service.order) > MaxPlanCount {
		evicted := service.order[0]
		service.order = service.order[1:]
		delete(service.plans, evicted)
		delete(service.requests, evicted)
		for key, record := range service.idempotency {
			if record.PlanID == evicted {
				delete(service.idempotency, key)
			}
		}
	}
}

func cloneRequest(
	request planning.NormalizedPlanRequest,
) planning.NormalizedPlanRequest {
	request.Program = append(json.RawMessage(nil), request.Program...)
	request.Rows = append([]planning.PlanRow(nil), request.Rows...)
	for index := range request.Rows {
		if request.Rows[index].RevPiIOSlot != nil {
			slot := *request.Rows[index].RevPiIOSlot
			request.Rows[index].RevPiIOSlot = &slot
		}
	}
	return request
}

func validateMutation(mutation MutationIdentity) error {
	if !safeWireIdentity.MatchString(strings.TrimSpace(mutation.CorrelationID)) ||
		!safeWireIdentity.MatchString(strings.TrimSpace(mutation.IdempotencyKey)) {
		return fmt.Errorf(
			"%w: bounded correlation and idempotency identities are required",
			ErrInvalidRequest,
		)
	}
	return nil
}

func productionProgram(raw json.RawMessage) bool {
	var document struct {
		ExecutionMode string `json:"execution_mode"`
	}
	return json.Unmarshal(raw, &document) == nil &&
		document.ExecutionMode == "production"
}

func sameJSONDocument(left json.RawMessage, right json.RawMessage) bool {
	var leftValue any
	var rightValue any
	return json.Unmarshal(left, &leftValue) == nil &&
		json.Unmarshal(right, &rightValue) == nil &&
		reflect.DeepEqual(leftValue, rightValue)
}

func requestFingerprint(
	profileID string,
	request planning.NormalizedPlanRequest,
	correlationID string,
) string {
	hash := sha256.New()
	_, _ = hash.Write([]byte(profileID))
	_, _ = hash.Write([]byte{0})
	_, _ = hash.Write([]byte(request.RequestID))
	_, _ = hash.Write([]byte{0})
	_, _ = hash.Write([]byte(correlationID))
	_, _ = hash.Write([]byte{0})
	_, _ = hash.Write(request.Program)
	return "sha256:" + hex.EncodeToString(hash.Sum(nil))
}

func clonePlan(
	plan planning.RobotV4AcceptedPlan,
) (planning.RobotV4AcceptedPlan, error) {
	raw, err := json.Marshal(plan)
	if err != nil {
		return planning.RobotV4AcceptedPlan{}, err
	}
	var clone planning.RobotV4AcceptedPlan
	if err := json.Unmarshal(raw, &clone); err != nil {
		return planning.RobotV4AcceptedPlan{}, err
	}
	return clone, nil
}
