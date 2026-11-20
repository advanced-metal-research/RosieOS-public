package execution

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"reflect"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/targets"
)

// DeterministicClock is a no-I/O clock for adversarial gateway tests. It is
// deliberately explicit rather than tied to wall time.
type DeterministicClock struct {
	mu  sync.Mutex
	now time.Time
}

func NewDeterministicClock(now time.Time) *DeterministicClock {
	return &DeterministicClock{now: now.UTC()}
}

func (clock *DeterministicClock) Now() time.Time {
	clock.mu.Lock()
	defer clock.mu.Unlock()
	return clock.now
}

func (clock *DeterministicClock) Advance(duration time.Duration) {
	clock.mu.Lock()
	defer clock.mu.Unlock()
	clock.now = clock.now.Add(duration)
}

type DeterministicOperationLog struct {
	mu      sync.Mutex
	entries []string
}

func (log *DeterministicOperationLog) Record(entry string) {
	if log == nil {
		return
	}
	log.mu.Lock()
	defer log.mu.Unlock()
	log.entries = append(log.entries, entry)
}

func (log *DeterministicOperationLog) Entries() []string {
	if log == nil {
		return nil
	}
	log.mu.Lock()
	defer log.mu.Unlock()
	return append([]string(nil), log.entries...)
}

// DeterministicTestTransport is the only concrete ServerTransport in this
// module. It records values in memory, has no address/subject/socket field, and
// cannot contact a target.
type DeterministicTestTransport struct {
	mu               sync.Mutex
	clock            Clock
	log              *DeterministicOperationLog
	failures         map[TransportAction]error
	receiptOverrides map[TransportAction]ExecutionIdentity
	loaded           *ExecutionIdentity
}

func NewDeterministicTestTransport(
	clock Clock,
	log *DeterministicOperationLog,
) *DeterministicTestTransport {
	return &DeterministicTestTransport{
		clock:            clock,
		log:              log,
		failures:         map[TransportAction]error{},
		receiptOverrides: map[TransportAction]ExecutionIdentity{},
	}
}

func (*DeterministicTestTransport) Capabilities() ServerTransportCapabilities {
	return ServerTransportCapabilities{
		Available:          true,
		ServerOwned:        true,
		TestOnly:           true,
		CommandSchema:      RobotV4CommandSchema,
		LeaderProtocol:     RobotV4LeaderProtocol,
		PlanPreload:        true,
		ControlledStartArm: true,
		PlanPlay:           true,
		ProtectiveStop:     true,
		ProtectiveDisarm:   true,
		ProtectiveWeldOff:  true,
		RawEndpointExposed: false,
	}
}

func (transport *DeterministicTestTransport) Fail(
	action TransportAction,
	err error,
) {
	transport.mu.Lock()
	defer transport.mu.Unlock()
	transport.failures[action] = err
}

func (transport *DeterministicTestTransport) OverrideReceiptIdentity(
	action TransportAction,
	identity ExecutionIdentity,
) {
	transport.mu.Lock()
	defer transport.mu.Unlock()
	transport.receiptOverrides[action] = identity
}

func (transport *DeterministicTestTransport) Preload(
	_ context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	transport.mu.Lock()
	defer transport.mu.Unlock()
	transport.log.Record(string(ActionPreload))
	if err := transport.failures[ActionPreload]; err != nil {
		return TransportReceipt{}, err
	}
	if command.Action != ActionPreload ||
		command.AcceptedPlan == nil ||
		command.AcceptedPlan.PlanID != command.Identity.PlanID ||
		command.AcceptedPlan.PlanRevision != command.Identity.PlanRevision ||
		command.AcceptedPlan.ProgramID != command.Identity.ProgramID ||
		command.AcceptedPlan.ProgramRevision !=
			command.Identity.ProgramRevision ||
		command.AcceptedPlan.ProgramDigest != command.Identity.ProgramDigest ||
		command.AcceptedPlan.TrajectoryDigest !=
			command.Identity.TrajectoryDigest ||
		command.AcceptedPlan.TargetID != command.Identity.Target.ProfileID ||
		command.AcceptedPlan.ManifestRevision !=
			command.Identity.Target.ManifestRevision {
		return TransportReceipt{}, errors.New("deterministic transport: invalid preload")
	}
	identity := command.Identity
	transport.loaded = &identity
	return transport.receipt(command, "preloaded"), nil
}

func (transport *DeterministicTestTransport) Arm(
	_ context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	transport.mu.Lock()
	defer transport.mu.Unlock()
	transport.log.Record(string(ActionArm))
	if err := transport.failures[ActionArm]; err != nil {
		return TransportReceipt{}, err
	}
	if command.Action != ActionArm ||
		transport.loaded == nil ||
		!transport.loaded.Equal(command.Identity) {
		return TransportReceipt{}, errors.New(
			"deterministic transport: controlled arm before exact preload",
		)
	}
	return transport.receipt(command, "armed_for_reviewed_start"), nil
}

func (transport *DeterministicTestTransport) Play(
	_ context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	transport.mu.Lock()
	defer transport.mu.Unlock()
	transport.log.Record(string(ActionPlay))
	if err := transport.failures[ActionPlay]; err != nil {
		return TransportReceipt{}, err
	}
	if command.Action != ActionPlay ||
		transport.loaded == nil ||
		!transport.loaded.Equal(command.Identity) {
		return TransportReceipt{}, errors.New("deterministic transport: play before exact preload")
	}
	return transport.receipt(command, "playing"), nil
}

func (transport *DeterministicTestTransport) Protect(
	_ context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	transport.mu.Lock()
	defer transport.mu.Unlock()
	transport.log.Record(string(command.Action))
	if command.Action != ActionStop &&
		command.Action != ActionDisarm &&
		command.Action != ActionWeldOff {
		return TransportReceipt{}, errors.New("deterministic transport: unsupported protective action")
	}
	if err := transport.failures[command.Action]; err != nil {
		return TransportReceipt{}, err
	}
	return transport.receipt(command, string(command.Action)+"_applied"), nil
}

func (transport *DeterministicTestTransport) receipt(
	command TransportCommand,
	result string,
) TransportReceipt {
	identity := command.Identity
	if override, ok := transport.receiptOverrides[command.Action]; ok {
		identity = override
	}
	return TransportReceipt{
		Schema:         TransportReceiptSchema,
		Action:         command.Action,
		CommandID:      command.Command.CommandID,
		IdempotencyKey: command.Command.IdempotencyKey,
		Accepted:       true,
		Applied:        true,
		Duplicate:      false,
		ServerOwned:    true,
		RecordedAt:     transport.clock.Now().UTC(),
		Identity:       identity,
		Result:         result,
	}
}

// DeterministicTestAuthority models lease state only in memory. It makes
// acquire/renew/release/cancel/fence ordering observable without implementing
// any Robot V4 sender.
type DeterministicTestAuthority struct {
	mu       sync.Mutex
	clock    Clock
	log      *DeterministicOperationLog
	active   targets.LeaseStatus
	sequence uint64
	failures map[string]error
}

func NewDeterministicTestAuthority(
	clock Clock,
	log *DeterministicOperationLog,
) *DeterministicTestAuthority {
	return &DeterministicTestAuthority{
		clock:    clock,
		log:      log,
		failures: map[string]error{},
	}
}

func (authority *DeterministicTestAuthority) Fail(operation string, err error) {
	authority.mu.Lock()
	defer authority.mu.Unlock()
	authority.failures[operation] = err
}

func (authority *DeterministicTestAuthority) Acquire(
	_ context.Context,
	request targets.LeaseAcquireRequest,
) (targets.LeaseStatus, error) {
	authority.mu.Lock()
	defer authority.mu.Unlock()
	authority.log.Record("lease_acquire")
	if err := authority.failures["lease_acquire"]; err != nil {
		return targets.LeaseStatus{}, err
	}
	if request.TargetProfileID == "" ||
		request.ControllerID == "" ||
		request.ControllerBootID == "" ||
		request.CampaignGeneration == 0 ||
		request.TTL < 500*time.Millisecond ||
		request.TTL > 2*time.Second {
		return targets.LeaseStatus{}, errors.New("deterministic authority: invalid acquire")
	}
	authority.sequence++
	authority.active = targets.LeaseStatus{
		TargetProfileID:    request.TargetProfileID,
		ControllerID:       request.ControllerID,
		CampaignGeneration: request.CampaignGeneration,
		Credential: targets.LeaseCredential{
			ControllerBootID: request.ControllerBootID,
			LeaseID:          fmt.Sprintf("memory-lease-%d", request.CampaignGeneration),
			FenceEpoch:       request.CampaignGeneration,
		},
		StatusSequence: authority.sequence,
		FreshUntil:     authority.clock.Now().Add(request.TTL),
	}
	return authority.active, nil
}

func (authority *DeterministicTestAuthority) Renew(
	_ context.Context,
	request targets.LeaseRenewRequest,
) (targets.LeaseStatus, error) {
	authority.mu.Lock()
	defer authority.mu.Unlock()
	authority.log.Record("lease_renew")
	if err := authority.failures["lease_renew"]; err != nil {
		return targets.LeaseStatus{}, err
	}
	if authority.active.TargetProfileID != request.TargetProfileID ||
		authority.active.ControllerID != request.ControllerID ||
		authority.active.CampaignGeneration != request.CampaignGeneration ||
		authority.active.Credential != request.Credential ||
		request.TTL < 500*time.Millisecond ||
		request.TTL > 2*time.Second {
		return targets.LeaseStatus{}, errors.New("deterministic authority: stale renew")
	}
	authority.sequence++
	authority.active.StatusSequence = authority.sequence
	authority.active.FreshUntil = authority.clock.Now().Add(request.TTL)
	return authority.active, nil
}

func (authority *DeterministicTestAuthority) Release(
	_ context.Context,
	request targets.LeaseReleaseRequest,
) error {
	authority.mu.Lock()
	defer authority.mu.Unlock()
	authority.log.Record("lease_release")
	if err := authority.failures["lease_release"]; err != nil {
		return err
	}
	if authority.active.TargetProfileID != request.TargetProfileID ||
		authority.active.ControllerID != request.ControllerID ||
		authority.active.CampaignGeneration != request.CampaignGeneration ||
		authority.active.Credential != request.Credential {
		return errors.New("deterministic authority: stale release")
	}
	authority.active = targets.LeaseStatus{}
	return nil
}

func (authority *DeterministicTestAuthority) Cancel(
	_ context.Context,
	request LeaseCancelRequest,
) error {
	authority.mu.Lock()
	defer authority.mu.Unlock()
	authority.log.Record("lease_cancel")
	if err := authority.failures["lease_cancel"]; err != nil {
		return err
	}
	if request.TargetProfileID == "" ||
		!request.Controller.Valid() ||
		request.CampaignGeneration == 0 {
		return errors.New("deterministic authority: invalid cancel")
	}
	if authority.active.CampaignGeneration == request.CampaignGeneration {
		authority.active = targets.LeaseStatus{}
	}
	return nil
}

func (authority *DeterministicTestAuthority) ValidateFence(
	_ context.Context,
	request targets.FenceValidationRequest,
) error {
	authority.mu.Lock()
	defer authority.mu.Unlock()
	authority.log.Record("fence_validate")
	if err := authority.failures["fence_validate"]; err != nil {
		return err
	}
	if authority.active != request.ObservedStatus ||
		authority.active.TargetProfileID != request.TargetProfileID ||
		authority.active.ControllerID != request.Controller.ControllerID ||
		authority.active.CampaignGeneration != request.CampaignGeneration ||
		authority.active.Credential != request.Credential ||
		request.CheckedAt.IsZero() ||
		!request.CheckedAt.Before(authority.active.FreshUntil) {
		return errors.New("deterministic authority: invalid fence")
	}
	return nil
}

// DeterministicTestEvidenceVerifier is an in-memory authoritative observation
// cache. It accepts only the exact snapshot installed by the test.
type DeterministicTestEvidenceVerifier struct {
	mu       sync.Mutex
	expected ExecutionReviewInput
	err      error
}

func NewDeterministicTestEvidenceVerifier(
	expected ExecutionReviewInput,
) *DeterministicTestEvidenceVerifier {
	verifier := &DeterministicTestEvidenceVerifier{}
	raw, err := json.Marshal(expected)
	if err == nil {
		err = json.Unmarshal(raw, &verifier.expected)
	}
	if err == nil {
		verifier.expected.PlanRequest.Program = append(
			json.RawMessage(nil),
			expected.PlanRequest.Program...,
		)
	}
	verifier.err = err
	return verifier
}

func (verifier *DeterministicTestEvidenceVerifier) Verify(
	_ context.Context,
	input ExecutionReviewInput,
) error {
	verifier.mu.Lock()
	defer verifier.mu.Unlock()
	if verifier.err != nil {
		return verifier.err
	}
	if !reflect.DeepEqual(verifier.expected, input) {
		return errors.New("deterministic evidence: snapshot was not server-injected")
	}
	return nil
}

func (verifier *DeterministicTestEvidenceVerifier) Fail(err error) {
	verifier.mu.Lock()
	defer verifier.mu.Unlock()
	verifier.err = err
}

type DeterministicTestAuditSink struct {
	mu        sync.Mutex
	envelopes []GatewayAuditEnvelope
	err       error
}

func (sink *DeterministicTestAuditSink) Append(
	_ context.Context,
	envelope GatewayAuditEnvelope,
) error {
	sink.mu.Lock()
	defer sink.mu.Unlock()
	if sink.err != nil {
		return sink.err
	}
	envelope.Reasons = append([]string(nil), envelope.Reasons...)
	sink.envelopes = append(sink.envelopes, envelope)
	return nil
}

func (sink *DeterministicTestAuditSink) Fail(err error) {
	sink.mu.Lock()
	defer sink.mu.Unlock()
	sink.err = err
}

func (sink *DeterministicTestAuditSink) Envelopes() []GatewayAuditEnvelope {
	sink.mu.Lock()
	defer sink.mu.Unlock()
	return append([]GatewayAuditEnvelope(nil), sink.envelopes...)
}

type DeterministicTestIdempotencyStore struct {
	mu        sync.Mutex
	reserved  map[string]struct{}
	committed map[string]struct{}
}

func NewDeterministicTestIdempotencyStore() *DeterministicTestIdempotencyStore {
	return &DeterministicTestIdempotencyStore{
		reserved:  map[string]struct{}{},
		committed: map[string]struct{}{},
	}
}

func (store *DeterministicTestIdempotencyStore) Reserve(
	_ context.Context,
	key string,
) (bool, error) {
	store.mu.Lock()
	defer store.mu.Unlock()
	if key == "" {
		return false, errors.New("deterministic idempotency: empty key")
	}
	if _, ok := store.committed[key]; ok {
		return true, nil
	}
	if _, ok := store.reserved[key]; ok {
		return false, errors.New("deterministic idempotency: key already reserved")
	}
	store.reserved[key] = struct{}{}
	return false, nil
}

func (store *DeterministicTestIdempotencyStore) Commit(
	_ context.Context,
	key string,
) error {
	store.mu.Lock()
	defer store.mu.Unlock()
	if _, ok := store.reserved[key]; !ok {
		return errors.New("deterministic idempotency: key was not reserved")
	}
	delete(store.reserved, key)
	store.committed[key] = struct{}{}
	return nil
}

func (store *DeterministicTestIdempotencyStore) Abort(
	_ context.Context,
	key string,
) error {
	store.mu.Lock()
	defer store.mu.Unlock()
	delete(store.reserved, key)
	return nil
}
