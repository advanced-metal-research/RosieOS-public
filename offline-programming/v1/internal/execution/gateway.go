package execution

import (
	"context"
	"errors"
	"fmt"
	"strconv"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/targets"
)

type storedReviewToken struct {
	token    ReviewToken
	consumed bool
}

type storedStartRequest struct {
	reviewToken string
	identity    ExecutionIdentity
	session     ExecutionSession
}

type Gateway struct {
	config GatewayConfig
	clock  Clock

	operationMu   sync.Mutex
	stateMu       sync.Mutex
	supervisorWG  sync.WaitGroup
	supervisor    context.Context
	cancelAll     context.CancelFunc
	nextID        uint64
	nextIntent    uint64
	nextCampaign  uint64
	tokens        map[string]storedReviewToken
	sessions      map[string]ExecutionSession
	startRequests map[string]storedStartRequest
	sessionStops  map[string]context.CancelFunc
	closed        bool
}

func NewGateway(config GatewayConfig) *Gateway {
	clock := config.Clock
	if clock == nil {
		clock = systemClock{}
	}
	supervisor, cancelAll := context.WithCancel(context.Background())
	return &Gateway{
		config:        config,
		clock:         clock,
		supervisor:    supervisor,
		cancelAll:     cancelAll,
		nextID:        1,
		nextIntent:    1,
		nextCampaign:  1,
		tokens:        map[string]storedReviewToken{},
		sessions:      map[string]ExecutionSession{},
		startRequests: map[string]storedStartRequest{},
		sessionStops:  map[string]context.CancelFunc{},
	}
}

func (gateway *Gateway) Capabilities() GatewayCapabilityReport {
	return capabilityReport(gateway.clock.Now(), gateway.config)
}

func (gateway *Gateway) Session(sessionID string) (ExecutionSession, bool) {
	return gateway.session(strings.TrimSpace(sessionID))
}

// Review re-evaluates every injected fact and issues a short-lived, one-use,
// server-stored token only after the approved review itself is audited.
func (gateway *Gateway) Review(
	ctx context.Context,
	input ExecutionReviewInput,
) (ExecutionReview, error) {
	now := gateway.clock.Now()
	review := reviewExecution(now, gateway.config, input)
	if err := gateway.verifyEvidence(ctx, input); err != nil {
		denyReview(&review, "authoritative_evidence_verification_failed", err.Error())
	}
	if !review.Ready {
		return review, reviewError(review)
	}

	token := ReviewToken{
		Schema:              ReviewTokenSchema,
		TokenID:             gateway.identifier("review"),
		IssuedAt:            now.UTC(),
		ExpiresAt:           now.Add(gateway.config.Bootstrap.ReviewTTL).UTC(),
		Controller:          gateway.config.Bootstrap.Controller,
		Identity:            input.Identity,
		StatusSequence:      input.Status.Sequence,
		RuntimePlanSequence: input.RuntimePlan.Sequence,
		EvidenceRequestID:   input.Evidence.RequestID,
	}
	gateway.stateMu.Lock()
	gateway.tokens[token.TokenID] = storedReviewToken{token: token}
	gateway.stateMu.Unlock()

	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "review",
		Action:        "review",
		Outcome:       "approved",
		TransportUsed: false,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Reasons:       []string{"all_execution_prerequisites_proved"},
	}); err != nil {
		gateway.stateMu.Lock()
		delete(gateway.tokens, token.TokenID)
		gateway.stateMu.Unlock()
		review.Ready = false
		review.Reasons = appendUnique(review.Reasons, "review_audit_failed")
		review.Checks = append(review.Checks, ReviewCheck{
			Code:   "review_audit_failed",
			Passed: false,
			Detail: err.Error(),
		})
		return review, fmt.Errorf("%w: review audit: %v", ErrUnavailable, err)
	}
	review.Token = &token
	return review, nil
}

// Start consumes a review token, acquires and validates a fresh lease/fence,
// preloads the exact reviewed plan, performs the internal controlled-start
// arm, renews the lease, then plays. There is no standalone browser Arm,
// Weld-On, target-enable, generic-command, or raw transport method.
func (gateway *Gateway) Start(
	ctx context.Context,
	request StartRequest,
) (ExecutionSession, error) {
	gateway.operationMu.Lock()
	defer gateway.operationMu.Unlock()

	gateway.stateMu.Lock()
	closed := gateway.closed
	gateway.stateMu.Unlock()
	if closed {
		return ExecutionSession{}, ErrUnavailable
	}
	if !safeASCII(request.RequestID, 128) {
		return ExecutionSession{}, fmt.Errorf("%w: request id is invalid", ErrReviewDenied)
	}
	if session, replay, err := gateway.startReplay(request); replay {
		return session, err
	}
	now := gateway.clock.Now()
	review := reviewExecution(now, gateway.config, request.Review)
	if err := gateway.verifyEvidence(ctx, request.Review); err != nil {
		denyReview(&review, "authoritative_evidence_verification_failed", err.Error())
	}
	if err := reviewError(review); err != nil {
		return ExecutionSession{}, err
	}
	token, err := gateway.consumeToken(now, request.ReviewToken, request.Review)
	if err != nil {
		return ExecutionSession{}, err
	}

	sessionID := gateway.identifier("session")
	campaign := gateway.campaign()
	controller := gateway.config.Bootstrap.Controller
	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "authority",
		Action:        "lease_acquire",
		Outcome:       "attempt",
		TransportUsed: false,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    controller,
		Identity:      token.Identity,
		Reasons:       []string{"one_use_review_token_consumed"},
	}); err != nil {
		return ExecutionSession{}, fmt.Errorf("%w: lease acquire audit: %v", ErrUnavailable, err)
	}

	acquireRequest := targets.LeaseAcquireRequest{
		TargetProfileID:    token.Identity.Target.ProfileID,
		ControllerID:       controller.ControllerID,
		ControllerBootID:   controller.ControllerBootID,
		RequestID:          gateway.identifier("acquire"),
		CampaignGeneration: campaign,
		TTL:                gateway.config.Bootstrap.LeaseTTL,
	}
	lease, acquireErr := gateway.config.Authority.Acquire(ctx, acquireRequest)
	if acquireErr != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, targets.LeaseStatus{},
			"lease_acquire_failed",
		)
		return ExecutionSession{}, errors.Join(
			fmt.Errorf("execution: acquire lease: %w", acquireErr),
			fallbackErr,
		)
	}
	if err := gateway.validateAndFence(ctx, now, token.Identity, campaign, lease); err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, lease,
			"acquired_lease_or_fence_invalid",
		)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}
	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "authority",
		Action:        "lease_acquire",
		Outcome:       "validated",
		TransportUsed: false,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    controller,
		Identity:      token.Identity,
		Lease:         leasePointer(lease),
		Reasons:       []string{"exact_server_fence_validated"},
	}); err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, lease,
			"lease_audit_failed",
		)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}

	acceptedPlan := request.Review.AcceptedPlan.Reference()
	preloadReceipt, err := gateway.submitEnergizing(
		ctx,
		ActionPreload,
		sessionID,
		token,
		campaign,
		lease,
		&acceptedPlan,
	)
	if err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, lease,
			"plan_preload_failed",
		)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}

	renewed, err := gateway.renewLease(ctx, sessionID, token, campaign, lease)
	if err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, lease,
			"lease_renew_failed_before_play",
		)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}
	lease = renewed

	armReceipt, err := gateway.submitEnergizing(
		ctx,
		ActionArm,
		sessionID,
		token,
		campaign,
		lease,
		nil,
	)
	if err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, lease,
			"controlled_start_arm_failed",
		)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}

	renewed, err = gateway.renewLease(
		ctx,
		sessionID,
		token,
		campaign,
		lease,
	)
	if err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, lease,
			"lease_renew_failed_after_arm",
		)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}
	lease = renewed

	playReceipt, err := gateway.submitEnergizing(
		ctx,
		ActionPlay,
		sessionID,
		token,
		campaign,
		lease,
		nil,
	)
	if err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx, sessionID, token, campaign, lease,
			"plan_play_failed",
		)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}

	session := ExecutionSession{
		Schema:             SessionSchema,
		SessionID:          sessionID,
		StartedAt:          gateway.clock.Now().UTC(),
		Identity:           token.Identity,
		Controller:         controller,
		ReviewTokenID:      token.TokenID,
		CampaignGeneration: campaign,
		Lease:              lease,
		PreloadReceipt:     preloadReceipt,
		ArmReceipt:         armReceipt,
		PlayReceipt:        playReceipt,
	}
	gateway.stateMu.Lock()
	gateway.sessions[sessionID] = session
	gateway.startRequests[request.RequestID] = storedStartRequest{
		reviewToken: request.ReviewToken,
		identity:    request.Review.Identity,
		session:     session,
	}
	gateway.stateMu.Unlock()
	gateway.startSupervisor(sessionID)
	return session, nil
}

// startReplay makes the browser-facing Start operation idempotent across a
// lost HTTP response. Only the exact same request ID, one-use review token,
// and accepted-plan identity may recover the original server session.
func (gateway *Gateway) startReplay(
	request StartRequest,
) (ExecutionSession, bool, error) {
	gateway.stateMu.Lock()
	defer gateway.stateMu.Unlock()
	stored, ok := gateway.startRequests[request.RequestID]
	if !ok {
		return ExecutionSession{}, false, nil
	}
	if stored.reviewToken != request.ReviewToken ||
		!stored.identity.Equal(request.Review.Identity) {
		return ExecutionSession{}, true, fmt.Errorf(
			"%w: start request id was already used for a different reviewed plan",
			ErrIdempotencyReplay,
		)
	}
	return stored.session, true, nil
}

// Renew is used by the server-owned session supervisor. It is public within
// the package API so a process lifecycle test can also drive one renewal
// deterministically.
func (gateway *Gateway) Renew(
	ctx context.Context,
	sessionID string,
) (ExecutionSession, error) {
	gateway.operationMu.Lock()
	defer gateway.operationMu.Unlock()

	session, ok := gateway.session(sessionID)
	if !ok {
		return ExecutionSession{}, ErrSessionNotFound
	}
	token := ReviewToken{
		TokenID:    session.ReviewTokenID,
		Controller: session.Controller,
		Identity:   session.Identity,
	}
	lease, err := gateway.renewLease(
		ctx,
		session.SessionID,
		token,
		session.CampaignGeneration,
		session.Lease,
	)
	if err != nil {
		fallbackErr := gateway.protectAndClose(
			ctx,
			session.SessionID,
			token,
			session.CampaignGeneration,
			session.Lease,
			"session_lease_renew_failed",
		)
		gateway.deleteSession(session.SessionID)
		return ExecutionSession{}, errors.Join(err, fallbackErr)
	}
	session.Lease = lease
	gateway.stateMu.Lock()
	gateway.sessions[session.SessionID] = session
	gateway.stateMu.Unlock()
	return session, nil
}

// Stop always attempts the complete non-energizing Stop, Disarm, Weld-Off
// tail before releasing authority. Each action is independently attempted.
func (gateway *Gateway) Stop(ctx context.Context, request StopRequest) error {
	gateway.operationMu.Lock()
	defer gateway.operationMu.Unlock()

	if !safeASCII(request.RequestID, 128) {
		return fmt.Errorf("%w: stop request id is invalid", ErrReviewDenied)
	}
	session, ok := gateway.session(request.SessionID)
	if !ok {
		return ErrSessionNotFound
	}
	gateway.deleteSession(session.SessionID)
	token := ReviewToken{
		TokenID:    session.ReviewTokenID,
		Controller: session.Controller,
		Identity:   session.Identity,
	}
	reason := strings.TrimSpace(request.Reason)
	if reason == "" {
		reason = "operator_stop"
	} else if len(reason) > 160 {
		reason = "unbounded_stop_reason_redacted"
	}
	return gateway.protectAndClose(
		ctx,
		session.SessionID,
		token,
		session.CampaignGeneration,
		session.Lease,
		reason,
	)
}

// Close stops every active session before ending the supervisor. It is safe to
// call more than once and is intended for process shutdown.
func (gateway *Gateway) Close(ctx context.Context) error {
	if gateway == nil {
		return nil
	}
	gateway.stateMu.Lock()
	if gateway.closed {
		gateway.stateMu.Unlock()
		return nil
	}
	gateway.closed = true
	sessionIDs := make([]string, 0, len(gateway.sessions))
	for sessionID := range gateway.sessions {
		sessionIDs = append(sessionIDs, sessionID)
	}
	gateway.stateMu.Unlock()

	var failures []error
	for _, sessionID := range sessionIDs {
		err := gateway.Stop(ctx, StopRequest{
			RequestID: gateway.identifier("shutdown"),
			SessionID: sessionID,
			Reason:    "offline_programming_shutdown",
		})
		if err != nil && !errors.Is(err, ErrSessionNotFound) {
			failures = append(failures, err)
		}
	}
	gateway.cancelAll()
	done := make(chan struct{})
	go func() {
		gateway.supervisorWG.Wait()
		close(done)
	}()
	select {
	case <-done:
	case <-ctx.Done():
		failures = append(failures, ctx.Err())
	}
	return errors.Join(failures...)
}

func (gateway *Gateway) startSupervisor(sessionID string) {
	if gateway == nil ||
		!gateway.config.Features.SessionMonitoringEnabled ||
		gateway.config.Observer == nil {
		return
	}
	gateway.stateMu.Lock()
	if gateway.closed {
		gateway.stateMu.Unlock()
		return
	}
	ctx, cancel := context.WithCancel(gateway.supervisor)
	gateway.sessionStops[sessionID] = cancel
	gateway.supervisorWG.Add(1)
	gateway.stateMu.Unlock()
	go gateway.superviseSession(ctx, sessionID)
}

func (gateway *Gateway) superviseSession(
	ctx context.Context,
	sessionID string,
) {
	defer gateway.supervisorWG.Done()
	interval := gateway.config.Bootstrap.LeaseTTL / 4
	if interval < 100*time.Millisecond {
		interval = 100 * time.Millisecond
	}
	if interval > 500*time.Millisecond {
		interval = 500 * time.Millisecond
	}
	ticker := time.NewTicker(interval)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}
		timeout := gateway.config.Bootstrap.LeaseTTL / 2
		if timeout < 500*time.Millisecond {
			timeout = 500 * time.Millisecond
		}
		cycle, cancel := context.WithTimeout(ctx, timeout)
		session, err := gateway.Renew(cycle, sessionID)
		if err == nil {
			err = gateway.verifyActiveUntilDeadline(cycle, session)
		}
		cancel()
		if err == nil {
			continue
		}
		stopContext, stopCancel := context.WithTimeout(
			context.WithoutCancel(ctx),
			20*time.Second,
		)
		_ = gateway.Stop(stopContext, StopRequest{
			RequestID: gateway.identifier("monitor-stop"),
			SessionID: sessionID,
			Reason:    "active_session_observation_failed",
		})
		stopCancel()
		return
	}
}

func (gateway *Gateway) verifyActiveUntilDeadline(
	ctx context.Context,
	session ExecutionSession,
) error {
	var latest error
	for {
		latest = gateway.config.Observer.VerifyActive(
			ctx,
			session,
			gateway.clock.Now(),
		)
		if latest == nil {
			return nil
		}
		timer := time.NewTimer(50 * time.Millisecond)
		select {
		case <-ctx.Done():
			if !timer.Stop() {
				<-timer.C
			}
			return errors.Join(latest, ctx.Err())
		case <-timer.C:
		}
	}
}

func (gateway *Gateway) submitEnergizing(
	ctx context.Context,
	action TransportAction,
	sessionID string,
	token ReviewToken,
	campaign uint64,
	lease targets.LeaseStatus,
	acceptedPlan *planning.RobotV4AcceptedPlanReference,
) (TransportReceipt, error) {
	now := gateway.clock.Now()
	command := gateway.command(
		action,
		sessionID,
		token,
		campaign,
		&lease.Credential,
		acceptedPlan,
		"",
	)
	if err := command.Validate(now); err != nil {
		return TransportReceipt{}, err
	}
	alreadyCommitted, err := gateway.config.Idempotency.Reserve(ctx, command.Command.IdempotencyKey)
	if err != nil {
		return TransportReceipt{}, fmt.Errorf("execution: reserve idempotency key: %w", err)
	}
	if alreadyCommitted {
		return TransportReceipt{}, ErrIdempotencyReplay
	}
	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "command",
		Action:        string(action),
		Outcome:       "attempt",
		TransportUsed: false,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Command:       commandIdentityPointer(command.Command),
		Lease:         leasePointer(lease),
		Reasons:       []string{"review_token_and_fence_validated"},
	}); err != nil {
		_ = gateway.config.Idempotency.Abort(ctx, command.Command.IdempotencyKey)
		return TransportReceipt{}, fmt.Errorf("execution: audit command attempt: %w", err)
	}

	var receipt TransportReceipt
	if action == ActionPreload {
		receipt, err = gateway.config.Transport.Preload(ctx, command)
	} else if action == ActionArm {
		receipt, err = gateway.config.Transport.Arm(ctx, command)
	} else {
		receipt, err = gateway.config.Transport.Play(ctx, command)
	}
	if err != nil {
		_ = gateway.config.Idempotency.Abort(ctx, command.Command.IdempotencyKey)
		return TransportReceipt{}, fmt.Errorf("execution: %s transport: %w", action, err)
	}
	if err := validateReceipt(gateway.clock.Now(), command, receipt); err != nil {
		_ = gateway.config.Idempotency.Abort(ctx, command.Command.IdempotencyKey)
		return TransportReceipt{}, err
	}
	if err := gateway.config.Idempotency.Commit(ctx, command.Command.IdempotencyKey); err != nil {
		return TransportReceipt{}, fmt.Errorf("execution: commit idempotency key: %w", err)
	}
	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "command",
		Action:        string(action),
		Outcome:       receipt.Result,
		TransportUsed: true,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Command:       commandIdentityPointer(command.Command),
		Lease:         leasePointer(lease),
		Reasons:       []string{"server_transport_applied_exact_identity"},
	}); err != nil {
		return receipt, fmt.Errorf("execution: audit command receipt: %w", err)
	}
	return receipt, nil
}

func (gateway *Gateway) renewLease(
	ctx context.Context,
	sessionID string,
	token ReviewToken,
	campaign uint64,
	previous targets.LeaseStatus,
) (targets.LeaseStatus, error) {
	now := gateway.clock.Now()
	renewed, err := gateway.config.Authority.Renew(ctx, targets.LeaseRenewRequest{
		TargetProfileID:    token.Identity.Target.ProfileID,
		ControllerID:       token.Controller.ControllerID,
		RequestID:          gateway.identifier("renew"),
		CampaignGeneration: campaign,
		TTL:                gateway.config.Bootstrap.LeaseTTL,
		Credential:         previous.Credential,
	})
	if err != nil {
		return targets.LeaseStatus{}, fmt.Errorf("execution: renew lease: %w", err)
	}
	if err := gateway.validateAndFence(ctx, now, token.Identity, campaign, renewed); err != nil {
		return targets.LeaseStatus{}, err
	}
	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "authority",
		Action:        "lease_renew",
		Outcome:       "validated",
		TransportUsed: false,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Lease:         leasePointer(renewed),
		Reasons:       []string{"fresh_exact_fence_revalidated"},
	}); err != nil {
		return targets.LeaseStatus{}, fmt.Errorf("execution: audit lease renewal: %w", err)
	}
	return renewed, nil
}

func (gateway *Gateway) validateAndFence(
	ctx context.Context,
	now time.Time,
	identity ExecutionIdentity,
	campaign uint64,
	status targets.LeaseStatus,
) error {
	controller := gateway.config.Bootstrap.Controller
	if status.TargetProfileID != identity.Target.ProfileID ||
		status.ControllerID != controller.ControllerID ||
		status.CampaignGeneration != campaign ||
		status.StatusSequence == 0 ||
		status.TransitionPending ||
		status.Credential.ControllerBootID != controller.ControllerBootID ||
		status.Credential.LeaseID == "" ||
		status.Credential.FenceEpoch == 0 ||
		status.FreshUntil.IsZero() ||
		now.IsZero() ||
		!now.Before(status.FreshUntil) ||
		status.FreshUntil.Sub(now) > gateway.config.Bootstrap.LeaseTTL {
		return errors.New("execution: lease status does not authorize the exact controller campaign")
	}
	if err := gateway.config.Authority.ValidateFence(ctx, targets.FenceValidationRequest{
		TargetProfileID:    identity.Target.ProfileID,
		Controller:         controller,
		CampaignGeneration: campaign,
		Credential:         status.Credential,
		ObservedStatus:     status,
		CheckedAt:          now.UTC(),
	}); err != nil {
		return fmt.Errorf("execution: validate application-time fence: %w", err)
	}
	return nil
}

func (gateway *Gateway) protectAndClose(
	ctx context.Context,
	sessionID string,
	token ReviewToken,
	campaign uint64,
	lease targets.LeaseStatus,
	reason string,
) error {
	// Protective actions must survive an expired HTTP request or a failed
	// supervisor cycle. Each production transport operation has its own
	// bounded deadline, so detaching cancellation here cannot wait forever.
	ctx = context.WithoutCancel(ctx)
	var failures []error
	for _, action := range []TransportAction{ActionStop, ActionDisarm, ActionWeldOff} {
		if err := gateway.protect(ctx, action, sessionID, token, campaign, lease, reason); err != nil {
			failures = append(failures, err)
		}
	}

	closeAction := "lease_cancel"
	if lease.Credential.LeaseID != "" && lease.Credential.FenceEpoch != 0 {
		closeAction = "lease_release"
	}
	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "authority",
		Action:        closeAction,
		Outcome:       "attempt",
		TransportUsed: false,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Lease:         leasePointer(lease),
		Reasons:       []string{reason},
	}); err != nil {
		failures = append(failures, fmt.Errorf("execution: audit authority close attempt: %w", err))
	}

	var closeErr error
	if lease.Credential.LeaseID != "" && lease.Credential.FenceEpoch != 0 {
		closeErr = gateway.config.Authority.Release(ctx, targets.LeaseReleaseRequest{
			TargetProfileID:    token.Identity.Target.ProfileID,
			ControllerID:       token.Controller.ControllerID,
			RequestID:          gateway.identifier("release"),
			CampaignGeneration: campaign,
			Credential:         lease.Credential,
		})
		if closeErr != nil {
			failures = append(failures, fmt.Errorf("execution: release lease: %w", closeErr))
		}
	} else {
		closeErr = gateway.config.Authority.Cancel(ctx, LeaseCancelRequest{
			TargetProfileID:    token.Identity.Target.ProfileID,
			Controller:         token.Controller,
			RequestID:          gateway.identifier("cancel"),
			CampaignGeneration: campaign,
		})
		if closeErr != nil {
			failures = append(failures, fmt.Errorf("execution: cancel lease campaign: %w", closeErr))
		}
	}
	closeOutcome := "closed"
	if closeErr != nil {
		closeOutcome = "failed"
	}
	if err := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "authority",
		Action:        closeAction,
		Outcome:       closeOutcome,
		TransportUsed: true,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Lease:         leasePointer(lease),
		Reasons:       []string{reason},
	}); err != nil {
		failures = append(failures, fmt.Errorf("execution: audit authority close result: %w", err))
	}
	return errors.Join(failures...)
}

func (gateway *Gateway) protect(
	ctx context.Context,
	action TransportAction,
	sessionID string,
	token ReviewToken,
	campaign uint64,
	lease targets.LeaseStatus,
	reason string,
) error {
	command := gateway.command(action, sessionID, token, campaign, nil, nil, reason)
	var failures []error
	if err := command.Validate(gateway.clock.Now()); err != nil {
		return err
	}

	// Audit and idempotency failures never suppress a protective action.
	alreadyCommitted, reserveErr := gateway.config.Idempotency.Reserve(
		ctx,
		command.Command.IdempotencyKey,
	)
	reserved := reserveErr == nil && !alreadyCommitted
	if reserveErr != nil {
		failures = append(failures, fmt.Errorf("execution: reserve protective idempotency: %w", reserveErr))
	}
	if auditErr := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "protective_tail",
		Action:        string(action),
		Outcome:       "attempt",
		TransportUsed: false,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Command:       commandIdentityPointer(command.Command),
		Lease:         leasePointer(lease),
		Reasons:       []string{reason},
	}); auditErr != nil {
		failures = append(failures, fmt.Errorf("execution: audit protective attempt: %w", auditErr))
	}
	if alreadyCommitted {
		failures = append(failures, ErrIdempotencyReplay)
	}
	receipt, err := gateway.config.Transport.Protect(ctx, command)
	if err != nil {
		if reserved {
			_ = gateway.config.Idempotency.Abort(ctx, command.Command.IdempotencyKey)
		}
		failures = append(failures, fmt.Errorf("execution: protective %s: %w", action, err))
	} else if receiptErr := validateReceipt(gateway.clock.Now(), command, receipt); receiptErr != nil {
		if reserved {
			_ = gateway.config.Idempotency.Abort(ctx, command.Command.IdempotencyKey)
		}
		failures = append(failures, receiptErr)
	} else if reserved {
		if commitErr := gateway.config.Idempotency.Commit(ctx, command.Command.IdempotencyKey); commitErr != nil {
			failures = append(failures, fmt.Errorf("execution: commit protective idempotency: %w", commitErr))
		}
	}
	if auditErr := gateway.appendAudit(ctx, GatewayAuditEnvelope{
		Phase:         "protective_tail",
		Action:        string(action),
		Outcome:       receipt.Result,
		TransportUsed: err == nil,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Command:       commandIdentityPointer(command.Command),
		Lease:         leasePointer(lease),
		Reasons:       []string{reason},
	}); auditErr != nil {
		failures = append(failures, fmt.Errorf("execution: audit protective result: %w", auditErr))
	}
	return errors.Join(failures...)
}

func (gateway *Gateway) command(
	action TransportAction,
	sessionID string,
	token ReviewToken,
	campaign uint64,
	credential *targets.LeaseCredential,
	acceptedPlan *planning.RobotV4AcceptedPlanReference,
	reason string,
) TransportCommand {
	now := gateway.clock.Now().UTC()
	commandID, intent := gateway.commandID(action)
	return TransportCommand{
		Schema:        TransportCommandSchema,
		Action:        action,
		SessionID:     sessionID,
		ReviewTokenID: token.TokenID,
		Controller:    token.Controller,
		Identity:      token.Identity,
		Command: CommandIdentity{
			CommandID: commandID,
			IdempotencyKey: targets.CommandIdempotencyKey(
				token.Identity.Target.ProfileID,
				MotionServerService,
				commandID,
			),
			IntentSequence:   intent,
			ControllerBootID: token.Controller.ControllerBootID,
			CreatedAt:        now,
		},
		CampaignGeneration: campaign,
		Credential:         credential,
		AcceptedPlan:       acceptedPlan,
		Reason:             reason,
	}
}

func validateReceipt(now time.Time, command TransportCommand, receipt TransportReceipt) error {
	if receipt.Schema != TransportReceiptSchema ||
		receipt.Action != command.Action ||
		receipt.CommandID != command.Command.CommandID ||
		receipt.IdempotencyKey != command.Command.IdempotencyKey ||
		!receipt.Accepted ||
		!receipt.Applied ||
		!receipt.ServerOwned ||
		receipt.RecordedAt.IsZero() ||
		receipt.RecordedAt.Before(command.Command.CreatedAt) ||
		now.Before(receipt.RecordedAt) ||
		!receipt.Identity.Equal(command.Identity) {
		return ErrTransportReceipt
	}
	return nil
}

func (gateway *Gateway) consumeToken(
	now time.Time,
	tokenID string,
	input ExecutionReviewInput,
) (ReviewToken, error) {
	gateway.stateMu.Lock()
	defer gateway.stateMu.Unlock()
	stored, ok := gateway.tokens[tokenID]
	if !ok ||
		stored.consumed ||
		stored.token.Schema != ReviewTokenSchema ||
		stored.token.TokenID != tokenID ||
		stored.token.Controller != gateway.config.Bootstrap.Controller ||
		!stored.token.Identity.Equal(input.Identity) ||
		input.Status.Sequence < stored.token.StatusSequence ||
		input.RuntimePlan.Sequence < stored.token.RuntimePlanSequence ||
		stored.token.EvidenceRequestID != input.Evidence.RequestID ||
		stored.token.IssuedAt.IsZero() ||
		now.Before(stored.token.IssuedAt) ||
		!now.Before(stored.token.ExpiresAt) {
		return ReviewToken{}, ErrReviewToken
	}
	stored.consumed = true
	gateway.tokens[tokenID] = stored
	return stored.token, nil
}

func (gateway *Gateway) appendAudit(
	ctx context.Context,
	envelope GatewayAuditEnvelope,
) error {
	now := gateway.clock.Now().UTC()
	envelope.Schema = AuditSchema
	envelope.EventID = gateway.identifier("audit")
	envelope.RecordedAt = now
	envelope.Reasons = append([]string(nil), envelope.Reasons...)
	if gateway.config.Audit == nil {
		return errors.New("execution: audit sink is unavailable")
	}
	return gateway.config.Audit.Append(ctx, envelope)
}

func (gateway *Gateway) verifyEvidence(
	ctx context.Context,
	input ExecutionReviewInput,
) error {
	if gateway.config.Evidence == nil {
		return errors.New("execution: authoritative evidence verifier is unavailable")
	}
	return gateway.config.Evidence.Verify(ctx, input)
}

func denyReview(review *ExecutionReview, code string, detail string) {
	if review == nil {
		return
	}
	review.Ready = false
	review.Reasons = appendUnique(review.Reasons, code)
	review.Checks = append(review.Checks, ReviewCheck{
		Code:   code,
		Passed: false,
		Detail: detail,
	})
	review.Token = nil
}

func (gateway *Gateway) identifier(label string) string {
	gateway.stateMu.Lock()
	defer gateway.stateMu.Unlock()
	value := gateway.nextID
	gateway.nextID++
	return gateway.config.Bootstrap.CommandIDPrefix + "-" + label + "-" + strconv.FormatUint(value, 10)
}

func (gateway *Gateway) commandID(action TransportAction) (string, uint64) {
	gateway.stateMu.Lock()
	defer gateway.stateMu.Unlock()
	idValue := gateway.nextID
	gateway.nextID++
	intent := gateway.nextIntent
	gateway.nextIntent++
	return gateway.config.Bootstrap.CommandIDPrefix + "-" + string(action) + "-" +
		strconv.FormatUint(idValue, 10), intent
}

func (gateway *Gateway) campaign() uint64 {
	gateway.stateMu.Lock()
	defer gateway.stateMu.Unlock()
	value := gateway.nextCampaign
	gateway.nextCampaign++
	return value
}

func (gateway *Gateway) session(sessionID string) (ExecutionSession, bool) {
	gateway.stateMu.Lock()
	defer gateway.stateMu.Unlock()
	session, ok := gateway.sessions[sessionID]
	return session, ok
}

func (gateway *Gateway) deleteSession(sessionID string) {
	gateway.stateMu.Lock()
	cancel := gateway.sessionStops[sessionID]
	delete(gateway.sessionStops, sessionID)
	delete(gateway.sessions, sessionID)
	gateway.stateMu.Unlock()
	if cancel != nil {
		cancel()
	}
}

func leasePointer(status targets.LeaseStatus) *targets.LeaseStatus {
	copy := status
	return &copy
}

func commandIdentityPointer(identity CommandIdentity) *CommandIdentity {
	copy := identity
	return &copy
}
