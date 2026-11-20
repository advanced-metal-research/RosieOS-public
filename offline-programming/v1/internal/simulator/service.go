package simulator

import (
	"context"
	"fmt"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

type Backend interface {
	// Implementations must enforce StartRequest resource limits before
	// allocating plan request or report buffers.
	Start(context.Context, StartRequest) (RuntimeSnapshot, error)
	Probe(context.Context) (RuntimeSnapshot, error)
	SubmitPlan(context.Context, planning.MotionServerSubmission) ([]byte, error)
	AcceptedPlanAssets() (planning.AcceptedPlanAssets, error)
	Stop(context.Context) error
}

type AvailabilityProvider interface {
	Availability() BackendAvailability
}

type Options struct {
	ServiceID             string
	Topology              Topology
	Limits                ResourceLimits
	ProofTTL              time.Duration
	PlannerRequestSubject string
	Clock                 func() time.Time
}

// Service owns lifecycle, readiness, proof freshness, bounded one-at-a-time
// planning, and report correlation. It intentionally has no arm, jog, play,
// weld, EtherCAT, RTCore, or generic command method.
type Service struct {
	backend Backend
	options Options

	operationMu sync.Mutex
	mu          sync.RWMutex
	status      Status
	nextPlan    uint64
}

func NewService(options Options, backend Backend) (*Service, error) {
	options.ServiceID = strings.TrimSpace(options.ServiceID)
	if options.ServiceID == "" {
		return nil, serviceError(
			ErrorInvalidConfiguration,
			"configure",
			"service identity is required",
			ErrInvalidConfiguration,
		)
	}
	if err := options.Topology.Validate(); err != nil {
		return nil, serviceError(ErrorInvalidConfiguration, "configure", err.Error(), ErrInvalidConfiguration)
	}
	if err := options.Limits.Validate(); err != nil {
		return nil, serviceError(ErrorInvalidConfiguration, "configure", err.Error(), ErrInvalidConfiguration)
	}
	if options.ProofTTL <= 0 || options.ProofTTL > 30*time.Second {
		return nil, serviceError(
			ErrorInvalidConfiguration,
			"configure",
			"proof TTL must be between zero and 30 seconds",
			ErrInvalidConfiguration,
		)
	}
	if strings.ContainsAny(options.PlannerRequestSubject, " \t\r\n") {
		return nil, serviceError(
			ErrorInvalidConfiguration,
			"configure",
			"planner request subject contains whitespace",
			ErrInvalidConfiguration,
		)
	}
	if options.Clock == nil {
		options.Clock = time.Now
	}
	now := options.Clock().UTC()
	service := &Service{
		backend:  backend,
		options:  options,
		nextPlan: 1,
		status: Status{
			Schema:         ServiceStatusSchema,
			ServiceID:      options.ServiceID,
			StatusSequence: 1,
			Phase:          PhaseStopped,
			ObservedAt:     now,
			Availability:   backendAvailability(backend),
			Topology:       options.Topology,
			Readiness: Readiness{
				Ready:   false,
				Reasons: []string{"service_stopped"},
			},
		},
	}
	return service, nil
}

func (service *Service) Status() Status {
	if service == nil {
		return Status{}
	}
	service.mu.RLock()
	defer service.mu.RUnlock()
	return cloneStatus(service.status)
}

func (service *Service) Start(ctx context.Context) (Status, error) {
	if service == nil {
		return Status{}, serviceError(ErrorInvalidLifecycle, "start", "nil service", ErrInvalidLifecycle)
	}
	service.operationMu.Lock()
	defer service.operationMu.Unlock()

	service.mu.Lock()
	if service.status.Phase != PhaseStopped && service.status.Phase != PhaseFailed {
		status := cloneStatus(service.status)
		service.mu.Unlock()
		return status, serviceError(
			ErrorInvalidLifecycle,
			"start",
			fmt.Sprintf("cannot start from %s", status.Phase),
			ErrInvalidLifecycle,
		)
	}
	service.updateLocked(PhaseStarting, Readiness{Reasons: []string{"runtime_starting"}}, SafetyProof{}, "", nil)
	service.status.Availability = backendAvailability(service.backend)
	service.mu.Unlock()

	availability := backendAvailability(service.backend)
	if !availability.Available {
		return service.fail("start", ErrorBackendUnavailable, availability.Reason, ErrBackendUnavailable)
	}
	startRequest := StartRequest{
		Topology: service.options.Topology,
		Limits:   service.options.Limits,
		Policy:   requiredSafetyPolicy(),
	}
	snapshot, err := service.backend.Start(ctx, startRequest)
	if err != nil {
		service.mu.Lock()
		service.status.Availability = backendAvailability(service.backend)
		service.mu.Unlock()
		return service.fail("start", ErrorBackendUnavailable, err.Error(), ErrBackendUnavailable)
	}
	now := service.options.Clock().UTC()
	if err := snapshot.Validate(now, service.options.Topology, service.options.ProofTTL); err != nil {
		_ = service.backend.Stop(context.WithoutCancel(ctx))
		return service.fail("start", ErrorUnsafeRuntime, err.Error(), ErrUnsafeRuntime)
	}
	service.mu.Lock()
	service.status.Availability = backendAvailability(service.backend)
	service.updateLocked(PhaseReady, Readiness{Ready: true, Reasons: []string{}}, snapshot.Safety, "", nil)
	status := cloneStatus(service.status)
	service.mu.Unlock()
	return status, nil
}

func (service *Service) Refresh(ctx context.Context) (Status, error) {
	if service == nil {
		return Status{}, serviceError(ErrorInvalidLifecycle, "refresh", "nil service", ErrInvalidLifecycle)
	}
	service.operationMu.Lock()
	defer service.operationMu.Unlock()
	return service.refreshLocked(ctx, "refresh")
}

func (service *Service) Stop(ctx context.Context) (Status, error) {
	if service == nil {
		return Status{}, serviceError(ErrorInvalidLifecycle, "stop", "nil service", ErrInvalidLifecycle)
	}
	service.operationMu.Lock()
	defer service.operationMu.Unlock()

	service.mu.Lock()
	if service.status.Phase == PhaseStopped {
		status := cloneStatus(service.status)
		service.mu.Unlock()
		return status, nil
	}
	service.updateLocked(PhaseStopping, Readiness{Reasons: []string{"runtime_stopping"}}, SafetyProof{}, "", nil)
	service.mu.Unlock()
	if service.backend != nil {
		if err := service.backend.Stop(ctx); err != nil {
			return service.fail("stop", ErrorBackendUnavailable, err.Error(), ErrBackendUnavailable)
		}
	}
	service.mu.Lock()
	service.status.Availability = backendAvailability(service.backend)
	service.updateLocked(PhaseStopped, Readiness{Reasons: []string{"service_stopped"}}, SafetyProof{}, "", nil)
	status := cloneStatus(service.status)
	service.mu.Unlock()
	return status, nil
}

func (service *Service) SubmitPlan(
	ctx context.Context,
	request planning.NormalizedPlanRequest,
) (planning.NormalizedPlanResult, error) {
	if service == nil {
		return planning.NormalizedPlanResult{}, serviceError(
			ErrorInvalidLifecycle,
			"plan",
			"nil service",
			ErrInvalidLifecycle,
		)
	}
	if err := request.Validate(); err != nil {
		return planning.NormalizedPlanResult{}, serviceError(
			ErrorInvalidPlan,
			"plan",
			err.Error(),
			ErrInvalidPlan,
		)
	}
	if request.Target.ProfileID != service.options.Topology.ProfileID ||
		request.RobotModel != service.options.Topology.RobotModel {
		return planning.NormalizedPlanResult{}, serviceError(
			ErrorInvalidPlan,
			"plan",
			"request target or robot model does not match the local runtime",
			ErrInvalidPlan,
		)
	}

	service.mu.Lock()
	if service.status.PlanInFlight {
		service.mu.Unlock()
		return planning.NormalizedPlanResult{}, serviceError(ErrorBusy, "plan", "another plan is in flight", ErrBusy)
	}
	if service.status.Phase != PhaseReady || !service.status.Readiness.Ready {
		service.mu.Unlock()
		return planning.NormalizedPlanResult{}, serviceError(ErrorNotReady, "plan", "local runtime is not ready", ErrNotReady)
	}
	service.status.PlanInFlight = true
	service.status.StatusSequence++
	service.status.ObservedAt = service.options.Clock().UTC()
	planRevision := service.nextPlan
	service.nextPlan++
	service.mu.Unlock()
	defer func() {
		service.mu.Lock()
		service.status.PlanInFlight = false
		service.status.StatusSequence++
		service.status.ObservedAt = service.options.Clock().UTC()
		service.mu.Unlock()
	}()

	service.operationMu.Lock()
	defer service.operationMu.Unlock()
	if _, err := service.refreshLocked(ctx, "plan_preflight"); err != nil {
		return planning.NormalizedPlanResult{}, err
	}
	submission, err := planning.NewMotionServerSubmission(
		request,
		planRevision,
		service.options.PlannerRequestSubject,
	)
	if err != nil {
		return planning.NormalizedPlanResult{}, service.recordPlanError(
			ErrorInvalidPlan,
			"plan",
			err.Error(),
			ErrInvalidPlan,
		)
	}
	if len(submission.Payload) > service.options.Limits.MaxPlanRequestBytes {
		return planning.NormalizedPlanResult{}, service.recordPlanError(
			ErrorResourceLimit,
			"plan",
			"motion-server request exceeds the configured byte limit",
			ErrResourceLimit,
		)
	}

	planContext, cancel := context.WithTimeout(ctx, service.options.Limits.PlanTimeout)
	defer cancel()
	raw, err := service.backend.SubmitPlan(planContext, submission)
	if err != nil {
		return planning.NormalizedPlanResult{}, service.recordPlanError(
			ErrorBackendUnavailable,
			"plan",
			err.Error(),
			ErrBackendUnavailable,
		)
	}
	if len(raw) == 0 || len(raw) > service.options.Limits.MaxPlanReportBytes {
		return planning.NormalizedPlanResult{}, service.recordPlanError(
			ErrorResourceLimit,
			"plan",
			"motion-server report is empty or exceeds the configured byte limit",
			ErrResourceLimit,
		)
	}
	receivedAt := service.options.Clock().UTC()
	result, err := planning.NormalizeMotionServerPlanResult(planning.ReportContext{
		Request:      request,
		PlanID:       fmt.Sprintf("%s/%s/%d", service.options.ServiceID, request.RequestID, planRevision),
		PlanRevision: planRevision,
		ReceivedAt:   receivedAt,
	}, raw, service.options.Limits.MaxTrajectorySamples)
	if err != nil {
		return planning.NormalizedPlanResult{}, service.recordPlanError(
			ErrorInvalidPlan,
			"plan",
			err.Error(),
			ErrInvalidPlan,
		)
	}

	if _, err := service.refreshLocked(ctx, "plan_postflight"); err != nil {
		result.Evidence.State = planning.PlanRejected
		result.ReportReasons = append(result.ReportReasons, "post_plan_runtime_proof_invalid")
		return result, err
	}
	if result.RuntimePlayable() {
		assets, assetErr := service.backend.AcceptedPlanAssets()
		if assetErr != nil {
			result.Evidence.State = planning.PlanRejected
			result.ReportReasons = append(
				result.ReportReasons,
				"accepted_plan_asset_identity_unavailable",
			)
			return result, service.recordPlanError(
				ErrorBackendUnavailable,
				"plan",
				assetErr.Error(),
				ErrBackendUnavailable,
			)
		}
		acceptedPlan, mintErr := planning.MintRobotV4AcceptedPlan(
			request,
			result,
			assets,
			planning.AcceptedPlanSimulationOnly,
			receivedAt,
		)
		if mintErr != nil {
			result.Evidence.State = planning.PlanRejected
			result.ReportReasons = append(
				result.ReportReasons,
				"accepted_plan_mint_failed",
			)
			return result, service.recordPlanError(
				ErrorInvalidPlan,
				"plan",
				mintErr.Error(),
				ErrInvalidPlan,
			)
		}
		result.AcceptedPlan = &acceptedPlan
	}
	if !result.RuntimePlayable() {
		code := ErrorPlanRejected
		sentinel := ErrPlanRejected
		if containsReason(result.ReportReasons, "plan_request_revision_missing") ||
			containsReason(result.ReportReasons, "plan_request_revision_mismatch") {
			code = ErrorUncorrelatedReport
			sentinel = ErrUncorrelatedReport
		}
		return result, service.recordPlanError(
			code,
			"plan",
			strings.Join(planRejectionReasons(result), ","),
			sentinel,
		)
	}

	service.mu.Lock()
	service.status.LastPlan = result.Evidence.Identity
	service.status.LastError = ""
	service.status.LastErrorCode = ""
	service.status.StatusSequence++
	service.status.ObservedAt = service.options.Clock().UTC()
	service.mu.Unlock()
	return result, nil
}

func (service *Service) refreshLocked(ctx context.Context, operation string) (Status, error) {
	service.mu.RLock()
	phase := service.status.Phase
	service.mu.RUnlock()
	if phase != PhaseReady {
		return service.Status(), serviceError(
			ErrorNotReady,
			operation,
			fmt.Sprintf("cannot probe from %s", phase),
			ErrNotReady,
		)
	}
	if service.backend == nil {
		return service.fail(operation, ErrorBackendUnavailable, "loopback backend is not installed", ErrBackendUnavailable)
	}
	snapshot, err := service.backend.Probe(ctx)
	if err != nil {
		_ = service.backend.Stop(context.WithoutCancel(ctx))
		return service.fail(operation, ErrorBackendUnavailable, err.Error(), ErrBackendUnavailable)
	}
	now := service.options.Clock().UTC()
	if err := snapshot.Validate(now, service.options.Topology, service.options.ProofTTL); err != nil {
		_ = service.backend.Stop(context.WithoutCancel(ctx))
		return service.fail(operation, ErrorUnsafeRuntime, err.Error(), ErrUnsafeRuntime)
	}
	service.mu.Lock()
	service.status.Availability = backendAvailability(service.backend)
	service.updateLocked(PhaseReady, Readiness{Ready: true, Reasons: []string{}}, snapshot.Safety, "", nil)
	status := cloneStatus(service.status)
	service.mu.Unlock()
	return status, nil
}

func (service *Service) fail(
	operation string,
	code ErrorCode,
	message string,
	cause error,
) (Status, error) {
	service.mu.Lock()
	service.updateLocked(
		PhaseFailed,
		Readiness{Ready: false, Reasons: []string{string(code)}},
		SafetyProof{},
		message,
		&code,
	)
	status := cloneStatus(service.status)
	service.mu.Unlock()
	return status, serviceError(code, operation, message, cause)
}

func (service *Service) recordPlanError(
	code ErrorCode,
	operation, message string,
	cause error,
) error {
	service.mu.Lock()
	service.status.LastErrorCode = code
	service.status.LastError = message
	service.status.StatusSequence++
	service.status.ObservedAt = service.options.Clock().UTC()
	service.mu.Unlock()
	return serviceError(code, operation, message, cause)
}

func (service *Service) updateLocked(
	phase LifecyclePhase,
	readiness Readiness,
	proof SafetyProof,
	lastError string,
	lastErrorCode *ErrorCode,
) {
	service.status.Phase = phase
	service.status.Readiness = Readiness{
		Ready:   readiness.Ready,
		Reasons: append([]string{}, readiness.Reasons...),
	}
	service.status.Safety = proof
	service.status.LastError = lastError
	service.status.LastErrorCode = ""
	if lastErrorCode != nil {
		service.status.LastErrorCode = *lastErrorCode
	}
	service.status.StatusSequence++
	service.status.ObservedAt = service.options.Clock().UTC()
}

func requiredSafetyPolicy() SafetyPolicy {
	return SafetyPolicy{
		Simulation:               true,
		NoRealMovement:           true,
		LoopbackOnly:             true,
		EtherCATOutputEnabled:    false,
		MotionOutputEnabled:      false,
		HardwareAccess:           false,
		PhysicalServicesStarted:  false,
		CommandSubmissionEnabled: false,
	}
}

func cloneStatus(status Status) Status {
	status.Readiness.Reasons = append([]string{}, status.Readiness.Reasons...)
	return status
}

func backendAvailability(backend Backend) BackendAvailability {
	if backend == nil {
		return BackendAvailability{
			Schema:            AvailabilitySchema,
			Available:         false,
			Code:              AvailabilityCanonicalBinary,
			Reason:            "canonical local planner backend is not installed",
			CanonicalPlanner:  false,
			ProcessBounded:    false,
			PersistentProcess: false,
			ExplicitOptIn:     true,
		}
	}
	if provider, ok := backend.(AvailabilityProvider); ok {
		return provider.Availability()
	}
	return BackendAvailability{
		Schema:            AvailabilitySchema,
		Available:         true,
		Code:              AvailabilityReady,
		Reason:            "",
		CanonicalPlanner:  true,
		ProcessBounded:    true,
		PersistentProcess: false,
		ExplicitOptIn:     true,
	}
}

func containsReason(reasons []string, wanted string) bool {
	for _, reason := range reasons {
		if reason == wanted {
			return true
		}
	}
	return false
}

func planRejectionReasons(result planning.NormalizedPlanResult) []string {
	reasons := append([]string(nil), result.ReportReasons...)
	reasons = appendUniqueReasons(reasons, result.Evidence.CorrelationReasons...)
	if !result.Evidence.Accepted {
		reasons = appendUniqueReasons(reasons, "planner_rejected")
	}
	if result.Evidence.Collision.State != planning.CheckClear {
		reasons = appendUniqueReasons(reasons, "collision_evidence_not_clear")
	}
	if result.Evidence.Singularity.State != planning.CheckClear {
		reasons = appendUniqueReasons(reasons, "singularity_evidence_not_clear")
	}
	if result.Evidence.Limits.State != planning.CheckClear {
		reasons = appendUniqueReasons(reasons, "limit_evidence_not_clear")
	}
	if result.Evidence.Runtime.State != planning.CheckClear || !result.Evidence.Runtime.Playable {
		reasons = appendUniqueReasons(reasons, "runtime_not_playable")
	}
	if len(reasons) == 0 {
		reasons = append(reasons, "canonical_plan_not_runtime_playable")
	}
	return reasons
}

func appendUniqueReasons(reasons []string, additions ...string) []string {
	for _, addition := range additions {
		if addition != "" && !containsReason(reasons, addition) {
			reasons = append(reasons, addition)
		}
	}
	return reasons
}
