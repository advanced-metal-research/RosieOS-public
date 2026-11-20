package execution

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"rosieos/offline-programming/v1/internal/targets"
	"github.com/nats-io/nats.go"
)

const (
	commandReplySchema           = "robot.v4.command-reply.v1"
	motionServerReplyComponent   = "motion-server"
	realtimeCoreReplyComponent   = "realtime-core-nats-bridge"
	defaultControllerTimeout     = 4 * time.Second
	maxControllerCommandBytes    = 2 * 1024 * 1024
	offlineProgrammingController = "offline-programming"
)

// ControllerTargetSource exposes only the server-side manifest binding.
// Implementations must never serialize the returned binding to the browser.
type ControllerTargetSource interface {
	PlanningTarget(string) (targets.ConnectedManifestTarget, bool)
}

type ControllerServiceResolver interface {
	ResolveNATS(context.Context, string) (string, error)
}

type controllerCommandReply struct {
	Schema    string `json:"schema"`
	Component string `json:"component"`
	Command   string `json:"command"`
	CommandID string `json:"command_id"`
	Accepted  bool   `json:"accepted"`
	Duplicate bool   `json:"duplicate"`
	State     string `json:"state"`
	Error     string `json:"error"`
}

type controllerStatus struct {
	Name                           string `json:"name"`
	StatusSchema                   string `json:"status_schema"`
	State                          string `json:"state"`
	UpdatedAt                      string `json:"updated_at"`
	PlanPreloaded                  bool   `json:"plan_preloaded"`
	LoadedProgramAvailable         bool   `json:"loaded_program_available"`
	LoadedProgramPlayable          bool   `json:"loaded_program_playable"`
	LoadedProgramRevision          uint64 `json:"loaded_program_revision"`
	PlanRevision                   uint64 `json:"plan_revision"`
	PlanSessionID                  string `json:"plan_session_id"`
	PlanPlaybackActive             bool   `json:"plan_playback_active"`
	AcceptedPlanID                 string `json:"accepted_plan_id"`
	AcceptedProgramDigest          string `json:"accepted_program_digest"`
	AcceptedTrajectoryDigest       string `json:"accepted_trajectory_digest"`
	AcceptedManifestRevision       string `json:"accepted_manifest_revision"`
	LeaderID                       string `json:"leader_id"`
	LeaderBootID                   string `json:"leader_boot_id"`
	LeaderLeaseID                  string `json:"leader_lease_id"`
	LeaderFenceEpoch               uint64 `json:"leader_fence_epoch"`
	LeaderCampaignGeneration       uint64 `json:"leader_campaign_generation"`
	LeaderLeaseFresh               bool   `json:"leader_lease_fresh"`
	LeaderExpiresInMS              int64  `json:"leader_expires_in_ms"`
	LeaderTransitionPending        bool   `json:"leader_transition_pending"`
	LatestLeaderResult             string `json:"latest_leader_result"`
	LatestLeaderRequestID          string `json:"latest_leader_request_id"`
	LatestLeaderCampaignGeneration uint64 `json:"latest_leader_campaign_generation"`
	LatestAppliedIntentSequence    uint64 `json:"latest_applied_cold_motion_intent_seq"`
	LatestAppliedPlanRevision      uint64 `json:"latest_applied_cold_plan_revision"`
	LatestAppliedResultCode        uint8  `json:"latest_applied_cold_result_code"`
	LatestAppliedCommandID         string `json:"latest_applied_cold_command_id"`
	LatestAppliedSenderID          string `json:"latest_applied_cold_sender_id"`
	LatestAppliedCommandKind       string `json:"latest_applied_cold_command_kind"`
	LatestAppliedResult            string `json:"latest_applied_cold_result"`
	LatestAppliedAccepted          bool   `json:"latest_applied_cold_accepted"`
	LatestAppliedZeroOutputSent    bool   `json:"latest_applied_cold_zero_output_sent"`
	CurrentOutputZeroHold          bool   `json:"current_output_zero_hold"`
	ServosArmedRequested           bool   `json:"servos_armed_requested"`
	Schema                         string `json:"schema"`
	Subject                        string `json:"subject"`
	Host                           string `json:"host"`
	Component                      string `json:"component"`
	PublishedAt                    string `json:"published_at"`
	Status                         struct {
		Name                  string `json:"name"`
		State                 string `json:"state"`
		UpdatedAt             string `json:"updated_at"`
		RTCoreMode            string `json:"rtcore_mode"`
		RuntimeModeEpoch      uint64 `json:"runtime_mode_epoch"`
		EtherCATOutputEnabled *bool  `json:"ethercat_output_enabled"`
		MotionOutputEnabled   *bool  `json:"motion_output_enabled"`
		ServosArmedRequested  *bool  `json:"servos_armed_requested"`
		ConfiguredAxisCount   int    `json:"configured_axis_count"`
		DS402EnabledAxes      int    `json:"ds402_enabled_axes"`
		AxisEnableMask        uint32 `json:"axis_enable_mask"`
		AxisErrorMask         uint32 `json:"axis_error_mask"`
		AxisFaultMask         uint32 `json:"axis_fault_mask"`
		RevPiOutputReadback   struct {
			OutputCount int    `json:"output_count"`
			ValuesMask  uint16 `json:"values_mask"`
			ValidMask   uint16 `json:"valid_mask"`
			Fresh       bool   `json:"fresh"`
		} `json:"revpi_output_readback"`
	} `json:"status"`
}

type controllerExchange interface {
	Command(
		context.Context,
		string,
		string,
		string,
		[]byte,
		string,
		func(controllerStatus) bool,
	) (controllerCommandReply, controllerStatus, time.Time, error)
	Observe(
		context.Context,
		string,
		string,
		func(controllerStatus) bool,
	) (controllerStatus, time.Time, error)
}

type NATSControllerOptions struct {
	Targets  ControllerTargetSource
	Resolver ControllerServiceResolver
	Exchange controllerExchange
	Timeout  time.Duration
}

// NATSController is the production, server-owned Robot V4 lease and command
// adapter. Core NATS request replies prove only that the owning component
// accepted a command. Motion commands additionally wait for the exact
// application record in Motion Server status before returning a receipt.
type NATSController struct {
	targets  ControllerTargetSource
	resolver ControllerServiceResolver
	exchange controllerExchange
	timeout  time.Duration
	sequence atomic.Uint64

	mu          sync.Mutex
	lastApplied map[string]targets.LeaseStatus
}

func NewNATSController(options NATSControllerOptions) (*NATSController, error) {
	if options.Targets == nil || options.Resolver == nil {
		return nil, errors.New("execution: target source and daemon NATS resolver are required")
	}
	timeout := options.Timeout
	if timeout == 0 {
		timeout = defaultControllerTimeout
	}
	if timeout < 500*time.Millisecond || timeout > 15*time.Second {
		return nil, errors.New("execution: NATS controller timeout is invalid")
	}
	exchange := options.Exchange
	if exchange == nil {
		exchange = &natsControllerExchange{}
	}
	return &NATSController{
		targets:     options.Targets,
		resolver:    options.Resolver,
		exchange:    exchange,
		timeout:     timeout,
		lastApplied: make(map[string]targets.LeaseStatus),
	}, nil
}

func (controller *NATSController) Capabilities() ServerTransportCapabilities {
	available := controller != nil &&
		controller.targets != nil &&
		controller.resolver != nil &&
		controller.exchange != nil
	return ServerTransportCapabilities{
		Available:          available,
		ServerOwned:        available,
		TestOnly:           false,
		CommandSchema:      RobotV4CommandSchema,
		LeaderProtocol:     RobotV4LeaderProtocol,
		PlanPreload:        available,
		ControlledStartArm: available,
		PlanPlay:           available,
		ProtectiveStop:     available,
		ProtectiveDisarm:   available,
		ProtectiveWeldOff:  available,
		RawEndpointExposed: false,
	}
}

func (controller *NATSController) Preload(
	ctx context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	if command.Action != ActionPreload || command.AcceptedPlan == nil {
		return TransportReceipt{}, errors.New("execution: exact accepted plan preload is required")
	}
	payload := controller.motionCommandPayload(command, "plan_preload", map[string]any{
		"plan_revision": command.Identity.PlanRevision,
		"program_id":    command.Identity.ProgramID,
		"accepted_plan": command.AcceptedPlan,
	})
	return controller.submitMotion(
		ctx,
		command,
		payload,
		func(status controllerStatus) bool {
			return exactAppliedMotion(status, command, "plan_preload",
				"accepted_plan_preloaded", 6) &&
				status.PlanPreloaded &&
				status.LoadedProgramAvailable &&
				status.LoadedProgramPlayable &&
				status.LoadedProgramRevision == command.Identity.PlanRevision &&
				status.PlanRevision == command.Identity.PlanRevision &&
				status.PlanSessionID == command.Identity.PlanID &&
				status.AcceptedPlanID == command.Identity.PlanID &&
				status.AcceptedProgramDigest == command.Identity.ProgramDigest &&
				status.AcceptedTrajectoryDigest == command.Identity.TrajectoryDigest &&
				status.AcceptedManifestRevision ==
					command.Identity.Target.ManifestRevision
		},
		"accepted_plan_preloaded",
	)
}

func (controller *NATSController) Arm(
	ctx context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	if command.Action != ActionArm || command.Credential == nil {
		return TransportReceipt{}, errors.New(
			"execution: controlled reviewed-start arm is required",
		)
	}
	target, natsURL, err := controller.resolve(
		ctx,
		command.Identity.Target.ProfileID,
	)
	if err != nil {
		return TransportReceipt{}, err
	}
	bounded, cancel := context.WithTimeout(ctx, controller.timeout)
	defer cancel()
	ready, _, err := controller.exchange.Observe(
		bounded,
		natsURL,
		target.Binding.RealtimeCoreStatusSubject,
		func(status controllerStatus) bool {
			return exactRTCoreReadyToArmStatus(
				status,
				target.Binding.RealtimeCoreStatusSubject,
			)
		},
	)
	if err != nil {
		return TransportReceipt{}, fmt.Errorf(
			"execution: observe exact disarmed RTCore mode binding: %w",
			err,
		)
	}
	payload := controller.baseCommand(command, "arm")
	payload["armed"] = true
	payload["runtime_mode"] = "live"
	payload["runtime_mode_epoch"] = ready.Status.RuntimeModeEpoch
	payload["controller_boot_id"] = command.Credential.ControllerBootID
	payload["leader_lease_id"] = command.Credential.LeaseID
	payload["leader_fence_epoch"] = command.Credential.FenceEpoch
	payload["campaign_generation"] = command.CampaignGeneration
	payload["motion_intent_seq"] = command.Command.IntentSequence
	raw, err := json.Marshal(payload)
	if err != nil {
		return TransportReceipt{}, err
	}
	reply, _, receivedAt, err := controller.exchange.Command(
		bounded,
		natsURL,
		target.Binding.CommandSubject,
		target.Binding.RealtimeCoreStatusSubject,
		raw,
		realtimeCoreReplyComponent,
		func(status controllerStatus) bool {
			return exactRTCoreArmedStatus(
				status,
				target.Binding.RealtimeCoreStatusSubject,
			)
		},
	)
	if err != nil {
		return TransportReceipt{}, err
	}
	return transportReceipt(
		command,
		reply,
		receivedAt,
		"armed_for_reviewed_start",
	), nil
}

func (controller *NATSController) Play(
	ctx context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	if command.Action != ActionPlay {
		return TransportReceipt{}, errors.New("execution: play command is required")
	}
	payload := controller.motionCommandPayload(command, "play", map[string]any{
		"plan_id":           command.Identity.PlanID,
		"plan_revision":     command.Identity.PlanRevision,
		"program_id":        command.Identity.ProgramID,
		"program_digest":    command.Identity.ProgramDigest,
		"trajectory_digest": command.Identity.TrajectoryDigest,
		"manifest_revision": command.Identity.Target.ManifestRevision,
	})
	return controller.submitMotion(
		ctx,
		command,
		payload,
		func(status controllerStatus) bool {
			return exactAppliedMotion(status, command, "play",
				"playback_started", 1) &&
				status.PlanPreloaded &&
				status.PlanPlaybackActive &&
				status.AcceptedPlanID == command.Identity.PlanID &&
				status.AcceptedProgramDigest == command.Identity.ProgramDigest &&
				status.AcceptedTrajectoryDigest == command.Identity.TrajectoryDigest &&
				status.AcceptedManifestRevision ==
					command.Identity.Target.ManifestRevision
		},
		"playback_started",
	)
}

func (controller *NATSController) Protect(
	ctx context.Context,
	command TransportCommand,
) (TransportReceipt, error) {
	switch command.Action {
	case ActionStop:
		payload := controller.motionCommandPayload(command, "stop", map[string]any{
			"plan_revision": command.Identity.PlanRevision,
			"program_id":    command.Identity.ProgramID,
		})
		return controller.submitMotion(
			ctx,
			command,
			payload,
			func(status controllerStatus) bool {
				return exactAppliedMotion(status, command, "stop",
					"stop_zero_hold_sent", 3) &&
					status.CurrentOutputZeroHold &&
					!status.PlanPlaybackActive
			},
			"stop_zero_hold_sent",
		)
	case ActionDisarm:
		return controller.submitBridge(
			ctx,
			command,
			"disarm",
			map[string]any{"armed": false},
			"disarm_applied",
		)
	case ActionWeldOff:
		return controller.submitBridge(
			ctx,
			command,
			"io",
			map[string]any{
				"slot":            5,
				"on":              false,
				"intent_sequence": command.Command.IntentSequence,
			},
			"weld_off_applied",
		)
	default:
		return TransportReceipt{}, errors.New("execution: unsupported protective action")
	}
}

func (controller *NATSController) Acquire(
	ctx context.Context,
	request targets.LeaseAcquireRequest,
) (targets.LeaseStatus, error) {
	target, natsURL, err := controller.resolve(ctx, request.TargetProfileID)
	if err != nil {
		return targets.LeaseStatus{}, err
	}
	commandID := controller.wireID("leader-acquire")
	payload, err := json.Marshal(map[string]any{
		"schema":              RobotV4CommandSchema,
		"command":             "leader_acquire",
		"robot":               target.Discovery.CellID,
		"command_id":          commandID,
		"sender_id":           request.ControllerID,
		"controller_boot_id":  request.ControllerBootID,
		"request_id":          request.RequestID,
		"campaign_generation": request.CampaignGeneration,
		"ttl_ms":              request.TTL.Milliseconds(),
	})
	if err != nil {
		return targets.LeaseStatus{}, err
	}
	status, receivedAt, err := controller.leaderCommand(
		ctx, target, natsURL, payload, "leader_acquire", commandID,
		func(status controllerStatus) bool {
			return exactLeaderResult(status, request.RequestID,
				request.CampaignGeneration, "acquired", "acquired_existing",
				"preempted_and_acquired") &&
				exactLeader(status, request.ControllerID,
					request.ControllerBootID, request.CampaignGeneration,
					targets.LeaseCredential{})
		},
	)
	if err != nil {
		return targets.LeaseStatus{}, err
	}
	lease := controller.leaseStatus(request.TargetProfileID, status, receivedAt)
	controller.rememberLease(lease)
	return lease, nil
}

func (controller *NATSController) Renew(
	ctx context.Context,
	request targets.LeaseRenewRequest,
) (targets.LeaseStatus, error) {
	target, natsURL, err := controller.resolve(ctx, request.TargetProfileID)
	if err != nil {
		return targets.LeaseStatus{}, err
	}
	commandID := controller.wireID("leader-renew")
	payload, err := json.Marshal(map[string]any{
		"schema":              RobotV4CommandSchema,
		"command":             "leader_renew",
		"robot":               target.Discovery.CellID,
		"command_id":          commandID,
		"sender_id":           request.ControllerID,
		"controller_boot_id":  request.Credential.ControllerBootID,
		"request_id":          request.RequestID,
		"campaign_generation": request.CampaignGeneration,
		"ttl_ms":              request.TTL.Milliseconds(),
		"lease_id":            request.Credential.LeaseID,
		"fence_epoch":         request.Credential.FenceEpoch,
	})
	if err != nil {
		return targets.LeaseStatus{}, err
	}
	status, receivedAt, err := controller.leaderCommand(
		ctx, target, natsURL, payload, "leader_renew", commandID,
		func(status controllerStatus) bool {
			return exactLeaderResult(status, request.RequestID,
				request.CampaignGeneration, "renewed") &&
				exactLeader(status, request.ControllerID,
					request.Credential.ControllerBootID,
					request.CampaignGeneration, request.Credential)
		},
	)
	if err != nil {
		return targets.LeaseStatus{}, err
	}
	lease := controller.leaseStatus(request.TargetProfileID, status, receivedAt)
	controller.rememberLease(lease)
	return lease, nil
}

func (controller *NATSController) Release(
	ctx context.Context,
	request targets.LeaseReleaseRequest,
) error {
	target, natsURL, err := controller.resolve(ctx, request.TargetProfileID)
	if err != nil {
		return err
	}
	commandID := controller.wireID("leader-release")
	payload, err := json.Marshal(map[string]any{
		"schema":              RobotV4CommandSchema,
		"command":             "leader_release",
		"robot":               target.Discovery.CellID,
		"command_id":          commandID,
		"sender_id":           request.ControllerID,
		"controller_boot_id":  request.Credential.ControllerBootID,
		"request_id":          request.RequestID,
		"campaign_generation": request.CampaignGeneration,
		"lease_id":            request.Credential.LeaseID,
		"fence_epoch":         request.Credential.FenceEpoch,
	})
	if err != nil {
		return err
	}
	_, _, err = controller.leaderCommand(
		ctx, target, natsURL, payload, "leader_release", commandID,
		func(status controllerStatus) bool {
			return exactLeaderResult(status, request.RequestID,
				request.CampaignGeneration, "released") &&
				!status.LeaderLeaseFresh &&
				!status.LeaderTransitionPending
		},
	)
	if err == nil {
		controller.forgetLease(request.TargetProfileID)
	}
	return err
}

func (controller *NATSController) Cancel(
	ctx context.Context,
	request LeaseCancelRequest,
) error {
	target, natsURL, err := controller.resolve(ctx, request.TargetProfileID)
	if err != nil {
		return err
	}
	commandID := controller.wireID("leader-cancel")
	payload, err := json.Marshal(map[string]any{
		"schema":              RobotV4CommandSchema,
		"command":             "leader_cancel",
		"robot":               target.Discovery.CellID,
		"command_id":          commandID,
		"sender_id":           request.Controller.ControllerID,
		"controller_boot_id":  request.Controller.ControllerBootID,
		"request_id":          request.RequestID,
		"campaign_generation": request.CampaignGeneration,
	})
	if err != nil {
		return err
	}
	_, _, err = controller.leaderCommand(
		ctx, target, natsURL, payload, "leader_cancel", commandID,
		func(status controllerStatus) bool {
			return exactLeaderResult(status, request.RequestID,
				request.CampaignGeneration, "cancelled") &&
				!status.LeaderLeaseFresh &&
				!status.LeaderTransitionPending
		},
	)
	if err == nil {
		controller.forgetLease(request.TargetProfileID)
	}
	return err
}

// ValidateFence performs an owner-applied renew with the exact credential.
// This is deliberately stronger than trusting a cached lease observation.
func (controller *NATSController) ValidateFence(
	ctx context.Context,
	request targets.FenceValidationRequest,
) error {
	controller.mu.Lock()
	known, ok := controller.lastApplied[request.TargetProfileID]
	controller.mu.Unlock()
	if !ok ||
		request.ObservedStatus != known ||
		known.ControllerID != request.Controller.ControllerID ||
		known.CampaignGeneration != request.CampaignGeneration ||
		known.Credential != request.Credential {
		return errors.New("execution: exact applied lease is not known")
	}
	now := request.CheckedAt
	if now.IsZero() {
		now = time.Now().UTC()
	}
	remaining := known.FreshUntil.Sub(now)
	if remaining < 500*time.Millisecond {
		return errors.New("execution: lease is too close to expiry for fence validation")
	}
	if remaining > 2*time.Second {
		remaining = 2 * time.Second
	}
	_, err := controller.Renew(ctx, targets.LeaseRenewRequest{
		TargetProfileID:    request.TargetProfileID,
		ControllerID:       request.Controller.ControllerID,
		RequestID:          controller.wireID("fence"),
		CampaignGeneration: request.CampaignGeneration,
		TTL:                remaining,
		Credential:         request.Credential,
	})
	return err
}

func (controller *NATSController) submitMotion(
	ctx context.Context,
	command TransportCommand,
	payload []byte,
	match func(controllerStatus) bool,
	result string,
) (TransportReceipt, error) {
	target, natsURL, err := controller.resolve(
		ctx, command.Identity.Target.ProfileID)
	if err != nil {
		return TransportReceipt{}, err
	}
	bounded, cancel := context.WithTimeout(ctx, controller.timeout)
	defer cancel()
	reply, _, receivedAt, err := controller.exchange.Command(
		bounded,
		natsURL,
		target.Binding.CommandSubject,
		target.Binding.MotionServerStatusSubject,
		payload,
		motionServerReplyComponent,
		match,
	)
	if err != nil {
		return TransportReceipt{}, err
	}
	return transportReceipt(command, reply, receivedAt, result), nil
}

func (controller *NATSController) submitBridge(
	ctx context.Context,
	command TransportCommand,
	wireCommand string,
	fields map[string]any,
	result string,
) (TransportReceipt, error) {
	target, natsURL, err := controller.resolve(
		ctx, command.Identity.Target.ProfileID)
	if err != nil {
		return TransportReceipt{}, err
	}
	payload := controller.baseCommand(command, wireCommand)
	for key, value := range fields {
		payload[key] = value
	}
	raw, err := json.Marshal(payload)
	if err != nil {
		return TransportReceipt{}, err
	}
	bounded, cancel := context.WithTimeout(ctx, controller.timeout)
	defer cancel()
	reply, _, receivedAt, err := controller.exchange.Command(
		bounded,
		natsURL,
		target.Binding.CommandSubject,
		"",
		raw,
		realtimeCoreReplyComponent,
		nil,
	)
	if err != nil {
		return TransportReceipt{}, err
	}
	return transportReceipt(command, reply, receivedAt, result), nil
}

func (controller *NATSController) leaderCommand(
	ctx context.Context,
	target targets.ConnectedManifestTarget,
	natsURL string,
	payload []byte,
	wireCommand string,
	commandID string,
	match func(controllerStatus) bool,
) (controllerStatus, time.Time, error) {
	bounded, cancel := context.WithTimeout(ctx, controller.timeout)
	defer cancel()
	reply, status, receivedAt, err := controller.exchange.Command(
		bounded,
		natsURL,
		target.Binding.CommandSubject,
		target.Binding.MotionServerStatusSubject,
		payload,
		motionServerReplyComponent,
		match,
	)
	if err != nil {
		return controllerStatus{}, time.Time{}, err
	}
	if reply.Command != wireCommand || reply.CommandID != commandID {
		return controllerStatus{}, time.Time{}, ErrTransportReceipt
	}
	return status, receivedAt, nil
}

func (controller *NATSController) resolve(
	ctx context.Context,
	profileID string,
) (targets.ConnectedManifestTarget, string, error) {
	if controller == nil || controller.targets == nil ||
		controller.resolver == nil || controller.exchange == nil {
		return targets.ConnectedManifestTarget{}, "", ErrUnavailable
	}
	target, ok := controller.targets.PlanningTarget(strings.TrimSpace(profileID))
	if !ok || target.Discovery.Target.Kind != targets.TargetRealCell {
		return targets.ConnectedManifestTarget{}, "", ErrUnavailable
	}
	if !roleAllowed(target.Binding.LeaderControllerRoles,
		offlineProgrammingController) {
		return targets.ConnectedManifestTarget{}, "",
			errors.New("execution: target manifest does not allow the offline-programming controller role")
	}
	natsURL, err := controller.resolver.ResolveNATS(
		ctx, target.Binding.MotionServerNATSServiceRef)
	if err != nil {
		return targets.ConnectedManifestTarget{}, "", err
	}
	return target, natsURL, nil
}

func (controller *NATSController) motionCommandPayload(
	command TransportCommand,
	wireCommand string,
	fields map[string]any,
) []byte {
	payload := controller.baseCommand(command, wireCommand)
	if command.Credential != nil {
		payload["controller_boot_id"] = command.Credential.ControllerBootID
		payload["leader_lease_id"] = command.Credential.LeaseID
		payload["leader_fence_epoch"] = command.Credential.FenceEpoch
	}
	payload["campaign_generation"] = command.CampaignGeneration
	payload["motion_intent_seq"] = command.Command.IntentSequence
	for key, value := range fields {
		payload[key] = value
	}
	raw, _ := json.Marshal(payload)
	return raw
}

func (*NATSController) baseCommand(
	command TransportCommand,
	wireCommand string,
) map[string]any {
	return map[string]any{
		"schema":        RobotV4CommandSchema,
		"command":       wireCommand,
		"robot":         command.Identity.Target.CellID,
		"command_id":    command.Command.CommandID,
		"sender_id":     command.Controller.ControllerID,
		"controller_id": command.Controller.ControllerID,
	}
}

func exactAppliedMotion(
	status controllerStatus,
	command TransportCommand,
	kind string,
	result string,
	code uint8,
) bool {
	return status.LatestAppliedIntentSequence == command.Command.IntentSequence &&
		status.LatestAppliedPlanRevision == command.Identity.PlanRevision &&
		status.LatestAppliedResultCode == code &&
		status.LatestAppliedCommandID == command.Command.CommandID &&
		status.LatestAppliedSenderID == command.Controller.ControllerID &&
		status.LatestAppliedCommandKind == kind &&
		status.LatestAppliedResult == result &&
		status.LatestAppliedAccepted
}

func exactLeaderResult(
	status controllerStatus,
	requestID string,
	campaign uint64,
	results ...string,
) bool {
	if status.LatestLeaderRequestID != requestID ||
		status.LatestLeaderCampaignGeneration != campaign ||
		status.LeaderTransitionPending {
		return false
	}
	for _, result := range results {
		if status.LatestLeaderResult == result {
			return true
		}
	}
	return false
}

func exactLeader(
	status controllerStatus,
	controllerID string,
	bootID string,
	campaign uint64,
	credential targets.LeaseCredential,
) bool {
	if !status.LeaderLeaseFresh ||
		status.LeaderID != controllerID ||
		status.LeaderBootID != bootID ||
		status.LeaderCampaignGeneration != campaign ||
		status.LeaderLeaseID == "" ||
		status.LeaderFenceEpoch == 0 ||
		status.LeaderExpiresInMS <= 0 {
		return false
	}
	return credential.LeaseID == "" ||
		(status.LeaderLeaseID == credential.LeaseID &&
			status.LeaderFenceEpoch == credential.FenceEpoch)
}

func (controller *NATSController) leaseStatus(
	profileID string,
	status controllerStatus,
	receivedAt time.Time,
) targets.LeaseStatus {
	sequence := controller.sequence.Add(1)
	return targets.LeaseStatus{
		TargetProfileID:    profileID,
		ControllerID:       status.LeaderID,
		CampaignGeneration: status.LeaderCampaignGeneration,
		Credential: targets.LeaseCredential{
			ControllerBootID: status.LeaderBootID,
			LeaseID:          status.LeaderLeaseID,
			FenceEpoch:       status.LeaderFenceEpoch,
		},
		StatusSequence: sequence,
		FreshUntil: receivedAt.Add(
			time.Duration(status.LeaderExpiresInMS) * time.Millisecond,
		).UTC(),
		TransitionPending: status.LeaderTransitionPending,
	}
}

func (controller *NATSController) rememberLease(status targets.LeaseStatus) {
	controller.mu.Lock()
	defer controller.mu.Unlock()
	controller.lastApplied[status.TargetProfileID] = status
}

func (controller *NATSController) forgetLease(profileID string) {
	controller.mu.Lock()
	defer controller.mu.Unlock()
	delete(controller.lastApplied, profileID)
}

func (controller *NATSController) wireID(label string) string {
	return fmt.Sprintf("op-%s-%d", label, controller.sequence.Add(1))
}

func roleAllowed(roles []string, wanted string) bool {
	for _, role := range roles {
		if role == wanted {
			return true
		}
	}
	return false
}

func exactRTCoreArmedStatus(
	envelope controllerStatus,
	expectedSubject string,
) bool {
	status := envelope.Status
	if !exactRTCoreStatusIdentity(envelope, expectedSubject) ||
		status.ServosArmedRequested == nil ||
		!*status.ServosArmedRequested ||
		status.DS402EnabledAxes != status.ConfiguredAxisCount {
		return false
	}
	allAxes := uint32(1<<uint(status.ConfiguredAxisCount)) - 1
	return status.AxisEnableMask == allAxes
}

func exactRTCoreReadyToArmStatus(
	envelope controllerStatus,
	expectedSubject string,
) bool {
	status := envelope.Status
	return exactRTCoreStatusIdentity(envelope, expectedSubject) &&
		status.ServosArmedRequested != nil &&
		!*status.ServosArmedRequested &&
		status.AxisEnableMask == 0 &&
		status.DS402EnabledAxes == 0
}

func exactRTCoreStatusIdentity(
	envelope controllerStatus,
	expectedSubject string,
) bool {
	status := envelope.Status
	now := time.Now().UTC()
	if envelope.Schema != "robot.v4.rtcore.status.v1" ||
		envelope.Subject != expectedSubject ||
		envelope.Host != rtcoreStatusHost(expectedSubject) ||
		envelope.Component != "rtcore" ||
		!freshRTCoreSourceTime(envelope.PublishedAt, now) ||
		!freshRTCoreSourceTime(status.UpdatedAt, now) ||
		status.Name != "robot-v4-rtcore" ||
		status.State != "running" ||
		status.RTCoreMode != "live" ||
		status.RuntimeModeEpoch == 0 ||
		status.EtherCATOutputEnabled == nil ||
		!*status.EtherCATOutputEnabled ||
		status.MotionOutputEnabled == nil ||
		!*status.MotionOutputEnabled ||
		status.ConfiguredAxisCount <= 0 ||
		status.ConfiguredAxisCount > 16 ||
		status.AxisErrorMask != 0 ||
		status.AxisFaultMask != 0 {
		return false
	}
	readback := status.RevPiOutputReadback
	weldBit := uint16(1 << 4)
	return readback.OutputCount == 14 &&
		readback.Fresh &&
		readback.ValidMask&((1<<14)-1) == (1<<14)-1 &&
		readback.ValuesMask&weldBit == 0
}

func freshRTCoreSourceTime(raw string, now time.Time) bool {
	value, err := time.Parse(time.RFC3339Nano, strings.TrimSpace(raw))
	if err != nil ||
		value.After(now.Add(time.Second)) ||
		now.Sub(value) > 3*time.Second {
		return false
	}
	return true
}

func rtcoreStatusHost(subject string) string {
	const prefix = "robot/v4/rtcore."
	const suffix = ".status"
	if !strings.HasPrefix(subject, prefix) ||
		!strings.HasSuffix(subject, suffix) {
		return ""
	}
	return strings.TrimSuffix(strings.TrimPrefix(subject, prefix), suffix)
}

func transportReceipt(
	command TransportCommand,
	reply controllerCommandReply,
	recordedAt time.Time,
	result string,
) TransportReceipt {
	return TransportReceipt{
		Schema:         TransportReceiptSchema,
		Action:         command.Action,
		CommandID:      command.Command.CommandID,
		IdempotencyKey: command.Command.IdempotencyKey,
		Accepted:       reply.Accepted,
		Applied:        reply.Accepted,
		Duplicate:      reply.Duplicate,
		ServerOwned:    true,
		RecordedAt:     recordedAt.UTC(),
		Identity:       command.Identity,
		Result:         result,
	}
}

type natsControllerExchange struct{}

func (*natsControllerExchange) Command(
	ctx context.Context,
	natsURL string,
	commandSubject string,
	statusSubject string,
	payload []byte,
	expectedComponent string,
	match func(controllerStatus) bool,
) (controllerCommandReply, controllerStatus, time.Time, error) {
	if len(payload) == 0 || len(payload) > maxControllerCommandBytes {
		return controllerCommandReply{}, controllerStatus{}, time.Time{},
			errors.New("execution: controller command payload is invalid")
	}
	connection, err := nats.Connect(
		natsURL,
		nats.Name("offline-programming-v1-execution"),
		nats.Timeout(3*time.Second),
		nats.MaxReconnects(0),
		nats.NoEcho(),
	)
	if err != nil {
		return controllerCommandReply{}, controllerStatus{}, time.Time{}, err
	}
	defer connection.Close()
	var statuses *nats.Subscription
	if match != nil {
		statuses, err = connection.SubscribeSync(statusSubject)
		if err != nil {
			return controllerCommandReply{}, controllerStatus{}, time.Time{}, err
		}
	}
	if err := connection.FlushWithContext(ctx); err != nil {
		return controllerCommandReply{}, controllerStatus{}, time.Time{}, err
	}
	replyMessage, err := connection.RequestWithContext(ctx, commandSubject, payload)
	if err != nil {
		return controllerCommandReply{}, controllerStatus{}, time.Time{}, err
	}
	var sent struct {
		Command   string `json:"command"`
		CommandID string `json:"command_id"`
	}
	var reply controllerCommandReply
	if json.Unmarshal(payload, &sent) != nil ||
		json.Unmarshal(replyMessage.Data, &reply) != nil ||
		reply.Schema != commandReplySchema ||
		reply.Component != expectedComponent ||
		reply.Command != sent.Command ||
		reply.CommandID != sent.CommandID ||
		!reply.Accepted {
		return controllerCommandReply{}, controllerStatus{}, time.Time{},
			errors.New("execution: owning Robot V4 component rejected the command")
	}
	if match == nil {
		return reply, controllerStatus{}, time.Now().UTC(), nil
	}
	for {
		message, err := statuses.NextMsgWithContext(ctx)
		if err != nil {
			return controllerCommandReply{}, controllerStatus{}, time.Time{}, err
		}
		var status controllerStatus
		if json.Unmarshal(message.Data, &status) == nil && match(status) {
			return reply, status, time.Now().UTC(), nil
		}
	}
}

func (*natsControllerExchange) Observe(
	ctx context.Context,
	natsURL string,
	statusSubject string,
	match func(controllerStatus) bool,
) (controllerStatus, time.Time, error) {
	connection, err := nats.Connect(
		natsURL,
		nats.Name("offline-programming-v1-execution-observation"),
		nats.Timeout(3*time.Second),
		nats.MaxReconnects(0),
		nats.NoEcho(),
	)
	if err != nil {
		return controllerStatus{}, time.Time{}, err
	}
	defer connection.Close()
	statuses, err := connection.SubscribeSync(statusSubject)
	if err != nil {
		return controllerStatus{}, time.Time{}, err
	}
	if err := connection.FlushWithContext(ctx); err != nil {
		return controllerStatus{}, time.Time{}, err
	}
	for {
		message, err := statuses.NextMsgWithContext(ctx)
		if err != nil {
			return controllerStatus{}, time.Time{}, err
		}
		var status controllerStatus
		if json.Unmarshal(message.Data, &status) == nil &&
			(match == nil || match(status)) {
			return status, time.Now().UTC(), nil
		}
	}
}

// Compile-time assertions keep the one production adapter on both authority
// boundaries used by the execution gateway.
var _ ServerTransport = (*NATSController)(nil)
var _ LeaseAuthority = (*NATSController)(nil)
