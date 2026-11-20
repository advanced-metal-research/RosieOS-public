package execution

import (
	"context"
	"encoding/json"
	"errors"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/targets"
)

type controllerTargetFixture struct {
	target targets.ConnectedManifestTarget
}

func (source controllerTargetFixture) PlanningTarget(
	profileID string,
) (targets.ConnectedManifestTarget, bool) {
	return source.target, profileID == source.target.Discovery.ProfileID
}

type controllerResolverFixture struct {
	calls int
}

func (resolver *controllerResolverFixture) ResolveNATS(
	context.Context,
	string,
) (string, error) {
	resolver.calls++
	return "nats://server-owned.test:4222", nil
}

type controllerExchangeFixture struct {
	t               *testing.T
	observed        controllerStatus
	commandStatuses func(map[string]any) controllerStatus
	payloads        []map[string]any
}

func (exchange *controllerExchangeFixture) Observe(
	_ context.Context,
	_ string,
	_ string,
	match func(controllerStatus) bool,
) (controllerStatus, time.Time, error) {
	if match != nil && !match(exchange.observed) {
		return controllerStatus{}, time.Time{},
			errors.New("fixture observation did not satisfy exact matcher")
	}
	return exchange.observed, time.Now().UTC(), nil
}

func (exchange *controllerExchangeFixture) Command(
	_ context.Context,
	_ string,
	_ string,
	_ string,
	raw []byte,
	component string,
	match func(controllerStatus) bool,
) (controllerCommandReply, controllerStatus, time.Time, error) {
	var payload map[string]any
	if err := json.Unmarshal(raw, &payload); err != nil {
		exchange.t.Fatalf("decode controller payload: %v", err)
	}
	exchange.payloads = append(exchange.payloads, payload)
	status := controllerStatus{}
	if exchange.commandStatuses != nil {
		status = exchange.commandStatuses(payload)
	}
	if match != nil && !match(status) {
		return controllerCommandReply{}, controllerStatus{}, time.Time{},
			errors.New("fixture command status did not satisfy exact matcher")
	}
	command, _ := payload["command"].(string)
	commandID, _ := payload["command_id"].(string)
	return controllerCommandReply{
		Schema:    commandReplySchema,
		Component: component,
		Command:   command,
		CommandID: commandID,
		Accepted:  true,
		State:     "accepted",
	}, status, time.Now().UTC(), nil
}

func TestNATSControllerRequiresManifestAuthorizedRole(t *testing.T) {
	target := executionControllerTarget()
	target.Binding.LeaderControllerRoles = []string{"steamdeck"}
	resolver := &controllerResolverFixture{}
	controller, err := NewNATSController(NATSControllerOptions{
		Targets:  controllerTargetFixture{target: target},
		Resolver: resolver,
		Exchange: &controllerExchangeFixture{t: t},
	})
	if err != nil {
		t.Fatal(err)
	}
	_, err = controller.Acquire(context.Background(), targets.LeaseAcquireRequest{
		TargetProfileID:    target.Discovery.ProfileID,
		ControllerID:       "offline-programming:server-test",
		ControllerBootID:   "op-test",
		RequestID:          "acquire-test",
		CampaignGeneration: 1,
		TTL:                time.Second,
	})
	if err == nil || resolver.calls != 0 {
		t.Fatalf(
			"unauthorized role reached resolver: calls=%d err=%v",
			resolver.calls,
			err,
		)
	}
}

func TestNATSControllerAcquiresAndRevalidatesExactAppliedFence(t *testing.T) {
	target := executionControllerTarget()
	exchange := &controllerExchangeFixture{t: t}
	exchange.commandStatuses = func(payload map[string]any) controllerStatus {
		command := payload["command"].(string)
		campaign := uint64(payload["campaign_generation"].(float64))
		status := controllerStatus{
			LeaderID:                       payload["sender_id"].(string),
			LeaderBootID:                   payload["controller_boot_id"].(string),
			LeaderLeaseID:                  "lease-exact-1",
			LeaderFenceEpoch:               9,
			LeaderCampaignGeneration:       campaign,
			LeaderLeaseFresh:               true,
			LeaderExpiresInMS:              1500,
			LatestLeaderResult:             "acquired",
			LatestLeaderRequestID:          payload["request_id"].(string),
			LatestLeaderCampaignGeneration: campaign,
		}
		if command == "leader_renew" {
			status.LatestLeaderResult = "renewed"
		}
		return status
	}
	controller, err := NewNATSController(NATSControllerOptions{
		Targets:  controllerTargetFixture{target: target},
		Resolver: &controllerResolverFixture{},
		Exchange: exchange,
	})
	if err != nil {
		t.Fatal(err)
	}
	request := targets.LeaseAcquireRequest{
		TargetProfileID:    target.Discovery.ProfileID,
		ControllerID:       "offline-programming:server-test",
		ControllerBootID:   "op-test",
		RequestID:          "acquire-test",
		CampaignGeneration: 4,
		TTL:                1500 * time.Millisecond,
	}
	lease, err := controller.Acquire(context.Background(), request)
	if err != nil {
		t.Fatal(err)
	}
	if lease.ControllerID != request.ControllerID ||
		lease.Credential.ControllerBootID != request.ControllerBootID ||
		lease.Credential.LeaseID != "lease-exact-1" ||
		lease.Credential.FenceEpoch != 9 {
		t.Fatalf("acquired lease identity changed: %+v", lease)
	}
	fenceRequest := targets.FenceValidationRequest{
		TargetProfileID: request.TargetProfileID,
		Controller: targets.ControllerIdentity{
			ControllerID:     request.ControllerID,
			ControllerBootID: request.ControllerBootID,
		},
		CampaignGeneration: request.CampaignGeneration,
		Credential:         lease.Credential,
		ObservedStatus:     lease,
		CheckedAt:          time.Now().UTC(),
	}
	fenceRequest.ObservedStatus.StatusSequence++
	if err := controller.ValidateFence(
		context.Background(),
		fenceRequest,
	); err == nil {
		t.Fatal("tampered observed lease was accepted for application-time fencing")
	}
	fenceRequest.ObservedStatus = lease
	if err := controller.ValidateFence(
		context.Background(),
		fenceRequest,
	); err != nil {
		t.Fatalf("exact fence revalidation: %v", err)
	}
	if len(exchange.payloads) != 2 ||
		exchange.payloads[0]["command"] != "leader_acquire" ||
		exchange.payloads[1]["command"] != "leader_renew" {
		t.Fatalf("unexpected authority exchange: %+v", exchange.payloads)
	}
}

func TestNATSControllerPreloadAndPlayUseExactAcceptedPlanIdentity(
	t *testing.T,
) {
	harness := newGatewayHarness(t)
	target := executionControllerTarget()
	exchange := &controllerExchangeFixture{t: t}
	exchange.commandStatuses = func(payload map[string]any) controllerStatus {
		command := payload["command"].(string)
		status := controllerStatus{
			PlanPreloaded:               true,
			LoadedProgramAvailable:      true,
			LoadedProgramPlayable:       true,
			LoadedProgramRevision:       harness.input.Identity.PlanRevision,
			PlanRevision:                harness.input.Identity.PlanRevision,
			PlanSessionID:               harness.input.Identity.PlanID,
			AcceptedPlanID:              harness.input.Identity.PlanID,
			AcceptedProgramDigest:       harness.input.Identity.ProgramDigest,
			AcceptedTrajectoryDigest:    harness.input.Identity.TrajectoryDigest,
			AcceptedManifestRevision:    harness.input.Identity.Target.ManifestRevision,
			LatestAppliedIntentSequence: uint64(payload["motion_intent_seq"].(float64)),
			LatestAppliedPlanRevision:   harness.input.Identity.PlanRevision,
			LatestAppliedCommandID:      payload["command_id"].(string),
			LatestAppliedSenderID:       payload["sender_id"].(string),
			LatestAppliedCommandKind:    command,
			LatestAppliedAccepted:       true,
		}
		if command == "plan_preload" {
			status.LatestAppliedResultCode = 6
			status.LatestAppliedResult = "accepted_plan_preloaded"
		} else {
			status.LatestAppliedResultCode = 1
			status.LatestAppliedResult = "playback_started"
			status.PlanPlaybackActive = true
		}
		return status
	}
	controller, err := NewNATSController(NATSControllerOptions{
		Targets:  controllerTargetFixture{target: target},
		Resolver: &controllerResolverFixture{},
		Exchange: exchange,
	})
	if err != nil {
		t.Fatal(err)
	}
	gateway := NewGateway(harness.config)
	token := ReviewToken{
		TokenID:    "review-exact",
		Controller: harness.config.Bootstrap.Controller,
		Identity:   harness.input.Identity,
	}
	credential := targets.LeaseCredential{
		ControllerBootID: token.Controller.ControllerBootID,
		LeaseID:          "lease-exact",
		FenceEpoch:       7,
	}
	reference := harness.input.AcceptedPlan.Reference()
	preload := gateway.command(
		ActionPreload,
		"session-exact",
		token,
		3,
		&credential,
		&reference,
		"",
	)
	receipt, err := controller.Preload(context.Background(), preload)
	if err != nil || !receipt.Identity.Equal(harness.input.Identity) {
		t.Fatalf("exact preload receipt=%+v err=%v", receipt, err)
	}
	play := gateway.command(
		ActionPlay,
		"session-exact",
		token,
		3,
		&credential,
		nil,
		"",
	)
	receipt, err = controller.Play(context.Background(), play)
	if err != nil || !receipt.Identity.Equal(harness.input.Identity) {
		t.Fatalf("exact play receipt=%+v err=%v", receipt, err)
	}
	if len(exchange.payloads) != 2 {
		t.Fatalf(
			"expected preload and play payloads, got %+v",
			exchange.payloads,
		)
	}
	accepted, ok := exchange.payloads[0]["accepted_plan"].(map[string]any)
	if !ok ||
		accepted["plan_id"] != harness.input.Identity.PlanID ||
		exchange.payloads[1]["trajectory_digest"] !=
			harness.input.Identity.TrajectoryDigest {
		t.Fatalf(
			"exact accepted plan identity missing: %+v",
			exchange.payloads,
		)
	}
}

func TestNATSControllerControlledArmUsesFreshExactRuntimeModeEpoch(
	t *testing.T,
) {
	harness := newGatewayHarness(t)
	target := executionControllerTarget()
	exchange := &controllerExchangeFixture{t: t}
	exchange.observed = rtcoreControllerStatus(
		target.Binding.RealtimeCoreStatusSubject,
		time.Now().UTC(),
		false,
		17,
	)
	exchange.commandStatuses = func(map[string]any) controllerStatus {
		return rtcoreControllerStatus(
			target.Binding.RealtimeCoreStatusSubject,
			time.Now().UTC(),
			true,
			17,
		)
	}
	controller, err := NewNATSController(NATSControllerOptions{
		Targets:  controllerTargetFixture{target: target},
		Resolver: &controllerResolverFixture{},
		Exchange: exchange,
	})
	if err != nil {
		t.Fatal(err)
	}
	gateway := NewGateway(harness.config)
	token := ReviewToken{
		TokenID:    "review-arm",
		Controller: harness.config.Bootstrap.Controller,
		Identity:   harness.input.Identity,
	}
	credential := targets.LeaseCredential{
		ControllerBootID: token.Controller.ControllerBootID,
		LeaseID:          "lease-arm",
		FenceEpoch:       11,
	}
	command := gateway.command(
		ActionArm,
		"session-arm",
		token,
		5,
		&credential,
		nil,
		"",
	)
	if _, err := controller.Arm(context.Background(), command); err != nil {
		t.Fatalf("controlled arm: %v", err)
	}
	if len(exchange.payloads) != 1 ||
		exchange.payloads[0]["runtime_mode"] != "live" ||
		exchange.payloads[0]["runtime_mode_epoch"] != float64(17) ||
		exchange.payloads[0]["leader_lease_id"] != credential.LeaseID ||
		exchange.payloads[0]["leader_fence_epoch"] !=
			float64(credential.FenceEpoch) {
		t.Fatalf(
			"controlled arm payload lost exact mode/fence: %+v",
			exchange.payloads,
		)
	}

	exchange.observed = rtcoreControllerStatus(
		target.Binding.RealtimeCoreStatusSubject,
		time.Now().UTC().Add(-4*time.Second),
		false,
		17,
	)
	if _, err := controller.Arm(context.Background(), command); err == nil {
		t.Fatal("stale RTCore readiness was accepted")
	}
}

func executionControllerTarget() targets.ConnectedManifestTarget {
	return targets.ConnectedManifestTarget{
		Discovery: targets.ConnectedTargetDiscoveryDescriptor{
			ProfileID: "bench-relay",
			CellID:    "bench-relay",
			Target: targets.TargetDescriptor{
				Kind: targets.TargetRealCell,
			},
		},
		Binding: targets.ConnectedTargetServerBinding{
			ServerTargetBinding: targets.ServerTargetBinding{
				CommandSubject:            "robot/v4/bench-relay.command",
				MotionServerStatusSubject: "robot/v4/motion-server.bench-relay.status",
				MotionServerPlanSubject:   "robot/v4/motion-server.bench-relay.plan",
				RealtimeCoreStatusSubject: "robot/v4/rtcore.bench-relay.status",
			},
			MotionServerNATSServiceRef: "daemon-v1:robot-v4-nats",
			LeaderControllerRoles:      []string{"steamdeck", "offline-programming"},
		},
	}
}

func rtcoreControllerStatus(
	subject string,
	sourceTime time.Time,
	armed bool,
	epoch uint64,
) controllerStatus {
	status := controllerStatus{
		Schema:      "robot.v4.rtcore.status.v1",
		Subject:     subject,
		Host:        rtcoreStatusHost(subject),
		Component:   "rtcore",
		PublishedAt: sourceTime.Format(time.RFC3339Nano),
	}
	status.Status.Name = "robot-v4-rtcore"
	status.Status.State = "running"
	status.Status.UpdatedAt = sourceTime.Format(time.RFC3339Nano)
	status.Status.RTCoreMode = "live"
	status.Status.RuntimeModeEpoch = epoch
	status.Status.EtherCATOutputEnabled = boolPointer(true)
	status.Status.MotionOutputEnabled = boolPointer(true)
	status.Status.ServosArmedRequested = boolPointer(armed)
	status.Status.ConfiguredAxisCount = 9
	status.Status.RevPiOutputReadback.OutputCount = 14
	status.Status.RevPiOutputReadback.ValidMask = (1 << 14) - 1
	status.Status.RevPiOutputReadback.Fresh = true
	if armed {
		status.Status.DS402EnabledAxes = 9
		status.Status.AxisEnableMask = (1 << 9) - 1
	}
	return status
}

func boolPointer(value bool) *bool {
	return &value
}
