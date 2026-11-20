package targets

import (
	"context"
	"errors"
	"testing"
	"time"
)

func testAxes() []AxisDescriptor {
	return []AxisDescriptor{{
		Index:                 0,
		Name:                  "J1",
		URDFJoint:             "J1",
		Kind:                  "arm",
		GearRatio:             100,
		Sign:                  1,
		CartesianParticipates: true,
	}}
}

func TestModeMachineRequiresSimulatorAndRealTargetIdentityProof(t *testing.T) {
	now := time.Date(2026, 7, 30, 9, 0, 0, 0, time.UTC)
	machine := NewModeMachine(now)
	simulator, err := LocalSimulatorDescriptor("local-simulator", testAxes())
	if err != nil {
		t.Fatalf("create simulator descriptor: %v", err)
	}
	if state, err := machine.Select(now.Add(time.Second), simulator, &SimulatorSafetyProof{
		ProfileID:           simulator.ProfileID,
		StatusSequence:      2,
		ObservedAt:          now,
		FreshUntil:          now.Add(2 * time.Second),
		NoRealMovement:      false,
		EtherCATEnabled:     false,
		MotionOutputEnabled: false,
	}); err == nil || state.Mode != TargetOfflinePreview || state.Phase != PhaseBlocked {
		t.Fatalf("unsafe simulator selection did not fail closed: state=%+v err=%v", state, err)
	}

	real := TargetDescriptor{
		Schema:         TargetDescriptorSchema,
		ProfileID:      "bench-relay",
		Label:          "Bench relay",
		Kind:           TargetRealCell,
		ManifestRef:    "rt-core/config/cells/bench-relay.json",
		ManifestSchema: "robot_v4_deployment_manifest_v1",
		CellID:         "bench-relay",
		Axes:           testAxes(),
		Capabilities: TargetCapabilities{
			Monitoring: true,
			Planning:   true,
		},
	}
	state, err := machine.Select(now.Add(2*time.Second), real, nil)
	if err != nil || state.Phase != PhaseConnecting {
		t.Fatalf("select real descriptor: state=%+v err=%v", state, err)
	}
	state, err = machine.Observe(now.Add(3*time.Second), TargetObservation{
		ProfileID:      "other-relay",
		ManifestRef:    real.ManifestRef,
		StatusSequence: 5,
		ObservedAt:     now.Add(2 * time.Second),
		FreshUntil:     now.Add(4 * time.Second),
		Connected:      true,
	})
	if err == nil || state.Phase != PhaseBlocked {
		t.Fatalf("mismatched real observation did not block: state=%+v err=%v", state, err)
	}
}

func TestCommandAuthorityAndIdempotencyFailClosed(t *testing.T) {
	now := time.Date(2026, 7, 30, 9, 30, 0, 0, time.UTC)
	credential := LeaseCredential{
		ControllerBootID: "server-boot-1",
		LeaseID:          "server-lease-1",
		FenceEpoch:       7,
	}
	authority := LeaseStatus{
		TargetProfileID: "bench-relay",
		ControllerID:    "offline-programming:server-1",
		Credential:      credential,
		StatusSequence:  14,
		FreshUntil:      now.Add(time.Second),
	}
	request := CommandRequest{
		Kind:            CommandPlay,
		TargetProfileID: "bench-relay",
		Service:         "motion-server",
		ControllerID:    authority.ControllerID,
		Identity: CommandIdentity{
			CommandID:        "execute-17",
			IdempotencyKey:   CommandIdempotencyKey("bench-relay", "motion-server", "execute-17"),
			IntentSequence:   17,
			ControllerBootID: credential.ControllerBootID,
			CreatedAt:        now.Add(-time.Millisecond),
		},
		Plan: CommandPlanIdentity{
			PlanID:          "plan-17",
			PlanRevision:    23,
			ProgramID:       "program-17",
			ProgramRevision: 11,
			ProgramHash:     "fnv1a64:1234",
		},
	}
	if err := request.Validate(now, authority); err == nil {
		t.Fatal("energizing command without lease credential was accepted")
	}
	request.Credential = &credential
	if err := request.Validate(now, authority); err != nil {
		t.Fatalf("exact command identity was rejected: %v", err)
	}
	request.Identity.IdempotencyKey = "execute-17"
	if err := request.Validate(now, authority); err == nil {
		t.Fatal("incorrect idempotency scope was accepted")
	}

	stop := CommandRequest{
		Kind:            CommandStop,
		TargetProfileID: "bench-relay",
		Service:         "motion-server",
		ControllerID:    authority.ControllerID,
		Identity: CommandIdentity{
			CommandID:        "stop-18",
			IdempotencyKey:   CommandIdempotencyKey("bench-relay", "motion-server", "stop-18"),
			IntentSequence:   18,
			ControllerBootID: credential.ControllerBootID,
			CreatedAt:        now.Add(-time.Millisecond),
		},
		Credential: &LeaseCredential{
			ControllerBootID: "stale-boot",
			LeaseID:          "stale-lease",
			FenceEpoch:       1,
		},
	}
	if err := stop.Validate(now, LeaseStatus{}); err != nil {
		t.Fatalf("fail-safe Stop depended on a fresh lease: %v", err)
	}
}

func TestAuthorityCommandsAndTeleopRemainDisabled(t *testing.T) {
	ctx := context.Background()
	authority := DisabledAuthorityController{}
	if _, err := authority.Acquire(ctx, LeaseAcquireRequest{}); !errors.Is(err, ErrAuthorityDisabled) {
		t.Fatalf("disabled authority acquire error = %v", err)
	}
	gateway := DisabledCommandGateway{}
	if receipt, err := gateway.Submit(ctx, CommandRequest{}); !errors.Is(err, ErrCommandExecutionDisabled) || receipt.Accepted {
		t.Fatalf("disabled command gateway = %+v, %v", receipt, err)
	}
	capability := DisabledTeleopCapability()
	if capability.Enabled || !capability.ServerOwned || capability.RawBrowserTransport {
		t.Fatalf("tele-op capability is not disabled/server-owned: %+v", capability)
	}
	teleop := DisabledHotCommandService{}
	if err := teleop.Submit(ctx, HotCommand{}); !errors.Is(err, ErrTeleopDisabled) {
		t.Fatalf("disabled tele-op submit error = %v", err)
	}
}

func TestFutureHotCommandBoundaryChecksFreshnessBoundsAndFence(t *testing.T) {
	now := time.Date(2026, 7, 30, 10, 0, 0, 0, time.UTC)
	credential := LeaseCredential{
		ControllerBootID: "server-boot-2",
		LeaseID:          "server-lease-2",
		FenceEpoch:       9,
	}
	authority := LeaseStatus{
		TargetProfileID: "local-simulator",
		ControllerID:    "offline-programming:server-1",
		Credential:      credential,
		StatusSequence:  20,
		FreshUntil:      now.Add(time.Second),
	}
	policy := HotCommandPolicy{
		MaximumInputAge:    100 * time.Millisecond,
		MaximumRateHz:      50,
		RequireDeadman:     true,
		RequireReleaseZero: true,
	}
	for index := range policy.MaximumAxisRadS {
		policy.MaximumAxisRadS[index] = 0.1
	}
	command := HotCommand{
		TargetProfileID: "local-simulator",
		ControllerID:    authority.ControllerID,
		CommandID:       "hot-1",
		InputSequence:   1,
		SampledAt:       now.Add(-10 * time.Millisecond),
		ExpiresAt:       now.Add(50 * time.Millisecond),
		DeadmanHeld:     true,
		CommandRateHz:   50,
		Credential:      credential,
	}
	if err := ValidateHotCommand(command, policy, now, authority); err != nil {
		t.Fatalf("valid future hot command rejected: %v", err)
	}
	command.AxisVelocityRadS[2] = 0.2
	if err := ValidateHotCommand(command, policy, now, authority); err == nil {
		t.Fatal("out-of-bounds future hot command accepted")
	}
}
