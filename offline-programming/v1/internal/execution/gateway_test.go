package execution

import (
	"context"
	"errors"
	"reflect"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/connected"
	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/targets"
)

type gatewayHarness struct {
	clock     *DeterministicClock
	log       *DeterministicOperationLog
	transport *DeterministicTestTransport
	authority *DeterministicTestAuthority
	evidence  *DeterministicTestEvidenceVerifier
	audit     *DeterministicTestAuditSink
	store     *DeterministicTestIdempotencyStore
	config    GatewayConfig
	input     ExecutionReviewInput
}

type rejectingActiveObserver struct {
	mu    sync.Mutex
	calls int
}

func (observer *rejectingActiveObserver) VerifyActive(
	context.Context,
	ExecutionSession,
	time.Time,
) error {
	observer.mu.Lock()
	observer.calls++
	observer.mu.Unlock()
	return errors.New("injected active read-back mismatch")
}

func (observer *rejectingActiveObserver) Calls() int {
	observer.mu.Lock()
	defer observer.mu.Unlock()
	return observer.calls
}

func TestGatewayDefaultsUnavailableAndDoesNotTouchTransport(t *testing.T) {
	harness := newGatewayHarness(t)
	gateway := NewGateway(GatewayConfig{Clock: harness.clock})
	capabilities := gateway.Capabilities()
	if capabilities.InfrastructureReady ||
		capabilities.PlanPreload ||
		capabilities.ControlledStartArm ||
		capabilities.PlanPlay ||
		capabilities.ProtectiveStop ||
		capabilities.Arm ||
		capabilities.WeldOn ||
		capabilities.Jog ||
		capabilities.Teleop ||
		capabilities.RawBrowserTransport {
		t.Fatalf("default gateway exposed capability: %+v", capabilities)
	}
	for _, reason := range []string{
		"gateway_feature_disabled",
		"manifest_allowlist_not_injected",
		"server_transport_not_injected",
		"robot_v4_server_controller_authority_not_enabled",
	} {
		if !containsString(capabilities.Reasons, reason) {
			t.Fatalf("missing default reason %q: %+v", reason, capabilities.Reasons)
		}
	}
	review, err := gateway.Review(context.Background(), harness.input)
	if !errors.Is(err, ErrReviewDenied) || review.Ready || review.Token != nil {
		t.Fatalf("default review did not fail closed: review=%+v err=%v", review, err)
	}
	if entries := harness.log.Entries(); len(entries) != 0 {
		t.Fatalf("default gateway used a transport: %+v", entries)
	}
}

func TestGatewayOrdersAcquireFencePreloadArmPlayThenProtectiveStopAndRelease(t *testing.T) {
	harness := newGatewayHarness(t)
	gateway := NewGateway(harness.config)
	review, err := gateway.Review(context.Background(), harness.input)
	if err != nil || !review.Ready || review.Token == nil {
		t.Fatalf("review failed: review=%+v err=%v", review, err)
	}
	session, err := gateway.Start(context.Background(), StartRequest{
		RequestID:   "start-1",
		ReviewToken: review.Token.TokenID,
		Review:      harness.input,
	})
	if err != nil {
		t.Fatalf("start: %v", err)
	}
	wantStart := []string{
		"lease_acquire",
		"fence_validate",
		"preload",
		"lease_renew",
		"fence_validate",
		"arm_for_reviewed_start",
		"lease_renew",
		"fence_validate",
		"play",
	}
	if got := harness.log.Entries(); !reflect.DeepEqual(got, wantStart) {
		t.Fatalf("start order = %+v, want %+v", got, wantStart)
	}
	if !session.Identity.Equal(harness.input.Identity) ||
		!session.PreloadReceipt.Identity.Equal(harness.input.Identity) ||
		!session.ArmReceipt.Identity.Equal(harness.input.Identity) ||
		!session.PlayReceipt.Identity.Equal(harness.input.Identity) {
		t.Fatalf("session lost exact identity: %+v", session)
	}
	if session.PreloadReceipt.CommandID == session.PlayReceipt.CommandID ||
		session.PreloadReceipt.IdempotencyKey ==
			session.PlayReceipt.IdempotencyKey ||
		!strings.HasPrefix(
			session.PreloadReceipt.IdempotencyKey,
			harness.input.Identity.Target.ProfileID+"+"+MotionServerService+"+",
		) {
		t.Fatalf("command identity/idempotency is not exact: %+v %+v",
			session.PreloadReceipt, session.PlayReceipt)
	}

	if err := gateway.Stop(context.Background(), StopRequest{
		RequestID: "stop-1",
		SessionID: session.SessionID,
		Reason:    "operator_stop",
	}); err != nil {
		t.Fatalf("stop: %v", err)
	}
	wantAll := append(wantStart, "stop", "disarm", "weld_off", "lease_release")
	if got := harness.log.Entries(); !reflect.DeepEqual(got, wantAll) {
		t.Fatalf("full order = %+v, want %+v", got, wantAll)
	}
	audits := harness.audit.Envelopes()
	if len(audits) < 10 {
		t.Fatalf("expected review/authority/command/protective audits, got %d", len(audits))
	}
	seenIDs := map[string]struct{}{}
	for _, envelope := range audits {
		if envelope.Schema != AuditSchema ||
			envelope.EventID == "" ||
			envelope.Controller != harness.config.Bootstrap.Controller ||
			!envelope.Identity.Equal(harness.input.Identity) {
			t.Fatalf("malformed audit envelope: %+v", envelope)
		}
		if _, duplicate := seenIDs[envelope.EventID]; duplicate {
			t.Fatalf("duplicate audit event id %q", envelope.EventID)
		}
		seenIDs[envelope.EventID] = struct{}{}
	}
}

func TestGatewayReviewRejectsAdversarialEvidenceBeforeAuthorityOrTransport(t *testing.T) {
	tests := []struct {
		name   string
		mutate func(*gatewayHarness)
		reason string
	}{
		{
			name: "project revision mismatch",
			mutate: func(harness *gatewayHarness) {
				harness.input.Evidence.Identity.ProjectRevision++
			},
			reason: "plan_evidence_identity_exact",
		},
		{
			name: "same target name changed manifest bytes",
			mutate: func(harness *gatewayHarness) {
				revision, _ := connected.ManifestRevision([]byte(`{"name":"changed"}`))
				harness.input.Identity.Target.ManifestRevision = revision
			},
			reason: "manifest_revision_allowlisted",
		},
		{
			name: "stale status",
			mutate: func(harness *gatewayHarness) {
				harness.input.Status.FreshUntil = harness.clock.Now()
			},
			reason: "fresh_authoritative_status_required",
		},
		{
			name: "motion server no longer ready",
			mutate: func(harness *gatewayHarness) {
				harness.input.Status.ServiceState = "stopped"
			},
			reason: "authoritative_target_ready_for_preload",
		},
		{
			name: "weld output already energized",
			mutate: func(harness *gatewayHarness) {
				harness.input.Status.Output.WeldOutput.Value = true
			},
			reason: "safe_runtime_output_state",
		},
		{
			name: "runtime preload evidence unavailable",
			mutate: func(harness *gatewayHarness) {
				harness.input.RuntimePlan.RuntimePreload.Value = false
			},
			reason: "authoritative_target_ready_for_preload",
		},
		{
			name: "raw physical io action",
			mutate: func(harness *gatewayHarness) {
				slot := 5
				harness.input.PlanRequest.Rows[0].RevPiIOSlot = &slot
				harness.input.PlanRequest.Rows[0].RevPiIOAction = "on"
			},
			reason: "raw_physical_io_actions_forbidden",
		},
		{
			name: "non server-owned transport",
			mutate: func(harness *gatewayHarness) {
				harness.config.Features.AllowDeterministicTestTransport = false
			},
			reason: "gateway_infrastructure_ready",
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			harness := newGatewayHarness(t)
			test.mutate(harness)
			gateway := NewGateway(harness.config)
			review, err := gateway.Review(context.Background(), harness.input)
			if !errors.Is(err, ErrReviewDenied) ||
				review.Ready ||
				review.Token != nil ||
				!containsString(review.Reasons, test.reason) {
				t.Fatalf("adversarial review accepted: reasons=%+v err=%v", review.Reasons, err)
			}
			if got := harness.log.Entries(); len(got) != 0 {
				t.Fatalf("denied review touched authority/transport: %+v", got)
			}
		})
	}
}

func TestGatewayReviewTokenIsOneUseExactAndExpiring(t *testing.T) {
	t.Run("exact start request replay is idempotent", func(t *testing.T) {
		harness := newGatewayHarness(t)
		gateway := NewGateway(harness.config)
		review, err := gateway.Review(context.Background(), harness.input)
		if err != nil {
			t.Fatal(err)
		}
		request := StartRequest{
			RequestID:   "start-once",
			ReviewToken: review.Token.TokenID,
			Review:      harness.input,
		}
		session, err := gateway.Start(context.Background(), request)
		if err != nil {
			t.Fatal(err)
		}
		beforeReplay := append([]string(nil), harness.log.Entries()...)
		replayed, err := gateway.Start(context.Background(), request)
		if err != nil {
			t.Fatalf("exact replay: %v", err)
		}
		if !reflect.DeepEqual(replayed, session) {
			t.Fatalf("replay session changed:\n got: %+v\nwant: %+v", replayed, session)
		}
		if got := harness.log.Entries(); !reflect.DeepEqual(got, beforeReplay) {
			t.Fatalf("replay emitted operations: before=%+v after=%+v", beforeReplay, got)
		}
		differentRequest := request
		differentRequest.RequestID = "start-twice"
		if _, err := gateway.Start(
			context.Background(),
			differentRequest,
		); !errors.Is(err, ErrReviewToken) {
			t.Fatalf("consumed token with new request id error = %v", err)
		}
		tamperedReplay := request
		tamperedReplay.Review.Identity.PlanRevision++
		if _, err := gateway.Start(
			context.Background(),
			tamperedReplay,
		); !errors.Is(err, ErrIdempotencyReplay) {
			t.Fatalf("request id collision error = %v", err)
		}
		if got := harness.log.Entries(); !reflect.DeepEqual(got, beforeReplay) {
			t.Fatalf("rejected replay emitted operations: before=%+v after=%+v", beforeReplay, got)
		}
		_ = gateway.Stop(context.Background(), StopRequest{
			RequestID: "cleanup",
			SessionID: session.SessionID,
			Reason:    "test_cleanup",
		})
	})

	t.Run("tampered exact identity", func(t *testing.T) {
		harness := newGatewayHarness(t)
		gateway := NewGateway(harness.config)
		review, err := gateway.Review(context.Background(), harness.input)
		if err != nil {
			t.Fatal(err)
		}
		tampered := harness.input
		tampered.Identity.PlanRevision++
		if _, err := gateway.Start(context.Background(), StartRequest{
			RequestID:   "tampered",
			ReviewToken: review.Token.TokenID,
			Review:      tampered,
		}); !errors.Is(err, ErrReviewDenied) {
			t.Fatalf("tampered identity error = %v", err)
		}
		if got := harness.log.Entries(); len(got) != 0 {
			t.Fatalf("tampered token touched transport: %+v", got)
		}
	})

	t.Run("expired", func(t *testing.T) {
		harness := newGatewayHarness(t)
		harness.config.Bootstrap.ReviewTTL = time.Second
		gateway := NewGateway(harness.config)
		review, err := gateway.Review(context.Background(), harness.input)
		if err != nil {
			t.Fatal(err)
		}
		harness.clock.Advance(1500 * time.Millisecond)
		if _, err := gateway.Start(context.Background(), StartRequest{
			RequestID:   "expired",
			ReviewToken: review.Token.TokenID,
			Review:      harness.input,
		}); !errors.Is(err, ErrReviewToken) {
			t.Fatalf("expired token error = %v", err)
		}
		if got := harness.log.Entries(); len(got) != 0 {
			t.Fatalf("expired token touched transport: %+v", got)
		}
	})
}

func TestGatewayReceiptMismatchTriggersEveryProtectiveActionAndNeverPlays(t *testing.T) {
	harness := newGatewayHarness(t)
	badIdentity := harness.input.Identity
	badIdentity.PlanRevision++
	harness.transport.OverrideReceiptIdentity(ActionPreload, badIdentity)
	gateway := NewGateway(harness.config)
	review, err := gateway.Review(context.Background(), harness.input)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := gateway.Start(context.Background(), StartRequest{
		RequestID:   "bad-receipt",
		ReviewToken: review.Token.TokenID,
		Review:      harness.input,
	}); !errors.Is(err, ErrTransportReceipt) {
		t.Fatalf("receipt mismatch error = %v", err)
	}
	want := []string{
		"lease_acquire",
		"fence_validate",
		"preload",
		"stop",
		"disarm",
		"weld_off",
		"lease_release",
	}
	if got := harness.log.Entries(); !reflect.DeepEqual(got, want) {
		t.Fatalf("fallback order = %+v, want %+v", got, want)
	}
}

func TestGatewayAuthorityFailuresCannotReachPreloadOrPlay(t *testing.T) {
	tests := []struct {
		name      string
		operation string
		want      []string
	}{
		{
			name:      "acquire outcome lost",
			operation: "lease_acquire",
			want: []string{
				"lease_acquire",
				"stop",
				"disarm",
				"weld_off",
				"lease_cancel",
			},
		},
		{
			name:      "application fence rejected",
			operation: "fence_validate",
			want: []string{
				"lease_acquire",
				"fence_validate",
				"stop",
				"disarm",
				"weld_off",
				"lease_release",
			},
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			harness := newGatewayHarness(t)
			harness.authority.Fail(test.operation, errors.New("injected authority failure"))
			gateway := NewGateway(harness.config)
			review, err := gateway.Review(context.Background(), harness.input)
			if err != nil {
				t.Fatal(err)
			}
			if _, err := gateway.Start(context.Background(), StartRequest{
				RequestID:   "authority-failure",
				ReviewToken: review.Token.TokenID,
				Review:      harness.input,
			}); err == nil {
				t.Fatal("authority failure was hidden")
			}
			if got := harness.log.Entries(); !reflect.DeepEqual(got, test.want) {
				t.Fatalf("authority failure order = %+v, want %+v", got, test.want)
			}
		})
	}
}

func TestGatewayProtectiveTailContinuesAcrossFailures(t *testing.T) {
	harness := newGatewayHarness(t)
	gateway := NewGateway(harness.config)
	review, err := gateway.Review(context.Background(), harness.input)
	if err != nil {
		t.Fatal(err)
	}
	session, err := gateway.Start(context.Background(), StartRequest{
		RequestID:   "start-protective-failure",
		ReviewToken: review.Token.TokenID,
		Review:      harness.input,
	})
	if err != nil {
		t.Fatal(err)
	}
	harness.transport.Fail(ActionStop, errors.New("stop unavailable"))
	harness.transport.Fail(ActionDisarm, errors.New("disarm unavailable"))
	harness.transport.Fail(ActionWeldOff, errors.New("weld off unavailable"))
	if err := gateway.Stop(context.Background(), StopRequest{
		RequestID: "stop-failures",
		SessionID: session.SessionID,
		Reason:    "fault",
	}); err == nil {
		t.Fatal("protective failures were hidden")
	}
	got := harness.log.Entries()
	wantTail := []string{"stop", "disarm", "weld_off", "lease_release"}
	if !reflect.DeepEqual(got[len(got)-len(wantTail):], wantTail) {
		t.Fatalf("protective failure tail = %+v, want %+v", got, wantTail)
	}
}

func TestGatewaySupervisorProtectsAndReleasesOnActiveReadBackFailure(
	t *testing.T,
) {
	harness := newGatewayHarness(t)
	observer := &rejectingActiveObserver{}
	harness.config.Bootstrap.LeaseTTL = 500 * time.Millisecond
	harness.config.Features.SessionMonitoringEnabled = true
	harness.config.Observer = observer
	gateway := NewGateway(harness.config)
	review, err := gateway.Review(context.Background(), harness.input)
	if err != nil {
		t.Fatal(err)
	}
	session, err := gateway.Start(context.Background(), StartRequest{
		RequestID:   "start-supervised",
		ReviewToken: review.Token.TokenID,
		Review:      harness.input,
	})
	if err != nil {
		t.Fatal(err)
	}
	deadline := time.Now().Add(3 * time.Second)
	for {
		if _, active := gateway.Session(session.SessionID); !active {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("supervisor did not close a session with rejected live proof")
		}
		time.Sleep(20 * time.Millisecond)
	}
	if observer.Calls() == 0 {
		t.Fatal("supervisor did not consult the active read-back observer")
	}
	got := harness.log.Entries()
	wantTail := []string{"stop", "disarm", "weld_off", "lease_release"}
	if len(got) < len(wantTail) ||
		!reflect.DeepEqual(got[len(got)-len(wantTail):], wantTail) {
		t.Fatalf("supervisor protective tail = %+v, want suffix %+v", got, wantTail)
	}
	closeContext, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	if err := gateway.Close(closeContext); err != nil {
		t.Fatalf("close supervised gateway: %v", err)
	}
}

func newGatewayHarness(t *testing.T) *gatewayHarness {
	t.Helper()
	now := time.Date(2026, 7, 30, 8, 0, 0, 0, time.UTC)
	clock := NewDeterministicClock(now)
	log := &DeterministicOperationLog{}
	transport := NewDeterministicTestTransport(clock, log)
	authority := NewDeterministicTestAuthority(clock, log)
	audit := &DeterministicTestAuditSink{}
	store := NewDeterministicTestIdempotencyStore()
	revision, err := connected.ManifestRevision([]byte(`{"name":"bench-relay","revision":1}`))
	if err != nil {
		t.Fatal(err)
	}
	identity := ExecutionIdentity{
		ProjectID:        "project-1",
		ProjectRevision:  11,
		ProgramID:        "program-7",
		ProgramRevision:  7,
		PlanID:           "plan-42",
		PlanRevision:     42,
		ProgramHash:      "fnv1a64:1234",
		ProgramDigest:    testSHA256Digest("a"),
		TrajectoryDigest: testSHA256Digest("b"),
		RobotModel:       planning.RobotModelRosie1400V3,
		Target: TargetRevisionIdentity{
			ProfileID:        "bench-relay",
			ManifestRef:      "rt-core/config/cells/bench-relay.json",
			CellID:           "bench-relay",
			ManifestRevision: revision,
		},
	}
	allowlist, err := NewStaticManifestAllowlist([]ManifestAllowance{{
		Target:     identity.Target,
		RobotModel: identity.RobotModel,
	}})
	if err != nil {
		t.Fatal(err)
	}
	targetIdentity := identity.Target.ConnectedIdentity()
	fact := func(value bool) connected.BooleanFact {
		return connected.BooleanFact{Known: true, Value: value}
	}
	safety := func(state string) connected.SafetyEvidence {
		return connected.SafetyEvidence{
			Available:  true,
			Clear:      true,
			State:      state,
			PointCount: 0,
		}
	}
	input := ExecutionReviewInput{
		Identity: identity,
		Discovery: targets.ConnectedTargetDiscoveryDescriptor{
			Schema:         targets.ConnectedTargetDiscoverySchema,
			ProfileID:      identity.Target.ProfileID,
			ManifestRef:    identity.Target.ManifestRef,
			ManifestSchema: "robot_v4_deployment_manifest_v1",
			CellID:         identity.Target.CellID,
			Services: []targets.ConnectedTargetServiceDescriptor{{
				Role:           MotionServerService,
				StatusStream:   true,
				PlanStream:     true,
				ReadOnly:       true,
				ServerMediated: true,
			}},
			Target: targets.TargetDescriptor{
				Schema:      targets.TargetDescriptorSchema,
				ProfileID:   identity.Target.ProfileID,
				ManifestRef: identity.Target.ManifestRef,
				CellID:      identity.Target.CellID,
				Kind:        targets.TargetRealCell,
			},
		},
		PlanRequest: planning.NormalizedPlanRequest{
			Schema:          planning.NormalizedPlanRequestSchema,
			RequestID:       "plan-request-1",
			ProjectID:       identity.ProjectID,
			ProjectRevision: identity.ProjectRevision,
			ProgramID:       identity.ProgramID,
			ProgramRevision: identity.ProgramRevision,
			RobotModel:      identity.RobotModel,
			Target: planning.TargetIdentity{
				ProfileID:   identity.Target.ProfileID,
				ManifestRef: identity.Target.ManifestRef,
			},
			SubmittedAt: now.Add(-2 * time.Second),
			Program: []byte(`{
				"schema_version":1,
				"program_id":"program-7",
				"program_revision":7,
				"base_program_revision":0,
				"program_digest":"` + testSHA256Digest("a") + `",
				"robot_model":"rosie_1400_v3",
				"robotic-cell-type":"rosie_1400_v3",
				"execution_mode":"production",
				"settings":{"revision":1},
				"defaults":{"default_tool_frame":"tool0","default_work_frame":"world"},
				"rows":[
					{"row_id":"row-1","revision":1,"enabled":true,"row_type":"motion","move_type":"joint_ptp","motion_phase":"start","tool_frame":"tool0","work_frame":"world","target_tcp_m":[0,0,0,0,0,0],"target_joints_rad":[0,0,0,0,0,0,0,0,0],"tcp_velocity_m_s":0.1,"joint_velocity_rad_s":0.2},
					{"row_id":"row-2","revision":1,"enabled":true,"row_type":"motion","move_type":"joint_ptp","motion_phase":"finish","tool_frame":"tool0","work_frame":"world","target_tcp_m":[0,0,0,0,0,0],"target_joints_rad":[0.1,0,0,0,0,0,0,0,0],"tcp_velocity_m_s":0.1,"joint_velocity_rad_s":0.2}
				]
			}`),
			Rows: []planning.PlanRow{
				{RowID: "row-1", Revision: 1, Enabled: true, RowType: "motion"},
				{RowID: "row-2", Revision: 1, Enabled: true, RowType: "motion"},
			},
		},
		Evidence: planning.NormalizedPlanEvidence{
			Schema:    planning.NormalizedPlanEvidenceSchema,
			RequestID: "plan-request-1",
			Identity: planning.PlanIdentity{
				ProjectID:       identity.ProjectID,
				ProjectRevision: identity.ProjectRevision,
				ProgramID:       identity.ProgramID,
				ProgramRevision: identity.ProgramRevision,
				PlanID:          identity.PlanID,
				PlanRevision:    identity.PlanRevision,
				ProgramHash:     identity.ProgramHash,
				RobotModel:      identity.RobotModel,
				Target: planning.TargetIdentity{
					ProfileID:   identity.Target.ProfileID,
					ManifestRef: identity.Target.ManifestRef,
				},
			},
			GeneratedAt:     now.Add(-time.Second),
			PlannerOwner:    "motion-server",
			PlannerBackend:  "tesseract",
			State:           planning.PlanServerValidated,
			Accepted:        true,
			ValidationState: "valid",
			SampleCount:     2,
			Collision: planning.CollisionEvidence{
				State: planning.CheckClear,
			},
			Singularity: planning.SingularityEvidence{
				State: planning.CheckClear,
			},
			Limits: planning.LimitEvidence{
				State: planning.CheckClear,
			},
			Runtime: planning.RuntimeEvidence{
				State:    planning.CheckClear,
				Playable: true,
			},
			CorrelationReasons: []string{},
		},
		Status: connected.StatusObservation{
			Schema:             connected.StatusFrameSchema,
			Target:             targetIdentity,
			SourcePeer:         "bench-relay",
			SourceSubject:      "private-status-binding",
			Sequence:           10,
			ReceivedAt:         now.Add(-100 * time.Millisecond),
			SourceUpdatedAt:    now.Add(-100 * time.Millisecond),
			FreshUntil:         now.Add(2 * time.Second),
			Connected:          true,
			IdentityCorrelated: true,
			ServiceName:        "robot-v4-cartesiand",
			ServiceState:       "running",
			Plan: connected.RuntimePlanFacts{
				Preloaded:              fact(false),
				LoadedProgramAvailable: fact(false),
				LoadedProgramPlayable:  fact(false),
				ServerValidated:        fact(false),
				PlayAuthorized:         fact(false),
				PlaybackActive:         fact(false),
				LoadedProgramRevision:  0,
				PlanRevision:           0,
				PlanSessionID:          "",
			},
			Output: connected.RuntimeOutputFacts{
				ArmRequested:     fact(false),
				CurrentZeroHold:  fact(true),
				StaleZeroHold:    fact(false),
				ActualServoArmed: fact(false),
				EtherCATEnabled:  fact(true),
				MotionOutput:     fact(true),
				WeldOutput:       fact(false),
			},
		},
		RuntimePlan: connected.PlanObservation{
			Schema:        connected.PlanFrameSchema,
			Target:        targetIdentity,
			SourcePeer:    "bench-relay",
			SourceSubject: "private-plan-binding",
			Sequence:      20,
			ReceivedAt:    now.Add(-100 * time.Millisecond),
			FreshUntil:    now.Add(2 * time.Second),
			Identity: connected.PlanIdentity{
				PlanID:          identity.PlanID,
				PlanRevision:    identity.PlanRevision,
				ProgramID:       identity.ProgramID,
				ProgramRevision: identity.ProgramRevision,
				ProgramHash:     identity.ProgramHash,
				RobotModel:      string(identity.RobotModel),
			},
			PlannerOwner:       "motion-server",
			PlannerBackend:     "tesseract",
			ValidationState:    "valid",
			Accepted:           fact(true),
			PlayEnabled:        fact(true),
			RuntimePreload:     fact(true),
			SampleCount:        2,
			IdentityCorrelated: true,
			Safety: connected.PlanSafetyFacts{
				Collision:   safety("tesseract_discrete_contact_checked"),
				Singularity: safety("jacobian_svd_checked"),
				Limits:      safety("clear"),
				Runtime:     safety("clear"),
			},
		},
	}
	acceptedPlan, err := planning.MintRobotV4AcceptedPlan(
		input.PlanRequest,
		planning.NormalizedPlanResult{
			Schema:   planning.NormalizedPlanResultSchema,
			Evidence: input.Evidence,
			Trajectory: []planning.TrajectorySample{
				{
					RowIndex:      0,
					WaypointIndex: 0,
					TimeS:         0,
					MoveType:      "joint_ptp",
					MotionPhase:   "start",
				},
				{
					RowIndex:      1,
					WaypointIndex: 1,
					TimeS:         1,
					MoveType:      "joint_ptp",
					MotionPhase:   "finish",
				},
			},
		},
		planning.AcceptedPlanAssets{
			PlannerBuildDigest:   testSHA256Digest("1"),
			TrajectoryMethod:     "tesseract_dense_trajectory",
			ManifestRevision:     revision,
			CellConfigRevision:   "cell-config-1",
			CalibrationDigest:    testSHA256Digest("2"),
			URDFDigest:           testSHA256Digest("3"),
			MeshDigest:           testSHA256Digest("4"),
			CollisionWorldDigest: testSHA256Digest("5"),
			SettingsRevision:     1,
		},
		planning.AcceptedPlanPhysicalTarget,
		now.Add(-time.Second),
	)
	if err != nil {
		t.Fatal(err)
	}
	identity.TrajectoryDigest = acceptedPlan.TrajectoryDigest
	input.Identity = identity
	input.AcceptedPlan = acceptedPlan
	evidence := NewDeterministicTestEvidenceVerifier(input)
	config := GatewayConfig{
		Bootstrap: ControllerBootstrap{
			Controller: targets.ControllerIdentity{
				ControllerID:     "offline-programming:server-1",
				ControllerBootID: "offline-programming-boot-1",
			},
			CommandIDPrefix:        "opg",
			BootstrappedAt:         now.Add(-time.Minute),
			ReviewTTL:              5 * time.Second,
			LeaseTTL:               1500 * time.Millisecond,
			MaxPlanEvidenceAge:     5 * time.Minute,
			StatusFreshnessCeiling: 3 * time.Second,
		},
		Features: FeatureGates{
			GatewayEnabled:                  true,
			ConnectedExecutionEnabled:       true,
			ServerControllerAuthority:       true,
			PlanPreloadEnabled:              true,
			ControlledStartArmEnabled:       true,
			PlanPlayEnabled:                 true,
			ProtectiveTailEnabled:           true,
			AuditRequired:                   true,
			AllowDeterministicTestTransport: true,
		},
		Allowlist:   allowlist,
		Evidence:    evidence,
		Transport:   transport,
		Authority:   authority,
		Audit:       audit,
		Idempotency: store,
		Clock:       clock,
	}
	return &gatewayHarness{
		clock:     clock,
		log:       log,
		transport: transport,
		authority: authority,
		evidence:  evidence,
		audit:     audit,
		store:     store,
		config:    config,
		input:     input,
	}
}

func containsString(values []string, wanted string) bool {
	for _, value := range values {
		if value == wanted {
			return true
		}
	}
	return false
}

func testSHA256Digest(char string) string {
	value := ""
	for len(value) < 64 {
		value += char
	}
	return "sha256:" + value[:64]
}
