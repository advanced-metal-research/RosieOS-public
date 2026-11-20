package targetplanning

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

type fixedNATSResolver struct {
	url string
	ref string
}

func (resolver *fixedNATSResolver) ResolveNATS(
	_ context.Context,
	ref string,
) (string, error) {
	resolver.ref = ref
	return resolver.url, nil
}

type fixturePlanningBroker struct {
	t       *testing.T
	target  testTargets
	request planning.NormalizedPlanRequest
	now     time.Time
	command []byte
}

func (broker *fixturePlanningBroker) RequestPlan(
	_ context.Context,
	natsURL string,
	commandSubject string,
	planSubject string,
	command []byte,
) ([]byte, error) {
	broker.t.Helper()
	if natsURL != "nats://127.0.0.1:4222" ||
		commandSubject != broker.target.target.Binding.CommandSubject ||
		planSubject != broker.target.target.Binding.MotionServerPlanSubject {
		broker.t.Fatalf(
			"private binding mismatch: %q %q %q",
			natsURL,
			commandSubject,
			planSubject,
		)
	}
	broker.command = append([]byte(nil), command...)
	var envelope struct {
		Schema       string          `json:"schema"`
		Command      string          `json:"command"`
		RequestID    string          `json:"request_id"`
		PlanRevision uint64          `json:"plan_revision"`
		ProgramID    string          `json:"program_id"`
		PlanRequest  json.RawMessage `json:"plan_request"`
	}
	if err := json.Unmarshal(command, &envelope); err != nil {
		broker.t.Fatal(err)
	}
	if envelope.Schema != robotV4CommandSchema ||
		envelope.Command != "plan_request" ||
		envelope.RequestID != broker.request.RequestID ||
		envelope.PlanRevision == 0 ||
		envelope.ProgramID != broker.request.ProgramID ||
		string(envelope.PlanRequest) != string(broker.request.Program) {
		broker.t.Fatalf("authored program envelope changed: %+v", envelope)
	}
	plan, err := (&testAdapter{now: broker.now}).Plan(
		context.Background(),
		broker.target.target,
		broker.request,
		MutationIdentity{},
	)
	if err != nil {
		broker.t.Fatal(err)
	}
	plan.PlanRevision = envelope.PlanRevision
	plan.PlanID = "target-issued-" + strings.TrimPrefix(
		plan.TrajectoryDigest,
		"sha256:",
	)[:24]
	raw, err := json.Marshal(map[string]any{
		"plan_request_revision": envelope.PlanRevision,
		"planner_status":        "planned",
		"accepted_plan":         plan,
	})
	if err != nil {
		broker.t.Fatal(err)
	}
	return raw, nil
}

func TestNATSAdapterSendsOnlyAuthoredProgramAndAcceptsTargetArtifact(
	t *testing.T,
) {
	now := time.Date(2026, 7, 30, 12, 0, 0, 0, time.UTC)
	target := targetFixture()
	target.Binding.MotionServerNATSServiceRef =
		"daemon-v1://bench-relay/motion-server/robot-v4-nats"
	target.Binding.CommandSubject =
		"robot/v4/robot.bench-relay.command"
	target.Binding.MotionServerPlanSubject =
		"robot/v4/motion-server.bench-relay.plan"
	request := requestFixture(t, target)
	resolver := &fixedNATSResolver{url: "nats://127.0.0.1:4222"}
	broker := &fixturePlanningBroker{
		t:       t,
		target:  testTargets{target: target},
		request: request,
		now:     now,
	}
	adapter, err := NewNATSAdapter(NATSAdapterOptions{
		Resolver: resolver,
		Broker:   broker,
		Timeout:  time.Second,
		SenderID: "offline-programming:test",
	})
	if err != nil {
		t.Fatal(err)
	}
	plan, err := adapter.Plan(
		context.Background(),
		target,
		request,
		MutationIdentity{
			CorrelationID:  "correlation-nats-0001",
			IdempotencyKey: "idempotency-nats-0001",
		},
	)
	if err != nil {
		t.Fatal(err)
	}
	if plan.AuthorityClassification != planning.AcceptedPlanPhysicalTarget ||
		plan.RequestID != request.RequestID ||
		resolver.ref != target.Binding.MotionServerNATSServiceRef ||
		len(broker.command) == 0 ||
		strings.Contains(string(broker.command), "trajectory") ||
		strings.Contains(string(broker.command), "accepted_plan") {
		t.Fatalf("unsafe target planning exchange: plan=%+v", plan)
	}
}

func TestDaemonNATSResolverRequiresExactReadyService(t *testing.T) {
	state := httptest.NewServer(http.HandlerFunc(
		func(writer http.ResponseWriter, request *http.Request) {
			if request.URL.Path != "/api/state" {
				t.Fatalf("state path=%q", request.URL.Path)
			}
			writer.Header().Set("Content-Type", "application/json")
			_, _ = writer.Write([]byte(`{
			  "nodes":[{
			    "peer":"bench-relay",
			    "role":"motion-server",
			    "node":"robot-v4-motion-server",
			    "advertised_addresses":["192.168.1.62"],
			    "service_endpoints":[{
			      "name":"robot-v4-nats",
			      "protocol":"nats",
			      "url":"daemon-v1://bench-relay/motion-server/robot-v4-nats",
			      "listen":"0.0.0.0:4222",
			      "status":"running",
			      "advertised_by":"bench-relay"
			    }]
			  }]
			}`))
		},
	))
	defer state.Close()
	resolver, err := NewDaemonNATSResolver(DaemonNATSResolverOptions{
		StateURL: state.URL,
		Client:   state.Client(),
	})
	if err != nil {
		t.Fatal(err)
	}
	resolved, err := resolver.ResolveNATS(
		context.Background(),
		"daemon-v1://bench-relay/motion-server/robot-v4-nats",
	)
	if err != nil {
		t.Fatal(err)
	}
	if resolved != "nats://192.168.1.62:4222" {
		t.Fatalf("resolved=%q", resolved)
	}
	if _, err := resolver.ResolveNATS(
		context.Background(),
		"daemon-v1://other-relay/motion-server/robot-v4-nats",
	); err == nil {
		t.Fatal("resolver crossed the selected peer identity")
	}
}
