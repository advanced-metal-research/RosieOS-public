package targetplanning

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"net/url"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/targets"
	"github.com/nats-io/nats.go"
)

const (
	robotV4CommandSchema = "robot.v4.robot-command.v1"
	maxNATSPlanPayload   = 900 * 1024
)

type NATSURLResolver interface {
	ResolveNATS(context.Context, string) (string, error)
}

// PlanningBroker is the private cold-path exchange used by the adapter. Its
// concrete NATS implementation subscribes before publishing, so a fast Motion
// Server cannot publish the accepted plan before the response listener exists.
type PlanningBroker interface {
	RequestPlan(
		context.Context,
		string,
		string,
		string,
		[]byte,
	) ([]byte, error)
}

type NATSAdapterOptions struct {
	Resolver NATSURLResolver
	Broker   PlanningBroker
	Timeout  time.Duration
	SenderID string
}

type NATSAdapter struct {
	resolver NATSURLResolver
	broker   PlanningBroker
	timeout  time.Duration
	senderID string
}

func NewNATSAdapter(options NATSAdapterOptions) (*NATSAdapter, error) {
	if options.Resolver == nil {
		return nil, errors.New("target planning: NATS service resolver is required")
	}
	timeout := options.Timeout
	if timeout == 0 {
		timeout = 90 * time.Second
	}
	if timeout < time.Second || timeout > 2*time.Minute {
		return nil, errors.New("target planning: NATS plan timeout is invalid")
	}
	senderID := strings.TrimSpace(options.SenderID)
	if senderID == "" {
		senderID = "offline-programming:planning"
	}
	if !safeWireIdentity.MatchString(senderID) {
		return nil, errors.New("target planning: NATS sender identity is invalid")
	}
	broker := options.Broker
	if broker == nil {
		broker = &natsPlanningBroker{}
	}
	return &NATSAdapter{
		resolver: options.Resolver,
		broker:   broker,
		timeout:  timeout,
		senderID: senderID,
	}, nil
}

func (adapter *NATSAdapter) Available(
	target targets.ConnectedManifestTarget,
) bool {
	return adapter != nil &&
		adapter.resolver != nil &&
		adapter.broker != nil &&
		strings.HasPrefix(
			target.Binding.MotionServerNATSServiceRef,
			"daemon-v1://",
		) &&
		validNATSSubject(target.Binding.CommandSubject) &&
		validNATSSubject(target.Binding.MotionServerPlanSubject)
}

func (adapter *NATSAdapter) Plan(
	ctx context.Context,
	target targets.ConnectedManifestTarget,
	request planning.NormalizedPlanRequest,
	mutation MutationIdentity,
) (planning.RobotV4AcceptedPlan, error) {
	if !adapter.Available(target) {
		return planning.RobotV4AcceptedPlan{}, ErrUnavailable
	}
	if err := request.Validate(); err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: %v",
			ErrInvalidRequest,
			err,
		)
	}
	if len(request.Program) > maxNATSPlanPayload {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: shared program exceeds the bounded NATS planning payload",
			ErrInvalidRequest,
		)
	}
	planRevision := targetPlanRevision(
		target.Discovery.ProfileID,
		request,
		mutation,
	)
	commandID := targetPlanCommandID(
		target.Discovery.ProfileID,
		request.RequestID,
		mutation.IdempotencyKey,
	)
	envelope, err := marshalTargetPlanRequest(
		target,
		request,
		adapter.senderID,
		commandID,
		planRevision,
	)
	if err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: encode target request: %v",
			ErrInvalidRequest,
			err,
		)
	}
	natsURL, err := adapter.resolver.ResolveNATS(
		ctx,
		target.Binding.MotionServerNATSServiceRef,
	)
	if err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: %v",
			ErrUnavailable,
			err,
		)
	}
	planContext, cancel := context.WithTimeout(ctx, adapter.timeout)
	defer cancel()
	raw, err := adapter.broker.RequestPlan(
		planContext,
		natsURL,
		target.Binding.CommandSubject,
		target.Binding.MotionServerPlanSubject,
		envelope,
	)
	if err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: %v",
			ErrUnavailable,
			err,
		)
	}
	var report struct {
		PlanRequestRevision uint64                        `json:"plan_request_revision"`
		PlannerStatus       string                        `json:"planner_status"`
		Error               string                        `json:"error"`
		AcceptedPlan        *planning.RobotV4AcceptedPlan `json:"accepted_plan"`
	}
	if err := json.Unmarshal(raw, &report); err != nil {
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: target plan report is invalid",
			ErrRejected,
		)
	}
	if report.PlanRequestRevision != planRevision ||
		report.PlannerStatus != "planned" ||
		report.AcceptedPlan == nil {
		reason := strings.TrimSpace(report.Error)
		if reason == "" {
			reason = "target did not issue an accepted physical plan"
		}
		return planning.RobotV4AcceptedPlan{}, fmt.Errorf(
			"%w: %s",
			ErrRejected,
			reason,
		)
	}
	if report.AcceptedPlan.RequestID != request.RequestID ||
		report.AcceptedPlan.PlanRevision != planRevision {
		return planning.RobotV4AcceptedPlan{}, ErrIdentityMismatch
	}
	return *report.AcceptedPlan, nil
}

func marshalTargetPlanRequest(
	target targets.ConnectedManifestTarget,
	request planning.NormalizedPlanRequest,
	senderID string,
	commandID string,
	planRevision uint64,
) ([]byte, error) {
	// Marshal the command metadata first, then splice in the already-validated
	// shared program. encoding/json compacts RawMessage values; constructing
	// the final field this way preserves the exact catalog bytes that the
	// browser and SteamDeck edit and that the Motion Server must plan.
	prefix, err := json.Marshal(struct {
		Schema       string `json:"schema"`
		Command      string `json:"command"`
		CommandID    string `json:"command_id"`
		RequestID    string `json:"request_id"`
		SenderID     string `json:"sender_id"`
		Robot        string `json:"robot"`
		PlanRevision uint64 `json:"plan_revision"`
		ProgramID    string `json:"program_id"`
	}{
		Schema:       robotV4CommandSchema,
		Command:      "plan_request",
		CommandID:    commandID,
		RequestID:    request.RequestID,
		SenderID:     senderID,
		Robot:        target.Discovery.CellID,
		PlanRevision: planRevision,
		ProgramID:    request.ProgramID,
	})
	if err != nil || len(prefix) == 0 || prefix[len(prefix)-1] != '}' {
		return nil, errors.New("encode target request metadata")
	}
	envelope := make(
		[]byte,
		0,
		len(prefix)+len(request.Program)+18,
	)
	envelope = append(envelope, prefix[:len(prefix)-1]...)
	envelope = append(envelope, `,"plan_request":`...)
	envelope = append(envelope, request.Program...)
	envelope = append(envelope, '}')
	return envelope, nil
}

func targetPlanRevision(
	profileID string,
	request planning.NormalizedPlanRequest,
	mutation MutationIdentity,
) uint64 {
	hash := sha256.New()
	_, _ = hash.Write([]byte(profileID))
	_, _ = hash.Write([]byte{0})
	_, _ = hash.Write([]byte(request.RequestID))
	_, _ = hash.Write([]byte{0})
	_, _ = hash.Write([]byte(mutation.IdempotencyKey))
	_, _ = hash.Write([]byte{0})
	_, _ = hash.Write(request.Program)
	sum := hash.Sum(nil)
	revision := binary.BigEndian.Uint64(sum[:8]) & ((1 << 63) - 1)
	if revision == 0 {
		return 1
	}
	return revision
}

func targetPlanCommandID(
	profileID string,
	requestID string,
	idempotencyKey string,
) string {
	sum := sha256.Sum256([]byte(
		profileID + "\x00" + requestID + "\x00" + idempotencyKey,
	))
	return "op-plan-" + hex.EncodeToString(sum[:16])
}

func validNATSSubject(value string) bool {
	value = strings.TrimSpace(value)
	return value != "" &&
		len(value) <= 256 &&
		!strings.ContainsAny(value, " \t\r\n*>")
}

type natsPlanningBroker struct{}

func (*natsPlanningBroker) RequestPlan(
	ctx context.Context,
	natsURL string,
	commandSubject string,
	planSubject string,
	command []byte,
) ([]byte, error) {
	if !validConcreteNATSURL(natsURL) ||
		!validNATSSubject(commandSubject) ||
		!validNATSSubject(planSubject) {
		return nil, errors.New("resolved NATS target is invalid")
	}
	connection, err := nats.Connect(
		natsURL,
		nats.Name("offline-programming-v1-target-planning"),
		nats.Timeout(3*time.Second),
		nats.MaxReconnects(0),
		nats.NoEcho(),
	)
	if err != nil {
		return nil, err
	}
	defer connection.Close()
	results, err := connection.SubscribeSync(planSubject)
	if err != nil {
		return nil, err
	}
	if err := connection.FlushWithContext(ctx); err != nil {
		return nil, err
	}
	reply, err := connection.RequestWithContext(
		ctx,
		commandSubject,
		command,
	)
	if err != nil {
		return nil, err
	}
	var acknowledgement struct {
		Schema    string `json:"schema"`
		Component string `json:"component"`
		Command   string `json:"command"`
		CommandID string `json:"command_id"`
		Accepted  bool   `json:"accepted"`
	}
	var sent struct {
		CommandID    string `json:"command_id"`
		PlanRevision uint64 `json:"plan_revision"`
	}
	if json.Unmarshal(command, &sent) != nil ||
		json.Unmarshal(reply.Data, &acknowledgement) != nil ||
		acknowledgement.Schema != "robot.v4.command-reply.v1" ||
		acknowledgement.Component != "motion-server" ||
		acknowledgement.Command != "plan_request" ||
		acknowledgement.CommandID != sent.CommandID ||
		!acknowledgement.Accepted {
		return nil, errors.New("Motion Server did not accept the plan request")
	}
	for {
		message, err := results.NextMsgWithContext(ctx)
		if err != nil {
			return nil, err
		}
		var correlation struct {
			PlanRequestRevision uint64 `json:"plan_request_revision"`
		}
		if json.Unmarshal(message.Data, &correlation) == nil &&
			correlation.PlanRequestRevision == sent.PlanRevision {
			return append([]byte(nil), message.Data...), nil
		}
	}
}

func validConcreteNATSURL(raw string) bool {
	raw = strings.TrimSpace(raw)
	parsed, err := url.Parse(raw)
	return err == nil &&
		parsed.Scheme == "nats" &&
		parsed.User == nil &&
		!strings.ContainsAny(raw, " \t\r\n") &&
		validDaemonHost(parsed.Hostname()) != ""
}
