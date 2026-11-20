//go:build linux

package denseexec

import (
	"context"
	"crypto/sha256"
	"crypto/tls"
	"crypto/x509"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"net/url"
	"os"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

// Unverified application budgets, in absolute units; receipts establish success.
const (
	rtOperationBudget = 30 * time.Second
	rtPollInterval    = 50 * time.Millisecond
)

// RTCoreConfig pins deployment identity before acquisition. Pair binding is an
// operator input, never evidence of the v4 pair service's current revision.
type RTCoreConfig struct {
	CartesianModel string `json:"-"`
	// MoveJerk bounds the jerk of one-off joint and Cartesian moves on every
	// axis, in rad/s^3: the selected cell's compiled machine jog_jerk, read
	// from the repository compile whose digest selection requires live
	// Describe to match. Never decoded from a catalogue or config file; zero
	// leaves moves acceleration-limited only.
	MoveJerk float64 `json:"-"`
	// UnattendedLoopAllowed makes every repeating Run program unattended: it
	// continues without any client's heartbeat (neither its starter's nor
	// the server-wide deadman) and any client may finish it. The cell
	// manifest's cell_deploy.olp.unattended_loop_allowed, an owner decision,
	// false when absent (two Rosie 1430 cells set it, 2026-10-11 23:32 PDT).
	UnattendedLoopAllowed bool            `json:"-"`
	Socket                string          `json:"socket"`
	Address               string          `json:"address"`
	CAFile                string          `json:"ca_file"`
	CertificateFile       string          `json:"certificate_file"`
	KeyFile               string          `json:"key_file"`
	ExpectedBackend       string          `json:"expected_backend,omitempty"`
	Binding               control.Binding `json:"binding"`
	DeploymentSHA256      string          `json:"deployment_sha256"`
	CapabilitiesDigest    string          `json:"capabilities_digest"`
	AxisMask              uint32          `json:"axis_mask"`
}

// RTCoreClient uses generated SDK types and independent urgent/renewal lanes.
// Tests inject this boundary, never substitute a native IPC writer.
type RTCoreClient interface {
	Describe(context.Context) (control.Description, error)
	Status(context.Context) (control.Status, error)
	EventsContext(context.Context, uint64) (control.EventBatch, error)
	Acquire(context.Context, string, control.Binding) (control.Grant, error)
	StartRenewal(context.Context, time.Duration) (<-chan control.Renewal, error)
	StopRenewal()
	Grant() control.Grant
	Command(context.Context, *control.Request) (control.Response, error)
	PrepareProgram(context.Context, []byte) (control.Program, error)
	Stop(context.Context) (control.Grant, error)
	Release(context.Context) (control.Grant, error)
	Close() error
}

type RTCoreManifest struct {
	Description  control.Description `json:"description"`
	AddressHost  string              `json:"address_host,omitempty"`
	PairRevision *uint64             `json:"pair_revision"`
	FactoryHash  *string             `json:"factory_hash"`
	URDF         *string             `json:"urdf"`
	HomeProfile  *string             `json:"home_profile"`
}

type RTCoreObservation struct {
	Inhibited      bool                `json:"inhibited"`
	StopUncertain  bool                `json:"stop_uncertain"`
	RecoveryAction string              `json:"recovery_action,omitempty"`
	Description    control.Description `json:"description"`
	RefusalKind    string              `json:"refusal_kind"`
	Refusal        string              `json:"refusal"`
	RefusalAfterNS uint64              `json:"refusal_after_ns"`
	Status         *control.Status     `json:"status"`
	Events         []control.Event     `json:"events"`
	Reason         string              `json:"reason"`
	Note           string              `json:"note,omitempty"`
}

// RTCoreController owns one public grant and one prepared identity. Stop cancels
// in-flight work before revocation; a late receipt cannot resurrect the session.
type RTCoreController struct {
	manualToken          string
	manualDeadline       time.Time
	manualSequence       uint64
	manualNativeSequence uint64
	manualFence          control.Fence
	c                    RTCoreClient
	cfg                  RTCoreConfig
	source               *blobSource
	mu                   sync.Mutex
	opMu                 operationMutex
	stopMu               operationMutex
	stopping             int
	opCancel             context.CancelFunc
	epoch                uint64
	closed               bool
	held                 bool
	inhibited            bool
	stopUncertain        bool
	armResetting         bool                   // the Reset in progress (alone or Arm's first step) may observe the reset_clears fault it is about to reset
	resetReference       *homeReferenceRecovery // only explicit Clear fault may clear an independently recovered reference latch
	homeRecovery         *homeReferenceRecovery // only the captured reference fault may remain visible during its reset
	homeResetPending     bool                   // a correlated Reset reply owns expected retirement of this grant
	acquiring            bool
	description          control.Description
	descriptionValid     bool
	describeMu           operationMutex
	observation          *control.Status
	events               []control.Event
	cursor               uint64
	adapter, daemon      string
	// One (sequence, time) anchor into the telemetry ring per daemon
	// incarnation, so a window read starts near the head (rt_core_telemetry.go).
	telemetryAnchor *telemetryAnchor
	// The description's resources, by digest, read once (rt_core_robot.go).
	resources        robotResources
	program          *control.Program
	session          *SessionInfo
	trajectory       TrajectorySummary
	lastHeartbeat    time.Time
	reason           string
	statusNote       string
	feedbackKind     string
	feedbackCode     string
	feedbackAfterNS  uint64
	feedbackReceipt  uint64
	feedbackPending  bool
	cancel           context.CancelFunc
	done             chan struct{}
	renewals         <-chan control.Renewal
	renewOwnerCancel context.CancelFunc
	nativeSeen       time.Time
	startSequence    uint64
	fence            control.Fence
	roundTrip        time.Duration
	effectiveLease   time.Duration
	jog              *jointJogHold
	cartesianResolve CartesianResolver
	cartesianService CartesianResolverService
	jogRevision      uint64
	jogReason        string
	// jogStop is the last retired hold's stop; jogQueued a press waiting for
	// it (jog_queue.go).
	jogStop   *jogStopRecord
	jogQueued *queuedJogBegin
	// lastJogEnd is the last classified hold end (jog_end.go), jogEndSeq its
	// sequence; jogEndBy the client whose Stop, Disarm or release is retiring
	// the hold right now, consumed by that retirement.
	lastJogEnd *JogEnd
	jogEndSeq  uint64
	jogEndBy   Client
	// jogEndCause is the client's end_cause of the End retiring the hold now.
	jogEndCause string
	// The live path-hold readout kept after the hold ended (path_live.go),
	// and the robot geometry it reads, from the server's robot store.
	lastPathLive   *PathLive
	lastPathLiveAt time.Time
	geometrySource func(string) (RobotGeometry, error)
	geometry       RobotGeometry
	geometryKnown  bool
	jogOpen        func(context.Context, uint32, uint64) (jointJogLane, error)
	tracePrepared  *preparedTrace
	pathOpen       func(context.Context, uint32, float64, float64, time.Duration) (pathJogLane, error)
	// restSinceNS is the Motion.TimeNS of the observation in which motion
	// last became done: per-axis targets published after it are the held
	// command at rest (the path hold's start rule).
	restSinceNS uint64
	move        *jointMoveRun
	moveState   JointMoveState
	// loop is the running repeat of a Run program press, lastLoop the
	// account of the last one that ended (run_loop.go).
	loop     *loopRun
	lastLoop *LoopStatus
	// ioArmed is the grant io_arm last armed the cell's outputs on, zero
	// once disarmed or fenced; ioDisarming while settleOutputs disarms it
	// (rt_core_outputs.go).
	ioArmed     control.Fence
	ioDisarming bool
}

func NewRTCore(cfg RTCoreConfig, source Config, client RTCoreClient) (*RTCoreController, error) {
	if cfg.ExpectedBackend != "" && cfg.ExpectedBackend != "ethercat" && cfg.ExpectedBackend != "simulation" {
		return nil, errors.New("rt_core expected_backend must be ethercat or simulation")
	}
	if cfg.Binding.PairID == "" || cfg.Binding.Revision == 0 ||
		!digest(cfg.Binding.MachineSHA256) || !digest(cfg.DeploymentSHA256) ||
		!digest(cfg.CapabilitiesDigest) || cfg.AxisMask == 0 || cfg.AxisMask > 511 || source.ControllerID == "" {
		return nil, errors.New("rt_core requires explicit pair binding, machine/deployment/contract digests, axis mask and controller identity")
	}
	if !finiteJoint(cfg.MoveJerk) || cfg.MoveJerk < 0 {
		return nil, errors.New("rt_core move jerk must be finite and non-negative")
	}
	if client == nil {
		var err error
		client, err = dialRTCore(cfg)
		if err != nil {
			return nil, err
		}
	}
	// Bound blob reads independently of the native motion grant.
	if source.HTTPClient == nil {
		source.HTTPClient = &http.Client{Timeout: rtOperationBudget}
	}
	r := &RTCoreController{c: client, cfg: cfg, source: &blobSource{cfg: source}, done: make(chan struct{})}
	ctx, cancel := context.WithCancel(context.Background())
	r.cancel = cancel
	go r.watch(ctx)
	return r, nil
}

func digest(s string) bool {
	if len(s) != 64 {
		return false
	}
	for _, c := range s {
		if !(c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
			return false
		}
	}
	return true
}

func (r *RTCoreController) Available() bool { return r != nil }

// ControllerID is the name this controller acquires rt-control grants with.
func (r *RTCoreController) ControllerID() string { return r.source.cfg.ControllerID }

func (r *RTCoreController) describe(ctx context.Context, cached ...bool) (control.Description, error) {
	ctx, cancel := context.WithTimeout(ctx, r.callBudget())
	defer cancel()
	if err := r.describeMu.LockContext(ctx); err != nil {
		return control.Description{}, err
	}
	defer r.describeMu.Unlock()
	if len(cached) != 0 {
		r.mu.Lock()
		d, valid := r.description, r.descriptionValid
		r.mu.Unlock()
		if valid {
			return d, nil
		}
	}
	started := time.Now()
	d, err := r.c.Describe(ctx)
	if err != nil {
		return d, err
	}
	measured := time.Since(started)
	if client, ok := r.c.(interface{ Timing() control.LeaseTiming }); ok {
		measured = client.Timing().RoundTrip
	}
	r.mu.Lock()
	r.roundTrip = measured
	r.description = d
	r.mu.Unlock()
	if d.ContractVersion != 2 || d.CapabilitiesDigest != r.cfg.CapabilitiesDigest ||
		(r.cfg.ExpectedBackend != "" && d.Backend != r.cfg.ExpectedBackend) ||
		(r.cfg.Binding.ConfigurationSHA256 != "" && d.ConfigurationSHA256 != r.cfg.Binding.ConfigurationSHA256) ||
		d.MachineSHA256 != r.cfg.Binding.MachineSHA256 || d.DeploymentSHA256 != r.cfg.DeploymentSHA256 ||
		len(d.Axes) == 0 || len(d.Axes) > 9 || r.cfg.AxisMask>>len(d.Axes) != 0 {
		return d, rtRefused("rt_core_identity_mismatch")
	}
	for _, name := range []string{"acquire", "renew", "release", "enable", "arm", "home", "stop", "prepare_program", "start_program", "status", "subscribe_events"} {
		found := false
		for _, capability := range d.Capabilities {
			if capability.Name == name && capability.State == control.CapabilityStateImplemented {
				found = true
			}
		}
		if !found {
			return d, rtRefused("capability_unimplemented: %s", name)
		}
	}
	r.mu.Lock()
	r.descriptionValid = true
	r.mu.Unlock()
	return d, nil
}

// timingLocked derives observation and receipt budgets from the effective
// grant and measured Describe RTT, preventing LAN thresholds from expiring
// healthy remote sessions. Call with mu held; before acquisition use cell policy.
func (r *RTCoreController) timingLocked() control.LeaseTiming {
	lease := r.effectiveLease
	if lease == 0 {
		lease = time.Duration(r.description.MaxGrantLeaseNS)
	}
	// Unverified compatibility lease for older fixtures/descriptions: 500 ms.
	if lease == 0 {
		lease = 500 * time.Millisecond
	}
	return control.TimingForLease(lease, r.roundTrip)
}

// callBudget uses the existing operation watchdog until Describe has measured
// transport, so a cold TLS connection cannot inherit a fictitious LAN RTT.
func (r *RTCoreController) callBudget() time.Duration {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.roundTrip == 0 {
		return rtOperationBudget
	}
	return r.timingLocked().CallBudget
}

// UnattendedLoopAllowed is this cell's manifest flag (RTCoreConfig); the
// server admits a pendant's program arm only when it is true.
func (r *RTCoreController) UnattendedLoopAllowed() bool { return r.cfg.UnattendedLoopAllowed }

func (r *RTCoreController) Capabilities(ctx context.Context) Capabilities {
	ctx, cancel := context.WithTimeout(ctx, r.callBudget())
	defer cancel()
	d, err := r.describe(ctx)
	caps := Capabilities{Backend: "rt_core", Available: true, Cell: r.cfg.Binding.PairID, ControllerID: r.source.cfg.ControllerID, RecoverZeroJogSpeeds: true,
		UnattendedLoopAllowed: r.cfg.UnattendedLoopAllowed}
	if err != nil {
		caps.Reason = err.Error()
		return caps
	}
	caps.DaemonReachable = true
	caps.RTCore = &RTCoreManifest{Description: d, AddressHost: rtCoreAddressHost(r.cfg.Address)}
	return caps
}

// Expose only the remote host for display; credentials and local paths remain
// server-side even when a caller supplies a malformed address.
func rtCoreAddressHost(address string) string {
	u, err := url.Parse(address)
	if err != nil || u.Scheme != "https" || u.Hostname() == "" || u.User != nil ||
		(u.Path != "" && u.Path != "/") || u.RawQuery != "" || u.Fragment != "" {
		return ""
	}
	return u.Host
}

// operation pins cancellation across all requests, preventing a Stop between
// validation and transmission from sending motion with replacement authority.
func (r *RTCoreController) operation(ctx context.Context, fn func(context.Context) error) error {
	return r.operationKind(ctx, "unknown", fn)
}

// pressEpochKey carries the Stop epoch of the press an operation serves (Arm,
// whose settle steps read the robot before it queues).
type pressEpochKey struct{}

// operationKind bounds one operation; `extra` lengthens the bound for work
// whose size is known up front (a program upload earns its bytes' time).
func (r *RTCoreController) operationKind(ctx context.Context, kind string, fn func(context.Context) error, extra ...time.Duration) error {
	r.mu.Lock()
	if r.move != nil {
		r.mu.Unlock()
		return jointJogFailure(rtRefused("jog_mode_conflict"))
	}
	epoch := r.epoch
	if entry, ok := ctx.Value(pressEpochKey{}).(uint64); ok {
		epoch = entry
	}
	r.mu.Unlock()
	budget := rtOperationBudget
	for _, more := range extra {
		budget += more
	}
	var armDeadline time.Time
	if kind == "arm" {
		// Leave one eighth of the overall window for fail-closed retirement.
		overall := r.armBudget()
		armDeadline = time.Now().Add(overall)
		budget = overall * (rtArmCallBudgets - 1) / rtArmCallBudgets
	}
	ctx, cancel := context.WithTimeout(ctx, budget)
	defer cancel()
	failure := func(err error) error {
		if kind == "arm" && errors.Is(err, context.DeadlineExceeded) {
			return httpError(409, "rt_core_arm_timeout", "%v", err)
		}
		return rtFailure(err)
	}
	if err := r.opMu.LockContext(ctx); err != nil {
		return failure(err)
	}
	defer r.opMu.Unlock()
	if err := r.stopMu.LockContext(ctx); err != nil {
		return failure(err)
	}
	r.mu.Lock()
	if r.closed {
		r.mu.Unlock()
		r.stopMu.Unlock()
		return httpError(409, "rt_core_closed", "controller closed")
	}
	if r.jog != nil {
		r.mu.Unlock()
		r.stopMu.Unlock()
		return jointJogFailure(rtRefused("jog_mode_conflict"))
	}
	if r.epoch != epoch || r.stopping != 0 {
		r.mu.Unlock()
		r.stopMu.Unlock()
		return rtFailure(&control.Rejected{Reason: "control_session_stale: operation queued before Stop"})
	}
	if r.inhibited || r.stopUncertain {
		reason := r.reason
		action := r.recoveryActionLocked()
		r.mu.Unlock()
		r.stopMu.Unlock()
		return httpError(409, "rt_core_inhibited", "%s Cause: %s", action, reason)
	}
	if err := ctx.Err(); err != nil {
		r.mu.Unlock()
		r.stopMu.Unlock()
		return rtFailure(err)
	}
	r.opCancel = cancel
	r.mu.Unlock()
	r.stopMu.Unlock()
	err := fn(ctx)
	if kind == "arm" && err != nil {
		var rejected *control.Rejected
		if errors.As(err, &rejected) && strings.SplitN(rejected.Reason, ":", 2)[0] == "busy" {
			err = httpError(409, "rt_core_arm_refused", "%v", err)
		}
	}
	if kind == "arm" && errors.Is(ctx.Err(), context.DeadlineExceeded) {
		err = httpError(409, "rt_core_arm_timeout", "%v", ctx.Err())
	}
	cancel()
	r.mu.Lock()
	r.opCancel = nil
	if r.epoch != epoch {
		reason := r.reason
		r.mu.Unlock()
		// The Stop owner already recorded the cause. A cancelled bulk reply must
		// not replace its typed refusal with context.Canceled or issue another Stop.
		return rtFailure(&control.Rejected{Reason: reason})
	}
	r.mu.Unlock()
	if err != nil {
		if kind == "io_test" {
			// A refused diagnostic output sample never Stops a concurrent motion
			// owner. Native test expiry and the existing Stop fence remain authoritative.
			return rtFailure(err)
		}
		if kind == "reset" {
			return r.resetFailure(err)
		}
		return r.failKind(err, kind, armDeadline)
	}
	r.mu.Lock()
	if operatorAcceptanceClears(r.feedbackKind, kind) {
		r.feedbackCode = ""
	}
	r.feedbackPending = true
	r.feedbackReceipt++
	r.mu.Unlock()
	return nil
}

func (r *RTCoreController) acquire(ctx context.Context) error {
	r.mu.Lock()
	held, epoch := r.held, r.epoch
	r.mu.Unlock()
	if held {
		return nil
	}
	d, err := r.describe(ctx)
	if err != nil {
		return err
	}
	var s control.Status
	var b control.EventBatch
	if client, ok := r.c.(interface {
		PrepareAcquisition(context.Context) (control.AcquisitionSnapshot, error)
	}); ok {
		var snapshot control.AcquisitionSnapshot
		snapshot, err = client.PrepareAcquisition(ctx)
		s, b = snapshot.Status, snapshot.Events
	} else {
		s, err = r.c.Status(ctx)
		if err == nil {
			b, err = r.c.EventsContext(ctx, 0)
		}
	}
	if err != nil {
		return err
	}
	if s.AdapterIncarnation == "" || s.DaemonIncarnation == "" || s.TimeNS == 0 {
		return control.ErrProtocol
	}
	// Discard old event history only at a fresh, explicit acquisition.
	if b.AdapterIncarnation != s.AdapterIncarnation {
		return control.ErrProtocol
	}
	r.mu.Lock()
	if r.epoch != epoch || ctx.Err() != nil {
		r.mu.Unlock()
		return rtRefused("control_session_stale: acquisition cancelled before request")
	}
	r.acquiring = true
	r.mu.Unlock()
	g, err := r.c.Acquire(ctx, r.source.cfg.ControllerID, r.cfg.Binding)
	r.mu.Lock()
	// A transport or protocol failure may hide an issued grant. Only a typed
	// refusal or pre-request closed client establishes that no grant was issued.
	var rejected *control.Rejected
	validGrant := g.Session != "" && g.Generation != 0 && !g.Stopping && g.LeaseMS > 0
	r.acquiring = err == nil && !validGrant || err != nil && !errors.As(err, &rejected) && !errors.Is(err, control.ErrClosed)
	if err == nil {
		r.fence = control.Fence{Session: g.Session, Generation: g.Generation}
	}
	r.mu.Unlock()
	if err != nil {
		return err
	}
	if !validGrant {
		return control.ErrProtocol
	}
	// Serialize keeper installation with Stop so cancelled acquisition cannot
	// leave renewal running after Stop has already joined its producer.
	if err := r.stopMu.LockContext(ctx); err != nil {
		return err
	}
	defer r.stopMu.Unlock()
	r.mu.Lock()
	current := r.epoch == epoch
	r.mu.Unlock()
	if err := ctx.Err(); err != nil {
		return err
	}
	if !current {
		return &control.Rejected{Reason: "control_session_stale"}
	}
	// Startup observes operation cancellation even while it owns stopMu. Once
	// installed, the keeper belongs to Stop rather than the completed Arm call.
	renewCtx, cancelRenewal := context.WithCancel(context.Background())
	abortStartup := context.AfterFunc(ctx, cancelRenewal)
	updates, err := r.c.StartRenewal(renewCtx, time.Duration(g.LeaseMS)*time.Millisecond/3)
	startupLive := abortStartup()
	if err != nil {
		cancelRenewal()
		r.c.StopRenewal()
		return err
	}
	r.mu.Lock()
	if !startupLive || ctx.Err() != nil || r.epoch != epoch {
		r.mu.Unlock()
		cancelRenewal()
		r.c.StopRenewal()
		return errors.Join(ctx.Err(), rtRefused("control_session_stale: keeper installation cancelled"))
	}
	defer r.mu.Unlock()
	r.held, r.reason, r.description = true, "", d
	r.fence = control.Fence{Session: g.Session, Generation: g.Generation}
	r.effectiveLease = time.Duration(g.LeaseMS) * time.Millisecond
	r.adapter, r.daemon, r.cursor = s.AdapterIncarnation, s.DaemonIncarnation, b.LatestSequence
	r.observation, r.renewals = &s, updates
	r.renewOwnerCancel = cancelRenewal
	r.lastHeartbeat = time.Now()
	r.nativeSeen = time.Now()
	if r.session == nil || r.session.State == "stopped" {
		r.session = &SessionInfo{ID: newSessionID(), Kind: "authority", State: "idle", HeartbeatDeadlineMS: DefaultHeartbeatDeadline.Milliseconds()}
	}
	return nil
}

type rtCommandFailure struct {
	error
	kind string
}

func (e *rtCommandFailure) Unwrap() error { return e.error }

func (r *RTCoreController) command(ctx context.Context, op string, identity control.Identity) error {
	return r.commandMasked(ctx, op, identity, r.cfg.AxisMask)
}

// commandMasked sends one native command for a subset of the configured axes
// (Home of one joint); every other command addresses the whole configured set.
func (r *RTCoreController) commandMasked(ctx context.Context, op string, identity control.Identity, mask uint32) error {
	r.mu.Lock()
	fence := r.fence
	epoch := r.epoch
	r.mu.Unlock()
	kind := op
	if op == "start_program" {
		kind = "program_start"
	}
	if op == "reset_fault" {
		kind = "reset"
	}
	request := &control.Request{Operation: op, Fence: fence, AxisMask: mask, Identity: identity}
	var response control.Response
	var err error
	if op == "home" {
		// Home replies only after native commissioning and durable anchor capture.
		// The SDK already budgets this synchronous operation separately. Keep the
		// enclosing operation's cancellation/deadline instead of clipping it to a
		// network round trip (or replaying a still-running Home after that timeout).
		response, err = r.c.Command(ctx, request)
	} else {
		response, err = rtRequest(r, ctx, "rt_core_request_slow", func(call context.Context) (control.Response, error) { return r.c.Command(call, request) })
	}
	if err == nil && (response.Operation != op || response.Sequence == 0) {
		err = control.ErrProtocol
	}
	if err != nil {
		return &rtCommandFailure{error: err, kind: kind}
	}
	r.mu.Lock()
	if r.epoch == epoch {
		if operatorAcceptanceClears(r.feedbackKind, kind) {
			r.feedbackCode = ""
		}
		r.feedbackPending = true
		r.feedbackReceipt++
	}
	r.mu.Unlock()
	if err == nil && op == "start_program" {
		r.mu.Lock()
		r.startSequence = response.Sequence
		r.mu.Unlock()
	}
	return err
}

func (r *RTCoreController) Arm(ctx context.Context, armed bool) (*ArmResult, error) {
	if !armed {
		r.noteJogEndBy(ctx)
		stopped, err := r.stopAction("disarm")
		return &ArmResult{OK: err == nil, Accepted: err == nil, Note: stopped.Note, Status: r.statusSnapshot()}, err
	}
	// A Stop after the press refuses this Arm even though it read the robot
	// before queueing: every operation it runs is fenced by the press's epoch.
	// A press that arms as one of its steps (Run program) already carries its
	// own epoch, so a Stop earlier in that press refuses this Arm too.
	if _, inPress := ctx.Value(pressEpochKey{}).(uint64); !inPress {
		r.mu.Lock()
		ctx = context.WithValue(ctx, pressEpochKey{}, r.epoch)
		r.mu.Unlock()
	}
	steps, err := r.settleForArm(ctx)
	if err != nil {
		return &ArmResult{OK: false, Accepted: false, Steps: steps, Status: r.statusSnapshot()}, err
	}
	err = r.operationKind(ctx, "arm", func(ctx context.Context) error {
		if err := r.acquire(ctx); err != nil {
			return err
		}
		if err := r.command(ctx, "enable", control.Identity{}); err != nil {
			return err
		}
		if err := r.command(ctx, "arm", control.Identity{}); err != nil {
			return err
		}
		return r.observeUntil(ctx, func(s control.Status) bool { return s.Core.Armed == 1 && r.ready(s) })
	})
	return &ArmResult{OK: err == nil, Accepted: err == nil, Steps: steps, Status: r.statusSnapshot()}, err
}

// settleForArm is Arm's first part, the recovery press's own steps
// (nextArmStep, settleStep): from one fresh native status it confirms an
// unconfirmed Stop and resets latched faults whose recovery is a reset, each
// journaled under "arm" with the client, then lets Arm proceed. It refuses,
// before anything is sent, a fault that needs a new Home, a restart or an
// unknown recovery, and after a refused Reset names the drive alarm still
// active. It never homes and never widens anything. With no fresh status it
// leaves the decision to Arm's own admission, as before.
func (r *RTCoreController) settleForArm(ctx context.Context) ([]RecoverStep, error) {
	var steps []RecoverStep
	journal := stepJournal(ctx, "arm", &steps)
	refuse := func(d recoverDecision) ([]RecoverStep, error) {
		if d.Code == "stop_unconfirmed" {
			// Every client knows an unconfirmed Stop by Arm's own code and
			// recovery action.
			r.mu.Lock()
			action, reason := r.recoveryActionLocked(), r.reason
			r.mu.Unlock()
			d.Code, d.Detail = "rt_core_inhibited", d.Detail+". "+action+" Cause: "+reason
		}
		journal(RecoverStep{Step: d.Step, Outcome: "refused", Detail: d.Code + ": " + d.Detail})
		return steps, httpError(409, d.Code, "%s", d.Detail)
	}
	tried := map[string]bool{}
	// Each step runs at most once per Arm; the bound is the number of steps.
	for range 4 {
		facts, _, err := r.statusFacts(ctx)
		if err != nil {
			return steps, nil
		}
		facts.Tried = tried
		decision := nextArmStep(facts)
		tried[decision.Action] = true
		switch decision.Action {
		case "arm":
			return steps, nil
		case "refuse":
			return refuse(decision)
		case "confirm_stop", "reset":
			if refusal := r.settleStep(ctx, decision, facts, journal); refusal != nil {
				return refuse(*refusal)
			}
		}
	}
	return refuse(refuseRecover("start", "arm_incomplete", "Arm did not settle the Stop and faults; Arm again"))
}

// resetCommand sends the native reset. The adapter retires the control session
// on every submitted reset, accepted or refused (reset_linux.go), and the SDK
// forgets that fence on the native reply; the local hold goes with it, so the
// grant is never reused and a refusal's cleanup Stop is the acknowledged
// nothing-held no-op. Sent on the retired session that Stop is refused
// control_session_stale on every retry and latches stop_uncertain: on a Rosie 1430
// arm (2026-10-11 12:19) Arm's reset refused fault_persists (J5 drive alarm
// Er41.1) and every later Arm and Reset was refused rt_core_inhibited. An
// unsent or uncertain reset leaves the SDK grant, and with it recovery, alone.
func (r *RTCoreController) resetCommand(ctx context.Context, mask uint32) error {
	err := r.commandMasked(ctx, "reset_fault", control.Identity{}, mask)
	if err == nil || r.c.Grant().Session == "" {
		r.dropRetiredGrant()
	}
	return err
}

// dropRetiredGrant forgets a grant the adapter has already retired (a reset
// retires the session on acceptance and on rejection alike). Nothing is held
// natively any more, so only the local keeper is torn down; no Stop is sent
// and the app session stays idle for the Arm that continues.
func (r *RTCoreController) dropRetiredGrant() {
	r.mu.Lock()
	r.held, r.renewals = false, nil
	ownerCancel := r.renewOwnerCancel
	r.renewOwnerCancel = nil
	r.fence = control.Fence{}
	r.mu.Unlock()
	if ownerCancel != nil {
		ownerCancel()
	}
	r.c.StopRenewal()
}

// resettableFaults admits a native reset attempt, not a claim that it will
// succeed. Conditional recovery is decided by native fresh evidence; a refused
// reset cannot proceed to Enable. Re-Home and restart faults are excluded.
func resettableFaults(s control.Status) bool {
	if s.Core.Recovery == nil || len(s.Core.Recovery.Faults) == 0 {
		return false
	}
	for _, fault := range s.Core.Recovery.Faults {
		if fault.Recovery != "reset_clears" && fault.Recovery != "reset_after_condition_clears" {
			return false
		}
	}
	return true
}

// Home establishes native Home evidence; GoHome remains unavailable because
// travelling to a saved v4 pose is a different, unimplemented operation.
func (r *RTCoreController) Home(ctx context.Context) error {
	return r.HomeAxes(ctx, r.cfg.AxisMask)
}

// HomeAxes runs the native Home on the axes in mask, a nonempty subset of the
// configured axes, and waits until Home is valid on every one of them with the
// commissioning worker idle. The other joints keep their Home.
func (r *RTCoreController) HomeAxes(ctx context.Context, mask uint32) error {
	if mask == 0 || mask&^r.cfg.AxisMask != 0 {
		return httpError(400, "home_axes_invalid", "home axes must be a nonempty subset of the configured axes (mask %#x)", r.cfg.AxisMask)
	}
	return r.operationKind(ctx, "home", func(ctx context.Context) error {
		// Home cannot run through a latched native safety fault. Only the
		// reference fault on these requested axes has an explicit reset-then-
		// Home recovery path; native Reset still qualifies the encoder proof.
		initial, err := rtRequest(r, ctx, "rt_core_status_slow", r.c.Status)
		if err != nil {
			return err
		}
		r.mu.Lock()
		held := r.held
		r.mu.Unlock()
		eligibility := initial
		// A new adapter grant retires a previous owner's terminal summary.
		// Do not confuse that history with an error of an authority still held.
		if !held && historicalHomeExecution(initial.Execution) {
			eligibility.Execution.Error = ""
		}
		recovery := referenceRecoveryForHome(eligibility, mask)
		if initial.Core.SafetyFaultMask != 0 && recovery == nil {
			return rtRefused("rt_core_fault: %d", initial.Core.SafetyFaultMask)
		}
		r.mu.Lock()
		r.homeRecovery = recovery
		r.mu.Unlock()
		defer func() { r.mu.Lock(); r.homeRecovery = nil; r.homeResetPending = false; r.mu.Unlock() }()
		if err := r.acquire(ctx); err != nil {
			return err
		}
		if recovery != nil {
			// Check the new owner's actual observation before sending Reset.
			// A historical error was eligible only for acquisition, never here.
			if err := r.observe(ctx); err != nil {
				return err
			}
			r.mu.Lock()
			r.homeResetPending = true
			r.mu.Unlock()
			if err := r.resetCommand(ctx, mask); err != nil {
				return fmt.Errorf("Home recovery reset refused: %w", err)
			}
			r.mu.Lock()
			r.homeResetPending = false
			r.mu.Unlock()
			if err := r.observeUntil(ctx, func(s control.Status) bool { return s.Core.SafetyFaultMask == 0 }); err != nil {
				return err
			}
			r.mu.Lock()
			r.homeRecovery = nil
			r.mu.Unlock()
			if err := r.acquire(ctx); err != nil {
				return err
			}
		}
		if err := r.commandMasked(ctx, "home", control.Identity{}, mask); err != nil {
			return err
		}
		return r.observeUntil(ctx, func(s control.Status) bool {
			return s.Core.HomeValidMask&mask == mask && s.Core.CommissioningPhase == 0
		})
	})
}

// loadBytes is what one Load stages: the fetched bytes, the uploaded test
// program they belong to (nil for a planner plan), where they came from and
// the notes the fetch earned.
type loadBytes struct {
	blob        []byte
	testProgram *TestProgram
	source      string
	notes       []string
}

// fetchForLoad is Load's first half: the identity checked and the bytes
// fetched, before any operation. Run program fetches once and stages the
// same bytes after it has moved to the start (run_program.go).
func (r *RTCoreController) fetchForLoad(ctx context.Context, request LoadRequest) (loadBytes, error) {
	if err := validateIdentity(request.Identity); err != nil {
		return loadBytes{}, httpError(400, "dense_identity_invalid", "%v", err)
	}
	// The blob is fetched before the operation so its size can lengthen the
	// operation's bound: the upload to the cell earns its bytes' time.
	fetchCtx, fetchCancel := context.WithTimeout(ctx, rtOperationBudget)
	blob, testProgram, kept, err := r.source.loadBlob(fetchCtx, request)
	fetchCancel()
	if err != nil {
		return loadBytes{}, rtFailure(err)
	}
	fetched := loadBytes{blob: blob, testProgram: testProgram, source: "planner"}
	if testProgram != nil {
		fetched.source = TestProgramSource
	} else if kept != "" {
		fetched.source = "client"
		fetched.notes = append(fetched.notes, kept)
	}
	return fetched, nil
}

// headerIdentity is the six-field identity a trajectory header names.
func headerIdentity(h densejointtraj.Header) Identity {
	return Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, ManifestRevision: h.ManifestRevision, PlanRevision: h.PlanRevision}
}

// loadOutputs runs processOutputsGate (rt_core_outputs.go) against the cell's
// described outputs, before any authority is taken.
func (r *RTCoreController) loadOutputs(ctx context.Context, h densejointtraj.Header, request LoadRequest) ([]string, error) {
	var d control.Description
	if len(h.ProcessMarkers) != 0 && !request.DryRun {
		var err error
		if d, err = r.describe(ctx, true); err != nil {
			return nil, err
		}
	}
	return processOutputsGate(h, request, d)
}

func (r *RTCoreController) Load(ctx context.Context, request LoadRequest) (*LoadResult, error) {
	fetched, err := r.fetchForLoad(ctx, request)
	if err != nil {
		return nil, err
	}
	return r.loadFetched(ctx, request, fetched)
}

// loadFetched is Load's second half: every check against the live machine,
// the acquisition and the native preparation of bytes already fetched.
func (r *RTCoreController) loadFetched(ctx context.Context, request LoadRequest, fetched loadBytes) (*LoadResult, error) {
	var result *LoadResult
	blob, testProgram, blobSource := fetched.blob, fetched.testProgram, fetched.source
	notes := append([]string(nil), fetched.notes...)
	err := r.operationKind(ctx, "prepare_program", func(ctx context.Context) error {
		r.mu.Lock()
		active := r.session != nil && r.session.State == "playing"
		r.mu.Unlock()
		if active {
			return rtRefused("program_already_executing")
		}
		plan, err := densejointtraj.Decode(blob)
		if err != nil {
			return httpError(400, "dense_trajectory_invalid", "%v", err)
		}
		h := plan.Header
		if headerIdentity(h) != request.Identity {
			return rtRefused("dense_identity_mismatch")
		}
		// The robot the plan was made against is the robot this machine is,
		// asked before anything is acquired (docs/ROBOT-DESCRIPTION-CONTRACT.md).
		note, err := r.robotGate(ctx, h.Robot)
		if err != nil {
			return err
		}
		if note != "" {
			notes = append(notes, note)
		}
		outputs, err := r.loadOutputs(ctx, h, request)
		if err != nil {
			return err
		}
		executionBlob, executionPlan := blob, plan
		if request.DryRun {
			executionBlob, executionPlan, err = densejointtraj.WithoutProcessOutputs(blob)
			if err != nil {
				return httpError(400, "dense_trajectory_invalid", "%v", err)
			}
		}
		executionDigest := executionPlan.Header.TrajectoryDigest
		// Check the current native reference before acquiring authority or sending
		// program bytes. A stale saved anchor must lead the operator to Home,
		// rather than failing later inside preparation/lease renewal. Native
		// admission still checks again, so a later reference loss stays fenced.
		status, err := r.c.Status(ctx)
		if err != nil {
			return err
		}
		if status.Core.HomeValidMask&r.cfg.AxisMask != r.cfg.AxisMask {
			return rtRefused("home_required: establish the current joint position reference in Home before loading the program")
		}
		r.mu.Lock()
		description := r.description
		r.mu.Unlock()
		if err := validatePlanningCapture(plan, status, description); err != nil {
			return err
		}
		if testProgram != nil {
			if _, err := r.testProgramGate(ctx, plan, status); err != nil {
				return err
			}
		}
		if err := r.acquire(ctx); err != nil {
			return err
		}
		p, err := r.c.PrepareProgram(ctx, executionBlob)
		if err != nil {
			r.mu.Lock()
			description := r.description
			r.mu.Unlock()
			return enrichLimitFailure(err, plan, description)
		}
		// With header markers the adapter binds the whole uploaded envelope:
		// its source identity is the sha256 of these exact bytes, and it must
		// report that the program needs process I/O. Without, the source
		// identity is the trajectory digest and no process I/O is allowed.
		sourceDigest := executionDigest
		hasProcessMarkers := len(h.ProcessMarkers) != 0 && !request.DryRun
		if hasProcessMarkers {
			sum := sha256.Sum256(executionBlob)
			sourceDigest = "sha256:" + hex.EncodeToString(sum[:])
		}
		if p.Identity.PlanID != request.PlanID || p.Identity.ProgramID != request.ProgramID || p.Identity.ProgramDigest != request.ProgramDigest ||
			p.Identity.TrajectoryDigest != executionDigest || p.Identity.ManifestRevision != request.ManifestRevision || p.Identity.PlanRevision != request.PlanRevision ||
			p.Identity.SourceDigest != sourceDigest || !digest(strings.TrimPrefix(p.Identity.NormalisedDigest, "sha256:")) || p.AxisMask != r.cfg.AxisMask ||
			p.RequiresProcessIO != hasProcessMarkers {
			return rtRefused("program_identity_or_process_mismatch")
		}
		r.mu.Lock()
		defer r.mu.Unlock()
		if ctx.Err() != nil {
			return ctx.Err()
		}
		r.program, r.trajectory = &p, summarize(h)
		r.session = &SessionInfo{ID: newSessionID(), Kind: SessionKindTrajectory, PlanID: request.PlanID, TrajectoryDigest: request.TrajectoryDigest, State: "loaded", HeartbeatDeadlineMS: DefaultHeartbeatDeadline.Milliseconds()}
		r.session.DryRun = request.DryRun
		r.session.ProcessOutputs = outputs
		if testProgram != nil {
			r.session.Source, r.session.TestProgram = TestProgramSource, testProgram.label()
		}
		r.session.ExecutionTrajectoryDigest = executionDigest
		r.session.ExecutionNormalisedDigest = p.Identity.NormalisedDigest
		r.lastHeartbeat = time.Now()
		result = &LoadResult{OK: true, SessionID: r.session.ID, Trajectory: r.trajectory, Notes: notes, BlobSource: blobSource}
		return nil
	}, control.UploadAllowance(len(blob)))
	if result != nil {
		result.Status = r.statusSnapshot()
	}
	return result, err
}

func (r *RTCoreController) Play(ctx context.Context, sessionID string) (*StatusBody, error) {
	// Arm runs under the same operation lock, so a Play started disarmed
	// waited out the whole operation budget for an Arm that could not come and
	// failed with whatever went stale first (rt_core_status_stale after 7.6 s,
	// a simulated Rosie 1430 arm, 2026-10-11), stopping the loaded session on the way.
	// Refused here, outside the operation, the session stays loaded.
	r.mu.Lock()
	disarmed := r.observation != nil && r.observation.Core.Armed == 0 &&
		r.session != nil && r.session.ID == sessionID && r.session.State == "loaded"
	r.mu.Unlock()
	if disarmed {
		return r.statusSnapshot(), httpError(409, "play_requires_arm", "the robot is disarmed; Arm, then Play")
	}
	// An io_arm refusal the operator can fix leaves the session loaded and
	// the robot armed: it is answered after the operation, not through its
	// failure path, which would Stop and release.
	var outputsRefused *Error
	err := r.operationKind(ctx, "program_start", func(ctx context.Context) error {
		r.mu.Lock()
		if r.program == nil || r.session == nil || r.session.ID != sessionID || r.session.State != "loaded" || !r.held {
			r.mu.Unlock()
			return rtRefused("control_session_stale")
		}
		identity := r.program.Identity
		outputs := r.session.ProcessOutputs
		r.mu.Unlock()
		if err := r.observeUntil(ctx, func(s control.Status) bool { return s.Core.Armed == 1 && r.ready(s) }); err != nil {
			return err
		}
		if len(outputs) != 0 {
			if err := r.armOutputs(ctx); err != nil {
				r.mu.Lock()
				observation := r.observation
				r.mu.Unlock()
				if refusal := ioArmRefusal(err, observation, outputs); refusal != nil {
					outputsRefused = refusal
					return nil
				}
				return err
			}
		}
		if err := r.command(ctx, "start_program", identity); err != nil {
			var rejected *control.Rejected
			if len(outputs) != 0 && errors.As(err, &rejected) && strings.HasPrefix(rejected.Reason, "io_not_armed") {
				return httpError(409, "io_not_armed", "the cell disarmed its outputs between io_arm and Start (a fast input opened or the terminal stopped exchanging); %s did not start. Check the interlock and press Run again", strings.Join(outputs, ", "))
			}
			return err
		}
		r.mu.Lock()
		defer r.mu.Unlock()
		if ctx.Err() != nil {
			return ctx.Err()
		}
		r.session.State, r.lastHeartbeat = "playing", time.Now()
		return nil
	})
	if err == nil && outputsRefused != nil {
		return r.statusSnapshot(), outputsRefused
	}
	return r.statusSnapshot(), err
}

func (r *RTCoreController) Pause(ctx context.Context, _ string) (*StatusBody, error) {
	err := r.fail(rtRefused("capability_unimplemented: pause"))
	return r.statusSnapshot(), err
}

func (r *RTCoreController) GoHome(context.Context) (*GoHomeResult, error) {
	return nil, r.fail(rtRefused("capability_unimplemented: go_home; use native Home explicitly"))
}

func (r *RTCoreController) Heartbeat(id string) (*HeartbeatResult, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.session == nil || r.session.ID != id || r.session.State == "stopped" {
		return nil, httpError(409, "dense_session_mismatch", "unknown session")
	}
	age := time.Since(r.lastHeartbeat).Milliseconds()
	r.lastHeartbeat = time.Now()
	return &HeartbeatResult{OK: true, AgeMS: age, DeadlineMS: DefaultHeartbeatDeadline.Milliseconds()}, nil
}

func (r *RTCoreController) ready(s control.Status) bool {
	for i := 0; i < 9; i++ {
		if r.cfg.AxisMask&(1<<i) != 0 && (i >= len(s.Axes) || !s.Axes[i].LogicalValid || s.Axes[i].Readiness != "ready") {
			return false
		}
	}
	return true
}

func (r *RTCoreController) observeUntil(ctx context.Context, predicate func(control.Status) bool) error {
	ticker := time.NewTicker(rtPollInterval)
	defer ticker.Stop()
	for {
		if err := r.observe(ctx); err != nil {
			return err
		}
		r.mu.Lock()
		s := r.observation
		r.mu.Unlock()
		if s != nil && predicate(*s) {
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

func (r *RTCoreController) observe(ctx context.Context) error {
	r.mu.Lock()
	epoch, adapter, daemon, fence := r.epoch, r.adapter, r.daemon, r.fence
	playing := r.session != nil && r.session.State == "playing"
	read, compact := r.statusReaderLocked(true)
	r.mu.Unlock()
	s, err := rtRequest(r, ctx, "rt_core_status_slow", read)
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.epoch != epoch || r.fence != fence {
		return nil // a predecessor's response cannot affect replacement authority
	}
	if err != nil {
		return err
	}
	if s.TimeNS == 0 || s.Motion.TimeNS == 0 {
		return control.ErrProtocol
	}
	if adapter != "" && (s.AdapterIncarnation != adapter || s.DaemonIncarnation != daemon) {
		return rtRefused("rt_core_incarnation_changed")
	}
	if r.observation != nil && s.AdapterIncarnation == r.observation.AdapterIncarnation &&
		s.DaemonIncarnation == r.observation.DaemonIncarnation && s.TimeNS < r.observation.TimeNS {
		return nil // discard stale fault/error evidence before interpreting it
	}
	// A latched fault refuses every observation so a cached sample cannot
	// hide it. The one exception is the Reset that clears it (ResetFault,
	// alone or as Arm's first step): while it runs and every latched fault
	// permits a native reset attempt, the faulted sample is admitted so
	// reset_fault reaches the machine and the mask can be watched down to
	// zero. Without this the reset was unreachable: the first observation
	// refused with rt_core_fault and the fault stayed latched (simulator,
	// cycle_deadline, 2026-09-19).
	resetting := r.armResetting
	homeRecovery := r.homeRecovery
	if !(resetting && resetAdmits(s, r.resetReference)) {
		// A new execution failure is never hidden by reference recovery.
		if homeRecovery != nil && s.Execution.Error != "" {
			return rtRefused("%s", s.Execution.Error)
		}
		if s.Core.SafetyFaultMask != 0 && !homeRecovery.allows(s) {
			return rtRefused("rt_core_fault: %d", s.Core.SafetyFaultMask)
		}
		for axisIndex, axis := range s.Axes {
			if axis.Readiness == "faulted" && !homeRecovery.allowsAxis(s, axisIndex) {
				return rtRefused("rt_core_fault: axis faulted")
			}
		}
		if s.Execution.Error != "" {
			return rtRefused("%s", s.Execution.Error)
		}
	}
	// A move may start while a compact read is in flight. Its prepared/consumed
	// handle receipts must not be replaced by that history-free response, nor
	// may it rejuvenate the move's retained evidence. Fault and incarnation
	// checks above still apply; the move's own next read always requests handles.
	if !compact || r.move == nil {
		if r.observation == nil || r.observation.Motion.TimeNS != s.Motion.TimeNS || r.nativeSeen.IsZero() {
			r.nativeSeen = time.Now()
		} else if time.Since(r.nativeSeen) > r.timingLocked().StatusAge {
			return rtRefused("rt_core_status_stale")
		}
		r.setObservationLocked(&s)
	}
	if playing && r.session != nil && r.session.State == "playing" && r.program != nil {
		if s.Execution.Identity != r.program.Identity {
			return rtRefused("program_identity_mismatch")
		}
		if s.Execution.NativeSequence != r.startSequence {
			return rtRefused("execution_sequence_mismatch")
		}
		if !r.ready(s) {
			return rtRefused("rt_core_readiness_lost")
		}
		switch s.Execution.State {
		case "completed":
			r.session.State = "done"
		case "executing":
		default:
			return rtRefused("execution_interrupted: %s", s.Execution.State)
		}
	}
	return nil
}

func (r *RTCoreController) watch(ctx context.Context) {
	defer close(r.done)
	observed := make(chan struct{})
	go func() { defer close(observed); r.watchObservations(ctx) }()
	defer func() { <-observed }()
	ticker := time.NewTicker(rtPollInterval)
	defer ticker.Stop()
	for {
		r.mu.Lock()
		updates := r.renewals
		renewEpoch := r.epoch
		r.mu.Unlock()
		select {
		case <-ctx.Done():
			return
		case update, ok := <-updates:
			r.mu.Lock()
			current := r.held && renewEpoch == r.epoch && r.renewals == updates
			fence := r.fence
			// Reset retires its grant before the HTTP reply can arrive. Only
			// that typed retirement may wait for the explicit reset's receipt;
			// link/protocol failures still take the independent Stop path.
			deferredRetirement := current && ok && r.homeResetPending && retiredHomeResetGrant(update.Err)
			if deferredRetirement && r.renewals == updates {
				r.renewals = nil // the terminated keeper's closed channel must not spin
			}
			r.mu.Unlock()
			if deferredRetirement {
				continue
			}
			if !current {
				continue
			}
			if !ok || update.Err != nil || update.Grant.Stopping || update.Grant.Session != fence.Session || update.Grant.Generation != fence.Generation {
				err := update.Err
				if err == nil {
					err = rtRefused("control_renewal_lost")
				}
				_ = r.failObserved(&rtAuthorityFailure{err}, observedAuthority{epoch: renewEpoch, fence: fence, renewals: updates})
			}
		case <-ticker.C:
			r.mu.Lock()
			// Accepted Cartesian steps own their bounded execution; browser
			// heartbeat loss must not turn a page close into a motion Stop.
			// Neither may it stop an unattended loop (owner decision
			// 2026-10-11 23:32 PDT): STOP, E-stop and faults guard it.
			unattended := r.loop != nil && !r.loop.attended
			lost := r.held && (r.move == nil || r.move.cartesian == nil) && !unattended && time.Since(r.lastHeartbeat) > DefaultHeartbeatDeadline
			r.mu.Unlock()
			if lost {
				_ = r.fail(rtRefused("ui_heartbeat_lost"))
			}
		}
	}
}

// watchObservations cannot delay renewal-loss handling behind a slow status.
// The independent authority watcher continues consuming keeper receipts.
func (r *RTCoreController) watchObservations(ctx context.Context) {
	ticker := time.NewTicker(rtPollInterval)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			r.mu.Lock()
			held, epoch, cursor, fence, updates := r.held, r.epoch, r.cursor, r.fence, r.renewals
			r.mu.Unlock()
			if !held {
				continue
			}
			err := r.observe(ctx)
			if err == nil {
				r.observeJointJog()
				r.settleOutputs()
			}
			if err == nil {
				var batch control.EventBatch
				batch, err = rtRequest(r, ctx, "rt_core_status_slow", func(call context.Context) (control.EventBatch, error) { return r.c.EventsContext(call, cursor) })
				if err == nil {
					r.mu.Lock()
					if r.epoch == epoch && r.fence == fence && r.renewals == updates {
						if batch.AdapterIncarnation != r.adapter {
							err = rtRefused("rt_core_incarnation_changed")
						}
						for _, e := range batch.Events {
							expectedArmClear := r.armResetting && e.Type == "fault_cleared" && e.Fault != nil &&
								(e.Fault.RecoveryClass == "reset_clears" || e.Fault.RecoveryClass == "reset_after_condition_clears")
							if e.Type == "events_dropped" || e.Fault != nil && !r.homeRecovery.expectedClear(e) && !r.resetReference.expectedClear(e) && !expectedArmClear || e.Type == "grant_expired" || e.Type == "grant_released" {
								err = rtRefused("rt_core_event: %s", e.Type)
							}
						}
						r.cursor, r.events = batch.NextSequence, batch.Events
					}
					r.mu.Unlock()
				}
			}
			r.mu.Lock()
			current := epoch == r.epoch && r.fence == fence
			r.mu.Unlock()
			if err != nil && current {
				_ = r.failObserved(err, observedAuthority{epoch: epoch, fence: fence, renewals: updates})
			}
		}
	}
}

// Status observes without acquiring authority, including before Home and after
// a fault has retired the session. A cached healthy sample must not hide recovery.
func (r *RTCoreController) Status(ctx context.Context) *StatusBody {
	return r.status(ctx, false)
}

// ControlStatus obtains the pendant's current evidence without transporting
// retired handles. Active joint moves still need full handle receipts.
func (r *RTCoreController) ControlStatus(ctx context.Context) *StatusBody {
	return r.status(ctx, true)
}

// statusReaderLocked selects before native I/O. Only joint-move completion
// consumes handle history; ordinary jogging, Trace and playback use current
// execution/motion fields. Call with mu held and recheck compact admission on
// response because a joint move can begin while the read is outstanding.
func (r *RTCoreController) statusReaderLocked(currentOnly bool) (func(context.Context) (control.Status, error), bool) {
	if currentOnly && r.move == nil {
		if observer, ok := r.c.(resetStatusObserver); ok {
			return observer.StatusObservation, true
		}
	}
	return r.c.Status, false
}

func (r *RTCoreController) status(ctx context.Context, currentOnly bool) *StatusBody {
	r.mu.Lock()
	receipt := r.feedbackReceipt
	description, described := r.description, r.descriptionValid
	operating := r.opCancel != nil
	read, compact := r.statusReaderLocked(currentOnly)
	r.mu.Unlock()
	var descriptionErr error
	// Discovery is optional for observation. An Arm already discovering on a
	// silent socket must not keep Status off its independent observation lane.
	if !described && !operating {
		call, cancel := context.WithTimeout(ctx, r.callBudget())
		description, descriptionErr = r.describe(call, true)
		cancel()
	}
	observed, statusErr := rtRequest(r, ctx, "rt_core_status_slow", read)
	r.mu.Lock()
	defer r.mu.Unlock()
	var observation *control.Status
	reason := r.reason
	if err := errors.Join(descriptionErr, statusErr); err != nil {
		if reason == "" {
			reason = err.Error()
		}
	} else if observed.TimeNS == 0 || observed.Motion.TimeNS == 0 || observed.AdapterIncarnation == "" || observed.DaemonIncarnation == "" {
		if reason == "" {
			reason = "rt_core_status_unavailable"
		}
	} else {
		observation = &observed
		// Fresh picker polls also supply readiness evidence. Completion order and
		// unchanged native timestamps cannot make old feedback young again.
		if r.held && (!compact || r.move == nil) && observed.AdapterIncarnation == r.adapter && observed.DaemonIncarnation == r.daemon &&
			(r.observation == nil || observed.TimeNS >= r.observation.TimeNS) {
			if r.observation == nil || observed.Motion.TimeNS != r.observation.Motion.TimeNS {
				r.nativeSeen = time.Now()
			}
			r.setObservationLocked(&observed)
		}
		// Only a read issued after this receipt can establish its server fence.
		// A pre-command read may finish later without becoming newer evidence.
		if r.feedbackPending && receipt == r.feedbackReceipt {
			r.feedbackAfterNS = observed.TimeNS
			r.feedbackPending = false
		}
	}
	return r.statusBodyLocked(observation, description, reason)
}

// Command responses use their already observed result; extra observation I/O
// must never delay the urgent Stop response or reuse a cancelled command context.
func (r *RTCoreController) statusSnapshot() *StatusBody {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.statusBodyLocked(r.observation, r.description, r.reason)
}

func (r *RTCoreController) statusBodyLocked(observation *control.Status, description control.Description, reason string) *StatusBody {
	// A completed refusal is not an in-flight operation. Preserve the native
	// detail without leaving the UI's busy transition latched after cancellation.
	if r.opCancel == nil && strings.SplitN(reason, ":", 2)[0] == "busy" {
		reason = "rt_core_request_refused: " + reason
	}
	// Remote monotonic clocks are unrelated. A command receipt fences retained
	// observations until the next fresh server sample establishes its watermark.
	after := r.feedbackAfterNS
	if r.feedbackPending {
		if observation != nil {
			after = observation.TimeNS
		}
		for _, event := range r.events {
			if event.TimeNS > after {
				after = event.TimeNS
			}
		}
	}
	body := &StatusBody{Backend: "rt_core", Available: !r.closed && observation != nil, RTCore: &RTCoreObservation{Inhibited: r.inhibited, StopUncertain: r.stopUncertain, RecoveryAction: r.recoveryActionLocked(), Description: description, Refusal: r.feedbackCode, RefusalKind: r.feedbackKind, RefusalAfterNS: after, Status: observation, Events: r.events, Reason: reason}, DaemonError: reason}
	body.RTCore.Note = r.statusNote
	body.LastJogEnd = r.lastJogEndLocked()
	body.JogLive = r.jogLiveLocked()
	if r.descriptionValid {
		robot := robotCellOf(description)
		body.Robot = &robot
	}
	body.RoundTripNS = int64(r.roundTrip)
	body.LinkLeaseNS = int64(r.timingLocked().Lease)
	if r.session != nil {
		s := *r.session
		s.HeartbeatAgeMS = time.Since(r.lastHeartbeat).Milliseconds()
		s.Lease.Held = r.held
		body.Session = &s
	}
	if observation != nil {
		armed := observation.Core.Armed == 1 && r.held && reason == ""
		body.Armed = &armed
	}
	body.Loop = r.loopStatusLocked()
	return body
}

// recoveryActionLocked names the recovery request and restart condition so a
// refusal cannot strand an operator with an unexplained local latch. Hold mu.
func (r *RTCoreController) recoveryActionLocked() string {
	if !r.inhibited && !r.stopUncertain {
		return ""
	}
	state := "Stop acknowledged; Release unconfirmed. "
	if r.stopUncertain {
		state = "Stop outcome unconfirmed. "
	}
	return state + "Retry POST /dense-execution/stop. Successful Stop and Release clear recovery without a server restart. If control_session_stale persists, restart the offline-programming server after the adapter reports no active grant."
}

func (r *RTCoreController) fail(cause error) error {
	return r.failKind(cause, "unknown")
}

func (r *RTCoreController) failKind(cause error, kind string, deadline ...time.Time) error {
	return r.failKindObserved(cause, kind, nil, deadline...)
}

func (r *RTCoreController) failObserved(cause error, owner observedAuthority) error {
	return r.failKindObserved(cause, "unknown", &owner)
}

func (r *RTCoreController) failKindObserved(cause error, kind string, owner *observedAuthority, deadline ...time.Time) error {
	var command *rtCommandFailure
	if errors.As(cause, &command) {
		kind = command.kind
	}
	// An unsent request cannot make motion uncertain. The SDK already consumed
	// its one reopen attempt; preserve authority and let the operator retry.
	var authority *rtAuthorityFailure
	if errors.Is(cause, control.ErrConnectionUnsent) && !errors.As(cause, &authority) {
		r.mu.Lock()
		if owner != nil && !owner.current(r) {
			r.mu.Unlock()
			return nil
		}
		r.statusNote = "rt_core_connection_reopened"
		r.mu.Unlock()
		return rtFailure(cause)
	}
	r.mu.Lock()
	if owner != nil && !owner.current(r) {
		r.mu.Unlock()
		return nil
	}
	if r.held && slowRequest(cause) {
		if r.statusNote == "" {
			r.statusNote = "rt_core_request_slow"
		}
		r.mu.Unlock()
		return httpError(409, "rt_core_request_slow", "%v", cause)
	}
	r.mu.Unlock()
	failure := rtFailure(cause)
	_, err := r.stopObserved(cause.Error(), owner, deadline...)
	if errors.Is(err, errObservationSuperseded) {
		return nil
	}
	r.mu.Lock()
	if owner == nil || r.epoch == owner.epoch+1 && !r.held {
		r.feedbackCode, r.feedbackKind = failure.Code, kind
		r.feedbackPending = true
		r.feedbackReceipt++
	}
	r.mu.Unlock()
	failure.Detail = errors.Join(cause, err).Error()
	return failure
}

// rtRefused gives locally detected backend failures the same typed reason as
// SDK refusals, preventing fail's Stop/Release cleanup from erasing their code.
func rtRefused(format string, args ...any) *control.Rejected {
	return &control.Rejected{Reason: fmt.Sprintf(format, args...)}
}

func rtFailure(cause error) *Error {
	code := "rt_core_failure"
	var rejected *control.Rejected
	var local *Error
	if errors.As(cause, &local) {
		failure := httpError(local.Status, local.Code, "%v", cause)
		failure.LimitViolation = local.LimitViolation
		return failure
	} else if errors.As(cause, &rejected) {
		code = strings.SplitN(rejected.Reason, ":", 2)[0]
	} else if errors.Is(cause, control.ErrConnectionUnsent) {
		code = "rt_core_connection_reopened"
	} else if errors.Is(cause, context.DeadlineExceeded) {
		code = "rt_core_request_slow"
	} else if errors.Is(cause, control.ErrTransport) {
		code = "rt_core_transport_lost"
	} else if errors.Is(cause, control.ErrProtocol) {
		code = "rt_core_protocol_error"
	} else if errors.Is(cause, control.ErrClosed) {
		code = "rt_core_closed"
	} else if errors.Is(cause, context.Canceled) {
		code = "rt_core_cancelled"
	} else if errors.Is(cause, context.DeadlineExceeded) {
		code = "rt_core_timeout"
	}
	failure := httpError(409, code, "%v", cause)
	if rejected != nil {
		failure.LimitViolation = decodeLimitViolation(rejected.ResponseData)
	}
	return failure
}

func (r *RTCoreController) Stop(ctx context.Context) (*StopResult, error) {
	r.noteJogEndBy(ctx)
	return r.stopAction("operator_stop")
}

// StopSelection preserves the selection owner's cause through urgent Stop and
// grant release, preventing a different window's selection from blaming STOP.
func (r *RTCoreController) StopSelection(ctx context.Context) (*StopResult, error) {
	r.noteJogEndBy(ctx)
	return r.stopAction("selection_changed")
}

// noteJogEndBy names the requesting client on a hold this Stop or Disarm
// retires (jog_end.go). It takes only r.mu, never the Stop lane.
func (r *RTCoreController) noteJogEndBy(ctx context.Context) {
	r.mu.Lock()
	if r.jog != nil {
		r.jogEndBy = ClientOf(ctx)
	}
	r.mu.Unlock()
}

func (r *RTCoreController) stopAction(reason string) (*StopResult, error) {
	result, err := r.stop(reason)
	if err != nil {
		failure := rtFailure(err)
		r.mu.Lock()
		r.feedbackCode, r.feedbackKind, r.feedbackPending = failure.Code, "stop", true
		r.feedbackReceipt++
		r.mu.Unlock()
		result.Status = r.statusSnapshot()
		return result, failure
	}
	return result, nil
}

// stop uses a fresh context after cancellation and releases only acknowledged
// Stops. Known absence of authority is an acknowledged no-op; uncertain Stop or
// Release outcomes remain distinct recovery states that both block motion.
func (r *RTCoreController) stop(reason string, deadline ...time.Time) (*StopResult, error) {
	return r.stopObserved(reason, nil, deadline...)
}

// A watcher may finish after Reset has installed a successor grant. Validate
// its captured owner in the same critical section that fences Stop/cancellation.
func (r *RTCoreController) stopObserved(reason string, owner *observedAuthority, deadline ...time.Time) (*StopResult, error) {
	until := time.Now().Add(r.callBudget())
	if len(deadline) != 0 && !deadline[0].IsZero() && deadline[0].Before(until) {
		until = deadline[0]
	}
	ctx, cancel := context.WithDeadline(context.Background(), until)
	defer cancel()
	r.mu.Lock()
	if owner != nil && !owner.current(r) {
		r.mu.Unlock()
		return nil, errObservationSuperseded
	}
	if reason == "operator_stop" && r.move != nil && r.move.cartesian != nil {
		reason = "stop"
	}
	r.stopping++
	r.epoch++
	// Publish the cause with the epoch before cancellation wakes an operation
	// or move observer. Urgent Stop may still be waiting for its lane.
	r.reason = reason
	if r.move != nil {
		r.moveState.State, r.moveState.Reason = "stopped", reason
	}
	if r.opCancel != nil {
		r.opCancel()
	}
	r.mu.Unlock()
	defer func() { r.mu.Lock(); r.stopping--; r.mu.Unlock() }()
	if err := r.stopMu.LockContext(ctx); err != nil {
		return &StopResult{Status: r.statusSnapshot()}, err
	}
	defer r.stopMu.Unlock()
	r.mu.Lock()
	nothingHeld := !r.held && !r.acquiring && r.fence.Session == "" && r.c.Grant().Session == ""
	acquisitionUnknown := r.acquiring
	r.held, r.renewals, r.program = false, nil, nil
	// rt-core writes every output OFF and clears I/O permission in the cycle
	// that applies this Stop; a later run needs a fresh grant and io_arm.
	r.ioArmed = control.Fence{}
	ownerCancel := r.renewOwnerCancel
	r.renewOwnerCancel = nil
	jog := r.retireJointJogLocked(reason)
	if r.move != nil {
		r.moveState.State, r.moveState.Reason = "stopped", reason
		r.move = nil
	}
	if r.session != nil {
		r.session.State, r.session.StopReason, r.session.LastError = "stopped", reason, reason
	}
	r.mu.Unlock()
	if ownerCancel != nil {
		ownerCancel()
	}
	r.c.StopRenewal()
	var err error
	stopped, attempts, note := true, 0, "nothing_held"
	if !nothingHeld {
		_, err = r.c.Stop(ctx)
		stopped, attempts, note = err == nil, 1, ""
	}
	// Urgent Stop revokes the fence first; End still names the original jog.
	if jog != nil {
		_ = r.endJointJog(jog)
	}
	if err == nil && !nothingHeld {
		_, err = r.c.Release(ctx)
	}
	if err != nil && !acquisitionUnknown && r.stopConfirmedByObservation(err) {
		log.Printf("dense-execution: stop %s: %v; a fresh status shows disarmed, no axis enabled, no lease and no grant: Stop confirmed", reason, err)
		stopped, note, err = true, "confirmed_by_observation", nil
	}
	r.mu.Lock()
	r.inhibited = err != nil && stopped
	r.stopUncertain = err != nil && !stopped
	if err != nil {
		r.reason = reason + ": " + err.Error()
	} else {
		r.fence = control.Fence{}
		r.acquiring = false
		if reason == "operator_stop" || reason == "stop" || reason == "disarm" {
			if operatorAcceptanceClears(r.feedbackKind, "stop") {
				r.feedbackCode = ""
			}
			r.feedbackPending = true
			r.feedbackReceipt++
		}
	}
	r.mu.Unlock()
	return &StopResult{OK: err == nil, Stopped: stopped, Attempts: attempts, Note: note, Status: r.statusSnapshot()}, err
}

// stopConfirmedByObservation settles a Stop or Release the adapter refused
// (a typed answer, so it is reachable and processed the request) from one
// fresh native observation (the reset admission's same-host freshness and
// identity proof): disarmed, no axis enabled, no Home, commissioning,
// trajectory or jog running, no lease and no grant. Nothing can move and nobody
// holds authority, which is all a Stop and Release acknowledgement proves.
// Without it a Stop on a session the adapter already retired is refused
// control_session_stale on every retry, and stop_uncertain refuses Arm and
// Reset until an OLP restart (a Rosie 1430 arm, 2026-10-11 12:19). A transport failure, a
// stale, foreign or changed publication, a lease or a grant keeps the outcome
// unconfirmed; the caller never asks while an acquisition's outcome is
// unknown. The SDK's dead fence is cleared so the next Acquire records its grant.
func (r *RTCoreController) stopConfirmedByObservation(cause error) bool {
	var rejected *control.Rejected
	if !errors.As(cause, &rejected) {
		return false
	}
	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()
	s, err := r.resetRead(ctx, nil)
	if err != nil || s.Core.LeaseValid != 0 || s.Grant == nil || s.Grant.GrantActive != 0 {
		return false
	}
	if client, ok := r.c.(interface{ SetFence(control.Fence) }); ok {
		client.SetFence(control.Fence{})
	}
	return true
}

func (r *RTCoreController) Close(context.Context) {
	r.mu.Lock()
	r.closed = true
	resolver := r.cartesianService
	r.cartesianService = nil
	r.mu.Unlock()
	if resolver != nil {
		resolver.Close()
	}
	r.cancel()
	_, _ = r.stop("server_shutdown")
	<-r.done
	// Join cancelled bulk work before closing its connection pools.
	r.opMu.Lock()
	defer r.opMu.Unlock()
	_ = r.c.Close()
}

// DecodeRTCoreConfig rejects misspelled configuration rather than running with
// an omitted authority binding or silently choosing the legacy transport.
func DecodeRTCoreConfig(raw []byte) (RTCoreConfig, error) {
	var cfg RTCoreConfig
	d := json.NewDecoder(strings.NewReader(string(raw)))
	d.DisallowUnknownFields()
	err := d.Decode(&cfg)
	if err == nil && d.Decode(new(any)) != io.EOF {
		return RTCoreConfig{}, errors.New("rt_core config must contain exactly one JSON object")
	}
	return cfg, err
}

var _ Backend = (*RTCoreController)(nil)

// DiscoverRTCore refreshes identity through verified SDK Describe without taking
// control. Pinned configuration/model evidence cannot be replaced by discovery.
func DiscoverRTCore(ctx context.Context, cfg RTCoreConfig) (RTCoreConfig, control.Description, error) {
	ctx, cancel := context.WithTimeout(ctx, rtOperationBudget)
	defer cancel()
	c, err := dialRTCore(cfg)
	if err != nil {
		return cfg, control.Description{}, rtFailure(err)
	}
	defer c.Close()
	d, err := c.Describe(ctx)
	if err != nil {
		return cfg, d, rtFailure(err)
	}
	if cfg.ExpectedBackend != "" && d.Backend != cfg.ExpectedBackend {
		return cfg, d, rtFailure(rtRefused("cell_backend_mismatch"))
	}
	if cfg.Binding.ConfigurationSHA256 != "" && cfg.Binding.ConfigurationSHA256 != d.ConfigurationSHA256 {
		return cfg, d, rtFailure(rtRefused("cell_configuration_mismatch"))
	}
	if d.ContractVersion != 2 || (d.Backend != "simulation" && d.Backend != "ethercat") || !digest(d.ConfigurationSHA256) || !digest(d.MachineSHA256) || !digest(d.DeploymentSHA256) || !digest(d.CapabilitiesDigest) || len(d.Axes) == 0 || len(d.Axes) > 9 || cfg.AxisMask == 0 || cfg.AxisMask>>len(d.Axes) != 0 {
		return cfg, d, rtFailure(rtRefused("cell_description_invalid"))
	}
	cfg.Binding.MachineSHA256, cfg.Binding.ConfigurationSHA256 = d.MachineSHA256, d.ConfigurationSHA256
	cfg.DeploymentSHA256, cfg.CapabilitiesDigest = d.DeploymentSHA256, d.CapabilitiesDigest
	return cfg, d, nil
}

// dialRTCore constructs verified public lanes without acquiring a grant.
func dialRTCore(cfg RTCoreConfig) (*control.Client, error) {
	var err error
	var client *control.Client
	if (cfg.Socket == "") == (cfg.Address == "") {
		return nil, errors.New("rt_core requires exactly one Unix socket or authenticated HTTPS address")
	}
	if cfg.Socket != "" {
		client, err = control.Dial(control.UnixPath(cfg.Socket))
	} else {
		pem, readErr := os.ReadFile(cfg.CAFile)
		if readErr != nil {
			return nil, readErr
		}
		roots := x509.NewCertPool()
		if !roots.AppendCertsFromPEM(pem) {
			return nil, errors.New("rt_core CA is invalid")
		}
		cert, certErr := tls.LoadX509KeyPair(cfg.CertificateFile, cfg.KeyFile)
		if certErr != nil {
			return nil, certErr
		}
		leaf, certErr := x509.ParseCertificate(cert.Certificate[0])
		if certErr != nil {
			return nil, certErr
		}
		// Check the presented pair during TLS verification so a known SAN
		// mismatch retains its cause instead of becoming a generic disconnect.
		client, err = control.DialTLS(cfg.Address, &tls.Config{RootCAs: roots, Certificates: []tls.Certificate{cert},
			VerifyConnection: func(tls.ConnectionState) error {
				pairs := 0
				for _, uri := range leaf.URIs {
					if uri.Scheme == "rosie-pair" {
						if uri.Opaque != cfg.Binding.PairID {
							return rtRefused("session_principal_mismatch: certificate pair differs from binding")
						}
						pairs++
					}
				}
				if pairs != 1 {
					return rtRefused("session_principal_mismatch: certificate requires one pair SAN")
				}
				return nil
			}})
	}
	if err != nil {
		return nil, err
	}
	return client, nil
}

// setObservationLocked records a native observation and the moment motion
// last became done.
func (r *RTCoreController) setObservationLocked(s *control.Status) {
	if s.Motion.Done && (r.observation == nil || !r.observation.Motion.Done) {
		r.restSinceNS = s.Motion.TimeNS
	}
	if h := r.jog; h != nil && h.path == nil && h.intent.cartesian == nil && h.intent.Axis >= 0 && h.intent.Axis < len(s.Axes) {
		if a := s.Axes[h.intent.Axis]; a.LogicalVelocity != nil && finiteJoint(*a.LogicalVelocity) {
			h.peakVelocity = math.Max(h.peakVelocity, math.Abs(*a.LogicalVelocity))
		}
	}
	r.observation = s
	r.attributeJogEndToDriveFaultLocked(s)
}
