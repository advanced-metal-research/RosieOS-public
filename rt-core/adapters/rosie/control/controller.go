//go:build linux

// Package control owns Rosie session/plan identity around the native RT owner.
// It contains no servo loop, drive counts, URDF catalogue or velocity follower.
package control

import (
	"context"
	"crypto/rand"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"os"
	"strings"
	"sync"
	"time"

	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/ipcclient"
)

// maxGrantLease reads compiled native policy; legacy fixtures retain the LAN default.
func (c *Controller) maxGrantLease() time.Duration {
	if c.description.MaxGrantLeaseNS != 0 {
		return time.Duration(c.description.MaxGrantLeaseNS)
	}
	return time.Duration(ipcclient.FastControlMaxGrantLeaseNs)
}

// The adapter depends only on the native command and observation boundary.
type nativeClient interface {
	Err() error
	Status() (ipcclient.Status, error)
	Motion() (ipcclient.MotionState, error)
	Enable(context.Context, uint32) (uint64, error)
	Arm(context.Context) (uint64, error)
	Home(context.Context, uint32) (uint64, error)
	FaultReset(context.Context, uint32) (uint64, error)
	Jog(context.Context, uint32, []float64, time.Duration) (uint64, error)
	WaitResult(context.Context, uint64) (ipcclient.CommandResult, error)
	PrepareTrajectory(context.Context, uint64, uint32, []ipcclient.Point) (uint64, error)
	StartTrajectory(context.Context, uint64) (uint64, error)
	DiscardTrajectory(context.Context, uint64) (uint64, error)
	Stop(context.Context) (uint64, error)
	MirrorAcquire(context.Context, ipcclient.GrantIdentity, uint64) (ipcclient.GrantReceipt, error)
	MirrorRenew(context.Context, ipcclient.GrantIdentity, uint64) (ipcclient.GrantReceipt, error)
}

// Controller is the single local adapter session. Its operation lock orders
// command submission; Stop revokes first, then fences any already-running writer.
type Controller struct {
	resources           *compiledResources
	events              eventRing
	eventObservation    eventObservation
	anchorStore         anchors.Store
	anchorPairRefusals  map[uint32]string // Protected by mu; cleared by a replacement capture or qualification.
	core                nativeClient
	native              *restartNative
	reconnect           *ipcclient.Options
	reconnectContext    context.Context
	daemonIncarnation   string
	adapterIncarnation  string
	sessions            map[string]sessionRecord
	description         ipcclient.Description
	binding             Binding
	axes                []string
	mu                  sync.Mutex
	operationMu         sync.Mutex
	preparations        preparationSlots
	jogUpdateMu         sync.Mutex // One bounded publication; never held by bulk or lifecycle waits.
	localJog            *ipcclient.LocalJogSession
	path                *pathSession // Protected by mu; retained after End for status.
	jogSourceSequence   uint64
	jogIngress          JogIngressObservation
	jogRefusalLog       jogRefusalLimiter
	authorityMu         sync.Mutex // Grant transactions; urgent Stop submission never waits here.
	grant               Grant
	expires             time.Time
	stopping            bool
	stopFlight          *stopFlight
	expiredSession      string
	handles             map[uint64]*preparation
	executionGeneration uint64
	requests            requestTable
	sessionCtx          context.Context
	cancel              context.CancelFunc
	program             *Program
	execution           Execution
	nextID              uint64
	done                chan struct{}
	wg                  sync.WaitGroup
	closeOnce           sync.Once
	maintenance         *maintenanceSource // Protected by mu; nil disables the notice (maintenance.go).
}

// DescribedDenseAxes is the dense axis map a cell derives from its own
// description: the ids of the described rotary axes (position_unit "rad"),
// in native index order, one wire column each. A linear axis has no place on
// the joint wire and is left out.
func DescribedDenseAxes(description ipcclient.Description) []string {
	var ids []string
	for _, axis := range description.Axes {
		if axis.PositionUnit == "rad" {
			ids = append(ids, axis.ID)
		}
	}
	return ids
}

func New(core *ipcclient.Client, binding Binding, compiledDirectory ...string) (*Controller, error) {
	description, err := core.Describe()
	if err != nil {
		return nil, err
	}
	// The dense axis map is the cell's own description, not a start flag: the
	// wire's columns map to the described rotary axes in order (a six-axis
	// Rosie 1420 maps six, the nine-motor bench nine, the one-motor
	// mini bench one). Nothing outside the robot definition and the machine
	// config states the axis count (owner direction 2026-09-20).
	denseAxisIDs := DescribedDenseAxes(description)
	var resources *compiledResources
	if len(compiledDirectory) > 1 {
		return nil, errors.New(ReasonResourceArtifactInvalid)
	}
	if len(compiledDirectory) == 1 && compiledDirectory[0] != "" {
		resources, err = loadCompiledResources(compiledDirectory[0], description)
		if err != nil {
			return nil, err
		}
	}
	if binding.PairID == "" || binding.Revision == 0 || !binding.matches(Binding{
		PairID: binding.PairID, Revision: binding.Revision, ConfigurationSHA256: description.ConfigurationSHA256,
		MachineSHA256: description.MachineSHA256}) {
		return nil, errors.New(ReasonInvalidPairConfigurationBinding)
	}
	binding.MachineSHA256 = description.MachineSHA256
	binding.ConfigurationSHA256 = description.ConfigurationSHA256
	clock, err := core.LocalJogClock()
	if err != nil {
		return nil, err
	}
	var incarnation [16]byte
	if _, err := rand.Read(incarnation[:]); err != nil {
		return nil, err
	}
	native := &restartNative{client: core}
	c := &Controller{core: native, native: native, daemonIncarnation: clock.Incarnation,
		resources:          resources,
		anchorStore:        anchors.Store{Directory: os.Getenv("ROSIE_RT_ANCHOR_DIR")},
		adapterIncarnation: hex.EncodeToString(incarnation[:]), sessions: make(map[string]sessionRecord),
		description: description, binding: binding, axes: append([]string(nil), denseAxisIDs...), done: make(chan struct{})}
	c.sessionCtx, c.cancel = context.WithCancel(context.Background())
	c.cancel()
	if state, err := c.core.Status(); err == nil {
		c.observeEventsLocked(state)
	}
	c.wg.Add(1)
	go c.watch()
	return c, nil
}

// ReconnectNative enables transport recovery without restoring any application
// authority. The existing watcher remains the sole reconnect owner.
func (c *Controller) ReconnectNative(ctx context.Context, options ipcclient.Options) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.reconnect, c.reconnectContext = &options, ctx
}

func (c *Controller) restartFenceLocked(f Fence) error {
	if f.Session == "" || c.sessions == nil {
		return nil
	}
	session, known := c.sessions[f.Session]
	if known && session.incarnation != c.daemonIncarnation {
		return errors.New(ReasonDaemonRestarted)
	}
	if !known && len(f.Session) == 64 {
		if _, err := hex.DecodeString(f.Session); err == nil {
			return errors.New(ReasonFence)
		}
	}
	return nil
}

// CheckRestartFence gives restart reasons precedence over clock/handle errors.
// It does not authorize an operation; normal session checks still apply.
func (c *Controller) CheckRestartFence(f Fence) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.restartFenceLocked(f)
}

// ProcessSnapshot pins the incarnation while reading its observations. A
// concurrent reconnect must never label predecessor state as successor state.
func (c *Controller) ProcessSnapshot() (ProcessStatus, error) {
	return c.processSnapshot(true)
}

// processSnapshot keeps all native evidence and lifecycle observations identical.
// Current-only readers skip constructing the historical handle list under mu.
func (c *Controller) processSnapshot(includeHistory bool) (ProcessStatus, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	status := ProcessStatus{DaemonIncarnation: c.daemonIncarnation, AdapterIncarnation: c.adapterIncarnation, JogIngress: c.jogIngress}
	var err error
	status.Core, status.Motion, status.Execution, err = c.snapshotWithHistoryLocked(includeHistory)
	if err != nil {
		return ProcessStatus{}, err
	}
	if c.native != nil {
		if status.Core.DaemonIncarnation != status.DaemonIncarnation {
			return ProcessStatus{}, errors.New(ReasonDaemonRestarted)
		}
		if grant, grantErr := c.native.FastObservation(); grantErr == nil {
			status.Grant = &grant
		}
		if jog, jogErr := c.native.JogObservation(); jogErr == nil {
			status.Jog = &jog
		}
	}
	return status, err
}

// Cancel and drain the predecessor before swapping transports. In particular,
// old background cleanup must never discard a successor's prepared handle.
func (c *Controller) reconnectLostNative() {
	c.mu.Lock()
	if c.reconnect == nil {
		c.mu.Unlock()
		return
	}
	options, parent := *c.reconnect, c.reconnectContext
	c.cancel()
	c.stopping = true
	c.grant.Session = ""
	c.program, c.localJog = nil, nil
	c.requests = requestTable{}
	c.retireHandlesLocked()
	flight := c.stopFlight
	c.mu.Unlock()
	if flight != nil {
		<-flight.done
	}
	c.authorityMu.Lock()
	defer c.authorityMu.Unlock()
	c.operationMu.Lock()
	defer c.operationMu.Unlock()
	c.jogUpdateMu.Lock()
	defer c.jogUpdateMu.Unlock()
	select {
	case <-c.done:
		return
	default:
	}
	old := c.native.current()
	_ = old.Close()
	// Unverified reconnect attempt budget: 250 ms; the watcher retries outages.
	const reconnectAttempt = 250 * time.Millisecond
	ctx, cancel := context.WithTimeout(parent, reconnectAttempt)
	defer cancel()
	next, err := ipcclient.Dial(ctx, options)
	if err != nil {
		return
	}
	clock, err := next.LocalJogClock()
	if err != nil {
		_ = next.Close()
		return
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	select {
	case <-c.done:
		_ = next.Close()
		return
	default:
	}
	if clock.Incarnation != c.daemonIncarnation {
		c.daemonIncarnation = clock.Incarnation
		// All handles belong to the old daemon, including terminal records.
		for _, handle := range c.handles {
			handle.State, handle.points = HandleRetired, nil
		}
		c.execution = Execution{State: "released", Error: ReasonDaemonRestarted}
	}
	c.native.mu.Lock()
	c.native.client = next
	c.native.mu.Unlock()
	c.stopping = false
}

// restartNative keeps one stable interface for concurrent ingress. Its read
// locks allow independent native lanes; replacement waits for their last call.
type restartNative struct {
	mu     sync.RWMutex
	client *ipcclient.Client
}

func (n *restartNative) current() *ipcclient.Client {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client
}

func (n *restartNative) Err() error {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.Err()
}

func (n *restartNative) Status() (ipcclient.Status, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.Status()
}

func (n *restartNative) Motion() (ipcclient.MotionState, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.Motion()
}

func (n *restartNative) LocalJogClock() (ipcclient.LocalJogClock, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.LocalJogClock()
}

func (n *restartNative) JogObservation() (ipcclient.JogObservation, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.JogObservation()
}

func (n *restartNative) FastObservation() (ipcclient.GrantObservation, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.FastObservation()
}

func (n *restartNative) Enable(ctx context.Context, mask uint32) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.Enable(ctx, mask)
}

func (n *restartNative) Arm(ctx context.Context) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.Arm(ctx)
}

func (n *restartNative) Home(ctx context.Context, mask uint32) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.Home(ctx, mask)
}

func (n *restartNative) RestoreAnchor(ctx context.Context, mask uint32) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.RestoreAnchor(ctx, mask)
}

func (n *restartNative) FaultReset(ctx context.Context, mask uint32) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.FaultReset(ctx, mask)
}

func (n *restartNative) Jog(ctx context.Context, mask uint32, velocity []float64, timeout time.Duration) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.Jog(ctx, mask, velocity, timeout)
}

func (n *restartNative) WaitResult(ctx context.Context, sequence uint64) (ipcclient.CommandResult, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return ipcclient.CommandResult{}, err
	}
	return n.client.WaitResult(ctx, sequence)
}

func (n *restartNative) PrepareTrajectory(ctx context.Context, id uint64, mask uint32, points []ipcclient.Point) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.PrepareTrajectory(ctx, id, mask, points)
}

func (n *restartNative) StartTrajectory(ctx context.Context, id uint64) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.StartTrajectory(ctx, id)
}

func (n *restartNative) DiscardTrajectory(ctx context.Context, id uint64) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.DiscardTrajectory(ctx, id)
}

func (n *restartNative) Stop(ctx context.Context) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.Stop(ctx)
}

func (n *restartNative) MirrorAcquire(ctx context.Context, identity ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return ipcclient.GrantReceipt{}, err
	}
	return n.client.MirrorAcquire(ctx, identity, deadline)
}

func (n *restartNative) MirrorRenew(ctx context.Context, identity ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return ipcclient.GrantReceipt{}, err
	}
	return n.client.MirrorRenew(ctx, identity, deadline)
}

func (n *restartNative) BeginLocalJog(ctx context.Context, identity ipcclient.GrantIdentity, mask uint32, origin, deadline uint64) (ipcclient.LocalJogSession, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return ipcclient.LocalJogSession{}, err
	}
	return n.client.BeginLocalJog(ctx, identity, mask, origin, deadline)
}

func (n *restartNative) UpdateLocalJog(ctx context.Context, session ipcclient.LocalJogSession, sequence, origin, deadline uint64, mask uint32, velocity []float64) error {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return err
	}
	return n.client.UpdateLocalJog(ctx, session, sequence, origin, deadline, mask, velocity)
}

func (n *restartNative) EndLocalJog(ctx context.Context, session ipcclient.LocalJogSession) error {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return err
	}
	return n.client.EndLocalJog(ctx, session)
}

func (c *Controller) Describe() Description {
	description := c.description
	var current *ipcclient.Status
	// Inventory changes independently of machine identity. Reuse the latest
	// native observation so the startup cache cannot hide held-device changes.
	if c.core != nil {
		if state, err := c.core.Status(); err == nil {
			description.Bus = state.Bus
			current = &state
		}
	}
	robot, drives := c.describeResources(current)
	return Description{NativeDescription: description, ContractVersion: ContractVersion,
		Robot: robot, Drives: drives,
		ControlIdleTimeoutNS: CompiledControlIdleTimeoutNS,
		CapabilitiesDigest:   CapabilitiesDigest, Capabilities: Capabilities()}
}
func (c *Controller) Snapshot() (ipcclient.Status, ipcclient.MotionState, Execution, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.snapshotLocked()
}

func (c *Controller) snapshotLocked() (ipcclient.Status, ipcclient.MotionState, Execution, error) {
	return c.snapshotWithHistoryLocked(true)
}

func (c *Controller) snapshotWithHistoryLocked(includeHistory bool) (ipcclient.Status, ipcclient.MotionState, Execution, error) {
	state, err := c.core.Status()
	motion, motionErr := c.core.Motion()
	if err == nil {
		err = motionErr
	}
	if c.core.Err() != nil || (c.grant.Session != "" && !time.Now().Before(c.expires)) {
		c.retireHandlesLocked()
	} else if err == nil {
		c.observeHandlesLocked(motion)
	}
	if err == nil {
		// Copy before annotating: a native client may retain its snapshot storage.
		if len(c.anchorPairRefusals) != 0 {
			state.Axes = append([]ipcclient.AxisStatus(nil), state.Axes...)
			for axis, field := range c.anchorPairRefusals {
				if int(axis) < len(state.Axes) {
					a := &state.Axes[axis]
					a.AnchorReason, a.AnchorIdentityChangedField = ReasonAnchorIdentityMismatch, field
					// Measured native enum contract: CoordinateReason::IdentityChanged = 3.
					a.CoordinateReason = 3
				}
			}
		}
		c.observeEventsLocked(state)
	}
	execution := c.execution
	// Keep the required array on the wire, without copying or sorting history.
	execution.Handles = []HandleRecord{}
	if includeHistory {
		execution.Handles = c.handleViewLocked()
	}
	execution.Generation = c.executionGeneration
	return state, motion, execution, err
}

// matches uses machine semantics for new grants; legacy callers retain their
// whole-artifact binding. A supplied wrong machine digest never falls back.
func (binding Binding) matches(expected Binding) bool {
	if binding.PairID != expected.PairID || binding.Revision != expected.Revision {
		return false
	}
	if binding.MachineSHA256 != "" {
		return binding.MachineSHA256 == expected.MachineSHA256
	}
	return binding.ConfigurationSHA256 != "" && binding.ConfigurationSHA256 == expected.ConfigurationSHA256
}

func (c *Controller) Acquire(controller string, binding Binding, requestedMS ...int) (Grant, error) {
	return c.acquire(controller, binding, sessionPrincipal{}, requestedMS...)
}

// acquire records transport ownership before publishing the grant, preventing
// a remote session from being admitted as local while its response is pending.
func (c *Controller) acquire(controller string, binding Binding, principal sessionPrincipal, requestedMS ...int) (Grant, error) {
	// Unverified absolute lease bound: 10000 ms; an omitted request preserves LAN behavior.
	requested := int(ipcclient.FastControlMaxGrantLeaseNs / 1000000)
	if len(requestedMS) > 1 || (len(requestedMS) == 1 && (requestedMS[0] < 0 || requestedMS[0] > 10000)) {
		return Grant{}, errors.New(ReasonInvalidRequestedLease)
	}
	if len(requestedMS) == 1 && requestedMS[0] > 0 {
		requested = requestedMS[0]
	}
	applicationLease := time.Duration(requested) * time.Millisecond
	if ceiling := c.maxGrantLease(); applicationLease > ceiling {
		applicationLease = ceiling
	}
	// Native expiry can become visible before the watcher finishes retiring its
	// grant. Join that authenticated Stop before attempting a successor, avoiding
	// a spurious control_already_owned after expired authority was observed.
	c.mu.Lock()
	if controller == "" || len(controller) > 63 || !binding.matches(c.binding) {
		c.mu.Unlock()
		return Grant{}, errors.New(ReasonAuthorityBindingMismatch)
	}
	expired := c.grant.Session != "" && !time.Now().Before(c.expires)
	expiredFence := Fence{c.grant.Session, c.grant.Generation}
	flight, expiredSession := c.stopFlight, c.expiredSession
	c.mu.Unlock()
	if expired {
		if _, err := c.stopGrant(expiredFence, true, true); err != nil {
			return Grant{}, err
		}
	} else if flight != nil && flight.fence.Session == expiredSession {
		<-flight.done
		if flight.err != nil {
			return Grant{}, flight.err
		}
	}
	deadlineNS := ipcclient.HostMonotonicNS() + uint64(applicationLease)
	expires := time.Now().Add(applicationLease)
	c.authorityMu.Lock()
	defer c.authorityMu.Unlock()
	c.mu.Lock()
	if controller == "" || len(controller) > 63 || !binding.matches(c.binding) {
		c.mu.Unlock()
		return Grant{}, errors.New(ReasonAuthorityBindingMismatch)
	}
	if c.stopping || c.grant.Session != "" {
		c.mu.Unlock()
		return Grant{}, errors.New(ReasonControlAlreadyOwned)
	}
	if err := c.core.Err(); err != nil {
		c.mu.Unlock()
		return Grant{}, err
	}
	var token [32]byte
	if _, err := rand.Read(token[:]); err != nil {
		c.mu.Unlock()
		return Grant{}, err
	}
	c.sessionCtx, c.cancel = context.WithCancel(context.Background())
	if c.grant.Generation == ^uint64(0) {
		c.cancel()
		c.mu.Unlock()
		return Grant{}, errors.New(ReasonApplicationGenerationExhausted)
	}
	binding.MachineSHA256 = c.binding.MachineSHA256
	binding.ConfigurationSHA256 = c.binding.ConfigurationSHA256
	c.grant = Grant{Session: hex.EncodeToString(token[:]), Generation: c.grant.Generation + 1, Controller: controller, Binding: binding, LeaseMS: int(applicationLease / time.Millisecond), DeadlineHostNS: deadlineNS}
	if c.sessions == nil {
		c.sessions = make(map[string]sessionRecord)
	}
	c.sessions[c.grant.Session] = sessionRecord{incarnation: c.daemonIncarnation, principal: principal, granted: time.Now()}
	c.expires = expires
	fence := Fence{c.grant.Session, c.grant.Generation}
	identity := c.nativeGrantIdentity()
	parent := c.sessionCtx
	c.mu.Unlock()
	ctx, cancel := context.WithDeadline(parent, expires)
	defer cancel()
	if _, err := c.core.MirrorAcquire(ctx, identity, deadlineNS); err != nil {
		return Grant{}, c.failAuthority(fence, err)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := c.authorize(fence); err != nil {
		return Grant{}, err
	}
	// Retiring the previous grant already retired its handles and program.
	// Only a successfully admitted new owner starts a new execution summary;
	// otherwise an old terminal error can make observers Stop a fresh Home.
	// Native fault/Home evidence and the historical event stream remain intact.
	c.execution = Execution{}
	c.grantEventLocked("grant_acquired", false)
	return c.grant, nil
}

func (c *Controller) nativeGrantIdentity() ipcclient.GrantIdentity {
	digest := sha256.Sum256([]byte(c.grant.Session))
	identity := ipcclient.GrantIdentity{Generation: c.grant.Generation}
	copy(identity.CapabilityID[:], digest[:16])
	return identity
}

// A publication error may mean native accepted the grant but its receipt was
// lost. Cancel local producers and use the independent urgent path before
// allowing another Acquire. Never continue on a locally renewed token alone.
func (c *Controller) failAuthority(fence Fence, cause error) error {
	c.mu.Lock()
	if c.stopping || c.grant.Session != fence.Session || c.grant.Generation != fence.Generation {
		c.mu.Unlock()
		return cause // Stop/Close already owns cancellation; do not affect a successor.
	}
	c.cancel()
	c.retireHandlesLocked()
	c.requests = requestTable{}
	c.grant.Session = ""
	c.grantEventLocked("grant_revoked", true)
	c.program = nil
	c.execution = Execution{State: "faulted", Error: cause.Error()}
	c.localJog = nil
	c.stopping = true
	c.mu.Unlock()
	_, stopErr := c.core.Stop(context.Background())
	err := errors.Join(cause, stopErr)
	c.mu.Lock()
	if c.grant.Generation == fence.Generation {
		select {
		case <-c.done:
		default:
			c.stopping = false
		}
		c.execution = Execution{State: "faulted", Error: err.Error()}
	}
	c.mu.Unlock()
	return err
}
func (c *Controller) authorize(f Fence) error {
	if err := c.restartFenceLocked(f); err != nil {
		return err
	}
	if c.stopping || c.grant.Session == "" || f.Session != c.grant.Session || f.Generation != c.grant.Generation || !time.Now().Before(c.expires) {
		return errors.New(ReasonControlSessionStale)
	}
	return nil
}

// Renew is a session keepalive, including while Stop fences writers. An older
// motion generation in the same session may renew and learns the current fence;
// it cannot submit motion. During Stop only the local lease advances, with
// stopping=true; native authority is mirrored after inhibition and writer drain.
// Expiry wins once the current deadline passes, even if renewal entered earlier.
func (c *Controller) Renew(f Fence) (Grant, error) {
	c.mu.Lock()
	applicationLease := time.Duration(c.grant.LeaseMS) * time.Millisecond
	c.mu.Unlock()
	deadlineNS := ipcclient.HostMonotonicNS() + uint64(applicationLease)
	expires := time.Now().Add(applicationLease)
	c.mu.Lock()
	if err := c.authorizeRenew(f); err != nil {
		c.mu.Unlock()
		return Grant{}, err
	}
	if c.stopping {
		grant := c.keepaliveLocked(expires, deadlineNS)
		c.mu.Unlock()
		return grant, nil
	}
	c.mu.Unlock()
	c.authorityMu.Lock()
	defer c.authorityMu.Unlock()
	c.mu.Lock()
	if err := c.authorizeRenew(f); err != nil {
		c.mu.Unlock()
		return Grant{}, err
	}
	if !time.Now().Before(expires) {
		c.mu.Unlock()
		return Grant{}, errors.New(ReasonExpired)
	}
	if c.stopping {
		grant := c.keepaliveLocked(expires, deadlineNS)
		c.mu.Unlock()
		return grant, nil
	}
	f = Fence{c.grant.Session, c.grant.Generation}
	identity := c.nativeGrantIdentity()
	parent := c.sessionCtx
	c.mu.Unlock()
	ctx, cancel := context.WithDeadline(parent, expires)
	defer cancel()
	_, mirrorErr := c.core.MirrorRenew(ctx, identity, deadlineNS)
	c.mu.Lock()
	if c.stopping && c.authorizeRenew(f) == nil && time.Now().Before(expires) {
		grant := c.keepaliveLocked(expires, deadlineNS)
		c.mu.Unlock()
		return grant, nil
	}
	if mirrorErr != nil {
		var rejected *ipcclient.GrantRejectedError
		expired := c.expiredSession == f.Session || !time.Now().Before(c.expires) ||
			(errors.As(mirrorErr, &rejected) && rejected.Reason == ipcclient.GrantReasonExpired)
		if expired {
			c.expiredSession = f.Session
		}
		c.mu.Unlock()
		cause := c.failAuthority(f, mirrorErr)
		if expired || !time.Now().Before(expires) {
			return Grant{}, errors.New(ReasonExpired)
		}
		return Grant{}, cause
	}
	defer c.mu.Unlock()
	if err := c.authorizeRenew(f); err != nil {
		return Grant{}, err
	}
	if !time.Now().Before(expires) {
		return Grant{}, errors.New(ReasonExpired)
	}
	c.expires = expires
	c.grant.DeadlineHostNS = deadlineNS
	c.grantEventLocked("grant_renewed", false)
	return c.grant, nil
}

func (c *Controller) authorizeRenew(f Fence) error {
	if err := c.restartFenceLocked(f); err != nil {
		return err
	}
	if f.Session != "" && (f.Session == c.expiredSession || (f.Session == c.grant.Session && !time.Now().Before(c.expires))) {
		return errors.New(ReasonExpired)
	}
	if c.grant.Session == "" || f.Session != c.grant.Session || f.Generation == 0 || f.Generation > c.grant.Generation {
		return errors.New(ReasonControlSessionStale)
	}
	return nil
}

func (c *Controller) keepaliveLocked(expires time.Time, deadline uint64) Grant {
	if expires.After(c.expires) {
		c.expires = expires
		c.grant.DeadlineHostNS = deadline
	}
	grant := c.grant
	grant.Stopping = true
	c.grantEventLocked("grant_renewed", true)
	return grant
}
func (c *Controller) operation(f Fence, fn func(context.Context) (uint64, error)) (uint64, error) {
	c.operationMu.Lock()
	defer c.operationMu.Unlock()
	c.mu.Lock()
	err := c.authorize(f)
	ctx := c.sessionCtx
	c.mu.Unlock()
	if err != nil {
		return 0, err
	}
	return fn(ctx)
}
func (c *Controller) Enable(f Fence, mask uint32) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		seq, err := c.core.Enable(ctx, mask)
		if err == nil {
			_, err = c.core.WaitResult(ctx, seq)
		}
		return seq, err
	})
}
func (c *Controller) Arm(f Fence) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		seq, err := c.core.Arm(ctx)
		if err == nil {
			_, err = c.core.WaitResult(ctx, seq)
		}
		return seq, err
	})
}

// Home belongs to the grant, including receipt draining and anchor capture.
// Losing an HTTP response cannot abandon a native job or select restoration.
func (c *Controller) Home(f Fence, mask uint32) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		before, beforeErr := c.core.Status()
		if c.anchorStore.Directory != "" && beforeErr != nil {
			return 0, beforeErr
		}
		homeCtx, cancel := context.WithTimeout(ctx, time.Minute)
		defer cancel()
		seq, err := c.core.Home(homeCtx, mask)
		if err == nil {
			_, err = c.core.WaitResult(homeCtx, seq)
		}
		if err == nil && c.anchorStore.Directory != "" {
			err = c.captureAnchors(homeCtx, mask, before.HomeEpoch)
		}
		if err != nil {
			_, _ = c.core.Stop(context.Background())
		}
		return seq, err
	})
}

// anchorRecord uses daemon observations in the count frame. Missing source or
// observed revision fails closed; configured identity alone cannot restore Home.
func anchorRecord(s ipcclient.Status, axis uint32, binding Binding) (anchors.Record, error) {
	if int(axis) >= len(s.Axes) {
		return anchors.Record{}, anchors.ErrSource
	}
	a := s.Axes[axis]
	if a.RevisionValid == 0 || a.PDOFresh == 0 || a.CoordinateValid == 0 || a.CoordinateSourceValid == 0 ||
		a.CoordinateCounts < math.MinInt32 || a.CoordinateCounts > math.MaxInt32 {
		return anchors.Record{}, anchors.ErrSource
	}
	return anchors.Record{PairID: binding.PairID, PairRevision: binding.Revision, CoordinateIdentitySHA256: a.CoordinateIdentitySHA256, CoordinateIdentity: a.CoordinateIdentity, Axis: axis,
		Vendor: a.VendorID, Product: a.ProductCode, Revision: a.Revision, Serial: a.Serial, SerialValid: a.SerialValid,
		HomeEpoch: s.HomeEpoch, AnchorCounts: int32(a.CoordinateCounts), HomeOffset: a.HomeOffset,
		TurnsCounts: a.AbsoluteSourceCounts, ToleranceCounts: a.AnchorToleranceCounts,
		DaemonIncarnation: s.DaemonIncarnation, CapturedHostNS: time.Now().UnixNano()}, nil
}

// Restore can precede a command-frame binding for the versioned raw A6 source.
// Keep capture strict: this record is only an offer, never persisted Home proof.
// Native still verifies the saved command frame, source, epochs and stationarity.
func restoreAnchorRecord(s ipcclient.Status, axis uint32, binding Binding) (anchors.Record, error) {
	if int(axis) >= len(s.Axes) {
		return anchors.Record{}, anchors.ErrSource
	}
	a := s.Axes[axis]
	if !strings.HasPrefix(a.CoordinateIdentity, "anchor.source=raw_multi_turn_encoder|3\n") {
		return anchorRecord(s, axis, binding)
	}
	proof := a.IndependentAnchorSource
	if proof == nil || !proof.Valid || proof.CompletedNS == 0 || proof.MaximumAgeNS == 0 ||
		proof.CompletedNS > s.TimeNS || s.TimeNS-proof.CompletedNS > proof.MaximumAgeNS ||
		a.RevisionValid == 0 || a.PDOFresh == 0 || s.ConfigVerifiedMask&(1<<axis) == 0 ||
		proof.EncoderCounts < math.MinInt32 || proof.EncoderCounts > math.MaxInt32 {
		return anchors.Record{}, anchors.ErrSource
	}
	return anchors.Record{PairID: binding.PairID, PairRevision: binding.Revision,
		CoordinateIdentitySHA256: a.CoordinateIdentitySHA256, CoordinateIdentity: a.CoordinateIdentity, Axis: axis,
		Vendor: a.VendorID, Product: a.ProductCode, Revision: a.Revision, Serial: a.Serial, SerialValid: a.SerialValid,
		HomeEpoch: s.HomeEpoch, AnchorCounts: a.PositionCounts, HomeOffset: a.HomeOffset,
		TurnsCounts: int32(proof.EncoderCounts), ToleranceCounts: a.AnchorToleranceCounts,
		DaemonIncarnation: s.DaemonIncarnation, CapturedHostNS: time.Now().UnixNano()}, nil
}

// captureAnchors waits for completed Home and qualified coordinates in one
// inhibited publication. Home validity alone can precede the coordinate source;
// persist only after every selected axis qualifies, never an older Home sample.
func (c *Controller) captureAnchors(ctx context.Context, mask uint32, previousEpoch uint64) error {
	// Unverified metrics observation polling interval: 10 milliseconds.
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()
	for {
		s, err := c.core.Status()
		if err != nil {
			return err
		}
		if s.HomeEpoch > previousEpoch && s.CommissioningPhase == 0 && s.HomeValidMask&mask == mask {
			if s.Armed != 0 || s.AxisEnableMask != 0 {
				return anchors.ErrSource
			}
			var records []anchors.Record
			ready := true
			for axis := uint32(0); axis < uint32(len(s.Axes)); axis++ {
				if mask&(1<<axis) == 0 {
					continue
				}
				r, err := anchorRecord(s, axis, c.binding)
				if err != nil {
					ready = false
					break
				}
				if !r.MatchesPair(c.binding.PairID, c.binding.Revision) {
					return anchors.ErrIdentity
				}
				records = append(records, r)
			}
			if ready {
				for _, r := range records {
					if err = c.anchorStore.Save(r); err != nil {
						return err
					}
					c.mu.Lock()
					delete(c.anchorPairRefusals, r.Axis)
					c.mu.Unlock()
				}
				return nil
			}
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

// RestoreAnchor submits only an explicit, qualified offer under the current
// application fence and returns recovery from a publication after its receipt,
// preventing cached Home evidence from being reported as a completed restore.
func (c *Controller) RestoreAnchor(f Fence, mask uint32) (uint64, *RecoveryStatus, error) {
	var recovery *RecoveryStatus
	sequence, err := c.operation(f, func(ctx context.Context) (uint64, error) {
		s, err := c.core.Status()
		if err != nil {
			return 0, err
		}
		if mask == 0 || mask & ^uint32((1<<len(s.Axes))-1) != 0 {
			return 0, errors.New("invalid_axis_mask")
		}
		if s.Armed != 0 || s.AxisEnableMask != 0 || s.CommissioningPhase != 0 {
			return 0, errors.New("mode_conflict")
		}
		for axis := uint32(0); axis < uint32(len(s.Axes)); axis++ {
			if mask&(1<<axis) == 0 {
				continue
			}
			saved, err := c.anchorStore.Load(axis)
			if err != nil {
				// Keep the public catalogue reason stable; native status carries
				// the exact anchor identity mismatch detail.
				if errors.Is(err, anchors.ErrSource) {
					return 0, anchors.ErrSource
				}
				return 0, err
			}
			// Pair authority is independent of source quality. Comparing the
			// saved binding first prevents invalid evidence from masking a wrong pair.
			field := ""
			if saved.PairID != c.binding.PairID {
				field = "pair.id"
			} else if saved.PairRevision != c.binding.Revision {
				field = "pair.revision"
			}
			c.mu.Lock()
			if field != "" {
				if c.anchorPairRefusals == nil {
					c.anchorPairRefusals = make(map[uint32]string)
				}
				c.anchorPairRefusals[axis] = field
			} else {
				delete(c.anchorPairRefusals, axis)
			}
			c.mu.Unlock()
			if field != "" {
				return 0, anchors.ErrIdentity
			}
			current, err := restoreAnchorRecord(s, axis, c.binding)
			if err != nil {
				return 0, err
			}
			if err = saved.Qualify(current, true); err != nil {
				return 0, err
			}
			// The daemon reads the same offer independently. An adapter-only
			// directory must report the missing native offer, not generic not_ready.
			switch s.Axes[axis].AnchorReason {
			case ReasonAnchorMissing:
				return 0, anchors.ErrMissing
			case ReasonAnchorIdentityMismatch:
				return 0, anchors.ErrIdentity
			case ReasonAnchorSourceInvalid:
				return 0, anchors.ErrSource
			case ReasonAnchorDisagrees:
				return 0, anchors.ErrDisagrees
			}
		}
		restorer, ok := c.core.(interface {
			RestoreAnchor(context.Context, uint32) (uint64, error)
		})
		if !ok {
			return 0, anchors.ErrSource
		}
		sequence, err := restorer.RestoreAnchor(ctx, mask)
		if err == nil {
			_, err = c.core.WaitResult(ctx, sequence)
		}
		if err != nil {
			return sequence, err
		}
		after := ipcclient.HostMonotonicNS()
		// Unverified recovery publication budget: 1 second; require observed state.
		ctx, cancel := context.WithTimeout(ctx, time.Second)
		defer cancel()
		// Unverified observation polling interval: 5 milliseconds.
		ticker := time.NewTicker(5 * time.Millisecond)
		defer ticker.Stop()
		for {
			state, err := c.core.Status()
			if err == nil && state.TimeNS >= after && state.CommissioningPhase == 0 &&
				state.HomeValidMask&mask == mask && state.Recovery != nil && state.Recovery.HomeValidMask&mask == mask {
				recovery = state.Recovery
				return sequence, nil
			}
			select {
			case <-ctx.Done():
				return sequence, errors.New(ReasonRecoveryObservationUnavailable)
			case <-ticker.C:
			}
		}
	})
	return sequence, recovery, err
}
func (c *Controller) Jog(f Fence, mask uint32, velocity []float64, timeout time.Duration) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		c.mu.Lock()
		playing := c.execution.State == "executing"
		c.mu.Unlock()
		if playing {
			return 0, errors.New(ReasonModeConflict)
		}
		seq, err := c.core.Jog(ctx, mask, velocity, timeout)
		if err == nil {
			_, err = c.core.WaitResult(ctx, seq)
		}
		return seq, err
	})
}
func (c *Controller) Prepare(f Fence, mask uint32, points []ipcclient.Point, identity ...Identity) (uint64, error) {
	release, err := c.admitPreparation(f, false)
	if err != nil {
		return 0, err
	}
	defer release()
	return c.preparationOperation(context.Background(), f, func(ctx context.Context) (uint64, error) {
		return c.prepare(f, ctx, mask, points, identity...)
	})
}

// prepare runs under operationMu. Retain the attempted identity even if its
// acknowledgement is lost, so a cancelled caller cannot strand a prepared slot.
func (c *Controller) prepare(f Fence, ctx context.Context, mask uint32, points []ipcclient.Point, identity ...Identity) (uint64, error) {
	c.nextID++
	id := c.nextID
	handle, err := c.core.PrepareTrajectory(ctx, id, mask, points)
	if err != nil {
		cleanupErr := c.discardPreparation(id)
		var rejected *ipcclient.RejectedError
		invalidated := errors.As(err, &rejected) && rejected.Result.Reason == ipcclient.ReasonInvalidTrajectory
		if errors.Is(err, ipcclient.ErrRevoked) || c.core.Err() != nil || invalidated || cleanupErr != nil {
			c.mu.Lock()
			previous := c.program
			c.retireNativeLossLocked(err)
			c.mu.Unlock()
			c.forgetProgram(previous, err)
		}
		if cleanupErr != nil {
			err = errors.Join(err, cleanupErr)
		}
		return 0, publicIOError(err)
	}
	owned := cloneTrajectoryPoints(points)
	c.mu.Lock()
	err = c.authorize(f)
	if err == nil {
		if c.handles == nil {
			c.handles = make(map[uint64]*preparation)
		}
		var immutable Identity
		if len(identity) != 0 {
			immutable = identity[0]
		}
		h := &preparation{HandleRecord: HandleRecord{Handle: handle, Generation: f.Generation, Identity: immutable}, mask: mask, points: owned, owner: c}
		transitionHandle(h, HandlePrepared)
		c.handles[handle] = h
		c.pruneTerminalHandlesLocked()
	}
	c.mu.Unlock()
	if err != nil {
		return 0, c.cleanupPreparation(handle, err)
	}
	return handle, nil
}

// Cleanup must outlive the revoked application context. The bounded native
// result establishes reclamation; an unavailable transport remains an explicit
// error and is never reported as a successful rollback.
func (c *Controller) cleanupPreparation(handle uint64, cause error) error {
	c.mu.Lock()
	if h := c.handles[handle]; h != nil {
		transitionHandle(h, HandleRetired)
	}
	c.mu.Unlock()
	if err := c.discardPreparation(handle); err != nil {
		return errors.Join(cause, err)
	}
	return cause
}

func (c *Controller) discardPreparation(handle uint64) error {
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if _, err := c.core.DiscardTrajectory(ctx, handle); err != nil {
		return fmt.Errorf("discard preparation %d: %w", handle, err)
	}
	return nil
}

func (c *Controller) Discard(f Fence, handle uint64) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		motion, _ := c.core.Motion()
		c.mu.Lock()
		c.observeHandlesLocked(motion)
		h := c.handles[handle]
		if h == nil {
			c.mu.Unlock()
			return 0, errors.New(ReasonUnknownHandle)
		}
		active := h.State == HandleStarted
		c.mu.Unlock()
		sequence, err := c.core.DiscardTrajectory(ctx, handle)
		if active {
			if err == nil {
				c.mu.Lock()
				transitionHandle(h, HandleConsumed)
				c.mu.Unlock()
				return sequence, errors.New(ReasonHandleActive)
			}
			var rejected *ipcclient.RejectedError
			if errors.As(err, &rejected) && rejected.Result.Reason == ipcclient.ReasonModeConflict {
				return sequence, &publicRejection{ReasonHandleActive, err}
			}
		}
		if err != nil {
			var rejected *ipcclient.RejectedError
			if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonModeConflict {
				c.mu.Lock()
				transitionHandle(h, HandleRetired)
				c.retireNativeLossLocked(err)
				program := c.program
				c.mu.Unlock()
				if program != nil && program.handle == handle {
					c.forgetProgram(program, err)
				}
			}
			return sequence, err
		}
		c.mu.Lock()
		defer c.mu.Unlock()
		transitionHandle(h, HandleDiscarded)
		if c.program != nil && c.program.handle == handle {
			c.program = nil
			c.execution.State = "discarded"
		}
		return sequence, c.authorize(f)
	})
}
func (c *Controller) Start(f Fence, handle uint64, identity ...Identity) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		return c.startHandle(ctx, f, handle, identity...)
	})
}

type stopFlight struct {
	done    chan struct{}
	fence   Fence
	release bool
	grant   Grant
	err     error
}

// Stop first fences public motion, then inhibits native outputs, drains writers
// and inhibits again. Renew may keep the session alive throughout this ordering.
// Release arriving during Stop joins that flight: inhibition completes before
// the session is released. Concurrent Stops share one result and never reacquire
// permission over a pending Release. Completion does not assert standstill.
func (c *Controller) Stop(f Fence, release bool) (Grant, error) {
	return c.stopGrant(f, release, false)
}

// An automatic expiry handoff may retire only its captured expired session;
// a concurrent Acquire must never let it inhibit a newly issued successor.
func (c *Controller) stopGrant(f Fence, release, expiredOnly bool) (Grant, error) {
	c.mu.Lock()
	if expiredOnly && (f.Session != c.grant.Session || time.Now().Before(c.expires)) &&
		(c.stopFlight == nil || c.stopFlight.fence.Session != f.Session) {
		c.mu.Unlock()
		return Grant{}, nil
	}
	// Stop authenticates session identity independently of motion authority.
	// Expired/revoked grants cannot move, but their known session can inhibit.
	// Restart fencing still rejects identities from another daemon incarnation.
	if err := c.authorizeStop(f); err != nil {
		c.mu.Unlock()
		return Grant{}, err
	}
	if flight := c.stopFlight; c.stopping && flight != nil && f.Session == flight.fence.Session &&
		(f.Generation == flight.fence.Generation || f.Generation == c.grant.Generation) {
		flight.release = flight.release || release
		c.mu.Unlock()
		<-flight.done
		return flight.grant, flight.err
	}
	// A recovery Stop must not reacquire expired or revoked motion authority.
	release = release || c.authorize(f) != nil
	c.stopping = true
	c.stopFlight = &stopFlight{done: make(chan struct{}), fence: f, release: release}
	c.cancel()
	c.retireHandlesLocked()
	c.program = nil
	c.localJog = nil
	c.grant.Generation++
	c.execution.State = "cancelled"
	if expiredOnly {
		c.grantEventLocked("grant_expired", true)
	} else if release {
		c.grantEventLocked("grant_released", true)
	} else {
		c.grantEventLocked("grant_revoked", true)
	}
	c.mu.Unlock()
	return c.finishStop()
}

// authorizeStop accepts only a previously issued session and a nonfuture
// generation; it never grants authority or accepts anonymous cancellation.
func (c *Controller) authorizeStop(f Fence) error {
	if err := c.restartFenceLocked(f); err != nil {
		return err
	}
	session, known := c.sessions[f.Session]
	valid := known && session.incarnation == c.daemonIncarnation ||
		f.Session == c.grant.Session || f.Session == c.expiredSession
	if f.Session == "" || !valid || f.Generation == 0 || f.Generation > c.grant.Generation {
		return errors.New(ReasonControlSessionStale)
	}
	return nil
}
func (c *Controller) finishStop() (result Grant, resultErr error) {
	c.mu.Lock()
	flight := c.stopFlight
	c.mu.Unlock()
	defer func() {
		c.mu.Lock()
		if flight != nil {
			flight.grant, flight.err = result, resultErr
			close(flight.done)
		}
		if c.stopFlight == flight {
			c.stopFlight = nil
		}
		c.mu.Unlock()
	}()
	_, err := c.core.Stop(context.Background()) // Bypasses operation/bulk locks.
	c.operationMu.Lock()
	_, fenceErr := c.core.Stop(context.Background())
	c.operationMu.Unlock()
	if err == nil {
		err = fenceErr
	}
	c.authorityMu.Lock()
	defer c.authorityMu.Unlock()
	c.mu.Lock()
	if c.grant.Session != "" && !time.Now().Before(c.expires) {
		c.expiredSession = c.grant.Session
		c.grant.Session = ""
	}
	if flight != nil && flight.release {
		c.grant.Session = ""
	}
	if err != nil {
		c.grant.Session = ""
		c.execution = Execution{State: "faulted", Error: err.Error()}
	}
	select {
	case <-c.done:
		c.mu.Unlock()
		return Grant{}, ipcclient.ErrClosed
	default:
	}
	if c.grant.Session == "" {
		c.requests = requestTable{}
		c.stopping = false
		grant := c.grant
		c.mu.Unlock()
		return grant, err
	}
	{
		c.sessionCtx, c.cancel = context.WithCancel(context.Background())
		deadlineNS := c.grant.DeadlineHostNS
		identity := c.nativeGrantIdentity()
		fence := Fence{c.grant.Session, c.grant.Generation}
		ctx, cancel := context.WithDeadline(c.sessionCtx, c.expires)
		c.mu.Unlock()
		_, mirrorErr := c.core.MirrorAcquire(ctx, identity, deadlineNS)
		cancel()
		c.mu.Lock()
		// Keepalive during reacquisition can extend the local deadline. Mirror
		// that extension before reporting a usable grant to prevent early expiry.
		for mirrorErr == nil && c.grant.Session == fence.Session && deadlineNS != c.grant.DeadlineHostNS {
			deadlineNS = c.grant.DeadlineHostNS
			ctx, cancel = context.WithDeadline(c.sessionCtx, c.expires)
			c.mu.Unlock()
			_, mirrorErr = c.core.MirrorRenew(ctx, identity, deadlineNS)
			cancel()
			c.mu.Lock()
		}
		if c.grant.Session != fence.Session || c.grant.Generation != fence.Generation {
			expired := c.expiredSession == fence.Session
			c.mu.Unlock()
			_, _ = c.core.Stop(context.Background())
			c.mu.Lock()
			select {
			case <-c.done:
			default:
				c.stopping = false
			}
			c.mu.Unlock()
			if expired {
				return Grant{}, errors.New(ReasonExpired)
			}
			return Grant{}, errors.New(ReasonControlSessionStale)
		}
		if mirrorErr == nil && !time.Now().Before(c.expires) {
			mirrorErr = context.DeadlineExceeded
		}
		if mirrorErr != nil {
			c.stopping = false
			c.mu.Unlock()
			return Grant{}, c.failAuthority(fence, mirrorErr)
		}
	}
	if flight != nil && flight.release {
		c.cancel()
		c.mu.Unlock()
		_, err = c.core.Stop(context.Background())
		c.mu.Lock()
		c.grant.Session = ""
		c.requests = requestTable{}
	}
	c.stopping = false
	grant := c.grant
	c.mu.Unlock()
	return grant, err
}
func (c *Controller) watch() {
	defer c.wg.Done()
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		select {
		case <-c.done:
			return
		case <-ticker.C:
			c.mu.Lock()
			if state, err := c.core.Status(); err == nil {
				c.observeEventsLocked(state)
				if motion, err := c.core.Motion(); err == nil {
					c.observeHandlesLocked(motion)
				}
			}
			expired := c.grant.Session != "" && !time.Now().Before(c.expires)
			lost := c.core.Err()
			needsStop := (expired || lost != nil) && c.grant.Session != "" && !c.stopping
			if expired || (lost != nil && c.grant.Session != "") {
				if expired {
					c.grantEventLocked("grant_expired", true)
				} else {
					c.grantEventLocked("grant_revoked", true)
				}
				if expired {
					c.expiredSession = c.grant.Session
				}
				c.requests = requestTable{}
				c.retireHandlesLocked()
				if needsStop {
					c.stopFlight = &stopFlight{done: make(chan struct{}), fence: Fence{c.grant.Session, c.grant.Generation}}
				}
				c.stopping = true
				c.cancel()
				c.grant.Generation++
				c.grant.Session = ""
				c.program = nil
				c.localJog = nil
				c.execution.State = "cancelled"
				if lost != nil {
					c.execution = Execution{State: "faulted", Error: lost.Error()}
				}
			}
			c.mu.Unlock()
			if needsStop {
				_, _ = c.finishStop()
			}
			if lost != nil {
				c.reconnectLostNative()
			}
		}
	}
}
func (c *Controller) Close() error {
	c.closeOnce.Do(func() {
		close(c.done)
		c.mu.Lock()
		c.cancel()
		c.retireHandlesLocked()
		c.requests = requestTable{}
		c.grant.Session = ""
		c.localJog = nil
		c.stopping = true
		c.mu.Unlock()
		_, _ = c.core.Stop(context.Background())
		c.wg.Wait()
		c.mu.Lock()
		ownsReconnect := c.reconnect != nil
		c.mu.Unlock()
		// A caller-supplied client stays observable after Controller.Close.
		// Explicit reconnect mode transfers transport cleanup to the controller.
		if c.native != nil && ownsReconnect {
			_ = c.native.current().Close()
		}
	})
	return nil
}

func (c *Controller) PrepareProgram(f Fence, blob []byte) (*Program, error) {
	release, err := c.admitPreparation(f, true)
	if err != nil {
		return nil, err
	}
	defer release()
	return c.prepareProgramRequest(context.Background(), f, blob)
}

func (c *Controller) prepareProgramRequest(request context.Context, f Fence, blob []byte) (*Program, error) {
	c.mu.Lock()
	err := c.authorize(f)
	playing := c.execution.State == "executing"
	c.mu.Unlock()
	if err != nil {
		return nil, err
	}
	if playing {
		return nil, errors.New(ReasonProgramAlreadyExecuting)
	}
	program, err := c.decodeProgram(blob)
	if err != nil {
		return nil, err
	}
	if program.Identity.ManifestRevision != c.binding.Revision {
		return nil, errors.New(ReasonManifestRevisionMismatch)
	}
	c.mu.Lock()
	ioDescription := append([]byte(nil), c.description.IO...)
	c.mu.Unlock()
	var ioQualification struct {
		Qualified bool `json:"torch_qualified"`
	}
	_ = json.Unmarshal(ioDescription, &ioQualification)
	program.torchQualified = ioQualification.Qualified
	if program.torch && !program.torchQualified {
		return nil, errors.New(ReasonProcessIoExecutorNotQualified)
	}
	if err := c.bindIOMarkers(program); err != nil {
		return nil, err
	}
	// Decode is intentionally outside operationMu. From native preparation to
	// publication, one lifecycle transaction excludes Start and replacement;
	// urgent Stop and authority renewal still take only the short state lock.
	_, err = c.preparationOperation(request, f, func(ctx context.Context) (uint64, error) {
		c.mu.Lock()
		previous := c.program
		playing := c.execution.State == "executing"
		c.mu.Unlock()
		if playing {
			return 0, errors.New(ReasonProgramAlreadyExecuting)
		}
		handle, err := c.prepare(f, ctx, program.AxisMask, program.points, program.Identity)
		if err != nil {
			// Capacity is a classified admission refusal; retain its native
			// receipt while leaving unclassified errors on the existing path.
			var rejected *ipcclient.RejectedError
			if errors.As(err, &rejected) && err == rejected && rejected.Result.Reason == ipcclient.ReasonCapacity {
				err = fmt.Errorf("%s: %w", ReasonBusy, err)
			}
			return 0, err // Healthy admission failure preserves the prior handle.
		}
		if ctx.Err() != nil {
			return 0, c.cleanupPreparation(handle, ctx.Err())
		}
		if previous != nil {
			if _, err := c.core.DiscardTrajectory(ctx, previous.handle); err != nil {
				var rejected *ipcclient.RejectedError
				if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonModeConflict {
					// Only InUse proves that Discard left the old handle intact.
					// Lost replies and invalid envelopes leave its status unknown.
					c.forgetProgram(previous, err)
				}
				return 0, c.cleanupPreparation(handle, err)
			}
		}
		c.mu.Lock()
		if previous != nil {
			if h := c.handles[previous.handle]; h != nil {
				transitionHandle(h, HandleSuperseded)
			}
		}
		err = c.authorize(f)
		if err == nil {
			err = ctx.Err()
		}
		if err == nil {
			program.handle = handle
			program.points = nil // Native owns the immutable execution buffer.
			c.program = program
			c.execution = Execution{State: "prepared", Identity: program.Identity}
		}
		c.mu.Unlock()
		if err != nil {
			c.forgetProgram(previous, err)
			return 0, c.cleanupPreparation(handle, err)
		}
		return handle, nil
	})
	if err != nil {
		return nil, err
	}
	return program, nil
}

func (c *Controller) forgetProgram(program *Program, cause error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if program != nil {
		if h := c.handles[program.handle]; h != nil {
			transitionHandle(h, HandleRetired)
		}
	}
	c.retireNativeLossLocked(cause)
	if program != nil && c.program == program {
		c.program = nil
		c.execution.State = "faulted"
		c.execution.Error = cause.Error()
	}
}
func (c *Controller) StartProgram(f Fence, identity Identity) (uint64, error) {
	return c.operation(f, func(ctx context.Context) (uint64, error) {
		c.mu.Lock()
		program := c.program
		if program == nil || program.Identity != identity {
			c.mu.Unlock()
			return 0, errors.New(ReasonProgramIdentityMismatch)
		}
		if c.execution.State == "executing" {
			c.mu.Unlock()
			return 0, errors.New(ReasonProgramAlreadyExecuting)
		}
		if program.torch && !program.torchQualified {
			c.mu.Unlock()
			return 0, errors.New(ReasonProcessIoExecutorNotQualified)
		}
		c.mu.Unlock()
		sequence, err := c.startHandle(ctx, f, program.handle, identity)
		if err != nil {
			return sequence, publicIOError(err)
		}
		c.mu.Lock()
		if err := c.authorize(f); err != nil {
			c.mu.Unlock()
			return sequence, err
		}
		c.execution = Execution{State: "executing", Identity: identity, NativeSequence: sequence}
		c.wg.Add(1)
		c.mu.Unlock()
		go c.observeProgram(ctx, program, sequence)
		return sequence, nil
	})
}
func (c *Controller) observeProgram(ctx context.Context, program *Program, sequence uint64) {
	defer c.wg.Done()
	for {
		select {
		case <-ctx.Done():
			return
		case <-time.After(10 * time.Millisecond):
		}
		motion, err := c.core.Motion()
		c.mu.Lock()
		if ctx.Err() != nil || c.program != program || c.execution.State != "executing" {
			c.mu.Unlock()
			return
		}
		if err != nil {
			c.execution.State = "faulted"
			c.execution.Error = err.Error()
			c.mu.Unlock()
			_, _ = c.core.Stop(context.Background())
			return
		}
		if motion.CommandSequence == sequence {
			for segment, start := range program.boundaries {
				if motion.PointIndex >= uint32(start) {
					c.execution.Segment = segment
				}
			}
			if motion.Done {
				if motion.State == 4 {
					c.execution.State = "completed"
				} else {
					c.execution.State = "faulted"
					c.execution.Error = "native_execution_failed"
				}
				c.mu.Unlock()
				return
			}
		}
		c.mu.Unlock()
	}
}
