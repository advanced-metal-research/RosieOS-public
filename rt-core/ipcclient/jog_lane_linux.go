//go:build linux

package ipcclient

import (
	"context"
	"encoding/hex"
	"errors"
	"fmt"
	"math"
	"runtime"
	"time"
)

var ErrJogUnavailable = errors.New("RTCore jog observation unavailable")

// A jog update that meets another writer of this connection's fast-control
// region is not published and not judged: Begin/End, a path chunk, a grant
// request and a second jog producer each hold the publication lock for one
// shared-memory publish. Likewise native rewrites an observation slot once
// per cycle. These are the only transient refusals; every identity, sequence,
// timing, authority and readiness refusal is terminal for its input. On a Rosie
// 1430 arm at 2026-10-10 22:01:13, one such collision, reported as not_ready, ended a
// held Cartesian jog with 195 ms of input lifetime left.
var (
	ErrJogPublisherBusy  = errors.New("jog publisher busy; replace with newest input")
	ErrFastPublisherBusy = errors.New("fast publisher busy; replace with newest input")
	// ErrSharedSlotBusy wraps a read whose every bounded attempt met the
	// native writer mid-publication; never a stale or invalid observation.
	ErrSharedSlotBusy = errors.New("RTCore shared slot busy")
)

// TransientJogRefusal reports whether err refused a jog update only because a
// publication lock or observation slot was momentarily busy. The same frame
// may be published again before its immutable deadline; nothing was admitted.
func TransientJogRefusal(err error) bool {
	return errors.Is(err, ErrJogPublisherBusy) || errors.Is(err, ErrFastPublisherBusy) || errors.Is(err, ErrSharedSlotBusy)
}

type JogRejectedError struct {
	Reason      uint32
	Observation JogObservation
}

func (e *JogRejectedError) Error() string {
	return fmt.Sprintf("RTCore rejected jog (reason %d)", e.Reason)
}

// LocalJogSession is a native reservation bound to one connection, grant,
// configuration/Home epoch and the negotiated host CLOCK_MONOTONIC mapping.
// A new Begin is required after expiry or End. Renewal cannot extend an input.
type LocalJogSession struct{ Identity JogControl }

type LocalJogClock struct {
	Domain            string `json:"domain"`
	Incarnation       string `json:"incarnation"`
	MappingGeneration uint64 `json:"mapping_generation"`
	NowHostNS         uint64 `json:"now_host_ns"`
}

func (c *Client) LocalJogClock() (LocalJogClock, error) {
	c.fastMu.RLock()
	defer c.fastMu.RUnlock()
	if c.fast == nil {
		return LocalJogClock{}, ErrApplicationGrantUnavailable
	}
	select {
	case <-c.done:
		return LocalJogClock{}, ErrClosed
	default:
	}
	return LocalJogClock{"CLOCK_MONOTONIC", hex.EncodeToString(c.fast.header.DaemonIncarnation[:]), 1, monotonicNS()}, nil
}

func (c *Client) JogObservation() (JogObservation, error) {
	c.fastMu.RLock()
	defer c.fastMu.RUnlock()
	if c.fast == nil {
		return JogObservation{}, ErrApplicationGrantUnavailable
	}
	select {
	case <-c.done:
		return JogObservation{}, ErrClosed
	default:
	}
	var words, scratch [JogObservationBytes / 8]uint64
	_, status := c.fast.jogObservation.read(words[:], scratch[:])
	// A busy writer is not a native NotReady decision. Bound this non-RT
	// acquisition to three snapshots, yielding between collisions. Never use a
	// cached/torn sample; persistent contention and all invalid data still fail.
	for attempt := 1; status == sharedWordsBusy && attempt < 3; attempt++ {
		runtime.Gosched()
		_, status = c.fast.jogObservation.read(words[:], scratch[:])
	}
	if status == sharedWordsBusy {
		return JogObservation{}, fmt.Errorf("%w: %w", ErrJogUnavailable, ErrSharedSlotBusy)
	}
	if status != sharedWordsOK {
		return JogObservation{}, ErrJogUnavailable
	}
	var encoded [JogObservationBytes]byte
	if sharedWordsToLE(encoded[:], words[:]) != sharedWordsOK {
		return JogObservation{}, ErrJogUnavailable
	}
	observation, ok := DecodeJogObservation(encoded[:])
	now := monotonicNS()
	if !ok || observation.ConnectionId != c.fast.header.ConnectionId || observation.NativeGeneration == 0 ||
		observation.ConfigurationEpoch == 0 || observation.HomeEpoch == 0 || observation.ClockMappingGeneration != 1 ||
		observation.NowHostNs == 0 || observation.NowHostNs > now || now-observation.NowHostNs > applicationGrantObservationMaxAgeNS ||
		observation.ControlAccepted > 1 || observation.Open > 1 || observation.HasInput > 1 || observation.VelocityScalePpm > 1_000_000 ||
		observation.ControlReason > JogReasonPathInvalidConfig || observation.UpdateReason > JogReasonPathInvalidConfig ||
		observation.StateReason > JogReasonPathInvalidConfig || observation.AxisMask & ^c.fast.header.ConfiguredMask != 0 {
		return JogObservation{}, ErrJogUnavailable
	}
	return observation, nil
}

// BeginLocalJog reserves a mode without moving. The fixed origin fences capture
// and the deadline bounds first-input wait; separating them preserves remote
// clock uncertainty without moving an input's immutable timestamps.
func (c *Client) BeginLocalJog(ctx context.Context, identity GrantIdentity, mask uint32, origin, deadline uint64) (LocalJogSession, error) {
	return c.beginLocalJog(ctx, identity, mask, origin, deadline, JogOperationBegin)
}

func (c *Client) beginLocalJog(ctx context.Context, identity GrantIdentity, mask uint32, origin, deadline uint64, operation uint32) (LocalJogSession, error) {
	// Unverified absolute first-input bound: 2 s; native uses the compiled input-age ceiling.
	now := monotonicNS()
	if !c.validMask(mask) || origin == 0 || origin > now || now-origin >= 2000000000 || deadline <= now || deadline-now > 2000000000 {
		return LocalJogSession{}, errors.New("invalid jog Begin dimensions/deadline")
	}
	c.grantMu.Lock()
	grant := c.admittedGrant
	c.grantMu.Unlock()
	if grant.Identity != identity || grant.ConnectionID == 0 || grant.NativeGeneration == 0 ||
		grant.ConfigurationEpoch == 0 || grant.HomeEpoch == 0 || grant.DeadlineNS <= now {
		return LocalJogSession{}, ErrRevoked
	}
	if deadline > grant.DeadlineNS {
		deadline = grant.DeadlineNS
	}
	request := JogControl{ConnectionId: grant.ConnectionID, NativeGeneration: grant.NativeGeneration,
		CapabilityId: identity.CapabilityID, AppGrantGeneration: identity.Generation,
		ConfigurationEpoch: grant.ConfigurationEpoch, HomeEpoch: grant.HomeEpoch, ClockMappingGeneration: 1,
		OriginHostNs: origin, DeadlineHostNs: deadline, AxisMask: mask, Operation: operation}
	observation, err := c.jogControl(ctx, request, deadline)
	if err != nil {
		return LocalJogSession{}, err
	}
	request.JogGeneration = observation.JogGeneration
	return LocalJogSession{Identity: request}, nil
}

func (c *Client) EndLocalJog(ctx context.Context, session LocalJogSession) error {
	request := session.Identity
	request.Operation = JogOperationEnd
	request.AxisMask, request.OriginHostNs, request.DeadlineHostNs = 0, 0, 0
	_, err := c.jogControl(ctx, request, monotonicNS()+applicationGrantReceiptTimeoutNS)
	return err
}

func (c *Client) jogControl(ctx context.Context, request JogControl, deadline uint64) (JogObservation, error) {
	generation := c.revocation.Load()
	wait := func() error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if generation != c.revocation.Load() {
			return ErrRevoked
		}
		if monotonicNS() >= deadline {
			return context.DeadlineExceeded
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-c.done:
			return ErrClosed
		case <-time.After(time.Millisecond):
			return nil
		}
	}
	for !c.jogControlMu.TryLock() {
		if err := wait(); err != nil {
			return JogObservation{}, err
		}
	}
	defer c.jogControlMu.Unlock()
	if err := ctx.Err(); err != nil {
		return JogObservation{}, err
	}
	if monotonicNS() >= deadline {
		return JogObservation{}, context.DeadlineExceeded
	}
	if c.jogTicket == math.MaxUint64 {
		return JogObservation{}, errors.New("jog ticket exhausted")
	}
	c.jogTicket++
	request.Ticket = c.jogTicket
	encoded := EncodeJogControl(request)
	var words [JogControlBytes / 8]uint64
	if sharedWordsFromLE(words[:], encoded[:]) != sharedWordsOK {
		return JogObservation{}, ErrJogUnavailable
	}
	c.fastPublishMu.Lock()
	c.fastMu.RLock()
	status := sharedWordsUnbound
	if c.fast != nil && generation == c.revocation.Load() && request.ConnectionId == c.fast.header.ConnectionId {
		_, status = c.fast.jogControl.publish(words[:])
	}
	c.fastMu.RUnlock()
	c.fastPublishMu.Unlock()
	if status != sharedWordsOK {
		return JogObservation{}, ErrRevoked
	}
	for {
		observation, err := c.JogObservation()
		if err == nil && observation.Ticket == request.Ticket {
			if observation.NativeGeneration != request.NativeGeneration || observation.ConnectionId != request.ConnectionId ||
				observation.CapabilityId != request.CapabilityId || observation.AppGrantGeneration != request.AppGrantGeneration {
				return JogObservation{}, ErrRevoked
			}
			if observation.ControlAccepted == 0 {
				return observation, &JogRejectedError{Reason: observation.ControlReason, Observation: observation}
			}
			return observation, nil
		}
		if err := wait(); err != nil {
			// An unobserved Begin is not an explicit rejection. Urgently fence
			// it before returning, without taking the bulk command lock.
			return JogObservation{}, errors.Join(err, c.disarmUrgent())
		}
	}
}

// UpdateLocalJog publishes once to the latest-value slot. Return is publication,
// not admission; JogObservation reports the RT-consumed sequence. The caller's
// origin/deadline remain unchanged by scheduling, backpressure or grant renewal.
func (c *Client) UpdateLocalJog(ctx context.Context, session LocalJogSession, sequence, origin, deadline uint64, mask uint32, velocity []float64) error {
	return c.updateLocalJog(ctx, session, sequence, origin, deadline, mask, velocity, 0)
}

// flags 1 is a path speed command: velocity[0] only, any axis mask entry.
func (c *Client) updateLocalJog(ctx context.Context, session LocalJogSession, sequence, origin, deadline uint64, mask uint32, velocity []float64, flags uint32) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	// Unverified absolute input-age ceiling: 2 s; native enforces compiled cell policy.
	if !c.validMask(mask) || mask & ^session.Identity.AxisMask != 0 || len(velocity) != int(c.axes) || sequence == 0 ||
		origin == 0 || deadline <= origin || deadline-origin > 2000000000 || origin > monotonicNS() || monotonicNS() >= deadline {
		return errors.New("invalid or expired jog update")
	}
	id := session.Identity
	c.grantMu.Lock()
	grant := c.admittedGrant
	c.grantMu.Unlock()
	if grant.Identity.CapabilityID != id.CapabilityId || grant.Identity.Generation != id.AppGrantGeneration ||
		grant.ConnectionID != id.ConnectionId || grant.NativeGeneration != id.NativeGeneration ||
		grant.ConfigurationEpoch != id.ConfigurationEpoch || grant.HomeEpoch != id.HomeEpoch ||
		grant.DeadlineNS <= monotonicNS() {
		return ErrRevoked
	}
	if deadline > grant.DeadlineNS {
		deadline = grant.DeadlineNS
	}
	request := JogUpdate{ConnId: id.ConnectionId, NativeGeneration: id.NativeGeneration,
		CapabilityId: id.CapabilityId, AppGrantGeneration: id.AppGrantGeneration, ConfigEpoch: id.ConfigurationEpoch,
		HomeEpoch: id.HomeEpoch, JogGeneration: id.JogGeneration, SourceSequence: sequence,
		EarliestOriginHostNs: origin, ImmutableDeadlineHostNs: deadline, ClockMappingGeneration: id.ClockMappingGeneration, AxisMask: mask, Flags: flags}
	for axis, value := range velocity {
		unused := mask&(1<<axis) == 0
		if flags != 0 {
			unused = axis != 0
		}
		if math.IsNaN(value) || math.IsInf(value, 0) || (unused && value != 0) {
			return errors.New("invalid jog vector")
		}
		request.Velocity[axis] = value
	}
	encoded := EncodeJogUpdate(request)
	var words [JogUpdateBytes / 8]uint64
	if sharedWordsFromLE(words[:], encoded[:]) != sharedWordsOK {
		return ErrJogUnavailable
	}
	if !c.jogPublishMu.TryLock() {
		return ErrJogPublisherBusy
	}
	defer c.jogPublishMu.Unlock()
	if !c.fastPublishMu.TryLock() {
		return ErrFastPublisherBusy
	}
	defer c.fastPublishMu.Unlock()
	c.fastMu.RLock()
	defer c.fastMu.RUnlock()
	if c.fast == nil || c.fast.header.ConnectionId != id.ConnectionId {
		return ErrRevoked
	}
	select {
	case <-c.done:
		return ErrClosed
	default:
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if monotonicNS() >= deadline {
		return context.DeadlineExceeded
	}
	if _, status := c.fast.jogUpdate.publish(words[:]); status != sharedWordsOK {
		return ErrJogUnavailable
	}
	return nil
}
