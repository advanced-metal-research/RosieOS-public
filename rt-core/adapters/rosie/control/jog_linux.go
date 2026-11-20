//go:build linux

package control

import (
	"context"
	"encoding/hex"
	"errors"
	"golang.org/x/sys/unix"
	"math"
	"net"
	"os"
	"rosieos/rt-core/ipcclient"
)

type localJogNative interface {
	BeginLocalJog(context.Context, ipcclient.GrantIdentity, uint32, uint64, uint64) (ipcclient.LocalJogSession, error)
	UpdateLocalJog(context.Context, ipcclient.LocalJogSession, uint64, uint64, uint64, uint32, []float64) error
	EndLocalJog(context.Context, ipcclient.LocalJogSession) error
	JogObservation() (ipcclient.JogObservation, error)
	LocalJogClock() (ipcclient.LocalJogClock, error)
}

// BeginJog reserves native jog mode on the independent local lane. Times are
// from the negotiated same-host monotonic clock, fixed at the producer. A
// maintenance notice refuses it here, not only in the control dispatcher: the
// remote jog stream's "begin" message calls this directly.
func (c *Controller) BeginJog(f Fence, mask uint32, origin, deadline uint64) (uint64, error) {
	if err := c.checkMaintenance(OperationBeginJog); err != nil {
		return 0, err
	}
	c.operationMu.Lock()
	defer c.operationMu.Unlock()
	core, ok := c.core.(localJogNative)
	if !ok {
		return 0, errors.New("independent_jog_unavailable")
	}
	c.mu.Lock()
	if err := c.authorize(f); err != nil {
		c.mu.Unlock()
		return 0, err
	}
	identity, ctx := c.nativeGrantIdentity(), c.sessionCtx
	c.mu.Unlock()
	session, err := core.BeginLocalJog(ctx, identity, mask, origin, deadline)
	if err != nil {
		return 0, err
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := c.authorize(f); err != nil {
		return 0, err
	}
	c.retirePathLocked()
	c.localJog, c.jogSourceSequence = &session, 0
	return session.Identity.JogGeneration, nil
}

// UpdateJog records one ingress outcome, including failures before publication;
// internal transport callers use updateJog to avoid counting the same frame twice.
// A transient collision is retried within its window and the input's deadline.
func (c *Controller) UpdateJog(f Fence, generation, sequence, origin, deadline uint64, mask uint32, velocity []float64) error {
	var err error
	reason, _, attempts := retryTransientJog(deadline, func() (uint32, error) {
		err = c.updateJog(f, generation, sequence, origin, deadline, mask, velocity)
		return jogIngressReason(err), err
	})
	c.recordJogIngress(sequence, reason, jogRefusal{transport: "update_jog", generation: generation, deadline: deadline, cause: err, attempts: attempts})
	return err
}

func (c *Controller) updateJog(f Fence, generation, sequence, origin, deadline uint64, mask uint32, velocity []float64) error {
	return c.updateJogFlags(f, generation, sequence, origin, deadline, mask, velocity, 0)
}

// flags bit 0 publishes velocity[0] as the path speed of a path session.
func (c *Controller) updateJogFlags(f Fence, generation, sequence, origin, deadline uint64, mask uint32, velocity []float64, flags uint32) error {
	core, ok := c.core.(localJogNative)
	if !ok {
		return errors.New("independent_jog_unavailable")
	}
	paths, pathCapable := c.core.(pathJogNative)
	if flags != 0 && !pathCapable {
		return errors.New(ReasonPathJogUnavailable)
	}
	// Competing producers do not create a queue of stale input. Each sender
	// retains only its latest unsent value and may publish a newer frame.
	if !c.jogUpdateMu.TryLock() {
		return errors.New("jog_publisher_busy")
	}
	defer c.jogUpdateMu.Unlock()
	c.mu.Lock()
	if err := c.authorize(f); err != nil {
		c.mu.Unlock()
		return err
	}
	if c.localJog == nil || generation != c.localJog.Identity.JogGeneration || sequence == 0 || sequence <= c.jogSourceSequence {
		c.mu.Unlock()
		return errors.New("jog_session_or_sequence_stale")
	}
	session, ctx := *c.localJog, c.sessionCtx
	if err := c.currentJogSessionLocked(core); err != nil {
		c.mu.Unlock()
		return err
	}
	c.mu.Unlock()
	if flags != 0 {
		if err := paths.UpdateLocalPathSpeed(ctx, session, sequence, origin, deadline, velocity[0]); err != nil {
			return err
		}
	} else if err := core.UpdateLocalJog(ctx, session, sequence, origin, deadline, mask, velocity); err != nil {
		return err
	}
	c.mu.Lock()
	if c.localJog != nil && c.localJog.Identity == session.Identity {
		c.jogSourceSequence = sequence
	}
	c.mu.Unlock()
	return nil
}

// currentJogSessionLocked requires a current open native reservation with the
// same epochs. Source loss and epoch changes cannot revive an old producer even
// when it supplies fresh timestamps and increasing sequences. c.mu pins Begin.
// An unreadable observation returns its read error, which ingress maps to
// not_ready; only a busy slot (ipcclient.ErrSharedSlotBusy) is transient.
func (c *Controller) currentJogSessionLocked(core localJogNative) error {
	observation, err := core.JogObservation()
	if err != nil {
		return err
	}
	id := c.localJog.Identity
	if observation.Open != 1 || observation.InputDeadlineHostNs <= ipcclient.HostMonotonicNS() ||
		observation.ConnectionId != id.ConnectionId || observation.NativeGeneration != id.NativeGeneration ||
		observation.CapabilityId != id.CapabilityId || observation.AppGrantGeneration != id.AppGrantGeneration ||
		observation.ConfigurationEpoch != id.ConfigurationEpoch || observation.HomeEpoch != id.HomeEpoch ||
		observation.ClockMappingGeneration != id.ClockMappingGeneration || observation.JogGeneration != id.JogGeneration {
		return &ipcclient.JogRejectedError{Reason: ipcclient.JogReasonWrongIdentity, Observation: observation}
	}
	return nil
}

func (c *Controller) EndJog(f Fence, generation uint64) error {
	core, ok := c.core.(localJogNative)
	if !ok {
		return errors.New("independent_jog_unavailable")
	}
	c.mu.Lock()
	if err := c.authorize(f); err != nil {
		c.mu.Unlock()
		return err
	}
	if c.localJog == nil || c.localJog.Identity.JogGeneration != generation {
		c.mu.Unlock()
		return errors.New("jog_session_stale")
	}
	session, ctx := *c.localJog, c.sessionCtx
	c.localJog = nil // Revoke local input before waiting for native acknowledgement.
	c.retirePathLocked()
	c.mu.Unlock()
	return core.EndLocalJog(ctx, session)
}

func (c *Controller) JogStatus() (ipcclient.JogObservation, error) {
	core, ok := c.core.(localJogNative)
	if !ok {
		return ipcclient.JogObservation{}, errors.New("independent_jog_unavailable")
	}
	return core.JogObservation()
}

// maxJogInputAge reads native cell policy; omitted legacy fixtures retain LAN timing.
func (c *Controller) maxJogInputAge() uint64 {
	if c.description.MaxJogInputAgeNS != 0 {
		return c.description.MaxJogInputAgeNS
	}
	return ipcclient.FastControlMaxJogInputAgeNs
}

func (c *Controller) JogClock() (ipcclient.LocalJogClock, error) {
	core, ok := c.core.(localJogNative)
	if !ok {
		return ipcclient.LocalJogClock{}, errors.New("independent_jog_unavailable")
	}
	return core.LocalJogClock()
}

func (c *Controller) checkJogClock(incarnation string) error {
	clock, err := c.JogClock()
	if err != nil {
		return err
	}
	if incarnation == "" || incarnation != clock.Incarnation {
		return errors.New("jog_clock_incarnation_mismatch")
	}
	return nil
}

// jogIngressReason maps transport and native failures into the same native
// reason family. Unknown failures remain not_ready, never apparent acceptance.
func jogIngressReason(err error) uint32 {
	if err == nil {
		return ipcclient.JogReasonNone
	}
	var rejected *ipcclient.JogRejectedError
	if errors.As(err, &rejected) {
		if rejected.Reason == ipcclient.JogReasonNone {
			return ipcclient.JogReasonNotReady
		}
		return rejected.Reason
	}
	switch {
	case errors.Is(err, ErrJogClockIdentity), errors.Is(err, ErrJogClockBackward):
		return ipcclient.JogReasonClockReset
	case errors.Is(err, ErrJogClockGeneration):
		return ipcclient.JogReasonInvalidGeneration
	case errors.Is(err, ErrJogClockInputExpired), errors.Is(err, ErrJogClockInputStale):
		return ipcclient.JogReasonExpired
	case errors.Is(err, ErrJogClockInvalid), errors.Is(err, ErrJogClockArithmetic), errors.Is(err, ErrJogClockInputFuture):
		return ipcclient.JogReasonInvalidTiming
	case errors.Is(err, ipcclient.ErrRevoked), errors.Is(err, context.Canceled):
		return ipcclient.JogReasonAuthorityLost
	}
	switch err.Error() {
	case "jog_invalid_frame", "invalid jog vector":
		return ipcclient.JogReasonInvalidVector
	case "invalid or expired jog update":
		return ipcclient.JogReasonInvalidTiming
	case ReasonJogSessionStale, ReasonDaemonRestarted, ReasonSessionPrincipalMismatch, ReasonFence:
		return ipcclient.JogReasonWrongIdentity
	case ReasonJogSessionOrSequenceStale:
		return ipcclient.JogReasonStaleSequence
	case ReasonControlSessionStale:
		return ipcclient.JogReasonAuthorityLost
	case ReasonJogStreamIdle:
		return ipcclient.JogReasonExpired
	default:
		return ipcclient.JogReasonNotReady
	}
}

// recordJogIngress owns admission diagnostics for every transport. Refusal
// counts and the last refusal survive acceptance, grant renewal and reconnect;
// only a new adapter process resets them. No record claims RT application.
// Every refusal is also written to the journal with its full jog reason and
// what the transport knows (detail), within the journal budget.
func (c *Controller) recordJogIngress(sequence uint64, reason uint32, detail ...jogRefusal) uint32 {
	full := reason
	c.mu.Lock()
	if reason >= uint32(len(c.jogIngress.RefusedByReason)) {
		reason = ipcclient.JogReasonNotReady
	}
	o := &c.jogIngress
	o.SourceSequence, o.Reason, o.NowHostNS = sequence, reason, ipcclient.HostMonotonicNS()
	if reason == ipcclient.JogReasonNone {
		c.mu.Unlock()
		return reason
	}
	o.Refused++
	o.RefusedByReason[reason]++
	o.LatestRefusal = JogIngressRefusal{SourceSequence: sequence, Reason: reason, NowHostNS: o.NowHostNS}
	var d jogRefusal
	if len(detail) != 0 {
		d = detail[0]
	}
	jogGeneration := d.generation
	if jogGeneration == 0 && c.localJog != nil {
		jogGeneration = c.localJog.Identity.JogGeneration
	}
	grantGeneration, refused, now := c.grant.Generation, o.Refused, o.NowHostNS
	write, suppressed := c.admitJogRefusalLogLocked(now)
	c.mu.Unlock()
	if write {
		logJogRefusal(sequence, full, d, grantGeneration, jogGeneration, refused, suppressed, now)
	}
	return reason
}

// JogIngress returns a value snapshot of ingress diagnostics, preventing callers
// from mutating cumulative counters or losing a refusal when input is accepted.
func (c *Controller) JogIngress() JogIngressObservation {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.jogIngress
}

// ListenJog binds a protected datagram endpoint without removing an existing
// owner's socket. Possession of the current unguessable session token is required.
func ListenJog(path string) (*net.UnixConn, error) {
	conn, err := net.ListenUnixgram("unixgram", &net.UnixAddr{Name: path, Net: "unixgram"})
	if err != nil {
		return nil, err
	}
	if err = os.Chmod(path, 0660); err != nil {
		conn.Close()
		os.Remove(path)
		return nil, err
	}
	return conn, nil
}

// validateLocalJog admits only the negotiated host clock and the current
// session. It does not refresh either deadline or queue a rejected sequence.
func (c *Controller) validateLocalJog(frame ipcclient.LocalJogUpdate, negotiatedClock ...string) uint32 {
	reason, _ := c.validateLocalJogCause(frame, negotiatedClock...)
	return reason
}

// validateLocalJogCause is validateLocalJog that also returns the read error
// behind a reason, so a busy observation slot is told apart from not_ready.
func (c *Controller) validateLocalJogCause(frame ipcclient.LocalJogUpdate, negotiatedClock ...string) (uint32, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	var token [64]byte
	hex.Encode(token[:], frame.SessionId[:])
	if string(token[:]) != c.grant.Session {
		return ipcclient.JogReasonWrongIdentity, nil
	}
	if frame.GrantGeneration != c.grant.Generation {
		return ipcclient.JogReasonInvalidGeneration, nil
	}
	if c.localJog == nil {
		return ipcclient.JogReasonClosed, nil
	}
	if frame.JogGeneration != c.localJog.Identity.JogGeneration {
		return ipcclient.JogReasonInvalidGeneration, nil
	}
	if c.authorize(Fence{c.grant.Session, frame.GrantGeneration}) != nil {
		return ipcclient.JogReasonAuthorityLost, nil
	}
	var incarnation string
	if len(negotiatedClock) == 1 {
		incarnation = negotiatedClock[0] // Trusted connection negotiation, never a producer field.
	} else {
		clock, err := c.JogClock()
		if err != nil {
			return ipcclient.JogReasonClockReset, err
		}
		incarnation = clock.Incarnation
	}
	var encodedClock [32]byte
	hex.Encode(encodedClock[:], frame.ClockIncarnation[:])
	if string(encodedClock[:]) != incarnation {
		return ipcclient.JogReasonClockReset, nil
	}
	if frame.SourceSequence == 0 || frame.SourceSequence <= c.jogSourceSequence {
		return ipcclient.JogReasonStaleSequence, nil
	}
	core, ok := c.core.(localJogNative)
	if !ok {
		return ipcclient.JogReasonNotReady, nil
	}
	if err := c.currentJogSessionLocked(core); err != nil {
		return jogIngressReason(err), err
	}
	if frame.Flags & ^uint32(1) != 0 || frame.AxisMask == 0 || frame.AxisMask & ^c.localJog.Identity.AxisMask != 0 {
		return ipcclient.JogReasonInvalidVector, nil
	}
	// A path session takes only path speed (flags bit 0: velocity[0] in path
	// units/s, all else zero, the session's whole mask); a joint session only
	// velocity vectors. Native refuses the same mismatch with mode_conflict.
	if (frame.Flags != 0) != c.pathOpenLocked() {
		return ipcclient.JogReasonModeConflict, nil
	}
	if frame.Flags != 0 && (frame.AxisMask != c.localJog.Identity.AxisMask || frame.Velocity[0] < 0) {
		return ipcclient.JogReasonInvalidVector, nil
	}
	for axis, velocity := range frame.Velocity {
		unused := frame.AxisMask&(1<<axis) == 0
		if frame.Flags != 0 {
			unused = axis != 0
		}
		if math.IsNaN(velocity) || math.IsInf(velocity, 0) || (unused && velocity != 0) {
			return ipcclient.JogReasonInvalidVector, nil
		}
	}
	now := ipcclient.HostMonotonicNS()
	if frame.DeadlineHostNs <= now {
		return ipcclient.JogReasonExpired, nil
	}
	if frame.OriginHostNs == 0 || frame.OriginHostNs > now || frame.DeadlineHostNs-frame.OriginHostNs > c.maxJogInputAge() {
		return ipcclient.JogReasonInvalidTiming, nil
	}
	return ipcclient.JogReasonNone, nil
}

// ServeJog has one publication owner. Each bounded drain keeps only its newest
// valid frame; rejected or superseded datagrams cannot become delayed motion.
// Closing conn stops the listener. A listener failure revokes authority in main.
func (c *Controller) ServeJog(conn *net.UnixConn) error {
	raw, err := conn.SyscallConn()
	if err != nil {
		return err
	}
	// Unverified ingress work bound: at most 64 datagrams per drain.
	const drainLimit = 64
	// Large enough for either frame kind; the exact length selects it.
	var packet [ipcclient.LocalPathSamplesBytes]byte
	for {
		n, _, flags, _, err := conn.ReadMsgUnix(packet[:], nil)
		if err != nil {
			return err
		}
		var newest ipcclient.LocalJogUpdate
		have := false
		admit := func(n, flags int) {
			if n == ipcclient.LocalPathSamplesBytes && flags&unix.MSG_TRUNC == 0 {
				// Path chunks are lossless and ordered: each is queued, never
				// replaced by a newer one like a latest-value jog frame.
				if chunk, err := ipcclient.DecodeLocalPathSamplesFrame(packet[:n]); err == nil {
					c.recordPathIngress(chunk.ChunkSequence, c.appendLocalPath(chunk))
				} else {
					c.recordJogIngress(0, ipcclient.JogReasonPathInvalidSample)
				}
				return
			}
			frame, ok := ipcclient.DecodeLocalJogUpdate(packet[:n])
			reason := uint32(ipcclient.JogReasonInvalidVector)
			var cause error
			if ok && flags&unix.MSG_TRUNC == 0 {
				reason, cause = c.validateLocalJogCause(frame)
			}
			// A busy slot is not a verdict on the frame: publication validates
			// it again, retrying within its window and deadline.
			if reason == ipcclient.JogReasonNone || jogTransient(cause) {
				if !have || frame.SourceSequence > newest.SourceSequence {
					newest, have = frame, true
				}
			} else {
				c.recordJogIngress(frame.SourceSequence, reason, jogRefusal{transport: "datagram", generation: frame.JogGeneration, deadline: frame.DeadlineHostNs, cause: cause})
			}
		}
		admit(n, flags)
		var readErr error
		err = raw.Control(func(fd uintptr) {
			for i := 1; i < drainLimit; i++ {
				n, _, flags, _, readErr = unix.Recvmsg(int(fd), packet[:], nil, unix.MSG_DONTWAIT)
				if readErr == unix.EAGAIN || readErr == unix.EWOULDBLOCK {
					readErr = nil
					break
				}
				if readErr != nil {
					break
				}
				admit(n, flags)
			}
		})
		if err != nil {
			return err
		}
		if readErr != nil {
			return readErr
		}
		if have {
			c.publishLocalJog(&newest)
		}
	}
}

// publishLocalJog shares validation and native slot publication across ingress
// transports; no remote path can bypass local session, sequence or vector checks.
// A refusal caused only by a busy publication lock or observation slot is
// retried within jogTransientRetryNS and the frame's own deadline, rerunning
// every check; the returned cause tells a still-transient refusal apart. The
// returned reason is the full jog reason; ingress counters clamp theirs.
func (c *Controller) publishLocalJog(frame *ipcclient.LocalJogUpdate, negotiatedClock ...string) (uint32, error) {
	reason, cause, attempts := retryTransientJog(frame.DeadlineHostNs, func() (uint32, error) {
		return c.publishLocalJogOnce(frame, negotiatedClock...)
	})
	transport := "datagram"
	if len(negotiatedClock) == 1 {
		transport = "wss"
	}
	c.recordJogIngress(frame.SourceSequence, reason, jogRefusal{transport: transport, generation: frame.JogGeneration,
		deadline: frame.DeadlineHostNs, cause: cause, attempts: attempts})
	return reason, cause
}

func (c *Controller) publishLocalJogOnce(frame *ipcclient.LocalJogUpdate, negotiatedClock ...string) (uint32, error) {
	reason, cause := c.validateLocalJogCause(*frame, negotiatedClock...)
	if reason == ipcclient.JogReasonNone {
		c.mu.Lock()
		fence := Fence{c.grant.Session, frame.GrantGeneration}
		c.mu.Unlock()
		err := c.updateJogFlags(fence, frame.JogGeneration, frame.SourceSequence, frame.OriginHostNs,
			frame.DeadlineHostNs, frame.AxisMask, frame.Velocity[:len(c.description.Axes)], frame.Flags)
		if err != nil {
			reason = jogIngressReason(err)
		}
		cause = err
	}
	return reason, cause
}
