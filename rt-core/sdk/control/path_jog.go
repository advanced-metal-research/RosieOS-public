//go:build linux

package control

import (
	"context"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"net"
	"path/filepath"
	"sync"
	"time"

	"golang.org/x/sys/unix"
	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
)

// PathJogCapability is Describe's path_jog: present only when the cell's
// machine configures jog_jerk. Absent means path holds must be refused.
type PathJogCapability = ipcclient.PathJogCapability

// PathJogStatus is GET /v1/jog/path: the native path observation plus the
// adapter's chunk handshake state.
type PathJogStatus = adapter.PathJogStatus

// PathJogObservation is the native path state of one session, read from the
// fast-control region by the adapter (never the 10 Hz status file). Path
// quantities are in the session's path units (spacing per sample).
type PathJogObservation struct {
	Generation                                           uint64
	Open, Stopping, Ended, HasSpeed, Fault               bool
	PathS, PathSpeed, PathAcceleration                   float64
	ConsumedIndex, PlayableEndIndex, Received, Underruns uint64
	SpeedSequence, SpeedDeadlineNS, NowNS                uint64
	Governing, StopReason, EndReason                     uint32
}

// PathJogRejection is a refused chunk, speed or session. NativeReason is the
// protocol/control.json jog reason when native or the adapter named one.
type PathJogRejection struct {
	Reason       string
	NativeReason uint32
}

func (r *PathJogRejection) Error() string { return r.Reason }

// Transient reports a refusal that judged nothing (JogRejection.Transient);
// the session resends the speed input and stays open.
func (r *PathJogRejection) Transient() bool { return jogTransientReason(r.Reason) }

var pathReasonNames = map[uint32]string{
	ipcclient.JogReasonClosed: "jog_closed", ipcclient.JogReasonModeConflict: "mode_conflict",
	ipcclient.JogReasonWrongIdentity: "jog_session_stale", ipcclient.JogReasonStaleSequence: "jog_session_or_sequence_stale",
	ipcclient.JogReasonExpired: adapter.ReasonJogInputDeadlineExpired, ipcclient.JogReasonAuthorityLost: adapter.ReasonControlSessionStale,
	ipcclient.JogReasonPathStartMismatch: "path_start_mismatch", ipcclient.JogReasonPathCapacity: "path_capacity",
	ipcclient.JogReasonPathOutOfOrder: "path_out_of_order", ipcclient.JogReasonPathInvalidSample: "path_invalid_sample",
	ipcclient.JogReasonPathEnded: "path_ended", ipcclient.JogReasonPathFault: "path_fault",
	ipcclient.JogReasonPathUnavailable: adapter.ReasonPathJogUnavailable, ipcclient.JogReasonPathInvalidConfig: "path_invalid_config",
}

func pathRejection(reason uint32) *PathJogRejection {
	name, ok := pathReasonNames[reason]
	if !ok {
		name = fmt.Sprintf("jog_native_rejected_%d", reason)
	}
	return &PathJogRejection{Reason: name, NativeReason: reason}
}

// Unverified producer window: 32 chunks (512 samples) sent ahead of native
// acknowledgement, half the adapter's 64-chunk queue, so a well-behaved
// producer never meets path_queue_full.
const pathJogWindow = 32

// Unverified local acknowledgement poll period: 1 ms, one native cycle.
const pathJogPoll = time.Millisecond

// PathJogSession streams one joint-space path to the native path jog lane
// and commands its single path speed. Geometry (Append) carries no time;
// every speed command carries the producer's capture time as origin and
// origin + lifetime as its immutable deadline, so a stalled producer stops
// along the path natively. A Unix client uses jog.sock datagrams and polls
// GET /v1/jog/path for acknowledgements; a TLS client uses the WSS jog stream
// and its path_ack replies.
type PathJogSession struct {
	client     *Client
	fence      Fence
	generation uint64
	mask       uint32
	axes       int
	maxChunk   int
	spacing    float64
	maxSpeed   float64
	lifetime   uint64
	sourceNow  func() uint64
	// opened is the session's earliest admissible origin on the source clock:
	// the begin_path_jog origin locally, the WSS Begin sample remotely. Native
	// (begin_origin_ns) and the WSS receiver (jog_clock_moved_backwards)
	// refuse any speed origin before it.
	opened uint64

	appendMu sync.Mutex // One Append at a time: chunks are ordered and lossless.
	chunk    uint64     // Last chunk_sequence sent.

	speedMu       sync.Mutex
	speedSequence uint64
	lastSpeedNS   uint64
	// Origin and deadline of the last frame sent (after the opening floor).
	lastOrigin, lastDeadline uint64

	// Local transport.
	conn  *net.UnixConn
	clock string

	// Remote transport.
	remote    *RemoteJogSession
	acks      chan pathAckMessage
	terminal  chan error
	closeOnce sync.Once

	endMu sync.Mutex
	ended bool
}

type pathAckMessage struct {
	Type   string `json:"type"`
	Seq    uint64 `json:"seq"`
	Reason uint32 `json:"reason"`
}

// NewPathJogSession opens a path jog session: local when the client has no
// remote address, otherwise over WSS. sourceNow is the producer's monotonic
// clock (ipcclient.HostMonotonicNS locally; nil selects it); lifetimeNS is
// the input lifetime of every speed command and must not exceed the cell's
// max_jog_input_age_ns. spacing (path units per sample) and maxSpeed (path
// units/s) are fixed for the session. Describe must advertise path_jog.
func (c *Client) NewPathJogSession(ctx context.Context, mask uint32, spacing, maxSpeed float64, lifetimeNS uint64, sourceNow func() uint64) (*PathJogSession, error) {
	if sourceNow == nil {
		sourceNow = ipcclient.HostMonotonicNS
	}
	if mask == 0 || math.IsNaN(spacing) || math.IsInf(spacing, 0) || !(spacing > 0) ||
		math.IsNaN(maxSpeed) || math.IsInf(maxSpeed, 0) || !(maxSpeed > 0) || lifetimeNS == 0 {
		return nil, errors.New("invalid path jog session")
	}
	description, err := c.Describe(ctx)
	if err != nil {
		return nil, err
	}
	if description.PathJog == nil || description.PathJog.MaxChunk == 0 {
		return nil, &PathJogRejection{Reason: adapter.ReasonPathJogUnavailable, NativeReason: ipcclient.JogReasonPathUnavailable}
	}
	if mask>>len(description.Axes) != 0 {
		return nil, errors.New("invalid path jog mask")
	}
	age := description.MaxJogInputAgeNS
	if age == 0 {
		age = ipcclient.FastControlMaxJogInputAgeNs
	}
	if lifetimeNS > age {
		return nil, &PathJogRejection{Reason: "jog_input_too_old"}
	}
	s := &PathJogSession{client: c, mask: mask, axes: len(description.Axes), spacing: spacing, maxSpeed: maxSpeed,
		lifetime: lifetimeNS, sourceNow: sourceNow, maxChunk: int(min(description.PathJog.MaxChunk, ipcclient.PATH_JOG_MAX_CHUNK))}
	if c.unixPath == "" {
		remote, err := c.openRemoteJog(ctx, mask, lifetimeNS, sourceNow, true)
		if err != nil {
			return nil, err
		}
		s.remote, s.fence, s.generation, s.opened = remote, remote.fence, remote.generation, remote.begun
		s.acks = make(chan pathAckMessage, pathJogWindow*2)
		s.terminal = make(chan error, 1)
		go s.readRemote()
		return s, nil
	}
	clock, err := c.JogClock(ctx)
	if err != nil {
		return nil, err
	}
	if clock.Domain != "CLOCK_MONOTONIC" || clock.MappingGeneration != 1 {
		return nil, errors.New("unqualified local jog clock")
	}
	conn, err := net.DialUnix("unixgram", nil, &net.UnixAddr{Name: filepath.Join(filepath.Dir(c.unixPath), "jog.sock"), Net: "unixgram"})
	if err != nil {
		return nil, errors.Join(ErrTransport, err)
	}
	origin := ipcclient.HostMonotonicNS()
	fence := c.Fence()
	response, err := c.Command(ctx, &Request{Operation: "begin_path_jog", Fence: fence, AxisMask: mask,
		ClockIncarnation: clock.Incarnation, SourceOriginHostNS: origin, DeadlineHostNS: origin + age})
	if err != nil {
		conn.Close()
		return nil, err
	}
	s.conn, s.clock, s.fence, s.generation, s.opened = conn, clock.Incarnation, fence, response.Handle, origin
	return s, nil
}

// Generation identifies the native jog generation of this path session.
func (s *PathJogSession) Generation() uint64 { return s.generation }

// TransientRefusals counts transient speed refusals answered by a resend
// (remote sessions only; the local datagram lane retries inside rt-control).
func (s *PathJogSession) TransientRefusals() uint64 {
	if s.remote == nil {
		return 0
	}
	return s.remote.TransientRefusals()
}

// Append streams samples first, first+1, ... in order. samples[i] has one
// value per described axis in logical joint units (only masked axes are
// used); caps, when non-nil, holds one per-sample speed cap in path units/s
// (+Inf for none). end marks the last sample with the application's reason.
// It splits into native chunks and returns when native accepted every chunk,
// or with *PathJogRejection naming the first refusal.
func (s *PathJogSession) Append(ctx context.Context, first uint64, samples [][]float64, caps []float64, end bool, endReason uint32) error {
	if caps != nil && len(caps) != len(samples) {
		return errors.New("invalid path caps")
	}
	if len(samples) == 0 && !end {
		return nil
	}
	s.appendMu.Lock()
	defer s.appendMu.Unlock()
	var frames []ipcclient.LocalPathSamples
	for at := 0; at < len(samples) || (end && at == 0 && len(samples) == 0); {
		count := min(s.maxChunk, len(samples)-at)
		token, err := hex.DecodeString(s.fence.Session)
		if err != nil || len(token) != 32 {
			return errors.New("invalid path session fence")
		}
		frame := ipcclient.LocalPathSamples{GrantGeneration: s.fence.Generation, JogGeneration: s.generation,
			ChunkSequence: s.chunk + uint64(len(frames)) + 1, FirstIndex: first + uint64(at), Spacing: s.spacing,
			MaxSpeed: s.maxSpeed, Count: uint32(count)}
		copy(frame.SessionId[:], token)
		for i := range frame.Caps {
			frame.Caps[i] = math.Inf(1)
		}
		for i := 0; i < count; i++ {
			row := samples[at+i]
			if len(row) != s.axes {
				return errors.New("invalid path sample width")
			}
			for axis, value := range row {
				if s.mask&(1<<axis) == 0 {
					continue
				}
				if math.IsNaN(value) || math.IsInf(value, 0) {
					return errors.New("invalid path sample")
				}
				frame.Samples[i*ipcclient.MaxAxes+axis] = value
			}
			if caps != nil {
				frame.Caps[i] = caps[at+i]
			}
		}
		at += count
		if end && at >= len(samples) {
			frame.Flags, frame.EndReason = 1, endReason
		}
		frames = append(frames, frame)
		if count == 0 {
			break
		}
	}
	if s.remote != nil {
		return s.appendRemote(ctx, frames)
	}
	return s.appendLocal(ctx, frames)
}

func (s *PathJogSession) appendLocal(ctx context.Context, frames []ipcclient.LocalPathSamples) error {
	accepted := s.chunk
	for i := 0; i < len(frames); {
		if frames[i].ChunkSequence-accepted <= pathJogWindow {
			encoded := ipcclient.EncodeLocalPathSamples(frames[i])
			raw, err := s.conn.SyscallConn()
			if err != nil {
				return errors.Join(ErrTransport, err)
			}
			var sendErr error
			err = raw.Control(func(fd uintptr) { sendErr = unix.Sendto(int(fd), encoded[:], unix.MSG_DONTWAIT, nil) })
			if err != nil {
				return errors.Join(ErrTransport, err)
			}
			if sendErr == nil {
				s.chunk = frames[i].ChunkSequence
				i++
				continue
			}
			if !errors.Is(sendErr, unix.EAGAIN) && !errors.Is(sendErr, unix.EWOULDBLOCK) {
				return errors.Join(ErrTransport, sendErr)
			}
		}
		var err error
		if accepted, err = s.awaitLocal(ctx, accepted, false); err != nil {
			return err
		}
	}
	_, err := s.awaitLocal(ctx, accepted, true)
	return err
}

// awaitLocal polls the adapter's handshake state until acknowledgement
// advances past accepted (or, with all, reaches the last chunk sent).
func (s *PathJogSession) awaitLocal(ctx context.Context, accepted uint64, all bool) (uint64, error) {
	for {
		status, err := get[PathJogStatus](s.client, ctx, "/v1/jog/path")
		if err != nil {
			return accepted, err
		}
		if status.Generation != s.generation {
			return accepted, &PathJogRejection{Reason: adapter.ReasonJogSessionStale, NativeReason: ipcclient.JogReasonWrongIdentity}
		}
		if status.Refused > accepted && status.Refused <= s.chunk && status.Reason != ipcclient.JogReasonNone {
			return accepted, pathRejection(status.Reason)
		}
		if status.Accepted > accepted {
			accepted = status.Accepted
		}
		if accepted >= s.chunk || (!all && s.chunk-accepted < pathJogWindow) {
			return accepted, nil
		}
		select {
		case <-ctx.Done():
			return accepted, ctx.Err()
		case <-time.After(pathJogPoll):
		}
	}
}

func (s *PathJogSession) appendRemote(ctx context.Context, frames []ipcclient.LocalPathSamples) error {
	accepted := s.chunk
	wait := func() error {
		select {
		case ack := <-s.acks:
			if ack.Reason != ipcclient.JogReasonNone {
				return pathRejection(ack.Reason)
			}
			if ack.Seq > accepted {
				accepted = ack.Seq
			}
			return nil
		case err := <-s.terminal:
			s.terminal <- err
			return err
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	for _, frame := range frames {
		for frame.ChunkSequence-accepted > pathJogWindow {
			if err := wait(); err != nil {
				return err
			}
		}
		encoded := ipcclient.EncodeLocalPathSamples(frame)
		s.remote.writeMu.Lock()
		if s.remote.ended {
			s.remote.writeMu.Unlock()
			return ErrClosed
		}
		err := s.remote.send(2, encoded[:])
		s.remote.writeMu.Unlock()
		if err != nil {
			return err
		}
		s.chunk = frame.ChunkSequence
	}
	for accepted < s.chunk {
		if err := wait(); err != nil {
			return err
		}
	}
	return nil
}

// readRemote is the stream's only reader: clock refresh replies, chunk
// acknowledgements, transient refusals (the speed input is resent and the
// hold continues) and the terminal refusal. On a Rosie 1430 arm at 2026-10-10 22:01:13, a
// publisher collision reported as jog_native_rejected_3 ended a held
// Cartesian jog here with 195 ms of input lifetime left.
func (s *PathJogSession) readRemote() {
	r := s.remote
	r.writeMu.Lock()
	r.readerActive = true
	r.writeMu.Unlock()
	for {
		body, err := r.frame()
		if err == nil {
			var ack pathAckMessage
			if json.Unmarshal(body, &ack) == nil && ack.Type == "path_ack" {
				s.acks <- ack
				continue
			}
			var reply remoteJogMessage
			if json.Unmarshal(body, &reply) != nil {
				err = ErrProtocol
			} else if reply.Type == "calibrated" {
				if err = r.acceptCalibration(reply); err == nil {
					continue
				}
			} else if reply.Type == "rejected" && reply.Reason != "" {
				rejection := jogRejection(reply.Reason, reply.Seq)
				if rejection.Transient() {
					if err = r.resendTransient(rejection); err == nil {
						continue
					}
				} else {
					err = &PathJogRejection{Reason: rejection.Reason, NativeReason: rejection.NativeReason}
				}
			} else {
				err = ErrProtocol
			}
		}
		r.conn.Close()
		s.terminal <- err
		return
	}
}

// ErrPathSpeedPredatesSession reports a speed command that was not sent: its
// input was captured before the session opened and either expired by then or
// a frame already carries an input from before the opening. It wraps
// ErrJogWouldBlock; the next fresh input is sent normally.
var ErrPathSpeedPredatesSession = fmt.Errorf("%w: input predates the path session", ErrJogWouldBlock)

// SetSpeed sends one path speed command (path units/s): origin = sourceNS on
// the producer clock, deadline = sourceNS + lifetime. It never blocks: a busy
// sender returns ErrJogWouldBlock. Speeds above the session ceiling apply at
// the ceiling natively. Append the first samples before the first SetSpeed.
//
// The input that opens a hold is captured before its setup (resolver, WSS
// calibration, Begin), so its sourceNS precedes the session's opening, which
// native and the WSS receiver refuse (a Rosie 1430 arm, 2026-10-07: every remote hold
// ended jog_clock_moved_backwards ~17 ms after a warm resolver). Such an
// input is sent with origin = the opening and its own deadline unchanged: it
// never gains lifetime, an input already expired at the opening is not sent,
// and only the first such frame is sent because native refuses an equal
// origin that extends the deadline.
func (s *PathJogSession) SetSpeed(speed float64, sourceNS uint64) error {
	if math.IsNaN(speed) || math.IsInf(speed, 0) || speed < 0 || sourceNS == 0 || sourceNS > ^uint64(0)-s.lifetime {
		return errors.New("invalid path speed")
	}
	if !s.speedMu.TryLock() {
		return ErrJogWouldBlock
	}
	defer s.speedMu.Unlock()
	if sourceNS < s.lastSpeedNS {
		return &PathJogRejection{Reason: "jog_invalid_timing", NativeReason: ipcclient.JogReasonInvalidTiming}
	}
	origin, deadline := max(sourceNS, s.opened), sourceNS+s.lifetime
	if deadline <= origin || (origin == s.lastOrigin && deadline > s.lastDeadline) {
		return ErrPathSpeedPredatesSession
	}
	if s.remote != nil {
		r := s.remote
		if !r.writeMu.TryLock() {
			return ErrJogWouldBlock
		}
		defer r.writeMu.Unlock()
		if r.ended {
			return ErrClosed
		}
		if s.client.Fence() != s.fence {
			return &PathJogRejection{Reason: adapter.ReasonControlSessionStale}
		}
		build := func(sequence uint64) ([ipcclient.LocalJogUpdateBytes]byte, error) {
			return buildPathSpeedFrame(s.fence, r.clock, s.generation, sequence, s.mask, s.axes, speed, origin, deadline)
		}
		frame, err := build(r.sequence + 1)
		if err != nil {
			return err
		}
		if err := r.sendInputLocked(frame, deadline, build); err != nil {
			return err
		}
		s.lastSpeedNS, s.lastOrigin, s.lastDeadline = sourceNS, origin, deadline
		return nil
	}
	frame, err := buildPathSpeedFrame(s.fence, s.clock, s.generation, s.speedSequence+1, s.mask, s.axes, speed, origin, deadline)
	if err != nil {
		return err
	}
	raw, err := s.conn.SyscallConn()
	if err != nil {
		return errors.Join(ErrTransport, err)
	}
	var sendErr error
	err = raw.Control(func(fd uintptr) { sendErr = unix.Sendto(int(fd), frame[:], unix.MSG_DONTWAIT, nil) })
	if errors.Is(sendErr, unix.EAGAIN) || errors.Is(sendErr, unix.EWOULDBLOCK) {
		return ErrJogWouldBlock
	}
	if err != nil || sendErr != nil {
		return errors.Join(ErrTransport, err, sendErr)
	}
	s.speedSequence++
	s.lastSpeedNS, s.lastOrigin, s.lastDeadline = sourceNS, origin, deadline
	return nil
}

// buildPathSpeedFrame encodes a local_jog_update with flags bit 0: velocity[0]
// is the path speed, every other entry zero, the session's whole axis mask.
func buildPathSpeedFrame(fence Fence, incarnation string, generation, sequence uint64, mask uint32, axes int, speed float64, originNS, deadlineNS uint64) ([ipcclient.LocalJogUpdateBytes]byte, error) {
	var frame ipcclient.LocalJogUpdate
	token, err := hex.DecodeString(fence.Session)
	clock, clockErr := hex.DecodeString(incarnation)
	if err != nil || clockErr != nil || len(token) != len(frame.SessionId) || len(clock) != len(frame.ClockIncarnation) ||
		fence.Generation == 0 || generation == 0 || sequence == 0 || mask == 0 || mask>>axes != 0 ||
		originNS == 0 || deadlineNS <= originNS || deadlineNS-originNS > 2000000000 {
		return [ipcclient.LocalJogUpdateBytes]byte{}, errors.New("invalid path speed frame")
	}
	copy(frame.SessionId[:], token)
	copy(frame.ClockIncarnation[:], clock)
	frame.GrantGeneration, frame.JogGeneration, frame.SourceSequence = fence.Generation, generation, sequence
	frame.OriginHostNs, frame.DeadlineHostNs, frame.AxisMask, frame.Flags = originNS, deadlineNS, mask, 1
	frame.Velocity[0] = speed
	return ipcclient.EncodeLocalJogUpdate(frame), nil
}

// Observe reads GET /v1/jog/path. A newer generation means this session was
// replaced and is reported as jog_session_stale. An older one is native's
// previous publication: the jog and path observations are separate slots, so
// right after Begin the path slot can still name the previous session for a
// cycle; Observe polls until it names this session.
func (s *PathJogSession) Observe(ctx context.Context) (PathJogObservation, error) {
	var o ipcclient.PathObservation
	for {
		status, err := get[PathJogStatus](s.client, ctx, "/v1/jog/path")
		if err != nil {
			return PathJogObservation{}, err
		}
		o = status.Observation
		if o.JogGeneration >= s.generation {
			break
		}
		select {
		case <-ctx.Done():
			return PathJogObservation{}, ctx.Err()
		case <-time.After(pathJogPoll):
		}
	}
	observation := PathJogObservation{Generation: o.JogGeneration, Open: o.Flags&1 != 0, Stopping: o.Flags&2 != 0,
		Ended: o.Flags&4 != 0, HasSpeed: o.Flags&8 != 0, Fault: o.Flags&16 != 0, PathS: o.PathS, PathSpeed: o.PathSpeed,
		PathAcceleration: o.PathAcceleration, ConsumedIndex: o.ConsumedIndex, PlayableEndIndex: o.PlayableEndIndex,
		Received: o.Received, Underruns: o.Underruns, SpeedSequence: o.SpeedSequence, SpeedDeadlineNS: o.SpeedDeadlineHostNs,
		NowNS: o.NowHostNs, Governing: o.Governing, StopReason: o.StopReason, EndReason: o.EndReason}
	if o.JogGeneration != s.generation {
		return observation, &PathJogRejection{Reason: adapter.ReasonJogSessionStale, NativeReason: ipcclient.JogReasonWrongIdentity}
	}
	return observation, nil
}

// Rejection returns when the session is terminated: remotely, the stream's
// refusal or failure; locally, a refused chunk or the native session closing
// (its PathJogObservation.StopReason tells release, expiry, Halt or fault).
func (s *PathJogSession) Rejection(ctx context.Context) error {
	if s.remote != nil {
		select {
		case err := <-s.terminal:
			s.terminal <- err
			return err
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	begun := false
	for {
		status, err := get[PathJogStatus](s.client, ctx, "/v1/jog/path")
		if err != nil {
			return err
		}
		if status.Generation != s.generation || status.Observation.JogGeneration > s.generation {
			return &PathJogRejection{Reason: adapter.ReasonJogSessionStale, NativeReason: ipcclient.JogReasonWrongIdentity}
		}
		if status.Refused != 0 && status.Reason != ipcclient.JogReasonNone {
			return pathRejection(status.Reason)
		}
		// An older native generation is the previous session's publication
		// (separate jog and path slots); wait for this session's.
		open := status.Observation.JogGeneration == s.generation && status.Observation.Flags&1 != 0
		if begun && !open {
			if status.Observation.Flags&16 != 0 {
				return pathRejection(ipcclient.JogReasonPathFault)
			}
			return &PathJogRejection{Reason: "path_closed"}
		}
		begun = begun || open
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(10 * pathJogPoll):
		}
	}
}

// End asks native to stop along the path (PathJog::end) and returns after
// native closed the session. The transport closes afterwards; a session
// that was already closed returns nil.
func (s *PathJogSession) End(ctx context.Context) error {
	s.endMu.Lock()
	defer s.endMu.Unlock()
	if s.ended {
		return nil
	}
	s.ended = true
	defer s.closeTransport()
	_, err := s.client.Command(ctx, &Request{Operation: "end_jog", Fence: s.fence, JogGeneration: s.generation})
	if err != nil {
		return err
	}
	for {
		status, err := get[PathJogStatus](s.client, ctx, "/v1/jog/path")
		if err != nil {
			return err
		}
		o := status.Observation
		if o.JogGeneration > s.generation || (o.JogGeneration == s.generation && o.Flags&1 == 0) {
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(pathJogPoll):
		}
	}
}

func (s *PathJogSession) closeTransport() {
	s.closeOnce.Do(func() {
		if s.remote != nil {
			s.remote.writeMu.Lock()
			s.remote.ended = true
			s.remote.writeMu.Unlock()
			s.remote.conn.Close()
		}
		if s.conn != nil {
			s.conn.Close()
		}
	})
}
