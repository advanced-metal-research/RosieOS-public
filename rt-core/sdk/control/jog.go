//go:build linux

package control

import (
	"context"
	"encoding/hex"
	"errors"
	"math"
	"net"
	"path/filepath"
	"sync"

	"golang.org/x/sys/unix"
	"rosieos/rt-core/ipcclient"
)

var ErrJogWouldBlock = errors.New("jog datagram not sent; replace with fresh input")

// BuildJogFrame encodes one generated local/WSS frame with caller-owned source
// times. It does not calibrate a remote clock or open a WSS producer connection.
// Invalid vectors, identities and timing are rejected before they can refresh jog.
func BuildJogFrame(fence Fence, incarnation string, generation, sequence uint64, mask uint32, velocities []float64, originNS, deadlineNS uint64) ([ipcclient.LocalJogUpdateBytes]byte, error) {
	var frame ipcclient.LocalJogUpdate
	bad := func() ([ipcclient.LocalJogUpdateBytes]byte, error) {
		return [ipcclient.LocalJogUpdateBytes]byte{}, errors.New("invalid jog frame")
	}
	token, err := hex.DecodeString(fence.Session)
	if err != nil || len(token) != len(frame.SessionId) {
		return bad()
	}
	clock, err := hex.DecodeString(incarnation)
	if err != nil || len(clock) != len(frame.ClockIncarnation) {
		return bad()
	}
	// Unverified absolute input lifetime: 2 s; admission enforces the compiled cell ceiling.
	if fence.Generation == 0 || generation == 0 || sequence == 0 || mask == 0 || mask>>len(frame.Velocity) != 0 || len(velocities) > len(frame.Velocity) || mask>>len(velocities) != 0 || originNS == 0 || deadlineNS <= originNS || deadlineNS-originNS > 2000000000 {
		return bad()
	}
	for i, v := range velocities {
		if math.IsNaN(v) || math.IsInf(v, 0) || (mask&(1<<i) == 0 && v != 0) {
			return bad()
		}
	}
	copy(frame.SessionId[:], token)
	copy(frame.ClockIncarnation[:], clock)
	copy(frame.Velocity[:], velocities)
	frame.GrantGeneration, frame.JogGeneration, frame.SourceSequence = fence.Generation, generation, sequence
	frame.OriginHostNs, frame.DeadlineHostNs, frame.AxisMask = originNS, deadlineNS, mask
	return ipcclient.EncodeLocalJogUpdate(frame), nil
}

// JogSession owns a fixed grant/jog generation and a nonblocking Unix datagram
// socket. It never adopts a newer Stop fence and cannot revive cancelled input.
type JogSession struct {
	client        *Client
	conn          *net.UnixConn
	fence         Fence
	clock         string
	generation    uint64
	mask          uint32
	mu            sync.Mutex
	sequence      uint64
	lastCaptureNS uint64
	closed        bool
}

// Generation identifies the immutable Begin receipt, preventing observers from
// applying an older generation's retirement to a newly opened session.
func (s *JogSession) Generation() uint64 { return s.generation }

// NewJogSession negotiates the host clock before BeginJog. deadlineNS is an
// absolute CLOCK_MONOTONIC deadline, never wall time or a receipt-relative TTL.
func (c *Client) NewJogSession(ctx context.Context, mask uint32, deadlineNS uint64) (*JogSession, error) {
	return c.NewJogSessionAt(ctx, mask, ipcclient.HostMonotonicNS(), deadlineNS)
}

// NewJogSessionAt retains the producer's CLOCK_MONOTONIC capture time across
// clock negotiation and BeginJog, preventing transport work from freshening input.
func (c *Client) NewJogSessionAt(ctx context.Context, mask uint32, originNS, deadlineNS uint64) (*JogSession, error) {
	return c.newJogSession(ctx, mask, originNS, deadlineNS, false)
}

// PrepareJogSession negotiates a lane without carrying an input sample. Its
// first-input allowance uses the published cell age; callers must capture fresh
// input after readiness rather than replaying input held across setup.
func (c *Client) PrepareJogSession(ctx context.Context, mask uint32) (*JogSession, error) {
	return c.newJogSession(ctx, mask, 0, 0, true)
}

func (c *Client) newJogSession(ctx context.Context, mask uint32, originNS, deadlineNS uint64, prepare bool) (*JogSession, error) {
	if c.unixPath == "" {
		return nil, errors.New("local jog requires a Unix client")
	}
	clock, err := c.JogClock(ctx)
	if err != nil {
		return nil, err
	}
	if clock.Domain != "CLOCK_MONOTONIC" || clock.MappingGeneration != 1 {
		return nil, errors.New("unqualified local jog clock")
	}
	if prepare {
		c.mu.Lock()
		age := c.maxJogInputAge
		c.mu.Unlock()
		if age == 0 {
			age = ipcclient.FastControlMaxJogInputAgeNs
		}
		originNS = ipcclient.HostMonotonicNS()
		deadlineNS = originNS + age
	}
	fence := c.Fence()
	if _, err := BuildJogFrame(fence, clock.Incarnation, 1, 1, mask, make([]float64, 16), originNS, deadlineNS); err != nil {
		return nil, err
	}
	conn, err := net.DialUnix("unixgram", nil, &net.UnixAddr{Name: filepath.Join(filepath.Dir(c.unixPath), "jog.sock"), Net: "unixgram"})
	if err != nil {
		return nil, errors.Join(ErrTransport, err)
	}
	request := Request{Operation: "begin_jog", Fence: fence, AxisMask: mask, ClockIncarnation: clock.Incarnation, SourceOriginHostNS: originNS, DeadlineHostNS: deadlineNS}
	response, err := c.Command(ctx, &request)
	if err != nil {
		conn.Close()
		return nil, err
	}
	return &JogSession{client: c, conn: conn, fence: fence, clock: clock.Incarnation, generation: response.Handle, mask: mask, lastCaptureNS: originNS}, nil
}

// Update emits at most one datagram at the caller's cadence. Congestion returns
// ErrJogWouldBlock immediately; no queue replays old input or waits on HTTP work.
func (s *JogSession) Update(velocities []float64, deadlineNS uint64) error {
	return s.UpdateAt(velocities, ipcclient.HostMonotonicNS(), deadlineNS)
}

// UpdateAt preserves producer capture and expiry. Sent samples require strictly
// increasing captures, preventing older input from advancing the wire sequence.
// The first sample may reuse Begin's capture; refused sends do not advance it.
func (s *JogSession) UpdateAt(velocities []float64, originNS, deadlineNS uint64) error {
	if !s.mu.TryLock() {
		return ErrJogWouldBlock
	}
	defer s.mu.Unlock()
	if s.closed {
		return ErrClosed
	}
	if originNS < s.lastCaptureNS || (s.sequence != 0 && originNS == s.lastCaptureNS) {
		return &Rejected{Reason: "jog_invalid_timing"}
	}
	if s.sequence == ^uint64(0) {
		return errors.New("jog sequence exhausted")
	}
	frame, err := BuildJogFrame(s.fence, s.clock, s.generation, s.sequence+1, s.mask, velocities, originNS, deadlineNS)
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
	s.sequence++
	s.lastCaptureNS = originNS
	return nil
}

// End closes the input socket before ending the original jog generation. It
// never rebinds to a later grant; uncertain acknowledgement requires Stop/expiry.
func (s *JogSession) End(ctx context.Context) error {
	s.mu.Lock()
	if s.closed {
		s.mu.Unlock()
		return nil
	}
	s.closed = true
	err := s.conn.Close()
	s.mu.Unlock()
	_, endErr := s.client.Command(ctx, &Request{Operation: "end_jog", Fence: s.fence, JogGeneration: s.generation})
	return errors.Join(err, endErr)
}
