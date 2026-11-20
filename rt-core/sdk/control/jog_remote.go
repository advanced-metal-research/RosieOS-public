//go:build linux

package control

import (
	"bufio"
	"context"
	"crypto/rand"
	"crypto/sha1"
	"crypto/tls"
	"encoding/base64"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
)

// Unverified transport bounds: 4096-byte messages and 20 ms writes. A failed
// write closes the stream because replaying a partially written update is unsafe.
const remoteJogPayloadLimit = 4096
const remoteJogWriteBudget = 20 * time.Millisecond

// JogRejection preserves the wire label and generated native reason code, when
// present. Unknown labels remain refusals, never successful motion admissions.
type JogRejection struct {
	Reason       string
	Sequence     uint64
	NativeReason uint32
}

func (r *JogRejection) Error() string { return r.Reason }

// Transient reports a refusal that judged nothing: the receiver met another
// writer's publication lock or a busy native observation slot beyond its own
// retry window. The input was not admitted and may be sent again, with a new
// sequence and its original origin and deadline. Every other refusal
// (identity, sequence, timing, authority, readiness, native verdicts) is
// terminal for the stream.
func (r *JogRejection) Transient() bool { return jogTransientReason(r.Reason) }

func jogTransientReason(reason string) bool { return reason == adapter.ReasonJogPublisherBusy }

// Unverified resend bound: 8 resends of one input after transient refusals.
// The input's own deadline bounds them first; past either bound the next
// fresh input carries on and native input expiry still stops motion.
const remoteJogTransientResends = 8

// remoteInput is the last input frame sent, kept only to resend it after a
// transient refusal. build re-encodes it under a new sequence with the same
// payload, origin and deadline: a resend never gains lifetime.
type remoteInput struct {
	sequence uint64
	deadline uint64 // Source clock.
	build    func(sequence uint64) ([ipcclient.LocalJogUpdateBytes]byte, error)
	resends  int
}

func jogRejection(reason string, seq uint64) *JogRejection {
	r := &JogRejection{Reason: reason, Sequence: seq}
	if suffix, ok := strings.CutPrefix(reason, "jog_native_rejected_"); ok {
		code, err := strconv.ParseUint(suffix, 10, 32)
		if err == nil {
			r.NativeReason = uint32(code)
		}
	}
	return r
}

type remoteJogMessage struct {
	Type              string `json:"type"`
	SourceIncarnation string `json:"source_incarnation,omitempty"`
	SourceNS          uint64 `json:"source_ns"`
	Seq               uint64 `json:"seq"`
	HostTXNS          uint64 `json:"host_tx_ns,omitempty"`
	HostRXNS          uint64 `json:"host_rx_ns,omitempty"`
	Reason            string `json:"reason,omitempty"`
	AxisMask          uint32 `json:"axis_mask,omitempty"`
	JogGeneration     uint64 `json:"jog_generation,omitempty"`
	Path              bool   `json:"path,omitempty"`
}

// RemoteJogSession owns one dedicated WSS stream and immutable grant/jog fences.
// Call Rejection for wire refusals and Observe for native state. Successful
// writes are not admissions. End must be called even after a transport failure.
// The receiver reports jog_stream_idle and calls EndJog after no complete frame
// within the published cell input-age ceiling after admission.
// This producer sends no automatic keepalive. Closed does not establish expiry.
// Rejection must run alongside Update: it consumes bounded clock-refresh replies
// as well as refusals. Refresh exchanges contain no motion input.
type RemoteJogSession struct {
	client                         *Client
	conn                           net.Conn
	reader                         *bufio.Reader
	fence                          Fence
	clock                          string
	generation, sequence, duration uint64
	mask                           uint32
	writeMu, readMu, observeMu     sync.Mutex
	ended                          bool
	sourceNow                      func() uint64
	calibrationSequence            uint64
	calibrationSource              uint64
	calibrationPending             uint8
	calibratedSource               uint64
	readerActive                   bool
	last                           remoteInput // writeMu.
	transient                      atomic.Uint64
	// begun is the Begin message's source sample: the receiver refuses any
	// input whose origin precedes it (jog_clock_moved_backwards).
	begun uint64
}

// NewRemoteJogSession completes transport and calibration before reserving native
// jog mode, preventing setup from consuming first-input expiry. sourceNow and
// Update timestamps must share one
// monotonic clock; nil selects CLOCK_MONOTONIC. durationNS bounds each input's
// source lifetime. Calibration does not claim bench qualification: default
// daemon flags still reject binary updates with jog_clock_unqualified.
func (c *Client) NewRemoteJogSession(ctx context.Context, mask uint32, durationNS uint64, sourceNow func() uint64) (_ *RemoteJogSession, err error) {
	return c.openRemoteJog(ctx, mask, durationNS, sourceNow, false)
}

// openRemoteJog is NewRemoteJogSession; pathJog begins a path jog session
// (protocol 1.14) on the same calibrated stream instead of a joint jog.
func (c *Client) openRemoteJog(ctx context.Context, mask uint32, durationNS uint64, sourceNow func() uint64, pathJog bool) (_ *RemoteJogSession, err error) {
	ctx, cancel := context.WithTimeout(ctx, c.requestBudget())
	defer cancel()
	if c.unixPath != "" {
		return nil, errors.New("remote jog requires a TLS client")
	}
	if sourceNow == nil {
		sourceNow = ipcclient.HostMonotonicNS
	}
	incarnation, err := NewRequestID()
	if err != nil {
		return nil, err
	}
	fence := c.Fence()
	if _, err := BuildJogFrame(fence, incarnation, 1, 1, mask, make([]float64, 16), 1, 1+durationNS); err != nil {
		return nil, err
	}
	c.mu.Lock()
	age := c.maxJogInputAge
	c.mu.Unlock()
	if age == 0 {
		age = ipcclient.FastControlMaxJogInputAgeNs
	}
	if durationNS > age {
		return nil, &JogRejection{Reason: "jog_input_too_old"}
	}
	s := &RemoteJogSession{client: c, fence: fence, clock: incarnation, duration: durationNS, mask: mask, sourceNow: sourceNow}
	defer func() {
		if err != nil {
			if s.conn != nil {
				s.conn.Close()
			}
			// Unverified cleanup bound: 1000 ms; failed cleanup leaves native expiry.
			cleanup, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			if s.generation != 0 {
				_, cleanupErr := c.Command(cleanup, &Request{Operation: "end_jog", Fence: fence, JogGeneration: s.generation})
				err = errors.Join(err, cleanupErr)
			}
		}
	}()
	// Reuse DialTLS's verified transport configuration on an independent upgrade
	// connection. net/http provides a writable 101 body but not socket deadlines;
	// dial the same transport's TLS config explicitly to bound partial writes.
	transport := c.control.Transport.(*http.Transport)
	u, _ := http.NewRequestWithContext(ctx, "GET", c.base, nil)
	address := u.URL.Host
	if u.URL.Port() == "" {
		address = net.JoinHostPort(u.URL.Hostname(), "443")
	}
	dialer := &tls.Dialer{Config: transport.TLSClientConfig.Clone()}
	c.mu.Lock()
	warm := c.warmReady
	c.mu.Unlock()
	if warm {
		s.conn, err = c.takeWarmJogConnection(ctx)
	} else {
		s.conn, err = dialer.DialContext(ctx, "tcp", address)
	}
	if err != nil {
		// No upgrade or native command has been attempted at this point.
		return nil, errors.Join(ErrConnectionUnsent, err)
	}
	stopCancel := context.AfterFunc(ctx, func() { s.conn.Close() })
	defer stopCancel()
	// Unverified handshake ceiling: 5000 ms, further shortened by caller context.
	d := time.Now().Add(5 * time.Second)
	if deadline, ok := ctx.Deadline(); ok && deadline.Before(d) {
		d = deadline
	}
	s.conn.SetDeadline(d)
	var nonce [16]byte
	if _, err = rand.Read(nonce[:]); err != nil {
		return nil, err
	}
	key := base64.StdEncoding.EncodeToString(nonce[:])
	path := fmt.Sprintf("/v1/jog?session=%s&generation=%d&jog_generation=0", fence.Session, fence.Generation)
	request := fmt.Sprintf("GET %s HTTP/1.1\r\nHost: %s\r\nConnection: Upgrade\r\nUpgrade: websocket\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: %s\r\n\r\n", path, u.URL.Host, key)
	if _, err = io.WriteString(s.conn, request); err != nil {
		return nil, errors.Join(ErrTransport, err)
	}
	s.reader = bufio.NewReader(s.conn)
	response, err := http.ReadResponse(s.reader, nil)
	if err != nil {
		return nil, errors.Join(ErrProtocol, err)
	}
	if response.StatusCode != http.StatusSwitchingProtocols {
		defer response.Body.Close()
		body, e := io.ReadAll(io.LimitReader(response.Body, remoteJogPayloadLimit))
		if e != nil {
			return nil, e
		}
		return nil, jogRejection(strings.TrimSpace(string(body)), 0)
	}
	digest := sha1.Sum([]byte(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"))
	if !strings.EqualFold(response.Header.Get("Upgrade"), "websocket") || !remoteJogToken(response.Header.Get("Connection"), "upgrade") || response.Header.Get("Sec-WebSocket-Accept") != base64.StdEncoding.EncodeToString(digest[:]) {
		return nil, ErrProtocol
	}
	var tx uint64
	for seq := uint64(1); seq <= 2; seq++ {
		source := sourceNow() // Second sample is taken only after reading first reply.
		if source == 0 {
			return nil, ErrProtocol
		}
		body, _ := json.Marshal(remoteJogMessage{Type: "calibrate", SourceIncarnation: incarnation, SourceNS: source, Seq: seq, HostTXNS: tx})
		if err = s.send(1, body); err != nil {
			return nil, err
		}
		var reply remoteJogMessage
		if reply, err = s.message(); err != nil {
			return nil, err
		}
		if reply.Type != "calibrated" || reply.Seq != seq || reply.SourceNS != source || reply.HostTXNS == 0 || reply.HostRXNS > reply.HostTXNS {
			return nil, ErrProtocol
		}
		tx = reply.HostTXNS
		s.calibrationSequence, s.calibratedSource = seq, source
	}
	source := sourceNow()
	if source == 0 {
		return nil, ErrProtocol
	}
	body, _ := json.Marshal(remoteJogMessage{Type: "begin", Seq: 3, HostTXNS: tx, AxisMask: mask, SourceNS: source, Path: pathJog})
	if err = s.send(1, body); err != nil {
		return nil, err
	}
	reply, err := s.message()
	if err != nil {
		return nil, err
	}
	if reply.Type != "begun" || reply.Seq != 3 || reply.JogGeneration == 0 {
		return nil, ErrProtocol
	}
	s.generation, s.begun = reply.JogGeneration, source
	s.conn.SetDeadline(time.Time{})
	return s, nil
}

func remoteJogToken(value, token string) bool {
	for _, part := range strings.Split(value, ",") {
		if strings.EqualFold(strings.TrimSpace(part), token) {
			return true
		}
	}
	return false
}

func (s *RemoteJogSession) send(op byte, payload []byte) error {
	if len(payload) > remoteJogPayloadLimit {
		return ErrProtocol
	}
	var mask [4]byte
	if _, err := rand.Read(mask[:]); err != nil {
		return err
	}
	packet := []byte{0x80 | op, 0x80}
	if len(payload) < 126 {
		packet[1] |= byte(len(payload))
	} else {
		packet[1] |= 126
		packet = append(packet, byte(len(payload)>>8), byte(len(payload)))
	}
	packet = append(packet, mask[:]...)
	for i, v := range payload {
		packet = append(packet, v^mask[i%4])
	}
	s.conn.SetWriteDeadline(time.Now().Add(remoteJogWriteBudget))
	for len(packet) > 0 {
		n, err := s.conn.Write(packet)
		if err != nil || n == 0 {
			s.conn.Close()
			return errors.Join(ErrTransport, err)
		}
		packet = packet[n:]
	}
	return nil
}

// frame reads one complete server text frame body.
func (s *RemoteJogSession) frame() ([]byte, error) {
	var header [4]byte
	if _, err := io.ReadFull(s.reader, header[:2]); err != nil {
		return nil, errors.Join(ErrTransport, err)
	}
	if header[0] != 0x81 || header[1]&0x80 != 0 {
		return nil, ErrProtocol
	}
	n := int(header[1] & 127)
	if n == 127 {
		return nil, ErrProtocol
	}
	if n == 126 {
		if _, err := io.ReadFull(s.reader, header[2:]); err != nil {
			return nil, err
		}
		n = int(binary.BigEndian.Uint16(header[2:]))
		if n < 126 {
			return nil, ErrProtocol
		}
	}
	if n > remoteJogPayloadLimit {
		return nil, ErrProtocol
	}
	body := make([]byte, n)
	if _, err := io.ReadFull(s.reader, body); err != nil {
		return nil, err
	}
	return body, nil
}

func (s *RemoteJogSession) message() (remoteJogMessage, error) {
	var m remoteJogMessage
	body, err := s.frame()
	if err != nil {
		return m, err
	}
	if json.Unmarshal(body, &m) != nil {
		return m, ErrProtocol
	}
	if m.Type == "rejected" && m.Reason != "" {
		return m, jogRejection(m.Reason, m.Seq)
	}
	return m, nil
}

// Update sends exactly one generated binary frame with the caller's source
// sample. Contention drops the attempt; a stream error closes it without replay.
func (s *RemoteJogSession) Update(velocities []float64, sourceNS uint64) error {
	if !s.writeMu.TryLock() {
		return ErrJogWouldBlock
	}
	defer s.writeMu.Unlock()
	if s.ended {
		return ErrClosed
	}
	if s.client.Fence() != s.fence {
		return jogRejection(adapter.ReasonControlSessionStale, s.sequence+1)
	}
	if s.sequence == ^uint64(0) || sourceNS > ^uint64(0)-s.duration {
		return errors.New("jog sequence or timestamp exhausted")
	}
	velocities = append([]float64(nil), velocities...)
	deadline := sourceNS + s.duration
	build := func(sequence uint64) ([ipcclient.LocalJogUpdateBytes]byte, error) {
		return BuildJogFrame(s.fence, s.clock, s.generation, sequence, s.mask, velocities, sourceNS, deadline)
	}
	frame, err := build(s.sequence + 1)
	if err != nil {
		return err
	}
	return s.sendInputLocked(frame, deadline, build)
}

// sendInputLocked sends one captured input frame (velocity or path speed)
// and starts a clock refresh when due. Caller holds writeMu. deadline (source
// clock) and build let a transient refusal resend exactly this input.
func (s *RemoteJogSession) sendInputLocked(frame [ipcclient.LocalJogUpdateBytes]byte, deadline uint64,
	build func(uint64) ([ipcclient.LocalJogUpdateBytes]byte, error)) error {
	s.sequence++
	if err := s.send(2, frame[:]); err != nil {
		return err
	}
	s.last = remoteInput{sequence: s.sequence, deadline: deadline, build: build}
	// Refresh only in response to caller input, before another input lifetime
	// elapses. No timer produces input or renews a captured sample's deadline.
	// Start a fresh two-message exchange: an old reply would include the entire
	// idle interval in its uncertainty. Only its new acknowledgement is echoed.
	now := s.sourceNow()
	if now < s.calibratedSource {
		s.conn.Close()
		return ErrProtocol
	}
	if s.readerActive && s.calibrationPending == 0 && now-s.calibratedSource >= s.duration/2 {
		return s.sendCalibration(0, 1)
	}
	return nil
}

// Caller holds writeMu. Acknowledgements are matched before they may advance the
// exchange. Neither a successful write nor a calibration acknowledgement admits
// velocity or changes the immutable source timestamp already sent above.
func (s *RemoteJogSession) sendCalibration(hostTX uint64, phase uint8) error {
	source := s.sourceNow()
	if source == 0 || source < s.calibratedSource || source < s.calibrationSource || s.calibrationSequence == ^uint64(0) {
		s.conn.Close()
		return ErrProtocol
	}
	s.calibrationSequence++
	s.calibrationSource, s.calibrationPending = source, phase
	body, _ := json.Marshal(remoteJogMessage{Type: "calibrate", SourceIncarnation: s.clock,
		SourceNS: source, Seq: s.calibrationSequence, HostTXNS: hostTX})
	return s.send(1, body)
}

func (s *RemoteJogSession) acceptCalibration(reply remoteJogMessage) error {
	s.writeMu.Lock()
	defer s.writeMu.Unlock()
	if s.ended {
		return ErrClosed
	}
	if s.calibrationPending == 0 || reply.Seq != s.calibrationSequence ||
		reply.SourceNS != s.calibrationSource || reply.HostTXNS == 0 || reply.HostRXNS > reply.HostTXNS {
		return ErrProtocol
	}
	if s.calibrationPending == 1 {
		// Capture after receipt; reusing the previous input timestamp would not
		// establish the required causal round trip.
		return s.sendCalibration(reply.HostTXNS, 2)
	}
	s.calibratedSource, s.calibrationPending = s.calibrationSource, 0
	return nil
}

// TransientRefusals counts transient refusals received on this stream; each
// resent its input when still current and within its deadline (Transient).
func (s *RemoteJogSession) TransientRefusals() uint64 { return s.transient.Load() }

// resendTransient answers a transient refusal without ending the stream. Only
// the latest input is resent, only before its source deadline and within
// remoteJogTransientResends; a newer input already supersedes an older one.
// The resend reuses the input's origin and deadline under the next sequence,
// so every receiver and native fence (sequence monotonic, origin never
// backwards, deadline within the input age) still applies. Errors are
// transport failures only.
func (s *RemoteJogSession) resendTransient(rejected *JogRejection) error {
	s.transient.Add(1)
	s.writeMu.Lock()
	defer s.writeMu.Unlock()
	if s.ended {
		return ErrClosed
	}
	last := s.last
	if last.build == nil || rejected.Sequence != last.sequence || last.resends >= remoteJogTransientResends ||
		s.sequence == ^uint64(0) || s.sourceNow() >= last.deadline {
		return nil
	}
	frame, err := last.build(s.sequence + 1)
	if err != nil {
		return nil
	}
	if err := s.sendInputLocked(frame, last.deadline, last.build); err != nil {
		return err
	}
	s.last.resends = last.resends + 1
	return nil
}

// Rejection waits for a terminal server refusal. A transient refusal resends
// its input (resendTransient) and keeps waiting. Cancellation or malformed
// framing closes this stream so a partial message can never be interpreted as
// another reply.
func (s *RemoteJogSession) Rejection(ctx context.Context) error {
	if !s.readMu.TryLock() {
		return ErrJogWouldBlock
	}
	defer s.readMu.Unlock()
	s.writeMu.Lock()
	s.readerActive = true
	s.writeMu.Unlock()
	defer func() {
		s.writeMu.Lock()
		s.readerActive = false
		s.writeMu.Unlock()
	}()
	stop := context.AfterFunc(ctx, func() { s.conn.Close() })
	defer stop()
	for {
		reply, err := s.message()
		if err == nil && reply.Type == "calibrated" {
			if err = s.acceptCalibration(reply); err == nil {
				continue
			}
		}
		var rejected *JogRejection
		if !errors.As(err, &rejected) {
			s.conn.Close()
			return errors.Join(ErrProtocol, err)
		}
		if rejected.Transient() {
			if err = s.resendTransient(rejected); err == nil {
				continue
			}
		}
		return err
	}
}

// Observe preserves /v1/status's native reason and reports expiry only for
// Expired. An elapsed input deadline cannot distinguish End, Stop or idle close.
func (s *RemoteJogSession) Observe(ctx context.Context) (JogObservation, error) {
	s.observeMu.Lock()
	defer s.observeMu.Unlock()
	status, err := s.client.Status(ctx)
	if err != nil {
		return JogObservation{}, err
	}
	if status.Jog == nil {
		return JogObservation{}, ErrProtocol
	}
	o := *status.Jog
	if o.JogGeneration != s.generation || o.AppGrantGeneration != s.fence.Generation {
		return o, jogRejection(adapter.ReasonJogSessionStale, o.ObservedSourceSequence)
	}
	if o.StateReason == ipcclient.JogReasonExpired {
		return o, &JogRejection{Reason: adapter.ReasonJogInputDeadlineExpired, Sequence: o.ObservedSourceSequence, NativeReason: o.StateReason}
	}
	return o, nil
}

// End uses the original fence through the control client before closing WSS;
// closing first can race the handler's own EndJog and lose the explicit receipt.
func (s *RemoteJogSession) End(ctx context.Context) error {
	s.writeMu.Lock()
	defer s.writeMu.Unlock()
	if s.ended {
		return nil
	}
	s.ended = true
	defer s.conn.Close()
	_, err := s.client.Command(ctx, &Request{Operation: "end_jog", Fence: s.fence, JogGeneration: s.generation})
	return err
}
