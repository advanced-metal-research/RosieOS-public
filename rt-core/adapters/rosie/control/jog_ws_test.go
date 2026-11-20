//go:build linux

package control

import (
	"bufio"
	"bytes"
	"context"
	"crypto/rand"
	"crypto/tls"
	"encoding/base64"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/controltest"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/internal/unixhttp"
	"sort"
	"syscall"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

type remoteMotionFixture struct {
	lifecycleFixture
	pki          *remoteTestPKI
	cert         tls.Certificate
	http         *http.Client
	url, address string
	handler      *RemoteHandler
	clock        ipcclient.LocalJogClock
	renewErrors  chan error
}

// The existing simulator fixture is the native oracle; each remote test releases
// its bootstrap grant and acquires through the actual HTTPS handler.
func startRemoteMotion(t *testing.T) *remoteMotionFixture {
	t.Helper()
	native := startLifecycleCore(t)
	if _, err := native.controller.Stop(native.fence, true); err != nil {
		t.Fatal(err)
	}
	p := newRemoteTestPKI(t, native.controller.binding.PairID)
	cert := p.certificate(t, "pendant", p.files.PairID, false, false)
	// Unverified simulation policy: 5 ms uncertainty, 1 ppb drift, one-second calibration age.
	limits := JogClockLimits{Qualified: true, MaxInputAgeNS: 200_000_000, MaxInputDurationNS: 200_000_000, MaxUncertaintyNS: 5_000_000, DriftPPB: 1, MaxCalibrationAgeNS: 1_000_000_000}
	local := NewHandler(native.controller)
	handler := NewRemoteHandler(native.controller, local, limits)
	address := remoteTestServer(t, p, handler)
	t.Cleanup(handler.Close)
	f := &remoteMotionFixture{lifecycleFixture: native, pki: p, cert: cert, http: p.client(t, &cert), url: "https://" + address, address: address, handler: handler}
	f.fence = Fence{}
	f.http.Timeout = 65 * time.Second // Unverified bulk failure bound; freshness assertions remain in ns.
	reply := f.must(t, Request{Operation: "acquire", Controller: "remote-test", Binding: native.controller.binding})
	var grant Grant
	if err := json.Unmarshal(reply.Data, &grant); err != nil {
		t.Fatal(err)
	}
	f.fence = Fence{grant.Session, grant.Generation}
	f.renewErrors = make(chan error, 1)
	stopRenew := simfixture.Renew(context.Background(), 50*time.Millisecond, func() bool {
		native.controller.mu.Lock()
		fence := Fence{native.controller.grant.Session, native.controller.grant.Generation}
		native.controller.mu.Unlock()
		if fence.Session != "" {
			if _, err := native.controller.Renew(fence); err != nil {
				select {
				case f.renewErrors <- err:
				default:
				}
			}
		}
		return true
	})
	t.Cleanup(stopRenew)
	f.must(t, Request{Operation: "enable", AxisMask: 511})
	f.must(t, Request{Operation: "arm"})
	waitRemoteStatus(t, f, func(s ipcclient.Status) bool {
		return s.Armed == 1 && s.AxisEnableMask == 511 && len(s.Axes) == 9 && s.Axes[0].Statusword&0x6f == 0x27
	})
	var err error
	f.clock, err = native.controller.JogClock()
	if err != nil {
		t.Fatal(err)
	}
	return f
}
func (f *remoteMotionFixture) request(request Request) (publicResetResponse, error) {
	request.Schema = "rosie.rt-control.request.v1"
	request.Fence = f.fence
	body, err := json.Marshal(request)
	if err != nil {
		return publicResetResponse{}, err
	}
	var reply publicResetResponse
	_, err = (controltest.Client{HTTP: f.http, Base: f.url}).Decode(context.Background(), http.MethodPost, "/v1/control", body, http.Header{"Content-Type": {"application/json"}}, &reply)
	return reply, err
}
func (f *remoteMotionFixture) must(t *testing.T, request Request) publicResetResponse {
	t.Helper()
	reply, err := f.request(request)
	if err != nil || reply.Error != "" {
		t.Fatalf("%s: %+v %v", request.Operation, reply, err)
	}
	if request.Operation == "stop" || request.Operation == "release" {
		var grant Grant
		if err := json.Unmarshal(reply.Data, &grant); err != nil {
			t.Fatal(err)
		}
		f.fence = Fence{grant.Session, grant.Generation}
	}
	return reply
}
func (f *remoteMotionFixture) begin(t *testing.T) uint64 {
	t.Helper()
	now := ipcclient.HostMonotonicNS()
	return f.must(t, Request{Operation: "begin_jog", AxisMask: 511, ClockIncarnation: f.clock.Incarnation, SourceOriginHostNS: now, DeadlineHostNS: now + 200_000_000}).Handle
}
func waitRemoteStatus(t *testing.T, f *remoteMotionFixture, predicate func(ipcclient.Status) bool) ipcclient.Status {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	state, err := f.client.Wait(ctx, predicate)
	if err != nil {
		t.Fatalf("status: %+v %v", state, err)
	}
	return state
}
func waitRemoteJog(t *testing.T, f *remoteMotionFixture, predicate func(ipcclient.JogObservation) bool) ipcclient.JogObservation {
	t.Helper()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	deadline := time.NewTimer(2 * time.Second)
	defer deadline.Stop()
	var last ipcclient.JogObservation
	for {
		observation, err := f.controller.JogStatus()
		last = observation
		if err == nil && predicate(observation) {
			return observation
		}
		select {
		case <-ticker.C:
		case <-deadline.C:
			// EndJog clears the native session during websocket cleanup. Keep
			// the earlier ingress refusal and current core state in the failure
			// report so a closed session does not hide its initiating cause.
			state, stateErr := f.client.Status()
			t.Fatalf("jog observation: %+v %v; ingress: %+v; core: %+v %v", last, err, f.controller.JogIngress(), state, stateErr)
		}
	}
}

type remoteWSClient struct {
	conn   *tls.Conn
	reader *bufio.Reader
	source JogClockIncarnation
	offset uint64
}

func (f *remoteMotionFixture) dial(t *testing.T, generation uint64) *remoteWSClient {
	t.Helper()
	conn, err := tls.Dial("tcp", f.address, &tls.Config{RootCAs: f.pki.roots, Certificates: []tls.Certificate{f.cert}, MinVersion: tls.VersionTLS13})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { conn.Close() })
	_ = conn.SetDeadline(time.Now().Add(3 * time.Second))
	var nonce [16]byte
	_, _ = rand.Read(nonce[:])
	key := base64.StdEncoding.EncodeToString(nonce[:])
	_, err = fmt.Fprintf(conn, "GET /v1/jog?session=%s&generation=%d&jog_generation=%d HTTP/1.1\r\nHost: %s\r\nConnection: Upgrade\r\nUpgrade: websocket\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: %s\r\n\r\n", f.fence.Session, f.fence.Generation, generation, f.address, key)
	if err != nil {
		t.Fatal(err)
	}
	reader := bufio.NewReader(conn)
	response, err := http.ReadResponse(reader, nil)
	if err != nil {
		t.Fatal(err)
	}
	if response.StatusCode != 101 {
		body, _ := io.ReadAll(response.Body)
		t.Fatalf("upgrade: %s %s", response.Status, body)
	}
	// Unverified fixture source offset: source monotonic time is host minus 1,000,000,000 ns.
	return &remoteWSClient{conn: conn, reader: reader, source: JogClockIncarnation{0xaa}, offset: 1_000_000_000}
}
func (w *remoteWSClient) send(t *testing.T, opcode byte, payload []byte) {
	t.Helper()
	if err := w.write(opcode, payload); err != nil {
		t.Fatal(err)
	}
}
func (w *remoteWSClient) write(opcode byte, payload []byte) error {
	header := []byte{128 | opcode, 128}
	if len(payload) < 126 {
		header[1] |= byte(len(payload))
	} else {
		header[1] |= 126
		header = append(header, byte(len(payload)>>8), byte(len(payload)))
	}
	var mask [4]byte
	_, _ = rand.Read(mask[:])
	header = append(header, mask[:]...)
	packet := append(header, payload...)
	for i := range payload {
		packet[len(header)+i] ^= mask[i%4]
	}
	_ = w.conn.SetWriteDeadline(time.Now().Add(time.Second))
	_, err := w.conn.Write(packet)
	return err
}
func (w *remoteWSClient) read(t *testing.T) (byte, []byte) {
	t.Helper()
	_ = w.conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	var header [2]byte
	if _, err := io.ReadFull(w.reader, header[:]); err != nil {
		t.Fatal(err)
	}
	if header[1]&128 != 0 {
		t.Fatal("server masked frame")
	}
	size := int(header[1] & 127)
	if size == 126 {
		var extended [2]byte
		if _, err := io.ReadFull(w.reader, extended[:]); err != nil {
			t.Fatal(err)
		}
		size = int(binary.BigEndian.Uint16(extended[:]))
	}
	if size > 4096 {
		t.Fatal("oversize server frame")
	}
	body := make([]byte, size)
	if _, err := io.ReadFull(w.reader, body); err != nil {
		t.Fatal(err)
	}
	return header[0] & 15, body
}
func (w *remoteWSClient) calibrate(t *testing.T) {
	t.Helper()
	var tx uint64
	for seq := uint64(1); seq <= 2; seq++ {
		message, _ := json.Marshal(jogCalibration{Type: "calibrate", Seq: seq, SourceIncarnation: hex.EncodeToString(w.source[:]), SourceNS: ipcclient.HostMonotonicNS() - w.offset, HostTXNS: tx})
		w.send(t, 1, message)
		opcode, body := w.read(t)
		var response struct {
			Type     string `json:"type"`
			Seq      uint64 `json:"seq"`
			HostTXNS uint64 `json:"host_tx_ns"`
			Reason   string `json:"reason"`
		}
		if json.Unmarshal(body, &response) != nil || opcode != 1 || response.Type != "calibrated" || response.Seq != seq || response.HostTXNS == 0 {
			t.Fatalf("calibration: %s", body)
		}
		tx = response.HostTXNS
	}
}
func (w *remoteWSClient) frame(f *remoteMotionFixture, generation, sequence uint64) ipcclient.LocalJogUpdate {
	frame := ipcclient.LocalJogUpdate{ClockIncarnation: [16]byte(w.source), GrantGeneration: f.fence.Generation, JogGeneration: generation, SourceSequence: sequence, AxisMask: 511}
	token, _ := hex.DecodeString(f.fence.Session)
	copy(frame.SessionId[:], token)
	frame.OriginHostNs = ipcclient.HostMonotonicNS() - w.offset
	frame.DeadlineHostNs = frame.OriginHostNs + 200_000_000
	frame.Velocity[0] = .05
	return frame
}
func (w *remoteWSClient) update(t *testing.T, frame ipcclient.LocalJogUpdate) {
	t.Helper()
	bytes := ipcclient.EncodeLocalJogUpdate(frame)
	w.send(t, 2, bytes[:])
}
func (w *remoteWSClient) rejection(t *testing.T, sequence uint64, reason string) {
	t.Helper()
	opcode, body := w.read(t)
	var reply struct {
		Type   string
		Seq    uint64
		Reason string
	}
	if json.Unmarshal(body, &reply) != nil || opcode != 1 || reply.Type != "rejected" || reply.Seq != sequence || reply.Reason != reason {
		t.Fatalf("rejection expected %d/%s: %s", sequence, reason, body)
	}
}

func TestRemoteJogRequiresQualifiedCalibration(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	w := f.dial(t, generation)
	before := waitRemoteStatus(t, f, func(s ipcclient.Status) bool { return true })
	frame := w.frame(f, generation, 1)
	w.update(t, frame)
	w.rejection(t, 1, "jog_clock_unqualified")
	after := waitRemoteStatus(t, f, func(s ipcclient.Status) bool { return s.TimeNS >= before.TimeNS+5_000_000 })
	if after.Axes[0].PositionCounts != before.Axes[0].PositionCounts {
		t.Fatal("unqualified input moved feedback")
	}
	observation := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.NowHostNs >= after.TimeNS })
	if observation.SourceSequence != 0 || observation.HasInput != 0 {
		t.Fatalf("unqualified publication: %+v", observation)
	}
	w.calibrate(t)
	w.update(t, frame)
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 && o.HasInput == 1 })
	waitRemoteStatus(t, f, func(s ipcclient.Status) bool { return s.Axes[0].PositionCounts >= before.Axes[0].PositionCounts+10 })
	w.conn.Close()
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 0 && o.HasInput == 0 })
}

func TestRemoteJogRejectsDifferentPrincipal(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	other := f.pki.certificate(t, "other-pendant", f.pki.files.PairID, false, false)
	client := f.pki.client(t, &other)
	response, err := client.Get(fmt.Sprintf("%s/v1/jog?session=%s&generation=%d&jog_generation=%d", f.url, f.fence.Session, f.fence.Generation, generation))
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	body, err := io.ReadAll(response.Body)
	if err != nil || response.StatusCode != http.StatusForbidden || !bytes.Contains(body, []byte(ReasonSessionPrincipalMismatch)) {
		t.Fatalf("principal binding: %d %s %v", response.StatusCode, body, err)
	}
	observation := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 1 })
	if observation.SourceSequence != 0 {
		t.Fatal("another principal published")
	}
}

func TestRemoteJogRejectsStaleAndFutureOnWire(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	w := f.dial(t, generation)
	w.calibrate(t)
	stale := w.frame(f, generation, 1)
	stale.OriginHostNs -= 201_000_000
	stale.DeadlineHostNs -= 201_000_000
	w.update(t, stale)
	w.rejection(t, 1, "jog_input_too_old")
	future := w.frame(f, generation, 2)
	future.OriginHostNs += 1_000_000_000
	future.DeadlineHostNs += 1_000_000_000
	w.update(t, future)
	w.rejection(t, 2, "jog_input_entirely_future")
	w.update(t, w.frame(f, generation, 3))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 3 })
	encoded := ipcclient.EncodeLocalJogUpdate(w.frame(f, generation, 4))
	f.must(t, Request{Operation: "stop"})
	// Attempt the retired frame after Stop; closure may already reject the write.
	_ = w.write(2, encoded[:])
	w.rejection(t, 0, ReasonControlSessionStale)
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 0 && o.HasInput == 0 })
	_ = w.conn.SetReadDeadline(time.Now().Add(time.Second))
	// A frame still queued at the receiver when it closes makes the kernel send RST
	// instead of FIN, so a reset is as valid a close as EOF once the refusal was read.
	if _, err := w.reader.ReadByte(); !errors.Is(err, io.EOF) && !errors.Is(err, syscall.ECONNRESET) {
		t.Fatalf("expected EOF or reset after Stop refusal: %v", err)
	}
}

// Partial frames cannot retain a cancelled grant or hide its terminal reason.
func TestRemoteJogStopDuringPartialFrame(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	w := f.dial(t, generation)
	w.calibrate(t)
	if _, err := w.conn.Write([]byte{0x82, 0xfe}); err != nil {
		t.Fatal(err)
	}
	f.must(t, Request{Operation: "stop"})
	w.rejection(t, 0, ReasonControlSessionStale)
	if _, err := w.reader.ReadByte(); !errors.Is(err, io.EOF) {
		t.Fatalf("expected EOF after partial-frame Stop refusal: %v", err)
	}
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 0 && o.HasInput == 0 })
}

// Idle and incomplete input receive the same typed deadline refusal before EOF;
// the receiver must still issue EndJog without an application End request.
func TestRemoteJogIdleDeadlineReason(t *testing.T) {
	for _, partial := range []bool{false, true} {
		t.Run(fmt.Sprintf("partial=%t", partial), func(t *testing.T) {
			f := startRemoteMotion(t)
			generation := f.begin(t)
			w := f.dial(t, generation)
			before := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 1 })
			start := time.Now()
			w.send(t, 9, nil)
			if opcode, _ := w.read(t); opcode != 10 {
				t.Fatal("missing idle-window pong")
			}
			if partial {
				if _, err := w.conn.Write([]byte{0x82, 0xfe}); err != nil {
					t.Fatal(err)
				}
			}
			w.rejection(t, 0, ReasonJogStreamIdle)
			// Wire idle bound: FastControlMaxJogInputAgeNs is 250,000,000 ns.
			if time.Since(start) < 250*time.Millisecond {
				t.Fatal("idle refusal preceded the 250 ms read deadline")
			}
			if _, err := w.reader.ReadByte(); !errors.Is(err, io.EOF) {
				t.Fatalf("expected EOF after idle refusal: %v", err)
			}
			waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool {
				return o.Ticket == before.Ticket+1 && o.ControlAccepted == 1 &&
					o.ControlReason == ipcclient.JogReasonNone && o.Open == 0 && o.HasInput == 0
			})
		})
	}
}

func TestRemoteJogPublicationDoesNotAllocate(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	w := remoteWSClient{source: JogClockIncarnation{0xaa}, offset: 1_000_000_000}
	frame := w.frame(f, generation, 1)
	clock, _ := hex.DecodeString(f.clock.Incarnation)
	copy(frame.ClockIncarnation[:], clock)
	frame.OriginHostNs = ipcclient.HostMonotonicNS()
	frame.DeadlineHostNs = frame.OriginHostNs + 200_000_000
	frame.Velocity = [16]float64{}
	var rejected uint32
	allocations := testing.AllocsPerRun(100, func() {
		frame.SourceSequence++
		if reason, _ := f.controller.publishLocalJog(&frame, f.clock.Incarnation); reason != 0 {
			rejected = reason
		}
	})
	if rejected != 0 || allocations != 0 {
		t.Fatalf("publication: rejection=%d allocations/frame=%.0f", rejected, allocations)
	}
}

func TestRemoteJogExpiredSessionCannotReplayQueuedInput(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	w := f.dial(t, generation)
	w.calibrate(t)
	frame := w.frame(f, generation, 1)
	frame.DeadlineHostNs = frame.OriginHostNs + 20_000_000
	w.update(t, frame)
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 && o.HasInput == 1 })
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 0 && o.HasInput == 0 })
	w.update(t, w.frame(f, generation, 2))
	w.rejection(t, 2, "jog_session_stale")
	observation := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 0 })
	if observation.ObservedSourceSequence != 1 {
		t.Fatalf("expired generation replayed: %+v", observation)
	}
}

func TestRemoteJogDeadlineIsConservativeAndImmutable(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	host, _ := hex.DecodeString(f.clock.Incarnation)
	source := JogClockIncarnation{0xaa}
	now := ipcclient.HostMonotonicNS()
	origin := now - 1_000_000_000
	binding := JogClockBinding{Source: source}
	copy(binding.Host[:], host)
	// Independent interval fixture: [1,000,000,000, 1,002,000,000] ns, width 2,000,000 ns.
	limits := JogClockLimits{Qualified: true, MaxInputAgeNS: 200_000_000, MaxInputDurationNS: 200_000_000, MaxUncertaintyNS: 5_000_000, DriftPPB: 1, MaxCalibrationAgeNS: 1_000_000_000}
	mapping, err := NewRemoteJogClock(binding, 1, JogClockExchange{HostSendNS: now, HostReceiveNS: now + 2_000_000, SourceReceiveNS: origin, SourceSendNS: origin}, limits)
	if err != nil {
		t.Fatal(err)
	}
	model := remoteJogClock{binding: binding, mapping: mapping, limits: limits}
	w := remoteWSClient{source: source, offset: 1_000_000_000}
	frame := w.frame(f, generation, 1)
	frame.OriginHostNs = origin
	frame.DeadlineHostNs = origin + 100_000_000
	mapped, admission, err := model.convert(frame, now+2_000_000, now+500_000_000)
	if err != nil {
		t.Fatal(err)
	}
	if mapped.DeadlineHostNs != frame.DeadlineHostNs+1_000_000_000 || mapped.OriginHostNs != now {
		t.Fatalf("conversion: %+v", mapped)
	}
	if reason, _ := f.controller.publishLocalJog(&mapped); reason != 0 {
		t.Fatalf("publication: %d", reason)
	}
	observation := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence == 1 })
	if observation.InputDeadlineHostNs != frame.DeadlineHostNs+1_000_000_000 {
		t.Fatalf("native deadline: %d", observation.InputDeadlineHostNs)
	}
	model.previous = &admission
	extended := frame
	extended.DeadlineHostNs += 50_000_000
	unchanged, _, err := model.convert(extended, now+3_000_000, now+800_000_000)
	if err != nil || unchanged.DeadlineHostNs != mapped.DeadlineHostNs {
		t.Fatalf("deadline extended: %+v %v", unchanged, err)
	}
	model.previous = nil
	_, _, err = model.convert(frame, now+200_000_001, now+500_000_000)
	if err != ErrJogClockInputStale {
		t.Fatalf("stale: %v", err)
	}
	future := frame
	future.OriginHostNs += 10_000_000
	future.DeadlineHostNs += 10_000_000
	_, _, err = model.convert(future, now+2_000_000, now+500_000_000)
	if err != ErrJogClockInputFuture {
		t.Fatalf("future: %v", err)
	}
}

func TestRemoteJogSecondConnectionSupersedes(t *testing.T) {
	f := startRemoteMotion(t)
	generation := f.begin(t)
	old := f.dial(t, generation)
	old.calibrate(t)
	second := f.dial(t, generation)
	for _, w := range []*remoteWSClient{old, second} {
		opcode, body := w.read(t)
		if opcode != 8 || len(body) < 2 || binary.BigEndian.Uint16(body) != 1008 {
			t.Fatalf("supersession close: %d %x", opcode, body)
		}
	}
	observation := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 0 && o.HasInput == 0 })
	// A retired connection cannot publish; even a racing write carries the retired native generation.
	frame := old.frame(f, generation, 1)
	encoded := ipcclient.EncodeLocalJogUpdate(frame)
	_, _ = old.conn.Write(encoded[:])
	after := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.NowHostNs >= observation.NowHostNs+5_000_000 })
	if after.SourceSequence != 0 || after.Open != 0 {
		t.Fatalf("old connection published: %+v", after)
	}
	f.must(t, Request{Operation: "enable", AxisMask: 511})
	f.must(t, Request{Operation: "arm"})
	fresh := f.begin(t)
	if fresh <= generation {
		t.Fatalf("generation did not advance: %d -> %d", generation, fresh)
	}
	next := f.dial(t, fresh)
	next.calibrate(t)
	next.update(t, next.frame(f, fresh, 1))
	waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool {
		return o.ObservedSourceSequence == 1 && o.JogGeneration == fresh
	})
}

func TestRemoteJogNotDelayedByHttpsUpload(t *testing.T) {
	f := startRemoteMotion(t)
	points := make([]ipcclient.Point, 250_000)
	for i := range points {
		points[i] = ipcclient.Point{Time: time.Duration(i) * time.Millisecond, Position: make([]float64, 9)}
	}
	body, err := json.Marshal(Request{Schema: "rosie.rt-control.request.v1", Operation: "prepare_trajectory", Fence: f.fence, AxisMask: 511, Points: points})
	if err != nil {
		t.Fatal(err)
	}
	generation := f.begin(t)
	w := f.dial(t, generation)
	w.calibrate(t)
	type result struct {
		reply publicResetResponse
		err   error
		end   uint64
	}
	done := make(chan result, 1)
	started := ipcclient.HostMonotonicNS()
	go func() {
		var reply publicResetResponse
		response, err := f.http.Post(f.url+"/v1/control", "application/json", bytes.NewReader(body))
		if err == nil {
			err = json.NewDecoder(response.Body).Decode(&reply)
			response.Body.Close()
		}
		done <- result{reply, err, ipcclient.HostMonotonicNS()}
	}()
	ticker := time.NewTicker(10 * time.Millisecond)
	defer ticker.Stop()
	ages := make([]uint64, 0, 20)
	observations := make([]ipcclient.JogObservation, 0, 20)
	for sequence := uint64(1); sequence <= 20; sequence++ {
		<-ticker.C
		frame := w.frame(f, generation, sequence)
		frame.Velocity[0] = .001
		w.update(t, frame)
		observation := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.ObservedSourceSequence >= sequence })
		if observation.ObservedSourceSequence != sequence || observation.UpdateReason != 0 || observation.HasInput != 1 || observation.InputDeadlineHostNs > frame.DeadlineHostNs+w.offset {
			t.Fatalf("sequence %d: %+v", sequence, observation)
		}
		if observation.ObservedOriginHostNs == 0 || observation.FirstObservedHostNs < observation.ObservedOriginHostNs ||
			observation.FirstObservedHostNs >= observation.InputDeadlineHostNs {
			t.Fatalf("sequence %d was not observed within its admitted input lifetime: %+v", sequence, observation)
		}
		ages = append(ages, observation.FirstObservedHostNs-observation.ObservedOriginHostNs)
		observations = append(observations, observation)
	}
	upload := <-done
	if upload.err != nil || upload.reply.Error != "" || upload.reply.Handle == 0 {
		select {
		case err := <-f.renewErrors:
			t.Logf("first renewal failure: %v", err)
		default:
		}
		state, statusErr := f.client.Status()
		t.Logf("native after upload: armed=%d enabled=%d fault=%d status_error=%v", state.Armed, state.AxisEnableMask, state.ExecutionFaultReasons, statusErr)
		t.Fatalf("upload: %+v", upload)
	}
	for _, o := range observations {
		if o.FirstObservedHostNs >= upload.end {
			t.Fatal("jog update waited for upload completion")
		}
		t.Logf("sequence=%d origin_host_ns=%d first_rt_now_host_ns=%d", o.ObservedSourceSequence, o.ObservedOriginHostNs, o.FirstObservedHostNs)
	}
	if upload.end-started < 100_000_000 {
		t.Fatal("upload did not exercise concurrent ingress")
	}
	sort.Slice(ages, func(i, j int) bool { return ages[i] < ages[j] })
	t.Logf("remote jog age ns: p50=%d p99=%d max=%d", ages[9], ages[19], ages[19])
}

func TestRemoteControlOpsMatchLocalSocket(t *testing.T) {
	f := startRemoteMotion(t)
	// Serve the same handler on a genuine protected Unix stream socket.
	dir, err := os.MkdirTemp("", "rc-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(dir) })
	path := filepath.Join(dir, "c.sock")
	listener, err := net.Listen("unix", path)
	if err != nil {
		t.Fatal(err)
	}
	server := &http.Server{Handler: f.handler.local}
	go func() { _ = server.Serve(listener) }()
	t.Cleanup(func() { server.Close() })
	transport := unixhttp.NewClient(path).Transport.(*http.Transport)
	t.Cleanup(transport.CloseIdleConnections)
	for _, lane := range []string{"https", "unix"} {
		t.Run(lane, func(t *testing.T) {
			f.must(t, Request{Operation: "release"})
			f.fence = Fence{}
			if lane == "unix" {
				f.http = &http.Client{Transport: transport, Timeout: 3 * time.Second}
				f.url = "http://local"
			}
			reply := f.must(t, Request{Operation: "acquire", Controller: "parity", Binding: f.controller.binding})
			var grant Grant
			if err := json.Unmarshal(reply.Data, &grant); err != nil {
				t.Fatal(err)
			}
			f.fence = Fence{grant.Session, grant.Generation}
			if grant.Controller != "parity" || grant.LeaseMS != 500 || grant.Binding != f.controller.binding {
				t.Fatalf("grant parity: %+v", grant)
			}
			f.must(t, Request{Operation: "enable", AxisMask: 511})
			f.must(t, Request{Operation: "arm"})
			waitRemoteStatus(t, f, func(s ipcclient.Status) bool {
				return s.Armed == 1 && s.AxisEnableMask == 511 && s.Axes[0].Statusword&0x6f == 0x27
			})
			prepared := f.must(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: []ipcclient.Point{{Position: make([]float64, 9)}, {Time: time.Second, Position: make([]float64, 9)}}})
			if prepared.Handle == 0 {
				t.Fatal("missing prepared handle")
			}
			started := f.must(t, Request{Operation: "start_trajectory", Handle: prepared.Handle})
			if started.Sequence == 0 {
				t.Fatal("missing start sequence")
			}
			f.must(t, Request{Operation: "stop"})
			waitRemoteStatus(t, f, func(s ipcclient.Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
		})
	}
}
