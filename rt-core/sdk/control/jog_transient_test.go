//go:build linux

package control

import (
	"bufio"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"io"
	"net"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// transientPeer is the receiver side of a remote jog stream on a pipe: it
// collects every binary input frame the producer sends and writes replies.
type transientPeer struct {
	conn   net.Conn
	inputs chan ipcclient.LocalJogUpdate
}

func newTransientPeer(t *testing.T, conn net.Conn) *transientPeer {
	p := &transientPeer{conn: conn, inputs: make(chan ipcclient.LocalJogUpdate, 64)}
	go func() {
		reader := bufio.NewReader(conn)
		for {
			var header [2]byte
			if _, err := io.ReadFull(reader, header[:]); err != nil {
				return
			}
			size := int(header[1] & 127)
			if size == 126 {
				var extended [2]byte
				if _, err := io.ReadFull(reader, extended[:]); err != nil {
					return
				}
				size = int(binary.BigEndian.Uint16(extended[:]))
			}
			var mask [4]byte
			body := make([]byte, size)
			if _, err := io.ReadFull(reader, mask[:]); err != nil {
				return
			}
			if _, err := io.ReadFull(reader, body); err != nil {
				return
			}
			for i := range body {
				body[i] ^= mask[i%4]
			}
			if header[0]&15 != 2 {
				continue // Clock refresh; no motion input.
			}
			frame, ok := ipcclient.DecodeLocalJogUpdate(body)
			if !ok {
				t.Errorf("undecodable input frame")
				return
			}
			p.inputs <- frame
		}
	}()
	return p
}

func (p *transientPeer) reject(t *testing.T, sequence uint64, reason string) {
	t.Helper()
	body, _ := json.Marshal(remoteJogMessage{Type: "rejected", Seq: sequence, Reason: reason})
	if len(body) >= 126 {
		t.Fatal("test reply too long")
	}
	if _, err := p.conn.Write(append([]byte{0x81, byte(len(body))}, body...)); err != nil {
		t.Fatal(err)
	}
}

func (p *transientPeer) input(t *testing.T) ipcclient.LocalJogUpdate {
	t.Helper()
	select {
	case frame := <-p.inputs:
		return frame
	case <-time.After(2 * time.Second):
		t.Fatal("no input frame")
	}
	return ipcclient.LocalJogUpdate{}
}

func (p *transientPeer) none(t *testing.T) {
	t.Helper()
	select {
	case frame := <-p.inputs:
		t.Fatalf("unexpected resend: %+v", frame)
	default:
	}
}

// remotePipe builds a remote session on a pipe with a controllable source
// clock; calibration is fresh so no refresh exchange interleaves.
func remotePipe(t *testing.T, now *atomic.Uint64) (*RemoteJogSession, *transientPeer) {
	client, peer := net.Pipe()
	t.Cleanup(func() { client.Close(); peer.Close() })
	fence := Fence{Session: strings.Repeat("ab", 32), Generation: 7}
	c := &Client{grant: Grant{Session: fence.Session, Generation: fence.Generation}}
	s := &RemoteJogSession{client: c, conn: client, reader: bufio.NewReader(client), fence: fence,
		clock: strings.Repeat("cd", 16), generation: 3, duration: 100_000_000, mask: 1,
		sourceNow: now.Load, calibratedSource: now.Load()}
	return s, newTransientPeer(t, peer)
}

// the 1430 cell 2026-10-10 22:01:13: the SDK ended a held Cartesian jog on one
// refusal that judged nothing. A transient refusal must resend the speed
// input (new sequence, same speed, origin and deadline) and keep the hold;
// a refusal of a superseded or expired input sends nothing; any other
// refusal still ends the session with its reason.
func TestPathJogTransientRefusalResendsAndKeepsHold(t *testing.T) {
	var now atomic.Uint64
	now.Store(10_000_000_000)
	remote, peer := remotePipe(t, &now)
	s := &PathJogSession{client: remote.client, fence: remote.fence, generation: remote.generation, mask: 1, axes: 1,
		lifetime: 100_000_000, sourceNow: now.Load, opened: now.Load() - 1, remote: remote,
		acks: make(chan pathAckMessage, 4), terminal: make(chan error, 1)}
	go s.readRemote()

	// The reader may still hold the writer for its resend; a producer
	// sends fresh input after ErrJogWouldBlock, so does this test.
	setSpeed := func(speed float64, source uint64) {
		t.Helper()
		for deadline := time.Now().Add(time.Second); ; time.Sleep(100 * time.Microsecond) {
			err := s.SetSpeed(speed, source)
			if err == nil {
				return
			}
			if !errors.Is(err, ErrJogWouldBlock) || time.Now().After(deadline) {
				t.Fatal(err)
			}
		}
	}
	setSpeed(0.05, now.Load())
	first := peer.input(t)
	peer.reject(t, first.SourceSequence, "jog_publisher_busy")
	resent := peer.input(t)
	if resent.SourceSequence != first.SourceSequence+1 || resent.OriginHostNs != first.OriginHostNs ||
		resent.DeadlineHostNs != first.DeadlineHostNs || resent.Velocity != first.Velocity || resent.Flags != 1 {
		t.Fatalf("resend %+v of %+v", resent, first)
	}

	now.Add(20_000_000)
	setSpeed(0.04, now.Load())
	newer := peer.input(t)
	peer.reject(t, resent.SourceSequence, "jog_publisher_busy") // Superseded by newer.
	now.Add(200_000_000)
	peer.reject(t, newer.SourceSequence, "jog_publisher_busy")    // Past its deadline.
	peer.reject(t, newer.SourceSequence, "jog_native_rejected_3") // Native not_ready stays terminal.

	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	err := s.Rejection(ctx)
	var rejection *PathJogRejection
	if !errors.As(err, &rejection) || rejection.Reason != "jog_native_rejected_3" ||
		rejection.NativeReason != ipcclient.JogReasonNotReady || rejection.Transient() {
		t.Fatalf("terminal refusal: %v", err)
	}
	peer.none(t)
	if got := s.TransientRefusals(); got != 3 {
		t.Fatalf("transient refusals %d, want 3", got)
	}
}

// The joint lane shares the rule; resends of one input stop at the bound and
// the stream stays open until a terminal refusal.
func TestRemoteJogTransientRefusalResendsWithinBound(t *testing.T) {
	var now atomic.Uint64
	now.Store(10_000_000_000)
	s, peer := remotePipe(t, &now)
	result := make(chan error, 1)
	go func() { result <- s.Rejection(context.Background()) }()
	// Wait until the reader is active, as a producer's Rejection would be.
	for deadline := time.Now().Add(time.Second); ; {
		s.writeMu.Lock()
		active := s.readerActive
		s.writeMu.Unlock()
		if active {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("reader never started")
		}
		time.Sleep(time.Millisecond)
	}
	must(t, s.Update([]float64{0.02}, now.Load()))
	frame := peer.input(t)
	original := frame
	for i := 0; i < remoteJogTransientResends; i++ {
		peer.reject(t, frame.SourceSequence, "jog_publisher_busy")
		next := peer.input(t)
		if next.SourceSequence != frame.SourceSequence+1 || next.OriginHostNs != original.OriginHostNs ||
			next.DeadlineHostNs != original.DeadlineHostNs || next.Velocity != original.Velocity {
			t.Fatalf("resend %d: %+v of %+v", i+1, next, original)
		}
		frame = next
	}
	peer.reject(t, frame.SourceSequence, "jog_publisher_busy") // Beyond the bound: nothing more.
	peer.reject(t, frame.SourceSequence, "jog_session_stale")
	select {
	case err := <-result:
		var rejection *JogRejection
		if !errors.As(err, &rejection) || rejection.Reason != "jog_session_stale" || rejection.Transient() {
			t.Fatalf("terminal refusal: %v", err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("terminal refusal did not end Rejection")
	}
	peer.none(t)
	if got := s.TransientRefusals(); got != remoteJogTransientResends+1 {
		t.Fatalf("transient refusals %d", got)
	}
}

// Safety and verdict refusals are never transient on either lane.
func TestJogRefusalClassification(t *testing.T) {
	if !jogRejection("jog_publisher_busy", 1).Transient() || !(&PathJogRejection{Reason: "jog_publisher_busy"}).Transient() {
		t.Fatal("jog_publisher_busy must be transient")
	}
	for _, reason := range []string{"jog_native_rejected_3", "jog_native_rejected_6", "jog_native_rejected_8",
		"jog_native_rejected_9", "jog_native_rejected_10", "jog_session_stale", "control_session_stale",
		"jog_input_deadline_expired", "jog_clock_moved_backwards", "jog_clock_unqualified", "jog_stream_idle",
		"jog_invalid_frame", "path_capacity", "busy", ""} {
		if jogRejection(reason, 1).Transient() || (&PathJogRejection{Reason: reason}).Transient() {
			t.Fatalf("%q classified transient", reason)
		}
	}
}
