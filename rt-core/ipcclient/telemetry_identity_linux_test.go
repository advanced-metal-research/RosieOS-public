//go:build linux

package ipcclient

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"net"
	"os"
	"path/filepath"
	"testing"
	"time"

	"golang.org/x/sys/unix"
	"rosieos/rt-core/internal/simfixture"
)

// Telemetry observation requires a native connection but no application grant;
// a simulation reader must receive a complete batch without acquiring authority.
func TestTelemetryWithoutApplicationGrant(t *testing.T) {
	c := startCore(t, "simulation.json").client
	if _, err := c.LocalJogClock(); !errors.Is(err, ErrApplicationGrantUnavailable) {
		t.Fatalf("fixture unexpectedly has an application grant clock: %v", err)
	}
	reader, err := c.OpenTelemetry()
	if err != nil {
		t.Fatal(err)
	}
	defer reader.Close()
	state, err := c.Status()
	if err != nil {
		t.Fatal(err)
	}
	if len(c.Incarnation()) != 32 || reader.DaemonIncarnation != c.Incarnation() || state.DaemonIncarnation != c.Incarnation() {
		t.Fatalf("identity mismatch: welcome=%s ring=%s status=%s", c.Incarnation(), reader.DaemonIncarnation, state.DaemonIncarnation)
	}
	if reader.AxisCount != 2 || reader.CyclePeriodNS != 1000000 {
		t.Fatalf("ring geometry: axes=%d cycle_ns=%d", reader.AxisCount, reader.CyclePeriodNS)
	}
	// Unverified fixture watchdog: 5000 ms; success requires one complete record.
	simfixture.PollTest(t, 5*time.Second, time.Millisecond, func() bool {
		batch, err := reader.ReadFrom(0, 1)
		if errors.Is(err, ErrTelemetryBusy) {
			return false
		}
		if err != nil {
			t.Fatal(err)
		}
		if len(batch.Records) == 0 {
			return false
		}
		if len(batch.Records) != 1 || batch.Records[0].Axes != 2 || batch.Records[0].Seq == 0 || batch.Records[0].TNs == 0 {
			t.Fatalf("incomplete telemetry batch: %+v", batch)
		}
		return true
	})
}

// A grant-free native handshake pins the daemon identity; a different daemon's
// valid ring must be refused before handing a reader to the caller.
func TestTelemetryHandshakeIncarnationMismatch(t *testing.T) {
	original := startCore(t, "simulation.json")
	foreign := startCore(t, "simulation.json")
	if original.client.Incarnation() == foreign.client.Incarnation() {
		t.Fatal("independent daemon handshakes shared an incarnation")
	}
	path := filepath.Join(filepath.Dir(original.options.SocketPath), "telemetry.ring")
	foreignPath := filepath.Join(filepath.Dir(foreign.options.SocketPath), "telemetry.ring")
	if err := os.Symlink(foreignPath, path+".replacement"); err != nil {
		t.Fatal(err)
	}
	if err := os.Rename(path+".replacement", path); err != nil {
		t.Fatal(err)
	}
	ring, err := OpenTelemetry(path)
	if err != nil {
		t.Fatal(err)
	}
	defer ring.Close()
	if ring.DaemonIncarnation != foreign.client.Incarnation() {
		t.Fatal("replacement ring does not match the foreign handshake")
	}
	if err := original.client.Err(); err != nil {
		t.Fatalf("original connection is not live: %v", err)
	}
	reader, err := original.client.OpenTelemetry()
	if reader != nil {
		reader.Close()
		t.Fatal("foreign ring handed to a connected reader")
	}
	if !errors.Is(err, ErrTelemetryIdentity) {
		t.Fatalf("foreign handshake identity: got %v, want ErrTelemetryIdentity", err)
	}
}

// The WELCOME identity uses the same byte order as the ring header; independent
// literal offsets catch generated codecs that shift or truncate the identity.
func TestTelemetryWelcomeIncarnationLayout(t *testing.T) {
	if VersionMinor != 14 || WelcomeV1Bytes != 96 || WelcomeV1DaemonIncarnationOffset != 64 || WelcomeV1Reserved1Offset != 80 {
		t.Fatal("WELCOME identity layout changed")
	}
	var raw [96]byte
	copy(raw[:], "GIPC")
	binary.LittleEndian.PutUint16(raw[4:], 1)
	binary.LittleEndian.PutUint16(raw[6:], 8)
	binary.LittleEndian.PutUint32(raw[8:], 96)
	identity := [16]byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
	copy(raw[64:80], identity[:])
	welcome, err := DecodeWelcomeV1(raw[:])
	if err != nil || welcome.DaemonIncarnation != identity {
		t.Fatalf("decoded identity: %+v %v", welcome, err)
	}
	encoded := EncodeWelcomeV1(welcome)
	if !bytes.Equal(encoded[:], raw[:]) {
		t.Fatalf("WELCOME bytes changed: %x", encoded)
	}
	raw[80] = 1
	if _, err := DecodeWelcomeV1(raw[:]); !errors.Is(err, ErrMalformedFrame) {
		t.Fatalf("nonzero reserved word accepted: %v", err)
	}
}

// Dial rejects absent, obsolete or inconsistent handshake identity before using
// observer files, preventing an application grant from supplying a fallback.
func TestTelemetryHandshakeAdmission(t *testing.T) {
	for _, tc := range []struct {
		name, reason string
		adapter      bool
		change       func(*WelcomeV1)
	}{
		{"missing", "invalid RTCore welcome incarnation", false, func(w *WelcomeV1) { w.DaemonIncarnation = [16]byte{} }},
		{"previous-minor", "invalid RTCore welcome/version", false, func(w *WelcomeV1) { w.VerMinor = 7 }},
		{"pre-current-reference-minor10", "invalid RTCore welcome/version", false, func(w *WelcomeV1) { w.VerMinor = 10 }},
		{"grant-mismatch", "RTCore application grant incarnation mismatch", true, func(w *WelcomeV1) { w.DaemonIncarnation[0] ^= 0xff }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := startCoreWithRole(t, "simulation.json", !tc.adapter)
			dir := filepath.Dir(f.options.SocketPath)
			socket := filepath.Join(dir, "welcome.sock")
			listener, err := net.ListenUnix("unixpacket", &net.UnixAddr{Name: socket, Net: "unixpacket"})
			if err != nil {
				t.Fatal(err)
			}
			defer listener.Close()
			welcome := WelcomeV1{Magic: kMagicGipc, VerMajor: VersionMajor, VerMinor: VersionMinor,
				Bytes: WelcomeV1Bytes, NumAxes: 2, CycleNs: 1000000,
				CmdRingCapacity: GRADIENT_CMD_RING_CAPACITY, CmdMsgBytes: GRADIENT_RING_MSG_BYTES,
				StatusRingCapacity: GRADIENT_STATUS_RING_CAPACITY, StatusMsgBytes: GRADIENT_RING_MSG_BYTES,
				DaemonIncarnation: f.client.incarnation}
			tc.change(&welcome)
			reply := EncodeWelcomeV1(welcome)
			server := make(chan error, 1)
			go func() {
				conn, err := listener.AcceptUnix()
				if err != nil {
					server <- err
					return
				}
				defer conn.Close()
				// Unverified fixture watchdog: 5000 ms; no success depends on waiting.
				if err := conn.SetDeadline(time.Now().Add(5 * time.Second)); err != nil {
					server <- err
					return
				}
				var hello [HelloV1Bytes]byte
				if n, err := conn.Read(hello[:]); err != nil || n != HelloV1Bytes {
					server <- fmt.Errorf("HELLO bytes=%d: %v", n, err)
					return
				}
				_, _, err = conn.WriteMsgUnix(reply[:], unix.UnixRights(f.client.fds...), nil)
				server <- err
			}()
			options := f.options
			options.SocketPath = socket
			// Unverified fixture watchdog: 5000 ms; success requires the explicit refusal.
			ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
			defer cancel()
			client, dialErr := Dial(ctx, options)
			if client != nil {
				client.Close()
				t.Error("invalid handshake admitted")
			}
			if dialErr == nil || dialErr.Error() != tc.reason {
				t.Errorf("handshake refusal: got %v, want %s", dialErr, tc.reason)
			}
			if err := <-server; err != nil {
				t.Fatal(err)
			}
		})
	}
}
