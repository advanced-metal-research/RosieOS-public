//go:build linux

package control

import (
	"bytes"
	"encoding/binary"
	"errors"
	"io"
	"net"
	"testing"
	"time"
)

type frameWriteConn struct {
	net.Conn
	writes   [][]byte
	deadline time.Time
	closed   bool
	short    bool
	failure  error
}

func (c *frameWriteConn) Write(p []byte) (int, error) {
	c.writes = append(c.writes, append([]byte(nil), p...))
	if c.short {
		return len(p) - 1, c.failure
	}
	return len(p), c.failure
}
func (c *frameWriteConn) SetWriteDeadline(d time.Time) error { c.deadline = d; return nil }
func (c *frameWriteConn) Close() error                       { c.closed = true; return nil }

func TestRemoteWebsocketWritesCompleteFrameOnce(t *testing.T) {
	for _, n := range []int{0, 2, 125, 126, 4096} {
		c := &frameWriteConn{}
		w := websocket{conn: c}
		payload := bytes.Repeat([]byte{0x5a}, n)
		before := time.Now()
		if err := w.writeFrame(1, payload); err != nil {
			t.Fatal(err)
		}
		after := time.Now()
		if len(c.writes) != 1 {
			t.Fatalf("payload %d: %d writes", n, len(c.writes))
		}
		p := c.writes[0]
		header := 2
		if p[0] != 0x81 || p[1]&0x80 != 0 {
			t.Fatal("incorrect opcode/FIN/mask", p[:2])
		}
		if n < 126 {
			if int(p[1]) != n {
				t.Fatal("short length")
			}
		} else {
			header = 4
			if p[1] != 126 || int(binary.BigEndian.Uint16(p[2:4])) != n {
				t.Fatal("extended length")
			}
		}
		if !bytes.Equal(p[header:], payload) {
			t.Fatal("payload altered")
		}
		if c.deadline.Before(before.Add(20*time.Millisecond)) || c.deadline.After(after.Add(20*time.Millisecond)) {
			t.Fatal("write deadline changed", c.deadline)
		}
		if c.closed || w.closed {
			t.Fatal("successful frame closed connection")
		}
	}
}
func TestRemoteWebsocketPartialOrFailedWriteClosesWithoutReplay(t *testing.T) {
	sentinel := errors.New("transport fixture")
	for _, tc := range []struct {
		name          string
		short         bool
		failure, want error
	}{
		{"short", true, nil, io.ErrShortWrite},
		{"partial-error", true, sentinel, sentinel},
		{"complete-error", false, sentinel, sentinel},
	} {
		t.Run(tc.name, func(t *testing.T) {
			c := &frameWriteConn{short: tc.short, failure: tc.failure}
			w := websocket{conn: c}
			if err := w.writeFrame(1, []byte("reply")); !errors.Is(err, tc.want) {
				t.Fatal(err)
			}
			if !c.closed || !w.closed || len(c.writes) != 1 {
				t.Fatal("failed write not retired")
			}
			if err := w.writeFrame(8, []byte{3, 232}); !errors.Is(err, net.ErrClosed) {
				t.Fatal(err)
			}
			if len(c.writes) != 1 {
				t.Fatal("partial frame replayed or appended")
			}
		})
	}
}
