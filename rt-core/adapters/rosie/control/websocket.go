//go:build linux

package control

import (
	"bufio"
	"crypto/sha1"
	"encoding/base64"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"strings"
	"sync"
	"time"
	"unicode/utf8"
)

// Unverified transport work bound: 4,096 payload bytes per unfragmented frame.
const websocketPayloadLimit = 4096

type websocket struct {
	conn    net.Conn
	reader  *bufio.Reader
	writeMu sync.Mutex
	closed  bool
	header  [14]byte // Reused by the sole reader; io.Reader calls must not allocate a header per frame.
}
type websocketError struct {
	code   uint16
	reason string
}

func (e *websocketError) Error() string { return e.reason }

func headerToken(value, token string) bool {
	for _, part := range strings.Split(value, ",") {
		if strings.EqualFold(strings.TrimSpace(part), token) {
			return true
		}
	}
	return false
}

// upgradeWebsocket accepts only RFC 6455 version 13 and preserves buffered
// input across hijacking; malformed upgrades never enter the motion lane.
func upgradeWebsocket(w http.ResponseWriter, r *http.Request) (*websocket, error) {
	key := r.Header.Get("Sec-WebSocket-Key")
	decoded, err := base64.StdEncoding.DecodeString(key)
	if r.Method != "GET" || r.ProtoMajor != 1 || !headerToken(r.Header.Get("Connection"), "upgrade") ||
		!strings.EqualFold(r.Header.Get("Upgrade"), "websocket") || r.Header.Get("Sec-WebSocket-Version") != "13" || err != nil || len(decoded) != 16 {
		http.Error(w, "invalid websocket upgrade", http.StatusBadRequest)
		return nil, errors.New("invalid websocket upgrade")
	}
	hijacker, ok := w.(http.Hijacker)
	if !ok {
		return nil, errors.New("hijacking unavailable")
	}
	conn, rw, err := hijacker.Hijack()
	if err != nil {
		return nil, err
	}
	digest := sha1.Sum([]byte(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"))
	_, err = fmt.Fprintf(rw, "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n", base64.StdEncoding.EncodeToString(digest[:]))
	if err == nil {
		err = rw.Flush()
	}
	if err != nil {
		conn.Close()
		return nil, err
	}
	_ = conn.SetDeadline(time.Time{})
	return &websocket{conn: conn, reader: rw.Reader}, nil
}

// readFrame uses the caller's reusable buffer and rejects oversized, unmasked,
// fragmented or reserved encodings before allocating or accepting payload data.
func (w *websocket) readFrame(payload []byte) (byte, []byte, error) {
	header := w.header[:]
	if _, err := io.ReadFull(w.reader, header[:2]); err != nil {
		return 0, nil, err
	}
	opcode := header[0] & 15
	if header[0]&128 == 0 || opcode == 0 {
		return 0, nil, &websocketError{1009, "fragmentation rejected"}
	}
	if header[0]&112 != 0 || header[1]&128 == 0 {
		return 0, nil, &websocketError{1002, "invalid frame flags"}
	}
	size := uint64(header[1] & 127)
	switch size {
	case 126:
		if _, err := io.ReadFull(w.reader, header[2:4]); err != nil {
			return 0, nil, err
		}
		size = uint64(binary.BigEndian.Uint16(header[2:4]))
		if size < 126 {
			return 0, nil, &websocketError{1002, "noncanonical length"}
		}
	case 127:
		if _, err := io.ReadFull(w.reader, header[2:10]); err != nil {
			return 0, nil, err
		}
		size = binary.BigEndian.Uint64(header[2:10])
		if size < 65536 || size>>63 != 0 {
			return 0, nil, &websocketError{1002, "noncanonical length"}
		}
	}
	if size > websocketPayloadLimit || size > uint64(len(payload)) {
		return 0, nil, &websocketError{1009, "payload too large"}
	}
	if opcode != 1 && opcode != 2 && opcode != 8 && opcode != 9 && opcode != 10 {
		return 0, nil, &websocketError{1002, "invalid opcode"}
	}
	if opcode >= 8 && size > 125 {
		return 0, nil, &websocketError{1002, "control payload too large"}
	}
	if _, err := io.ReadFull(w.reader, header[:4]); err != nil {
		return 0, nil, err
	}
	payload = payload[:int(size)]
	if _, err := io.ReadFull(w.reader, payload); err != nil {
		return 0, nil, err
	}
	for i := range payload {
		payload[i] ^= header[i%4]
	}
	if opcode == 1 && !utf8.Valid(payload) {
		return 0, nil, &websocketError{1007, "invalid UTF-8"}
	}
	if opcode == 8 {
		if len(payload) == 1 {
			return 0, nil, &websocketError{1002, "invalid close payload"}
		}
		if len(payload) >= 2 {
			code := binary.BigEndian.Uint16(payload)
			if code < 1000 || code >= 5000 || code == 1004 || code == 1005 || code == 1006 || code == 1015 || (code >= 1016 && code < 3000) {
				return 0, nil, &websocketError{1002, "invalid close code"}
			}
			if !utf8.Valid(payload[2:]) {
				return 0, nil, &websocketError{1007, "invalid close reason"}
			}
		}
	}
	return opcode, payload, nil
}

func (w *websocket) writeFrame(opcode byte, payload []byte) error {
	w.writeMu.Lock()
	defer w.writeMu.Unlock()
	if w.closed {
		return net.ErrClosed
	}
	// Unverified network write bound: a blocked client cannot retain the jog owner beyond 20 ms.
	_ = w.conn.SetWriteDeadline(time.Now().Add(20 * time.Millisecond))
	var header [4]byte
	header[0] = 128 | opcode
	n := 2
	if len(payload) < 126 {
		header[1] = byte(len(payload))
	} else {
		header[1] = 126
		binary.BigEndian.PutUint16(header[2:], uint16(len(payload)))
		n = 4
	}
	// Keep header and payload in one transport write. Separate TLS records can
	// expose small-frame replies to buffering on an intermediate TCP/SSH hop.
	packet := make([]byte, n+len(payload))
	copy(packet, header[:n])
	copy(packet[n:], payload)
	written, err := w.conn.Write(packet)
	if err == nil && written != len(packet) {
		err = io.ErrShortWrite
	}
	if err != nil {
		// Never append or replay after a partial frame: its wire state is unknown.
		w.closed = true
		_ = w.conn.Close()
	}
	return err
}

func (w *websocket) close(code uint16) {
	var payload [2]byte
	binary.BigEndian.PutUint16(payload[:], code)
	_ = w.writeFrame(8, payload[:])
	w.writeMu.Lock()
	w.closed = true
	w.conn.Close()
	w.writeMu.Unlock()
}
