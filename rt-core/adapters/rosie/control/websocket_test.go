//go:build linux

package control

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"errors"
	"io"
	"testing"

	"rosieos/rt-core/ipcclient"
)

func TestRemoteWebsocketRejectsInvalidFrames(t *testing.T) {
	cases := []struct {
		name   string
		packet []byte
		code   uint16
	}{
		{"unmasked", []byte{0x82, 0}, 1002},
		{"fragmented", []byte{0x02, 0x80}, 1009},
		{"continuation", []byte{0x80, 0x80}, 1009},
		{"oversize", []byte{0x82, 0xfe, 0x10, 0x01}, 1009},
		{"reserved", []byte{0xc2, 0x80}, 1002},
		{"unknown opcode", []byte{0x83, 0x80}, 1002},
		{"noncanonical length", []byte{0x82, 0xfe, 0, 1}, 1002},
		{"oversize control", []byte{0x89, 0xfe, 0, 126}, 1002},
		{"invalid text", []byte{0x81, 0x81, 0, 0, 0, 0, 0xff}, 1007},
		{"one byte close", []byte{0x88, 0x81, 0, 0, 0, 0, 0}, 1002},
		{"invalid close code", []byte{0x88, 0x82, 0, 0, 0, 0, 3, 237}, 1002},
	}
	for _, test := range cases {
		t.Run(test.name, func(t *testing.T) {
			w := websocket{reader: bufio.NewReader(bytes.NewReader(test.packet))}
			var buffer [4096]byte
			_, _, err := w.readFrame(buffer[:])
			var invalid *websocketError
			if !errors.As(err, &invalid) || invalid.code != test.code {
				t.Fatalf("expected close %d: %v", test.code, err)
			}
		})
	}
}

func TestRemoteWebsocketMaskedOpcodesAndPayloadLimit(t *testing.T) {
	for _, opcode := range []byte{1, 2, 8, 9, 10} {
		payload := []byte("hello")
		if opcode == 8 {
			payload = []byte{3, 232}
		}
		packet := []byte{128 | opcode, 128 | byte(len(payload)), 1, 2, 3, 4}
		for i, b := range payload {
			packet = append(packet, b^byte(i%4+1))
		}
		w := websocket{reader: bufio.NewReader(bytes.NewReader(packet))}
		var buffer [4096]byte
		op, decoded, err := w.readFrame(buffer[:])
		if err != nil || op != opcode || !bytes.Equal(decoded, payload) {
			t.Fatalf("opcode %d: %x %v", opcode, decoded, err)
		}
	}
	packet := make([]byte, 8+4096)
	packet[0] = 0x82
	packet[1] = 0xfe
	binary.BigEndian.PutUint16(packet[2:4], 4096)
	w := websocket{reader: bufio.NewReader(bytes.NewReader(packet))}
	var buffer [4096]byte
	opcode, payload, err := w.readFrame(buffer[:])
	if err != nil || opcode != 2 || len(payload) != 4096 {
		t.Fatalf("4096-byte frame: %d %v", len(payload), err)
	}
}

type repeatedFrame struct {
	packet []byte
	at     int
}

func (r *repeatedFrame) Read(p []byte) (int, error) {
	if len(r.packet) == 0 {
		return 0, io.EOF
	}
	n := copy(p, r.packet[r.at:])
	r.at = (r.at + n) % len(r.packet)
	return n, nil
}

func TestRemoteWebsocketUpdateDecoderDoesNotAllocate(t *testing.T) {
	encoded := ipcclient.EncodeLocalJogUpdate(ipcclient.LocalJogUpdate{SourceSequence: 1})
	packet := []byte{0x82, 0xfe, byte(len(encoded) >> 8), byte(len(encoded)), 0, 0, 0, 0}
	packet = append(packet, encoded[:]...)
	w := websocket{reader: bufio.NewReader(&repeatedFrame{packet: packet})}
	var buffer [4096]byte
	allocations := testing.AllocsPerRun(1000, func() {
		opcode, payload, err := w.readFrame(buffer[:])
		if err != nil || opcode != 2 {
			panic("invalid fixture frame")
		}
		if _, ok := ipcclient.DecodeLocalJogUpdate(payload); !ok {
			panic("invalid codec fixture")
		}
	})
	if allocations != 0 {
		t.Fatalf("update decoder allocated %.0f objects/frame", allocations)
	}
}
