package ipcclient

import (
	"bytes"
	"encoding/hex"
	"os"
	"strings"
	"testing"
)

func TestLocalJogIndependentFrame(t *testing.T) {
	raw, err := os.ReadFile("local_jog_update_golden.hex")
	if err != nil {
		t.Fatal(err)
	}
	golden, err := hex.DecodeString(strings.TrimSpace(string(raw)))
	if err != nil {
		t.Fatal(err)
	}
	if LocalJogUpdateBytes != 224 {
		t.Fatal("local datagram geometry")
	}
	frame := LocalJogUpdate{GrantGeneration: 7, JogGeneration: 9, SourceSequence: 11,
		OriginHostNs: 1000000000, DeadlineHostNs: 1100000000, AxisMask: 3, Velocity: [16]float64{.125, -.125}}
	for i := range frame.SessionId {
		frame.SessionId[i] = byte(i + 1)
	}
	for i := range frame.ClockIncarnation {
		frame.ClockIncarnation[i] = byte(160 + i)
	}
	encoded := EncodeLocalJogUpdate(frame)
	if !bytes.Equal(encoded[:], golden) {
		t.Fatalf("local frame differs: %x", encoded)
	}
	decoded, ok := DecodeLocalJogUpdate(golden)
	if !ok || decoded != frame {
		t.Fatalf("local frame decode: %+v", decoded)
	}
	for size := 0; size < 224; size++ {
		if _, ok := DecodeLocalJogUpdate(golden[:size]); ok {
			t.Fatalf("short frame %d accepted", size)
		}
	}
	if _, ok := DecodeLocalJogUpdate(append(golden, 0)); ok {
		t.Fatal("oversized frame accepted")
	}
}
