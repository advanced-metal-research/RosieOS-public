package ipcclient

import (
	"bytes"
	"encoding/hex"
	"math"
	"os"
	"strings"
	"testing"
)

// These representations include signed zero, subnormals, infinities and both
// quiet/signalling NaN payloads. The codec moves bits; admission rejects values
// inappropriate for motion. No floating-point equality can verify this set.
var velocityBits = [16]uint64{
	0, 0x8000000000000000, 0x3ff0000000000000, 0xc004000000000000,
	1, 0x8000000000000001, 0x7fefffffffffffff, 0x0010000000000000,
	0x7ff0000000000000, 0xfff0000000000000, 0x7ff8000000001234, 0x7ff0000000004321,
	0xfff800000000abcd, 0x400921fb54442d18, 0xffefffffffffffff, 0x3fb999999999999a,
}

func jogFixture() JogUpdate {
	value := JogUpdate{
		ConnId: 0x0706050403020100, NativeGeneration: 0x0f0e0d0c0b0a0908,
		AppGrantGeneration: 0x2726252423222120, ConfigEpoch: 0x2f2e2d2c2b2a2928,
		HomeEpoch: 0x3736353433323130, JogGeneration: 0x3f3e3d3c3b3a3938,
		SourceSequence: 0x4746454443424140, EarliestOriginHostNs: 0x4f4e4d4c4b4a4948,
		ImmutableDeadlineHostNs: 0x5756555453525150, ClockMappingGeneration: 0x5f5e5d5c5b5a5958,
		AxisMask: 0x63626160, Flags: 0x67666564, Reserved: 0xefeeedecebeae9e8,
	}
	for i := range value.CapabilityId {
		value.CapabilityId[i] = byte(0x10 + i)
	}
	for i, bits := range velocityBits {
		value.Velocity[i] = math.Float64frombits(bits)
	}
	return value
}

func TestJogUpdateIndependentGoldenBytes(t *testing.T) {
	if JogUpdateBytes != 240 {
		t.Fatal("jog update must remain exactly 240 bytes")
	}
	raw, err := os.ReadFile("jog_update_golden.hex")
	if err != nil {
		t.Fatal(err)
	}
	golden, err := hex.DecodeString(strings.Join(strings.Fields(string(raw)), ""))
	if err != nil || len(golden) != 240 {
		t.Fatalf("invalid independent golden fixture: %v, %d bytes", err, len(golden))
	}
	want := jogFixture()
	encoded := EncodeJogUpdate(want)
	if !bytes.Equal(encoded[:], golden) {
		t.Fatalf("encoder changed fixed offsets or representation:\n got %x\nwant %x", encoded, golden)
	}
	decoded, ok := DecodeJogUpdate(golden)
	if !ok {
		t.Fatal("decoder rejected an exact-length record containing uninterpreted float bits")
	}
	for i, bits := range velocityBits {
		if got := math.Float64bits(decoded.Velocity[i]); got != bits {
			t.Fatalf("float representation %d changed: got %016x want %016x", i, got, bits)
		}
		decoded.Velocity[i], want.Velocity[i] = 0, 0
	}
	if decoded != want {
		t.Fatalf("decoded integers/capability changed: got %+v want %+v", decoded, want)
	}
	// Every non-exact length, including a valid frame with trailing bytes,
	// fails before it can return any partially decoded fields.
	var payload [256]byte
	copy(payload[:], golden)
	for length := 0; length <= len(payload); length++ {
		if length == 240 {
			continue
		}
		if got, ok := DecodeJogUpdate(payload[:length]); ok || got != (JogUpdate{}) {
			t.Fatalf("accepted or partly decoded length %d", length)
		}
	}
	if got, ok := DecodeJogUpdate(nil); ok || got != (JogUpdate{}) {
		t.Fatal("nil payload was accepted")
	}
}

func TestScalarFloatRepresentation(t *testing.T) {
	want := [16]byte{0, 0, 0, 0, 0, 0, 0, 0x80, 0x98, 0xba, 0xdc, 0xfe, 0x10, 0x32, 0x54, 0x76}
	value := ScalarSample{Value: math.Float64frombits(1 << 63), Marker: 0xfedcba98, Reserved: 0x76543210}
	if got := EncodeScalarSample(value); got != want {
		t.Fatalf("scalar f64 golden bytes changed: %x", got)
	}
	for _, bits := range velocityBits {
		value.Value = math.Float64frombits(bits)
		encoded := EncodeScalarSample(value)
		decoded, ok := DecodeScalarSample(encoded[:])
		if !ok || math.Float64bits(decoded.Value) != bits || decoded.Marker != value.Marker || decoded.Reserved != value.Reserved {
			t.Fatalf("scalar bits changed: %016x %+v", bits, decoded)
		}
	}
}
