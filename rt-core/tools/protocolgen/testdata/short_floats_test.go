package ipcclient

import (
	"math"
	"testing"
)

func TestShortFloatArraysIndependentGolden(t *testing.T) {
	if ShortFloatsBytes != 32 {
		t.Fatal("short-array layout changed")
	}
	value := ShortFloats{
		Single: [1]float64{math.Float64frombits(0x8000000000000000)},
		Pair:   [2]float64{math.Float64frombits(0x7ff8000000001234), math.Float64frombits(1)},
		Tag:    [8]byte{0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17},
	}
	golden := [32]byte{
		0, 0, 0, 0, 0, 0, 0, 0x80,
		0x34, 0x12, 0, 0, 0, 0, 0xf8, 0x7f,
		1, 0, 0, 0, 0, 0, 0, 0,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	}
	if encoded := EncodeShortFloats(value); encoded != golden {
		t.Fatalf("short arrays changed endian, width or offset: %x", encoded)
	}
	decoded, ok := DecodeShortFloats(golden[:])
	if !ok || math.Float64bits(decoded.Single[0]) != 0x8000000000000000 ||
		math.Float64bits(decoded.Pair[0]) != 0x7ff8000000001234 ||
		math.Float64bits(decoded.Pair[1]) != 1 || decoded.Tag != value.Tag {
		t.Fatalf("short-array bits changed: %+v, %v", decoded, ok)
	}
	var wire [40]byte
	copy(wire[:], golden[:])
	for size := 0; size <= len(wire); size++ {
		if size == 32 {
			continue
		}
		if decoded, ok := DecodeShortFloats(wire[:size]); ok || decoded != (ShortFloats{}) {
			t.Fatalf("non-exact record decoded: %d %+v", size, decoded)
		}
	}
}
