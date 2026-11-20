package ipcclient

import "testing"

func TestByteMarkerIndependentGolden(t *testing.T) {
	if ByteMarkerBytes != 1 {
		t.Fatal("minimum byte array gained padding")
	}
	for _, marker := range []byte{0, 1, 0x80, 0xff} {
		value := ByteMarker{Value: [1]byte{marker}}
		if got := EncodeByteMarker(value); got != [1]byte{marker} {
			t.Fatalf("byte representation changed: %x", got)
		}
		decoded, ok := DecodeByteMarker([]byte{marker})
		if !ok || decoded != value {
			t.Fatalf("byte decoded incorrectly: %+v %v", decoded, ok)
		}
	}
	for _, wire := range [][]byte{nil, {}, {0xff, 0}} {
		if value, ok := DecodeByteMarker(wire); ok || value != (ByteMarker{}) {
			t.Fatalf("non-exact byte record accepted: %x %+v", wire, value)
		}
	}
}
