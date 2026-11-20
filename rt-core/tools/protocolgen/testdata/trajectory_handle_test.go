package ipcclient

import "testing"

func TestTrajectoryHandleGoldenBytes(t *testing.T) {
	if TrajectoryHandleBytes != 8 || CommandResultBytes != 40 {
		t.Fatal("wire widths changed")
	}
	const handle = uint64(0xfedcba9876543210)
	want := [8]byte{0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe}
	if got := EncodeTrajectoryHandle(TrajectoryHandle{Handle: handle}); got != want {
		t.Fatalf("encoder did not match independent little-endian bytes: %x", got)
	}
	if decoded, ok := DecodeTrajectoryHandle(want[:]); !ok || decoded.Handle != handle {
		t.Fatalf("decoder changed golden handle: %+v %t", decoded, ok)
	}
	for _, value := range []uint64{0, 1, 1 << 63, ^uint64(0)} {
		encoded := EncodeTrajectoryHandle(TrajectoryHandle{Handle: value})
		decoded, ok := DecodeTrajectoryHandle(encoded[:])
		if !ok || decoded.Handle != value {
			t.Fatalf("truncated boundary handle %x: %+v %t", value, decoded, ok)
		}
	}
}

func TestTrajectoryHandleRejectsNonExactPayloads(t *testing.T) {
	if value, ok := DecodeTrajectoryHandle(nil); ok || value != (TrajectoryHandle{}) {
		t.Fatal("nil payload was accepted")
	}
	var payload [24]byte
	for size := 0; size <= len(payload); size++ {
		if size == 8 {
			continue
		}
		value, ok := DecodeTrajectoryHandle(payload[:size])
		if ok || value != (TrajectoryHandle{}) {
			t.Fatalf("accepted payload length %d", size)
		}
	}
}
