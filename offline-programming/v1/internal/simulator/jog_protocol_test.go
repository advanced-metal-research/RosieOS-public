package simulator

import (
	"encoding/binary"

	"math"

	"testing"
)

// The pendant ABI is fixed by the Deck and the daemon; these tests pin the Go
// port to the C++ layout (robot_v4_steamdeck_motion_packets.hpp).
func TestJoystickIntentMatchesPendantABI(t *testing.T) {
	axes := [6]float64{0.5, -0.25, 0, 0, 0, 0.125}
	packet := encodeJoystickIntent(7, JoystickFrameBase, axes, [6]float64{}, 0.75, false)

	if len(packet) != joystickIntentHeaderBytes+len(pendantToolID) {
		t.Fatalf("packet length = %d, want %d", len(packet), joystickIntentHeaderBytes+len(pendantToolID))
	}
	if got := binary.BigEndian.Uint16(packet[0:2]); got != 0x4a49 {
		t.Fatalf("magic = %#x, want 0x4a49", got)
	}
	if packet[2] != 1 {
		t.Fatalf("version = %d, want 1", packet[2])
	}
	if got := binary.BigEndian.Uint32(packet[11:15]); got != 7 {
		t.Fatalf("sequence = %d, want 7", got)
	}
	if packet[15] != JoystickFrameBase {
		t.Fatalf("frame = %d, want base", packet[15])
	}
	if int(packet[16]) != len(pendantToolID) {
		t.Fatalf("tool length = %d, want %d", packet[16], len(pendantToolID))
	}
	for index, want := range axes {
		offset := 17 + index*4
		got := math.Float32frombits(binary.BigEndian.Uint32(packet[offset : offset+4]))
		if got != float32(want) {
			t.Fatalf("axis %d = %v, want %v", index, got, float32(want))
		}
	}
	if got := math.Float32frombits(binary.BigEndian.Uint32(packet[41:45])); got != 0.75 {
		t.Fatalf("speed scale = %v, want 0.75", got)
	}
	if packet[45] != 1 {
		t.Fatalf("deadman should derive from active axes")
	}
	if got := string(packet[70:]); got != pendantToolID {
		t.Fatalf("tool id = %q, want %q", got, pendantToolID)
	}
}

func TestJoystickIntentZeroHoldKeepsDeadmanWhenForced(t *testing.T) {
	packet := encodeJoystickIntent(1, JoystickFrameBase, [6]float64{}, [6]float64{}, 1, true)
	if packet[45] != 1 {
		t.Fatalf("forced deadman must survive zero axes (release-edge zero-hold)")
	}
	released := encodeJoystickIntent(2, JoystickFrameBase, [6]float64{}, [6]float64{}, 1, false)
	if released[45] != 0 {
		t.Fatalf("release without force must drop the deadman")
	}
}

func TestJointJogPacksSelectionLikeThePendant(t *testing.T) {
	axes := jointJogAxes(8, 0.4)
	if axes[0] != 0.4 {
		t.Fatalf("joints beyond the 6-wide packet ride slot 0, got %v", axes)
	}
	axes = jointJogAxes(3, -0.2)
	if axes[3] != -0.2 {
		t.Fatalf("joint 3 rides slot 3, got %v", axes)
	}
	toolTCP := jointJogToolTCP(8)
	if toolTCP[0] != 8 || toolTCP[5] != jointSelectedIndexToolTCPMagic {
		t.Fatalf("selected joint sidecar = %v", toolTCP)
	}
}
