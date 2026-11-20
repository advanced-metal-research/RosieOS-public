package simulator

import (
	"encoding/binary"

	"math"

	"time"
)

const (
	joystickIntentMagic       = 0x4a49
	joystickIntentVersion     = 1
	joystickIntentVersionV2   = 2
	joystickIntentHeaderBytes = 70
	joystickIntentMaxBytes    = 101

	// Frames per robot_v4_steamdeck_game.cpp:176-189 (BASE is the zero value).
	JoystickFrameBase      byte = 0
	JoystickFrameTool      byte = 1
	JoystickFrameJoint     byte = 2
	JoystickFrameTelemetry byte = 4
	JoystickFrameArm       byte = 5

	// Joint-frame sidecar values (robot_v4_cartesian_daemon.cpp:146-154):
	// the selected joint index rides tool_tcp[0] with a magic in tool_tcp[5].
	jointSelectedIndexToolTCPIndex      = 0
	jointSelectedIndexToolTCPMagicIndex = 5
	jointSelectedIndexToolTCPMagic      = 7603.0

	// Wire-compatible pendant identity: the daemon reads "<instance>" from
	// the packet tool_id and forms sender "steamdeck:<instance>"; the leader
	// allowlist requires an instance that starts "steamdeck-"
	// (robot_v4_cartesian_leader_authority.hpp:429-441,
	// robot_v4_cartesian_nats_protocol.hpp:3064-3074). This session is
	// pendant seat 1 of the isolated simulation cell.
	pendantToolID = "steamdeck-sim"
)

// deadmanFromAxes mirrors the pendant: any axis beyond the 0.02 deadband
// holds the deadman down (encode_axes_intent).
func deadmanFromAxes(axes [6]float64, force bool) bool {
	deadman := force
	for _, value := range axes {
		deadman = deadman || math.Abs(value) > 0.02
	}
	return deadman
}

// encodeJoystickIntent builds the V1 (unfenced) pendant datagram.
func encodeJoystickIntent(
	sequence uint32,
	frame byte,
	axes [6]float64,
	toolTCP [6]float64,
	speedScale float64,
	forceDeadman bool,
) []byte {
	return encodeJoystickIntentFenced(sequence, frame, axes, toolTCP, speedScale, forceDeadman, 0, "")
}

// encodeJoystickIntentFenced builds the pendant datagram; with a granted
// leader lease it emits the fenced V2 form (encode_axes_intent's `fenced`
// branch: BE64 fence epoch + u8 lease length + lease id after the tool id).
func encodeJoystickIntentFenced(
	sequence uint32,
	frame byte,
	axes [6]float64,
	toolTCP [6]float64,
	speedScale float64,
	forceDeadman bool,
	leaderFenceEpoch uint64,
	leaderLeaseID string,
) []byte {
	toolID := pendantToolID
	fenced := leaderFenceEpoch != 0 && leaderLeaseID != "" && len(leaderLeaseID) <= 63
	fenceBytes := 0
	if fenced {
		fenceBytes = 8 + 1 + len(leaderLeaseID)
	}
	packet := make([]byte, joystickIntentHeaderBytes+len(toolID)+fenceBytes)
	binary.BigEndian.PutUint16(packet[0:2], joystickIntentMagic)
	if fenced {
		packet[2] = joystickIntentVersionV2
	} else {
		packet[2] = joystickIntentVersion
	}
	offset := 3
	binary.BigEndian.PutUint64(packet[offset:offset+8], uint64(time.Now().UnixNano()))
	offset += 8
	binary.BigEndian.PutUint32(packet[offset:offset+4], sequence)
	offset += 4
	packet[offset] = frame
	offset++
	packet[offset] = byte(len(toolID))
	offset++
	for _, axis := range axes {
		binary.BigEndian.PutUint32(packet[offset:offset+4], math.Float32bits(float32(axis)))
		offset += 4
	}
	clamped := math.Min(1, math.Max(0, speedScale))
	binary.BigEndian.PutUint32(packet[offset:offset+4], math.Float32bits(float32(clamped)))
	offset += 4
	if deadmanFromAxes(axes, forceDeadman) {
		packet[offset] = 1
	}
	offset++
	for _, value := range toolTCP {
		binary.BigEndian.PutUint32(packet[offset:offset+4], math.Float32bits(float32(value)))
		offset += 4
	}
	copy(packet[offset:], toolID)
	offset += len(toolID)
	if fenced {
		binary.BigEndian.PutUint64(packet[offset:offset+8], leaderFenceEpoch)
		offset += 8
		packet[offset] = byte(len(leaderLeaseID))
		offset++
		copy(packet[offset:], leaderLeaseID)
	}
	return packet
}

// jointJogToolTCP builds the joint-frame sidecar for a selected joint.
func jointJogToolTCP(selectedJoint int) [6]float64 {
	var toolTCP [6]float64
	toolTCP[jointSelectedIndexToolTCPIndex] = float64(selectedJoint)
	toolTCP[jointSelectedIndexToolTCPMagicIndex] = jointSelectedIndexToolTCPMagic
	return toolTCP
}

// jointJogAxes places the normalized velocity in the packet axis slot the
// pendant uses (slot = joint index below 6, else slot 0 with the selection
// riding the tool_tcp sidecar) — robot_v4_steamdeck_motion_packets.hpp:74-81.
func jointJogAxes(selectedJoint int, normalizedVelocity float64) [6]float64 {
	var axes [6]float64
	slot := selectedJoint
	if slot < 0 {
		slot = 0
	}
	if slot >= 6 {
		slot = 0
	}
	axes[slot] = normalizedVelocity
	return axes
}
