package motion_server

import (
	"encoding/binary"
	"encoding/json"
	"math"

	"os"

	"testing"
	"time"
)

func legacyPlanPreloadPacketForTest(revision uint64) []byte {
	const sampleCount = 2
	packet := make([]byte, 14+sampleCount*28)
	binary.BigEndian.PutUint16(packet[0:2], 0x5050)
	packet[2] = 1
	binary.BigEndian.PutUint64(packet[3:11], revision)
	binary.BigEndian.PutUint16(packet[11:13], sampleCount)
	packet[13] = 0 // empty session id
	cursor := 14
	for sample := 0; sample < sampleCount; sample++ {
		binary.BigEndian.PutUint32(packet[cursor:cursor+4], math.Float32bits(float32(sample)))
		cursor += 4
		for axis := 0; axis < 6; axis++ {
			binary.BigEndian.PutUint32(packet[cursor:cursor+4], math.Float32bits(0))
			cursor += 4
		}
	}
	return packet
}

func legacyUDPPlanPlayPacketForTest(revision uint64) []byte {
	packet := make([]byte, 12)
	binary.BigEndian.PutUint16(packet[0:2], 0x5043)
	packet[2] = 1
	packet[3] = 1
	binary.BigEndian.PutUint64(packet[4:12], revision)
	return packet
}

func pathScrubUDPPacketForTest(direction int8, sequence, revision uint64, cursorMS uint32, programID, senderID string) []byte {
	packet := make([]byte, 26+len(programID)+len(senderID))
	binary.BigEndian.PutUint16(packet[0:2], 0x5053)
	packet[2] = 1
	packet[3] = byte(direction)
	binary.BigEndian.PutUint64(packet[4:12], sequence)
	binary.BigEndian.PutUint64(packet[12:20], revision)
	binary.BigEndian.PutUint32(packet[20:24], cursorMS)
	packet[24] = byte(len(programID))
	packet[25] = byte(len(senderID))
	copy(packet[26:], programID)
	copy(packet[26+len(programID):], senderID)
	return packet
}

func waitForPlanAuthorizationStatus(t *testing.T, path string, match func(map[string]any) bool) map[string]any {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	var latest map[string]any
	for time.Now().Before(deadline) {
		raw, err := os.ReadFile(path)
		if err == nil {
			var report map[string]any
			if json.Unmarshal(raw, &report) == nil {
				latest = report
				if match(report) {
					return report
				}
			}
		}
		time.Sleep(10 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for plan authorization status; latest=%+v", latest)
	return nil
}
