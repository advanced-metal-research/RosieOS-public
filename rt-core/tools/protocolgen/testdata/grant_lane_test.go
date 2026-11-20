package ipcclient

import (
	"bytes"
	"testing"
)

func TestGrantLaneIndependentGolden(t *testing.T) {
	if FastControlMagic != 0x46545243 || FastControlRegionBytes != 4096 ||
		FastControlGrantOffset != 128 || FastControlObservationOffset != 256 || FastControlMaxGrantLeaseNs != 500000000 {
		t.Fatal("fast-control geometry or native lease cap changed")
	}
	if FastHeaderBytes != 96 || GrantRequestBytes != 80 || GrantObservationBytes != 128 {
		t.Fatal("grant lane record sizes changed")
	}
	if [3]uint32{GrantOperationAcquire, GrantOperationRenew, GrantOperationRevoke} != [3]uint32{1, 2, 3} {
		t.Fatal("grant operation wire values changed")
	}
	if [13]uint32{GrantReasonNone, GrantReasonInvalid, GrantReasonConnectionMismatch,
		GrantReasonNativeGenerationMismatch, GrantReasonDisconnected, GrantReasonBusy,
		GrantReasonNotInhibited, GrantReasonStaleGeneration, GrantReasonIdentityMismatch,
		GrantReasonExpired, GrantReasonClockRegression, GrantReasonInactive, GrantReasonAxisMaskMismatch} !=
		[13]uint32{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12} {
		t.Fatal("grant reason wire values changed")
	}
	header := FastHeader{
		Magic: 0x0706050403020100, VersionMajor: 0x0b0a0908, VersionMinor: 0x0f0e0d0c,
		RegionBytes: 0x1716151413121110, ConnectionId: 0x2f2e2d2c2b2a2928,
		CycleNs: 0x5756555453525150, ConfiguredMask: 0x5b5a5958, Reserved: 0x5f5e5d5c,
	}
	request := GrantRequest{
		Ticket: 0x0706050403020100, ConnectionId: 0x0f0e0d0c0b0a0908,
		NativeGeneration: 0x1716151413121110, AppGrantGeneration: 0x2f2e2d2c2b2a2928,
		OriginHostNs: 0x3736353433323130, DeadlineHostNs: 0x3f3e3d3c3b3a3938,
		AxisMask: 0x43424140, Operation: 0x47464544, Reserved: 0x4f4e4d4c4b4a4948,
	}
	observation := GrantObservation{
		Ticket: 0x0706050403020100, ConnectionId: 0x0f0e0d0c0b0a0908,
		NativeGeneration: 0x1716151413121110, AppGrantGeneration: 0x1f1e1d1c1b1a1918,
		DeadlineHostNs: 0x3736353433323130, NowHostNs: 0x3f3e3d3c3b3a3938,
		ConfigurationEpoch: 0x4746454443424140, HomeEpoch: 0x4f4e4d4c4b4a4948,
		StopAckId: 0x5756555453525150, GrantReason: 0x5b5a5958, GrantActive: 0x5f5e5d5c,
		AxisMask: 0x63626160, Reserved: 0x67666564, Reserved1: 0x6f6e6d6c6b6a6968,
		Reserved2: 0x7776757473727170, Reserved3: 0x7f7e7d7c7b7a7978,
	}
	for i := 0; i < 16; i++ {
		header.DaemonIncarnation[i] = byte(0x18 + i)
		header.ConfigurationDigestLo[i] = byte(0x30 + i)
		header.ConfigurationDigestHi[i] = byte(0x40 + i)
		request.CapabilityId[i] = byte(0x18 + i)
		observation.CapabilityId[i] = byte(0x20 + i)
	}
	h, r, o := EncodeFastHeader(header), EncodeGrantRequest(request), EncodeGrantObservation(observation)
	for _, tc := range []struct {
		name    string
		size    int
		encoded []byte
		valid   func([]byte) bool
		invalid func([]byte) bool
	}{
		{"header", 96, h[:], func(p []byte) bool { got, ok := DecodeFastHeader(p); return ok && got == header },
			func(p []byte) bool { got, ok := DecodeFastHeader(p); return !ok && got == (FastHeader{}) }},
		{"request", 80, r[:], func(p []byte) bool { got, ok := DecodeGrantRequest(p); return ok && got == request },
			func(p []byte) bool { got, ok := DecodeGrantRequest(p); return !ok && got == (GrantRequest{}) }},
		{"observation", 128, o[:], func(p []byte) bool { got, ok := DecodeGrantObservation(p); return ok && got == observation },
			func(p []byte) bool { got, ok := DecodeGrantObservation(p); return !ok && got == (GrantObservation{}) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			// Independently specified fields encode as the literal byte sequence
			// 00,01,... at every offset. Reserved/enum semantics are not a codec job.
			golden := make([]byte, tc.size)
			for i := range golden {
				golden[i] = byte(i)
			}
			if !bytes.Equal(tc.encoded, golden) || !tc.valid(golden) {
				t.Fatalf("fixed bytes changed: %x", tc.encoded)
			}
			payload := append(append([]byte{}, golden...), make([]byte, 8)...)
			for size := 0; size <= len(payload); size++ {
				if size != tc.size && !tc.invalid(payload[:size]) {
					t.Fatalf("accepted or partly decoded size %d", size)
				}
			}
			if !tc.invalid(nil) {
				t.Fatal("accepted nil")
			}
		})
	}
	if got := EncodeTrajectoryHandle(TrajectoryHandle{Handle: 0x0123456789abcdef}); got != [8]byte{0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01} {
		t.Fatalf("production trajectory handle changed: %x", got)
	}
}
