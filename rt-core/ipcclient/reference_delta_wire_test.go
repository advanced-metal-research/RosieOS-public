//go:build linux

package ipcclient

import "testing"

func TestVerifiedDeltaRestoreDistinctWire(t *testing.T) {
	if VersionMinor != 14 || MSG_CMD_RESTORE_REFERENCE_VERIFIED_DELTA != 267 {
		t.Fatal("explicit operation/version contract changed")
	}
	p := EncodeCmdRestoreReferenceVerifiedDeltaV1(CmdRestoreReferenceVerifiedDeltaV1{AxisMask: 32})
	got, err := DecodeCmdRestoreReferenceVerifiedDeltaV1Frame(p[:])
	if err != nil || got.AxisMask != 32 || got.Flags != 0 {
		t.Fatalf("round trip: %+v %v", got, err)
	}
	p[4] = 1
	if _, err = DecodeCmdRestoreReferenceVerifiedDeltaV1Frame(p[:]); err == nil {
		t.Fatal("unknown restore flags accepted")
	}
	if _, err = DecodeCmdRestoreReferenceVerifiedDeltaV1Frame(p[:7]); err == nil {
		t.Fatal("short restore payload accepted")
	}
}
