package ipcclient

import (
	"encoding/binary"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// An actual retained V1 golden dump must be rejected before visiting any record;
// callers can distinguish incompatibility from truncation and name both layouts.
func TestTelemetryV1DumpTypedMismatch(t *testing.T) {
	// Measured pre-P8-34 transitive digest and 16-axis record geometry.
	const oldDigest = "ae42a2ad68df32eff0d7745b7791a3c8b8e2f629991bbdb600b57d303e77b59c"
	raw, err := os.ReadFile("../tools/protocolgen/testdata/golden/CycleCaptureRecordV1.bin")
	if err != nil {
		t.Fatal(err)
	}
	index := TelemetryDumpIndex{ProtocolMajor: 1, ProtocolMinor: 10, LayoutDigest: oldDigest,
		CyclePeriodNS: 1000000, AxisCount: 16, RecordBytes: 2088, FirstSequence: 7, LastSequence: 7,
		WindowFirstSequence: 7, WindowLastSequence: 7, RecordCount: 1, AbsoluteKeys: make([][]string, 16)}
	metadata, err := json.Marshal(index)
	if err != nil {
		t.Fatal(err)
	}
	header := make([]byte, 65536)
	copy(header, "RTTDMP1\n")
	binary.LittleEndian.PutUint32(header[8:], 65536)
	binary.LittleEndian.PutUint32(header[12:], uint32(len(metadata)))
	copy(header[16:], metadata)
	path := filepath.Join(t.TempDir(), "v1.bin")
	if err := os.WriteFile(path, append(header, raw...), 0600); err != nil {
		t.Fatal(err)
	}
	visited := false
	_, err = ReadTelemetryDump(path, func(CycleCaptureRecordV2, TelemetryDumpIndex) error { visited = true; return nil })
	var mismatch *TelemetryLayoutMismatchError
	if !errors.As(err, &mismatch) || visited || mismatch.StoredDigest != oldDigest || mismatch.ExpectedDigest != CycleCaptureRecordV2LayoutDigest ||
		!strings.Contains(err.Error(), oldDigest) || !strings.Contains(err.Error(), CycleCaptureRecordV2LayoutDigest) {
		t.Fatalf("mismatch=%v visited=%t", err, visited)
	}
	var h TelemetryBatchHeaderV1
	copy(h.RecordLayoutDigest[:], oldDigest)
	if err := ValidateTelemetryHeader(h); !errors.As(err, &mismatch) {
		t.Fatalf("batch mismatch: %v", err)
	}
}

// writeTelemetryDump writes a native dump envelope around records encoded by
// this build, declaring the given protocol and digest.
func writeTelemetryDump(t *testing.T, major, minor uint32, digest string, seqs ...uint64) string {
	t.Helper()
	const axes = 6
	recordBytes := uint32(CycleCaptureRecordV2AxOffset + axes*CycleCaptureAxisV2Bytes)
	index := TelemetryDumpIndex{ProtocolMajor: major, ProtocolMinor: minor, LayoutDigest: digest, CyclePeriodNS: 1000000,
		AxisCount: axes, RecordBytes: recordBytes, WindowFirstSequence: 1, WindowLastSequence: 1, MissingRecords: 1,
		AbsoluteKeys: make([][]string, axes), Trigger: "stop"}
	if len(seqs) > 0 {
		index.FirstSequence, index.LastSequence = seqs[0], seqs[len(seqs)-1]
		index.WindowFirstSequence, index.WindowLastSequence = seqs[0], seqs[len(seqs)-1]
		index.RecordCount = uint64(len(seqs))
		index.MissingRecords = seqs[len(seqs)-1] - seqs[0] + 1 - uint64(len(seqs))
	}
	metadata, err := json.Marshal(index)
	if err != nil {
		t.Fatal(err)
	}
	out := make([]byte, 65536)
	copy(out, "RTTDMP1\n")
	binary.LittleEndian.PutUint32(out[8:], 65536)
	binary.LittleEndian.PutUint32(out[12:], uint32(len(metadata)))
	copy(out[16:], metadata)
	for i, seq := range seqs {
		record := CycleCaptureRecordV2{TNs: 1e9 + seq*1e6, Seq: seq, Axes: axes}
		record.Ax[0].Position = int32(100 * i)
		raw := EncodeCycleCaptureRecordV2(record)
		out = append(out, raw[:recordBytes]...)
	}
	path := filepath.Join(t.TempDir(), "stop.bin")
	if err := os.WriteFile(path, out, 0o600); err != nil {
		t.Fatal(err)
	}
	return path
}

// One compatibility rule for every dump reader: a protocol minor that differs
// from this build's is read when the record layout digest is this build's
// (the 1430 cell 1.13 dumps under the 1.14 decoder), and named in CompatibilityNote; a
// foreign digest or another major is refused before any record is visited.
func TestTelemetryDumpMinorReadUnderSameDigest(t *testing.T) {
	var positions []int32
	index, err := ReadTelemetryDump(writeTelemetryDump(t, VersionMajor, VersionMinor-1, CycleCaptureRecordV2LayoutDigest, 7, 8, 10),
		func(r CycleCaptureRecordV2, _ TelemetryDumpIndex) error {
			positions = append(positions, r.Ax[0].Position)
			return nil
		})
	if err != nil || len(positions) != 3 || positions[2] != 200 || index.MissingRecords != 1 {
		t.Fatalf("minor %d with this build's digest: %v, positions %v", VersionMinor-1, err, positions)
	}
	if note := index.CompatibilityNote(); !strings.Contains(note, "same record layout digest") {
		t.Errorf("cross-minor note %q", note)
	}
	if index, err := ReadTelemetryDump(writeTelemetryDump(t, VersionMajor, VersionMinor, CycleCaptureRecordV2LayoutDigest, 1), nil); err != nil || index.CompatibilityNote() != "" {
		t.Errorf("same protocol: %v, note %q", err, index.CompatibilityNote())
	}
	visited := false
	visit := func(CycleCaptureRecordV2, TelemetryDumpIndex) error { visited = true; return nil }
	var mismatch *TelemetryLayoutMismatchError
	if _, err := ReadTelemetryDump(writeTelemetryDump(t, VersionMajor, VersionMinor, strings.Repeat("0", 64), 1), visit); !errors.As(err, &mismatch) || visited {
		t.Errorf("foreign digest: %v, visited %t", err, visited)
	}
	if _, err := ReadTelemetryDump(writeTelemetryDump(t, VersionMajor+1, VersionMinor, CycleCaptureRecordV2LayoutDigest, 1), visit); err == nil || visited {
		t.Errorf("major %d admitted (visited %t)", VersionMajor+1, visited)
	}
}
