package rtctl

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"rosieos/rt-core/ipcclient"
	"strings"
	"testing"
)

func telemetryPublicationFixture(t *testing.T) (ipcclient.TelemetryPublication, []byte) {
	t.Helper()
	// Large integer clocks/cursors must survive without a float64 round trip.
	const first = uint64(9007199254740997)
	header := ipcclient.TelemetryBatchHeaderV1{
		Magic: ipcclient.TelemetryBatchMagic, Version: 1,
		HeaderBytes:   ipcclient.TelemetryBatchHeaderV1Bytes,
		RecordBytes:   ipcclient.CycleCaptureRecordV2Bytes,
		CyclePeriodNs: 1000000, AxisCount: 1, RecordCount: 2,
		FirstSequence: first, LastSequence: first + 1, Dropped: 3,
	}
	copy(header.AdapterIncarnation[:], strings.Repeat("a", 32))
	copy(header.DaemonIncarnation[:], strings.Repeat("b", 32))
	copy(header.MachineSha256[:], strings.Repeat("c", 64))
	copy(header.DeploymentSha256[:], strings.Repeat("d", 64))
	copy(header.RecordLayoutDigest[:], ipcclient.CycleCaptureRecordV2LayoutDigest)
	batch := ipcclient.TelemetryPublication{Header: header}
	for i := uint64(0); i < 2; i++ {
		record := ipcclient.CycleCaptureRecordV2{TNs: first + i*1000000, Seq: first + i, Cycle: 81 + i, WorkNs: 900, Axes: 1}
		record.Ax[0].Position = -123
		record.Ax[0].CoordinateCounts = -456
		record.Ax[0].CoordinateValid = 1
		record.Ax[0].CoordinateSourceValid = 1
		record.Ax[0].VelocityActualCountsPerS = -789
		record.Ax[0].VelocityActualValid = 1
		batch.Records = append(batch.Records, record)
	}
	raw, err := batch.Encode()
	if err != nil {
		t.Fatal(err)
	}
	return batch, raw
}

func TestTelemetryBatchToJSONL(t *testing.T) {
	batch, raw := telemetryPublicationFixture(t)
	path := filepath.Join(t.TempDir(), "publication.rtb")
	if err := os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	var out, errors bytes.Buffer
	if code := RunTelemetryCLI([]string{"batch-to-jsonl", path}, &out, &errors); code != 0 {
		t.Fatalf("code=%d: %s", code, &errors)
	}
	lines := bytes.Split(bytes.TrimSuffix(out.Bytes(), []byte("\n")), []byte("\n"))
	if len(lines) != 3 {
		t.Fatalf("got %d lines, want envelope and two cycles", len(lines))
	}
	var header map[string]json.RawMessage
	if err := json.Unmarshal(lines[0], &header); err != nil {
		t.Fatal(err)
	}
	h := batch.Header
	want := map[string]string{
		"schema":              `"rosie.rt-core.batch-jsonl.v1"`,
		"raw_sha256":          fmt.Sprintf(`"%x"`, sha256.Sum256(raw)),
		"adapter_incarnation": fmt.Sprintf("%q", strings.Repeat("a", 32)),
		"daemon_incarnation":  fmt.Sprintf("%q", strings.Repeat("b", 32)),
		"machine_sha256":      fmt.Sprintf("%q", strings.Repeat("c", 64)),
		"deployment_sha256":   fmt.Sprintf("%q", strings.Repeat("d", 64)),
		"cycle_period_ns":     "1000000", "axis_count": "1",
		"record_layout_digest": fmt.Sprintf("%q", ipcclient.CycleCaptureRecordV2LayoutDigest),
		"first_sequence":       "9007199254740997", "last_sequence": "9007199254740998",
		"dropped": "3", "record_count": "2",
	}
	if len(header) != len(want) {
		t.Fatalf("header has %d fields, want %d", len(header), len(want))
	}
	for key, value := range want {
		if string(header[key]) != value {
			t.Fatalf("header %s=%s want=%s", key, header[key], value)
		}
	}
	for i, line := range lines[1:] {
		var row struct {
			Time     uint64 `json:"t_ns"`
			Sequence uint64 `json:"seq"`
			Axes     []struct {
				CoordinateCounts int64 `json:"coordinate_counts"`
				CoordinateValid  bool  `json:"coordinate_valid"`
			} `json:"ax"`
		}
		if err := json.Unmarshal(line, &row); err != nil {
			t.Fatal(err)
		}
		if row.Time != batch.Records[i].TNs || row.Sequence != batch.Records[i].Seq || len(row.Axes) != 1 || row.Axes[0].CoordinateCounts != -456 || !row.Axes[0].CoordinateValid {
			t.Fatalf("row=%+v", row)
		}
	}

	// Identical native records use exactly the dump converter's existing JSON
	// projection. Batches carry no absolute-field catalog, so neither fixture has one.
	recordBytes := uint32(ipcclient.CycleCaptureRecordV2AxOffset + ipcclient.CycleCaptureAxisV2Bytes)
	index := ipcclient.TelemetryDumpIndex{
		ProtocolMajor: ipcclient.VersionMajor, ProtocolMinor: ipcclient.VersionMinor,
		LayoutDigest: ipcclient.CycleCaptureRecordV2LayoutDigest, CyclePeriodNS: h.CyclePeriodNs,
		AxisCount: h.AxisCount, RecordBytes: recordBytes, RecordCount: uint64(h.RecordCount),
		FirstSequence: h.FirstSequence, LastSequence: h.LastSequence,
		WindowFirstSequence: h.FirstSequence, WindowLastSequence: h.LastSequence,
		AbsoluteKeys: make([][]string, h.AxisCount), Trigger: "mark",
	}
	metadata, err := json.Marshal(index)
	if err != nil {
		t.Fatal(err)
	}
	dump := make([]byte, 65536)
	copy(dump, "RTTDMP1\n")
	binary.LittleEndian.PutUint32(dump[8:], 65536)
	binary.LittleEndian.PutUint32(dump[12:], uint32(len(metadata)))
	copy(dump[16:], metadata)
	for _, record := range batch.Records {
		encoded := ipcclient.EncodeCycleCaptureRecordV2(record)
		dump = append(dump, encoded[:recordBytes]...)
	}
	if err := os.WriteFile(path, dump, 0600); err != nil {
		t.Fatal(err)
	}
	var dumpOut bytes.Buffer
	if code := RunTelemetryCLI([]string{"dump-to-jsonl", path}, &dumpOut, &errors); code != 0 {
		t.Fatalf("dump code=%d: %s", code, &errors)
	}
	wantRows := append(bytes.Join(lines[1:], []byte("\n")), '\n')
	if !bytes.Equal(dumpOut.Bytes(), wantRows) {
		t.Fatal("batch and dump cycle projections differ")
	}
}

func TestTelemetryBatchToJSONLRejectsMalformedBeforeOutput(t *testing.T) {
	batch, raw := telemetryPublicationFixture(t)
	copy(batch.Header.RecordLayoutDigest[:], strings.Repeat("0", 64))
	badHeader := ipcclient.EncodeTelemetryBatchHeaderV1(batch.Header)
	badLayout := append(append([]byte(nil), badHeader[:]...), raw[ipcclient.TelemetryBatchHeaderV1Bytes:]...)
	for name, malformed := range map[string][]byte{
		"short-header":    raw[:ipcclient.TelemetryBatchHeaderV1Bytes-1],
		"short-record":    raw[:len(raw)-1],
		"trailing-byte":   append(append([]byte(nil), raw...), 0),
		"layout-mismatch": badLayout,
		"oversized":       make([]byte, ipcclient.TelemetryBatchMaxBytes+1),
	} {
		t.Run(name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "malformed.rtb")
			if err := os.WriteFile(path, malformed, 0600); err != nil {
				t.Fatal(err)
			}
			var out, errors bytes.Buffer
			if code := RunTelemetryCLI([]string{"batch-to-jsonl", path}, &out, &errors); code != 1 || out.Len() != 0 || errors.Len() == 0 {
				t.Fatalf("malformed batch: code=%d output=%q errors=%q", code, &out, &errors)
			}
		})
	}
}

func TestTelemetryDeploymentIdentityAndAdmission(t *testing.T) {
	baseline, err := fresh().compile()
	if err != nil {
		t.Fatal(err)
	}
	fixture := fresh()
	fixture.put("machine.runtime.telemetry", `{"retention_ms":1000,"dump_count":3,"dump_bytes":1000000,"directory":"/var/lib/rosie/telemetry"}`)
	changed, err := fixture.compile()
	if err != nil {
		t.Fatal(err)
	}
	if changed.MachineSHA256 != baseline.MachineSHA256 || changed.DeploymentSHA256 == baseline.DeploymentSHA256 ||
		!strings.Contains(strings.Join(changed.Argv, " "), "--telemetry-retention-ms 1000 --telemetry-dump-count 3 --telemetry-dump-bytes 1000000 --telemetry-directory /var/lib/rosie/telemetry") {
		t.Fatal("telemetry did not remain deployment-only")
	}
	for _, block := range []string{`{"dump_bytes":0}`, `{"dump_bytes":null}`, `{"dump_bytes":9007199254740992}`, `{"directory":"C:/windows"}`} {
		fixture := fresh()
		fixture.put("machine.runtime.telemetry", block)
		if _, err := fixture.compile(); err == nil {
			t.Fatalf("invalid telemetry accepted: %s", block)
		}
	}
	for _, block := range []string{`{"retention_ms":99}`, `{"retention_ms":3600001}`, `{"dump_count":0}`, `{"dump_count":101}`, `{"retention_ms":null}`, `{"retention":100}`, `{"directory":"relative"}`, `{"directory":"/"}`, `{"directory":"/tmp/.."}`, `{"directory":null}`, `{"directory":"/nul\u0000"}`} {
		fixture := fresh()
		fixture.put("machine.runtime.telemetry", block)
		if _, err := fixture.compile(); err == nil {
			t.Fatalf("invalid telemetry accepted: %s", block)
		}
	}
}

// An independent binary fixture pins field names, signed counts and absolute
// keys; a changed layout digest must fail before producing any JSONL.
func TestTelemetryDumpToJSONLAndInvalidDigest(t *testing.T) {
	index := ipcclient.TelemetryDumpIndex{
		ProtocolMajor: ipcclient.VersionMajor, ProtocolMinor: ipcclient.VersionMinor,
		LayoutDigest: ipcclient.CycleCaptureRecordV2LayoutDigest, CyclePeriodNS: 1000000,
		AxisCount: 1, RecordBytes: 592, FirstSequence: 7, LastSequence: 7,
		WindowFirstSequence: 7, WindowLastSequence: 7, RecordCount: 1, Trigger: "mark",
		AbsoluteKeys: [][]string{{"turns<>&\"\n"}},
	}
	header := make([]byte, 65536)
	copy(header, "RTTDMP1\n")
	binary.LittleEndian.PutUint32(header[8:], 65536)
	// Wire measurement: 272-byte prefix plus one 320-byte axis record.
	raw := make([]byte, 592)
	binary.LittleEndian.PutUint64(raw, 123)
	binary.LittleEndian.PutUint64(raw[8:], 7)
	binary.LittleEndian.PutUint64(raw[16:], 8)
	binary.LittleEndian.PutUint64(raw[24:], 900)
	binary.LittleEndian.PutUint32(raw[32:], 1)
	binary.LittleEndian.PutUint32(raw[272:], 0xffffffff)
	binary.LittleEndian.PutUint32(raw[276:], 42)
	binary.LittleEndian.PutUint16(raw[280:], 39)
	binary.LittleEndian.PutUint32(raw[288:], 0xfffffffe)
	raw[352] = 1
	binary.LittleEndian.PutUint32(raw[368:], 0xfffe1dc0) // -123456 counts/s.
	binary.LittleEndian.PutUint32(raw[372:], 789)
	binary.LittleEndian.PutUint32(raw[376:], 1)
	binary.LittleEndian.PutUint32(raw[380:], 1)
	// DI1 occupies bit 16; validity and S-ON state must survive JSONL export.
	binary.LittleEndian.PutUint32(raw[384:], 65536)
	binary.LittleEndian.PutUint32(raw[388:], 1)
	binary.LittleEndian.PutUint32(raw[392:], 1)
	binary.LittleEndian.PutUint32(raw[396:], 1)
	binary.LittleEndian.PutUint16(raw[528:], 65215) // -321 per-mille torque.
	raw[546] = 1
	binary.LittleEndian.PutUint32(raw[472:], 3)
	binary.LittleEndian.PutUint32(raw[480:], 321)
	binary.LittleEndian.PutUint32(raw[204:], 1)
	binary.LittleEndian.PutUint32(raw[208:], 4096)
	path := filepath.Join(t.TempDir(), "sample.bin")
	write := func() {
		metadata, _ := json.Marshal(index)
		binary.LittleEndian.PutUint32(header[12:], uint32(len(metadata)))
		copy(header[16:], metadata)
		if err := os.WriteFile(path, append(header, raw...), 0600); err != nil {
			t.Fatal(err)
		}
	}
	raw[556] = 1
	raw[557] = 2
	binary.LittleEndian.PutUint16(raw[558:], 13)
	write()
	var out, errors bytes.Buffer
	if code := RunTelemetryCLI([]string{"dump-to-jsonl", path}, &out, &errors); code != 0 {
		t.Fatalf("code=%d: %s", code, &errors)
	}
	var row map[string]any
	if err := json.Unmarshal(out.Bytes(), &row); err != nil {
		t.Fatal(err)
	}
	ax := row["ax"].([]any)[0].(map[string]any)
	for key, want := range map[string]any{"calibration_valid": float64(1), "position_state": float64(2), "audit_reason": float64(13), "p": float64(-1), "tp": float64(42), "sw": float64(39), "velocity_actual_counts_per_s": float64(-123456), "following_error_counts": float64(789), "di_bits": float64(65536), "di_valid": true, "external_enable_active": true, "external_enable_valid": true, "torque_raw": float64(-321), "torque_valid": true, "collision_torque_cycles": float64(3), "collision_peak_torque_raw": float64(321)} {
		if ax[key] != want {
			t.Fatalf("%s=%v want=%v", key, ax[key], want)
		}
	}
	if row["safety_fault_mask"] != float64(1) || row["execution_fault_reasons"] != float64(4096) {
		t.Fatal(row)
	}
	if af := ax["af"].([]any)[0].(map[string]any); af["k"] != "turns<>&\"\n" || af["v"] != float64(-2) || af["ok"] != float64(1) {
		t.Fatal(af)
	}
	valid, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	for name, malformed := range map[string][]byte{
		"short-prefix":      valid[:15],
		"short-header":      valid[:65535],
		"short-record":      valid[:len(valid)-1],
		"extra-record-byte": append(append([]byte(nil), valid...), 0),
	} {
		t.Run(name, func(t *testing.T) {
			if err := os.WriteFile(path, malformed, 0600); err != nil {
				t.Fatal(err)
			}
			out.Reset()
			errors.Reset()
			if code := RunTelemetryCLI([]string{"dump-to-jsonl", path}, &out, &errors); code != 1 || out.Len() != 0 {
				t.Fatal("malformed dump emitted records")
			}
		})
	}
	index.LayoutDigest = strings.Repeat("0", 64)
	write()
	out.Reset()
	errors.Reset()
	if code := RunTelemetryCLI([]string{"dump-to-jsonl", path}, &out, &errors); code != 1 || out.Len() != 0 {
		t.Fatal("invalid layout emitted records")
	}
}
