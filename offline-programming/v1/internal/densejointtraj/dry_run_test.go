package densejointtraj

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"testing"
)

func headerAndBody(t *testing.T, blob []byte) (map[string]json.RawMessage, []byte) {
	t.Helper()
	n := int(binary.BigEndian.Uint32(blob[:4]))
	var h map[string]json.RawMessage
	if err := json.Unmarshal(blob[4:4+n], &h); err != nil {
		t.Fatal(err)
	}
	return h, blob[4+n:]
}

func TestDryRunPreservesEveryMotionByteAndOriginal(t *testing.T) {
	source, _ := loadFixture(t)
	h, body := headerAndBody(t, source)
	h["process_markers"] = json.RawMessage(`[{"segment":1,"sample":0,"action":"gas","value":true},{"segment":1,"sample":1,"action":"gas","value":false}]`)
	h["producer_note"] = json.RawMessage(`{"retained":"verbatim metadata"}`)
	raw, _ := json.Marshal(h)
	source = append(binary.BigEndian.AppendUint32(nil, uint32(len(raw))), raw...)
	source = append(source, body...)
	before := append([]byte(nil), source...)
	original, err := Decode(source)
	if err != nil {
		t.Fatal(err)
	}
	dry, plan, err := WithoutProcessOutputs(source)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(before, source) {
		t.Fatal("source artifact mutated")
	}
	if plan.Header.TrajectoryDigest == original.Header.TrajectoryDigest {
		t.Fatal("torch removal did not change digest")
	}
	if _, err := Decode(dry); err != nil {
		t.Fatal(err)
	}
	derivedHeader, derivedBody := headerAndBody(t, dry)
	if _, exists := derivedHeader["process_markers"]; exists {
		t.Fatal("process outputs retained")
	}
	for key, value := range h {
		if key == "process_markers" || key == "trajectory_digest" || key == "segments" {
			continue
		}
		if !bytes.Equal(value, derivedHeader[key]) {
			t.Fatalf("metadata %s changed", key)
		}
	}
	for at := 0; at < len(body); at += RecordBytes {
		if !bytes.Equal(body[at:at+RecordBytes-1], derivedBody[at:at+RecordBytes-1]) {
			t.Fatalf("motion bytes changed at %d", at)
		}
		if derivedBody[at+RecordBytes-1] != 0 {
			t.Fatal("torch flag retained")
		}
	}
	for i, segment := range plan.Segments {
		if segment.Kind != original.Segments[i].Kind || segment.SourceID != original.Segments[i].SourceID {
			t.Fatal("segment identity changed")
		}
	}
	again, _, err := WithoutProcessOutputs(dry)
	if err != nil || !bytes.Equal(again, dry) {
		t.Fatalf("not idempotent: %v", err)
	}

	// An otherwise motion-only plan can still carry header output intents.
	dryHeader, dryBody := headerAndBody(t, dry)
	dryHeader["process_markers"] = h["process_markers"]
	raw, _ = json.Marshal(dryHeader)
	markerOnly := append(binary.BigEndian.AppendUint32(nil, uint32(len(raw))), raw...)
	markerOnly = append(markerOnly, dryBody...)
	stripped, markerPlan, err := WithoutProcessOutputs(markerOnly)
	if err != nil || !bytes.Equal(stripped, dry) || markerPlan.Header.TrajectoryDigest != plan.Header.TrajectoryDigest {
		t.Fatalf("header-only outputs not removed canonically: %v", err)
	}
}

func TestDryRunDoesNotRepairInvalidSource(t *testing.T) {
	source, _ := loadFixture(t)
	source[len(source)-1] ^= FlagTorchOn
	if _, _, err := WithoutProcessOutputs(source); err == nil {
		t.Fatal("corrupt source silently repaired")
	}
}
