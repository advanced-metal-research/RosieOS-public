package densejointtraj

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"os"
	"reflect"
	"testing"
)

// The checked-in reference pins payload bytes and metadata independently of
// Encode, preventing a matching encoder/decoder mistake from passing roundtrip.
func TestEncodeRetainsReferencePayloadAndHeader(t *testing.T) {
	raw, err := os.ReadFile("testdata/corpus/dense_joint_trajectory_reference.rdt")
	if err != nil {
		t.Fatal(err)
	}
	plan, err := Decode(raw)
	if err != nil {
		t.Fatal(err)
	}
	before, err := json.Marshal(plan.Header)
	if err != nil {
		t.Fatal(err)
	}
	encoded, err := Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	originalHeaderSize := int(binary.BigEndian.Uint32(raw))
	encodedHeaderSize := int(binary.BigEndian.Uint32(encoded))
	if !bytes.Equal(raw[4+originalHeaderSize:], encoded[4+encodedHeaderSize:]) {
		t.Fatal("encoded sample payload differs from reference")
	}
	var header Header
	if err := json.Unmarshal(encoded[4:4+encodedHeaderSize], &header); err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(header, plan.Header) {
		t.Fatal("encoded metadata differs from reference")
	}
	after, err := json.Marshal(plan.Header)
	if err != nil || !bytes.Equal(before, after) {
		t.Fatal("encoding changed the caller's header")
	}
}
