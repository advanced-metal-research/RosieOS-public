package ipcclient

import (
	"encoding/binary"
	"testing"
)

// Independent absolute offsets protect the additive I/O status from a matching
// encoder/decoder mistake that could report intent as physical readback.
func TestCellIOStatusWireLayout(t *testing.T) {
	var bytes [3400]byte
	binary.LittleEndian.PutUint64(bytes[936:944], 10000000)
	binary.LittleEndian.PutUint64(bytes[944:952], 10)
	binary.LittleEndian.PutUint32(bytes[960:964], 1)
	binary.LittleEndian.PutUint32(bytes[964:968], 1)
	copy(bytes[968:1000], "consent")
	binary.LittleEndian.PutUint64(bytes[1000:1008], 9000000)
	binary.LittleEndian.PutUint32(bytes[1012:1016], 1)
	copy(bytes[1864:1896], "gas")
	binary.LittleEndian.PutUint64(bytes[1896:1904], 15000000)
	binary.LittleEndian.PutUint32(bytes[1912:1916], 1)
	binary.LittleEndian.PutUint32(bytes[1924:1928], 1)
	state, err := DecodeStatusSnapshotV1Frame(bytes[:])
	if err != nil {
		t.Fatal(err)
	}
	if state.Io.TimeNs != 10000000 || state.Io.Cycle != 10 || state.Io.InputCount != 1 || state.Io.OutputCount != 1 || state.Io.Inputs[0].ObservedNs != 9000000 || state.Io.Inputs[0].Value != 0 || state.Io.Inputs[0].Valid != 1 {
		t.Fatal(state.Io)
	}
	o := state.Io.Outputs[0]
	if o.ExpiryNs != 15000000 || o.Intent != 1 || o.Commanded != 0 || o.Readback != 0 || o.Valid != 1 {
		t.Fatal(o)
	}
	if EncodeStatusSnapshotV1(state) != bytes {
		t.Fatal("I/O status changed on re-encoding")
	}
}
