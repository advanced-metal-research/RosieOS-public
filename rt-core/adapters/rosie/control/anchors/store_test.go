package anchors

import (
	"crypto/sha256"
	"errors"
	"fmt"
	"testing"
)

func validRecord() Record {
	identity := "anchor.source=raw_multi_turn_encoder|3\nevidence.mode=multi_turn\n"
	return Record{
		PairID:                   "cell",
		PairRevision:             1,
		CoordinateIdentitySHA256: fmt.Sprintf("%x", sha256.Sum256([]byte(identity))),
		CoordinateIdentity:       identity,
		Axis:                     0,
		Vendor:                   1,
		Product:                  2,
		Revision:                 3,
		Serial:                   4,
		SerialValid:              1,
		HomeEpoch:                1,
		AnchorCounts:             100,
		HomeOffset:               0,
		TurnsCounts:              1000,
		ToleranceCounts:          16,
		DaemonIncarnation:        "0123456789abcdef0123456789abcdef",
		CapturedHostNS:           1,
	}
}

func TestStoreRoundTripUsesExactV3Record(t *testing.T) {
	store := Store{Directory: t.TempDir()}
	record := validRecord()
	if err := store.Save(record); err != nil {
		t.Fatal(err)
	}
	got, err := store.Load(record.Axis)
	if err != nil {
		t.Fatal(err)
	}
	if got != record {
		t.Fatalf("round trip changed record: got %+v want %+v", got, record)
	}
}

func TestQualifyRefusesWholeTurnDifference(t *testing.T) {
	saved := validRecord()
	current := saved
	if err := saved.Qualify(current, true); err != nil {
		t.Fatal(err)
	}
	current.TurnsCounts += 131072
	if err := saved.Qualify(current, true); !errors.Is(err, ErrDisagrees) {
		t.Fatalf("whole-turn difference accepted: %v", err)
	}
}

func TestVerifiedDeltaComparesExactly(t *testing.T) {
	saved := validRecord()
	current := saved
	current.AnchorCounts += 262144
	current.TurnsCounts += 262144
	if err := saved.QualifyVerifiedDelta(current, true); err != nil {
		t.Fatal(err)
	}
	current.TurnsCounts -= 131072
	if err := saved.QualifyVerifiedDelta(current, true); !errors.Is(err, ErrDisagrees) {
		t.Fatalf("whole-turn delta disagreement accepted: %v", err)
	}
}
