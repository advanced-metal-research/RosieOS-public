// Package anchors stores qualified Home evidence. A stored record is an offer
// for explicit inhibited native restoration, never startup motion authority.
package anchors

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"hash/crc32"
	"io"
	"log"
	"os"
	"path/filepath"
	"strings"
)

// Record binds the count frame to the adapter's pair, per-axis count semantics
// and observed drive identity, preventing restoration across pair bindings.
type Record struct {
	PairID                            string
	PairRevision                      uint64
	CoordinateIdentitySHA256          string
	CoordinateIdentity                string
	Axis                              uint32
	Vendor, Product, Revision, Serial uint32
	SerialValid                       uint32
	HomeEpoch                         uint64
	AnchorCounts                      int32
	HomeOffset                        int32
	TurnsCounts                       int32
	ToleranceCounts                   uint64
	DaemonIncarnation                 string
	CapturedHostNS                    int64
}

type Store struct{ Directory string }

// These are stable operator reasons for stored evidence and native restoration.
var (
	ErrMissing   = errors.New("anchor_missing")
	ErrIdentity  = errors.New("anchor_identity_mismatch")
	ErrSource    = errors.New("anchor_source_invalid")
	ErrDisagrees = errors.New("anchor_disagrees")
	ErrIO        = errors.New("anchor_store_io")
	// ErrCurrentReference refuses a rosie-anchor-v5 file left by the removed
	// current-pose zero capture. Home comes only from drive homing (HM35).
	ErrCurrentReference = fmt.Errorf("%w: current-pose reference anchors are not accepted; establish Home with drive homing (HM35)", ErrSource)
)

// storeIO keeps filesystem details in the log and returns a stable reason,
// preventing storage paths from escaping through public restore refusals.
func storeIO(path string, err error) error {
	if err == nil {
		return nil
	}
	log.Printf("anchor_store_io path=%q: %v", path, err)
	return ErrIO
}

func validHex(s string, size int) bool {
	b, err := hex.DecodeString(s)
	return err == nil && len(b) == size && strings.ToLower(s) == s
}

func (r Record) valid() bool {
	return r.PairID != "" && r.PairRevision != 0 && r.CoordinateIdentity != "" && fmt.Sprintf("%x", sha256.Sum256([]byte(r.CoordinateIdentity))) == r.CoordinateIdentitySHA256 && validHex(r.DaemonIncarnation, 16) &&
		r.Axis < 16 && r.SerialValid <= 1 && r.HomeEpoch > 0 && r.CapturedHostNS > 0 &&
		r.ToleranceCounts <= uint64(^uint32(0)>>1)
}

func (r Record) payload() string {
	payload := fmt.Sprintf("rosie-anchor-v3 %s %d %d %d %d %d %d %d %d %d %d %d %d %s %d %x %x %d\n",
		r.CoordinateIdentitySHA256, r.Axis, r.Vendor, r.Product, r.Revision, r.Serial, r.SerialValid,
		r.HomeEpoch, r.AnchorCounts, r.HomeOffset, r.TurnsCounts, 0,
		r.ToleranceCounts, r.DaemonIncarnation, r.CapturedHostNS, r.CoordinateIdentity, r.PairID, r.PairRevision)
	return payload
}

func (s Store) Path(axis uint32) string {
	return filepath.Join(s.Directory, fmt.Sprintf("axis-%d.anchor", axis))
}

// Save syncs a private temporary file before atomic replacement, then syncs the
// directory. A crash cannot expose a partially written replacement as evidence.
func (s Store) Save(r Record) error {
	if s.Directory == "" || !r.valid() {
		return ErrSource
	}
	if err := os.MkdirAll(s.Directory, 0700); err != nil {
		return storeIO(s.Directory, err)
	}
	f, err := os.CreateTemp(s.Directory, ".anchor-*")
	if err != nil {
		return storeIO(s.Directory, err)
	}
	defer os.Remove(f.Name())
	defer f.Close()
	payload := r.payload()
	if _, err = fmt.Fprintf(f, "%s%08x\n", payload, crc32.ChecksumIEEE([]byte(payload))); err != nil {
		return storeIO(f.Name(), err)
	}
	// A deployment may explicitly provision a dedicated setgid directory for
	// the adapter writer and native reader. Preserve private 0600 files by
	// default; grant group read only in that restricted, non-writable directory.
	info, err := os.Stat(s.Directory)
	if err != nil {
		return storeIO(s.Directory, err)
	}
	if info.Mode()&os.ModeSetgid != 0 && info.Mode().Perm() == 0750 {
		if err = f.Chmod(0640); err != nil {
			return storeIO(f.Name(), err)
		}
	}
	if err = f.Sync(); err != nil {
		return storeIO(f.Name(), err)
	}
	if err = f.Close(); err != nil {
		return storeIO(f.Name(), err)
	}
	if err = os.Rename(f.Name(), s.Path(r.Axis)); err != nil {
		return storeIO(s.Path(r.Axis), err)
	}
	d, err := os.Open(s.Directory)
	if err != nil {
		return storeIO(s.Directory, err)
	}
	defer d.Close()
	return storeIO(s.Directory, d.Sync())
}

// Load checks the complete canonical record and its checksum. The checksum
// detects file alteration; filesystem permissions protect against replacement
// by an unauthorized writer. Native independently checks identity and source.
func (s Store) Load(axis uint32) (Record, error) {
	var r Record
	if s.Directory == "" || axis >= 16 {
		return r, ErrMissing
	}
	f, err := os.Open(s.Path(axis))
	if errors.Is(err, os.ErrNotExist) {
		return r, ErrMissing
	}
	if err != nil {
		return r, storeIO(s.Path(axis), err)
	}
	defer f.Close()
	// Unverified storage bound: 16384 bytes, including the canonical field table.
	raw, err := io.ReadAll(io.LimitReader(f, 16385))
	if err != nil {
		return r, storeIO(s.Path(axis), err)
	}
	parts := bytes.Split(raw, []byte{'\n'})
	if len(raw) > 16384 || len(parts) != 3 || len(parts[2]) != 0 {
		return r, ErrSource
	}
	payloadBytes := append(append([]byte(nil), parts[0]...), '\n')
	if string(parts[1]) != fmt.Sprintf("%08x", crc32.ChecksumIEEE(payloadBytes)) {
		return r, ErrSource
	}
	// Old records lack the independent adapter pair binding. Require Home rather
	// than reinterpret their configuration or pair-bearing coordinate digest.
	if bytes.HasPrefix(parts[0], []byte("rosie-anchor-v1 ")) || bytes.HasPrefix(parts[0], []byte("rosie-anchor-v2 ")) {
		return r, ErrIdentity
	}
	if bytes.HasPrefix(parts[0], []byte("rosie-anchor-v5 ")) {
		return r, ErrCurrentReference
	}
	var magic, encoded, encodedPair string
	var retiredPeriod uint32
	n, err := fmt.Sscan(string(parts[0]), &magic, &r.CoordinateIdentitySHA256, &r.Axis, &r.Vendor,
		&r.Product, &r.Revision, &r.Serial, &r.SerialValid, &r.HomeEpoch, &r.AnchorCounts,
		&r.HomeOffset, &r.TurnsCounts, &retiredPeriod, &r.ToleranceCounts, &r.DaemonIncarnation, &r.CapturedHostNS, &encoded, &encodedPair, &r.PairRevision)
	decoded, decodeErr := hex.DecodeString(encoded)
	r.CoordinateIdentity = string(decoded)
	pair, pairErr := hex.DecodeString(encodedPair)
	r.PairID = string(pair)
	if err != nil || decodeErr != nil || pairErr != nil || n != 19 || magic != "rosie-anchor-v3" || retiredPeriod != 0 || !r.valid() || r.Axis != axis {
		return Record{}, ErrSource
	}
	payload := r.payload()
	want := fmt.Sprintf("%s%08x\n", payload, crc32.ChecksumIEEE([]byte(payload)))
	if string(raw) != want {
		return Record{}, ErrSource
	}
	return r, nil
}

// Qualify compares the independent encoder frame exactly. This offer alone
// never grants Home validity; native repeats command, source and epoch checks.
func (r Record) Qualify(current Record, sourceValid bool) error {
	if !r.MatchesPair(current.PairID, current.PairRevision) || r.CoordinateIdentitySHA256 == "" || r.CoordinateIdentitySHA256 != current.CoordinateIdentitySHA256 || r.Axis != current.Axis ||
		r.Vendor != current.Vendor || r.Product != current.Product || r.Revision != current.Revision ||
		r.SerialValid != current.SerialValid || (r.SerialValid != 0 && r.Serial != current.Serial) ||
		r.ToleranceCounts != current.ToleranceCounts {
		return ErrIdentity
	}
	if !sourceValid {
		return ErrSource
	}
	delta := int64(r.TurnsCounts) - int64(current.TurnsCounts)
	if delta < 0 {
		delta = -delta
	}
	if uint64(delta) > r.ToleranceCounts {
		return ErrDisagrees
	}
	return nil
}

// QualifyVerifiedDelta is an offer for the distinct explicit operation only.
// The original HM35 v3 record is never rewritten; native independently obtains a
// bracketed stationary sample and checks hard limits before binding saved L0.
// There is no delta cap: any position within the joint's hard limits may be
// restored. The full signed encoder and command deltas preserve turns and are
// compared exactly.
func (r Record) QualifyVerifiedDelta(current Record, sourceValid bool) error {
	if !r.MatchesPair(current.PairID, current.PairRevision) || r.CoordinateIdentitySHA256 == "" || r.CoordinateIdentitySHA256 != current.CoordinateIdentitySHA256 || r.CoordinateIdentity != current.CoordinateIdentity || r.Axis != current.Axis || r.Vendor != current.Vendor || r.Product != current.Product || r.Revision != current.Revision || r.SerialValid != current.SerialValid || (r.SerialValid != 0 && r.Serial != current.Serial) || r.ToleranceCounts != current.ToleranceCounts {
		return ErrIdentity
	}
	if !sourceValid || !r.valid() {
		return ErrSource
	}
	if r.HomeOffset != current.HomeOffset {
		return ErrDisagrees
	}
	dC, dE := int64(current.AnchorCounts)-int64(r.AnchorCounts), int64(current.TurnsCounts)-int64(r.TurnsCounts)
	delta := dC - dE
	if delta < 0 {
		delta = -delta
	}
	if uint64(delta) > r.ToleranceCounts {
		return ErrDisagrees
	}
	return nil
}

// MatchesPair checks the adapter-owned binding independently of the compiled
// digest, preventing a matching machine from authorizing another pair's anchor.
func (r Record) MatchesPair(id string, revision uint64) bool {
	return r.PairID != "" && r.PairRevision != 0 && r.PairID == id && r.PairRevision == revision
}
