package ipcclient

import (
	"errors"
	"fmt"
	"io"
)

// Measured ASCII RTB1 in little endian; versioned public batch discriminator.
const TelemetryBatchMagic = 0x31425452

// Unverified allocation ceiling: 4096 records, matching the live ring reader.
const TelemetryBatchMaxRecords = 4096

// Measured maximum wire allocation: 312 + 4096*5392 = 22085944 bytes.
const TelemetryBatchMaxBytes = TelemetryBatchHeaderV1Bytes + TelemetryBatchMaxRecords*CycleCaptureRecordV2Bytes

type TelemetryPublication struct {
	Header  TelemetryBatchHeaderV1
	Records []CycleCaptureRecordV2
}

// TelemetryLayoutMismatchError identifies both layouts before any record is
// decoded, preventing an old dump or producer from silently changing meanings.
type TelemetryLayoutMismatchError struct {
	StoredDigest   string
	ExpectedDigest string
}

func (e *TelemetryLayoutMismatchError) Error() string {
	return fmt.Sprintf("telemetry layout digest mismatch: stored=%s expected=%s; V1 dumps require the matching old rtctl", e.StoredDigest, e.ExpectedDigest)
}

func validateTelemetryDigest(stored string) error {
	if stored != CycleCaptureRecordV2LayoutDigest {
		return &TelemetryLayoutMismatchError{StoredDigest: stored, ExpectedDigest: CycleCaptureRecordV2LayoutDigest}
	}
	return nil
}

// ValidateTelemetryHeader rejects incompatible geometry before allocating a body.
// Empty batches carry the resume cursor in LastSequence and FirstSequence zero.
func ValidateTelemetryHeader(h TelemetryBatchHeaderV1) error {
	if err := validateTelemetryDigest(string(h.RecordLayoutDigest[:])); err != nil {
		return err
	}
	hex := func(p []byte) bool {
		for _, c := range p {
			if !(c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
				return false
			}
		}
		return true
	}
	if h.Magic != TelemetryBatchMagic || h.Version != 1 || h.HeaderBytes != TelemetryBatchHeaderV1Bytes ||
		h.RecordBytes != CycleCaptureRecordV2Bytes || h.RecordCount > TelemetryBatchMaxRecords ||
		h.AxisCount < 1 || h.AxisCount > MaxAxes || h.CyclePeriodNs < 250000 || h.CyclePeriodNs > 10000000 ||
		string(h.RecordLayoutDigest[:]) != CycleCaptureRecordV2LayoutDigest || !hex(h.AdapterIncarnation[:]) || !hex(h.DaemonIncarnation[:]) || !hex(h.MachineSha256[:]) || !hex(h.DeploymentSha256[:]) ||
		h.Reserved != 0 || (h.RecordCount == 0 && h.FirstSequence != 0) ||
		(h.RecordCount > 0 && (h.FirstSequence == 0 || h.LastSequence < h.FirstSequence || h.LastSequence-h.FirstSequence != uint64(h.RecordCount)-1)) {
		return errors.New("invalid telemetry batch header")
	}
	return nil
}

// ValidateAfter enforces explicit loss without wrapping uint64 cursors.
func (b TelemetryPublication) ValidateAfter(after uint64) error {
	h := b.Header
	if h.Dropped > ^uint64(0)-after {
		return errors.New("invalid telemetry loss")
	}
	next := after + h.Dropped
	if h.RecordCount == 0 {
		if h.LastSequence != next {
			return errors.New("invalid empty telemetry cursor")
		}
	} else if next == ^uint64(0) || h.FirstSequence != next+1 {
		return errors.New("unreported telemetry loss")
	}
	return nil
}

// ReadTelemetryPublication reads one self-framed batch regardless of transport
// chunk boundaries; a truncated record or unexplained sequence gap fails closed.
func ReadTelemetryPublication(r io.Reader) (TelemetryPublication, error) {
	var b TelemetryPublication
	var raw [TelemetryBatchHeaderV1Bytes]byte
	if _, err := io.ReadFull(r, raw[:]); err != nil {
		return b, err
	}
	h, err := DecodeTelemetryBatchHeaderV1(raw[:])
	if err != nil {
		return b, err
	}
	if err = ValidateTelemetryHeader(h); err != nil {
		return b, err
	}
	b.Header = h
	b.Records = make([]CycleCaptureRecordV2, 0, h.RecordCount)
	var record [CycleCaptureRecordV2Bytes]byte
	for i := uint32(0); i < h.RecordCount; i++ {
		if _, err = io.ReadFull(r, record[:]); err != nil {
			return TelemetryPublication{}, err
		}
		v, e := DecodeCycleCaptureRecordV2(record[:])
		if e != nil {
			return TelemetryPublication{}, e
		}
		if v.Seq != h.FirstSequence+uint64(i) || v.Axes != h.AxisCount {
			return TelemetryPublication{}, errors.New("invalid telemetry record identity")
		}
		b.Records = append(b.Records, v)
	}
	return b, nil
}

// Encode serializes generated layouts only, refusing malformed local batches
// before any bytes are published on HTTP or NATS.
func (b TelemetryPublication) Encode() ([]byte, error) {
	if err := ValidateTelemetryHeader(b.Header); err != nil {
		return nil, err
	}
	if len(b.Records) != int(b.Header.RecordCount) {
		return nil, errors.New("telemetry record count mismatch")
	}
	h := EncodeTelemetryBatchHeaderV1(b.Header)
	raw := make([]byte, 0, TelemetryBatchHeaderV1Bytes+len(b.Records)*CycleCaptureRecordV2Bytes)
	raw = append(raw, h[:]...)
	for i, r := range b.Records {
		if r.Seq != b.Header.FirstSequence+uint64(i) || r.Axes != b.Header.AxisCount {
			return nil, errors.New("invalid telemetry record identity")
		}
		record := EncodeCycleCaptureRecordV2(r)
		if _, err := DecodeCycleCaptureRecordV2(record[:]); err != nil {
			return nil, err
		}
		raw = append(raw, record[:]...)
	}
	return raw, nil
}
