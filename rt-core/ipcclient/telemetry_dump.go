package ipcclient

import (
	"bufio"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
)

// TelemetryDumpIndex describes raw, full configured-axis records and any loss.
// The generated layout digest prevents an old converter from misreading fields.
type TelemetryDumpIndex struct {
	File                string                 `json:"file"`
	Result              string                 `json:"result"`
	Triggers            []TelemetryDumpTrigger `json:"triggers"`
	PostTriggerRecords  uint64                 `json:"post_trigger_records"`
	ProtocolMajor       uint32                 `json:"protocol_major"`
	ProtocolMinor       uint32                 `json:"protocol_minor"`
	LayoutDigest        string                 `json:"layout_digest"`
	CyclePeriodNS       uint64                 `json:"cycle_period_ns"`
	AxisCount           uint32                 `json:"axis_count"`
	RecordBytes         uint32                 `json:"record_bytes"`
	FirstSequence       uint64                 `json:"first_sequence"`
	LastSequence        uint64                 `json:"last_sequence"`
	WindowFirstSequence uint64                 `json:"window_first_sequence"`
	WindowLastSequence  uint64                 `json:"window_last_sequence"`
	TriggerSequence     uint64                 `json:"trigger_sequence"`
	RecordCount         uint64                 `json:"record_count"`
	MissingRecords      uint64                 `json:"missing_records"`
	OverwrittenRecords  uint64                 `json:"overwritten_records"`
	Trigger             string                 `json:"trigger"`
	Label               string                 `json:"label"`
	AbsoluteKeys        [][]string             `json:"absolute_keys"`
}

// TelemetryDumpTrigger preserves each observed reason/axis/sequence; nil Axis
// denotes a group event, preventing a Stop or mark from being assigned to axis 0.
type TelemetryDumpTrigger struct {
	Reason      string  `json:"reason"`
	FaultReason uint32  `json:"fault_reason"`
	Axis        *uint32 `json:"axis"`
	Sequence    uint64  `json:"sequence"`
	Label       string  `json:"label"`
}

// CheckCompatible is the one rule for decoding a dump another build wrote. The
// record layout digest proves every field is read at the offset it was written
// at, so a dump whose protocol minor differs from this build's is read when its
// digest is this build's (1.13 and 1.14 share it, so one decoder reads the whole
// dump history of a Rosie 1430 arm), and a foreign digest is refused with
// TelemetryLayoutMismatchError before any record is decoded: its fields would be
// misread. A different major is a different contract and is refused.
func (index TelemetryDumpIndex) CheckCompatible() error {
	if err := validateTelemetryDigest(index.LayoutDigest); err != nil {
		return err
	}
	if index.ProtocolMajor != VersionMajor {
		return fmt.Errorf("telemetry dump protocol %d.%d: this build reads major %d", index.ProtocolMajor, index.ProtocolMinor, VersionMajor)
	}
	return nil
}

// CompatibilityNote says when a dump was read across protocol minors under
// CheckCompatible, so a report names the decoder that read it; empty otherwise.
func (index TelemetryDumpIndex) CompatibilityNote() string {
	if index.ProtocolMinor == VersionMinor {
		return ""
	}
	return fmt.Sprintf("protocol %d.%d read with the %d.%d decoder: same record layout digest", index.ProtocolMajor, index.ProtocolMinor, VersionMajor, VersionMinor)
}

// ReadTelemetryDump bounds all file-derived lengths before reading records and
// validates monotonic sequences/counts, refusing truncated or incompatible
// dumps; compatibility across protocol minors is CheckCompatible's rule.
func ReadTelemetryDump(path string, visit func(CycleCaptureRecordV2, TelemetryDumpIndex) error) (TelemetryDumpIndex, error) {
	var index TelemetryDumpIndex
	file, err := os.Open(path)
	if err != nil {
		return index, err
	}
	defer file.Close()
	// Measured: a 220 MB Stop dump from a Rosie 1430 arm holds 99,957-99,991 records
	// (2026-10-06/07); one 1 MiB buffer replaces a read call per record.
	f := bufio.NewReaderSize(file, 1<<20)
	var prefix [16]byte
	if _, err = io.ReadFull(f, prefix[:]); err != nil {
		return index, err
	}
	headerBytes, jsonBytes := binary.LittleEndian.Uint32(prefix[8:]), binary.LittleEndian.Uint32(prefix[12:])
	// Measured native envelopes: original 65536 and coalesced 262144 byte headers.
	if string(prefix[:8]) != "RTTDMP1\n" || (headerBytes != 65536 && headerBytes != 262144) || jsonBytes == 0 || jsonBytes > headerBytes-16 {
		return index, errors.New("invalid telemetry dump header")
	}
	header := make([]byte, headerBytes-16)
	if _, err = io.ReadFull(f, header); err != nil {
		return index, err
	}
	if err = json.Unmarshal(header[:jsonBytes], &index); err != nil {
		return index, err
	}
	if err = index.CheckCompatible(); err != nil {
		return index, err
	}
	if index.AxisCount < 1 || index.AxisCount > MaxAxes ||
		index.CyclePeriodNS < 250000 || index.CyclePeriodNS > 10000000 ||
		index.RecordBytes != CycleCaptureRecordV2AxOffset+index.AxisCount*CycleCaptureAxisV2Bytes ||
		index.WindowFirstSequence < 1 || index.WindowLastSequence < index.WindowFirstSequence ||
		index.RecordCount > index.WindowLastSequence-index.WindowFirstSequence+1 ||
		index.MissingRecords != index.WindowLastSequence-index.WindowFirstSequence+1-index.RecordCount ||
		len(index.AbsoluteKeys) != int(index.AxisCount) {
		return index, errors.New("incompatible telemetry dump layout/counts")
	}
	for _, keys := range index.AbsoluteKeys {
		if len(keys) > 16 {
			return index, errors.New("too many absolute keys")
		}
	}
	st, err := file.Stat()
	if err != nil {
		return index, err
	}
	if uint64(st.Size()-int64(headerBytes))/uint64(index.RecordBytes) != index.RecordCount ||
		uint64(st.Size()) != uint64(headerBytes)+index.RecordCount*uint64(index.RecordBytes) {
		return index, errors.New("truncated or trailing telemetry records")
	}
	var previous uint64
	for n := uint64(0); n < index.RecordCount; n++ {
		var raw [CycleCaptureRecordV2Bytes]byte
		if _, err = io.ReadFull(f, raw[:index.RecordBytes]); err != nil {
			return index, err
		}
		record, err := DecodeCycleCaptureRecordV2(raw[:])
		if err != nil || record.Axes != index.AxisCount || record.Seq <= previous ||
			record.Seq < index.WindowFirstSequence || record.Seq > index.WindowLastSequence ||
			(n == 0 && record.Seq != index.FirstSequence) || (n+1 == index.RecordCount && record.Seq != index.LastSequence) {
			return index, fmt.Errorf("invalid telemetry record %d", n)
		}
		previous = record.Seq
		if visit != nil {
			if err = visit(record, index); err != nil {
				return index, err
			}
		}
	}
	if index.RecordCount == 0 && (index.FirstSequence != 0 || index.LastSequence != 0) {
		return index, errors.New("nonzero empty telemetry sequences")
	}
	return index, nil
}
