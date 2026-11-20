//go:build linux

package ipcclient

import (
	"context"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"sync"
	"sync/atomic"
	"unicode/utf8"
	"unsafe"

	"golang.org/x/sys/unix"
)

// Measured native geometry: metadata page, 272-byte record prefix, 288 bytes/axis.
const TelemetryHeaderBytes = 4096

// Unverified native resident admission ceiling: 2 GiB including metadata.
const TelemetryMaxBytes = 2 * 1024 * 1024 * 1024

// Unverified reader allocation ceiling: 4096 complete records per batch.
const telemetryBatchLimit = 4096

// TelemetryReader owns a read-only mapping. Close serializes with reads so a
// concurrent consumer cannot access unmapped memory; no lock touches the daemon.
type TelemetryReader struct {
	mu                                              sync.Mutex
	mapping                                         []byte
	incarnation                                     [2]uint64
	DaemonIncarnation                               string
	Capacity, CyclePeriodNS, AxisCount, RecordBytes uint64
}

// OpenTelemetry maps this connection's live observer ring off the native lanes.
// The immutable ring identity must match the native handshake; a live connection
// alone cannot detect replacement of either observer endpoint.
func (c *Client) OpenTelemetry() (*TelemetryReader, error) {
	if err := c.Err(); err != nil {
		return nil, err
	}
	r, err := OpenTelemetry(filepath.Join(filepath.Dir(c.options.SocketPath), "telemetry.ring"))
	if err != nil {
		return nil, err
	}
	if err = c.Err(); err != nil {
		r.Close()
		return nil, err
	}
	if r.DaemonIncarnation != c.Incarnation() || r.incarnation == [2]uint64{} {
		r.Close()
		return nil, ErrTelemetryIdentity
	}
	return r, nil
}

// Written returns a coherent head for future-cursor validation; busy metadata
// fails explicitly instead of being mistaken for an empty ring.
func (r *TelemetryReader) Written() (uint64, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.mapping == nil {
		return 0, os.ErrClosed
	}
	word := func(offset uint64) uint64 { return atomic.LoadUint64((*uint64)(unsafe.Pointer(&r.mapping[offset]))) }
	if !r.sameIncarnation(word) {
		return 0, ErrTelemetryIdentity
	}
	w, _, ok := sampleTelemetryHeader(word, r.Capacity)
	if !r.sameIncarnation(word) {
		return 0, ErrTelemetryIdentity
	}
	if !ok {
		return 0, ErrTelemetryBusy
	}
	return w, nil
}

// OpenTelemetry opens runtime/telemetry.ring, validates the generated layout and
// sealed extent before mapping, and fails closed on stale/incompatible mappings.
func OpenTelemetry(path string) (*TelemetryReader, error) {
	if runtime.GOARCH != "amd64" && runtime.GOARCH != "arm64" {
		return nil, errors.New("unsupported telemetry architecture")
	}
	f, err := os.Open(path)
	if err != nil {
		f, err = openTelemetryDescriptor(path + ".sock")
		if err != nil {
			return nil, err
		}
	}
	defer f.Close()
	return mapTelemetry(f)
}

// The protected native socket transfers an O_RDONLY fd across service UIDs;
// /proc permission restrictions must not force rt-control to share daemon UID.
func openTelemetryDescriptor(path string) (*os.File, error) {
	fd, err := unix.Socket(unix.AF_UNIX, unix.SOCK_SEQPACKET|unix.SOCK_CLOEXEC, 0)
	if err != nil {
		return nil, err
	}
	defer unix.Close(fd)
	// Unverified off-cycle descriptor admission timeout: 2 seconds.
	if err = unix.SetsockoptTimeval(fd, unix.SOL_SOCKET, unix.SO_RCVTIMEO, &unix.Timeval{Sec: 2}); err != nil {
		return nil, err
	}
	if err = unix.SetsockoptTimeval(fd, unix.SOL_SOCKET, unix.SO_SNDTIMEO, &unix.Timeval{Sec: 2}); err != nil {
		return nil, err
	}
	if err = unix.Connect(fd, &unix.SockaddrUnix{Name: path}); err != nil {
		return nil, err
	}
	data, ancillary := make([]byte, 1), make([]byte, unix.CmsgSpace(4))
	n, controlBytes, flags, _, err := unix.Recvmsg(fd, data, ancillary, unix.MSG_CMSG_CLOEXEC)
	if err != nil {
		return nil, err
	}
	messages, err := unix.ParseSocketControlMessage(ancillary[:controlBytes])
	if err != nil {
		return nil, err
	}
	var descriptors []int
	for _, message := range messages {
		rights, rightsErr := unix.ParseUnixRights(&message)
		if rightsErr != nil {
			err = rightsErr
			continue
		}
		descriptors = append(descriptors, rights...)
	}
	if err != nil || n != 1 || data[0] != 'T' || flags&(unix.MSG_CTRUNC|unix.MSG_TRUNC) != 0 || len(descriptors) != 1 {
		for _, descriptor := range descriptors {
			unix.Close(descriptor)
		}
		return nil, errors.New("invalid telemetry descriptor response")
	}
	access, err := unix.FcntlInt(uintptr(descriptors[0]), unix.F_GETFL, 0)
	if err != nil || access&unix.O_ACCMODE != unix.O_RDONLY {
		unix.Close(descriptors[0])
		return nil, errors.New("telemetry descriptor is not read-only")
	}
	return os.NewFile(uintptr(descriptors[0]), path), nil
}

func mapTelemetry(f *os.File) (*TelemetryReader, error) {
	st, err := f.Stat()
	if err != nil {
		return nil, err
	}
	if st.Size() < TelemetryHeaderBytes || st.Size() > TelemetryMaxBytes {
		return nil, errors.New("invalid telemetry size")
	}
	seals, err := unix.FcntlInt(f.Fd(), unix.F_GET_SEALS, 0)
	if err != nil || seals&(unix.F_SEAL_GROW|unix.F_SEAL_SHRINK) != (unix.F_SEAL_GROW|unix.F_SEAL_SHRINK) {
		return nil, errors.New("telemetry extent is not sealed")
	}
	var header [TelemetryHeaderBytes]byte
	if _, err = f.ReadAt(header[:], 0); err != nil {
		return nil, err
	}
	if err := validateTelemetryDigest(string(header[128:192])); err != nil {
		return nil, err
	}
	word := func(n int) uint64 { return binary.LittleEndian.Uint64(header[n*8:]) }
	r := &TelemetryReader{Capacity: word(3), RecordBytes: word(4), CyclePeriodNS: word(5), AxisCount: word(6)}
	r.incarnation = [2]uint64{word(9), word(10)}
	r.DaemonIncarnation = hex.EncodeToString(header[72:88])
	if word(0) != 0x31474e4952545452 || word(1) != VersionMajor|uint64(VersionMinor)<<32 ||
		word(2) != uint64(st.Size()) || r.AxisCount < 1 || r.AxisCount > MaxAxes ||
		r.RecordBytes != uint64(CycleCaptureRecordV2AxOffset)+r.AxisCount*CycleCaptureAxisV2Bytes ||
		r.Capacity < 1 || r.Capacity > TelemetryMaxBytes/(r.RecordBytes+8) ||
		word(2) != TelemetryHeaderBytes+r.Capacity*(r.RecordBytes+8) || r.CyclePeriodNS < 250000 || r.CyclePeriodNS > 10000000 ||
		string(header[128:192]) != CycleCaptureRecordV2LayoutDigest {
		return nil, errors.New("incompatible telemetry layout")
	}
	r.mapping, err = unix.Mmap(int(f.Fd()), 0, int(st.Size()), unix.PROT_READ, unix.MAP_SHARED)
	if err != nil {
		return nil, err
	}
	return r, nil
}

func (r *TelemetryReader) Close() error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.mapping == nil {
		return nil
	}
	err := unix.Munmap(r.mapping)
	r.mapping = nil
	return err
}

// TelemetryBatch reports exact cursor loss and the ring-wide count of overwritten
// records. NextSequence can be passed directly to ReadFrom, including busy reads.
type TelemetryBatch struct {
	Records                                   []CycleCaptureRecordV2
	NextSequence, Drops, Overwritten, Written uint64
}

// TelemetryCorruptSlotError identifies a committed record that failed validation.
// The cursor stays at Sequence; callers must stop or explicitly skip it, avoiding
// silent evidence loss and endless retries mistaken for an empty ring.
type TelemetryCorruptSlotError struct {
	Sequence uint64
}

func (e *TelemetryCorruptSlotError) Error() string {
	return fmt.Sprintf("invalid telemetry record at sequence %d", e.Sequence)
}

// ErrTelemetryBusy identifies a transient incoherent header; retrying preserves
// the cursor and prevents busy metadata from masquerading as an empty ring.
var ErrTelemetryBusy = errors.New("busy telemetry metadata")

// ErrTelemetryIdentity refuses a missing or foreign ring identity, preventing
// records from being labelled with a different daemon's native handshake.
var ErrTelemetryIdentity = errors.New("telemetry incarnation mismatch")

// sameIncarnation checks immutable header words for every read, including an
// empty batch; a changed header never supplies records or cursor evidence.
func (r *TelemetryReader) sameIncarnation(word func(uint64) uint64) bool {
	return word(72) == r.incarnation[0] && word(80) == r.incarnation[1]
}

// sampleTelemetryHeader bounds retries of separately published counters; an
// inconsistent pair must never manufacture cursor loss or an empty-ring claim.
func sampleTelemetryHeader(word func(uint64) uint64, capacity uint64) (uint64, uint64, bool) {
	// Unverified observer retry bound: eight header samples per call.
	for attempt := 0; attempt < 8; attempt++ {
		written, overwritten := word(56), word(64)
		expected := uint64(0)
		if written > capacity {
			expected = written - capacity
		}
		if overwritten == expected {
			return written, overwritten, true
		}
		runtime.Gosched()
	}
	return 0, 0, false
}

// ReadFrom copies committed records once each; a busy/torn slot ends this batch
// without advancing its cursor or exposing partial data. Sequence zero starts
// at the oldest retained record; other cursors report every skipped sequence.
// A torn Written/Overwritten pair retries eight times with scheduler yields;
// exhaustion returns ErrTelemetryBusy and the original cursor, not evidence of
// an empty ring. A corrupt committed slot returns TelemetryCorruptSlotError and
// keeps its cursor so the caller decides whether to stop or skip that record.
func (r *TelemetryReader) ReadFrom(sequence uint64, limit int) (TelemetryBatch, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	b := TelemetryBatch{NextSequence: sequence}
	originalCursor := sequence
	if r.mapping == nil {
		return b, os.ErrClosed
	}
	if limit < 1 || limit > telemetryBatchLimit {
		return b, errors.New("invalid telemetry batch limit")
	}
	word := func(offset uint64) uint64 { return atomic.LoadUint64((*uint64)(unsafe.Pointer(&r.mapping[offset]))) }
	if !r.sameIncarnation(word) {
		return b, ErrTelemetryIdentity
	}
	// A descheduled observer may span several publications between these words.
	// Accept only a consistent committed pair; a busy pair preserves the cursor.
	var coherent bool
	b.Written, b.Overwritten, coherent = sampleTelemetryHeader(word, r.Capacity)
	if !coherent {
		return TelemetryBatch{NextSequence: sequence}, ErrTelemetryBusy
	}
	oldest := uint64(1)
	if b.Written >= r.Capacity {
		oldest = b.Written - r.Capacity + 1
	}
	if sequence == 0 {
		sequence = oldest
	}
	if sequence < oldest {
		b.Drops = oldest - sequence
		sequence = oldest
	}
	b.NextSequence = sequence
	for len(b.Records) < limit && sequence <= b.Written {
		offset := TelemetryHeaderBytes + ((sequence-1)%r.Capacity)*(r.RecordBytes+8)
		if word(offset) != sequence*2 {
			break
		}
		var raw [CycleCaptureRecordV2Bytes]byte
		for i := uint64(0); i < r.RecordBytes; i += 8 {
			binary.LittleEndian.PutUint64(raw[i:], word(offset+8+i))
		}
		if word(offset) != sequence*2 {
			break
		}
		record, err := DecodeCycleCaptureRecordV2(raw[:])
		if err != nil || record.Seq != sequence || uint64(record.Axes) != r.AxisCount {
			return b, &TelemetryCorruptSlotError{Sequence: sequence}
		}
		b.Records = append(b.Records, record)
		sequence++
		b.NextSequence = sequence
	}
	if !r.sameIncarnation(word) {
		return TelemetryBatch{NextSequence: originalCursor}, ErrTelemetryIdentity
	}
	return b, nil
}

// MarkTelemetry admits a bounded UTF-8 label on the native command lane.
// A receipt acknowledges queued observation work, not successful disk output.
func (c *Client) MarkTelemetry(ctx context.Context, label string) (uint64, error) {
	if len(label) < 1 || len(label) > 128 || !utf8.ValidString(label) {
		return 0, errors.New("telemetry label must contain 1..128 UTF-8 bytes")
	}
	mark := TelemetryMarkV1{Length: uint32(len(label))}
	copy(mark.Label[:], label)
	raw := EncodeTelemetryMarkV1(mark)
	seq, err := c.command(ctx, MsgTelemetryMark, raw[:])
	if err == nil {
		_, err = c.WaitResult(ctx, seq)
	}
	return seq, err
}
