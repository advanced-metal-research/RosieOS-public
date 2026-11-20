//go:build linux

package ipcclient

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"time"

	"golang.org/x/sys/unix"
)

const (
	applicationGrantRegionBytes = FastControlRegionBytes
	applicationGrantMagic       = FastControlMagic
	applicationGrantRequestAt   = FastControlGrantOffset
	applicationGrantObserveAt   = FastControlObservationOffset
	// Unverified absolute cell lease ceiling: 10 s; native enforces the compiled lower ceiling.
	applicationGrantAbsoluteMaxLease = uint64(10000000000)
	// Go-local receipt timeout from original call entry, including lock waits.
	// It only shortens the caller's absolute deadline. Timeout after publication
	// is uncertain and requires Stop/disconnect; it never extends the native cap.
	applicationGrantReceiptTimeoutNS = uint64(250 * time.Millisecond)
	// Go-local refusal of an older observation, independent of the native lease.
	// This unqualified host policy is not an RT latency or acquisition-age guarantee.
	applicationGrantObservationMaxAgeNS = uint64(100 * time.Millisecond)
)

var ErrApplicationGrantUnavailable = errors.New("RTCore application grant region unavailable")
var ErrFastObservationUnavailable = errors.New("RTCore fast observation unavailable or stale")

type GrantIdentity struct {
	CapabilityID [16]byte
	Generation   uint64
}

type GrantReceipt struct {
	Identity           GrantIdentity
	RequestSequence    uint64
	ConnectionID       uint64
	NativeGeneration   uint64
	ConfigurationEpoch uint64
	HomeEpoch          uint64
	DeadlineNS         uint64
}

// GrantRejectedError is a correlated explicit native rejection. The observation
// can retain an older active identity, so it is not an echo of the failed request.
type GrantRejectedError struct {
	RequestSequence uint64
	Reason          uint32
}

func (e *GrantRejectedError) Error() string {
	return fmt.Sprintf("RTCore rejected application grant request %d (reason %d)", e.RequestSequence, e.Reason)
}

// GrantUncertainError means a published request has no usable acknowledgement.
// The caller must Stop or disconnect and clear its application grant. It must
// never interpret cancellation, expiry or a lost receipt as native rejection.
type GrantUncertainError struct{ Cause error }

func (e *GrantUncertainError) Error() string {
	return "RTCore application grant outcome uncertain: " + e.Cause.Error()
}
func (e *GrantUncertainError) Unwrap() error { return e.Cause }

// HostMonotonicNS is the daemon's same-host clock domain. Compute a grant's
// absolute expiry at application ingress, before waiting on work or locks.
func HostMonotonicNS() uint64 { return monotonicNS() }

type applicationGrantRegion struct {
	data           []byte
	header         FastHeader
	request        sharedWords
	observation    sharedWords
	jogControl     sharedWords
	jogUpdate      sharedWords
	jogObservation sharedWords
	// Path jog (protocol 1.14): one acknowledged chunk at a time, and the
	// native observation that acknowledges it.
	pathSamples     sharedWords
	pathObservation sharedWords
}

func mapApplicationGrantRegion(fd int, digest []byte, axes uint32, cycle time.Duration) (_ *applicationGrantRegion, err error) {
	var stat unix.Stat_t
	if err := unix.Fstat(fd, &stat); err != nil {
		return nil, err
	}
	if stat.Size != applicationGrantRegionBytes {
		return nil, errors.New("invalid RTCore application grant region size")
	}
	seals, err := unix.FcntlInt(uintptr(fd), unix.F_GET_SEALS, 0)
	const requiredSeals = unix.F_SEAL_GROW | unix.F_SEAL_SHRINK | unix.F_SEAL_SEAL
	if err != nil || seals&requiredSeals != requiredSeals {
		return nil, errors.New("RTCore application grant region requires immutable size seals")
	}
	mapping, err := unix.Mmap(fd, 0, applicationGrantRegionBytes, unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err != nil {
		return nil, err
	}
	defer func() {
		if err != nil {
			_ = unix.Munmap(mapping)
		}
	}()
	header, ok := DecodeFastHeader(mapping[:FastHeaderBytes])
	if !ok || header.Magic != applicationGrantMagic || header.VersionMajor != VersionMajor || header.VersionMinor != VersionMinor ||
		header.RegionBytes != applicationGrantRegionBytes || header.DaemonIncarnation == [16]byte{} || header.ConnectionId == 0 ||
		header.Reserved != 0 || axes == 0 || axes > maxAxes || header.ConfiguredMask != (uint32(1)<<axes)-1 || header.CycleNs != uint64(cycle) {
		return nil, errors.New("invalid RTCore application grant header/identity")
	}
	if len(digest) != 32 || !bytes.Equal(header.ConfigurationDigestLo[:], digest[:16]) || !bytes.Equal(header.ConfigurationDigestHi[:], digest[16:]) {
		return nil, errors.New("RTCore configuration/backend identity mismatch in application grant header")
	}
	// These bytes are immutable reserved storage. Concurrent request/observation
	// words are deliberately excluded from all non-atomic reads.
	for _, span := range [][2]int{{FastHeaderBytes, applicationGrantRequestAt},
		{applicationGrantRequestAt + 8 + GrantRequestBytes, applicationGrantObserveAt},
		{applicationGrantObserveAt + 8 + GrantObservationBytes, FastControlJogControlOffset},
		{FastControlJogControlOffset + 8 + JogControlBytes, FastControlJogUpdateOffset},
		{FastControlJogUpdateOffset + 8 + JogUpdateBytes, FastControlJogObservationOffset},
		{FastControlJogObservationOffset + 8 + JogObservationBytes, FastControlPathSamplesOffset},
		{FastControlPathSamplesOffset + 8 + PathSamplesBytes, FastControlPathObservationOffset},
		{FastControlPathObservationOffset + 8 + PathObservationBytes, applicationGrantRegionBytes}} {
		for _, value := range mapping[span[0]:span[1]] {
			if value != 0 {
				return nil, errors.New("invalid RTCore application grant reserved storage")
			}
		}
	}
	request, status := bindSharedWords(mapping[applicationGrantRequestAt:applicationGrantRequestAt+8+GrantRequestBytes], GrantRequestBytes/8)
	if status != sharedWordsOK {
		return nil, fmt.Errorf("invalid RTCore grant publication region: %d", status)
	}
	observation, status := bindSharedWords(mapping[applicationGrantObserveAt:applicationGrantObserveAt+8+GrantObservationBytes], GrantObservationBytes/8)
	if status != sharedWordsOK {
		return nil, fmt.Errorf("invalid RTCore grant observation region: %d", status)
	}
	jogControl, status := bindSharedWords(mapping[FastControlJogControlOffset:FastControlJogControlOffset+8+JogControlBytes], JogControlBytes/8)
	if status != sharedWordsOK {
		return nil, errors.New("invalid jog control slot")
	}
	jogUpdate, status := bindSharedWords(mapping[FastControlJogUpdateOffset:FastControlJogUpdateOffset+8+JogUpdateBytes], JogUpdateBytes/8)
	if status != sharedWordsOK {
		return nil, errors.New("invalid jog update slot")
	}
	jogObservation, status := bindSharedWords(mapping[FastControlJogObservationOffset:FastControlJogObservationOffset+8+JogObservationBytes], JogObservationBytes/8)
	if status != sharedWordsOK {
		return nil, errors.New("invalid jog observation slot")
	}
	pathSamples, status := bindSharedWords(mapping[FastControlPathSamplesOffset:FastControlPathSamplesOffset+8+PathSamplesBytes], PathSamplesBytes/8)
	if status != sharedWordsOK {
		return nil, errors.New("invalid path samples slot")
	}
	pathObservation, status := bindSharedWords(mapping[FastControlPathObservationOffset:FastControlPathObservationOffset+8+PathObservationBytes], PathObservationBytes/8)
	if status != sharedWordsOK {
		return nil, errors.New("invalid path observation slot")
	}
	return &applicationGrantRegion{mapping, header, request, observation, jogControl, jogUpdate, jogObservation, pathSamples, pathObservation}, nil
}

func (c *Client) closeApplicationGrantRegion() {
	c.fastMu.Lock()
	defer c.fastMu.Unlock()
	if c.fast != nil {
		mapping := c.fast.data
		c.fast = nil // Invalidate every reader/publisher before releasing storage.
		_ = unix.Munmap(mapping)
	}
}

// FastObservation makes one bounded snapshot attempt, never waits for a writer,
// and never substitutes a cached value. It does not take the command/upload lock.
func (c *Client) FastObservation() (GrantObservation, error) {
	c.fastMu.RLock()
	defer c.fastMu.RUnlock()
	select {
	case <-c.done:
		return GrantObservation{}, ErrClosed
	default:
	}
	if c.fast == nil {
		return GrantObservation{}, ErrApplicationGrantUnavailable
	}
	var words, scratch [GrantObservationBytes / 8]uint64
	_, status := c.fast.observation.read(words[:], scratch[:])
	if status == sharedWordsBusy {
		return GrantObservation{}, fmt.Errorf("%w: %w", ErrFastObservationUnavailable, ErrSharedSlotBusy)
	}
	if status == sharedWordsUnpublished {
		return GrantObservation{}, ErrFastObservationUnavailable
	}
	if status != sharedWordsOK {
		return GrantObservation{}, fmt.Errorf("RTCore grant observation read: %d", status)
	}
	var encoded [GrantObservationBytes]byte
	if sharedWordsToLE(encoded[:], words[:]) != sharedWordsOK {
		return GrantObservation{}, errors.New("RTCore grant observation conversion failed")
	}
	observation, ok := DecodeGrantObservation(encoded[:])
	if !ok || observation.ConnectionId != c.fast.header.ConnectionId || observation.NativeGeneration == 0 ||
		observation.ConfigurationEpoch == 0 || observation.HomeEpoch == 0 || observation.GrantActive > 1 ||
		observation.GrantReason > GrantReasonAxisMaskMismatch || observation.AxisMask & ^c.fast.header.ConfiguredMask != 0 ||
		observation.Reserved != 0 || observation.Reserved1 != 0 || observation.Reserved2 != 0 || observation.Reserved3 != 0 {
		return GrantObservation{}, errors.New("invalid RTCore grant observation identity/encoding")
	}
	now := monotonicNS()
	if observation.NowHostNs == 0 || observation.NowHostNs > now {
		return GrantObservation{}, errors.New("invalid RTCore grant observation clock")
	}
	if now-observation.NowHostNs > applicationGrantObservationMaxAgeNS {
		return GrantObservation{}, ErrFastObservationUnavailable
	}
	if observation.GrantActive != 0 && (observation.CapabilityId == [16]byte{} || observation.AppGrantGeneration == 0 ||
		observation.AxisMask != c.fast.header.ConfiguredMask || observation.DeadlineHostNs <= observation.NowHostNs) {
		return GrantObservation{}, errors.New("invalid RTCore active grant observation")
	}
	return observation, nil
}

func (c *Client) MirrorAcquire(ctx context.Context, identity GrantIdentity, deadlineNS uint64) (GrantReceipt, error) {
	generation := c.revocation.Load()
	stopAck := c.stopAck.Load()
	origin := monotonicNS()
	return c.mirrorGrant(ctx, GrantOperationAcquire, identity, origin, deadlineNS, generation, stopAck)
}

func (c *Client) MirrorRenew(ctx context.Context, identity GrantIdentity, deadlineNS uint64) (GrantReceipt, error) {
	generation := c.revocation.Load()
	stopAck := c.stopAck.Load()
	origin := monotonicNS()
	return c.mirrorGrant(ctx, GrantOperationRenew, identity, origin, deadlineNS, generation, stopAck)
}

func (c *Client) mirrorGrant(ctx context.Context, operation uint32, identity GrantIdentity, origin, deadline, generation, entryStopAck uint64) (GrantReceipt, error) {
	if identity.CapabilityID == [16]byte{} || identity.Generation == 0 || deadline <= origin || deadline-origin > applicationGrantAbsoluteMaxLease {
		return GrantReceipt{}, errors.New("invalid RTCore application grant identity/absolute expiry")
	}
	limit := deadline
	if origin <= ^uint64(0)-applicationGrantReceiptTimeoutNS && origin+applicationGrantReceiptTimeoutNS < limit {
		limit = origin + applicationGrantReceiptTimeoutNS
	}
	wait := func() error {
		if err := ctx.Err(); err != nil {
			return err
		}
		select {
		case <-c.done:
			return ErrClosed
		default:
		}
		if c.revocation.Load() != generation {
			return ErrRevoked
		}
		if monotonicNS() >= limit {
			return context.DeadlineExceeded
		}
		timer := time.NewTimer(time.Millisecond)
		defer timer.Stop()
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-c.done:
			return ErrClosed
		case <-timer.C:
			return nil
		}
	}
	for !c.grantMu.TryLock() {
		if err := wait(); err != nil {
			return GrantReceipt{}, err
		}
	}
	defer c.grantMu.Unlock()
	if c.revocation.Load() != generation {
		return GrantReceipt{}, ErrRevoked
	}
	if c.grantUncertain {
		if operation != GrantOperationAcquire || c.stopAck.Load() == 0 || c.stopAck.Load() == c.grantUncertainStopAck {
			return GrantReceipt{}, &GrantUncertainError{Cause: errors.New("prior request requires acknowledged Stop or reconnect")}
		}
		c.grantUncertain = false
	}
	if err := ctx.Err(); err != nil {
		return GrantReceipt{}, err
	}
	if monotonicNS() >= limit {
		return GrantReceipt{}, context.DeadlineExceeded
	}
	var observation GrantObservation
	for {
		if c.revocation.Load() != generation {
			return GrantReceipt{}, ErrRevoked
		}
		var err error
		observation, err = c.FastObservation()
		if err == nil && observation.StopAckId >= entryStopAck {
			break
		}
		if err != nil && !errors.Is(err, ErrFastObservationUnavailable) {
			return GrantReceipt{}, err
		}
		if err = wait(); err != nil {
			return GrantReceipt{}, err
		}
	}
	nativeGeneration := observation.NativeGeneration
	if operation == GrantOperationRenew {
		if c.admittedGrant.Identity != identity || c.admittedGrant.NativeGeneration == 0 {
			return GrantReceipt{}, errors.New("RTCore grant renewal requires its admitted identity")
		}
		nativeGeneration = c.admittedGrant.NativeGeneration
	}
	if c.grantTicket == ^uint64(0) {
		return GrantReceipt{}, errors.New("RTCore grant request sequence exhausted; reconnect required")
	}
	c.grantTicket++
	request := GrantRequest{Ticket: c.grantTicket, ConnectionId: observation.ConnectionId, NativeGeneration: nativeGeneration,
		CapabilityId: identity.CapabilityID, AppGrantGeneration: identity.Generation, OriginHostNs: origin,
		DeadlineHostNs: deadline, AxisMask: (uint32(1) << c.axes) - 1, Operation: operation}
	encoded := EncodeGrantRequest(request)
	var words [GrantRequestBytes / 8]uint64
	if sharedWordsFromLE(words[:], encoded[:]) != sharedWordsOK {
		return GrantReceipt{}, errors.New("RTCore grant publication conversion failed")
	}
	if err := ctx.Err(); err != nil {
		return GrantReceipt{}, err
	}
	if monotonicNS() >= limit {
		return GrantReceipt{}, context.DeadlineExceeded
	}
	c.fastMu.RLock()
	c.fastPublishMu.Lock()
	select {
	case <-c.done:
		c.fastPublishMu.Unlock()
		c.fastMu.RUnlock()
		return GrantReceipt{}, ErrClosed
	default:
	}
	if c.revocation.Load() != generation {
		c.fastPublishMu.Unlock()
		c.fastMu.RUnlock()
		return GrantReceipt{}, ErrRevoked
	}
	if c.fast == nil {
		c.fastPublishMu.Unlock()
		c.fastMu.RUnlock()
		return GrantReceipt{}, ErrApplicationGrantUnavailable
	}
	_, status := c.fast.request.publish(words[:])
	c.fastPublishMu.Unlock()
	c.fastMu.RUnlock()
	if status != sharedWordsOK {
		return GrantReceipt{}, fmt.Errorf("RTCore grant publication unavailable: %d", status)
	}
	uncertain := func(cause error) (GrantReceipt, error) {
		c.admittedGrant = GrantReceipt{}
		c.grantUncertain = true
		c.grantUncertainStopAck = entryStopAck
		return GrantReceipt{}, &GrantUncertainError{Cause: cause}
	}
	for {
		if err := ctx.Err(); err != nil {
			return uncertain(err)
		}
		if monotonicNS() >= limit {
			return uncertain(context.DeadlineExceeded)
		}
		if c.revocation.Load() != generation {
			if err := c.Err(); err != nil {
				return uncertain(err)
			}
			return uncertain(ErrRevoked)
		}
		observation, err := c.FastObservation()
		if err == nil && observation.Ticket == request.Ticket {
			if observation.GrantReason != GrantReasonNone {
				c.admittedGrant = GrantReceipt{}
				return GrantReceipt{}, &GrantRejectedError{request.Ticket, observation.GrantReason}
			}
			if observation.GrantActive == 0 || observation.NativeGeneration != request.NativeGeneration ||
				observation.AppGrantGeneration != identity.Generation || observation.CapabilityId != identity.CapabilityID ||
				observation.DeadlineHostNs != deadline || observation.DeadlineHostNs <= monotonicNS() {
				return uncertain(ErrRevoked)
			}
			receipt := GrantReceipt{
				Identity: identity, RequestSequence: request.Ticket,
				ConnectionID: observation.ConnectionId, NativeGeneration: nativeGeneration,
				ConfigurationEpoch: observation.ConfigurationEpoch, HomeEpoch: observation.HomeEpoch,
				DeadlineNS: deadline,
			}
			c.fastPublishMu.Lock()
			if err := ctx.Err(); err != nil {
				c.fastPublishMu.Unlock()
				return uncertain(err)
			}
			if monotonicNS() >= limit {
				c.fastPublishMu.Unlock()
				return uncertain(context.DeadlineExceeded)
			}
			if c.revocation.Load() != generation {
				c.fastPublishMu.Unlock()
				return uncertain(ErrRevoked)
			}
			c.admittedGrant = receipt
			c.fastPublishMu.Unlock()
			return receipt, nil
		}
		if err == nil && observation.Ticket > request.Ticket {
			return uncertain(errors.New("RTCore grant receipt sequence advanced unexpectedly"))
		}
		if err != nil && !errors.Is(err, ErrFastObservationUnavailable) {
			return uncertain(err)
		}
		if err := wait(); err != nil {
			return uncertain(err)
		}
	}
}
