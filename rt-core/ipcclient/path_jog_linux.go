//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"math"
	"runtime"
	"time"
)

var ErrPathUnavailable = errors.New("RTCore path observation unavailable")

// PathChunk is one native path_samples chunk without its identity: up to
// PathJogMaxChunk samples of every native axis in logical joint units
// (row-major, unmasked axes zero), with optional per-sample speed caps in path
// units/s (+Inf = none). Spacing and MaxSpeed must repeat the session's first
// chunk; native refuses a change.
type PathChunk struct {
	First     uint64
	Spacing   float64
	MaxSpeed  float64
	Count     uint32
	End       bool
	EndReason uint32
	Caps      [PATH_JOG_MAX_CHUNK]float64
	Samples   [PATH_JOG_MAX_CHUNK * maxAxes]float64
}

// BeginLocalPathJog reserves a path jog session (protocol 1.14 path_begin)
// with the same identity, generation and first-input admission as
// BeginLocalJog. Nothing moves before a chunk and a speed command arrive.
func (c *Client) BeginLocalPathJog(ctx context.Context, identity GrantIdentity, mask uint32, origin, deadline uint64) (LocalJogSession, error) {
	return c.beginLocalJog(ctx, identity, mask, origin, deadline, JogOperationPathBegin)
}

// PathObservation reads the native path_observation slot. Like
// JogObservation it never returns a cached or torn sample, and refuses one
// from another connection or older than the observation age bound.
func (c *Client) PathObservation() (PathObservation, error) {
	c.fastMu.RLock()
	defer c.fastMu.RUnlock()
	if c.fast == nil {
		return PathObservation{}, ErrApplicationGrantUnavailable
	}
	select {
	case <-c.done:
		return PathObservation{}, ErrClosed
	default:
	}
	var words, scratch [PathObservationBytes / 8]uint64
	_, status := c.fast.pathObservation.read(words[:], scratch[:])
	for attempt := 1; status == sharedWordsBusy && attempt < 3; attempt++ {
		runtime.Gosched()
		_, status = c.fast.pathObservation.read(words[:], scratch[:])
	}
	if status != sharedWordsOK {
		return PathObservation{}, ErrPathUnavailable
	}
	var encoded [PathObservationBytes]byte
	if sharedWordsToLE(encoded[:], words[:]) != sharedWordsOK {
		return PathObservation{}, ErrPathUnavailable
	}
	observation, err := DecodePathObservationFrame(encoded[:])
	now := monotonicNS()
	if err != nil || observation.ConnId != c.fast.header.ConnectionId || observation.NativeGeneration == 0 ||
		observation.ClockMappingGeneration != 1 || observation.NowHostNs == 0 || observation.NowHostNs > now ||
		now-observation.NowHostNs > applicationGrantObservationMaxAgeNS || observation.ChunkResult > JogReasonPathInvalidConfig {
		return PathObservation{}, ErrPathUnavailable
	}
	return observation, nil
}

// AppendLocalPath publishes one chunk and waits for native to acknowledge
// exactly this chunk_sequence for this session. path_samples is lossless, not
// latest-value: the caller must not publish the next chunk before this
// returns. A native refusal returns *JogRejectedError with the jog reason.
func (c *Client) AppendLocalPath(ctx context.Context, session LocalJogSession, sequence uint64, chunk PathChunk) (PathObservation, error) {
	if sequence == 0 || chunk.Count > PATH_JOG_MAX_CHUNK || (chunk.Count == 0 && !chunk.End) ||
		math.IsNaN(chunk.Spacing) || math.IsInf(chunk.Spacing, 0) || !(chunk.Spacing > 0) ||
		math.IsNaN(chunk.MaxSpeed) || math.IsInf(chunk.MaxSpeed, 0) || !(chunk.MaxSpeed > 0) {
		return PathObservation{}, errors.New("invalid path chunk")
	}
	id := session.Identity
	frame := PathSamples{ConnId: id.ConnectionId, NativeGeneration: id.NativeGeneration, CapabilityId: id.CapabilityId,
		AppGrantGeneration: id.AppGrantGeneration, ConfigEpoch: id.ConfigurationEpoch, HomeEpoch: id.HomeEpoch,
		JogGeneration: id.JogGeneration, ClockMappingGeneration: id.ClockMappingGeneration, ChunkSequence: sequence,
		FirstIndex: chunk.First, Spacing: chunk.Spacing, MaxSpeed: chunk.MaxSpeed, Count: chunk.Count,
		EndReason: chunk.EndReason, Caps: chunk.Caps, Samples: chunk.Samples}
	if chunk.End {
		frame.Flags = 1
	}
	encoded := EncodePathSamples(frame)
	var words [PathSamplesBytes / 8]uint64
	if sharedWordsFromLE(words[:], encoded[:]) != sharedWordsOK {
		return PathObservation{}, ErrPathUnavailable
	}
	generation := c.revocation.Load()
	c.pathPublishMu.Lock()
	defer c.pathPublishMu.Unlock()
	c.fastPublishMu.Lock()
	c.fastMu.RLock()
	status := sharedWordsUnbound
	if c.fast != nil && generation == c.revocation.Load() && id.ConnectionId == c.fast.header.ConnectionId {
		_, status = c.fast.pathSamples.publish(words[:])
	}
	c.fastMu.RUnlock()
	c.fastPublishMu.Unlock()
	if status != sharedWordsOK {
		return PathObservation{}, ErrRevoked
	}
	// The cyclic owner consumes a chunk on its next cycle; the grant receipt
	// bound covers scheduling delay without letting a dead core hold the
	// writer indefinitely.
	deadline := monotonicNS() + applicationGrantReceiptTimeoutNS
	for {
		observation, err := c.PathObservation()
		if err == nil && observation.JogGeneration == id.JogGeneration && observation.ChunkSequenceSeen == sequence {
			if observation.ChunkResult != JogReasonNone {
				return observation, &JogRejectedError{Reason: observation.ChunkResult}
			}
			return observation, nil
		}
		if err := ctx.Err(); err != nil {
			return PathObservation{}, err
		}
		if generation != c.revocation.Load() {
			return PathObservation{}, ErrRevoked
		}
		if monotonicNS() >= deadline {
			return PathObservation{}, context.DeadlineExceeded
		}
		select {
		case <-ctx.Done():
			return PathObservation{}, ctx.Err()
		case <-c.done:
			return PathObservation{}, ErrClosed
		case <-time.After(200 * time.Microsecond):
		}
	}
}

// UpdateLocalPathSpeed publishes one path speed command (jog_update flags
// bit 0, velocity[0] in path units/s) to the latest-value update slot. Like
// UpdateLocalJog, return is publication, not admission; the caller's origin
// and deadline are never refreshed by scheduling, backpressure or renewal.
func (c *Client) UpdateLocalPathSpeed(ctx context.Context, session LocalJogSession, sequence, origin, deadline uint64, speed float64) error {
	if math.IsNaN(speed) || math.IsInf(speed, 0) || speed < 0 {
		return errors.New("invalid path speed")
	}
	velocity := make([]float64, c.axes)
	if len(velocity) == 0 {
		return errors.New("invalid path speed")
	}
	velocity[0] = speed
	return c.updateLocalJog(ctx, session, sequence, origin, deadline, session.Identity.AxisMask, velocity, 1)
}
