//go:build linux

package control

import (
	"context"
	"encoding/hex"
	"errors"
	"math"
	"sync"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Unverified ingress bound: 64 unacknowledged chunks (1024 samples, the
// native ring capacity) per path session. A producer that outruns native
// acknowledgement is refused with path_queue_full instead of growing memory.
const pathQueueChunks = 64

// Unverified retry period while native reports path_capacity: one cycle-scale
// wait before resending the same chunk, so playback can free ring space.
const pathCapacityRetry = 2 * time.Millisecond

type pathJogNative interface {
	BeginLocalPathJog(context.Context, ipcclient.GrantIdentity, uint32, uint64, uint64) (ipcclient.LocalJogSession, error)
	AppendLocalPath(context.Context, ipcclient.LocalJogSession, uint64, ipcclient.PathChunk) (ipcclient.PathObservation, error)
	UpdateLocalPathSpeed(context.Context, ipcclient.LocalJogSession, uint64, uint64, uint64, float64) error
	PathObservation() (ipcclient.PathObservation, error)
}

type pathItem struct {
	sequence uint64
	chunk    ipcclient.PathChunk
}

// pathAck reports one chunk's native outcome to a transport that waits for
// it (the remote stream); local producers poll PathJogStatus instead.
type pathAck struct {
	sequence uint64
	reason   uint32
}

// pathSession owns the lossless chunk handshake for one path jog generation:
// ingress validates and queues chunks in order; one writer publishes a chunk,
// waits for native to acknowledge exactly that chunk_sequence, then sends the
// next. Fields other than queue/done are protected by Controller.mu.
type pathSession struct {
	generation uint64
	session    ipcclient.LocalJogSession
	queue      chan pathItem
	done       chan struct{}
	stop       sync.Once
	next       uint64 // Last chunk_sequence admitted to the queue.
	inflight   uint32
	accepted   uint64
	refused    uint64
	reason     uint32
	ended      bool
	sink       chan<- pathAck
}

func (p *pathSession) retire() {
	p.ended = true
	p.stop.Do(func() { close(p.done) })
}

// BeginPathJog reserves a native path jog session (protocol 1.14) with the
// same fences as BeginJog. It requires Describe path_jog: without the machine
// jog_jerk the native generator has no jerk bound and refuses path_begin. Like
// BeginJog it refuses under a maintenance notice for the remote jog stream.
func (c *Controller) BeginPathJog(f Fence, mask uint32, origin, deadline uint64) (uint64, error) {
	if err := c.checkMaintenance(OperationBeginPathJog); err != nil {
		return 0, err
	}
	c.operationMu.Lock()
	defer c.operationMu.Unlock()
	core, ok := c.core.(pathJogNative)
	if !ok || c.description.PathJog == nil {
		return 0, errors.New(ReasonPathJogUnavailable)
	}
	c.mu.Lock()
	if err := c.authorize(f); err != nil {
		c.mu.Unlock()
		return 0, err
	}
	identity, ctx := c.nativeGrantIdentity(), c.sessionCtx
	c.mu.Unlock()
	session, err := core.BeginLocalPathJog(ctx, identity, mask, origin, deadline)
	if err != nil {
		return 0, err
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := c.authorize(f); err != nil {
		return 0, err
	}
	c.retirePathLocked()
	p := &pathSession{generation: session.Identity.JogGeneration, session: session,
		queue: make(chan pathItem, pathQueueChunks), done: make(chan struct{})}
	c.path = p
	c.localJog, c.jogSourceSequence = &session, 0
	go c.runPathWriter(ctx, core, p)
	return session.Identity.JogGeneration, nil
}

// retirePathLocked stops the writer of the current path session; its last
// outcome stays readable through PathJogStatus. Caller holds c.mu.
func (c *Controller) retirePathLocked() {
	if c.path != nil {
		c.path.retire()
	}
}

// pathOpenLocked reports whether the current jog session is a live path
// session. Caller holds c.mu.
func (c *Controller) pathOpenLocked() bool {
	return c.path != nil && !c.path.ended && c.localJog != nil && c.path.generation == c.localJog.Identity.JogGeneration
}

// setPathSink routes chunk acknowledgements of one generation to a waiting
// transport; nil detaches it. Sends never block the writer.
func (c *Controller) setPathSink(generation uint64, sink chan<- pathAck) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.path != nil && c.path.generation == generation {
		c.path.sink = sink
	}
}

// appendLocalPath validates one producer chunk against the current session
// and queues it. It never waits on native: the writer owns the handshake.
func (c *Controller) appendLocalPath(frame ipcclient.LocalPathSamples) uint32 {
	c.mu.Lock()
	defer c.mu.Unlock()
	var token [64]byte
	hex.Encode(token[:], frame.SessionId[:])
	if string(token[:]) != c.grant.Session {
		return ipcclient.JogReasonWrongIdentity
	}
	if frame.GrantGeneration != c.grant.Generation {
		return ipcclient.JogReasonInvalidGeneration
	}
	if c.authorize(Fence{c.grant.Session, frame.GrantGeneration}) != nil {
		return ipcclient.JogReasonAuthorityLost
	}
	if c.localJog == nil {
		return ipcclient.JogReasonClosed
	}
	if frame.JogGeneration != c.localJog.Identity.JogGeneration {
		return ipcclient.JogReasonInvalidGeneration
	}
	if !c.pathOpenLocked() {
		return ipcclient.JogReasonModeConflict
	}
	p := c.path
	reason := uint32(ipcclient.JogReasonNone)
	switch {
	case frame.ChunkSequence != p.next+1:
		reason = ipcclient.JogReasonPathOutOfOrder
	case frame.Reserved != 0 || frame.Flags & ^uint32(1) != 0 || frame.Count > ipcclient.PATH_JOG_MAX_CHUNK ||
		(frame.Count == 0 && frame.Flags == 0):
		reason = ipcclient.JogReasonPathInvalidSample
	case math.IsNaN(frame.Spacing) || math.IsInf(frame.Spacing, 0) || !(frame.Spacing > 0) ||
		math.IsNaN(frame.MaxSpeed) || math.IsInf(frame.MaxSpeed, 0) || !(frame.MaxSpeed > 0):
		reason = ipcclient.JogReasonPathInvalidConfig
	}
	if reason == ipcclient.JogReasonNone {
		item := pathItem{sequence: frame.ChunkSequence, chunk: ipcclient.PathChunk{First: frame.FirstIndex,
			Spacing: frame.Spacing, MaxSpeed: frame.MaxSpeed, Count: frame.Count, End: frame.Flags&1 != 0,
			EndReason: frame.EndReason, Caps: frame.Caps, Samples: frame.Samples}}
		select {
		case p.queue <- item:
			p.next = frame.ChunkSequence
			return ipcclient.JogReasonNone
		default:
			reason = ipcclient.JogReasonPathCapacity
		}
	}
	p.refused, p.reason = frame.ChunkSequence, reason
	return reason
}

// recordPathIngress keeps chunk refusals in the shared ingress diagnostics.
func (c *Controller) recordPathIngress(sequence uint64, reason uint32) {
	if reason != ipcclient.JogReasonNone {
		c.recordJogIngress(sequence, reason, jogRefusal{transport: "path_chunk"})
	}
}

// runPathWriter publishes queued chunks one at a time. path_capacity is
// retried (playback frees ring space); any other refusal or a lost
// acknowledgement ends the session through EndJog, which stops along the
// path natively.
func (c *Controller) runPathWriter(ctx context.Context, core pathJogNative, p *pathSession) {
	for {
		var item pathItem
		select {
		case <-p.done:
			return
		case <-ctx.Done():
			return
		case item = <-p.queue:
		}
		c.mu.Lock()
		p.inflight = 1
		c.mu.Unlock()
		reason := uint32(ipcclient.JogReasonNone)
		for {
			_, err := core.AppendLocalPath(ctx, p.session, item.sequence, item.chunk)
			reason = jogIngressReason(err)
			if reason != ipcclient.JogReasonPathCapacity {
				break
			}
			select {
			case <-p.done:
				return
			case <-ctx.Done():
				return
			case <-time.After(pathCapacityRetry):
			}
		}
		c.mu.Lock()
		p.inflight = 0
		if reason == ipcclient.JogReasonNone {
			p.accepted = item.sequence
		} else {
			p.refused, p.reason = item.sequence, reason
		}
		sink := p.sink
		ended := p.ended
		c.mu.Unlock()
		if sink != nil {
			select {
			case sink <- pathAck{item.sequence, reason}:
			default:
			}
		}
		if reason != ipcclient.JogReasonNone {
			c.recordJogIngress(item.sequence, reason)
			if !ended {
				c.endPathAfterRefusal(p)
			}
			return
		}
	}
}

func (c *Controller) endPathAfterRefusal(p *pathSession) {
	core, ok := c.core.(localJogNative)
	c.mu.Lock()
	if !ok || c.path != p || c.localJog == nil || c.localJog.Identity.JogGeneration != p.generation {
		p.retire()
		c.mu.Unlock()
		return
	}
	session, ctx := *c.localJog, c.sessionCtx
	c.localJog = nil
	p.retire()
	c.mu.Unlock()
	_ = core.EndLocalJog(ctx, session)
}

// PathJogStatus is the path observation read from the fast-control region
// (never the 10 Hz status file) plus this adapter's handshake state.
func (c *Controller) PathJogStatus() (PathJogStatus, error) {
	core, ok := c.core.(pathJogNative)
	if !ok {
		return PathJogStatus{}, errors.New(ReasonPathJogUnavailable)
	}
	observation, err := core.PathObservation()
	if err != nil {
		return PathJogStatus{}, err
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	status := PathJogStatus{Observation: observation}
	if p := c.path; p != nil {
		status.Generation, status.Accepted, status.Refused, status.Reason = p.generation, p.accepted, p.refused, p.reason
		status.Queued = uint32(len(p.queue)) + p.inflight
	}
	return status, nil
}

func (n *restartNative) BeginLocalPathJog(ctx context.Context, identity ipcclient.GrantIdentity, mask uint32, origin, deadline uint64) (ipcclient.LocalJogSession, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return ipcclient.LocalJogSession{}, err
	}
	return n.client.BeginLocalPathJog(ctx, identity, mask, origin, deadline)
}

func (n *restartNative) AppendLocalPath(ctx context.Context, session ipcclient.LocalJogSession, sequence uint64, chunk ipcclient.PathChunk) (ipcclient.PathObservation, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return ipcclient.PathObservation{}, err
	}
	return n.client.AppendLocalPath(ctx, session, sequence, chunk)
}

func (n *restartNative) UpdateLocalPathSpeed(ctx context.Context, session ipcclient.LocalJogSession, sequence, origin, deadline uint64, speed float64) error {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return err
	}
	return n.client.UpdateLocalPathSpeed(ctx, session, sequence, origin, deadline, speed)
}

func (n *restartNative) PathObservation() (ipcclient.PathObservation, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.PathObservation()
}
