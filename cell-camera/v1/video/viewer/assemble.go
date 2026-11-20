package viewer

import (
	"time"

	"github.com/pion/rtp"
	"github.com/pion/rtp/codecs"
)

// Unverified bounds. NACK resends a lost packet within about one round trip
// (37 ms direct and 197 ms through Tailscale's DERP relay, a Steam Deck
// 2026-10-19); a gap still open after maxGap is a loss, and the picture waits
// for the next key frame (the sender is asked for one). maxHeld caps memory.
const (
	maxGap  = 300 * time.Millisecond
	maxHeld = 1024
)

// assembler turns the RTP packets of one H.264 stream into Annex-B access
// units. A unit is released as soon as its last packet (the marker) and every
// packet before it have arrived, so the picture never waits for the next
// frame to start: Pion's samplebuilder does, about 66 ms at 15 fps. Packets
// may arrive out of order (NACK resends them late). After a loss, units are
// dropped until a key frame (IDR), since a decoder fed a broken reference
// shows garbage until then; lost reports that, so the caller asks the
// sender for a key frame.
type assembler struct {
	held     map[uint16]*rtp.Packet
	next     uint16 // the first packet of the next unit
	synced   bool   // next is known: a marker has been seen
	needKey  bool
	waitFrom time.Time // when the unit at next first waited on a missing packet
}

func newAssembler() *assembler { return &assembler{held: map[uint16]*rtp.Packet{}, needKey: true} }

// push takes one packet, received at now, and returns the units it
// completed, in order, and whether packets were lost (the caller then asks
// for a key frame).
func (a *assembler) push(p *rtp.Packet, now time.Time) (units [][]byte, lost bool) {
	if !a.synced {
		// The first unit starts after the first marker seen.
		if p.Marker {
			a.next, a.synced = p.SequenceNumber+1, true
		}
		return nil, false
	}
	if int16(p.SequenceNumber-a.next) < 0 {
		return nil, false // a resend of a packet already used or dropped
	}
	a.held[p.SequenceNumber] = p
	units = a.drain(nil)
	// Packets waiting behind a gap: a resend may still fill it, until maxGap.
	if !a.gapAhead() {
		a.waitFrom = time.Time{}
		return units, false
	}
	if a.waitFrom.IsZero() {
		a.waitFrom = now
	}
	if now.Sub(a.waitFrom) <= maxGap && len(a.held) <= maxHeld {
		return units, false
	}
	a.skipGap()
	units = a.drain(units)
	a.waitFrom = time.Time{}
	if a.gapAhead() {
		a.waitFrom = now
	}
	return units, true
}

// drain appends every unit that is complete from next on.
func (a *assembler) drain(units [][]byte) [][]byte {
	for {
		unit, end, ok := a.complete()
		if !ok {
			return units
		}
		a.release(end)
		if unit != nil {
			units = append(units, unit)
		}
	}
}

// gapAhead reports a packet missing before some held packet, rather than
// the unit at next simply not having all arrived yet in order.
func (a *assembler) gapAhead() bool {
	if len(a.held) == 0 {
		return false
	}
	if _, have := a.held[a.next]; !have {
		return true
	}
	for s := a.next; ; s++ {
		if _, have := a.held[s]; !have {
			for t := range a.held {
				if int16(t-s) > 0 {
					return true
				}
			}
			return false
		}
	}
}

// complete finds the unit that starts at next and ends at a marker with no
// packet missing. unit is nil when that unit is dropped (no key frame yet).
func (a *assembler) complete() (unit []byte, end uint16, ok bool) {
	for s := a.next; ; s++ {
		p, have := a.held[s]
		if !have {
			return nil, 0, false
		}
		if p.Marker {
			end = s
			break
		}
		if s-a.next > maxHeld {
			return nil, 0, false
		}
	}
	var depacketizer codecs.H264Packet
	var data []byte
	for s := a.next; ; s++ {
		out, err := depacketizer.Unmarshal(a.held[s].Payload)
		if err != nil {
			a.needKey = true
			return nil, end, true
		}
		data = append(data, out...)
		if s == end {
			break
		}
	}
	if a.needKey {
		if !hasIDR(data) {
			return nil, end, true
		}
		a.needKey = false
	}
	return data, end, true
}

func (a *assembler) release(end uint16) {
	for s := a.next; s != end+1; s++ {
		delete(a.held, s)
	}
	a.next = end + 1
}

// skipGap moves next past the missing packets to the start of the first unit
// held after them: the packet after the first held marker.
func (a *assembler) skipGap() {
	a.needKey = true
	first, found := uint16(0), false
	for s := range a.held {
		if p := a.held[s]; p.Marker && (!found || int16(s-first) < 0) {
			first, found = s, true
		}
	}
	if !found {
		clear(a.held)
		a.synced = false
		return
	}
	for s := range a.held {
		if int16(s-first) <= 0 {
			delete(a.held, s)
		}
	}
	a.next = first + 1
}

// hasIDR reports whether an Annex-B unit holds an IDR slice (NAL type 5).
func hasIDR(unit []byte) bool {
	for i := 0; i+3 < len(unit); i++ {
		if unit[i] == 0 && unit[i+1] == 0 && unit[i+2] == 1 && unit[i+3]&0x1f == 5 {
			return true
		}
	}
	return false
}
