package viewer

import (
	"bytes"
	"testing"
	"time"

	"github.com/pion/rtp"
)

// stream packetizes units the way the sender does (Pion's H.264 payloader,
// FU-A for large NALs), one RTP timestamp per unit, the marker on its last
// packet.
func stream(t *testing.T, units [][]byte) []*rtp.Packet {
	t.Helper()
	var out []*rtp.Packet
	var seq uint16 = 65500 // wraps inside the test
	for i, u := range units {
		var payloads [][]byte
		for _, nal := range bytes.Split(u, []byte{0, 0, 0, 1}) {
			if len(nal) == 0 {
				continue
			}
			payloads = append(payloads, fragment(nal)...)
		}
		for j, p := range payloads {
			out = append(out, &rtp.Packet{Header: rtp.Header{SequenceNumber: seq, Timestamp: uint32(i * 6000), Marker: j == len(payloads)-1}, Payload: p})
			seq++
		}
	}
	return out
}

// fragment is one NAL as RTP payloads: whole, or FU-A pieces of 1000 bytes.
func fragment(nal []byte) [][]byte {
	if len(nal) <= 1000 {
		return [][]byte{nal}
	}
	head, body := nal[0], nal[1:]
	var out [][]byte
	for i := 0; i < len(body); i += 1000 {
		end := min(i+1000, len(body))
		fu := byte(0)
		if i == 0 {
			fu |= 0x80
		}
		if end == len(body) {
			fu |= 0x40
		}
		out = append(out, append([]byte{head&0xe0 | 28, fu | head&0x1f}, body[i:end]...))
	}
	return out
}

func idr(n int) []byte {
	return append([]byte{0, 0, 0, 1, 0x67, 0x42, 0xe0, 0x1f, 0, 0, 0, 1, 0x68, 0xce, 0x38, 0x80, 0, 0, 0, 1, 0x65}, bytes.Repeat([]byte{byte(n)}, 2500)...)
}

func slice(n int) []byte {
	return append([]byte{0, 0, 0, 1, 0x41}, bytes.Repeat([]byte{byte(n)}, 1500)...)
}

// arrival is when a packet of this test stream arrives: its unit's time
// (15 units a second), the same clock as the RTP timestamp.
func arrival(p *rtp.Packet) time.Time {
	return time.Unix(0, 0).Add(time.Duration(p.Timestamp) * time.Second / 90000)
}

func feed(a *assembler, packets []*rtp.Packet) (units [][]byte, lost bool) {
	for i, p := range packets {
		at := arrival(p)
		if i > 0 && at.Before(arrival(packets[i-1])) {
			at = arrival(packets[i-1]) // a resend arrives late, never early
		}
		u, l := a.push(p, at)
		units = append(units, u...)
		lost = lost || l
	}
	return units, lost
}

func TestUnitsStartAtAKeyFrameAndLeaveOnTheirLastPacket(t *testing.T) {
	// The stream joined mid-unit: that unit and the slice before the key
	// frame cannot be decoded and are dropped.
	in := [][]byte{slice(1), slice(2), idr(3), slice(4), slice(5)}
	packets := stream(t, in)[1:]
	a := newAssembler()
	var got [][]byte
	for i, p := range packets {
		units, lost := a.push(p, arrival(p))
		if lost {
			t.Fatal("no packet was lost")
		}
		if len(units) > 0 && !p.Marker {
			t.Fatalf("packet %d released a unit without its marker", i)
		}
		got = append(got, units...)
	}
	if len(got) != 3 || !bytes.Equal(got[0], in[2]) || !bytes.Equal(got[1], in[3]) || !bytes.Equal(got[2], in[4]) {
		t.Fatalf("got %d units, want the key frame and the two after it", len(got))
	}
}

func TestOutOfOrderPacketsAreReassembled(t *testing.T) {
	in := [][]byte{slice(0), idr(1), slice(2), slice(3)}
	packets := stream(t, in)
	// A resend: the second packet of the key frame arrives after the rest.
	k := 2
	reordered := append(append(append([]*rtp.Packet{}, packets[:k]...), packets[k+1:]...), packets[k])
	got, lost := feed(newAssembler(), reordered)
	if lost || len(got) != 3 || !bytes.Equal(got[0], in[1]) || !bytes.Equal(got[2], in[3]) {
		t.Fatalf("got %d units (lost %v)", len(got), lost)
	}
}

func TestALossWaitsForTheNextKeyFrame(t *testing.T) {
	var in [][]byte
	in = append(in, slice(0), idr(1))
	for i := 2; i < 120; i++ {
		in = append(in, slice(i))
	}
	in = append(in, idr(120), slice(121))
	packets := stream(t, in)
	// Lose one packet of slice 5 for good.
	var dropAt int
	for i, p := range packets {
		if p.Timestamp == 5*6000 {
			dropAt = i
			break
		}
	}
	packets = append(packets[:dropAt:dropAt], packets[dropAt+1:]...)
	got, lost := feed(newAssembler(), packets)
	if !lost {
		t.Fatal("the loss was not reported")
	}
	if !bytes.Equal(got[0], in[1]) || !bytes.Equal(got[1], in[2]) || !bytes.Equal(got[3], in[4]) {
		t.Fatal("the units before the loss must come through")
	}
	if !bytes.Equal(got[4], in[120]) || !bytes.Equal(got[5], in[121]) || len(got) != 6 {
		t.Fatalf("after the loss only the next key frame and what follows it may pass: %d units", len(got))
	}
}
