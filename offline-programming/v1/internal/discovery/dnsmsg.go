package discovery

import (
	"encoding/binary"
	"errors"
	"strings"
)

// The few DNS record types DNS-SD needs (RFC 6763). Anything else in a
// received packet is skipped, never interpreted.
const (
	typeA   uint16 = 1
	typePTR uint16 = 12
	typeTXT uint16 = 16
	typeSRV uint16 = 33
	typeANY uint16 = 255

	classIN uint16 = 1
	// classTopBit is the cache-flush bit in a record and the
	// unicast-response (QU) bit in a question (RFC 6762 sections 10.2, 5.4).
	classTopBit uint16 = 0x8000

	flagResponse uint16 = 0x8000
	flagAuth     uint16 = 0x0400
)

// maxPacket bounds every packet read or built. RFC 6762 section 17 allows
// 9000 bytes; one Ethernet frame is what this package ever sends.
const maxPacket = 9000

var errMalformed = errors.New("discovery: malformed DNS message")

type dnsQuestion struct {
	Name  string
	Type  uint16
	Class uint16
}

type dnsRecord struct {
	Name  string
	Type  uint16
	Class uint16
	TTL   uint32
	// Exactly one of these is meaningful, by Type.
	A      [4]byte
	Target string // PTR target or SRV target
	Port   uint16 // SRV
	TXT    []string
}

type dnsMessage struct {
	ID        uint16
	Flags     uint16
	Questions []dnsQuestion
	Answers   []dnsRecord
	Extra     []dnsRecord
}

// canonical lower-cases a name and ends it with a dot: DNS names compare
// case-insensitively and mDNS names are always absolute.
func canonical(name string) string {
	name = strings.ToLower(name)
	if !strings.HasSuffix(name, ".") {
		name += "."
	}
	return name
}

func appendName(b []byte, name string) ([]byte, error) {
	name = strings.TrimSuffix(name, ".")
	if name != "" {
		for _, label := range splitLabels(name) {
			if len(label) == 0 || len(label) > 63 {
				return nil, errMalformed
			}
			b = append(b, byte(len(label)))
			b = append(b, label...)
		}
	}
	return append(b, 0), nil
}

// splitLabels splits on dots. Instance labels in this package never hold a
// dot (sanitizeLabel removes them), so no escaping is needed.
func splitLabels(name string) []string { return strings.Split(name, ".") }

func (m dnsMessage) pack() ([]byte, error) {
	b := make([]byte, 12, 512)
	binary.BigEndian.PutUint16(b[0:], m.ID)
	binary.BigEndian.PutUint16(b[2:], m.Flags)
	binary.BigEndian.PutUint16(b[4:], uint16(len(m.Questions)))
	binary.BigEndian.PutUint16(b[6:], uint16(len(m.Answers)))
	binary.BigEndian.PutUint16(b[10:], uint16(len(m.Extra)))
	var err error
	for _, q := range m.Questions {
		if b, err = appendName(b, q.Name); err != nil {
			return nil, err
		}
		b = binary.BigEndian.AppendUint16(b, q.Type)
		b = binary.BigEndian.AppendUint16(b, q.Class)
	}
	for _, r := range append(append([]dnsRecord{}, m.Answers...), m.Extra...) {
		if b, err = appendName(b, r.Name); err != nil {
			return nil, err
		}
		b = binary.BigEndian.AppendUint16(b, r.Type)
		b = binary.BigEndian.AppendUint16(b, r.Class)
		b = binary.BigEndian.AppendUint32(b, r.TTL)
		var data []byte
		switch r.Type {
		case typeA:
			data = r.A[:]
		case typePTR:
			if data, err = appendName(nil, r.Target); err != nil {
				return nil, err
			}
		case typeSRV:
			data = make([]byte, 6)
			binary.BigEndian.PutUint16(data[4:], r.Port)
			if data, err = appendName(data, r.Target); err != nil {
				return nil, err
			}
		case typeTXT:
			for _, s := range r.TXT {
				if len(s) > 255 {
					return nil, errMalformed
				}
				data = append(data, byte(len(s)))
				data = append(data, s...)
			}
			if len(data) == 0 {
				data = []byte{0}
			}
		default:
			return nil, errMalformed
		}
		b = binary.BigEndian.AppendUint16(b, uint16(len(data)))
		b = append(b, data...)
	}
	if len(b) > maxPacket {
		return nil, errMalformed
	}
	return b, nil
}

// readName decodes a possibly compressed name. Pointers must point strictly
// backwards, which also bounds the walk: a crafted loop is refused.
func readName(msg []byte, off int) (string, int, error) {
	var labels []string
	end := -1
	total := 0
	for {
		if off >= len(msg) {
			return "", 0, errMalformed
		}
		n := int(msg[off])
		switch {
		case n == 0:
			if end < 0 {
				end = off + 1
			}
			return strings.Join(labels, ".") + ".", end, nil
		case n&0xC0 == 0xC0:
			if off+1 >= len(msg) {
				return "", 0, errMalformed
			}
			ptr := int(binary.BigEndian.Uint16(msg[off:]) & 0x3FFF)
			if ptr >= off {
				return "", 0, errMalformed
			}
			if end < 0 {
				end = off + 2
			}
			off = ptr
		case n&0xC0 != 0:
			return "", 0, errMalformed
		default:
			if off+1+n > len(msg) {
				return "", 0, errMalformed
			}
			total += n + 1
			if total > 255 {
				return "", 0, errMalformed
			}
			labels = append(labels, string(msg[off+1:off+1+n]))
			off += 1 + n
		}
	}
}

func unpack(msg []byte) (dnsMessage, error) {
	var m dnsMessage
	if len(msg) < 12 || len(msg) > maxPacket {
		return m, errMalformed
	}
	m.ID = binary.BigEndian.Uint16(msg[0:])
	m.Flags = binary.BigEndian.Uint16(msg[2:])
	qd := int(binary.BigEndian.Uint16(msg[4:]))
	an := int(binary.BigEndian.Uint16(msg[6:]))
	ns := int(binary.BigEndian.Uint16(msg[8:]))
	ar := int(binary.BigEndian.Uint16(msg[10:]))
	off := 12
	for i := 0; i < qd; i++ {
		name, next, err := readName(msg, off)
		if err != nil || next+4 > len(msg) {
			return m, errMalformed
		}
		m.Questions = append(m.Questions, dnsQuestion{
			Name:  name,
			Type:  binary.BigEndian.Uint16(msg[next:]),
			Class: binary.BigEndian.Uint16(msg[next+2:]),
		})
		off = next + 4
	}
	for i := 0; i < an+ns+ar; i++ {
		name, next, err := readName(msg, off)
		if err != nil || next+10 > len(msg) {
			return m, errMalformed
		}
		r := dnsRecord{
			Name:  name,
			Type:  binary.BigEndian.Uint16(msg[next:]),
			Class: binary.BigEndian.Uint16(msg[next+2:]),
			TTL:   binary.BigEndian.Uint32(msg[next+4:]),
		}
		length := int(binary.BigEndian.Uint16(msg[next+8:]))
		start := next + 10
		if start+length > len(msg) {
			return m, errMalformed
		}
		data := msg[start : start+length]
		known := true
		switch r.Type {
		case typeA:
			if length != 4 {
				return m, errMalformed
			}
			copy(r.A[:], data)
		case typePTR:
			if r.Target, _, err = readName(msg, start); err != nil {
				return m, errMalformed
			}
		case typeSRV:
			if length < 7 {
				return m, errMalformed
			}
			r.Port = binary.BigEndian.Uint16(data[4:])
			if r.Target, _, err = readName(msg, start+6); err != nil {
				return m, errMalformed
			}
		case typeTXT:
			for j := 0; j < len(data); {
				n := int(data[j])
				if j+1+n > len(data) {
					return m, errMalformed
				}
				if n > 0 {
					r.TXT = append(r.TXT, string(data[j+1:j+1+n]))
				}
				j += 1 + n
			}
		default:
			known = false
		}
		off = start + length
		if !known {
			continue
		}
		if i < an {
			m.Answers = append(m.Answers, r)
		} else {
			m.Extra = append(m.Extra, r)
		}
	}
	return m, nil
}
