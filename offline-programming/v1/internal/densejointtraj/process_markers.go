package densejointtraj

import (
	"crypto/sha256"
	"encoding/hex"
	"strconv"
	"strings"
	"unicode/utf16"
)

// ProcessMarkersDigest is "sha256:" of the planner's canonical marker list
// (weld_planner weldplan/io_schedule.py markers_digest: Python json.dumps
// with sort_keys and compact separators, ensure_ascii on). The v1 trajectory
// digest covers samples only, so this digest is what binds an output
// schedule to the plan the operator made; a Go json.Marshal would differ
// from it on key order and on '<', '>', '&' and non-ASCII names.
func ProcessMarkersDigest(markers []ProcessMarker) string {
	var b strings.Builder
	b.WriteByte('[')
	for i, m := range markers {
		if i > 0 {
			b.WriteByte(',')
		}
		b.WriteString(`{"action":`)
		writePythonJSONString(&b, m.Action)
		if m.Monitor != "" {
			b.WriteString(`,"monitor":`)
			writePythonJSONString(&b, m.Monitor)
		}
		b.WriteString(`,"sample":`)
		b.WriteString(strconv.Itoa(m.Sample))
		b.WriteString(`,"segment":`)
		b.WriteString(strconv.Itoa(m.Segment))
		if m.TimeoutMS != 0 {
			if m.StableMS != 0 {
				b.WriteString(`,"stable_ms":`)
				b.WriteString(strconv.FormatUint(uint64(m.StableMS), 10))
			}
			b.WriteString(`,"timeout_ms":`)
			b.WriteString(strconv.FormatUint(uint64(m.TimeoutMS), 10))
		}
		b.WriteString(`,"value":`)
		b.WriteString(strconv.FormatBool(m.Value))
		b.WriteByte('}')
	}
	b.WriteByte(']')
	sum := sha256.Sum256([]byte(b.String()))
	return "sha256:" + hex.EncodeToString(sum[:])
}

// writePythonJSONString writes s as Python's json.dumps(ensure_ascii=True)
// does: the two-character escapes it knows, every other control character
// and every non-ASCII code point as lowercase \uXXXX (UTF-16 surrogate
// pairs above the BMP).
func writePythonJSONString(b *strings.Builder, s string) {
	const hexDigits = "0123456789abcdef"
	escape := func(u uint16) {
		b.WriteString(`\u`)
		for shift := 12; shift >= 0; shift -= 4 {
			b.WriteByte(hexDigits[(u>>shift)&0xf])
		}
	}
	b.WriteByte('"')
	for _, r := range s {
		switch {
		case r == '"':
			b.WriteString(`\"`)
		case r == '\\':
			b.WriteString(`\\`)
		case r == '\n':
			b.WriteString(`\n`)
		case r == '\r':
			b.WriteString(`\r`)
		case r == '\t':
			b.WriteString(`\t`)
		case r == '\b':
			b.WriteString(`\b`)
		case r == '\f':
			b.WriteString(`\f`)
		case r < 0x20:
			escape(uint16(r))
		case r < 0x80:
			b.WriteRune(r)
		case r > 0xffff:
			hi, lo := utf16.EncodeRune(r)
			escape(uint16(hi))
			escape(uint16(lo))
		default:
			escape(uint16(r))
		}
	}
	b.WriteByte('"')
}
