package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"regexp"
	"strconv"
	"strings"
)

// recordDigest hashes the transitive layout for every language, preventing
// nested layout changes or duplicated digest algorithms from splitting identity.
func recordDigest(r wireRecord, known map[string]wireRecord) string {
	var layout func(wireRecord) []byte
	layout = func(r wireRecord) []byte {
		raw, _ := json.Marshal(r)
		for _, f := range r.Fields {
			if child, ok := known[f.base]; ok {
				raw = append(raw, layout(child)...)
			}
		}
		return raw
	}
	return fmt.Sprintf("%x", sha256.Sum256(layout(r)))
}

type wireField struct {
	Name        string  `json:"name"`
	Type        string  `json:"type"`
	Offset      int     `json:"offset"`
	Size        int     `json:"size"`
	Alignment   int     `json:"alignment"`
	Reserved    bool    `json:"reserved,omitempty"`
	Padding     bool    `json:"padding,omitempty"`
	AllowedBits *uint64 `json:"allowed_bits,omitempty"`
	ArrayBound  *int    `json:"array_bound,omitempty"`
	base        string
	count       int
}
type wireRecord struct {
	Name       string      `json:"name,omitempty"`
	Size       int         `json:"size"`
	Alignment  int         `json:"alignment"`
	Fields     []wireField `json:"fields"`
	MajorField string      `json:"major_field,omitempty"`
	legacy     bool
	key        string
}
type wireConstant struct {
	Name    string `json:"name"`
	Type    string `json:"type"`
	Value   uint64 `json:"value"`
	Comment string `json:"comment"`
}
type wireEnum struct {
	Type    string          `json:"type"`
	Values  json.RawMessage `json:"values"`
	entries []wireConstant
}

var wireType = regexp.MustCompile(`^([A-Za-z][A-Za-z0-9_]*)(?:\[([1-9][0-9]*)\])?$`)
var wireName = regexp.MustCompile(`^[A-Za-z][A-Za-z0-9_]*$`)
var scalarWidth = map[string]int{"u8": 1, "u16": 2, "u32": 4, "u64": 8, "i8": 1, "i16": 2, "i32": 4, "i64": 8, "f64": 8}
var cppScalar = map[string]string{"u8": "uint8_t", "u16": "uint16_t", "u32": "uint32_t", "u64": "uint64_t", "i8": "int8_t", "i16": "int16_t", "i32": "int32_t", "i64": "int64_t", "f64": "double"}
var goScalar = map[string]string{"u8": "byte", "u16": "uint16", "u32": "uint32", "u64": "uint64", "i8": "int8", "i16": "int16", "i32": "int32", "i64": "int64", "f64": "float64"}

// strictJSON rejects unknown members and duplicate keys before layout validation;
// a misspelled reserved marker must never silently disable frame validation.
func strictJSON(data []byte, out any) error {
	var check func(json.RawMessage) error
	check = func(raw json.RawMessage) error {
		raw = bytes.TrimSpace(raw)
		if len(raw) == 0 {
			return fmt.Errorf("empty JSON")
		}
		switch raw[0] {
		case '{':
			members, err := object(raw)
			if err != nil {
				return err
			}
			for _, m := range members {
				if err := check(m.value); err != nil {
					return err
				}
			}
		case '[':
			var entries []json.RawMessage
			if err := json.Unmarshal(raw, &entries); err != nil {
				return err
			}
			for _, e := range entries {
				if err := check(e); err != nil {
					return err
				}
			}
		}
		return nil
	}
	if err := check(data); err != nil {
		return err
	}
	d := json.NewDecoder(bytes.NewReader(data))
	d.DisallowUnknownFields()
	return d.Decode(out)
}

func parseWireRecord(name string, data []byte, known map[string]wireRecord) (wireRecord, error) {
	var r wireRecord
	if err := strictJSON(data, &r); err != nil {
		return r, err
	}
	if r.Name == "" {
		r.Name = name
	}
	if r.Name != name || !wireName.MatchString(name) || cppKeyword(name) || r.Size < 1 || r.Size > maxRecordBytes || len(r.Fields) == 0 {
		return r, fmt.Errorf("invalid layout %s", name)
	}
	offset, alignment := 0, 1
	seen := map[string]bool{}
	for i := range r.Fields {
		f := &r.Fields[i]
		parts := wireType.FindStringSubmatch(f.Type)
		if parts == nil || !identifier.MatchString(f.Name) || cppKeyword(f.Name) || seen[camel(f.Name)] {
			return r, fmt.Errorf("invalid field %s.%s", name, f.Name)
		}
		seen[camel(f.Name)] = true
		f.base = parts[1]
		width := scalarWidth[f.base]
		a := width
		if width == 0 {
			child, ok := known[f.base]
			if !ok {
				return r, fmt.Errorf("unknown field type %s", f.base)
			}
			width = child.Size
			a = child.Alignment
		}
		if parts[2] != "" {
			n, err := strconv.Atoi(parts[2])
			// Unverified default array bound: 32 elements; explicit schema bounds
			// remain constrained by the record byte ceiling before multiplication.
			bound := 32
			if f.ArrayBound != nil {
				bound = *f.ArrayBound
			}
			if err != nil || bound < 1 || bound > maxRecordBytes || n < 1 || n > bound || n > maxRecordBytes/width {
				return r, fmt.Errorf("invalid array %s", f.Type)
			}
			f.count = n
			width *= n
		} else if f.ArrayBound != nil {
			return r, fmt.Errorf("array bound on scalar %s.%s", name, f.Name)
		}
		if f.Offset != offset || f.Size != width || f.Alignment != a || offset%a != 0 {
			return r, fmt.Errorf("invalid offset/size/alignment for %s.%s", name, f.Name)
		}
		if f.Padding && (!f.Reserved || f.base != "u8" || !strings.HasPrefix(f.Name, "reserved")) {
			return r, fmt.Errorf("invalid named padding %s.%s", name, f.Name)
		}
		if strings.HasPrefix(f.Name, "reserved") && !f.Reserved {
			return r, fmt.Errorf("reserved field lacks constraint: %s.%s", name, f.Name)
		}
		if f.AllowedBits != nil && (f.count != 0 || !strings.HasPrefix(f.base, "u") || (f.Size < 8 && *f.AllowedBits >= uint64(1)<<(f.Size*8))) {
			return r, fmt.Errorf("invalid flag mask %s.%s", name, f.Name)
		}
		offset += width
		if a > alignment {
			alignment = a
		}
	}
	if offset != r.Size || alignment != r.Alignment || offset%alignment != 0 {
		return r, fmt.Errorf("invalid final size/alignment %s", name)
	}
	if r.MajorField != "" {
		found := false
		for _, f := range r.Fields {
			found = found || f.Name == r.MajorField && f.base == "u16" && f.count == 0
		}
		if !found {
			return r, fmt.Errorf("invalid major field %s", name)
		}
	}
	return r, nil
}

func parseNative(s *protocolSchema, values map[string]json.RawMessage) error {
	encoded, ok := values["native_records"]
	if !ok {
		return nil
	}
	var contract struct{ Endianness, Alignment, Publication, Compatibility string }
	if err := strictJSON(values["wire_contract"], &contract); err != nil {
		return err
	}
	if contract.Endianness != "little" || contract.Alignment == "" || contract.Publication == "" || contract.Compatibility == "" {
		return fmt.Errorf("native wire contract incomplete")
	}
	// The existing small records retain their raw, boolean codec API. Frame
	// decoders below add rejection without changing the SDK's existing callers.
	for _, entry := range append([]rawMember{{"command_result", values["command_result"]}}, mustMembers(values["records"])...) {
		r, err := parseWireRecord(entry.name, entry.value, nil)
		if err != nil {
			return err
		}
		r.key = entry.name
		r.Name = camel(entry.name)
		r.legacy = true
		s.wireRecords = append(s.wireRecords, r)
	}
	var entries []json.RawMessage
	if err := json.Unmarshal(encoded, &entries); err != nil {
		return err
	}
	known := map[string]wireRecord{}
	for _, data := range entries {
		var head struct {
			Name string `json:"name"`
		}
		if err := json.Unmarshal(data, &head); err != nil {
			return err
		}
		if _, ok := known[head.Name]; ok {
			return fmt.Errorf("duplicate native record %s", head.Name)
		}
		r, err := parseWireRecord(head.Name, data, known)
		if err != nil {
			return err
		}
		r.key = r.Name
		known[r.Name] = r
		s.wireRecords = append(s.wireRecords, r)
	}
	if len(entries) == 0 {
		return fmt.Errorf("native records empty")
	}
	if err := strictJSON(values["native_constants"], &s.wireConstants); err != nil {
		return err
	}
	if err := strictJSON(values["native_enums"], &s.wireEnums); err != nil {
		return err
	}
	seen := map[string]bool{}
	validate := func(c wireConstant) error {
		width := scalarWidth[c.Type]
		if !wireName.MatchString(c.Name) || cppKeyword(c.Name) || seen[c.Name] || !strings.HasPrefix(c.Type, "u") || width == 0 || width < 8 && c.Value >= uint64(1)<<(width*8) {
			return fmt.Errorf("invalid native constant %s", c.Name)
		}
		seen[c.Name] = true
		return nil
	}
	for _, c := range s.wireConstants {
		if c.Comment == "" {
			return fmt.Errorf("constant needs provenance %s", c.Name)
		}
		if err := validate(c); err != nil {
			return err
		}
	}
	for i := range s.wireEnums {
		e := &s.wireEnums[i]
		members, err := object(e.Values)
		if err != nil {
			return err
		}
		for _, m := range members {
			var value uint64
			if err := json.Unmarshal(m.value, &value); err != nil || string(m.value) == "null" {
				return fmt.Errorf("invalid enum %s", m.name)
			}
			c := wireConstant{Name: m.name, Type: e.Type, Value: value}
			if err := validate(c); err != nil {
				return err
			}
			e.entries = append(e.entries, c)
		}
	}
	// These aliases are established ABI names, not independent version/axis decisions.
	for _, c := range s.wireConstants {
		for _, base := range s.constants {
			if (c.Name == "kVerMajor" && base.name == "version_major" || c.Name == "kVerMinor" && base.name == "version_minor" || c.Name == "GRADIENT_MAX_AXES" && base.name == "max_axes") && c.Value != uint64(base.value) {
				return fmt.Errorf("native constant disagrees with %s", base.name)
			}
		}
	}
	return nil
}
func mustMembers(data []byte) []rawMember { members, _ := object(data); return members }

func generateNative(s protocolSchema, g *strings.Builder) ([]byte, error) {
	if len(s.wireRecords) == 0 {
		return nil, nil
	}
	var c strings.Builder
	c.WriteString("// Generated by tools/protocolgen; edit protocol/control.json.\n#pragma once\n#include \"control_protocol.hpp\"\n#include <array>\n#include <cstddef>\n#include <cstdint>\n#include <cstring>\n#include <limits>\n// Private copies only: shared slots require acquire/release and seqlock validation.\n")
	c.WriteString("namespace gradient::ipc::v1 {\n")
	c.WriteString(`// Copy scalar bits without alignment or aliasing assumptions. Little-endian
// hosts use memcpy directly; other hosts retain explicit wire byte ordering.
namespace detail {
template<class Bits, class T> inline void store_native_le(uint8_t* bytes, const T& value) noexcept {
  static_assert(sizeof(Bits) == sizeof(T));
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(bytes, &value, sizeof(T));
#else
  Bits bits{}; std::memcpy(&bits, &value, sizeof(T));
  for (std::size_t b=0; b<sizeof(T); ++b) bytes[b] = static_cast<uint8_t>(bits >> (8*b));
#endif
}
template<class Bits, class T> inline void load_native_le(const uint8_t* bytes, T& value) noexcept {
  static_assert(sizeof(Bits) == sizeof(T));
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(&value, bytes, sizeof(T));
#else
  Bits bits{};
  for (std::size_t b=0; b<sizeof(T); ++b) bits |= static_cast<Bits>(bytes[b]) << (8*b);
  std::memcpy(&value, &bits, sizeof(T));
#endif
}
} // namespace detail
`)
	emitConstant := func(v wireConstant, enumeration bool) {
		comment := v.Comment
		if comment == "" {
			comment = "Measured existing native 1.6 wire enumeration."
		}
		if enumeration {
			fmt.Fprintf(&c, "// %s\n%s = %d,\n", comment, v.Name, v.Value)
		} else {
			fmt.Fprintf(&c, "// %s\ninline constexpr %s %s = %d;\n", comment, cppScalar[v.Type], v.Name, v.Value)
		}
		fmt.Fprintf(g, "// %s\nconst %s = %d\n", comment, v.Name, v.Value)
	}
	for _, v := range s.wireConstants {
		emitConstant(v, false)
	}
	for _, e := range s.wireEnums {
		fmt.Fprintf(&c, "enum : %s {\n", cppScalar[e.Type])
		for _, v := range e.entries {
			emitConstant(v, true)
		}
		c.WriteString("};\n")
	}
	g.WriteString(`
// MalformedFrameError identifies a rejected fixed frame. No partially decoded
// value escapes on failure, including an incompatible handshake major version.
type MalformedFrameError struct { Record, Field string }
func (e *MalformedFrameError) Error() string { return "malformed native frame: " + e.Record + "." + e.Field }
func (e *MalformedFrameError) Is(target error) bool { return target == ErrMalformedFrame }
var ErrMalformedFrame = &MalformedFrameError{}
`)
	known := map[string]wireRecord{}
	for _, r := range s.wireRecords {
		known[r.Name] = r
	}
	for _, r := range s.wireRecords {
		if r.Name != "CycleCaptureRecordV2" {
			continue
		}
		digest := recordDigest(r, known)
		fmt.Fprintf(&c, "// Measured SHA256 of generated transitive record layout.\ninline constexpr char %sLayoutDigest[] = %q;\n", r.Name, digest)
		fmt.Fprintf(g, "// Measured SHA256 of generated transitive record layout.\nconst %sLayoutDigest = %q\n", r.Name, digest)
	}
	for _, r := range s.wireRecords {
		if r.legacy {
			fmt.Fprintf(&c, "} // namespace gradient::ipc::v1\nnamespace rosie::protocol {\n")
		} else {
			fmt.Fprintf(&c, "struct %s {\n", r.Name)
			fmt.Fprintf(g, "// Measured existing native wire size, in bytes.\nconst %sBytes = %d\ntype %s struct {\n", r.Name, r.Size, r.Name)
			for _, f := range r.Fields {
				ct, gt := cppScalar[f.base], goScalar[f.base]
				if ct == "" {
					ct = f.base
					gt = f.base
				}
				if f.count != 0 {
					fmt.Fprintf(&c, "  %s %s[%d];\n", ct, f.Name, f.count)
					gt = fmt.Sprintf("[%d]%s", f.count, gt)
				} else {
					fmt.Fprintf(&c, "  %s %s;\n", ct, f.Name)
				}
				fmt.Fprintf(g, "%s %s\n", camel(f.Name), gt)
			}
			c.WriteString("};\n")
			g.WriteString("}\n")
		}
		fmt.Fprintf(&c, "static_assert(sizeof(%s) == %d);\nstatic_assert(alignof(%s) == %d);\n", r.Name, r.Size, r.Name, r.Alignment)
		for _, f := range r.Fields {
			fmt.Fprintf(&c, "static_assert(offsetof(%s, %s) == %d);\n", r.Name, f.Name, f.Offset)
			fmt.Fprintf(g, "// Measured existing native field offset, in bytes.\nconst %s%sOffset = %d\n", r.Name, camel(f.Name), f.Offset)
		}
		if !r.legacy {
			writeNativeCodec(&c, g, r, known)
		}
		writeFrameDecoder(&c, g, r, known)
		if r.legacy {
			c.WriteString("} // namespace rosie::protocol\nnamespace gradient::ipc::v1 {\n")
		}
	}
	c.WriteString("} // namespace gradient::ipc::v1\n")
	return []byte(c.String()), nil
}

// wireLoopLeaves emits fixed-bound array loops using schema offsets and strides;
// nested indices remain distinct so no axis can alias another axis's bytes.
func wireLoopLeaves(c, g *strings.Builder, r wireRecord, known map[string]wireRecord, cp, gp, offset string, depth int, visit func(*strings.Builder, *strings.Builder, wireField, string, string, string)) {
	for _, f := range r.Fields {
		var cb, gb strings.Builder
		cfield, gfield := cp+"."+f.Name, gp+"."+camel(f.Name)
		at := fmt.Sprintf("%s%d", offset, f.Offset)
		index := fmt.Sprintf("i%d", depth)
		nextDepth := depth
		if f.count != 0 {
			cfield += "[" + index + "]"
			gfield += "[" + index + "]"
			at += fmt.Sprintf("+%s*%d", index, f.Size/f.count)
			nextDepth++
		}
		if child, ok := known[f.base]; ok {
			wireLoopLeaves(&cb, &gb, child, known, cfield, gfield, at+"+", nextDepth, visit)
		} else {
			visit(&cb, &gb, f, cfield, gfield, at)
		}
		// Validation and padding-only encoders need no empty array traversal.
		if cb.Len() != 0 {
			if f.count != 0 {
				fmt.Fprintf(c, "  for (std::size_t %s=0; %s<%d; ++%s) {\n", index, index, f.count, index)
			}
			c.WriteString(cb.String())
			if f.count != 0 {
				c.WriteString("  }\n")
			}
		}
		if gb.Len() != 0 {
			if f.count != 0 {
				fmt.Fprintf(g, "for %s := 0; %s < %d; %s++ {\n", index, index, f.count, index)
			}
			g.WriteString(gb.String())
			if f.count != 0 {
				g.WriteString("}\n")
			}
		}
	}
}

func writeNativeCodec(c, g *strings.Builder, r wireRecord, known map[string]wireRecord) {
	// Layout validation forbids gaps; every byte, including named padding, is
	// assigned below so a separate whole-record zero fill is unnecessary.
	fmt.Fprintf(c, "inline std::array<uint8_t, %d> encode_%s(const %s& value) noexcept {\n  std::array<uint8_t, %d> bytes;\n", r.Size, r.Name, r.Name, r.Size)
	fmt.Fprintf(g, "func Encode%s(value %s) [%sBytes]byte {\nvar p [%sBytes]byte\n", r.Name, r.Name, r.Name, r.Name)
	wireLoopLeaves(c, g, r, known, "value", "value", "", 0, func(c, g *strings.Builder, f wireField, cv, gv, off string) {
		if f.Padding {
			fmt.Fprintf(c, "  bytes[%s] = 0;\n", off)
			return
		}
		width := scalarWidth[f.base]
		fmt.Fprintf(c, "  detail::store_native_le<uint%d_t>(bytes.data()+%s, %s);\n", width*8, off, cv)
		if width == 1 {
			fmt.Fprintf(g, "p[%s] = byte(%s)\n", off, gv)
		} else {
			value := fmt.Sprintf("uint%d(%s)", width*8, gv)
			if f.base == "f64" {
				value = "math.Float64bits(" + gv + ")"
			}
			fmt.Fprintf(g, "binary.LittleEndian.PutUint%d(p[%s:], %s)\n", width*8, off, value)
		}
	})
	c.WriteString("  return bytes;\n}\n")
	g.WriteString("return p\n}\n")
	// Raw decoding remains private; public entry points validate before exposure.
	fmt.Fprintf(c, "inline bool decode_%s_raw(const uint8_t* bytes, std::size_t size, %s& out) noexcept {\n  if (!bytes || size != %d) return false;\n  %s decoded{};\n", r.Name, r.Name, r.Size, r.Name)
	fmt.Fprintf(g, "func decode%sRaw(p []byte) %s {\nvar decoded %s\n", r.Name, r.Name, r.Name)
	wireLoopLeaves(c, g, r, known, "decoded", "decoded", "", 0, func(c, g *strings.Builder, f wireField, cv, gv, off string) {
		width := scalarWidth[f.base]
		fmt.Fprintf(c, "  detail::load_native_le<uint%d_t>(bytes+%s, %s);\n", width*8, off, cv)
		value := fmt.Sprintf("binary.LittleEndian.Uint%d(p[%s:])", width*8, off)
		if width == 1 {
			value = "p[" + off + "]"
		}
		if f.base == "f64" {
			value = "math.Float64frombits(" + value + ")"
		} else {
			value = goScalar[f.base] + "(" + value + ")"
		}
		fmt.Fprintf(g, "%s = %s\n", gv, value)
	})
	c.WriteString("  out = decoded; return true;\n}\n")
	g.WriteString("return decoded\n}\n")
}

func writeFrameDecoder(c, g *strings.Builder, r wireRecord, known map[string]wireRecord) {
	cName := "decode_" + r.key + "_frame"
	fmt.Fprintf(c, "// Decode a complete private frame; failure leaves out unchanged.\ninline bool %s(const uint8_t* bytes, std::size_t size, %s& out) noexcept {\n  if (!bytes || size != %d) return false;\n", cName, r.Name, r.Size)
	fmt.Fprintf(g, "// Decode%sFrame rejects malformed fixed frames before exposing any fields.\nfunc Decode%sFrame(p []byte) (%s, error) {\n", r.Name, r.Name, r.Name)
	fail := func(c, g *strings.Builder, field, goCond, cppCond string) {
		label := strconv.Quote(field)
		var indices []string
		for _, part := range strings.Split(field, "[")[1:] {
			index, _, _ := strings.Cut(part, "]")
			indices = append(indices, index)
			label = strings.Replace(label, "["+index+"]", "[%d]", 1)
		}
		if len(indices) != 0 {
			label = "fmt.Sprintf(" + label + ", " + strings.Join(indices, ", ") + ")"
		}
		fmt.Fprintf(g, "if %s { return %s{}, &MalformedFrameError{Record: %q, Field: %s} }\n", goCond, r.Name, r.Name, label)
		fmt.Fprintf(c, "  if (%s) return false;\n", cppCond)
	}
	// The C++ size guard above and Go guard below precede every byte load.
	fmt.Fprintf(g, "if len(p) != %d { return %s{}, &MalformedFrameError{Record: %q, Field: \"size\"} }\n", r.Size, r.Name, r.Name)
	if r.MajorField != "" {
		for _, f := range r.Fields {
			if f.Name == r.MajorField {
				fail(c, g, f.Name, fmt.Sprintf("binary.LittleEndian.Uint16(p[%d:]) != VersionMajor", f.Offset), fmt.Sprintf("(uint16_t(bytes[%d]) | (uint16_t(bytes[%d]) << 8)) != kVerMajor", f.Offset, f.Offset+1))
			}
		}
	}
	wireLoopLeaves(c, g, r, known, "value", "value", "", 0, func(c, g *strings.Builder, f wireField, cv, gv, off string) {
		width := scalarWidth[f.base]
		if f.Reserved {
			for i := 0; i < width; i++ {
				fail(c, g, cv, fmt.Sprintf("p[%s+%d] != 0", off, i), fmt.Sprintf("bytes[%s+%d] != 0", off, i))
			}
		}
		if f.AllowedBits != nil {
			read := fmt.Sprintf("binary.LittleEndian.Uint%d(p[%s:])", width*8, off)
			if width == 1 {
				read = "p[" + off + "]"
			}
			mask := uint64(0)
			if width == 8 {
				mask = ^*f.AllowedBits
			} else {
				mask = ((uint64(1) << (8 * width)) - 1) ^ *f.AllowedBits
			}
			var terms []string
			for i := 0; i < width; i++ {
				terms = append(terms, fmt.Sprintf("(uint64_t(bytes[%s+%d]) << %d)", off, i, 8*i))
			}
			fail(c, g, cv, fmt.Sprintf("%s & %d != 0", read, mask), fmt.Sprintf("((%s) & UINT64_C(%d)) != 0", strings.Join(terms, " | "), mask))
		}
	})
	if r.legacy {
		if r.key == "command_result" {
			c.WriteString("  CommandResult decoded{};\n")
			for _, f := range r.Fields {
				fmt.Fprintf(c, "  for (std::size_t b=0;b<%d;++b) decoded.%s |= static_cast<uint%d_t>(bytes[%d+b]) << (8*b);\n", f.Size, f.Name, f.Size*8, f.Offset)
			}
			c.WriteString("  out=decoded; return true;\n}\n")
			g.WriteString("return decodeCommandResult(p), nil\n}\n")
			// CommandResult previously exposed only a decoder; generate its encoder.
			fmt.Fprintf(g, "func EncodeCommandResult(value CommandResult) [CommandResultBytes]byte {\nvar p [CommandResultBytes]byte\n")
			fmt.Fprintf(c, "inline std::array<uint8_t, %d> encode_command_result(const CommandResult& value) noexcept {\nstd::array<uint8_t, %d> bytes{};\n", r.Size, r.Size)
			for _, f := range r.Fields {
				fmt.Fprintf(g, "binary.LittleEndian.PutUint%d(p[%d:], value.%s)\n", f.Size*8, f.Offset, camel(f.Name))
				fmt.Fprintf(c, "for (std::size_t b=0;b<%d;++b) bytes[%d+b]=static_cast<uint8_t>(value.%s >> (8*b));\n", f.Size, f.Offset, f.Name)
			}
			g.WriteString("return p\n}\n")
			c.WriteString("return bytes;\n}\n")
		} else {
			fmt.Fprintf(c, "  return decode_%s(bytes, size, out);\n}\n", r.key)
			fmt.Fprintf(g, "value, _ := Decode%s(p)\nreturn value, nil\n}\n", r.Name)
		}
	} else {
		fmt.Fprintf(c, "  return decode_%s_raw(bytes, size, out);\n}\ninline bool decode_%s(const uint8_t* bytes, std::size_t size, %s& out) noexcept { return %s(bytes,size,out); }\n", r.Name, r.Name, r.Name, cName)
		fmt.Fprintf(g, "return decode%sRaw(p), nil\n}\nfunc Decode%s(p []byte) (%s,error) { return Decode%sFrame(p) }\n", r.Name, r.Name, r.Name, r.Name)
	}
}
