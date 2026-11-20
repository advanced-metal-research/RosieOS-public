package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"go/format"
	"io"
	"regexp"
	"strconv"
	"strings"
)

type constant struct {
	name  string
	value uint32
}

type field struct {
	name  string
	kind  string
	width int
	count int // zero for a scalar, otherwise the fixed array length
}

type protocolSchema struct {
	constants       []constant
	fields          []field
	records         []record
	sharedConstants []sharedConstant // C++/Go contract; no new transitional Python surface.
	wireRecords     []wireRecord
	wireConstants   []wireConstant
	wireEnums       []wireEnum
}

type record struct {
	name   string
	fields []field
	width  uint32
}

type rawMember struct {
	name  string
	value json.RawMessage
}

// JSON object order is part of the existing generator's artifact format. A Go
// map would lose it; reject duplicate keys rather than silently picking a value.
func object(data []byte) ([]rawMember, error) {
	decoder := json.NewDecoder(bytes.NewReader(data))
	opening, err := decoder.Token()
	if err != nil || opening != json.Delim('{') {
		return nil, fmt.Errorf("expected JSON object")
	}
	var members []rawMember
	seen := make(map[string]bool)
	for decoder.More() {
		key, err := decoder.Token()
		if err != nil {
			return nil, err
		}
		name, ok := key.(string)
		if !ok || seen[name] {
			return nil, fmt.Errorf("duplicate or invalid object key %q", key)
		}
		seen[name] = true
		var value json.RawMessage
		if err := decoder.Decode(&value); err != nil {
			return nil, err
		}
		members = append(members, rawMember{name, value})
	}
	if _, err := decoder.Token(); err != nil {
		return nil, err
	}
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); err != io.EOF {
		return nil, fmt.Errorf("unexpected data after object")
	}
	return members, nil
}

var identifier = regexp.MustCompile(`^[a-z][a-z0-9]*(?:_[a-z0-9]+)*$`)

func parseSchema(data []byte) (protocolSchema, error) {
	var schema protocolSchema
	members, err := object(data)
	if err != nil {
		return schema, err
	}
	values := make(map[string]json.RawMessage)
	for _, member := range members {
		switch member.name {
		case "version_major", "version_minor", "max_axes", "max_trajectory_points", "messages", "results", "reasons", "command_result":
			values[member.name] = member.value
		case "records", "grant_operations", "grant_reasons", "jog_operations", "jog_reasons", "fast_control", "wire_contract", "native_records", "native_constants", "native_enums":
			values[member.name] = member.value
		default:
			return schema, fmt.Errorf("unknown schema member %q", member.name)
		}
	}
	for _, required := range []string{"version_major", "version_minor", "max_axes", "max_trajectory_points", "messages", "results", "reasons", "command_result"} {
		if _, ok := values[required]; !ok {
			return schema, fmt.Errorf("required schema member %q is missing", required)
		}
	}
	for _, name := range []string{"version_major", "version_minor", "max_axes", "max_trajectory_points"} {
		var value uint32
		if err := json.Unmarshal(values[name], &value); err != nil || bytes.Equal(bytes.TrimSpace(values[name]), []byte("null")) {
			return schema, fmt.Errorf("%s must be an unsigned 32-bit integer", name)
		}
		if (name == "version_major" && value == 0) || (name == "max_axes" && (value == 0 || value > 32)) ||
			(name == "max_trajectory_points" && value < 2) {
			return schema, fmt.Errorf("invalid %s: %d", name, value)
		}
		schema.constants = append(schema.constants, constant{name, value})
	}
	for _, group := range []struct{ name, prefix string }{
		{"messages", "msg"}, {"results", "result"}, {"reasons", "reason"},
		{"grant_operations", "grant_operation"}, {"grant_reasons", "grant_reason"},
		{"jog_operations", "jog_operation"},
		{"jog_reasons", "jog_reason"},
	} {
		encoded, present := values[group.name]
		if !present { // Optional grant enums leave earlier schemas byte-identical.
			continue
		}
		entries, err := object(encoded)
		if err != nil || len(entries) == 0 {
			return schema, fmt.Errorf("%s must be a nonempty object: %v", group.name, err)
		}
		for _, entry := range entries {
			var value uint32
			if !identifier.MatchString(entry.name) {
				return schema, fmt.Errorf("invalid %s identifier %q", group.name, entry.name)
			}
			if err := json.Unmarshal(entry.value, &value); err != nil || bytes.Equal(bytes.TrimSpace(entry.value), []byte("null")) {
				return schema, fmt.Errorf("%s.%s must be an unsigned 32-bit integer", group.name, entry.name)
			}
			schema.constants = append(schema.constants, constant{group.prefix + "_" + entry.name, value})
		}
	}
	result, err := parseRecord("command_result", values["command_result"])
	if err != nil {
		return schema, err
	}
	for _, field := range result.fields {
		if field.count != 0 || (field.kind != "u32" && field.kind != "u64") {
			return schema, fmt.Errorf("command_result supports only scalar u32/u64 fields")
		}
	}
	schema.fields = result.fields
	schema.constants = append(schema.constants, constant{"command_result_bytes", result.width})
	if encoded, ok := values["records"]; ok {
		entries, err := object(encoded)
		if err != nil {
			return schema, fmt.Errorf("records must be an object: %w", err)
		}
		seenTypes := map[string]bool{"CommandResult": true}
		for _, entry := range entries {
			if !identifier.MatchString(entry.name) || seenTypes[camel(entry.name)] {
				return schema, fmt.Errorf("invalid or duplicate record identifier %q", entry.name)
			}
			seenTypes[camel(entry.name)] = true
			record, err := parseRecord(entry.name, entry.value)
			if err != nil {
				return schema, err
			}
			schema.records = append(schema.records, record)
			schema.constants = append(schema.constants, constant{entry.name + "_bytes", record.width})
		}
	}
	if encoded, ok := values["fast_control"]; ok {
		schema.sharedConstants, err = parseFastControl(encoded, schema.records)
		if err != nil {
			return schema, err
		}
	}
	seenConstants := make(map[string]bool)
	for _, entry := range schema.constants {
		name := camel(entry.name)
		if seenConstants[name] {
			return schema, fmt.Errorf("duplicate exported constant %q", name)
		}
		seenConstants[name] = true
	}
	for _, entry := range schema.sharedConstants {
		name := camel(entry.name)
		if seenConstants[name] {
			return schema, fmt.Errorf("duplicate exported constant %q", name)
		}
		seenConstants[name] = true
	}
	if err := parseNative(&schema, values); err != nil {
		return schema, err
	}
	return schema, nil
}

// These are small, fixed control records, not arbitrary buffers or nested data.
// Both bounds are checked before multiplication/addition determines the layout.
const maxArrayElements = 16

// Unverified generator allocation ceiling: 8192 bytes; measured capture V2 is 5392.
// Transport-specific command bounds remain independently enforced.
const maxRecordBytes = 8192

var arrayType = regexp.MustCompile(`^([a-z][a-z0-9]*)\[([0-9]+)\]$`)

// bound is the field's explicit schema array_bound, or zero for the default.
func parseFieldType(kind string, bound int) (base string, count, elementBytes int, err error) {
	base = kind
	if parts := arrayType.FindStringSubmatch(kind); parts != nil {
		base = parts[1]
		count, err = strconv.Atoi(parts[2])
		// The local grant token is exactly 32 bytes; a field may declare a
		// larger explicit array_bound (the path sample chunk); all other arrays
		// retain the 16-element bound (unverified resource sizing). The record
		// byte ceiling still applies before any layout is accepted.
		limit := maxArrayElements
		if bound > limit {
			limit = bound
		}
		if err != nil || count < 1 || (count > limit && !(base == "u8" && count == 32)) || strconv.Itoa(count) != parts[2] {
			return "", 0, 0, fmt.Errorf("array length must be an integer from 1 to %d: %q", limit, kind)
		}
		if base != "u8" && base != "f64" && base != "i32" {
			return "", 0, 0, fmt.Errorf("unsupported array element type %q", base)
		}
	} else if kind != "u32" && kind != "u64" && kind != "f64" {
		return "", 0, 0, fmt.Errorf("unsupported field type %q", kind)
	}
	elementBytes = map[string]int{"u8": 1, "i32": 4, "u32": 4, "u64": 8, "f64": 8}[base]
	return base, count, elementBytes, nil
}

func parseRecord(name string, encoded []byte) (record, error) {
	parsed := record{name: name}
	bounds := map[string]int{}
	if len(bytes.TrimSpace(encoded)) > 0 && bytes.TrimSpace(encoded)[0] == '{' {
		layout, err := parseWireRecord(name, encoded, nil)
		if err != nil {
			return parsed, err
		}
		var pairs [][]string
		for _, f := range layout.Fields {
			pairs = append(pairs, []string{f.Name, f.Type})
			if f.ArrayBound != nil {
				bounds[f.Name] = *f.ArrayBound
			}
		}
		encoded, _ = json.Marshal(pairs)
	}
	var fields [][]string
	if err := json.Unmarshal(encoded, &fields); err != nil || len(fields) == 0 {
		return parsed, fmt.Errorf("%s must contain [name,type] pairs", name)
	}
	seen := make(map[string]bool)
	offset, alignment := 0, 0
	for _, pair := range fields {
		if len(pair) != 2 || !identifier.MatchString(pair[0]) {
			return parsed, fmt.Errorf("invalid %s field %v", name, pair)
		}
		name, kind := pair[0], pair[1]
		if seen[camel(name)] || cppKeyword(name) {
			return parsed, fmt.Errorf("duplicate or reserved field identifier %q", name)
		}
		seen[camel(name)] = true
		base, count, elementBytes, err := parseFieldType(kind, bounds[name])
		if err != nil {
			return parsed, err
		}
		width := elementBytes
		if count != 0 {
			width *= count
		}
		if width > maxRecordBytes-offset {
			return parsed, fmt.Errorf("record %s exceeds %d-byte layout limit", parsed.name, maxRecordBytes)
		}
		if offset%elementBytes != 0 {
			return parsed, fmt.Errorf("field %s introduces implicit C++ padding", name)
		}
		parsed.fields = append(parsed.fields, field{name: name, kind: base, width: width, count: count})
		offset += width
		if elementBytes > alignment {
			alignment = elementBytes
		}
	}
	if offset%alignment != 0 {
		return parsed, fmt.Errorf("%s introduces implicit C++ tail padding", name)
	}
	parsed.width = uint32(offset)
	return parsed, nil
}

func cppKeyword(name string) bool {
	const keywords = " alignas alignof and and_eq asm auto bitand bitor bool break case catch char char16_t char32_t class compl const constexpr const_cast continue decltype default delete do double dynamic_cast else enum explicit export extern false float for friend goto if inline int long mutable namespace new noexcept not not_eq nullptr operator or or_eq private protected public register reinterpret_cast return short signed sizeof static static_assert static_cast struct switch template this thread_local throw true try typedef typeid typename union unsigned using virtual void volatile wchar_t while xor xor_eq "
	return strings.Contains(keywords, " "+name+" ")
}

// Match Python str.title on the schema's restricted ASCII identifiers, including
// letter groups after digits. This keeps existing exported Go names unchanged.
func camel(name string) string {
	var output strings.Builder
	initial := true
	for _, ch := range name {
		if ch == '_' {
			initial = true
			continue
		}
		if ch >= 'a' && ch <= 'z' {
			if initial {
				ch -= 'a' - 'A'
			}
			initial = false
		} else {
			initial = true
		}
		output.WriteRune(ch)
	}
	return output.String()
}

type artifact struct {
	path string
	data []byte
}

func generate(schema protocolSchema) ([]artifact, error) {
	var cpp, goSource strings.Builder
	hasFloat := false
	for _, record := range schema.records {
		for _, field := range record.fields {
			hasFloat = hasFloat || field.kind == "f64"
		}
	}
	cpp.WriteString("// Generated by tools/protocolgen; edit protocol/control.json.\n#pragma once\n#include <cstdint>\n")
	if len(schema.records) != 0 {
		cpp.WriteString("#include <array>\n#include <cstddef>\n")
	}
	if hasFloat {
		cpp.WriteString("#include <cstring>\n#include <limits>\n")
	}
	cpp.WriteString("namespace rosie::protocol {\n")
	if hasFloat {
		cpp.WriteString("static_assert(sizeof(double) == sizeof(uint64_t) && std::numeric_limits<double>::is_iec559);\n")
	}
	goSource.WriteString("// Code generated by tools/protocolgen; DO NOT EDIT.\n\npackage ipcclient\n\n")
	if hasFloat {
		goSource.WriteString("import (\n\"encoding/binary\"\n\"math\"\n)\n\n")
	} else {
		goSource.WriteString("import \"encoding/binary\"\n\n")
	}
	for _, entry := range schema.constants {
		fmt.Fprintf(&cpp, "inline constexpr uint32_t %s = %d;\n", entry.name, entry.value)
		fmt.Fprintf(&goSource, "const %s = %d\n", camel(entry.name), entry.value)
	}
	for _, entry := range schema.sharedConstants {
		fmt.Fprintf(&cpp, "inline constexpr uint64_t %s = UINT64_C(%d);\n", entry.name, entry.value)
		fmt.Fprintf(&goSource, "const %s = %d\n", camel(entry.name), entry.value)
	}
	cpp.WriteString("struct CommandResult {\n")
	goSource.WriteString("\ntype CommandResult struct {\n")
	for _, field := range schema.fields {
		fmt.Fprintf(&cpp, "  uint%d_t %s;\n", field.width*8, field.name)
		fmt.Fprintf(&goSource, "\t%s uint%d\n", camel(field.name), field.width*8)
	}
	cpp.WriteString("};\nstatic_assert(sizeof(CommandResult) == command_result_bytes);\n")
	goSource.WriteString("}\n\nfunc decodeCommandResult(p []byte) CommandResult {\n\treturn CommandResult{\n")
	offset := 0
	for _, field := range schema.fields {
		fmt.Fprintf(&goSource, "\t\t%s: binary.LittleEndian.Uint%d(p[%d:]),\n", camel(field.name), field.width*8, offset)
		offset += field.width
	}
	goSource.WriteString("\t}\n}\n")
	for _, record := range schema.records {
		writeRecord(&cpp, &goSource, record)
	}
	cpp.WriteString("}\n")
	native, err := generateNative(schema, &goSource)
	if err != nil {
		return nil, err
	}
	goText := goSource.String()
	if strings.Contains(goText, "fmt.Sprintf(") {
		goText = strings.Replace(goText, "import \"encoding/binary\"", "import (\n\"encoding/binary\"\n)", 1)
		goText = strings.Replace(goText, "import (\n", "import (\n\"fmt\"\n", 1)
	}
	formattedGo, err := format.Source([]byte(goText))
	if err != nil {
		return nil, fmt.Errorf("format generated Go: %w", err)
	}
	outputs := []artifact{
		{"clients/cpp/include/rosie/control_protocol.hpp", []byte(cpp.String())},
		{"ipcclient/protocol_generated.go", formattedGo},
		{"include/control_protocol.hpp", []byte("#include \"../clients/cpp/include/rosie/control_protocol.hpp\"\n")},
	}
	if native != nil {
		// The SDK owns the declarations; forwarding includes retain daemon paths
		// and these guards prevent redeclarations when both paths are included.
		outputs[0].data = []byte("#ifndef ROSIE_CONTROL_PROTOCOL_GENERATED_HPP\n#define ROSIE_CONTROL_PROTOCOL_GENERATED_HPP\n" + cpp.String() + "#endif\n")
		native = []byte("#ifndef ROSIE_NATIVE_PROTOCOL_GENERATED_HPP\n#define ROSIE_NATIVE_PROTOCOL_GENERATED_HPP\n" + string(native) + "#endif\n")
		outputs = append(outputs, artifact{"include/protocol_generated.hpp", []byte("#include \"../clients/cpp/include/rosie/protocol_generated.hpp\"\n")})
		outputs = append(outputs, artifact{"clients/ts/rt_protocol_generated.ts", generateTypeScript(schema)})
		outputs = append(outputs, artifact{"clients/cpp/include/rosie/protocol_generated.hpp", native})
	}
	return outputs, nil
}

func writeRecord(cpp, goSource *strings.Builder, record record) {
	typeName := camel(record.name)
	sizeName := record.name + "_bytes"
	goSizeName := camel(sizeName)
	extended, hasFloat := false, false
	fmt.Fprintf(cpp, "struct %s {\n", typeName)
	fmt.Fprintf(goSource, "\ntype %s struct {\n", typeName)
	for _, field := range record.fields {
		cppType, goType := map[string]string{"u8": "uint8_t", "i32": "int32_t", "u32": "uint32_t", "u64": "uint64_t", "f64": "double"}[field.kind],
			map[string]string{"u8": "byte", "i32": "int32", "u32": "uint32", "u64": "uint64", "f64": "float64"}[field.kind]
		if field.count != 0 {
			cppType = fmt.Sprintf("std::array<%s, %d>", cppType, field.count)
			goType = fmt.Sprintf("[%d]%s", field.count, goType)
		}
		fmt.Fprintf(cpp, "  %s %s;\n", cppType, field.name)
		fmt.Fprintf(goSource, "\t%s %s\n", camel(field.name), goType)
		extended = extended || field.count != 0 || field.kind == "f64"
		hasFloat = hasFloat || field.kind == "f64"
	}
	fmt.Fprintf(cpp, "};\nstatic_assert(sizeof(%s) == %s);\n", typeName, sizeName)
	if extended {
		offset := 0
		for _, field := range record.fields {
			fmt.Fprintf(cpp, "static_assert(offsetof(%s, %s) == %d);\n", typeName, field.name, offset)
			offset += field.width
		}
	}
	fmt.Fprintf(cpp, "inline std::array<uint8_t, %s> encode_%s(const %s& value) noexcept {\n  std::array<uint8_t, %s> bytes{};\n", sizeName, record.name, typeName, sizeName)
	fmt.Fprintf(goSource, "}\n\nfunc Encode%s(value %s) [%s]byte {\n\tvar p [%s]byte\n", typeName, typeName, goSizeName, goSizeName)
	offset := 0
	for _, field := range record.fields {
		switch field.kind {
		case "u8":
			fmt.Fprintf(cpp, "  for (std::size_t element = 0; element < %d; ++element)\n    bytes[%d + element] = value.%s[element];\n", field.count, offset, field.name)
			fmt.Fprintf(goSource, "\tcopy(p[%d:%d], value.%s[:])\n", offset, offset+field.width, camel(field.name))
		case "i32":
			fmt.Fprintf(cpp, "  for (std::size_t element = 0; element < %d; ++element) {\n    const uint32_t bits = static_cast<uint32_t>(value.%s[element]);\n    for (std::size_t byte = 0; byte < 4; ++byte)\n      bytes[%d + element * 4 + byte] = static_cast<uint8_t>(bits >> (8 * byte));\n  }\n", field.count, field.name, offset)
			fmt.Fprintf(goSource, "\tfor index, element := range value.%s {\n\t\tbinary.LittleEndian.PutUint32(p[%d+index*4:%d+index*4], uint32(element))\n\t}\n", camel(field.name), offset, offset+4)
		case "f64":
			cppValue, cppOffset := "value."+field.name, strconv.Itoa(offset)
			if field.count != 0 {
				fmt.Fprintf(cpp, "  for (std::size_t element = 0; element < %d; ++element) {\n", field.count)
				cppValue += "[element]"
				cppOffset += " + element * 8"
				fmt.Fprintf(goSource, "\tfor index, element := range value.%s {\n\t\tbinary.LittleEndian.PutUint64(p[%d+index*8:%d+index*8], math.Float64bits(element))\n\t}\n", camel(field.name), offset, offset+8)
			} else {
				cpp.WriteString("  {\n")
				fmt.Fprintf(goSource, "\tbinary.LittleEndian.PutUint64(p[%d:%d], math.Float64bits(value.%s))\n", offset, offset+8, camel(field.name))
			}
			fmt.Fprintf(cpp, "    uint64_t bits = 0;\n    std::memcpy(&bits, &%s, sizeof(bits));\n    for (std::size_t byte = 0; byte < 8; ++byte)\n      bytes[%s + byte] = static_cast<uint8_t>(bits >> (8 * byte));\n  }\n", cppValue, cppOffset)
		default:
			fmt.Fprintf(cpp, "  for (std::size_t byte = 0; byte < %d; ++byte)\n    bytes[%d + byte] = static_cast<uint8_t>(value.%s >> (8 * byte));\n", field.width, offset, field.name)
			fmt.Fprintf(goSource, "\tbinary.LittleEndian.PutUint%d(p[%d:%d], value.%s)\n", field.width*8, offset, offset+field.width, camel(field.name))
		}
		offset += field.width
	}
	cpp.WriteString("  return bytes;\n}\n")
	fmt.Fprintf(cpp, "inline bool decode_%s(const uint8_t* bytes, std::size_t size, %s& out) noexcept {\n  if (!bytes || size != %s) return false;\n  %s decoded{};\n", record.name, typeName, sizeName, typeName)
	fmt.Fprintf(goSource, "\treturn p\n}\n\nfunc Decode%s(p []byte) (%s, bool) {\n\tif len(p) != %s {\n\t\treturn %s{}, false\n\t}\n", typeName, typeName, goSizeName, typeName)
	if extended {
		fmt.Fprintf(goSource, "\tvar decoded %s\n", typeName)
	} else {
		fmt.Fprintf(goSource, "\treturn %s{\n", typeName)
	}
	offset = 0
	for _, field := range record.fields {
		switch field.kind {
		case "u8":
			fmt.Fprintf(cpp, "  for (std::size_t element = 0; element < %d; ++element)\n    decoded.%s[element] = bytes[%d + element];\n", field.count, field.name, offset)
			fmt.Fprintf(goSource, "\tcopy(decoded.%s[:], p[%d:%d])\n", camel(field.name), offset, offset+field.width)
		case "i32":
			fmt.Fprintf(cpp, "  for (std::size_t element = 0; element < %d; ++element) {\n    uint32_t bits = 0;\n    for (std::size_t byte = 0; byte < 4; ++byte)\n      bits |= static_cast<uint32_t>(bytes[%d + element * 4 + byte]) << (8 * byte);\n    std::memcpy(&decoded.%s[element], &bits, sizeof(bits));\n  }\n", field.count, offset, field.name)
			fmt.Fprintf(goSource, "\tfor index := range decoded.%s {\n\t\tdecoded.%s[index] = int32(binary.LittleEndian.Uint32(p[%d+index*4:%d+index*4]))\n\t}\n", camel(field.name), camel(field.name), offset, offset+4)
		case "f64":
			cppValue, cppOffset := "decoded."+field.name, strconv.Itoa(offset)
			if field.count != 0 {
				fmt.Fprintf(cpp, "  for (std::size_t element = 0; element < %d; ++element) {\n", field.count)
				cppValue += "[element]"
				cppOffset += " + element * 8"
				fmt.Fprintf(goSource, "\tfor index := range decoded.%s {\n\t\tdecoded.%s[index] = math.Float64frombits(binary.LittleEndian.Uint64(p[%d+index*8:%d+index*8]))\n\t}\n", camel(field.name), camel(field.name), offset, offset+8)
			} else {
				cpp.WriteString("  {\n")
				fmt.Fprintf(goSource, "\tdecoded.%s = math.Float64frombits(binary.LittleEndian.Uint64(p[%d:%d]))\n", camel(field.name), offset, offset+8)
			}
			fmt.Fprintf(cpp, "    uint64_t bits = 0;\n    for (std::size_t byte = 0; byte < 8; ++byte)\n      bits |= static_cast<uint64_t>(bytes[%s + byte]) << (8 * byte);\n    std::memcpy(&%s, &bits, sizeof(bits));\n  }\n", cppOffset, cppValue)
		default:
			fmt.Fprintf(cpp, "  for (std::size_t byte = 0; byte < %d; ++byte)\n    decoded.%s |= static_cast<uint%d_t>(bytes[%d + byte]) << (8 * byte);\n", field.width, field.name, field.width*8, offset)
			if extended {
				fmt.Fprintf(goSource, "\tdecoded.%s = binary.LittleEndian.Uint%d(p[%d:%d])\n", camel(field.name), field.width*8, offset, offset+field.width)
			} else {
				fmt.Fprintf(goSource, "\t\t%s: binary.LittleEndian.Uint%d(p[%d:%d]),\n", camel(field.name), field.width*8, offset, offset+field.width)
			}
		}
		offset += field.width
	}
	if hasFloat {
		cpp.WriteString("  std::memcpy(&out, &decoded, sizeof(decoded));\n")
	} else {
		cpp.WriteString("  out = decoded;\n")
	}
	cpp.WriteString("  return true;\n}\n")
	if extended {
		goSource.WriteString("\treturn decoded, true\n}\n")
	} else {
		goSource.WriteString("\t}, true\n}\n")
	}
}

func equivalent(path string, actual, expected []byte) bool {
	normalize := func(data []byte) []byte {
		data = bytes.ReplaceAll(data, []byte("\r\n"), []byte("\n"))
		if strings.HasSuffix(path, ".go") {
			if formatted, err := format.Source(data); err == nil {
				return formatted
			}
		}
		return data
	}
	return bytes.Equal(normalize(actual), normalize(expected))
}
