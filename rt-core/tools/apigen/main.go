// Command apigen generates the public contract from canonical schema bytes.
// Check mode is read-only and refuses drift so clients cannot silently diverge.
package main

import (
	"bytes"
	"crypto/sha256"
	_ "embed"
	"encoding/json"
	"flag"
	"fmt"
	"go/format"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"
)

type Field struct {
	Name        string `json:"name"`
	JSON        string `json:"json"`
	Type        string `json:"type"`
	JSONType    string `json:"json_type"`
	Required    bool   `json:"required"`
	OmitEmpty   bool   `json:"omit_empty,omitempty"`
	Embedded    bool   `json:"embedded,omitempty"`
	Constraints string `json:"constraints"`
}
type Type struct {
	Binary   bool    `json:"binary,omitempty"`
	Name     string  `json:"name"`
	Alias    string  `json:"go_alias,omitempty"`
	Existing bool    `json:"go_existing,omitempty"`
	Fields   []Field `json:"fields"`
}
type Reason struct {
	Name    string `json:"name"`
	Meaning string `json:"meaning"`
}
type CapabilitySpec struct {
	Name           string   `json:"name"`
	Description    string   `json:"description,omitempty"`
	State          string   `json:"state"`
	Transport      string   `json:"transport"`
	RequestType    string   `json:"request_type"`
	RequiredFields []string `json:"required_fields"`
	ResponseType   string   `json:"response_type"`
}
type Schema struct {
	ContractVersion  uint32           `json:"contract_version"`
	RequestSchema    string           `json:"request_schema"`
	ResponseSchema   string           `json:"response_schema"`
	CanonicalBytes   string           `json:"canonical_bytes"`
	Types            []Type           `json:"types"`
	Capabilities     []CapabilitySpec `json:"capabilities"`
	CapabilityStates []string         `json:"capability_states"`
	Reasons          []Reason         `json:"reasons"`
	HandleStates     []string         `json:"handle_states"`
	Rules            json.RawMessage  `json:"rules"`
}

//go:embed json.hpp.tmpl
var cppRuntime string

func identifier(s string) string {
	var out strings.Builder
	upper := true
	for _, r := range s {
		if r >= 'a' && r <= 'z' || r >= 'A' && r <= 'Z' || r >= '0' && r <= '9' {
			if upper && r >= 'a' && r <= 'z' {
				r -= 32
			}
			out.WriteRune(r)
			upper = false
		} else {
			upper = true
		}
	}
	return out.String()
}
func base(t string) string { return regexp.MustCompile(`^(\*|\[[0-9]*\])+`).ReplaceAllString(t, "") }
func scalar(t string) bool {
	switch t {
	case "string", "bool", "int", "int64", "int32", "uint64", "uint32", "uint16", "uint8", "float64", "json", "rawjson", "HandleState":
		return true
	}
	return false
}

// scalarMap accepts only string keys and primitive values, preventing unsupported
// containers or structured values from reaching either generated codec.
func scalarMap(t string) bool {
	if !strings.HasPrefix(t, "map[string]") {
		return false
	}
	switch strings.TrimPrefix(t, "map[string]") {
	case "string", "bool", "int", "int64", "int32", "uint64", "uint32", "uint16", "uint8", "float64":
		return true
	}
	return false
}

// parse rejects ambiguous names and unsupported types before emitting any file.
func parse(body []byte) (Schema, error) {
	var s Schema
	d := json.NewDecoder(bytes.NewReader(body))
	d.DisallowUnknownFields()
	if err := d.Decode(&s); err != nil {
		return s, err
	}
	if d.Decode(new(any)) != io.EOF {
		return s, fmt.Errorf("trailing schema data")
	}
	if s.ContractVersion != 2 || s.RequestSchema != "rosie.rt-control.request.v1" || s.ResponseSchema != "rosie.rt-control.response.v1" {
		return s, fmt.Errorf("unsupported contract identity")
	}
	if !bytes.HasSuffix(body, []byte("\n")) || bytes.Contains(body, []byte("\r")) {
		return s, fmt.Errorf("schema must use LF with a final newline")
	}
	names := map[string]bool{}
	for _, t := range s.Types {
		if t.Name == "" || names[t.Name] {
			return s, fmt.Errorf("duplicate or empty type %q", t.Name)
		}
		names[t.Name] = true
	}
	for _, t := range s.Types {
		fields := map[string]bool{}
		for _, f := range t.Fields {
			if fields[f.JSON] || f.JSON == "" {
				return s, fmt.Errorf("duplicate field %s.%s", t.Name, f.JSON)
			}
			fields[f.JSON] = true
			if !scalar(base(f.Type)) && !scalarMap(base(f.Type)) && !names[base(f.Type)] {
				return s, fmt.Errorf("unknown type %s.%s: %s", t.Name, f.Name, f.Type)
			}
			if f.Constraints == "" {
				return s, fmt.Errorf("missing constraints %s.%s", t.Name, f.Name)
			}
		}
	}
	names = map[string]bool{}
	constants := map[string]bool{}
	for _, r := range s.Reasons {
		id := identifier(r.Name)
		if names[r.Name] || constants[id] || r.Name == "" || r.Meaning == "" || strings.ContainsAny(r.Meaning, "\r\n") {
			return s, fmt.Errorf("duplicate or invalid reason %q", r.Name)
		}
		names[r.Name] = true
		constants[id] = true
	}
	names = map[string]bool{}
	states := map[string]bool{}
	for _, state := range s.CapabilityStates {
		if state == "" || states[state] {
			return s, fmt.Errorf("duplicate or empty capability state %q", state)
		}
		states[state] = true
	}
	for _, c := range s.Capabilities {
		if names[c.Name] || c.Name == "" {
			return s, fmt.Errorf("duplicate capability %q", c.Name)
		}
		names[c.Name] = true
		if !states[c.State] {
			return s, fmt.Errorf("unknown capability state %q", c.State)
		}
	}
	return s, nil
}
func goType(t string) string {
	switch t {
	case "json":
		return "any"
	case "rawjson":
		return "json.RawMessage"
	}
	return t
}
func cppType(t string) string {
	if scalarMap(t) {
		return "std::map<std::string, " + cppType(strings.TrimPrefix(t, "map[string]")) + ">"
	}
	if strings.HasPrefix(t, "*") {
		return "std::optional<" + cppType(t[1:]) + ">"
	}
	if strings.HasPrefix(t, "[]") {
		return "std::vector<" + cppType(t[2:]) + ">"
	}
	if strings.HasPrefix(t, "[") {
		end := strings.Index(t, "]")
		return "std::array<" + cppType(t[end+1:]) + ", " + t[1:end] + ">"
	}
	switch t {
	case "string", "HandleState":
		return "std::string"
	case "int":
		return "std::int64_t"
	case "float64":
		return "double"
	case "json", "rawjson":
		return "Json"
	case "uint8", "uint16", "uint32", "uint64", "int32", "int64":
		return "std::" + t + "_t"
	}
	if t == "bool" {
		return "bool"
	}
	return "::rosie::rt_control::" + t
}
func ordered(types []Type) ([]Type, error) {
	byName := map[string]Type{}
	for _, t := range types {
		byName[t.Name] = t
	}
	marks := map[string]int{}
	var out []Type
	var visit func(string) error
	visit = func(n string) error {
		if marks[n] == 2 {
			return nil
		}
		if marks[n] == 1 {
			return fmt.Errorf("recursive type %s", n)
		}
		t, ok := byName[n]
		if !ok {
			return nil
		}
		marks[n] = 1
		for _, f := range t.Fields {
			if err := visit(base(f.Type)); err != nil {
				return err
			}
		}
		marks[n] = 2
		out = append(out, t)
		return nil
	}
	for _, t := range types {
		if err := visit(t.Name); err != nil {
			return nil, err
		}
	}
	return out, nil
}
func generate(body []byte) (map[string][]byte, error) {
	s, err := parse(body)
	if err != nil {
		return nil, err
	}
	types, err := ordered(s.Types)
	if err != nil {
		return nil, err
	}
	digest := fmt.Sprintf("%x", sha256.Sum256(body))
	var g, c bytes.Buffer
	fmt.Fprint(&g, "// Code generated by tools/apigen; DO NOT EDIT.\n//go:build linux\n\npackage control\n\nimport (\"encoding/json\"; \"fmt\"; \"rosieos/rt-core/ipcclient\")\n\n")
	fmt.Fprintf(&g, "// Contract identity is unverified outside the tested application boundary.\nconst (ContractVersion = %d; RequestSchema = %q; ResponseSchema = %q; CapabilitiesDigest = %q)\n", s.ContractVersion, s.RequestSchema, s.ResponseSchema, digest)
	fmt.Fprint(&g, "type Operation string\n// Operation labels are unverified outside the tested application boundary.\nconst (\n")
	for _, v := range s.Capabilities {
		fmt.Fprintf(&g, "Operation%s = %q\n", identifier(v.Name), v.Name)
	}
	fmt.Fprint(&g, ")\n")
	fmt.Fprint(&g, "// Reason labels preserve the observed adapter errors; hardware behavior is unverified.\nconst (\n")
	for _, r := range s.Reasons {
		fmt.Fprintf(&g, "// %s\nReason%s = %q\n", r.Meaning, identifier(r.Name), r.Name)
	}
	fmt.Fprint(&g, ")\n")
	fmt.Fprint(&g, "type HandleState string\n// Handle labels are unverified outside the simulated adapter boundary.\nconst (\n")
	for _, v := range s.HandleStates {
		fmt.Fprintf(&g, "Handle%s HandleState = %q\n", identifier(v), v)
	}
	fmt.Fprint(&g, ")\n")
	for _, t := range types {
		if t.Existing {
			continue
		}
		if t.Alias != "" {
			fmt.Fprintf(&g, "type %s = %s\n", t.Name, t.Alias)
			continue
		}
		fmt.Fprintf(&g, "type %s struct {\n", t.Name)
		for _, f := range t.Fields {
			if f.Embedded {
				fmt.Fprintf(&g, "%s\n", goType(f.Type))
				continue
			}
			tag := f.JSON
			if f.OmitEmpty {
				tag += ",omitempty"
			}
			fmt.Fprintf(&g, "%s %s `json:%q`\n", f.Name, goType(f.Type), tag)
		}
		fmt.Fprint(&g, "}\n")
		if t.Binary {
			fmt.Fprintf(&g, "// MarshalJSON refuses binary transport values, preventing accidental base64 JSON publication.\nfunc (%s) MarshalJSON() ([]byte,error) { return nil, fmt.Errorf(\"binary transport required\") }\n", t.Name)
		}
	}
	fmt.Fprint(&g, "// Capability state labels are unverified outside the tested application boundary.\nconst (\n")
	for _, state := range s.CapabilityStates {
		fmt.Fprintf(&g, "CapabilityState%s = %q\n", identifier(state), state)
	}
	fmt.Fprint(&g, ")\n")
	fmt.Fprint(&g, "// Capabilities returns a fresh inventory, preventing caller mutation of the contract.\nfunc Capabilities() []CapabilityInfo { return []CapabilityInfo{\n")
	for _, v := range s.Capabilities {
		fmt.Fprintf(&g, "{%q,%q,%q},\n", v.Name, v.State, v.Transport)
	}
	fmt.Fprint(&g, "} }\n")
	fmt.Fprint(&g, "// Capability returns the exact named contract entry; unknown names have no permission.\nfunc Capability(name string) (CapabilityInfo, bool) { switch name {\n")
	for _, v := range s.Capabilities {
		fmt.Fprintf(&g, "case Operation%s: return CapabilityInfo{%q,%q,%q},true\n", identifier(v.Name), v.Name, v.State, v.Transport)
	}
	fmt.Fprint(&g, "};return CapabilityInfo{},false }\n")
	fmt.Fprint(&g, "// KnownOperation recognizes contract names without authorizing their transport or implementation.\nfunc KnownOperation(name string) bool { _,ok:=Capability(name);return ok }\n")
	fmt.Fprintln(&c, "// Code generated by tools/apigen; DO NOT EDIT.\n#pragma once\n"+cppRuntime)
	fmt.Fprintf(&c, "// Contract identity is unverified outside the tested application boundary.\ninline constexpr std::uint32_t ContractVersion = %d;\ninline constexpr const char* RequestSchema = %q;\ninline constexpr const char* ResponseSchema = %q;\ninline constexpr const char* CapabilitiesDigest = %q;\n", s.ContractVersion, s.RequestSchema, s.ResponseSchema, digest)
	fmt.Fprint(&c, "enum class Operation {\n")
	for _, v := range s.Capabilities {
		fmt.Fprintf(&c, "%s,\n", identifier(v.Name))
	}
	fmt.Fprint(&c, "};\n")
	fmt.Fprint(&c, "inline Json to_json(Operation value) { switch (value) {\n")
	for _, v := range s.Capabilities {
		fmt.Fprintf(&c, "case Operation::%s: return to_json(std::string(%q));\n", identifier(v.Name), v.Name)
	}
	fmt.Fprint(&c, "} throw std::runtime_error(\"unknown operation enum\"); }\ninline void from_json(const Json& json, Operation& value) { std::string name; from_json(json, name);\n")
	for _, v := range s.Capabilities {
		fmt.Fprintf(&c, "if (name == %q) { value = Operation::%s; return; }\n", v.Name, identifier(v.Name))
	}
	fmt.Fprint(&c, "throw std::runtime_error(\"unknown operation name\"); }\n")
	for _, r := range s.Reasons {
		fmt.Fprintf(&c, "// %s Hardware behavior is unverified.\ninline constexpr const char* Reason%s = %q;\n", r.Meaning, identifier(r.Name), r.Name)
	}
	for _, v := range s.HandleStates {
		fmt.Fprintf(&c, "// Handle label; hardware behavior is unverified.\ninline constexpr const char* Handle%s = %q;\n", identifier(v), v)
	}
	for _, state := range s.CapabilityStates {
		fmt.Fprintf(&c, "// Capability state label; hardware behavior is unverified.\ninline constexpr const char* CapabilityState%s = %q;\n", identifier(state), state)
	}
	for _, t := range types {
		fmt.Fprintf(&c, "struct %s {\n", t.Name)
		for _, f := range t.Fields {
			fmt.Fprintf(&c, "  %s %s{};\n", cppType(f.Type), f.Name)
		}
		fmt.Fprint(&c, "};\n")
		if t.Binary {
			continue
		}
		fmt.Fprintf(&c, "inline Json to_json(const %s& value) {\n  Json out = Json::object_value();\n", t.Name)
		for _, f := range t.Fields {
			if f.Embedded {
				fmt.Fprintf(&c, "  for (auto& field : to_json(value.%s).object) out.object.push_back(std::move(field));\n", f.Name)
			} else {
				if f.OmitEmpty {
					fmt.Fprintf(&c, "  if (!::rosie::rt_control::empty(value.%s))", f.Name)
				}
				fmt.Fprintf(&c, "  out.object.emplace_back(%q, to_json(value.%s));\n", f.JSON, f.Name)
			}
		}
		fmt.Fprint(&c, "  return out;\n}\n")
		fmt.Fprintf(&c, "inline void from_json(const Json& json, %s& value) {\n  require_object(json);\n  %s next{};\n", t.Name, t.Name)
		for _, f := range t.Fields {
			if f.Embedded {
				if !f.Required {
					for _, embedded := range types {
						if embedded.Name == f.Type {
							for _, member := range embedded.Fields {
								fmt.Fprintf(&c, "  field(json, %q, next.%s.%s, false);\n", member.JSON, f.Name, member.Name)
							}
						}
					}
				} else {
					fmt.Fprintf(&c, "  from_json(json, next.%s);\n", f.Name)
				}
			} else {
				fmt.Fprintf(&c, "  field(json, %q, next.%s, %t);\n", f.JSON, f.Name, f.Required)
			}
		}
		if t.Name == "Request" {
			var known []string
			for _, f := range t.Fields {
				if f.Embedded {
					for _, embedded := range types {
						if embedded.Name == f.Type {
							for _, member := range embedded.Fields {
								known = append(known, fmt.Sprintf("member.first == %q", member.JSON))
							}
						}
					}
				} else {
					known = append(known, fmt.Sprintf("member.first == %q", f.JSON))
				}
			}
			fmt.Fprintf(&c, "  for (const auto& member : json.object) if (!(%s)) throw std::runtime_error(\"unknown request field\");\n", strings.Join(known, " || "))
			fmt.Fprint(&c, "  for (const auto& member : json.object) if (member.first == \"request_id\" && member.second.kind == Json::Kind::Null) throw std::runtime_error(\"invalid_request_id\");\n")
			fmt.Fprint(&c, "  if (next.RequestID) { if (next.RequestID->empty() || next.RequestID->size() > 64) throw std::runtime_error(\"invalid_request_id\"); for (unsigned char ch : *next.RequestID) if (ch < 0x20 || ch > 0x7e) throw std::runtime_error(\"invalid_request_id\"); }\n")
		}
		fmt.Fprint(&c, "  value = std::move(next);\n}\n")
	}
	fmt.Fprint(&c, "// encode/decode preserve integer precision and reject malformed JSON without partial output mutation.\ntemplate<class T> std::string encode(const T& value) { return dump(to_json(value)); }\ntemplate<class T> bool decode(std::string_view wire, T& value, std::string* error = nullptr) { try { T next{}; from_json(Parser(wire).parse(), next); value = std::move(next); return true; } catch(const std::exception& e) { if (error) *error=e.what(); return false; } }\n")
	fmt.Fprint(&c, "} // namespace rosie::rt_control\n")
	goBytes, err := format.Source(g.Bytes())
	if err != nil {
		return nil, err
	}
	return map[string][]byte{"adapters/rosie/control/api_generated.go": goBytes, "clients/cpp/include/rosie/rt_control_api_generated.hpp": c.Bytes(), "clients/ts/rt_control_api_generated.ts": generateTypeScript(s, digest)}, nil
}

// writeOutputs checks every artifact without rewriting it in check mode.
func writeOutputs(root string, outputs map[string][]byte, check bool) error {
	names := make([]string, 0, len(outputs))
	for name := range outputs {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		path := filepath.Join(root, name)
		want := outputs[name]
		if check {
			got, err := os.ReadFile(path)
			if err != nil {
				return err
			}
			if !bytes.Equal(got, want) {
				return fmt.Errorf("generated API drift: %s", name)
			}
		} else {
			if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
				return err
			}
			if err := os.WriteFile(path, want, 0644); err != nil {
				return err
			}
		}
	}
	return nil
}
func main() {
	check := flag.Bool("check", false, "fail if generated outputs differ")
	flag.Parse()
	body, err := os.ReadFile("protocol/application-v1.schema.json")
	if err == nil {
		var outputs map[string][]byte
		outputs, err = generate(body)
		if err == nil {
			var doc []byte
			doc, err = os.ReadFile("docs/APPLICATION_API.md")
			if err == nil {
				outputs["docs/APPLICATION_API.md"], err = documentation(body, doc)
			}
		}
		if err == nil {
			err = writeOutputs(".", outputs, *check)
		}
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
