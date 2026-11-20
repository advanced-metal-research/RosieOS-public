// Package fixturedelta expands a rosie.fixture-delta.v1 file back into the
// full JSON array that test consumers read.
//
// A fixture oracle that pins every field of a wire record for every case is
// mostly repeated defaults. The delta file keeps one template (key order and
// default per field) and, per case, only the values that differ. Expansion is
// the only way to read it: consumers in Go, C++ and TypeScript receive the
// expanded array, so the strict wire-shape parsers keep seeing every field and
// the oracle itself stays a checked-in, non-generated file.
//
// Template nodes:
//
//	object: {"keys": [...], "object": {key: node}, "nullDefault"?: true}
//	        absent key -> null when nullDefault, omitted when the child is
//	        optional, otherwise the all-defaults object
//	array:  {"array": itemNode, "default": <full default array>}
//	        absent -> default array; null -> null; list -> each element is
//	        expanded as a present delta against itemNode
//	scalar: {"default": value}
//	        absent -> default; present -> the delta value verbatim
//	optional child: node carries "optional": true
//
// Scalars are copied as raw JSON so number formatting is preserved.
package fixturedelta

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
)

// Schema is the only delta format this package expands.
const Schema = "rosie.fixture-delta.v1"

type node struct {
	Optional    bool                       `json:"optional"`
	Keys        []string                   `json:"keys"`
	Object      map[string]json.RawMessage `json:"object"`
	NullDefault bool                       `json:"nullDefault"`
	Array       json.RawMessage            `json:"array"`
	Default     json.RawMessage            `json:"default"`
	children    map[string]*node
	item        *node
}

func parseNode(raw json.RawMessage) (*node, error) {
	n := &node{}
	if err := json.Unmarshal(raw, n); err != nil {
		return nil, err
	}
	if n.Object != nil {
		n.children = make(map[string]*node, len(n.Object))
		for _, key := range n.Keys {
			child, ok := n.Object[key]
			if !ok {
				return nil, fmt.Errorf("template key %q listed without a node", key)
			}
			parsed, err := parseNode(child)
			if err != nil {
				return nil, fmt.Errorf("template key %q: %w", key, err)
			}
			n.children[key] = parsed
		}
		if len(n.children) != len(n.Object) {
			return nil, errors.New("template object has nodes missing from keys")
		}
	}
	if len(n.Array) != 0 {
		item, err := parseNode(n.Array)
		if err != nil {
			return nil, fmt.Errorf("template array item: %w", err)
		}
		n.item = item
	}
	return n, nil
}

func (n *node) isObject() bool { return n.Object != nil }
func (n *node) isArray() bool  { return len(n.Array) != 0 }

// Expand returns the expanded JSON array for a delta file's bytes.
func Expand(compact []byte) ([]byte, error) {
	var file struct {
		Schema   string            `json:"schema"`
		Template json.RawMessage   `json:"template"`
		Cases    []json.RawMessage `json:"cases"`
	}
	if err := json.Unmarshal(compact, &file); err != nil {
		return nil, fmt.Errorf("fixture delta: %w", err)
	}
	if file.Schema != Schema {
		return nil, fmt.Errorf("fixture delta schema %q, want %q", file.Schema, Schema)
	}
	root, err := parseNode(file.Template)
	if err != nil {
		return nil, fmt.Errorf("fixture delta template: %w", err)
	}
	if !root.isObject() {
		return nil, errors.New("fixture delta template root must be an object node")
	}
	var out bytes.Buffer
	out.WriteString("[\n")
	for i, c := range file.Cases {
		out.WriteString("  ")
		if _, err := expand(&out, root, c, true, 1); err != nil {
			return nil, fmt.Errorf("fixture delta case %d: %w", i, err)
		}
		if i+1 < len(file.Cases) {
			out.WriteString(",")
		}
		out.WriteString("\n")
	}
	out.WriteString("]\n")
	return out.Bytes(), nil
}

// ExpandFile expands the delta file at path.
func ExpandFile(path string) ([]byte, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	return Expand(raw)
}

// WriteExpanded expands the delta file at path into dir and returns the
// written file, for consumers that take a fixture path on their command line.
func WriteExpanded(path, dir string) (string, error) {
	expanded, err := ExpandFile(path)
	if err != nil {
		return "", err
	}
	name := strings.TrimSuffix(filepath.Base(path), ".delta.json") + ".json"
	target := filepath.Join(dir, name)
	if err := os.WriteFile(target, expanded, 0o644); err != nil {
		return "", err
	}
	return target, nil
}

func isNull(raw json.RawMessage) bool { return bytes.Equal(bytes.TrimSpace(raw), []byte("null")) }

// expand writes the value for n given its delta; it returns false when the
// key must be omitted (an optional child with no delta).
func expand(out *bytes.Buffer, n *node, delta json.RawMessage, present bool, depth int) (bool, error) {
	if n.Optional && !present {
		return false, nil
	}
	switch {
	case n.isObject():
		if !present {
			if n.NullDefault {
				out.WriteString("null")
				return true, nil
			}
			return writeObject(out, n, nil, depth)
		}
		if isNull(delta) {
			out.WriteString("null")
			return true, nil
		}
		var fields map[string]json.RawMessage
		if err := json.Unmarshal(delta, &fields); err != nil {
			return false, fmt.Errorf("object delta: %w", err)
		}
		for key := range fields {
			if _, ok := n.children[key]; !ok {
				return false, fmt.Errorf("delta key %q is not in the template", key)
			}
		}
		return writeObject(out, n, fields, depth)
	case n.isArray():
		if !present {
			out.Write(bytes.TrimSpace(n.Default))
			return true, nil
		}
		if isNull(delta) {
			out.WriteString("null")
			return true, nil
		}
		var elements []json.RawMessage
		if err := json.Unmarshal(delta, &elements); err != nil {
			return false, fmt.Errorf("array delta: %w", err)
		}
		if len(elements) == 0 {
			out.WriteString("[]")
			return true, nil
		}
		out.WriteString("[\n")
		for i, e := range elements {
			indent(out, depth+1)
			if _, err := expand(out, n.item, e, true, depth+1); err != nil {
				return false, fmt.Errorf("element %d: %w", i, err)
			}
			if i+1 < len(elements) {
				out.WriteString(",")
			}
			out.WriteString("\n")
		}
		indent(out, depth)
		out.WriteString("]")
		return true, nil
	default:
		if !present {
			out.Write(bytes.TrimSpace(n.Default))
			return true, nil
		}
		out.Write(bytes.TrimSpace(delta))
		return true, nil
	}
}

func writeObject(out *bytes.Buffer, n *node, fields map[string]json.RawMessage, depth int) (bool, error) {
	var body bytes.Buffer
	written := 0
	for _, key := range n.Keys {
		delta, present := fields[key]
		var value bytes.Buffer
		emitted, err := expand(&value, n.children[key], delta, present, depth+1)
		if err != nil {
			return false, fmt.Errorf("%q: %w", key, err)
		}
		if !emitted {
			continue
		}
		if written > 0 {
			body.WriteString(",\n")
		}
		indent(&body, depth+1)
		quoted, err := json.Marshal(key)
		if err != nil {
			return false, err
		}
		body.Write(quoted)
		body.WriteString(": ")
		body.Write(value.Bytes())
		written++
	}
	if written == 0 {
		out.WriteString("{}")
		return true, nil
	}
	out.WriteString("{\n")
	out.Write(body.Bytes())
	out.WriteString("\n")
	indent(out, depth)
	out.WriteString("}")
	return true, nil
}

func indent(out *bytes.Buffer, depth int) {
	for i := 0; i < depth; i++ {
		out.WriteString("  ")
	}
}
