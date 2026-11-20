package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"sort"
	"strings"
)

// documentation replaces every schema block from canonical inputs while keeping
// authored prose outside the blocks, preventing an incomplete published catalogue.
func documentation(schema, document []byte) ([]byte, error) {
	s, err := parse(schema)
	if err != nil {
		return nil, err
	}
	escape := func(text string) string {
		return strings.NewReplacer("|", "\\|", "\n", "<br>").Replace(text)
	}
	var capabilities, types, reasons, rules bytes.Buffer
	fmt.Fprintln(&capabilities, "| Operation | State | Transport | Request type; semantic required fields | Schema result |\n|---|---|---|---|---|")
	for _, c := range s.Capabilities {
		fields := strings.Join(c.RequiredFields, ", ")
		if fields == "" {
			fields = "none"
		}
		fmt.Fprintf(&capabilities, "| `%s` | %s | %s | %s; %s | %s |\n", c.Name, c.State, escape(c.Transport), c.RequestType, fields, c.ResponseType)
	}
	for i, typ := range s.Types {
		if i != 0 {
			fmt.Fprintln(&types)
		}
		fmt.Fprintf(&types, "### %s\n\n| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |\n|---|---|---|---|---|\n", typ.Name)
		for _, f := range typ.Fields {
			name := f.JSON
			if f.Embedded {
				name = "flattened " + f.Type
			}
			fmt.Fprintf(&types, "| `%s` | %s / %s | %t | %t | %s |\n", name, f.JSONType, escape(f.Type), f.Required, f.OmitEmpty, escape(f.Constraints))
		}
	}
	fmt.Fprintln(&reasons, "| Literal reason | Schema meaning |\n|---|---|")
	for _, r := range s.Reasons {
		fmt.Fprintf(&reasons, "| %s | %s |\n", escape(r.Name), escape(r.Meaning))
	}
	fmt.Fprintln(&rules, "| Rule | Contract |\n|---|---|")
	var ruleValues map[string]any
	decoder := json.NewDecoder(bytes.NewReader(s.Rules))
	decoder.UseNumber()
	if err := decoder.Decode(&ruleValues); err != nil {
		return nil, err
	}
	var renderRules func(string, map[string]any)
	renderRules = func(prefix string, values map[string]any) {
		keys := make([]string, 0, len(values))
		for key := range values {
			keys = append(keys, key)
		}
		sort.Strings(keys)
		for _, key := range keys {
			value := values[key]
			if nested, ok := value.(map[string]any); ok {
				renderRules(prefix+key+".", nested)
				continue
			}
			text, ok := value.(string)
			if !ok {
				var body bytes.Buffer
				encoder := json.NewEncoder(&body)
				encoder.SetEscapeHTML(false)
				_ = encoder.Encode(value)
				text = strings.TrimSuffix(body.String(), "\n")
			}
			fmt.Fprintf(&rules, "| %s | %s |\n", prefix+key, escape(text))
		}
	}
	renderRules("", ruleValues)
	result := string(document)
	for _, block := range []struct{ name, text string }{
		{"CAPABILITIES", capabilities.String()}, {"RULES", rules.String()}, {"TYPES", types.String()}, {"REASONS", reasons.String()},
	} {
		begin, end := "<!-- BEGIN SCHEMA "+block.name+" -->", "<!-- END SCHEMA "+block.name+" -->"
		if strings.Count(result, begin) != 1 || strings.Count(result, end) != 1 {
			return nil, fmt.Errorf("missing or duplicate schema documentation block: %s", block.name)
		}
		first, last := strings.Index(result, begin)+len(begin), strings.Index(result, end)
		if first > last {
			return nil, fmt.Errorf("reversed schema documentation block: %s", block.name)
		}
		result = result[:first] + "\n" + block.text + result[last:]
	}
	return []byte(result), nil
}
