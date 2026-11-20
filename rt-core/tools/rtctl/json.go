package rtctl

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"math/big"
	"sort"
	"strconv"
	"strings"
	"unicode/utf16"
)

type member struct {
	key   string
	value any
}

// object retains JSON insertion order so startup overrides do not reorder SDOs.
type object []member

func (o object) get(key string) any {
	for _, m := range o {
		if m.key == key {
			return m.value
		}
	}
	reject("missing field %s", key)
	return nil
}

func (o object) optional(key string, fallback any) any {
	for _, m := range o {
		if m.key == key {
			return m.value
		}
	}
	return fallback
}

func (o *object) set(key string, value any) {
	for i := range *o {
		if (*o)[i].key == key {
			(*o)[i].value = value
			return
		}
	}
	*o = append(*o, member{key, value})
}

type invalid struct{ message string }

func reject(format string, args ...any) { panic(invalid{fmt.Sprintf(format, args...)}) }

// validationBoundary converts only deliberate input rejections to errors; bugs still panic.
func validationBoundary(err *error) {
	if caught := recover(); caught != nil {
		if failure, ok := caught.(*RobotDefinitionError); ok {
			*err = failure
		} else if failure, ok := caught.(*BrakeOverrideError); ok {
			*err = failure
		} else if failure, ok := caught.(invalid); ok {
			*err = fmt.Errorf("%s", failure.message)
		} else {
			panic(caught)
		}
	}
}

func asObject(value any) object {
	result, ok := value.(object)
	if !ok {
		reject("expected a JSON object")
	}
	return result
}

func asArray(value any) []any {
	result, ok := value.([]any)
	if !ok {
		reject("expected a JSON array")
	}
	return result
}

func asString(value any) string {
	result, ok := value.(string)
	if !ok {
		reject("expected a string")
	}
	return result
}

func asBool(value any) bool {
	result, ok := value.(bool)
	if !ok {
		reject("expected a boolean")
	}
	return result
}

func decode(data []byte) any {
	d := json.NewDecoder(bytes.NewReader(data))
	d.UseNumber()
	var read func() any
	read = func() any {
		t, err := d.Token()
		if err != nil {
			reject("JSON: %v", err)
		}
		if delimiter, ok := t.(json.Delim); ok {
			switch delimiter {
			case '{':
				value := object{}
				for d.More() {
					key, err := d.Token()
					if err != nil {
						reject("JSON: %v", err)
					}
					value.set(asString(key), read())
				}
				if _, err := d.Token(); err != nil {
					reject("JSON: %v", err)
				}
				return value
			case '[':
				value := []any{}
				for d.More() {
					value = append(value, read())
				}
				if _, err := d.Token(); err != nil {
					reject("JSON: %v", err)
				}
				return value
			default:
				reject("unexpected JSON delimiter")
			}
		}
		return t
	}
	value := read()
	if _, err := d.Token(); err != io.EOF {
		reject("trailing JSON input")
	}
	return value
}

func integer(value any, minimum int64, maximum uint64, label string) string {
	n, ok := value.(json.Number)
	i, valid := new(big.Int).SetString(string(n), 10)
	if !ok || !valid || strings.ContainsAny(string(n), ".eE") ||
		i.Cmp(big.NewInt(minimum)) < 0 || i.Cmp(new(big.Int).SetUint64(maximum)) > 0 {
		reject("%s must be an integer in [%d, %d]", label, minimum, maximum)
	}
	return i.String()
}

func finite(value any, label string) float64 {
	n, ok := value.(json.Number)
	if !ok {
		reject("%s must be finite", label)
	}
	if !strings.ContainsAny(string(n), ".eE") {
		// JSON integer -0 becomes Python int(0) before conversion to float.
		i, valid := new(big.Int).SetString(string(n), 10)
		if !valid {
			reject("%s must be finite", label)
		}
		n = json.Number(i.String())
	}
	f, err := strconv.ParseFloat(string(n), 64)
	if err != nil || math.IsInf(f, 0) || math.IsNaN(f) {
		reject("%s must be finite", label)
	}
	return f
}

func token(value any) string {
	s := asString(value)
	if s == "" {
		reject("invalid empty identifier")
	}
	for _, c := range s {
		if !(c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z' || c >= '0' && c <= '9' || c == '_') {
			reject("invalid identifier: %q", s)
		}
	}
	return s
}

// pythonFloat emits Python's shortest binary64 representation, preventing hash
// and private-line differences at integral values and exponent transitions.
func pythonFloat(f float64) string {
	if math.IsNaN(f) || math.IsInf(f, 0) {
		reject("nonfinite JSON number")
	}
	e := strconv.FormatFloat(f, 'e', -1, 64)
	pos := strings.LastIndexByte(e, 'e')
	exponent, _ := strconv.Atoi(e[pos+1:])
	// Unverified compatibility thresholds from Python repr; these are decimal exponents.
	if exponent >= -4 && exponent < 16 {
		s := strconv.FormatFloat(f, 'f', -1, 64)
		if !strings.ContainsRune(s, '.') {
			s += ".0"
		}
		return s
	}
	return e
}

// asciiQuote quotes with ASCII-only escapes and no HTML escaping, matching the
// retired compiler's output that the parity goldens pin; identity bytes are preserved.
func asciiQuote(s string) string {
	var b strings.Builder
	b.WriteByte('"')
	for _, r := range s {
		switch r {
		case '"', '\\':
			b.WriteByte('\\')
			b.WriteRune(r)
		case '\b':
			b.WriteString(`\b`)
		case '\f':
			b.WriteString(`\f`)
		case '\n':
			b.WriteString(`\n`)
		case '\r':
			b.WriteString(`\r`)
		case '\t':
			b.WriteString(`\t`)
		default:
			if r < 0x20 || r >= 0x7f {
				if r <= 0xffff {
					fmt.Fprintf(&b, `\u%04x`, r)
				} else {
					hi, lo := utf16.EncodeRune(r)
					fmt.Fprintf(&b, `\u%04x\u%04x`, hi, lo)
				}
			} else {
				b.WriteRune(r)
			}
		}
	}
	b.WriteByte('"')
	return b.String()
}

func numberString(n json.Number) string {
	if strings.ContainsAny(string(n), ".eE") {
		return pythonFloat(finite(n, "JSON number"))
	}
	i, ok := new(big.Int).SetString(string(n), 10)
	if !ok {
		reject("invalid JSON integer")
	}
	return i.String()
}

// canonical includes unknown metadata and sorted object keys; dropping either changes the SHA.
func canonical(value any) string {
	switch v := value.(type) {
	case nil:
		return "null"
	case bool:
		return strconv.FormatBool(v)
	case string:
		return asciiQuote(v)
	case json.Number:
		return numberString(v)
	case []any:
		parts := make([]string, len(v))
		for i, item := range v {
			parts[i] = canonical(item)
		}
		return "[" + strings.Join(parts, ",") + "]"
	case object:
		ordered := append(object(nil), v...)
		sort.Slice(ordered, func(i, j int) bool { return ordered[i].key < ordered[j].key })
		parts := make([]string, len(ordered))
		for i, m := range ordered {
			parts[i] = asciiQuote(m.key) + ":" + canonical(m.value)
		}
		return "{" + strings.Join(parts, ",") + "}"
	default:
		reject("unsupported JSON value %T", value)
	}
	return ""
}

func numericEqual(value any, want int) bool {
	if b, ok := value.(bool); ok {
		return b && want == 1 || !b && want == 0
	}
	n, ok := value.(json.Number)
	if !ok {
		return false
	}
	f, err := strconv.ParseFloat(string(n), 64)
	return err == nil && f == float64(want)
}

func truth(value any) bool {
	switch v := value.(type) {
	case nil:
		return false
	case bool:
		return v
	case string:
		return v != ""
	case object:
		return len(v) != 0
	case []any:
		return len(v) != 0
	case json.Number:
		return !numericEqual(v, 0)
	default:
		reject("unsupported truth value")
	}
	return false
}
