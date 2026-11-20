//go:build linux

package ipcclient

import (
	"fmt"
	"os/exec"
	"strings"
	"testing"
)

func TestHomeWriteProfileChecksDeclaredScalarWidth(t *testing.T) {
	f := compiledForValidation(t, "simulation-home.json")
	const original = "op|write_sdo|24700|0|i32|0"
	if !strings.Contains(f.profile, original) {
		t.Fatal("Home fixture lost its initial truth write")
	}
	for _, test := range []struct {
		typeName, value string
		accepted        bool
	}{
		{"u8", "-1", false}, {"u8", "256", false}, {"u8", "255", true},
		{"i8", "-129", false}, {"i8", "128", false}, {"i8", "-128", true},
		{"u16", "-1", false}, {"u16", "65536", false}, {"u16", "65535", true},
		{"i16", "-32769", false}, {"i16", "32768", false}, {"i16", "-32768", true},
		{"u32", "-1", false}, {"u32", "4294967296", false}, {"u32", "4294967295", true},
		{"i32", "-2147483649", false}, {"i32", "2147483648", false}, {"i32", "-2147483648", true},
	} {
		t.Run(test.typeName+"_"+test.value, func(t *testing.T) {
			profile := strings.Replace(f.profile, original,
				fmt.Sprintf("op|write_sdo|24700|0|%s|%s", test.typeName, test.value), 1)
			profile = editCoordinateField(t, profile, 0, "home.policy", "truth|24700|0|i32;sign|1;write_sdo|24700|0|"+test.typeName+"|"+test.value)
			output, err := f.validate(t, profile)
			if test.accepted {
				if err != nil {
					t.Fatalf("representable scalar rejected: %v: %s", err, output)
				}
			} else if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 {
				t.Fatalf("out-of-range Home scalar silently narrowed: %v: %s", err, output)
			}
		})
	}
}
