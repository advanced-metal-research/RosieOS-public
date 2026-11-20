package rtctl

import (
	"encoding/hex"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

var coordinateIdentityKeys = []string{
	"anchor.source", "direction.sign", "drive.product_code", "drive.revision_no",
	"drive.vendor_id", "encoder.counts_per_motor_rev", "evidence.mode",
	"gear.denominator", "gear.numerator", "home.policy", "tracking.mode",
	"wire.counts_per_rev", "wire.revs_per_axis_rev",
}

func linearA6IdentityFixture(t *testing.T) *fixture {
	t.Helper()
	f := fresh()
	f.put("profile", string(readTestFile(t, "config/drives/a6ec_bench_1to1.json")))
	f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
	f.put("machine.axes.0.wire_counts_per_rev", "131072")
	f.put("machine.axes.0.wire_revs_per_axis_rev", "1")
	f.put("machine.axes.0.coordinate_evidence_mode", `"multi_turn"`)
	return f
}

func TestCoordinateIdentityAnchorSourceVersion(t *testing.T) {
	generic, err := fresh().compile()
	if err != nil {
		t.Fatal(err)
	}
	if generic.CoordinateIdentity[0] != "anchor.source=none|0\n"+
		"direction.sign=1\ndrive.product_code=1\ndrive.revision_no=0\ndrive.vendor_id=1\n"+
		"encoder.counts_per_motor_rev=1000\nevidence.mode=multi_turn\ngear.denominator=1\n"+
		"gear.numerator=1\nhome.policy=none\ntracking.mode=continuous\n"+
		"wire.counts_per_rev=1000\nwire.revs_per_axis_rev=1\n" ||
		generic.CoordinateIdentitySHA256[0] != "4856cdd9c8f30c6b63944406bd2b1b515e98fe9933a333ac811366f7d6c7a674" {
		t.Fatal("generic continuous identity changed", generic.CoordinateIdentity)
	}

	linear, err := linearA6IdentityFixture(t).compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.HasPrefix(linear.CoordinateIdentity[0], "anchor.source=raw_multi_turn_encoder|3\n") ||
		!strings.Contains(linear.CoordinateIdentity[0], "tracking.mode=1\n") {
		t.Fatal(linear.CoordinateIdentity[0])
	}
	changed := linearA6IdentityFixture(t)
	changed.put("profile.coordinate_evidence.anchor_source.version", "4")
	if _, err := changed.compile(); err == nil || !strings.Contains(err.Error(), "anchor source version must be 3") {
		t.Fatalf("non-v3 A6 source accepted: %v", err)
	}
}

func TestCoordinateIdentityExistingConfigurationsWithoutPair(t *testing.T) {
	var files []string
	for _, pattern := range []string{
		filepath.Join(testRoot(t), "config/machines", "*.json"),
		filepath.Join(testRoot(t), "config/machines/bench", "*.json"),
		filepath.Join(testRoot(t), "config/machines/simulation", "*.json"),
	} {
		matches, err := filepath.Glob(pattern)
		if err != nil {
			t.Fatal(err)
		}
		files = append(files, matches...)
	}
	for _, name := range files {
		t.Run(name, func(t *testing.T) {
			raw, err := os.ReadFile(name)
			if err != nil {
				t.Fatal(err)
			}
			compiled, err := Compile(raw, filepath.Join(testRoot(t), "config/drives"))
			if err != nil {
				t.Fatal(err)
			}
			if len(compiled.CoordinateIdentity) == 0 ||
				len(compiled.CoordinateIdentity) != len(compiled.CoordinateIdentitySHA256) {
				t.Fatal("missing coordinate identities")
			}
			for axis, identity := range compiled.CoordinateIdentity {
				if digest, err := hex.DecodeString(compiled.CoordinateIdentitySHA256[axis]); err != nil ||
					len(digest) != 32 || compiled.CoordinateIdentitySHA256[axis] != identityDigest(identity) {
					t.Fatal("invalid coordinate digest")
				}
				var keys []string
				for _, line := range strings.Split(strings.TrimSuffix(identity, "\n"), "\n") {
					key, value, ok := strings.Cut(line, "=")
					if !ok || value == "" {
						t.Fatal(line)
					}
					keys = append(keys, key)
				}
				if !reflect.DeepEqual(keys, coordinateIdentityKeys) {
					t.Fatalf("axis %d keys: %v", axis, keys)
				}
			}
		})
	}
}

func TestCoordinateIdentityIgnoresLimitsAndPairMetadata(t *testing.T) {
	before, err := linearA6IdentityFixture(t).compile()
	if err != nil {
		t.Fatal(err)
	}
	afterFixture := linearA6IdentityFixture(t)
	afterFixture.put("machine.axes.0.limits.max", "0.75")
	afterFixture.put("machine.pair", `{"id":"other","revision":9}`)
	after, err := afterFixture.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(before.CoordinateIdentity, after.CoordinateIdentity) ||
		!reflect.DeepEqual(before.CoordinateIdentitySHA256, after.CoordinateIdentitySHA256) ||
		before.MachineSHA256 == after.MachineSHA256 {
		t.Fatal("limits/pair identity partition changed")
	}
}
