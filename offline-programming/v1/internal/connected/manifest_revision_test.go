package connected

import "testing"

func TestManifestRevisionBindsExactBytes(t *testing.T) {
	first, err := ManifestRevision([]byte(`{"name":"bench-relay"}`))
	if err != nil {
		t.Fatalf("manifest revision: %v", err)
	}
	same, _ := ManifestRevision([]byte(`{"name":"bench-relay"}`))
	changed, _ := ManifestRevision([]byte("{\"name\":\"bench-relay\"}\n"))
	if first != same || first == changed || !ValidManifestRevision(first) {
		t.Fatalf("manifest revision is not an exact stable digest: %q %q %q", first, same, changed)
	}
	for _, invalid := range []string{"", "sha256:", "sha256:not-hex", "md5:abcd"} {
		if ValidManifestRevision(invalid) {
			t.Fatalf("invalid manifest revision accepted: %q", invalid)
		}
	}
	if _, err := ManifestRevision(nil); err == nil {
		t.Fatal("empty manifest produced a revision")
	}
}
