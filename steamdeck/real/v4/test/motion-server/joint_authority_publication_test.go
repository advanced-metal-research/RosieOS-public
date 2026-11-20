package motion_server

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// Wire-level acceptance is exercised on the bench. This guards the routing
// regression: partial joint diagnostics must not replace authority snapshots.
func TestJointOutputPublishesCompleteAuthority(t *testing.T) {

	raw, err := os.ReadFile(filepath.Join(findRepoRoot(t), "motion-server/v1/src/rt_core_cartesian_runtime.hpp"))
	if err != nil {
		t.Fatal(err)
	}
	source := string(raw)
	for _, required := range []string{`add("leader_lease_id", leader.leader_lease_id)`, `add("leader_fence_epoch", leader.leader_fence_epoch)`, `add("rt_core_status", backend.status())`, `add("axes", backend.status().Core.Axes)`, `add("v4_fields_available", false)`, `rt_core_cartesian_status(backend, live, authority.status(`, `publish_status_to_nats_async(options, body)`} {
		if !strings.Contains(source, required) {
			t.Fatalf("native authority publication missing %q", required)
		}
	}
	if strings.Contains(source, "fresh_output_pending") {
		t.Fatal("input activity gates native authority publication")
	}

}
