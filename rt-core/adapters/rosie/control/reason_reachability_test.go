//go:build linux

package control

import (
	"encoding/json"
	"os"
	"testing"

	"rosieos/rt-core/ipcclient"
)

// Storage failure must reach the public HTTP error body without submitting a
// native restoration; the inventory mutation test pins this same sentinel.
func TestAnchorIOPublicReachability(t *testing.T) {
	c, native, fence := restoreObservationFixture(t, func(int) ipcclient.Status {
		t.Fatal("unexpected observation")
		return ipcclient.Status{}
	})
	path := c.anchorStore.Path(0)
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(path, 0700); err != nil {
		t.Fatal(err)
	}
	sequence, recovery, err := c.RestoreAnchor(fence, 1)
	if sequence != 0 || recovery != nil || native.restores != 0 || err == nil {
		t.Fatalf("restore: sequence=%d recovery=%+v submissions=%d error=%v", sequence, recovery, native.restores, err)
	}
	outcome := encodeResponse(Response{Schema: ResponseSchema, Operation: OperationRestoreAnchor}, err)
	var response Response
	if err := json.Unmarshal(outcome.body, &response); err != nil {
		t.Fatal(err)
	}
	if outcome.status != 409 || response.Error != "anchor_store_io" {
		t.Fatalf("HTTP %d: %s", outcome.status, outcome.body)
	}
}
