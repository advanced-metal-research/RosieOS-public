package main

import (
	"encoding/json"
	"fmt"
	"os"
	"testing"
)

// Run only through the documented Go overlay: exercise canonical normalization
// without contacting a daemon or publishing any intent.
func TestReleaseResolveOnly(t *testing.T) {
	restore := withRobotV4ManifestPath(os.Getenv("RELEASE_REVIEW_MANIFEST"))
	defer restore()
	fixture, _, err := loadRobotV4ProcessIntentFixture(os.Getenv("RELEASE_REVIEW_FIXTURE"))
	if err != nil {
		t.Fatal(err)
	}
	intents, err := robotV4ProcessIntentsWithDeploymentManifest(fixture.Intents)
	if err != nil {
		t.Fatal(err)
	}
	raw, err := json.Marshal(intents)
	if err != nil {
		t.Fatal(err)
	}
	fmt.Println("RELEASE_RESOLVED=" + string(raw))
}
