package denseexec

import (
	"context"
	"encoding/base64"
	"net/http"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
)

func keptFixture(t *testing.T) ([]byte, Identity) {
	t.Helper()
	blob := testProgramBlob(t, 0x3f, [codec.AxisCount]float64{}, testProgramRobot(), nil)
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	h := plan.Header
	return blob, Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, ManifestRevision: h.ManifestRevision, PlanRevision: h.PlanRevision}
}

// A planner that no longer holds the digest is answered by the client's kept
// copy, only when that copy is exactly the plan the Load names; the planner's
// own copy wins whenever it answers.
func TestLoadFallsBackToTheClientsKeptTrajectory(t *testing.T) {
	blob, identity := keptFixture(t)
	missing := func(context.Context, string) ([]byte, error) {
		return nil, httpError(http.StatusNotFound, "dense_blob_not_found", "the weld planner motion server has no dense trajectory")
	}
	kept := base64.StdEncoding.EncodeToString(blob)

	source := &blobSource{cfg: Config{FetchBlob: missing}}
	got, program, note, err := source.loadBlob(context.Background(), LoadRequest{Identity: identity, DenseBlobBase64: kept})
	if err != nil || string(got) != string(blob) || program != nil || !strings.Contains(note, "dense_blob_not_found") || !strings.Contains(note, "kept") {
		t.Fatalf("kept fallback: %v note=%q", err, note)
	}

	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: identity}); errorCode(err) != "dense_blob_not_found" {
		t.Fatalf("no kept copy must keep the planner's refusal: %v", err)
	}

	planner := []byte("planner bytes")
	answering := &blobSource{cfg: Config{FetchBlob: func(context.Context, string) ([]byte, error) { return planner, nil }}}
	got, _, note, err = answering.loadBlob(context.Background(), LoadRequest{Identity: identity, DenseBlobBase64: kept})
	if err != nil || string(got) != string(planner) || note != "" {
		t.Fatalf("the planner's copy must win: %q %v %q", got, err, note)
	}

	other := identity
	other.PlanRevision++
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: other, DenseBlobBase64: kept}); errorCode(err) != "dense_identity_mismatch" {
		t.Fatalf("another plan's identity: %v", err)
	}
	other = identity
	other.TrajectoryDigest = "sha256:" + strings.Repeat("e", 64)
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: other, DenseBlobBase64: kept}); errorCode(err) != "dense_identity_mismatch" {
		t.Fatalf("another digest: %v", err)
	}

	// One flipped bit in the last sample: the recomputed digest no longer
	// matches the header's, so the copy is not the plan it claims to be.
	tampered := append([]byte(nil), blob...)
	tampered[len(tampered)-1] ^= 1
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: identity, DenseBlobBase64: base64.StdEncoding.EncodeToString(tampered)}); errorCode(err) != "dense_trajectory_invalid" {
		t.Fatalf("tampered copy: %v", err)
	}

	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: identity, DenseBlobBase64: "not base64!"}); errorCode(err) != "dense_blob_invalid" {
		t.Fatalf("not base64: %v", err)
	}

	// Larger than the planner fetch's own 64 MiB cap: refused before decoding.
	oversize := strings.Repeat("A", base64.StdEncoding.EncodedLen(maxBlobBytes+3))
	if _, _, _, err := source.loadBlob(context.Background(), LoadRequest{Identity: identity, DenseBlobBase64: oversize}); errorCode(err) != "dense_blob_too_large" {
		t.Fatalf("oversize copy: %v", err)
	}
}
