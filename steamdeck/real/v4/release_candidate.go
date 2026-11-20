package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strings"
	"time"
)

const releaseCandidateRepository = "advanced-metal-research/RosieOS"

var releaseCandidateIDPattern = regexp.MustCompile(`^sha256:[0-9a-f]{64}$`)

// releaseCandidate is the verified subset the managed release transaction
// needs. The Python candidate module remains the one full bundle verifier.
type releaseCandidate struct {
	ID                    string
	Commit                string
	TargetProfile         string
	Package               string
	ArtifactSHA256        string
	PackageManifestSHA256 string
}

type releaseCandidateManifest struct {
	Schema      string `json:"schema"`
	CandidateID string `json:"candidate_id"`
	Identity    struct {
		Component string `json:"component"`
		Source    struct {
			Repository string `json:"repository"`
			Commit     string `json:"commit"`
		} `json:"source"`
		Target struct {
			Profile      string `json:"profile"`
			OS           string `json:"os"`
			Architecture string `json:"architecture"`
		} `json:"target"`
		Artifact struct {
			SHA256                string `json:"sha256"`
			Bytes                 int64  `json:"bytes"`
			MediaType             string `json:"media_type"`
			PackageManifestSHA256 string `json:"package_manifest_sha256"`
		} `json:"artifact"`
	} `json:"identity"`
	Publication struct {
		Assets struct {
			Artifact struct {
				Name string `json:"name"`
			} `json:"artifact"`
		} `json:"assets"`
	} `json:"publication"`
}

// loadReleaseCandidate verifies a complete local bundle before any host is
// contacted. It then binds the selected ID to the exact package path.
func loadReleaseCandidate(repo, candidatePath, expectedID string) (releaseCandidate, error) {
	if !releaseCandidateIDPattern.MatchString(expectedID) {
		return releaseCandidate{}, fmt.Errorf("--candidate-id must be sha256 followed by 64 lowercase hex characters")
	}
	root, err := filepath.Abs(candidatePath)
	if err != nil {
		return releaseCandidate{}, fmt.Errorf("resolve candidate path: %w", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Minute)
	defer cancel()
	command := exec.CommandContext(ctx, "python3", "-m", "rosie_build1_candidate", "verify", "--candidate", root)
	command.Env = append(os.Environ(), "PYTHONPATH="+filepath.Join(repo, "tools", "build1-release-worker"))
	raw, err := command.CombinedOutput()
	if ctx.Err() != nil {
		return releaseCandidate{}, fmt.Errorf("candidate verification exceeded 2 minutes")
	}
	if err != nil {
		return releaseCandidate{}, fmt.Errorf("candidate verification failed: %s", strings.TrimSpace(string(raw)))
	}
	return releaseCandidateFromManifest(root, expectedID, raw)
}

// releaseCandidateFromManifest checks the fields that cross into deployment.
// The complete manifest and evidence inventory were already verified above.
func releaseCandidateFromManifest(root, expectedID string, raw []byte) (releaseCandidate, error) {
	var manifest releaseCandidateManifest
	if err := json.Unmarshal(raw, &manifest); err != nil {
		return releaseCandidate{}, fmt.Errorf("decode verified candidate manifest: %w", err)
	}
	identity := manifest.Identity
	switch {
	case manifest.Schema != "rosie.release-candidate.v1":
		return releaseCandidate{}, fmt.Errorf("candidate schema differs")
	case manifest.CandidateID != expectedID:
		return releaseCandidate{}, fmt.Errorf("candidate ID differs from --candidate-id")
	case identity.Component != "rt-core":
		return releaseCandidate{}, fmt.Errorf("candidate component is %q, want rt-core", identity.Component)
	case identity.Source.Repository != releaseCandidateRepository:
		return releaseCandidate{}, fmt.Errorf("candidate repository is %q, want %s", identity.Source.Repository, releaseCandidateRepository)
	case !cellDeployHexCommit.MatchString(identity.Source.Commit):
		return releaseCandidate{}, fmt.Errorf("candidate source commit is invalid")
	case identity.Target.OS != "linux" || identity.Target.Architecture != "aarch64":
		return releaseCandidate{}, fmt.Errorf("candidate target is %s/%s, want linux/aarch64", identity.Target.OS, identity.Target.Architecture)
	case identity.Artifact.MediaType != "application/gzip" || identity.Artifact.Bytes <= 0:
		return releaseCandidate{}, fmt.Errorf("candidate artifact contract differs")
	case !rtCoreHexSHA256.MatchString(identity.Artifact.SHA256):
		return releaseCandidate{}, fmt.Errorf("candidate artifact sha256 is invalid")
	case !rtCoreHexSHA256.MatchString(identity.Artifact.PackageManifestSHA256):
		return releaseCandidate{}, fmt.Errorf("candidate package manifest sha256 is invalid")
	}
	name := manifest.Publication.Assets.Artifact.Name
	if name == "" || filepath.Base(name) != name {
		return releaseCandidate{}, fmt.Errorf("candidate artifact name is invalid")
	}
	packagePath := filepath.Join(root, name)
	info, err := os.Lstat(packagePath)
	if err != nil {
		return releaseCandidate{}, fmt.Errorf("candidate artifact: %w", err)
	}
	if !info.Mode().IsRegular() || info.Size() != identity.Artifact.Bytes {
		return releaseCandidate{}, fmt.Errorf("candidate artifact file differs")
	}
	return releaseCandidate{
		ID:                    manifest.CandidateID,
		Commit:                identity.Source.Commit,
		TargetProfile:         identity.Target.Profile,
		Package:               packagePath,
		ArtifactSHA256:        identity.Artifact.SHA256,
		PackageManifestSHA256: identity.Artifact.PackageManifestSHA256,
	}, nil
}

func (candidate releaseCandidate) validateTarget(ctx cellDeployContext) error {
	want := strings.ToLower(ctx.RelayPeer + "-" + ctx.ServoPeer)
	if candidate.TargetProfile != want {
		return fmt.Errorf("candidate target profile is %q, want %q for this cell", candidate.TargetProfile, want)
	}
	return nil
}
