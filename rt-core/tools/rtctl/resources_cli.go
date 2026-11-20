//go:build linux

package rtctl

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"

	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/sdk/control"
)

// fetchResources requires a member digest of the current robot resource set.
// The manifest retains URI mappings; digest filenames prevent remote resource
// names from escaping --out. Exclusive publication never overwrites local data.
func fetchResources(ctx context.Context, client *control.Client, sha, output string) (string, error) {
	if output == "" || !adapter.ValidResourceDigest(sha) {
		return "", fmt.Errorf("resources fetch requires a SHA-256 and --out directory")
	}
	description, err := client.Describe(ctx)
	if err != nil {
		return "", err
	}
	robot := description.Robot
	if robot == nil {
		return "", &control.Rejected{Reason: adapter.ReasonResourceUnknown}
	}
	found := false
	for _, resource := range robot.Resources {
		found = found || resource.SHA256 == sha
	}
	if !found {
		return "", &control.Rejected{Reason: adapter.ReasonResourceUnknown}
	}
	if len(robot.Resources) > adapter.MaxResourceEntries {
		return "", control.ErrProtocol
	}
	if err := os.MkdirAll(output, 0755); err != nil {
		return "", err
	}
	stage, err := os.MkdirTemp(output, ".resources-")
	if err != nil {
		return "", err
	}
	defer os.RemoveAll(stage)
	seen := map[string]bool{}
	var total uint64
	for _, resource := range robot.Resources {
		if !adapter.ValidResourceDigest(resource.SHA256) || resource.Bytes > adapter.MaxResourceBytes {
			return "", control.ErrProtocol
		}
		if seen[resource.SHA256] {
			continue
		}
		seen[resource.SHA256] = true
		total += resource.Bytes
		if total > adapter.MaxResourceSetBytes {
			return "", control.ErrProtocol
		}
		reader, err := client.Resource(ctx, resource.SHA256)
		if err != nil {
			return "", err
		}
		body, err := io.ReadAll(reader)
		reader.Close()
		if err != nil {
			return "", err
		}
		if uint64(len(body)) != resource.Bytes {
			return "", control.ErrProtocol
		}
		if err := os.WriteFile(filepath.Join(stage, resource.SHA256), body, 0644); err != nil {
			return "", err
		}
	}
	manifest, err := json.MarshalIndent(robot, "", "  ")
	if err != nil {
		return "", err
	}
	if err := os.WriteFile(filepath.Join(stage, "resources.json"), append(manifest, '\n'), 0644); err != nil {
		return "", err
	}
	// Linking staged files is exclusive and cannot follow an existing symlink.
	// Publish the manifest last so incomplete sets never appear complete.
	for digest := range seen {
		if err := os.Link(filepath.Join(stage, digest), filepath.Join(output, digest)); err != nil {
			return "", err
		}
	}
	if err := os.Link(filepath.Join(stage, "resources.json"), filepath.Join(output, "resources.json")); err != nil {
		return "", err
	}
	return filepath.Join(output, "resources.json"), nil
}
