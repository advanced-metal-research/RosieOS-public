package connected

import (
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"strings"
)

const ManifestRevisionPrefix = "sha256:"

// ManifestRevision returns an exact digest of the manifest bytes that were
// parsed into a connected-target binding. It is intentionally not derived from
// a path or profile name: a same-name manifest edit is a different execution
// target revision.
func ManifestRevision(raw []byte) (string, error) {
	if len(raw) == 0 {
		return "", errors.New("connected: manifest bytes are required")
	}
	sum := sha256.Sum256(raw)
	return ManifestRevisionPrefix + hex.EncodeToString(sum[:]), nil
}

func ValidManifestRevision(value string) bool {
	if !strings.HasPrefix(value, ManifestRevisionPrefix) {
		return false
	}
	digest := strings.TrimPrefix(value, ManifestRevisionPrefix)
	if len(digest) != sha256.Size*2 {
		return false
	}
	_, err := hex.DecodeString(digest)
	return err == nil
}
