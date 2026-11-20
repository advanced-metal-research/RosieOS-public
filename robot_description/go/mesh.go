package robotdesc

import "strings"

// MeshResourcePath resolves only description-local package URIs. The package
// label may differ from the robot id; callers must still require the resulting
// path in the exact identity-pinned manifest. No package search or file/network
// access is performed here.
func MeshResourcePath(uri string) (string, bool) {
	if !strings.HasPrefix(uri, "package://") {
		return "", false
	}
	pkg, rel, found := strings.Cut(strings.TrimPrefix(uri, "package://"), "/")
	if !found || pkg == "" || !strings.HasPrefix(rel, "meshes/") {
		return "", false
	}
	for _, c := range pkg {
		if !(c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z' || c >= '0' && c <= '9' || c == '_') {
			return "", false
		}
	}
	if strings.ContainsAny(rel, "\\%?#\x00\r\n") {
		return "", false
	}
	for _, part := range strings.Split(rel, "/") {
		if part == "" || part == "." || part == ".." {
			return "", false
		}
	}
	return rel, true
}
