package robotdesc

import "testing"

func TestMeshResourcePath(t *testing.T) {
	for _, uri := range []string{
		"package://rosie_1430_v1/meshes/link_6.dae",
		"package://example_arm/meshes/link_6.dae",
	} {
		if path, ok := MeshResourcePath(uri); !ok || path != "meshes/link_6.dae" {
			t.Fatalf("%s: %q %v", uri, path, ok)
		}
	}
	for _, uri := range []string{
		"file:///meshes/link_6.dae", "https://host/meshes/link_6.dae",
		"package:///meshes/link_6.dae", "package://../meshes/link_6.dae",
		"package://robot/meshes/../robot.urdf", "package://robot/meshes//link.dae",
		"package://robot/meshes/%2e%2e/link.dae", "package://robot/meshes/link.dae?x=y",
		"package://robot/robot.urdf", "package://robot/meshes/dir\\link.dae",
	} {
		if path, ok := MeshResourcePath(uri); ok {
			t.Fatalf("admitted %q as %q", uri, path)
		}
	}
}
