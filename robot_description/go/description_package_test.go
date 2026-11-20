package robotdesc

import "testing"

// TestCatalogueNamesTheURDFMeshPackage keeps the mesh package beside the model
// id: a description may name its meshes under another package (one robot's
// copy of a family's description), and a viewer finds the meshes and the
// family through it rather than through a list of model ids.
func TestCatalogueNamesTheURDFMeshPackage(t *testing.T) {
	for raw, want := range map[string]string{
		`<mesh filename="package://example_family_v1/meshes/link_1.dae"/>`:              "example_family_v1",
		`<mesh filename="file:///abs.dae"/><mesh filename="package://p/meshes/a.dae"/>`: "p",
		`<mesh filename="package://p/../escape.dae"/>`:                                  "",
		`<robot name="no_meshes"/>`:                                                     "",
	} {
		if got := urdfMeshPackage([]byte(raw)); got != want {
			t.Errorf("urdfMeshPackage(%q) = %q, want %q", raw, got, want)
		}
	}
	description, err := At("../robots").Load("rosie_1420_v1")
	if err != nil {
		t.Fatal(err)
	}
	if got := description.Robot().URDFPackage; got != "rosie_1420_v1" {
		t.Fatalf("rosie_1420_v1 catalogue package = %q", got)
	}
}
