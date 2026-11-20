//go:build linux

package targets

import (
	"path/filepath"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"testing"
)

// TestExampleArmCellResolvesAsA1420Arm keeps example-arm-1420, the public
// placeholder arm cell, resolvable by the offline programming app from its
// manifest alone: the Rosie 1420 V1 model, its compiled pin, its pair and its
// documentation-address endpoint, with no cell id written into code.
func TestExampleArmCellResolvesAsA1420Arm(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	c, err := ResolveCellConfiguration(root, "example-arm-1420")
	if err != nil {
		t.Fatal(err)
	}
	if c.ModelID != "rosie_1420_v1" || c.SHA256 != "02da32195876513916a142d18250458778089a03c60dfbdf266a4405d651d89b" || c.PairID != "example-arm-1420" || c.Revision != 1 || c.Address != "https://192.0.2.21:8443" {
		t.Fatalf("wrong example cell: %+v", c)
	}
	if robotmodel.Class(c.ModelID) != "arm" || manifestHost(root, "example-arm-1420") != "example-relay" {
		t.Fatal("example-arm-1420 not admitted as an arm on example-relay")
	}
	if c.UnattendedLoopAllowed {
		t.Fatal("the placeholder cell must not allow unattended loops")
	}
}
