package cells

import (
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// TestExampleArmCellCompilesAndCannotReachAHost keeps the public placeholder
// arm cells, example-arm-1420 and example-arm-1430, working bindings of the
// Rosie 1420 V1 and Rosie 1430 V1 descriptions on six axes while their hosts
// stay documentation addresses (RFC 5737, 192.0.2.0/24) and their login a
// placeholder, so tests and new cells can copy them and nobody can deploy one
// to hardware by mistake.
func TestExampleArmCellCompilesAndCannotReachAHost(t *testing.T) {
	root, err := filepath.Abs("../../..")
	if err != nil {
		t.Fatal(err)
	}
	for cell, pin := range map[string]string{
		"example-arm-1420.json": "02da32195876513916a142d18250458778089a03c60dfbdf266a4405d651d89b",
		"example-arm-1430.json": "95f304fd2b2fcc6598c86afd77d9e5489d4388717c20c444fccda42e8b76c3b3",
	} {
		t.Run(cell, func(t *testing.T) { checkExampleArmCell(t, root, cell, pin) })
	}
}

func checkExampleArmCell(t *testing.T, root, cell, pin string) {
	raw, err := os.ReadFile(filepath.Join(root, "rt-core/config/cells", cell))
	if err != nil {
		t.Fatal(err)
	}
	results, err := CheckManifest(root, raw)
	if err != nil && !errors.Is(err, ErrCompilerWarnings) {
		t.Fatal(err)
	}
	if len(results) != 1 || results[0].AxisCount != 6 || results[0].SHA256 != pin {
		t.Fatalf("example arm binding: %+v", results)
	}
	var manifest struct {
		Connections map[string]struct {
			Host    string `json:"host"`
			SSHUser string `json:"ssh_user"`
		} `json:"connections"`
		Nodes []struct {
			SSHHost    string `json:"ssh_host"`
			RemoteUser string `json:"remote_user"`
			Core       *struct {
				RemoteListen string `json:"remote_listen"`
				Activate     bool   `json:"activate"`
			} `json:"rt_core"`
		} `json:"nodes"`
	}
	if err := json.Unmarshal(raw, &manifest); err != nil {
		t.Fatal(err)
	}
	documentation := func(address string) bool { return strings.HasPrefix(address, "192.0.2.") }
	for name, c := range manifest.Connections {
		if !documentation(c.Host) || c.SSHUser != "changeme" {
			t.Errorf("connection %s reaches %s as %s; want a 192.0.2.x address and changeme", name, c.Host, c.SSHUser)
		}
	}
	for _, node := range manifest.Nodes {
		if !documentation(node.SSHHost) || node.RemoteUser != "changeme" {
			t.Errorf("node reaches %s as %s; want a 192.0.2.x address and changeme", node.SSHHost, node.RemoteUser)
		}
		if node.Core != nil && (node.Core.Activate || !documentation(node.Core.RemoteListen)) {
			t.Errorf("rt_core binding activates or listens on %s; want activate false on a 192.0.2.x address", node.Core.RemoteListen)
		}
	}
}
