//go:build linux

package control

import "testing"

// TestKeepAliveDescribeCompiledProfiles preserves both compiled profile leases
// while publishing the listener timing consumed by Go and Deck reservations.
func TestKeepAliveDescribeCompiledProfiles(t *testing.T) {
	for _, tc := range []struct{ lease, age uint64 }{{500000000, 250000000}, {3000000000, 750000000}} {
		c := &Controller{}
		c.description.MaxGrantLeaseNS, c.description.MaxJogInputAgeNS = tc.lease, tc.age
		d := c.Describe()
		if d.ControlIdleTimeoutNS != 90000000000 || d.MaxGrantLeaseNS != tc.lease || d.MaxJogInputAgeNS != tc.age {
			t.Fatalf("description timing: %+v", d)
		}
	}
}
