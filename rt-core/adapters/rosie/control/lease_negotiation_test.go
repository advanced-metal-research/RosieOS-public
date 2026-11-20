//go:build linux

package control

import (
	"context"
	"rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

type leaseNative struct {
	nativeClient
	deadline uint64
}

func (n *leaseNative) Err() error { return nil }
func (n *leaseNative) MirrorAcquire(_ context.Context, _ ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	n.deadline = deadline
	return ipcclient.GrantReceipt{DeadlineNS: deadline}, nil
}
func (n *leaseNative) MirrorRenew(ctx context.Context, id ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	return n.MirrorAcquire(ctx, id, deadline)
}

// TestCellLeaseNegotiation checks the adapter/native deadline agreement and
// renewal retention; a preset ceiling cannot replace a client's shorter lease.
func TestCellLeaseNegotiation(t *testing.T) {
	for _, tc := range []struct {
		ceiling       uint64
		request, want int
	}{
		{3000000000, 0, 500}, {3000000000, 500, 500}, {3000000000, 1350, 1350},
		{3000000000, 5000, 3000}, {500000000, 3000, 500}, {10000000000, 10000, 10000},
	} {
		native := &leaseNative{}
		binding := Binding{PairID: "cell", Revision: 1, ConfigurationSHA256: "config"}
		c := &Controller{core: native, binding: binding, description: ipcclient.Description{MaxGrantLeaseNS: tc.ceiling}}
		before := ipcclient.HostMonotonicNS()
		g, err := c.Acquire("operator", binding, tc.request)
		if err != nil {
			t.Fatal(err)
		}
		defer c.cancel()
		if g.LeaseMS != tc.want || native.deadline != g.DeadlineHostNS || native.deadline-before < uint64(tc.want)*1000000 {
			t.Fatalf("acquire=%+v native=%d", g, native.deadline)
		}
		g, err = c.Renew(Fence{g.Session, g.Generation})
		if err != nil || g.LeaseMS != tc.want || native.deadline != g.DeadlineHostNS {
			t.Fatalf("renew=%+v err=%v", g, err)
		}
		c.expires = time.Now().Add(-time.Nanosecond)
		if _, err = c.Renew(Fence{g.Session, g.Generation}); err == nil || err.Error() != ReasonExpired {
			t.Fatalf("expired renewal=%v", err)
		}
	}
	for _, request := range []int{-1, 10001} {
		c := &Controller{}
		if _, err := c.Acquire("operator", Binding{}, request); err == nil || err.Error() != ReasonInvalidRequestedLease {
			t.Fatalf("accepted %d: %v", request, err)
		}
	}
}
