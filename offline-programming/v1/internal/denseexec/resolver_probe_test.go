package denseexec

import (
	"context"
	"os"
	"testing"
	"time"
)

// TestResolverProbeSSH runs the configured resolver command exactly as the
// server does (opt-in: set OLP_CARTESIAN_RESOLVER and OLP_CARTESIAN_RESOLVER_REPO),
// so a cell-side motion server can be checked from the offline-programming host
// before the app server is pointed at it. Verified 2026-09-16 against bench-relay:
// 2.1 s including the ssh handshake, 101 waypoints for a 100 mm X step.
func TestResolverProbeSSH(t *testing.T) {
	binary := os.Getenv("OLP_CARTESIAN_RESOLVER")
	repo := os.Getenv("OLP_CARTESIAN_RESOLVER_REPO")
	if binary == "" || repo == "" {
		t.Skip("probe needs OLP_CARTESIAN_RESOLVER and OLP_CARTESIAN_RESOLVER_REPO")
	}
	p := NewCartesianProcess(binary, repo, "rosie_1420_v1")
	defer p.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	start := time.Now()
	req := CartesianResolveRequest{Operation: "move", Model: "rosie_1420_v1", Frame: "base", Fraction: 0.05, Pose: [6]float64{-0.2758, 0.4155, -0.0616, 0.146, 0.0996, 0.0528}, Lower: [6]float64{-3.1, -2, -2.5, -3.1, -2.2, -6.2}, Upper: [6]float64{3.1, 2, 2.5, 3.1, 2.2, 6.2}}
	req.Delta[0] = -0.1
	out, err := p.Resolve(ctx, req)
	t.Logf("elapsed %v err=%v accepted=%v reason=%q waypoints=%d", time.Since(start), err, out.Accepted, out.Reason, len(out.Waypoints))
	if err != nil || !out.Accepted {
		t.Fatalf("probe failed: err=%v out=%+v", err, out)
	}
}
