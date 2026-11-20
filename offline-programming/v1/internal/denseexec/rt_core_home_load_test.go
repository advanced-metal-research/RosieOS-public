//go:build linux

package denseexec

import (
	"context"
	"errors"
	"testing"
)

// Both a source migration (all anchors invalid) and a single lost reference
// must be rejected before authority acquisition or trajectory upload. Home
// remains an explicit available operation and permits a subsequent fresh load.
func TestRTCoreLoadRequiresCurrentHomeReference(t *testing.T) {
	for _, valid := range []uint32{0, 511 &^ (1 << 4)} {
		for _, dryRun := range []bool{false, true} {
			r, f, id := rtFake(t)
			f.mu.Lock()
			f.status.Core.HomeValidMask = valid
			f.mu.Unlock()
			_, err := r.Load(context.Background(), LoadRequest{Identity: id, DryRun: dryRun})
			var failure *Error
			if !errors.As(err, &failure) || failure.Code != "home_required" {
				t.Fatalf("mask %#x dry run %v: wanted Home guidance, got %v", valid, dryRun, err)
			}
			f.mu.Lock()
			calls := append([]string(nil), f.calls...)
			f.mu.Unlock()
			for _, call := range calls {
				if call == "acquire" || call == "prepare_program" || call == "home" || call == "arm" || call == "enable" {
					t.Fatalf("invalid reference sent %s: %v", call, calls)
				}
			}
			if err := r.Home(context.Background()); err != nil {
				t.Fatalf("explicit Home was blocked: %v", err)
			}
			// Normal load uses the fixture's identity. Dry-run identity derivation
			// is covered separately by the process-output stripping tests.
			if _, err := r.Load(context.Background(), LoadRequest{Identity: id}); err != nil {
				t.Fatalf("load after explicit Home failed: %v", err)
			}
		}
	}
}
