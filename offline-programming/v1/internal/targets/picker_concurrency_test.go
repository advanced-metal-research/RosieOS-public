//go:build linux

package targets

import (
	"context"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
)

type pollDuringArmBackend struct {
	denseexec.Backend
	entered, release chan struct{}
}

func (b *pollDuringArmBackend) Arm(ctx context.Context, _ bool) (*denseexec.ArmResult, error) {
	close(b.entered)
	select {
	case <-b.release:
		return &denseexec.ArmResult{OK: true}, nil
	case <-ctx.Done():
		return nil, ctx.Err()
	}
}
func (b *pollDuringArmBackend) Status(ctx context.Context) *denseexec.StatusBody {
	// Review remote RTT stimulus: 250 ms.
	timer := time.NewTimer(250 * time.Millisecond)
	defer timer.Stop()
	select {
	case <-timer.C:
		return &denseexec.StatusBody{Available: true}
	case <-ctx.Done():
		return &denseexec.StatusBody{}
	}
}

// TestReviewPickerPollsDuringArm requires status receipts before ARM is allowed
// to finish, preventing a selection lock from serializing UI polls behind work.
func TestReviewPickerPollsDuringArm(t *testing.T) {
	b := &pollDuringArmBackend{entered: make(chan struct{}), release: make(chan struct{})}
	p := &Picker{backend: b}
	// Unverified test watchdog: 3000 ms; four received polls establish success.
	ctx, cancel := context.WithTimeout(context.Background(), 3000*time.Millisecond)
	defer cancel()
	armed := make(chan error, 1)
	go func() { _, err := p.Arm(ctx, true); armed <- err }()
	<-b.entered
	for i := 0; i < 4; i++ {
		if !p.Status(ctx).Available {
			t.Fatal("poll queued behind ARM")
		}
	}
	close(b.release)
	if err := <-armed; err != nil {
		t.Fatal(err)
	}
}
