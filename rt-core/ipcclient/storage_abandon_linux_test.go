//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"testing"
	"time"
)

// Exercise BEGIN without the client's whole-upload transaction so failure
// paths cannot hide a leaked Writing claim behind a successful COMMIT.
func storageBeginUpload(t *testing.T, c *Client, id uint64) {
	t.Helper()
	payload := make([]byte, 24)
	wire.PutUint64(payload, id)
	wire.PutUint32(payload[8:], 3)
	wire.PutUint32(payload[16:], 250000)
	sequence, err := c.command(context.Background(), 0x0120, payload)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := c.WaitResult(context.Background(), sequence); err != nil {
		t.Fatalf("BEGIN %d: %v", id, err)
	}
}

func TestStorageAbandonedUploadReturnsWritingCapacity(t *testing.T) {
	for _, failure := range []string{"abort", "incomplete-commit", "replacement-begin", "stop", "disconnect"} {
		t.Run(failure, func(t *testing.T) {
			f := startCore(t, "simulation.json")
			c := f.client
			storageBeginUpload(t, c, 20001)
			switch failure {
			case "abort":
				if _, err := legacyAbort(t, c, 20001, 0, 0); err != nil {
					t.Fatal(err)
				}
			case "incomplete-commit":
				payload := make([]byte, 16)
				wire.PutUint64(payload, 20001)
				sequence, err := c.command(context.Background(), 0x0122, payload)
				if err != nil {
					t.Fatal(err)
				}
				_, err = c.WaitResult(context.Background(), sequence)
				var rejected *RejectedError
				if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonInvalidTrajectory {
					t.Fatalf("incomplete COMMIT: %v", err)
				}
			case "replacement-begin":
				storageBeginUpload(t, c, 20002)
				if _, err := legacyAbort(t, c, 20002, 0, 0); err != nil {
					t.Fatal(err)
				}
			case "stop":
				if _, err := c.Stop(context.Background()); err != nil {
					t.Fatal(err)
				}
			case "disconnect":
				if err := c.Close(); err != nil {
					t.Fatal(err)
				}
				ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
				defer cancel()
				for {
					var err error
					c, err = Dial(ctx, f.options)
					if err == nil {
						break
					}
					select {
					case <-ctx.Done():
						t.Fatalf("daemon did not accept the successor: %v", err)
					case <-time.After(time.Millisecond):
					}
				}
				t.Cleanup(func() { _ = c.Close() })
			}
			requireUnknownHandle(t, c, 20001)
			// All three slots must remain available without restarting the daemon.
			for id := uint64(20003); id <= 20005; id++ {
				prepareStationary(t, c, id)
			}
		})
	}
}
