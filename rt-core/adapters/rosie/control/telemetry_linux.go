//go:build linux

package control

import (
	"context"
	"errors"
	"unicode/utf8"

	"rosieos/rt-core/ipcclient"
)

// MarkTelemetry authorizes observation work against the current session without
// taking the motion operation lock; a busy Home must not block diagnostic marks.
// Invalid labels return the schema reason before reaching native admission.
// Each native acceptance publishes a receipt before returning; refused work
// cannot appear as a mark, and a receipt never claims successful disk output.
func (c *Controller) MarkTelemetry(f Fence, label string) (uint64, error) {
	c.mu.Lock()
	err := c.authorize(f)
	ctx := c.sessionCtx
	event := Event{Type: "telemetry_mark", DaemonIncarnation: c.daemonIncarnation, AdapterIncarnation: c.adapterIncarnation}
	marker, ok := c.core.(interface {
		MarkTelemetry(context.Context, string) (uint64, error)
	})
	c.mu.Unlock()
	if err != nil {
		return 0, err
	}
	if len(label) < 1 || len(label) > 128 || !utf8.ValidString(label) {
		return 0, errors.New(ReasonTelemetryLabelInvalid)
	}
	if !ok {
		return 0, errors.New(ReasonCapabilityUnimplemented)
	}
	sequence, err := marker.MarkTelemetry(ctx, label)
	if err == nil {
		// Receipt time belongs to the host monotonic clock, not a potentially
		// older motion sample. Identity remains that of the authorizing session.
		event.TimeNS = ipcclient.HostMonotonicNS()
		event.Mark = &TelemetryMarkEvent{Label: label, NativeSequence: sequence, Generation: f.Generation}
		c.events.append(event)
	}
	return sequence, err
}

// Hold the native incarnation through admission so reconnect cannot send a
// diagnostic mark from a retired session into the replacement daemon.
func (n *restartNative) MarkTelemetry(ctx context.Context, label string) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return n.client.MarkTelemetry(ctx, label)
}
