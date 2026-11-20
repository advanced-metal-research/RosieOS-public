//go:build linux

package publisher

import (
	"context"
	"errors"
	"fmt"
	"time"

	contract "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Unverified rt-core-owned versioned JetStream archive name.
const TelemetryStream = "ROSIE_RT_CORE_TELEMETRY"

// Unverified byte retention: one 100000-cycle ring window at 1 kHz uses
// 100000*5392 + (100000/500)*312 = 539262400 bytes at 500 ms cadence.
// The unverified 512 MiB byte cap can evict records before the full ring window;
// headers, splits and JetStream subject overhead also consume this budget.
const DefaultTelemetryMaxBytes int64 = 512 * 1024 * 1024

type TelemetryObserver interface {
	TelemetryBatches(context.Context, uint64) (control.TelemetryBatch, error)
}

func telemetrySubject(host string) string {
	return "rosie/rt-core." + TopicToken(host) + ".telemetry.v1"
}
func desiredTelemetryStream(maxBytes int64) streamConfig {
	return streamConfig{TelemetryStream, []string{"rosie/rt-core.*.telemetry.v1"}, "limits", "file", "old", -1, maxBytes, 0}
}

// publishTelemetry splits complete records to the negotiated NATS payload bound.
// Only acknowledged records advance after; retry can duplicate uncertain acks
// but cannot erase loss or silently advance over unpublished records.
func (n *natsConn) publishTelemetry(host string, b control.TelemetryBatch, after *uint64) error {
	if !n.payloadAdvertised {
		return fmt.Errorf("%w: telemetry requires advertised max_payload", ErrStreamConfig)
	}
	if err := ipcclient.ValidateTelemetryHeader(b.Header); err != nil {
		return fmt.Errorf("%w: %w", ErrContract, err)
	}
	if int(b.Header.RecordCount) != len(b.Records) {
		return fmt.Errorf("%w: malformed telemetry batch", ErrContract)
	}
	limit := (n.maxPayload - ipcclient.TelemetryBatchHeaderV1Bytes) / ipcclient.CycleCaptureRecordV2Bytes
	if limit < 1 {
		return fmt.Errorf("%w: max_payload cannot hold a telemetry record", ErrStreamConfig)
	}
	if err := b.ValidateAfter(*after); err != nil {
		return fmt.Errorf("%w: %v", ErrContract, err)
	}
	if len(b.Records) == 0 {
		if b.Header.Dropped == 0 {
			return nil
		}
		raw, err := b.Encode()
		if err != nil {
			return err
		}
		if err = n.publishStream(telemetrySubject(host), raw, TelemetryStream); err != nil {
			return err
		}
		*after = b.Header.LastSequence
		return nil
	}
	for start := 0; start < len(b.Records); start += limit {
		end := min(start+limit, len(b.Records))
		part := b
		part.Records = b.Records[start:end]
		part.Header.RecordCount = uint32(end - start)
		part.Header.FirstSequence = part.Records[0].Seq
		part.Header.LastSequence = part.Records[len(part.Records)-1].Seq
		if start > 0 {
			part.Header.Dropped = 0
		}
		raw, err := part.Encode()
		if err != nil {
			return fmt.Errorf("%w: %v", ErrContract, err)
		}
		if err = n.publishStream(telemetrySubject(host), raw, TelemetryStream); err != nil {
			return err
		}
		*after = part.Header.LastSequence
	}
	return nil
}

// sendTelemetry owns an independent connection and retained cursor. Full-rate
// capture survives status sampling/backpressure; ring overwrite remains explicit.
func (p *Publisher) sendTelemetry(ctx context.Context, observer TelemetryObserver) error {
	return p.sendTelemetryWithDelay(ctx, observer, delay)
}

// sendTelemetryWithDelay shares production retry control flow with deterministic
// backoff checks, preventing wall-clock scheduling from standing in for evidence.
func (p *Publisher) sendTelemetryWithDelay(ctx context.Context, observer TelemetryObserver, wait func(context.Context, time.Duration) bool) error {
	var conn *natsConn
	defer func() {
		if conn != nil {
			conn.close()
		}
	}()
	var after uint64
	var incarnation [32]byte
	var daemon [32]byte
	first := true
	ticker := time.NewTicker(p.config.TelemetryCadence)
	defer ticker.Stop()
	backoff := reconnectMin
	for {
		if ctx.Err() != nil {
			return nil
		}
		var err error
		httpFailure := false
		if conn == nil {
			conn, err = dialNATS(ctx, p.config.URL)
			if err == nil {
				err = conn.ensureStreamConfig(desiredTelemetryStream(p.config.TelemetryMaxBytes))
			}
			if err == nil {
				err = conn.checkPublisher(telemetrySubject(p.config.Host), p.config.TelemetryCadence)
			}
		}
		if err == nil {
			for {
				readCtx, cancel := context.WithTimeout(ctx, operationTimeout)
				b, e := observer.TelemetryBatches(readCtx, after)
				cancel()
				err = e
				if err != nil {
					httpFailure = true
					var refusal *control.Rejected
					if errors.As(err, &refusal) && refusal.Reason != contract.ReasonTelemetryUnavailable && refusal.Reason != contract.ReasonTelemetryBusy {
						err = fmt.Errorf("%w: %v", ErrContract, err)
					}
					break
				}
				if !first && (b.Header.AdapterIncarnation != incarnation || b.Header.DaemonIncarnation != daemon) {
					return fmt.Errorf("%w: telemetry incarnation changed", ErrContract)
				}
				first = false
				incarnation = b.Header.AdapterIncarnation
				daemon = b.Header.DaemonIncarnation
				if err = conn.publishTelemetry(p.config.Host, b, &after); err != nil {
					break
				}
				if len(b.Records) < ipcclient.TelemetryBatchMaxRecords || ctx.Err() != nil {
					break
				}
			}
		}
		if ctx.Err() != nil {
			return nil
		}
		if err != nil {
			if permanent(err) {
				return err
			}
			// HTTP observation failures only back off polling; keeping the broker
			// connection prevents unrelated telemetry refusals from reconnecting it.
			if conn != nil && !httpFailure {
				conn.close()
				conn = nil
			}
			if !wait(ctx, backoff) {
				return nil
			}
			backoff = min(backoff*2, reconnectMax)
			continue
		}
		backoff = reconnectMin
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
		}
	}
}
