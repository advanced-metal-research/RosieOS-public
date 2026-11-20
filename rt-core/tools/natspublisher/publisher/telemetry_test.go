//go:build linux

package publisher

import (
	"bytes"
	"context"
	"errors"
	"reflect"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type telemetryObserverFunc func(context.Context, uint64) (control.TelemetryBatch, error)

func (f telemetryObserverFunc) TelemetryBatches(ctx context.Context, after uint64) (control.TelemetryBatch, error) {
	return f(ctx, after)
}

// Reject V1 before publishing or retrying, preserving both digests through the
// direct batch and wrapped SDK error paths so operators can identify the layout.
func TestTelemetryV1MismatchIsPermanent(t *testing.T) {
	// Measured V1 digest and full record size from the pre-P8-34 schema.
	const oldDigest = "ae42a2ad68df32eff0d7745b7791a3c8b8e2f629991bbdb600b57d303e77b59c"
	h := ipcclient.TelemetryBatchHeaderV1{Magic: 0x31425452, Version: 1, HeaderBytes: 312,
		RecordBytes: 2088, AxisCount: 1, CyclePeriodNs: 1000000}
	copy(h.AdapterIncarnation[:], strings.Repeat("a", 32))
	copy(h.DaemonIncarnation[:], strings.Repeat("b", 32))
	copy(h.MachineSha256[:], strings.Repeat("c", 64))
	copy(h.DeploymentSha256[:], strings.Repeat("d", 64))
	copy(h.RecordLayoutDigest[:], oldDigest)
	check := func(t *testing.T, err error) {
		t.Helper()
		var mismatch *ipcclient.TelemetryLayoutMismatchError
		if !errors.As(err, &mismatch) || mismatch.StoredDigest != oldDigest ||
			mismatch.ExpectedDigest != ipcclient.CycleCaptureRecordV2LayoutDigest ||
			!strings.Contains(err.Error(), oldDigest) || !strings.Contains(err.Error(), ipcclient.CycleCaptureRecordV2LayoutDigest) || !permanent(err) {
			t.Fatalf("expected permanent typed layout mismatch naming both digests: %v", err)
		}
	}
	t.Run("direct", func(t *testing.T) {
		// Measured V2 batch bound: 312 + 2*5392 bytes.
		n := natsConn{payloadAdvertised: true, maxPayload: 11096}
		var after uint64
		err := n.publishTelemetry("layout-test", control.TelemetryBatch{Header: h}, &after)
		check(t, err)
		if !errors.Is(err, ErrContract) || after != 0 {
			t.Fatalf("contract/cursor changed: after=%d err=%v", after, err)
		}
	})
	for _, wrapped := range []bool{false, true} {
		name := "observer_batch"
		if wrapped {
			name = "observer_sdk_error"
		}
		t.Run(name, func(t *testing.T) {
			b := newBroker(t)
			ctx, cancel := context.WithTimeout(context.Background(), observation)
			defer cancel()
			p := &Publisher{config: Config{URL: b.url(), Host: "layout-test", TelemetryCadence: time.Millisecond, TelemetryMaxBytes: DefaultTelemetryMaxBytes}}
			calls, retries := 0, 0
			err := p.sendTelemetryWithDelay(ctx, telemetryObserverFunc(func(context.Context, uint64) (control.TelemetryBatch, error) {
				calls++
				if wrapped {
					return control.TelemetryBatch{}, errors.Join(control.ErrProtocol, &ipcclient.TelemetryLayoutMismatchError{
						StoredDigest: oldDigest, ExpectedDigest: ipcclient.CycleCaptureRecordV2LayoutDigest})
				}
				return control.TelemetryBatch{Header: h}, nil
			}), func(context.Context, time.Duration) bool { retries++; return false })
			check(t, err)
			b.mu.Lock()
			defer b.mu.Unlock()
			if calls != 1 || retries != 0 || b.connections != 1 || len(b.telemetryBatches) != 0 {
				t.Fatalf("calls=%d retries=%d connections=%d published=%d", calls, retries, b.connections, len(b.telemetryBatches))
			}
		})
	}
}

// An HTTP refusal must back off only polling. Two later acknowledged records on
// the original broker connection prove that the refusal did not tear it down.
func TestTelemetryRefusalKeepsNATSConnection(t *testing.T) {
	for _, reason := range []string{"telemetry_unavailable", "telemetry_busy"} {
		t.Run(reason, func(t *testing.T) { testTelemetryRefusalKeepsNATSConnection(t, reason) })
	}
}

func testTelemetryRefusalKeepsNATSConnection(t *testing.T, reason string) {
	b := newBroker(t)
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	p := &Publisher{config: Config{URL: b.url(), Host: "other-relay", TelemetryCadence: time.Millisecond, TelemetryMaxBytes: DefaultTelemetryMaxBytes}}
	h := ipcclient.TelemetryBatchHeaderV1{Magic: 0x31425452, Version: 1, HeaderBytes: 312, RecordBytes: 5392, CyclePeriodNs: 1000000, AxisCount: 1, RecordCount: 1}
	copy(h.AdapterIncarnation[:], strings.Repeat("a", 32))
	copy(h.DaemonIncarnation[:], strings.Repeat("b", 32))
	copy(h.MachineSha256[:], strings.Repeat("c", 64))
	copy(h.DeploymentSha256[:], strings.Repeat("d", 64))
	copy(h.RecordLayoutDigest[:], ipcclient.CycleCaptureRecordV2LayoutDigest)
	calls := 0
	var backoffs []time.Duration
	err := p.sendTelemetryWithDelay(ctx, telemetryObserverFunc(func(_ context.Context, after uint64) (control.TelemetryBatch, error) {
		calls++
		if calls == 2 {
			if after != 1 {
				t.Fatalf("cursor before refusal=%d", after)
			}
			return control.TelemetryBatch{}, &control.Rejected{Reason: reason}
		}
		if calls == 3 {
			if after != 1 || len(backoffs) != 1 || backoffs[0] != 250*time.Millisecond {
				t.Fatal("HTTP refusal lost cursor or backoff")
			}
		}
		if calls == 4 {
			b.mu.Lock()
			defer b.mu.Unlock()
			if after != 2 || b.connections != 1 || len(b.peers) != 1 || len(b.telemetryBatches) != 2 {
				t.Fatalf("after=%d connections=%d peers=%d batches=%d", after, b.connections, len(b.peers), len(b.telemetryBatches))
			}
			cancel()
			return control.TelemetryBatch{}, context.Canceled
		}
		h.FirstSequence, h.LastSequence = after+1, after+1
		return control.TelemetryBatch{Header: h, Records: []ipcclient.CycleCaptureRecordV2{{Seq: after + 1, Axes: 1}}}, nil
	}), func(_ context.Context, backoff time.Duration) bool {
		if calls != 2 {
			t.Fatalf("polls at backoff=%d", calls)
		}
		backoffs = append(backoffs, backoff)
		return true
	})
	if err != nil || calls != 4 {
		t.Fatalf("calls=%d err=%v", calls, err)
	}
}

// The broker independently records stream geometry and split payloads. Exact
// sequence/loss checks prevent max_payload splitting from dropping cycle records.
func TestTelemetrySubjectStreamAndSplitting(t *testing.T) {
	// Wire measurement: 312-byte header plus two 5392-byte records is 11096 bytes.
	b := newBroker(t, func(b *broker) { b.info = `{"jetstream":true,"max_payload":11096}` })
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	n, err := dialNATS(ctx, b.url())
	must(t, err)
	defer n.close()
	must(t, n.ensureStreamConfig(desiredTelemetryStream(DefaultTelemetryMaxBytes)))
	h := ipcclient.TelemetryBatchHeaderV1{Magic: 0x31425452, Version: 1, HeaderBytes: 312, RecordBytes: 5392, CyclePeriodNs: 1000000, AxisCount: 9, RecordCount: 5, FirstSequence: 8, LastSequence: 12, Dropped: 7}
	copy(h.AdapterIncarnation[:], strings.Repeat("a", 32))
	copy(h.DaemonIncarnation[:], strings.Repeat("e", 32))
	copy(h.MachineSha256[:], strings.Repeat("b", 64))
	copy(h.DeploymentSha256[:], strings.Repeat("c", 64))
	copy(h.RecordLayoutDigest[:], ipcclient.CycleCaptureRecordV2LayoutDigest)
	batch := control.TelemetryBatch{Header: h}
	for seq := uint64(8); seq <= 12; seq++ {
		record := ipcclient.CycleCaptureRecordV2{Seq: seq, Axes: 9, TNs: seq * 1000000}
		record.SafetyFaultMask, record.ExecutionFaultReasons = 1, 4096
		record.Ax[0].TorqueRaw, record.Ax[0].TorqueValid = -321, 1
		record.Ax[0].CollisionTorqueCycles, record.Ax[0].CollisionPeakTorqueRaw = 3, 321
		record.Ax[0].CoordinateCounts = -9007199254740993
		record.Ax[0].VelocityActualCountsPerS = -123456
		record.Ax[0].FollowingErrorCounts = 789
		record.Ax[0].VelocityActualValid = 1
		record.Ax[0].FollowingErrorValid = 1
		// A6 DI1 occupies bit 16; its validity and enabled state must survive splitting.
		record.Ax[0].DiBits, record.Ax[0].DiValid = 65536, 1
		record.Ax[0].ExternalEnableActive, record.Ax[0].ExternalEnableValid = 1, 1
		batch.Records = append(batch.Records, record)
	}
	var after uint64
	must(t, n.publishTelemetry("other-relay", batch, &after))
	if after != 12 {
		t.Fatalf("ack cursor %d", after)
	}
	b.mu.Lock()
	defer b.mu.Unlock()
	want := streamConfig{"ROSIE_RT_CORE_TELEMETRY", []string{"rosie/rt-core.*.telemetry.v1"}, "limits", "file", "old", -1, 536870912, 0}
	if b.telemetryConfig == nil || !reflect.DeepEqual(*b.telemetryConfig, want) || len(b.telemetryBatches) != 3 || len(b.last["rosie/rt-core.other-relay.telemetry.v1"]) == 0 {
		t.Fatalf("telemetry stream or subject missing: %+v batches=%d", b.telemetryConfig, len(b.telemetryBatches))
	}
	after = 0
	var records []ipcclient.CycleCaptureRecordV2
	for i, raw := range b.telemetryBatches {
		if len(raw) > 11096 {
			t.Fatal("negotiated payload exceeded")
		}
		part, err := ipcclient.ReadTelemetryPublication(bytes.NewReader(raw))
		must(t, err)
		must(t, part.ValidateAfter(after))
		if i > 0 && part.Header.Dropped != 0 {
			t.Fatal("loss repeated across split")
		}
		after = part.Header.LastSequence
		records = append(records, part.Records...)
	}
	if !reflect.DeepEqual(records, batch.Records) {
		t.Fatal("split changed records")
	}
}

func TestTelemetryStreamRepairOrRefuse(t *testing.T) {
	for _, incompatible := range []bool{false, true} {
		b := newBroker(t, func(b *broker) {
			c := desiredTelemetryStream(4096)
			if incompatible {
				c.Storage = "memory"
			}
			b.telemetryConfig = &c
		})
		ctx, cancel := context.WithCancel(context.Background())
		n, err := dialNATS(ctx, b.url())
		must(t, err)
		err = n.ensureStreamConfig(desiredTelemetryStream(DefaultTelemetryMaxBytes))
		n.close()
		cancel()
		if incompatible {
			if !errors.Is(err, ErrStreamConfig) {
				t.Fatal(err)
			}
		} else {
			must(t, err)
			b.mu.Lock()
			actual := b.telemetryConfig.MaxBytes
			b.mu.Unlock()
			if actual != 536870912 {
				t.Fatal(actual)
			}
		}
	}
}

// Missing or undersized broker limits fail before any network publication;
// a loss-only batch still requires an acknowledgement before advancing a cursor.
func TestTelemetryPayloadAdmissionAndLossOnly(t *testing.T) {
	h := ipcclient.TelemetryBatchHeaderV1{Magic: 0x31425452, Version: 1, HeaderBytes: 312, RecordBytes: 5392, CyclePeriodNs: 1000000, AxisCount: 1, LastSequence: 7, Dropped: 7}
	copy(h.AdapterIncarnation[:], strings.Repeat("a", 32))
	copy(h.DaemonIncarnation[:], strings.Repeat("e", 32))
	copy(h.MachineSha256[:], strings.Repeat("b", 64))
	copy(h.DeploymentSha256[:], strings.Repeat("c", 64))
	copy(h.RecordLayoutDigest[:], ipcclient.CycleCaptureRecordV2LayoutDigest)
	batch := control.TelemetryBatch{Header: h}
	for _, n := range []*natsConn{{maxPayload: 1048576}, {maxPayload: 1887, payloadAdvertised: true}} {
		var after uint64
		if err := n.publishTelemetry("other-relay", batch, &after); !errors.Is(err, ErrStreamConfig) || after != 0 {
			t.Fatalf("payload admission: cursor=%d err=%v", after, err)
		}
	}
	b := newBroker(t)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	n, err := dialNATS(ctx, b.url())
	must(t, err)
	defer n.close()
	must(t, n.ensureStreamConfig(desiredTelemetryStream(DefaultTelemetryMaxBytes)))
	var after uint64
	must(t, n.publishTelemetry("other-relay", batch, &after))
	if after != 7 {
		t.Fatal("loss-only acknowledgement did not advance cursor")
	}
	raw := b.latest("rosie/rt-core.other-relay.telemetry.v1")
	got, err := ipcclient.ReadTelemetryPublication(bytes.NewReader(raw))
	must(t, err)
	if len(raw) != 312 || got.Header.Dropped != 7 || got.Header.LastSequence != 7 || got.Header.RecordCount != 0 {
		t.Fatalf("loss-only publication: %+v", got.Header)
	}
}
