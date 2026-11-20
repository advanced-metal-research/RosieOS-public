//go:build linux

package control

import (
	"context"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"fmt"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/ipcclient"
)

// Script observations independently of the receipt so stale or absent recovery
// cannot masquerade as successful restoration at the public dispatch boundary.
type restoreObservationCore struct {
	nativeClient
	initial  ipcclient.Status
	observe  func(int) ipcclient.Status
	reads    int
	restores int
}

func (n *restoreObservationCore) Err() error { return nil }
func (n *restoreObservationCore) Motion() (ipcclient.MotionState, error) {
	return ipcclient.MotionState{}, nil
}

func (n *restoreObservationCore) Status() (ipcclient.Status, error) {
	n.reads++
	if n.reads == 1 {
		return n.initial, nil
	}
	return n.observe(n.reads - 1), nil
}

func (n *restoreObservationCore) RestoreAnchor(context.Context, uint32) (uint64, error) {
	n.restores++
	return 19, nil
}

func (n *restoreObservationCore) WaitResult(context.Context, uint64) (ipcclient.CommandResult, error) {
	return ipcclient.CommandResult{Sequence: 19}, nil
}

func restoreObservationFixture(t *testing.T, observe func(int) ipcclient.Status) (*Controller, *restoreObservationCore, Fence) {
	t.Helper()
	dir, err := os.MkdirTemp(filepath.Join("..", "..", "..", "build"), "restore-observation-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	store := anchors.Store{Directory: dir}
	n := &restoreObservationCore{observe: observe, initial: ipcclient.Status{
		ConfigurationSHA256: strings.Repeat("a", 64), DaemonIncarnation: strings.Repeat("b", 32), HomeEpoch: 2,
		Axes: []ipcclient.AxisStatus{{RevisionValid: 1, PDOFresh: 1, CoordinateValid: 1, CoordinateSourceValid: 1}},
	}}
	record, err := anchorRecord(n.initial, 0, Binding{PairID: "observation-test", Revision: 1})
	if err != nil {
		t.Fatal(err)
	}
	record.CoordinateIdentity = "encoder.counts_per_motor_rev=131072\n"
	record.CoordinateIdentitySHA256 = fmt.Sprintf("%x", sha256.Sum256([]byte(record.CoordinateIdentity)))
	n.initial.Axes[0].CoordinateIdentity, n.initial.Axes[0].CoordinateIdentitySHA256 = record.CoordinateIdentity, record.CoordinateIdentitySHA256
	if err = store.Save(record); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	// Unverified test grant ceiling: 3 seconds; success requires scripted observations.
	c := &Controller{core: n, anchorStore: store, sessionCtx: ctx, cancel: cancel,
		binding: Binding{PairID: "observation-test", Revision: 1},
		grant:   Grant{Session: "owner", Generation: 7}, expires: time.Now().Add(3 * time.Second)}
	return c, n, Fence{Session: "owner", Generation: 7}
}

func TestRestoreAnchorRefusesAdapterPairMismatch(t *testing.T) {
	for _, field := range []string{"id", "revision"} {
		for _, sourceValid := range []uint32{0, 1} {
			t.Run(fmt.Sprintf("%s/source%d", field, sourceValid), func(t *testing.T) {
				c, n, fence := restoreObservationFixture(t, func(int) ipcclient.Status {
					t.Fatal("pair mismatch submitted native restore")
					return ipcclient.Status{}
				})
				if field == "id" {
					c.binding.PairID = "other"
				} else {
					c.binding.Revision = 2
				}
				n.initial.Axes[0].CoordinateSourceValid = sourceValid
				n.initial.Axes[0].AnchorReason = "anchor_ready"
				seq, recovery, err := c.RestoreAnchor(fence, 1)
				if !errors.Is(err, anchors.ErrIdentity) || seq != 0 || recovery != nil || n.reads != 1 {
					t.Fatalf("pair mismatch: seq=%d recovery=%+v reads=%d err=%v", seq, recovery, n.reads, err)
				}
				n.observe = func(int) ipcclient.Status { return n.initial }
				status := func() ipcclient.AxisStatus {
					t.Helper()
					response := httptest.NewRecorder()
					NewHandler(c).ServeHTTP(response, httptest.NewRequest("GET", "/v1/status", nil))
					var envelope struct {
						Data ProcessStatus `json:"data"`
					}
					if response.Code != 200 || json.Unmarshal(response.Body.Bytes(), &envelope) != nil || len(envelope.Data.Core.Axes) != 1 {
						t.Fatalf("public status: %d %s", response.Code, response.Body.String())
					}
					return envelope.Data.Core.Axes[0]
				}
				a := status()
				if a.AnchorReason != ReasonAnchorIdentityMismatch || a.AnchorIdentityChangedField != "pair."+field || a.CoordinateReason != 3 || n.restores != 0 {
					t.Fatalf("pair refusal publication: %+v restores=%d", a, n.restores)
				}
				if n.initial.Axes[0].AnchorIdentityChangedField != "" || n.initial.Axes[0].CoordinateReason != 0 {
					t.Fatal("adapter changed native observation storage")
				}
				n.initial.HomeEpoch, n.initial.HomeValidMask = 3, 1
				n.initial.Axes[0].CoordinateSourceValid = 1
				if err := c.captureAnchors(c.sessionCtx, 1, 2); err != nil {
					t.Fatal(err)
				}
				a = status()
				if a.AnchorIdentityChangedField != "" || a.CoordinateReason != 0 || a.AnchorReason != "anchor_ready" {
					t.Fatalf("replacement capture retained pair refusal: %+v", a)
				}
			})
		}
	}
}

func TestRestoreAnchorReturnsOnlyFreshCompletedRecovery(t *testing.T) {
	c, n, fence := restoreObservationFixture(t, func(read int) ipcclient.Status {
		state := ipcclient.Status{TimeNS: ipcclient.HostMonotonicNS(), HomeValidMask: 1,
			Recovery: &RecoveryStatus{HomeValidMask: 1, ResetSequence: 11, Faults: []FaultRecovery{}, Outcomes: []FaultRecovery{}}}
		switch read {
		case 1:
			state.TimeNS = 1 // A complete but stale snapshot cannot establish restoration.
		case 2:
			state.Recovery = nil
		case 3:
			state.CommissioningPhase = 1
		case 4:
			state.HomeValidMask = 0
		case 5:
			state.Recovery.HomeValidMask = 0
		}
		return state
	})
	response, err := NewHandler(c).dispatch(Request{Operation: OperationRestoreAnchor, Fence: fence, AxisMask: 1})
	if err != nil {
		t.Fatal(err)
	}
	body, err := json.Marshal(response.Data)
	if err != nil {
		t.Fatal(err)
	}
	var recovery RecoveryStatus
	if err = json.Unmarshal(body, &recovery); err != nil {
		t.Fatal(err)
	}
	if n.reads != 7 || response.Sequence != 19 || recovery.HomeValidMask != 1 || recovery.ResetSequence != 11 ||
		recovery.Faults == nil || len(recovery.Faults) != 0 || recovery.Outcomes == nil || len(recovery.Outcomes) != 0 || recovery.Reason != "" {
		t.Fatalf("restore response: reads=%d response=%+v recovery=%+v", n.reads, response, recovery)
	}
}

func TestRestoreAnchorMissingObservationFailsClosed(t *testing.T) {
	var cancel context.CancelFunc
	c, n, fence := restoreObservationFixture(t, func(int) ipcclient.Status {
		cancel()
		return ipcclient.Status{TimeNS: ipcclient.HostMonotonicNS(), HomeValidMask: 1}
	})
	cancel = c.cancel
	sequence, recovery, err := c.RestoreAnchor(fence, 1)
	if sequence != 19 || recovery != nil || err == nil || err.Error() != ReasonRecoveryObservationUnavailable || n.reads != 2 {
		t.Fatalf("missing recovery: sequence=%d recovery=%+v err=%v reads=%d", sequence, recovery, err, n.reads)
	}
	if !errors.Is(c.sessionCtx.Err(), context.Canceled) {
		t.Fatal("fixture did not cancel the observation")
	}
}

// A locally readable record is insufficient when the native process cannot
// read or qualify its own offer. Preserve the typed missing-evidence reason.
func TestRestoreAnchorReportsNativeOfferRefusal(t *testing.T) {
	for _, reason := range []string{ReasonAnchorMissing, ReasonAnchorIdentityMismatch, ReasonAnchorSourceInvalid, ReasonAnchorDisagrees} {
		t.Run(reason, func(t *testing.T) {
			c, n, fence := restoreObservationFixture(t, func(int) ipcclient.Status {
				t.Fatal("refused native offer must not submit restoration")
				return ipcclient.Status{}
			})
			n.initial.Axes[0].AnchorReason = reason
			sequence, recovery, err := c.RestoreAnchor(fence, 1)
			if sequence != 0 || recovery != nil || err == nil || err.Error() != reason {
				t.Fatalf("offer refusal: sequence=%d recovery=%+v error=%v", sequence, recovery, err)
			}
		})
	}
}

// A Home-valid publication can precede qualified coordinates. Capture must wait
// for the complete group, preserving older files until every axis is eligible.
func TestCaptureAnchorsWaitsForCoordinatePublication(t *testing.T) {
	c, n, _ := restoreObservationFixture(t, nil)
	state := n.initial
	state.HomeEpoch, state.HomeValidMask = 3, 3
	state.Axes = append(state.Axes, state.Axes[0])
	n.initial = state
	n.initial.Axes = append([]ipcclient.AxisStatus(nil), state.Axes...)
	n.initial.Axes[1].CoordinateSourceValid = 0
	n.observe = func(read int) ipcclient.Status {
		old, err := c.anchorStore.Load(0)
		if err != nil || old.HomeEpoch != 2 {
			t.Fatalf("partial group captured before source publication: %+v %v", old, err)
		}
		return state
	}
	if err := c.captureAnchors(c.sessionCtx, 3, 2); err != nil {
		t.Fatal(err)
	}
	if n.reads != 2 {
		t.Fatalf("capture observations: %d", n.reads)
	}
	for axis := uint32(0); axis < 2; axis++ {
		record, err := c.anchorStore.Load(axis)
		if err != nil || record.HomeEpoch != 3 {
			t.Fatalf("captured axis %d: %+v %v", axis, record, err)
		}
	}
}

// An explicit restore may use fresh raw evidence before a command origin exists.
// This must not make the same publication eligible for Home-anchor capture.
func TestRestoreIndependentAnchorBeforeBinding(t *testing.T) {
	for _, tc := range []struct {
		name   string
		change func(*Controller, *ipcclient.Status)
		want   error
	}{
		{"fresh", nil, nil},
		{"absent", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource = nil }, anchors.ErrSource},
		{"invalid", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource.Valid = false }, anchors.ErrSource},
		{"stale", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource.CompletedNS = 899 }, anchors.ErrSource},
		{"future", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource.CompletedNS = 1001 }, anchors.ErrSource},
		{"missing age bound", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource.MaximumAgeNS = 0 }, anchors.ErrSource},
		{"no PDO", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].PDOFresh = 0 }, anchors.ErrSource},
		{"no readback", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].RevisionValid = 0 }, anchors.ErrSource},
		{"no verified config", func(_ *Controller, s *ipcclient.Status) { s.ConfigVerifiedMask = 0 }, anchors.ErrSource},
		{"raw overflow", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource.EncoderCounts = 1 << 32 }, anchors.ErrSource},
		{"wrong identity", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].CoordinateIdentitySHA256 = strings.Repeat("c", 64) }, anchors.ErrIdentity},
		{"wrong pair", func(c *Controller, _ *ipcclient.Status) { c.binding.PairID = "replacement" }, anchors.ErrIdentity},
		{"raw displacement", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource.EncoderCounts += 17 }, anchors.ErrDisagrees},
		{"whole-period reset", func(_ *Controller, s *ipcclient.Status) { s.Axes[0].IndependentAnchorSource.EncoderCounts += 131072 }, anchors.ErrDisagrees},
	} {
		t.Run(tc.name, func(t *testing.T) {
			c, n, fence := restoreObservationFixture(t, func(int) ipcclient.Status {
				return ipcclient.Status{TimeNS: ipcclient.HostMonotonicNS(), HomeValidMask: 1,
					Recovery: &ipcclient.RecoveryStatus{HomeValidMask: 1}}
			})
			saved, err := c.anchorStore.Load(0)
			if err != nil {
				t.Fatal(err)
			}
			saved.CoordinateIdentity = "anchor.source=raw_multi_turn_encoder|3\nencoder.counts_per_motor_rev=131072\n"
			saved.CoordinateIdentitySHA256 = fmt.Sprintf("%x", sha256.Sum256([]byte(saved.CoordinateIdentity)))
			saved.TurnsCounts, saved.ToleranceCounts = 100, 16
			if err = c.anchorStore.Save(saved); err != nil {
				t.Fatal(err)
			}
			n.initial.TimeNS, n.initial.ConfigVerifiedMask = 1000, 1
			a := &n.initial.Axes[0]
			a.CoordinateIdentity, a.CoordinateIdentitySHA256 = saved.CoordinateIdentity, saved.CoordinateIdentitySHA256
			a.CoordinateValid, a.CoordinateSourceValid = 0, 0
			a.AnchorToleranceCounts = 16
			a.IndependentAnchorSource = &ipcclient.IndependentAnchorSource{Valid: true, EncoderCounts: 100, CompletedNS: 950, MaximumAgeNS: 100}
			if _, err := anchorRecord(n.initial, 0, c.binding); !errors.Is(err, anchors.ErrSource) {
				t.Fatal("unbound raw proof became capturable Home evidence", err)
			}
			if tc.change != nil {
				tc.change(c, &n.initial)
			}
			seq, recovery, err := c.RestoreAnchor(fence, 1)
			if tc.want != nil {
				if !errors.Is(err, tc.want) || seq != 0 || recovery != nil || n.restores != 0 {
					t.Fatalf("unqualified source submitted restore: seq=%d recovery=%+v err=%v calls=%d", seq, recovery, err, n.restores)
				}
			} else if err != nil || seq != 19 || recovery == nil || n.restores != 1 {
				t.Fatalf("qualified explicit restore blocked: seq=%d recovery=%+v err=%v calls=%d", seq, recovery, err, n.restores)
			}
		})
	}
}
