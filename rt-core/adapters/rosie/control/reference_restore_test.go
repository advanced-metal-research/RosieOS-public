//go:build linux

package control

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"fmt"
	"hash/crc32"
	"net/http/httptest"
	"os"
	"path/filepath"
	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/ipcclient"
	"strings"
	"testing"
	"time"
)

type deltaCore struct {
	nativeClient
	before, after      ipcclient.Status
	submitted, stopped int
	observations       []ipcclient.Status
	afterReads         int
	afterRead          func()
}

func (n *deltaCore) Err() error                             { return nil }
func (n *deltaCore) Motion() (ipcclient.MotionState, error) { return ipcclient.MotionState{}, nil }
func (n *deltaCore) Status() (ipcclient.Status, error) {
	if n.submitted == 0 {
		return n.before, nil
	}
	n.afterReads++
	if n.afterRead != nil {
		n.afterRead()
	}
	if len(n.observations) > 0 {
		s := n.observations[0]
		n.observations = n.observations[1:]
		return s, nil
	}
	return n.after, nil
}
func (n *deltaCore) RestoreReferenceVerifiedDelta(context.Context, uint32) (uint64, error) {
	n.submitted++
	return 19, nil
}
func (n *deltaCore) WaitResult(context.Context, uint64) (ipcclient.CommandResult, error) {
	return ipcclient.CommandResult{Sequence: 19}, nil
}
func (n *deltaCore) Stop(context.Context) (uint64, error) { n.stopped++; return 20, nil }

// deltaFixture is an HM35 (v3) anchor whose calibrated drive offset (-10)
// deliberately differs from -C0, displaced 17 counts in both frames.
func deltaFixture(t *testing.T) (*Controller, *deltaCore, Fence, anchors.Record) {
	t.Helper()
	fields := "anchor.source=raw_multi_turn_encoder|3\nhome.policy=truth|607c|0|i32;sign|-1\n"
	binding := Binding{PairID: "synthetic", Revision: 1}
	saved := anchors.Record{PairID: binding.PairID, PairRevision: binding.Revision, CoordinateIdentity: fields,
		CoordinateIdentitySHA256: fmt.Sprintf("%x", sha256.Sum256([]byte(fields))), HomeEpoch: 3, AnchorCounts: 123, HomeOffset: -10,
		TurnsCounts: 456, ToleranceCounts: 16, DaemonIncarnation: strings.Repeat("b", 32), CapturedHostNS: 1}
	now := ipcclient.HostMonotonicNS()
	before := ipcclient.Status{DaemonIncarnation: saved.DaemonIncarnation, ConfigurationSHA256: strings.Repeat("a", 64), ConfigurationEpoch: 1,
		HomeEpoch: 1, TimeNS: now - 2000000, ConfigVerifiedMask: 1, Axes: []ipcclient.AxisStatus{{RevisionValid: 1, PDOFresh: 1, CoordinateValid: 1,
			CoordinateSourceValid: 1, AnchorToleranceCounts: 16, CoordinateIdentity: fields,
			CoordinateIdentitySHA256: saved.CoordinateIdentitySHA256, PositionCounts: saved.AnchorCounts + 17, HomeOffset: saved.HomeOffset}}}
	before.Axes[0].IndependentAnchorSource = &ipcclient.IndependentAnchorSource{Valid: true, EncoderCounts: int64(saved.TurnsCounts + 17), CompletedNS: before.TimeNS, MaximumAgeNS: 3000000000}
	after := before
	after.TimeNS = now
	after.HomeEpoch = 2
	after.HomeValidMask = 1
	after.Axes = append([]ipcclient.AxisStatus(nil), before.Axes...)
	after.Axes[0].AnchorDeltaRestore = &ipcclient.AnchorDeltaRestoreStatus{Method: verifiedDeltaMethod, AnchorVersion: 3, Valid: true,
		SavedCommandCounts: saved.AnchorCounts, SavedEncoderCounts: saved.TurnsCounts, SavedLogicalOffsetCounts: saved.HomeOffset,
		SavedDriveHomeOffsetRaw: 10, SavedHomeEpoch: saved.HomeEpoch, BoundRestoreCommandSequence: 19, BoundConfigurationEpoch: after.ConfigurationEpoch,
		BoundHomeEpoch: after.HomeEpoch, BoundAxisCoordinateEpoch: 4, BoundCompletedNS: after.TimeNS - 1000000, BoundPDOReceipt: 17,
		BoundDeltaCommandCounts: 17, BoundDeltaEncoderCounts: 17}
	n := &deltaCore{before: before, after: after}
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	c := &Controller{core: n, sessionCtx: ctx, cancel: cancel, binding: binding, anchorStore: anchors.Store{Directory: t.TempDir()},
		grant: Grant{Session: "owner", Generation: 7}, expires: time.Now().Add(time.Second),
		description: ipcclient.Description{Axes: []ipcclient.AxisDescription{{ReferenceVerifiedDeltaRestore: &ipcclient.ReferenceVerifiedDeltaRestorePolicy{Method: verifiedDeltaMethod}}}}}
	if err := c.anchorStore.Save(saved); err != nil {
		t.Fatal(err)
	}
	return c, n, Fence{Session: "owner", Generation: 7}, saved
}

func TestVerifiedDeltaPublicDispatchPreservesV3Origin(t *testing.T) {
	c, n, f, _ := deltaFixture(t)
	original, _ := os.ReadFile(c.anchorStore.Path(0))
	mixed := n.after
	mixed.Axes = append([]ipcclient.AxisStatus(nil), mixed.Axes...)
	mixed.Axes[0].AnchorDeltaRestore = nil
	n.observations = []ipcclient.Status{mixed}
	body, _ := json.Marshal(Request{Schema: RequestSchema, Operation: OperationRestoreReferenceVerifiedDelta, Fence: f, AxisMask: 1})
	response := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(response, httptest.NewRequest("POST", "/v1/control", bytes.NewReader(body)))
	var env RawResponse
	var proof VerifiedDeltaRestoreReceipt
	if response.Code != 200 || json.Unmarshal(response.Body.Bytes(), &env) != nil || json.Unmarshal(env.Data, &proof) != nil ||
		proof.AnchorDeltaRestore == nil || env.Sequence != 19 || proof.AnchorDeltaRestore.BoundRestoreCommandSequence != 19 ||
		proof.AnchorDeltaRestore.SavedLogicalOffsetCounts != -10 || n.afterReads < 2 {
		t.Fatalf("%d %s", response.Code, response.Body.String())
	}
	after, _ := os.ReadFile(c.anchorStore.Path(0))
	if !bytes.Equal(original, after) {
		t.Fatal("v3 anchor rewritten")
	}
}

func TestVerifiedDeltaFinalReceiptRejectsStaleWrongJobAndIdentity(t *testing.T) {
	for name, mutate := range map[string]func(*ipcclient.Status){
		"stale":             func(s *ipcclient.Status) { s.TimeNS = ipcclient.HostMonotonicNS() - 300000000 },
		"future":            func(s *ipcclient.Status) { s.TimeNS = ipcclient.HostMonotonicNS() + 1000000000 },
		"old_epoch":         func(s *ipcclient.Status) { s.HomeEpoch-- },
		"changed_identity":  func(s *ipcclient.Status) { s.Axes[0].CoordinateIdentity += "bad" },
		"drive_offset":      func(s *ipcclient.Status) { s.Axes[0].HomeOffset++ },
		"serial":            func(s *ipcclient.Status) { s.Axes[0].SerialValid = 1; s.Axes[0].Serial++ },
		"wrong_version":     func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.AnchorVersion = 5 },
		"capture_zero":      func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.SavedLogicalOffsetCounts = -123 },
		"drive_raw":         func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.SavedDriveHomeOffsetRaw++ },
		"saved_epoch":       func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.SavedHomeEpoch++ },
		"saved_command":     func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.SavedCommandCounts++ },
		"saved_encoder":     func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.SavedEncoderCounts++ },
		"old_job":           func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.BoundRestoreCommandSequence-- },
		"command_disagrees": func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.BoundDeltaCommandCounts = 9103 },
		"mismatch":          func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.BoundDeltaEncoderCounts += 17 },
		"opposite":          func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.BoundDeltaEncoderCounts = -17 },
		"invalid":           func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore.Valid = false },
		"missing":           func(s *ipcclient.Status) { s.Axes[0].AnchorDeltaRestore = nil },
	} {
		t.Run(name, func(t *testing.T) {
			_, n, _, saved := deltaFixture(t)
			n.after.Axes = append([]ipcclient.AxisStatus(nil), n.after.Axes...)
			if proof := n.after.Axes[0].AnchorDeltaRestore; proof != nil {
				copy := *proof
				n.after.Axes[0].AnchorDeltaRestore = &copy
			}
			mutate(&n.after)
			if _, e := verifiedDeltaReceipt(n.after, n.before, 0, 19, saved); e == nil {
				t.Fatal("invalid final receipt accepted")
			}
		})
	}
}

func TestVerifiedDeltaAdmissionAndCancellation(t *testing.T) {
	for _, trial := range []string{"no_store", "unsupported", "lease", "stop", "stale_final"} {
		t.Run(trial, func(t *testing.T) {
			c, n, f, _ := deltaFixture(t)
			switch trial {
			case "no_store":
				c.anchorStore.Directory = ""
			case "unsupported":
				c.description.Axes[0].ReferenceVerifiedDeltaRestore = nil
			case "lease":
				c.expires = time.Now().Add(-time.Second)
			case "stop":
				n.afterRead = func() { c.cancel() }
			case "stale_final":
				n.after.TimeNS = ipcclient.HostMonotonicNS() - 300000000
			}
			_, ref, err := c.RestoreReferenceVerifiedDelta(f, 1)
			if err == nil || ref != nil {
				t.Fatal("bad admission accepted")
			}
			if trial == "stop" || trial == "stale_final" {
				if n.stopped == 0 {
					t.Fatal("failed active operation omitted Stop")
				}
			} else if n.submitted != 0 {
				t.Fatal("preflight failure submitted")
			}
		})
	}
}

// No delta cap: a large agreeing native delta (170 degrees on a 100:1 joint)
// is accepted; native has already checked the restored position's hard limits.
func TestVerifiedDeltaReceiptHasNoDeltaCap(t *testing.T) {
	_, n, _, saved := deltaFixture(t)
	n.after.Axes = append([]ipcclient.AxisStatus(nil), n.after.Axes...)
	proof := *n.after.Axes[0].AnchorDeltaRestore
	proof.BoundDeltaCommandCounts, proof.BoundDeltaEncoderCounts = 6189397+16, 6189397
	n.after.Axes[0].AnchorDeltaRestore = &proof
	if _, err := verifiedDeltaReceipt(n.after, n.before, 0, 19, saved); err != nil {
		t.Fatal(err)
	}
	proof.BoundDeltaCommandCounts++
	if _, err := verifiedDeltaReceipt(n.after, n.before, 0, 19, saved); err == nil {
		t.Fatal("17-count disagreement accepted")
	}
}

func TestVerifiedDeltaIgnoresUnqualifiedSerialBytes(t *testing.T) {
	_, n, _, saved := deltaFixture(t)
	n.after.Axes = append([]ipcclient.AxisStatus(nil), n.after.Axes...)
	n.after.Axes[0].Serial++
	if _, err := verifiedDeltaReceipt(n.after, n.before, 0, 19, saved); err != nil {
		t.Fatal(err)
	}
}

// A rosie-anchor-v5 file left by the removed current-pose capture is refused
// before anything is submitted, with the reason surfaced to the operator.
func TestVerifiedDeltaRefusesCurrentReferenceAnchorFile(t *testing.T) {
	c, n, f, _ := deltaFixture(t)
	payload := "rosie-anchor-v5 " + strings.Repeat("a", 64) + " 0 0 0 0 0 0 3 123 -10 456 131072 16 " + strings.Repeat("b", 32) +
		" 1 61 73 1 independent_absolute_current_pose_v1 5 1 3 4 0 123 456 -123 10 110 200 17\n"
	file := fmt.Sprintf("%s%08x\n", payload, crc32.ChecksumIEEE([]byte(payload)))
	if err := os.WriteFile(filepath.Join(c.anchorStore.Directory, filepath.Base(c.anchorStore.Path(0))), []byte(file), 0o600); err != nil {
		t.Fatal(err)
	}
	_, proof, err := c.RestoreReferenceVerifiedDelta(f, 1)
	if !errors.Is(err, anchors.ErrCurrentReference) || !errors.Is(err, anchors.ErrSource) || proof != nil || n.submitted != 0 {
		t.Fatalf("v5 anchor admitted: %v", err)
	}
	if !strings.Contains(err.Error(), "establish Home with drive homing (HM35)") {
		t.Fatal(err)
	}
	if got, _ := os.ReadFile(c.anchorStore.Path(0)); string(got) != file {
		t.Fatal("v5 anchor rewritten")
	}
}
