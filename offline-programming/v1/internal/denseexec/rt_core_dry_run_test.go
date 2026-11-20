//go:build linux

package denseexec

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"strings"
	"sync"
	"testing"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

// The native-side double checks the uploaded content, then echoes its identity.
// Real process-I/O qualification remains the adapter's existing responsibility.
type dryRunRTCore struct {
	*fakeRTCore
	captureMu     sync.Mutex
	blobs         [][]byte
	wrongIdentity bool
}

func (f *dryRunRTCore) PrepareProgram(ctx context.Context, blob []byte) (control.Program, error) {
	f.captureMu.Lock()
	f.blobs = append(f.blobs, append([]byte(nil), blob...))
	f.captureMu.Unlock()
	p, err := f.fakeRTCore.PrepareProgram(ctx, blob)
	if err != nil {
		return p, err
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		return p, err
	}
	var header map[string]json.RawMessage
	json.Unmarshal(blob[4:4+binary.BigEndian.Uint32(blob[:4])], &header)
	if len(header["process_markers"]) != 0 {
		return p, &control.Rejected{Reason: "process_io_executor_not_qualified"}
	}
	for _, segment := range plan.Segments {
		for _, sample := range segment.Samples {
			if sample.TorchOn {
				return p, &control.Rejected{Reason: "process_io_executor_not_qualified"}
			}
		}
	}
	if !f.wrongIdentity {
		p.Identity.TrajectoryDigest = plan.Header.TrajectoryDigest
		p.Identity.SourceDigest = plan.Header.TrajectoryDigest
	}
	p.Identity.NormalisedDigest = strings.Repeat("f", 64)
	return p, nil
}

func TestRTCoreDryRunLoadsExistingMotionAndStartsDerivedIdentity(t *testing.T) {
	r, fake, id := rtFake(t)
	native := &dryRunRTCore{fakeRTCore: fake}
	r.c = native
	ctx := context.Background()
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, DryRun: true})
	if err != nil {
		t.Fatal(err)
	}
	s := loaded.Status.Session
	if !s.DryRun || s.TrajectoryDigest != id.TrajectoryDigest || loaded.Trajectory.TrajectoryDigest != id.TrajectoryDigest ||
		s.ExecutionTrajectoryDigest == id.TrajectoryDigest || s.ExecutionNormalisedDigest != strings.Repeat("f", 64) {
		t.Fatalf("lost source/execution distinction: %+v", s)
	}
	fake.mu.Lock()
	if len(fake.calls) != 2 || fake.calls[0] != "acquire" || fake.calls[1] != "prepare_program" {
		t.Fatalf("Load caused extra operations: %v", fake.calls)
	}
	fake.mu.Unlock()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatal(err)
	}
	fake.mu.Lock()
	started := fake.started
	fake.mu.Unlock()
	if started.TrajectoryDigest != s.ExecutionTrajectoryDigest || started.SourceDigest != s.ExecutionTrajectoryDigest || started.NormalisedDigest != s.ExecutionNormalisedDigest {
		t.Fatalf("Start did not echo prepared execution identity: %+v", started)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err == nil {
		t.Fatal("stale dry-run session replayed")
	}
	if _, err := r.Load(ctx, LoadRequest{Identity: id}); err == nil || !strings.Contains(err.Error(), "process_io_executor_not_qualified") {
		t.Fatalf("normal load bypassed native process qualification: %v", err)
	}
	source, _ := fixtureBlob(t)
	native.captureMu.Lock()
	defer native.captureMu.Unlock()
	if len(native.blobs) != 2 || !bytes.Equal(native.blobs[1], source) {
		t.Fatal("normal reload changed original artifact")
	}
}

func TestRTCoreDryRunSourceAndPreparedIdentityAreBothChecked(t *testing.T) {
	for _, failure := range []string{"corrupt_source", "source_identity", "prepared_identity"} {
		t.Run(failure, func(t *testing.T) {
			r, f, id := rtFake(t)
			native := &dryRunRTCore{fakeRTCore: f, wrongIdentity: failure == "prepared_identity"}
			r.c = native
			if failure == "corrupt_source" {
				blob, _ := fixtureBlob(t)
				blob[len(blob)-1] ^= 1
				r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return blob, nil }
			}
			if failure == "source_identity" {
				id.PlanID = "different-plan"
			}
			if _, err := r.Load(context.Background(), LoadRequest{Identity: id, DryRun: true}); err == nil {
				t.Fatal("invalid identity accepted")
			}
			if session := r.statusSnapshot().Session; session != nil && session.Kind == SessionKindTrajectory {
				t.Fatal("failed load installed trajectory session")
			}
			native.captureMu.Lock()
			defer native.captureMu.Unlock()
			if failure != "prepared_identity" && len(native.blobs) != 0 {
				t.Fatal("invalid source reached native prepare")
			}
		})
	}
}

func TestRTCoreDryRunCancelledPrepareCannotInstallSession(t *testing.T) {
	r, f, id := rtFake(t)
	r.c = &dryRunRTCore{fakeRTCore: f}
	f.prepareEntered = make(chan struct{})
	f.prepareWait = make(chan struct{})
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	done := make(chan error, 1)
	go func() { _, err := r.Load(ctx, LoadRequest{Identity: id, DryRun: true}); done <- err }()
	<-f.prepareEntered
	cancel()
	if err := <-done; err == nil {
		t.Fatal("cancelled Load accepted")
	}
	if session := r.statusSnapshot().Session; session != nil && session.Kind == SessionKindTrajectory {
		t.Fatal("cancelled Load installed trajectory session")
	}
}

func TestRTCoreNormalLoadNeverAddsOutputsToMotionOnlySource(t *testing.T) {
	r, f, _ := rtFake(t)
	blob, _ := fixtureBlob(t)
	dry, plan, err := densejointtraj.WithoutProcessOutputs(blob)
	if err != nil {
		t.Fatal(err)
	}
	id := fixtureIdentity(plan)
	r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return dry, nil }
	native := &dryRunRTCore{fakeRTCore: f}
	r.c = native
	loaded, err := r.Load(context.Background(), LoadRequest{Identity: id})
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Status.Session.DryRun || loaded.Status.Session.ExecutionTrajectoryDigest != id.TrajectoryDigest {
		t.Fatal("normal load misrepresented source")
	}
	native.captureMu.Lock()
	defer native.captureMu.Unlock()
	if len(native.blobs) != 1 || !bytes.Equal(native.blobs[0], dry) {
		t.Fatal("normal load rewrote motion-only source")
	}
}
