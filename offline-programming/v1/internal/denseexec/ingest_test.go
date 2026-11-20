package denseexec

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"io"
	"net"
	"strings"
	"testing"
	"time"
)

// The wire is [u64 BE size][u32 BE json_len][JSON][blob]: a client that
// gets the two length prefixes wrong is refused by the daemon with
// request_invalid and nothing else, so the exact bytes are pinned here.
func TestIngestUploadFramesControlJSONThenBlob(t *testing.T) {
	blob, plan := fixtureBlob(t)
	daemon := newFakeDaemon()
	client := &IngestClient{Addr: startFakeIngest(t, daemon)}

	reply, err := client.Upload(context.Background(), blob)
	if err != nil {
		t.Fatalf("upload: %v", err)
	}
	if !reply.OK || reply.Kind != "upload" || reply.TrajectoryDigest != plan.Header.TrajectoryDigest {
		t.Fatalf("upload reply %+v", reply)
	}
	if reply.TotalSampleCount != plan.Header.Sampling.TotalSampleCount || reply.SegmentCount != len(plan.Segments) {
		t.Fatalf("upload reply counts %+v", reply)
	}

	daemon.mu.Lock()
	frame := daemon.lastFrame
	daemon.mu.Unlock()
	jsonLen := binary.BigEndian.Uint32(frame[:4])
	control := frame[4 : 4+jsonLen]
	if string(control) != `{"kind":"upload"}` {
		t.Fatalf("control JSON %q", control)
	}
	if body := frame[4+jsonLen:]; len(body) != len(blob) || string(body) != string(blob) {
		t.Fatalf("blob not carried verbatim: %d bytes vs %d", len(body), len(blob))
	}

	if _, err := client.Validate(context.Background(), blob); err != nil {
		t.Fatalf("validate: %v", err)
	}
	daemon.mu.Lock()
	frame = daemon.lastFrame
	daemon.mu.Unlock()
	if string(frame[4:4+binary.BigEndian.Uint32(frame[:4])]) != `{"kind":"validate"}` {
		t.Fatalf("validate control %q", frame[4:])
	}
}

// Revisions are integers on the wire; the daemon reads them with json_u64
// and a string would count as missing.
func TestIngestPreloadCarriesIdentityAsIntegers(t *testing.T) {
	blob, plan := fixtureBlob(t)
	daemon := newFakeDaemon()
	client := &IngestClient{Addr: startFakeIngest(t, daemon)}
	if _, err := client.Upload(context.Background(), blob); err != nil {
		t.Fatalf("upload: %v", err)
	}
	identity := fixtureIdentity(plan)
	reply, err := client.Preload(context.Background(), identity)
	if err != nil {
		t.Fatalf("preload: %v", err)
	}
	if !reply.OK || reply.Kind != "preload" || reply.TotalSampleCount != plan.Header.Sampling.TotalSampleCount {
		t.Fatalf("preload reply %+v", reply)
	}
	daemon.mu.Lock()
	frame := daemon.lastFrame
	daemon.mu.Unlock()
	control := string(frame[4 : 4+binary.BigEndian.Uint32(frame[:4])])
	var decoded map[string]json.RawMessage
	if err := json.Unmarshal([]byte(control), &decoded); err != nil {
		t.Fatalf("control JSON: %v", err)
	}
	for _, key := range []string{"kind", "trajectory_digest", "plan_id", "program_id",
		"program_digest", "manifest_revision", "plan_revision"} {
		if _, ok := decoded[key]; !ok {
			t.Fatalf("preload control lacks %s: %s", key, control)
		}
	}
	if strings.HasPrefix(string(decoded["manifest_revision"]), `"`) ||
		strings.HasPrefix(string(decoded["plan_revision"]), `"`) {
		t.Fatalf("revisions must be integers: %s", control)
	}
	if len(frame) != 4+len(control) {
		t.Fatalf("preload carried a %d-byte body; it must carry none", len(frame)-4-len(control))
	}

	wrong := identity
	wrong.PlanID = "someone-else"
	_, err = client.Preload(context.Background(), wrong)
	var rejection *Rejection
	if !errors.As(err, &rejection) || rejection.Reason != "identity_mismatch" || rejection.Kind != "preload" {
		t.Fatalf("expected identity_mismatch rejection, got %v", err)
	}
}

func TestIngestRejectionCarriesSegmentIndexOnlyWhenNamed(t *testing.T) {
	blob, _ := fixtureBlob(t)
	daemon := newFakeDaemon()
	client := &IngestClient{Addr: startFakeIngest(t, daemon)}

	corrupted := append([]byte(nil), blob...)
	corrupted[len(corrupted)-2] ^= 0xff // inside the last block's bytes
	_, err := client.Upload(context.Background(), corrupted)
	var rejection *Rejection
	if !errors.As(err, &rejection) || rejection.Reason != "block_sha256_mismatch" {
		t.Fatalf("expected block_sha256_mismatch, got %v", err)
	}
	if rejection.SegmentIndex == nil {
		t.Fatalf("a per-segment rejection must name the segment: %+v", rejection)
	}

	_, err = client.Preload(context.Background(), Identity{TrajectoryDigest: "sha256:" + strings.Repeat("0", 64)})
	if !errors.As(err, &rejection) || rejection.Reason != "plan_not_found" {
		t.Fatalf("expected plan_not_found, got %v", err)
	}
	if rejection.SegmentIndex != nil {
		t.Fatalf("plan_not_found names no segment: %+v", rejection)
	}
}

func TestIngestStatusDecodesExecutorLeniently(t *testing.T) {
	blob, plan := fixtureBlob(t)
	daemon := newFakeDaemon()
	client := &IngestClient{Addr: startFakeIngest(t, daemon)}
	if _, err := client.Upload(context.Background(), blob); err != nil {
		t.Fatalf("upload: %v", err)
	}
	if _, err := client.Preload(context.Background(), fixtureIdentity(plan)); err != nil {
		t.Fatalf("preload: %v", err)
	}
	daemon.setExecutor("playing", 1, 3, 0.03)

	status, err := client.Status(context.Background())
	if err != nil {
		t.Fatalf("status: %v", err)
	}
	staged := status.StagedPlan()
	if staged == nil || staged.TrajectoryDigest != plan.Header.TrajectoryDigest {
		t.Fatalf("staged %+v", staged)
	}
	executor, ok := status.Executor()
	if !ok {
		t.Fatalf("executor absent: %s", status.ExecutorRaw)
	}
	if executor.State != "playing" || executor.SegmentIndex != 1 || executor.SampleIndex != 3 {
		t.Fatalf("executor %+v", executor)
	}
	if executor.Armed == nil || !*executor.Armed {
		t.Fatalf("armed not decoded: %+v", executor)
	}
	if executor.Authority == nil || executor.Authority.RuntimeMode != "csp" || executor.Authority.RuntimeModeEpoch != 7 {
		t.Fatalf("authority %+v", executor.Authority)
	}
	if executor.Feedback == nil || len(executor.Feedback.Axes) == 0 {
		t.Fatalf("feedback axes missing: %+v", executor.Feedback)
	}

	// An older daemon without the new fields, and a daemon with no
	// executor at all: both decode, neither errors.
	var old StatusReply
	if err := json.Unmarshal([]byte(`{"ok":true,"staged":null,"executor":{"state":"idle","segment_index":-1,"t_s":0,"sample_index":0,"torch_on":false,"feedback_age_ms":-1,"authority_fresh":false},"last_error":null}`), &old); err != nil {
		t.Fatalf("old status: %v", err)
	}
	executor, ok = old.Executor()
	if !ok || executor.Armed != nil || executor.Authority != nil || executor.Feedback != nil {
		t.Fatalf("old executor must decode with nil extensions: %+v", executor)
	}
	daemon.mu.Lock()
	daemon.executorAbsent = true
	daemon.mu.Unlock()
	status, err = client.Status(context.Background())
	if err != nil {
		t.Fatalf("status: %v", err)
	}
	if string(status.ExecutorRaw) != `"absent"` {
		t.Fatalf("executor raw %s", status.ExecutorRaw)
	}
	if _, ok := status.Executor(); ok {
		t.Fatalf(`"absent" must not decode as an executor`)
	}
}

// A reply frame claiming more than 64 MiB is refused before allocation.
func TestIngestRefusesOversizedReplyFrame(t *testing.T) {
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	defer listener.Close()
	go func() {
		conn, err := listener.Accept()
		if err != nil {
			return
		}
		defer conn.Close()
		// Consume the whole request before answering, and hold the
		// connection until the client is done: a close while the client is
		// still writing surfaces as a reset, not as the cap it is testing.
		var header [12]byte
		if _, err := io.ReadFull(conn, header[:]); err != nil {
			return
		}
		rest := binary.BigEndian.Uint64(header[:8]) - 4
		if _, err := io.CopyN(io.Discard, conn, int64(rest)); err != nil {
			return
		}
		var size [8]byte
		binary.BigEndian.PutUint64(size[:], MaxFrameBytes+1)
		_, _ = conn.Write(size[:])
		_, _ = conn.Read(header[:1]) // until the client hangs up
	}()
	client := &IngestClient{Addr: listener.Addr().String(), Deadline: 2 * time.Second}
	_, err = client.Status(context.Background())
	if err == nil || !strings.Contains(err.Error(), "outside 4..") {
		t.Fatalf("expected frame cap error, got %v", err)
	}
}

func TestIngestHonoursContextCancellation(t *testing.T) {
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	defer listener.Close()
	go func() {
		conn, err := listener.Accept()
		if err != nil {
			return
		}
		time.Sleep(3 * time.Second) // never answers in time
		conn.Close()
	}()
	ctx, cancel := context.WithTimeout(context.Background(), 200*time.Millisecond)
	defer cancel()
	started := time.Now()
	_, err = (&IngestClient{Addr: listener.Addr().String()}).Status(ctx)
	if !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("expected deadline error, got %v", err)
	}
	if elapsed := time.Since(started); elapsed > 1500*time.Millisecond {
		t.Fatalf("cancellation took %s; the read must unblock on the context", elapsed)
	}
}
