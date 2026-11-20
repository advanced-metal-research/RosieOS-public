package execution

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func TestJSONLAuditSinkDurablyAppendsEnvelope(t *testing.T) {
	path := filepath.Join(t.TempDir(), "audit", "execution.jsonl")
	sink, err := NewJSONLAuditSink(path)
	if err != nil {
		t.Fatal(err)
	}
	envelope := GatewayAuditEnvelope{
		Schema:     AuditSchema,
		EventID:    "audit-1",
		RecordedAt: time.Date(2026, 7, 30, 8, 0, 0, 0, time.UTC),
		Phase:      "review",
		Action:     "review",
		Outcome:    "approved",
		Reasons:    []string{"test"},
	}
	if err := sink.Append(context.Background(), envelope); err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if len(raw) == 0 || raw[len(raw)-1] != '\n' {
		t.Fatalf("audit is not newline-terminated: %q", raw)
	}
	var persisted GatewayAuditEnvelope
	if err := json.Unmarshal(raw[:len(raw)-1], &persisted); err != nil {
		t.Fatal(err)
	}
	if persisted.EventID != envelope.EventID ||
		persisted.Outcome != envelope.Outcome {
		t.Fatalf("persisted audit changed: %+v", persisted)
	}
}

func TestFileIdempotencyStoreSurvivesProcessReload(t *testing.T) {
	path := filepath.Join(t.TempDir(), "state", "idempotency.json")
	store, err := NewFileIdempotencyStore(path)
	if err != nil {
		t.Fatal(err)
	}
	committed, err := store.Reserve(context.Background(), "command-1")
	if err != nil || committed {
		t.Fatalf("first reservation committed=%v err=%v", committed, err)
	}
	if err := store.Commit(context.Background(), "command-1"); err != nil {
		t.Fatal(err)
	}
	reloaded, err := NewFileIdempotencyStore(path)
	if err != nil {
		t.Fatal(err)
	}
	committed, err = reloaded.Reserve(context.Background(), "command-1")
	if err != nil || !committed {
		t.Fatalf("durable replay committed=%v err=%v", committed, err)
	}
	if committed, err = reloaded.Reserve(
		context.Background(),
		"command-2",
	); err != nil || committed {
		t.Fatalf("second reservation committed=%v err=%v", committed, err)
	}
	if err := reloaded.Abort(context.Background(), "command-2"); err != nil {
		t.Fatal(err)
	}
	if committed, err = reloaded.Reserve(
		context.Background(),
		"command-2",
	); err != nil || committed {
		t.Fatalf("aborted key could not be reused: committed=%v err=%v", committed, err)
	}
}
