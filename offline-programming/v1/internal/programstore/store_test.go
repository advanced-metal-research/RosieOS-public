package programstore

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"testing"
	"time"
)

var deck = Client{ID: "deck-0123456789abcdef", Label: "rosie-pendant-v5 (steamdeck-operator)", Kind: "pendant", Address: "127.0.0.1"}
var tab = Client{ID: "tab-0011223344556677", Label: "Browser (Chrome on macOS) on http://127.0.0.1:5189", Kind: "browser", Address: "127.0.0.1"}

// project is a minimal OLP project document as exportProjectJSON writes it.
func project(id, name string, revision int, extra string) []byte {
	return []byte(fmt.Sprintf(`{"schema":"offline-programming.project-export.v1","exportedAt":"2026-10-11T00:00:00Z","project":`+
		`{"schema":"offline-programming.project.v1","identity":{"id":%q,"name":%q,"revision":%d},`+
		`"robot":{"model":"rosie_1430_v1","description":{"modelId":"rosie_1430_v1","robotDescriptionSha256":"abc123"}},`+
		`"program":{"programId":%q},"nodes":[%s]}}`, id, name, revision, id, extra))
}

func open(t *testing.T, dir string) *Store {
	t.Helper()
	s, err := Open(dir)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { s.Close() })
	return s
}

func save(t *testing.T, s *Store, id string, base uint64, doc []byte, client Client) Head {
	t.Helper()
	result, err := s.Save(SaveRequest{ProgramID: id, BaseRevision: base, Document: doc, Client: client, SaveID: fmt.Sprintf("save-%s-%d", id, base)})
	if err != nil {
		t.Fatalf("save %s from %d: %v", id, base, err)
	}
	return result.Head
}

func TestSaveFromStaleRevisionIsRefusedAndNeverOverwrites(t *testing.T) {
	s := open(t, t.TempDir())
	first := save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	if first.Revision != 1 || first.Name != "Bracket" || first.RobotModel != "rosie_1430_v1" || first.DescriptionSHA256 != "abc123" || first.ProgramRef != "p1" {
		t.Fatalf("first head %+v", first)
	}
	deckEdit := save(t, s, "p1", 1, project("p1", "Bracket", 2, `{"id":"deck"}`), deck)
	if deckEdit.Revision != 2 || deckEdit.UpdatedBy.Label != deck.Label {
		t.Fatalf("deck head %+v", deckEdit)
	}
	// The browser still edits revision 1.
	_, err := s.Save(SaveRequest{ProgramID: "p1", BaseRevision: 1, Document: project("p1", "Bracket", 2, `{"id":"tab"}`), Client: tab, SaveID: "tab-save"})
	var conflict *ConflictError
	if !errors.As(err, &conflict) || conflict.Current == nil || conflict.Current.Revision != 2 || conflict.Current.UpdatedBy.ID != deck.ID {
		t.Fatalf("stale save: %v (%+v)", err, conflict)
	}
	if !strings.Contains(err.Error(), "revision 2") || !strings.Contains(err.Error(), "steamdeck-operator") {
		t.Fatalf("conflict sentence %q", err)
	}
	_, doc, err := s.Get("p1", 0)
	if err != nil || !strings.Contains(string(doc), `"id":"deck"`) || strings.Contains(string(doc), `"id":"tab"`) {
		t.Fatalf("head document after refused save: %s %v", doc, err)
	}
	// Creating a program that exists is a conflict too, not a reset.
	if _, err := s.Save(SaveRequest{ProgramID: "p1", BaseRevision: 0, Document: project("p1", "Bracket", 1, ""), Client: tab}); !errors.As(err, &conflict) {
		t.Fatalf("create over existing: %v", err)
	}
	// And saving from a revision of a program the cell never had.
	if _, err := s.Save(SaveRequest{ProgramID: "p9", BaseRevision: 3, Document: project("p9", "X", 1, ""), Client: tab}); !errors.As(err, &conflict) || conflict.Current != nil {
		t.Fatalf("save to missing from 3: %v", err)
	}
}

func TestConcurrentSavesFromOneRevisionAcceptExactlyOne(t *testing.T) {
	s := open(t, t.TempDir())
	save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	const writers = 16
	var wg sync.WaitGroup
	var mu sync.Mutex
	accepted, conflicts := 0, 0
	for i := 0; i < writers; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			_, err := s.Save(SaveRequest{ProgramID: "p1", BaseRevision: 1, SaveID: fmt.Sprintf("w%d", i),
				Document: project("p1", "Bracket", 2, fmt.Sprintf(`{"id":"w%d"}`, i)), Client: Client{Label: fmt.Sprintf("writer %d", i)}})
			mu.Lock()
			defer mu.Unlock()
			var conflict *ConflictError
			switch {
			case err == nil:
				accepted++
			case errors.As(err, &conflict) && conflict.Current.Revision == 2:
				conflicts++
			default:
				t.Errorf("writer %d: %v", i, err)
			}
		}(i)
	}
	wg.Wait()
	if accepted != 1 || conflicts != writers-1 {
		t.Fatalf("accepted %d conflicts %d", accepted, conflicts)
	}
	head, _ := s.Head("p1")
	if head.Revision != 2 {
		t.Fatalf("head %+v", head)
	}
}

func TestStoreSurvivesRestartAndRefusesASecondServer(t *testing.T) {
	dir := t.TempDir()
	s, err := Open(dir)
	if err != nil {
		t.Fatal(err)
	}
	save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	save(t, s, "p1", 1, project("p1", "Bracket", 2, `{"id":"n1"}`), deck)
	save(t, s, "p2", 0, project("p2", "Frame", 1, ""), deck)
	if _, err := s.SetCurrent("p1", deck); err != nil {
		t.Fatal(err)
	}
	storeID := s.ID()
	if runtime.GOOS != "windows" {
		if _, err := Open(dir); err == nil || !strings.Contains(err.Error(), "another OLP server") {
			t.Fatalf("second open: %v", err)
		}
	}
	if err := s.Close(); err != nil {
		t.Fatal(err)
	}

	r := open(t, dir)
	if r.ID() != storeID {
		t.Fatalf("store id %s after restart, was %s", r.ID(), storeID)
	}
	if r.Instance() == s.Instance() {
		t.Fatal("a restarted server must announce a new instance so clients re-read")
	}
	snap := r.Snapshot()
	if len(snap.Programs) != 2 || snap.Current == nil || snap.Current.ProgramID != "p1" || snap.Current.SetBy.Label != deck.Label {
		t.Fatalf("snapshot after restart %+v", snap)
	}
	head, doc, err := r.Get("p1", 0)
	if err != nil || head.Revision != 2 || !strings.Contains(string(doc), `"id":"n1"`) {
		t.Fatalf("p1 after restart %+v %s %v", head, doc, err)
	}
	// Revision numbers continue; nothing restarts at 1.
	if next := save(t, r, "p1", 2, project("p1", "Bracket", 3, ""), tab); next.Revision != 3 {
		t.Fatalf("revision after restart %d", next.Revision)
	}
	if _, old, err := r.Get("p1", 1); err != nil || strings.Contains(string(old), "n1") {
		t.Fatalf("revision 1 is kept and unchanged: %s %v", old, err)
	}
}

func TestRetriedSaveWithTheSameSaveIDIsAReplay(t *testing.T) {
	s := open(t, t.TempDir())
	save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	request := SaveRequest{ProgramID: "p1", BaseRevision: 1, SaveID: "deck-save-7", Document: project("p1", "Bracket", 2, ""), Client: deck}
	first, err := s.Save(request)
	if err != nil || first.Replayed {
		t.Fatal(err)
	}
	if first.Version != s.Snapshot().Version {
		t.Fatalf("save version %d, snapshot version %d", first.Version, s.Snapshot().Version)
	}
	again, err := s.Save(request)
	if err != nil || !again.Replayed || again.Head.Revision != 2 {
		t.Fatalf("retry: %+v %v", again, err)
	}
	if again.Version != first.Version {
		t.Fatalf("replay version %d, accepted version %d", again.Version, first.Version)
	}
	if revs, _ := s.Revisions("p1"); len(revs) != 2 {
		t.Fatalf("a replay writes nothing: %v", revs)
	}
	// A different save from the same base is still a conflict.
	request.SaveID = "deck-save-8"
	if _, err := s.Save(request); err == nil {
		t.Fatal("a new save from a stale base was accepted")
	}
}

func TestTheProgramOnTheRobotCannotBeEditedOrSwitchedAway(t *testing.T) {
	s := open(t, t.TempDir())
	save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	save(t, s, "p2", 0, project("p2", "Frame", 1, ""), tab)
	if _, err := s.SetCurrent("p1", deck); err != nil {
		t.Fatal(err)
	}
	running := "p1"
	s.SetExecuting(func() (string, string) {
		if running == "" {
			return "", ""
		}
		return running, "the robot has this program playing (Stop to release)"
	})
	var locked *LockedError
	if _, err := s.Save(SaveRequest{ProgramID: "p1", BaseRevision: 1, Document: project("p1", "Bracket", 2, ""), Client: tab}); !errors.As(err, &locked) {
		t.Fatalf("edit of running program: %v", err)
	}
	if _, err := s.Delete("p1", 1, tab); !errors.As(err, &locked) {
		t.Fatalf("delete of running program: %v", err)
	}
	if _, err := s.SetCurrent("p2", tab); !errors.As(err, &locked) || !strings.Contains(err.Error(), "after Stop") {
		t.Fatalf("switching current away from the running program: %v", err)
	}
	// Other programs stay editable.
	save(t, s, "p2", 1, project("p2", "Frame", 2, ""), tab)
	snap := s.Snapshot()
	if snap.Executing == nil || snap.Executing.ProgramID != "p1" || snap.Executing.Reason == "" {
		t.Fatalf("snapshot executing %+v", snap.Executing)
	}
	running = ""
	save(t, s, "p1", 1, project("p1", "Bracket", 2, ""), tab)
	if _, err := s.SetCurrent("p2", tab); err != nil {
		t.Fatal(err)
	}
}

func TestChangeFeedWakesOnSaveAndPresence(t *testing.T) {
	s := open(t, t.TempDir())
	v0 := s.Snapshot().Version
	if v0 == 0 {
		t.Fatal("version 0 is reserved for a client that has seen nothing")
	}
	done := make(chan uint64, 1)
	go func() { done <- s.Wait(context.Background(), v0, 5*time.Second, nil) }()
	time.Sleep(50 * time.Millisecond)
	save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	select {
	case v := <-done:
		if v <= v0 {
			t.Fatalf("woke at %d", v)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("a save did not wake the feed")
	}
	v1 := s.Snapshot().Version
	go func() { done <- s.Wait(context.Background(), v1, 5*time.Second, nil) }()
	time.Sleep(50 * time.Millisecond)
	s.Touch("id:deck", deck, "p1", true)
	select {
	case v := <-done:
		if v <= v1 {
			t.Fatalf("woke at %d", v)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("a new editor did not wake the feed")
	}
	// The same presence again changes nothing.
	v2 := s.Snapshot().Version
	s.Touch("id:deck", deck, "p1", true)
	if s.Snapshot().Version != v2 {
		t.Fatal("renewing presence bumped the version")
	}
	start := time.Now()
	if v := s.Wait(context.Background(), v2, 150*time.Millisecond, nil); v != v2 || time.Since(start) < 100*time.Millisecond {
		t.Fatalf("idle wait returned %d after %s", v, time.Since(start))
	}
	// What the robot runs is not a store event: the caller's stale check ends the wait.
	start = time.Now()
	s.Wait(context.Background(), v2, 5*time.Second, func() bool { return true })
	if time.Since(start) > 100*time.Millisecond {
		t.Fatal("stale check did not end the wait")
	}
}

func TestPresenceExpiresWhenAClientStopsAsking(t *testing.T) {
	s := open(t, t.TempDir())
	now := time.Date(2026, 10, 11, 18, 0, 0, 0, time.UTC)
	s.now = func() time.Time { return now }
	s.Touch("id:deck", deck, "p1", true)
	s.Touch("id:tab", tab, "p1", false)
	snap := s.Snapshot()
	if len(snap.Presence) != 2 {
		t.Fatalf("presence %+v", snap.Presence)
	}
	now = now.Add(PresenceTTL - time.Second)
	s.Touch("id:tab", tab, "p1", false)
	now = now.Add(2 * time.Second)
	snap = s.Snapshot()
	if len(snap.Presence) != 1 || snap.Presence[0].Label != tab.Label {
		t.Fatalf("after deck went quiet %+v", snap.Presence)
	}
	s.Touch("id:tab", tab, "", false)
	if len(s.Snapshot().Presence) != 0 {
		t.Fatal("watching nothing must clear presence")
	}
}

func TestDocumentsAreBoundedAndMustNameTheirProgram(t *testing.T) {
	s := open(t, t.TempDir())
	big := project("p1", "Big", 1, `{"pad":"`+strings.Repeat("x", MaxDocumentBytes)+`"}`)
	if _, err := s.Save(SaveRequest{ProgramID: "p1", Document: big, Client: tab}); !errors.Is(err, ErrTooLarge) {
		t.Fatalf("oversized: %v", err)
	}
	var invalid *InvalidError
	cases := map[string][]byte{
		"wrong id":     project("p2", "X", 1, ""),
		"not json":     []byte("{nope"),
		"wrong schema": []byte(`{"schema":"robot.v4.program.v1","identity":{"id":"p1","name":"X"},"robot":{"model":"m"}}`),
		"no model":     []byte(`{"schema":"offline-programming.project.v1","identity":{"id":"p1","name":"X"},"robot":{}}`),
	}
	for name, doc := range cases {
		if _, err := s.Save(SaveRequest{ProgramID: "p1", Document: doc, Client: tab}); !errors.As(err, &invalid) {
			t.Errorf("%s: %v", name, err)
		}
	}
	for _, id := range []string{"", ".", "..", "../x", "a/b", strings.Repeat("a", 129)} {
		if _, err := s.Save(SaveRequest{ProgramID: id, Document: project(id, "X", 1, ""), Client: tab}); !errors.Is(err, ErrInvalidID) {
			t.Errorf("id %q: %v", id, err)
		}
	}
}

func TestOldRevisionsArePrunedAndStoredBytesAreVerbatim(t *testing.T) {
	dir := t.TempDir()
	s := open(t, dir)
	var head Head
	for i := 0; i < KeepRevisions+5; i++ {
		head = save(t, s, "p1", uint64(i), project("p1", "Bracket", i+1, fmt.Sprintf(`{"i":%d}`, i)), tab)
	}
	revs, _ := s.Revisions("p1")
	if len(revs) != KeepRevisions || revs[0] != 6 || revs[len(revs)-1] != head.Revision {
		t.Fatalf("kept %v", revs)
	}
	// The project is kept byte for byte: its description pin included.
	sent := project("p1", "Bracket", 99, `{"x":1.25e-3}`)
	head = save(t, s, "p1", head.Revision, sent, tab)
	_, doc, _ := s.Get("p1", 0)
	var envelope struct{ Project json.RawMessage }
	_ = json.Unmarshal(sent, &envelope)
	if string(doc) != string(envelope.Project) {
		t.Fatalf("stored\n%s\nsent\n%s", doc, envelope.Project)
	}
	if head.ProjectRevision != 99 {
		t.Fatalf("project revision %d", head.ProjectRevision)
	}
}

func TestDeleteIsATombstoneThatASaveRestores(t *testing.T) {
	s := open(t, t.TempDir())
	save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	if _, err := s.SetCurrent("p1", tab); err != nil {
		t.Fatal(err)
	}
	var conflict *ConflictError
	if _, err := s.Delete("p1", 0, tab); !errors.As(err, &conflict) {
		t.Fatalf("delete from a stale revision: %v", err)
	}
	tomb, err := s.Delete("p1", 1, tab)
	if err != nil || !tomb.Deleted || tomb.Revision != 2 {
		t.Fatalf("delete %+v %v", tomb, err)
	}
	snap := s.Snapshot()
	if len(snap.Programs) != 0 || snap.Current != nil {
		t.Fatalf("after delete %+v", snap)
	}
	if _, _, err := s.Get("p1", 0); !errors.Is(err, ErrNotFound) {
		t.Fatalf("get deleted: %v", err)
	}
	if _, _, err := s.Get("p1", 1); err != nil {
		t.Fatalf("the last live revision stays readable: %v", err)
	}
	if restored := save(t, s, "p1", 2, project("p1", "Bracket", 2, ""), tab); restored.Revision != 3 {
		t.Fatalf("restored %+v", restored)
	}
}

func TestAuditLogNamesWhoChangedWhat(t *testing.T) {
	dir := t.TempDir()
	s := open(t, dir)
	save(t, s, "p1", 0, project("p1", "Bracket", 1, ""), tab)
	_, _ = s.Save(SaveRequest{ProgramID: "p1", BaseRevision: 0, Document: project("p1", "Bracket", 1, ""), Client: deck})
	_, _ = s.SetCurrent("p1", deck)
	f, err := os.Open(filepath.Join(dir, "audit.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	var lines []auditEntry
	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		var entry auditEntry
		if err := json.Unmarshal(scanner.Bytes(), &entry); err != nil {
			t.Fatal(err)
		}
		lines = append(lines, entry)
	}
	if len(lines) != 3 {
		t.Fatalf("audit %+v", lines)
	}
	if lines[0].Action != "create" || lines[0].Result != "ok" || lines[0].Client.ID != tab.ID || lines[0].SHA256 == "" {
		t.Fatalf("create line %+v", lines[0])
	}
	if lines[1].Result != "conflict" || lines[1].Client.Label != deck.Label {
		t.Fatalf("refused line %+v", lines[1])
	}
	if lines[2].Action != "current" || lines[2].Client.Address != "127.0.0.1" {
		t.Fatalf("current line %+v", lines[2])
	}
}
