package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/programstore"
)

// loadBackend is a robot that can have one program loaded: Load starts a
// session, Status reports it until release().
type loadBackend struct {
	identityBackend
	mu      sync.Mutex
	session *denseexec.SessionInfo
}

func (b *loadBackend) Load(_ context.Context, request denseexec.LoadRequest) (*denseexec.LoadResult, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.session = &denseexec.SessionInfo{ID: "session-1", State: "loaded", PlanID: request.PlanID}
	return &denseexec.LoadResult{OK: true, SessionID: "session-1"}, nil
}

func (b *loadBackend) Status(context.Context) *denseexec.StatusBody {
	b.mu.Lock()
	defer b.mu.Unlock()
	body := &denseexec.StatusBody{Backend: "rt_core", Available: true}
	if b.session != nil {
		copy := *b.session
		body.Session = &copy
	}
	return body
}

func (b *loadBackend) set(state string) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if state == "" {
		b.session = nil
	} else if b.session != nil {
		b.session.State = state
	}
}

func cellProject(id, name string, revision int, nodes string) json.RawMessage {
	return json.RawMessage(fmt.Sprintf(`{"schema":"offline-programming.project-export.v1","project":{"schema":"offline-programming.project.v1",`+
		`"identity":{"id":%q,"name":%q,"revision":%d},"robot":{"model":"rosie_1430_v1","description":{"robotDescriptionSha256":"d1"}},`+
		`"program":{"programId":%q},"nodes":[%s]}}`, id, name, revision, id, nodes))
}

type cellClient struct {
	t      *testing.T
	base   string
	id     string
	label  string
	client *http.Client
}

func (c cellClient) do(method, path string, body any) (int, map[string]any) {
	c.t.Helper()
	var reader io.Reader
	if body != nil {
		raw, _ := json.Marshal(body)
		reader = bytes.NewReader(raw)
	}
	request, _ := http.NewRequest(method, c.base+cellProgramsPath+path, reader)
	request.Header.Set(clientIDHeader, c.id)
	request.Header.Set("X-Rosie-Client", c.label)
	request.Header.Set("Content-Type", "application/json")
	response, err := c.client.Do(request)
	if err != nil {
		c.t.Fatal(err)
	}
	defer response.Body.Close()
	var decoded map[string]any
	_ = json.NewDecoder(response.Body).Decode(&decoded)
	return response.StatusCode, decoded
}

func newCellProgramsServer(t *testing.T, dir string, backend denseexec.Backend) (*httptest.Server, *programstore.Store) {
	t.Helper()
	store, err := programstore.Open(dir)
	if err != nil {
		t.Fatal(err)
	}
	server := httptest.NewServer(NewServer(ServerOptions{DenseExecution: backend, ProgramStore: store, StoreLabel: "example-relay"}).Handler())
	t.Cleanup(func() { server.Close(); store.Close() })
	return server, store
}

func TestCellProgramsDisabledWithoutAStore(t *testing.T) {
	server := httptest.NewServer(NewServer(ServerOptions{}).Handler())
	defer server.Close()
	c := cellClient{t: t, base: server.URL, id: "tab-0011223344556677", label: "browser", client: server.Client()}
	for _, path := range []string{"", "/changes", "/programs/p1"} {
		if status, body := c.do("GET", path, nil); status != http.StatusServiceUnavailable || body["error"] != "program_store_disabled" {
			t.Fatalf("GET %q: %d %v", path, status, body)
		}
	}
}

func TestTwoClientsEditOneProgramThroughTheCell(t *testing.T) {
	server, store := newCellProgramsServer(t, t.TempDir(), &identityBackend{})
	tab := cellClient{t: t, base: server.URL, id: "tab-0011223344556677", label: "", client: server.Client()}
	deck := cellClient{t: t, base: server.URL, id: "deck-0123456789abcdef", label: "rosie-pendant-v5 (steamdeck-operator)", client: server.Client()}

	status, body := tab.do("POST", "/programs/p1/save", map[string]any{"base_revision": 0, "save_id": "s1", "document": cellProject("p1", "Bracket", 1, "")})
	if status != 200 || body["program"].(map[string]any)["revision"].(float64) != 1 {
		t.Fatalf("create: %d %v", status, body)
	}
	if body["version"].(float64) != float64(store.Snapshot().Version) {
		t.Fatalf("save version does not identify its store snapshot: %v", body)
	}
	status, body = deck.do("POST", "/current", map[string]any{"program_id": "p1"})
	if status != 200 {
		t.Fatalf("current: %d %v", status, body)
	}
	status, body = deck.do("GET", "", nil)
	if status != 200 || body["store"].(map[string]any)["label"] != "example-relay" || body["current"].(map[string]any)["program_id"] != "p1" {
		t.Fatalf("list: %d %v", status, body)
	}

	// The browser waits on the feed while the Deck edits.
	version := uint64(body["version"].(float64))
	instance := body["store"].(map[string]any)["instance"].(string)
	woke := make(chan map[string]any, 1)
	go func() {
		_, feed := tab.do("GET", fmt.Sprintf("/changes?since=%d&instance=%s&watch=p1&editing=0&timeout_ms=5000", version, instance), nil)
		woke <- feed
	}()
	time.Sleep(100 * time.Millisecond)
	status, body = deck.do("POST", "/programs/p1/save", map[string]any{"base_revision": 1, "save_id": "d1", "document": cellProject("p1", "Bracket", 2, `{"id":"deck-node"}`)})
	if status != 200 {
		t.Fatalf("deck save: %d %v", status, body)
	}
	var feed map[string]any
	select {
	case feed = <-woke:
	case <-time.After(3 * time.Second):
		t.Fatal("the browser's feed did not wake on the Deck's save")
	}
	watched := feed["watched"].(map[string]any)
	if watched["revision"].(float64) != 2 || !strings.Contains(watched["updated_by"].(map[string]any)["label"].(string), "steamdeck-operator") {
		t.Fatalf("watched after deck save: %v", watched)
	}

	// The browser's own edit from revision 1 is a conflict naming the Deck's revision.
	status, body = tab.do("POST", "/programs/p1/save", map[string]any{"base_revision": 1, "save_id": "s2", "document": cellProject("p1", "Bracket", 2, `{"id":"tab-node"}`)})
	if status != http.StatusConflict || body["error"] != "program_conflict" {
		t.Fatalf("stale save: %d %v", status, body)
	}
	current := body["current"].(map[string]any)
	if current["revision"].(float64) != 2 || current["updated_by"].(map[string]any)["id"] != deck.id || !strings.Contains(body["detail"].(string), "steamdeck-operator") {
		t.Fatalf("conflict body %v", body)
	}
	status, body = tab.do("GET", "/programs/p1", nil)
	if status != 200 || !strings.Contains(string(cellJSON(body["document"])), "deck-node") {
		t.Fatalf("the Deck's revision must stand: %d %v", status, body)
	}

	// Presence: the browser watched p1; the Deck sees it, marked as not itself.
	_, body = deck.do("GET", "/changes?since=0&watch=p1&editing=1", nil)
	presence := body["presence"].([]any)
	if len(presence) != 2 {
		t.Fatalf("presence %v", presence)
	}
	for _, raw := range presence {
		p := raw.(map[string]any)
		you, _ := p["you"].(bool)
		if (p["id"] == deck.id) != you {
			t.Fatalf("you marks the asking client only: %v", p)
		}
		if p["id"] == deck.id && p["editing"] != true {
			t.Fatalf("deck editing flag %v", p)
		}
	}
}

func cellJSON(value any) []byte {
	raw, _ := json.Marshal(value)
	return raw
}

func TestTheLoadedProgramIsLockedUntilItsSessionEnds(t *testing.T) {
	previous := cellProgramsExecutingPoll
	cellProgramsExecutingPoll = 20 * time.Millisecond
	defer func() { cellProgramsExecutingPoll = previous }()
	backend := &loadBackend{}
	server, _ := newCellProgramsServer(t, t.TempDir(), backend)
	tab := cellClient{t: t, base: server.URL, id: "tab-0011223344556677", client: server.Client()}
	if status, body := tab.do("POST", "/programs/p1/save", map[string]any{"base_revision": 0, "document": cellProject("p1", "Bracket", 1, "")}); status != 200 {
		t.Fatalf("create: %d %v", status, body)
	}
	if status, body := tab.do("POST", "/programs/p2/save", map[string]any{"base_revision": 0, "document": cellProject("p2", "Frame", 1, "")}); status != 200 {
		t.Fatalf("create p2: %d %v", status, body)
	}
	if status, _ := tab.do("POST", "/current", map[string]any{"program_id": "p1"}); status != 200 {
		t.Fatal("current")
	}
	// Load goes through the OLP's own route, as both clients load.
	load, _ := json.Marshal(map[string]any{"trajectory_digest": "sha256:t", "plan_id": "plan-1", "program_id": "p1", "program_digest": "sha256:p"})
	response, err := server.Client().Post(server.URL+denseExecutionPath+"/load", "application/json", bytes.NewReader(load))
	if err != nil || response.StatusCode != 200 {
		t.Fatalf("load: %v %v", err, response)
	}
	response.Body.Close()
	backend.set("playing")
	time.Sleep(60 * time.Millisecond)

	status, body := tab.do("POST", "/programs/p1/save", map[string]any{"base_revision": 1, "document": cellProject("p1", "Bracket", 2, "")})
	if status != http.StatusLocked || body["error"] != "program_executing" || !strings.Contains(body["detail"].(string), "playing") {
		t.Fatalf("edit while playing: %d %v", status, body)
	}
	if status, body := tab.do("POST", "/current", map[string]any{"program_id": "p2"}); status != http.StatusLocked {
		t.Fatalf("switch current while playing: %d %v", status, body)
	}
	_, list := tab.do("GET", "", nil)
	if executing, _ := list["executing"].(map[string]any); executing == nil || executing["program_id"] != "p1" {
		t.Fatalf("list executing %v", list["executing"])
	}
	if status, _ := tab.do("POST", "/programs/p2/save", map[string]any{"base_revision": 1, "document": cellProject("p2", "Frame", 2, "")}); status != 200 {
		t.Fatal("another program must stay editable")
	}

	backend.set("")
	deadline := time.Now().Add(2 * time.Second)
	for {
		status, body = tab.do("POST", "/programs/p1/save", map[string]any{"base_revision": 1, "document": cellProject("p1", "Bracket", 2, "")})
		if status == 200 {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("lock never released: %d %v", status, body)
		}
		time.Sleep(20 * time.Millisecond)
	}
}

func TestCellProgramsSurviveAnOLPRestart(t *testing.T) {
	dir := t.TempDir()
	store, err := programstore.Open(dir)
	if err != nil {
		t.Fatal(err)
	}
	first := httptest.NewServer(NewServer(ServerOptions{ProgramStore: store}).Handler())
	c := cellClient{t: t, base: first.URL, id: "deck-0123456789abcdef", label: "rosie-pendant-v5 (steamdeck-operator)", client: first.Client()}
	c.do("POST", "/programs/p1/save", map[string]any{"base_revision": 0, "make_current": true, "document": cellProject("p1", "Bracket", 1, `{"id":"n"}`)})
	first.Close()
	store.Close()

	second, _ := newCellProgramsServer(t, dir, nil)
	c.base, c.client = second.URL, second.Client()
	status, body := c.do("GET", "/programs/p1", nil)
	if status != 200 || body["program"].(map[string]any)["revision"].(float64) != 1 || !strings.Contains(string(cellJSON(body["document"])), `"id":"n"`) {
		t.Fatalf("after restart: %d %v", status, body)
	}
	_, list := c.do("GET", "", nil)
	if list["current"].(map[string]any)["program_id"] != "p1" {
		t.Fatalf("current after restart %v", list["current"])
	}
}

func TestOversizedProgramIsRefusedWithItsLimit(t *testing.T) {
	server, _ := newCellProgramsServer(t, t.TempDir(), nil)
	c := cellClient{t: t, base: server.URL, id: "tab-0011223344556677", client: server.Client()}
	pad := `{"pad":"` + strings.Repeat("x", programstore.MaxDocumentBytes+70<<10) + `"}`
	status, body := c.do("POST", "/programs/p1/save", map[string]any{"base_revision": 0, "document": cellProject("p1", "Big", 1, pad)})
	if status != http.StatusRequestEntityTooLarge || body["error"] != "program_too_large" {
		t.Fatalf("oversized: %d %v", status, body["error"])
	}
	status, body = c.do("POST", "/programs/p1/save", map[string]any{"base_revision": 0, "document": cellProject("p2", "Wrong", 1, "")})
	if status != http.StatusBadRequest || body["error"] != "invalid_program" {
		t.Fatalf("mismatched id: %d %v", status, body)
	}
}
