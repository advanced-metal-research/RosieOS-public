package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"net/http"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/programstore"
)

// The cell program store's API (internal/programstore): the programs every
// client of this robot edits, kept on this OLP host. GET and POST only, so the
// Steam Deck's OlpClient needs nothing new, and a long-poll change feed rather
// than a stream, for the same reason.
const (
	cellProgramsPath   = "/api/offline-programming/v1/cell-programs"
	cellProgramsSchema = "offline-programming.cell-programs.v1"
	// One save request: the document plus its small envelope.
	maxCellProgramSaveBytes = programstore.MaxDocumentBytes + 64<<10
)

// cellProgramsExecutingPoll is how often the robot's session is read while a
// program is loaded, to release the edit lock when it ends. Unverified: the
// Deck polls status every 250 ms anyway, so one more read a second is small.
var cellProgramsExecutingPoll = time.Second

// cellPrograms is the store as this server serves it, with the one fact the
// store cannot know: which program the robot has loaded.
type cellPrograms struct {
	store   *programstore.Store
	label   string
	host    string
	backend denseexec.Backend

	mu                            sync.Mutex
	loadedSession, loadedProgram  string
	executingRef, executingReason string
	watching                      bool
}

func newCellPrograms(store *programstore.Store, label string, backend denseexec.Backend) *cellPrograms {
	if store == nil {
		return nil
	}
	host, _ := os.Hostname()
	if strings.TrimSpace(label) == "" {
		label = host
	}
	c := &cellPrograms{store: store, label: label, host: host, backend: backend}
	store.SetExecuting(c.executing)
	return c
}

func (c *cellPrograms) executing() (string, string) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.executingRef, c.executingReason
}

// noteLoaded records a dense Load: until its session ends, the program it
// carried is locked against edits (an edit could not change what plays, and
// would leave the spreadsheet showing something other than the robot does).
func (c *cellPrograms) noteLoaded(programID, sessionID string) {
	if c == nil || c.backend == nil || strings.TrimSpace(programID) == "" || sessionID == "" {
		return
	}
	c.mu.Lock()
	c.loadedSession, c.loadedProgram = sessionID, programID
	c.executingRef, c.executingReason = programID, "the robot has this program loaded (Stop to release)"
	start := !c.watching
	c.watching = true
	c.mu.Unlock()
	c.store.Notify()
	if start {
		go c.watchSession()
	}
}

func (c *cellPrograms) watchSession() {
	for {
		time.Sleep(cellProgramsExecutingPoll)
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		status := c.backend.Status(ctx)
		cancel()
		if status == nil {
			// Unreadable: keep the lock rather than guess the robot stopped.
			continue
		}
		c.mu.Lock()
		state := ""
		if status.Session != nil && status.Session.ID == c.loadedSession {
			state = status.Session.State
		}
		ref, reason := "", ""
		switch state {
		case "loaded", "staged", "playing", "paused":
			ref, reason = c.loadedProgram, fmt.Sprintf("the robot has this program %s (Stop to release)", state)
		}
		changed := ref != c.executingRef || reason != c.executingReason
		c.executingRef, c.executingReason = ref, reason
		done := ref == ""
		if done {
			c.watching, c.loadedSession, c.loadedProgram = false, "", ""
		}
		c.mu.Unlock()
		if changed {
			c.store.Notify()
		}
		if done {
			return
		}
	}
}

func (server *Server) registerCellPrograms(mux *http.ServeMux) {
	mux.HandleFunc("GET "+cellProgramsPath, server.cellProgramsList)
	mux.HandleFunc("GET "+cellProgramsPath+"/changes", server.cellProgramsChanges)
	mux.HandleFunc("POST "+cellProgramsPath+"/current", server.cellProgramsCurrent)
	mux.HandleFunc("GET "+cellProgramsPath+"/programs/{id}", server.cellProgramGet)
	mux.HandleFunc("GET "+cellProgramsPath+"/programs/{id}/revisions", server.cellProgramRevisions)
	mux.HandleFunc("POST "+cellProgramsPath+"/programs/{id}/save", server.cellProgramSave)
	mux.HandleFunc("POST "+cellProgramsPath+"/programs/{id}/delete", server.cellProgramDelete)
}

// cellProgramClient is who sent a request, as the store records it: the same
// identity the connected-clients list and the controller use (control.go).
func cellProgramClient(r *http.Request) (string, programstore.Client) {
	who := controlClientOf(r)
	client := programstore.Client{Label: who.Label, Kind: who.Kind, Address: who.Address}
	if id := strings.TrimSpace(r.Header.Get(clientIDHeader)); validClientID(id) {
		client.ID = id
	}
	return who.key, client
}

func cellProgramsError(w http.ResponseWriter, status int, code, detail string, extra map[string]any) {
	body := map[string]any{"ok": false, "error": code, "detail": detail}
	for k, v := range extra {
		body[k] = v
	}
	writeJSON(w, status, body)
}

func (server *Server) cellProgramsReady(w http.ResponseWriter) *cellPrograms {
	if server.programs == nil {
		cellProgramsError(w, http.StatusServiceUnavailable, "program_store_disabled",
			"this OLP server keeps no cell program store (serve --program-store, OFFLINE_PROGRAMMING_PROGRAM_STORE); programs stay on each client", nil)
		return nil
	}
	return server.programs
}

type cellPresenceView struct {
	programstore.Presence
	You bool `json:"you,omitempty"`
}

func (c *cellPrograms) view(you string, watched string) map[string]any {
	snap := c.store.Snapshot()
	presence := make([]cellPresenceView, 0, len(snap.Presence))
	for _, p := range snap.Presence {
		presence = append(presence, cellPresenceView{Presence: p, You: p.PresenceKey() == you})
	}
	body := map[string]any{
		"schema": cellProgramsSchema, "enabled": true,
		"store":   map[string]any{"id": snap.StoreID, "instance": snap.Instance, "label": c.label, "host": c.host},
		"version": snap.Version, "current": snap.Current, "executing": snap.Executing,
		"programs": snap.Programs, "presence": presence,
		"limits": map[string]any{"document_bytes": programstore.MaxDocumentBytes, "programs": programstore.MaxPrograms, "kept_revisions": programstore.KeepRevisions},
	}
	if watched != "" {
		if head, ok := c.store.Head(watched); ok {
			body["watched"] = head
		} else {
			body["watched"] = nil
		}
	}
	return body
}

func (server *Server) cellProgramsList(w http.ResponseWriter, r *http.Request) {
	c := server.cellProgramsReady(w)
	if c == nil {
		return
	}
	key, _ := cellProgramClient(r)
	writeJSON(w, http.StatusOK, c.view(key, ""))
}

// cellProgramsChanges is the change feed: it answers at once when the client
// has seen an older version or another server instance, else when anything
// changes (a save, the current program, who is watching or editing, what the
// robot runs) or after timeout_ms. Each request renews the client's presence.
func (server *Server) cellProgramsChanges(w http.ResponseWriter, r *http.Request) {
	c := server.cellProgramsReady(w)
	if c == nil {
		return
	}
	q := r.URL.Query()
	watch := strings.TrimSpace(q.Get("watch"))
	if watch != "" && !programstore.ValidID(watch) {
		cellProgramsError(w, http.StatusBadRequest, "invalid_program_id", programstore.ErrInvalidID.Error(), nil)
		return
	}
	since, _ := strconv.ParseUint(q.Get("since"), 10, 64)
	timeout := programstore.WaitLimit
	if ms, err := strconv.Atoi(q.Get("timeout_ms")); err == nil && ms >= 0 {
		timeout = time.Duration(ms) * time.Millisecond
	}
	key, client := cellProgramClient(r)
	if before, after := c.store.Touch(key, client, watch, q.Get("editing") == "1"); since == before {
		since = after
	}
	if q.Get("instance") == c.store.Instance() {
		c.store.Wait(r.Context(), since, timeout, nil)
	}
	writeJSON(w, http.StatusOK, c.view(key, watch))
}

func (server *Server) cellProgramGet(w http.ResponseWriter, r *http.Request) {
	c := server.cellProgramsReady(w)
	if c == nil {
		return
	}
	var revision uint64
	if raw := r.URL.Query().Get("revision"); raw != "" {
		value, err := strconv.ParseUint(raw, 10, 64)
		if err != nil || value == 0 {
			cellProgramsError(w, http.StatusBadRequest, "invalid_program_revision", "revision must be a positive integer", nil)
			return
		}
		revision = value
	}
	head, document, err := c.store.Get(r.PathValue("id"), revision)
	if err != nil {
		server.cellProgramFailure(w, err)
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"schema": cellProgramsSchema, "program": head, "document": json.RawMessage(document)})
}

func (server *Server) cellProgramRevisions(w http.ResponseWriter, r *http.Request) {
	c := server.cellProgramsReady(w)
	if c == nil {
		return
	}
	revisions, err := c.store.Revisions(r.PathValue("id"))
	if err != nil {
		server.cellProgramFailure(w, err)
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"schema": cellProgramsSchema, "program_id": r.PathValue("id"), "revisions": revisions})
}

type cellProgramSaveBody struct {
	BaseRevision uint64          `json:"base_revision"`
	SaveID       string          `json:"save_id"`
	MakeCurrent  bool            `json:"make_current"`
	Document     json.RawMessage `json:"document"`
}

func (server *Server) cellProgramSave(w http.ResponseWriter, r *http.Request) {
	c := server.cellProgramsReady(w)
	if c == nil {
		return
	}
	r.Body = http.MaxBytesReader(w, r.Body, maxCellProgramSaveBytes)
	var body cellProgramSaveBody
	decoder := json.NewDecoder(r.Body)
	if err := decoder.Decode(&body); err != nil {
		var tooLarge *http.MaxBytesError
		if errors.As(err, &tooLarge) {
			server.cellProgramFailure(w, programstore.ErrTooLarge)
			return
		}
		cellProgramsError(w, http.StatusBadRequest, "invalid_program_save", "save body is not {base_revision, save_id, document}: "+err.Error(), nil)
		return
	}
	if len(body.SaveID) > 128 {
		cellProgramsError(w, http.StatusBadRequest, "invalid_program_save", "save_id is longer than 128 characters", nil)
		return
	}
	_, client := cellProgramClient(r)
	id := r.PathValue("id")
	result, err := c.store.Save(programstore.SaveRequest{ProgramID: id, BaseRevision: body.BaseRevision, SaveID: body.SaveID,
		Document: body.Document, Client: client, MakeCurrent: body.MakeCurrent})
	if err != nil {
		log.Printf("cell-programs: save %s from r%d by %s refused: %v", id, body.BaseRevision, client.Label, err)
		server.cellProgramFailure(w, err)
		return
	}
	if !result.Replayed {
		log.Printf("cell-programs: %s r%d saved by %s (%d bytes)", id, result.Head.Revision, client.Label, result.Head.Bytes)
	}
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "program": result.Head, "replayed": result.Replayed, "version": result.Version, "current_refused": result.CurrentRefused})
}

func (server *Server) cellProgramDelete(w http.ResponseWriter, r *http.Request) {
	c := server.cellProgramsReady(w)
	if c == nil {
		return
	}
	var body struct {
		BaseRevision uint64 `json:"base_revision"`
	}
	if err := decodeBoundedJSON(w, r, 4<<10, &body); err != nil {
		cellProgramsError(w, http.StatusBadRequest, "invalid_program_delete", "delete body is not {base_revision}", nil)
		return
	}
	_, client := cellProgramClient(r)
	head, err := c.store.Delete(r.PathValue("id"), body.BaseRevision, client)
	if err != nil {
		server.cellProgramFailure(w, err)
		return
	}
	log.Printf("cell-programs: %s deleted at r%d by %s", head.ProgramID, head.Revision, client.Label)
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "program": head})
}

func (server *Server) cellProgramsCurrent(w http.ResponseWriter, r *http.Request) {
	c := server.cellProgramsReady(w)
	if c == nil {
		return
	}
	var body struct {
		ProgramID string `json:"program_id"`
	}
	if err := decodeBoundedJSON(w, r, 4<<10, &body); err != nil {
		cellProgramsError(w, http.StatusBadRequest, "invalid_program_current", "body is not {program_id}", nil)
		return
	}
	_, client := cellProgramClient(r)
	current, err := c.store.SetCurrent(body.ProgramID, client)
	if err != nil {
		server.cellProgramFailure(w, err)
		return
	}
	log.Printf("cell-programs: current program %s set by %s", current.ProgramID, client.Label)
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "current": current})
}

func (server *Server) cellProgramFailure(w http.ResponseWriter, err error) {
	var conflict *programstore.ConflictError
	var locked *programstore.LockedError
	var invalid *programstore.InvalidError
	switch {
	case errors.As(err, &conflict):
		cellProgramsError(w, http.StatusConflict, "program_conflict", err.Error(),
			map[string]any{"current": conflict.Current, "base_revision": conflict.Base})
	case errors.As(err, &locked):
		cellProgramsError(w, http.StatusLocked, "program_executing", locked.Reason, nil)
	case errors.As(err, &invalid):
		cellProgramsError(w, http.StatusBadRequest, "invalid_program", invalid.Reason, nil)
	case errors.Is(err, programstore.ErrInvalidID):
		cellProgramsError(w, http.StatusBadRequest, "invalid_program_id", err.Error(), nil)
	case errors.Is(err, programstore.ErrNotFound):
		cellProgramsError(w, http.StatusNotFound, "program_not_found", "the cell has no such program (or revision)", nil)
	case errors.Is(err, programstore.ErrTooLarge):
		cellProgramsError(w, http.StatusRequestEntityTooLarge, "program_too_large", err.Error(), nil)
	case errors.Is(err, programstore.ErrFull):
		cellProgramsError(w, http.StatusInsufficientStorage, "program_store_full", err.Error(), nil)
	default:
		log.Printf("cell-programs: %v", err)
		cellProgramsError(w, http.StatusInternalServerError, "program_store_failed", err.Error(), nil)
	}
}
