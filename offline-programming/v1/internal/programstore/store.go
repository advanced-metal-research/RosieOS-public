// Package programstore is the cell's program store: the robot programs that
// the browser OLP and the Steam Deck pendant both edit, kept on the cell's own
// OLP host so every client of one robot opens one program with one revision
// history.
//
// Before it, a program lived only where it was edited -- a browser origin's
// IndexedDB, or the Deck's current.olp.json -- and two clients of the same
// robot each held a different spreadsheet with nothing to tell them apart.
// The store exists to make that impossible without making it silent: every
// save names the revision it was edited from, a save from a stale revision is
// refused with the server's revision (never merged, never overwritten), and
// what the robot is running cannot be edited underneath it.
//
// The store holds the client's document verbatim (the OLP project, exactly as
// `exportProjectJSON` wraps it); OLP's own TypeScript validates it on open.
// The server reads only the identity it needs to file it and to list it.
//
// Layout under the store directory, which deliberately lives outside any
// deployment tree so a release or rollback never takes programs with it:
//
//	STORE                     marker: schema and the store's persistent id
//	LOCK                      flock: one OLP server per store
//	state.json                the cell's current program
//	audit.jsonl[.1]           every mutation and refused save: who, when, what
//	programs/<id>/head.json   the latest revision's metadata
//	programs/<id>/rNNNNNNNN.json  one immutable file per revision (the last KeepRevisions)
package programstore

import (
	"context"
	"crypto/rand"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	Schema         = "offline-programming.program-store.v1"
	RevisionSchema = "offline-programming.program-revision.v1"
	exportSchema   = "offline-programming.project-export.v1"
	projectSchema  = "offline-programming.project.v1"

	// MaxDocumentBytes bounds one program document. Design bound, unmeasured
	// against real programs: it is the response bound the catalog bridge
	// already accepts (internal/catalog MaxResponseBytes), and an OLP project
	// keeps its plan as a summary plus a digest, never the dense samples.
	MaxDocumentBytes = 16 << 20
	// MaxPrograms bounds the live programs in one cell's store. Design bound.
	MaxPrograms = 512
	// KeepRevisions is how many revision files each program keeps; older ones
	// are deleted as new ones are written. This history is the store's backup
	// of every recent edit (olp install also archives the whole store).
	// Design bound: autosave writes a revision per settled edit.
	KeepRevisions = 50
	// MaxStoreBytes refuses saves once the kept revision files reach it, so a
	// runaway client fills this bound, not the OLP host's disk. Design bound.
	MaxStoreBytes = 4 << 30
	// PresenceTTL: a client is listed as viewing or editing while its change
	// feed request is this recent. Clients wait at most WaitLimit per request,
	// so a live client renews well inside it.
	PresenceTTL = 45 * time.Second
	// WaitLimit bounds one change-feed wait.
	WaitLimit = 25 * time.Second

	auditRotateBytes = 16 << 20
)

var (
	ErrNotFound  = errors.New("program not found")
	ErrTooLarge  = fmt.Errorf("program document exceeds %d bytes", MaxDocumentBytes)
	ErrFull      = errors.New("program store is full")
	ErrInvalidID = errors.New("program id must be 1-128 of [A-Za-z0-9._:-] and not start with '.'")
	programID    = regexp.MustCompile(`^[A-Za-z0-9_:-][A-Za-z0-9._:-]{0,127}$`)
)

// ValidID reports whether id can name a program (and its directory).
func ValidID(id string) bool { return programID.MatchString(id) }

// Client is who made a change, as the OLP server identifies requests
// (control.go): the tab's or pendant's X-Rosie-Client-Id, its label and the
// peer address. Behind the SSH tunnel every address is loopback; the id and
// label are what tell clients apart there.
type Client struct {
	ID      string `json:"id,omitempty"`
	Label   string `json:"label"`
	Kind    string `json:"kind,omitempty"`
	Address string `json:"address,omitempty"`
}

// Head is one revision's metadata: what the list shows and what a conflict
// answers with.
type Head struct {
	ProgramID string `json:"program_id"`
	// Revision is the store's own counter for this program: 1 for the first
	// save, +1 for every accepted save or delete, never reused. It is not the
	// project's identity.revision, which each client advances locally.
	Revision  uint64 `json:"revision"`
	Deleted   bool   `json:"deleted,omitempty"`
	CreatedAt string `json:"created_at"`
	UpdatedAt string `json:"updated_at"`
	UpdatedBy Client `json:"updated_by"`
	// SaveID is the client's id for the save that wrote this revision; a
	// retried save with the same id is answered with this revision instead of
	// a conflict against itself.
	SaveID string `json:"save_id,omitempty"`
	SHA256 string `json:"sha256,omitempty"`
	Bytes  int    `json:"bytes"`
	Name   string `json:"name"`
	// RobotModel and DescriptionSHA256 are the program's robot and the robot
	// description it is pinned to (docs/ROBOT-DESCRIPTION-CONTRACT.md), read
	// from the document so a list can say which robot a program is for.
	RobotModel        string `json:"robot_model"`
	DescriptionSHA256 string `json:"description_sha256,omitempty"`
	// ProgramRef is the document's program.programId: the program_id a plan
	// made from it carries to dense execution's Load.
	ProgramRef      string `json:"program_ref,omitempty"`
	ProjectRevision uint64 `json:"project_revision"`
}

// Current is the cell's current program: what the Deck shows and what a
// client opens by default.
type Current struct {
	ProgramID string `json:"program_id"`
	SetAt     string `json:"set_at"`
	SetBy     Client `json:"set_by"`
}

// Presence is one client watching a program through the change feed.
type Presence struct {
	ProgramID string `json:"program_id"`
	Client
	Editing  bool   `json:"editing"`
	LastSeen string `json:"last_seen"`
	key      string
	seen     time.Time
}

// ConflictError is a save or delete from a revision that is no longer the
// program's: the client edited something another client has since changed.
type ConflictError struct {
	Base    uint64
	Current *Head // nil: the program does not exist and base was not 0
}

func (e *ConflictError) Error() string {
	if e.Current == nil {
		return fmt.Sprintf("program does not exist on the cell; a save from revision %d needs revision 0", e.Base)
	}
	by := e.Current.UpdatedBy.Label
	if by == "" {
		by = "another client"
	}
	return fmt.Sprintf("the cell has revision %d of this program (saved by %s at %s); this edit was made from revision %d",
		e.Current.Revision, by, e.Current.UpdatedAt, e.Base)
}

// LockedError is a change refused because the robot is running the program
// (or, for the current program, running another one).
type LockedError struct{ Reason string }

func (e *LockedError) Error() string { return e.Reason }

// InvalidError is a document the store cannot file.
type InvalidError struct{ Reason string }

func (e *InvalidError) Error() string { return e.Reason }

// Executing names what the robot has loaded or is playing: the program_id a
// dense Load carried and the operator-facing reason, or "" for nothing.
type Executing func() (programRef, reason string)

type Store struct {
	dir      string
	id       string
	instance string
	lockFile *os.File
	now      func() time.Time

	mu        sync.Mutex
	heads     map[string]*Head
	current   *Current
	bytes     int64
	version   uint64
	changed   chan struct{}
	presence  map[string]*Presence
	executing Executing
}

type marker struct {
	Schema    string `json:"schema"`
	StoreID   string `json:"store_id"`
	CreatedAt string `json:"created_at"`
}

// Open opens (creating if needed) the store at dir and takes its lock. A
// second server on the same directory is refused, not interleaved.
func Open(dir string) (*Store, error) {
	if !filepath.IsAbs(dir) {
		return nil, fmt.Errorf("program store %q must be an absolute path", dir)
	}
	if err := os.MkdirAll(filepath.Join(dir, "programs"), 0o700); err != nil {
		return nil, fmt.Errorf("program store: %w", err)
	}
	lock, err := lockDir(filepath.Join(dir, "LOCK"))
	if err != nil {
		return nil, fmt.Errorf("program store %s: %w", dir, err)
	}
	s := &Store{dir: dir, lockFile: lock, now: time.Now, heads: map[string]*Head{}, presence: map[string]*Presence{},
		changed: make(chan struct{}), version: 1, instance: randomID()}
	if err := s.load(); err != nil {
		unlockDir(lock)
		return nil, err
	}
	return s, nil
}

// Close releases the store's lock.
func (s *Store) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.lockFile == nil {
		return nil
	}
	err := unlockDir(s.lockFile)
	s.lockFile = nil
	return err
}

func (s *Store) Dir() string      { return s.dir }
func (s *Store) ID() string       { return s.id }
func (s *Store) Instance() string { return s.instance }

// SetExecuting installs what the robot is running (the OLP server's dense
// execution); nil means nothing ever locks.
func (s *Store) SetExecuting(executing Executing) {
	s.mu.Lock()
	s.executing = executing
	s.mu.Unlock()
}

func (s *Store) load() error {
	markerPath := filepath.Join(s.dir, "STORE")
	raw, err := os.ReadFile(markerPath)
	switch {
	case errors.Is(err, os.ErrNotExist):
		m := marker{Schema: Schema, StoreID: "store-" + randomID(), CreatedAt: s.stamp()}
		data, _ := json.MarshalIndent(m, "", "  ")
		if err := atomicWrite(markerPath, append(data, '\n')); err != nil {
			return fmt.Errorf("program store marker: %w", err)
		}
		s.id = m.StoreID
	case err != nil:
		return fmt.Errorf("program store marker: %w", err)
	default:
		var m marker
		if json.Unmarshal(raw, &m) != nil || m.Schema != Schema || m.StoreID == "" {
			return fmt.Errorf("program store marker %s is not %s", markerPath, Schema)
		}
		s.id = m.StoreID
	}
	entries, err := os.ReadDir(filepath.Join(s.dir, "programs"))
	if err != nil {
		return fmt.Errorf("program store: %w", err)
	}
	for _, entry := range entries {
		if !entry.IsDir() || !ValidID(entry.Name()) {
			continue
		}
		raw, err := os.ReadFile(filepath.Join(s.dir, "programs", entry.Name(), "head.json"))
		if errors.Is(err, os.ErrNotExist) {
			// A crash before the first head was written: nothing was ever
			// acknowledged for this program.
			continue
		}
		var head Head
		if err != nil || json.Unmarshal(raw, &head) != nil || head.ProgramID != entry.Name() || head.Revision == 0 {
			return fmt.Errorf("program store: programs/%s/head.json is unreadable; move it aside to start without it", entry.Name())
		}
		s.heads[head.ProgramID] = &head
		for _, rev := range s.revisionFiles(head.ProgramID) {
			if info, err := os.Stat(s.revisionPath(head.ProgramID, rev)); err == nil {
				s.bytes += info.Size()
			}
		}
	}
	if raw, err := os.ReadFile(filepath.Join(s.dir, "state.json")); err == nil {
		var state struct {
			Current *Current `json:"current"`
		}
		if json.Unmarshal(raw, &state) == nil && state.Current != nil {
			if head := s.heads[state.Current.ProgramID]; head != nil && !head.Deleted {
				s.current = state.Current
			}
		}
	}
	return nil
}

func (s *Store) stamp() string { return s.now().UTC().Format(time.RFC3339Nano) }

func (s *Store) programDir(id string) string { return filepath.Join(s.dir, "programs", id) }

func (s *Store) revisionPath(id string, revision uint64) string {
	return filepath.Join(s.programDir(id), fmt.Sprintf("r%08d.json", revision))
}

func (s *Store) revisionFiles(id string) []uint64 {
	entries, _ := os.ReadDir(s.programDir(id))
	var out []uint64
	for _, entry := range entries {
		name := entry.Name()
		if !strings.HasPrefix(name, "r") || !strings.HasSuffix(name, ".json") {
			continue
		}
		if n, err := strconv.ParseUint(strings.TrimSuffix(strings.TrimPrefix(name, "r"), ".json"), 10, 64); err == nil {
			out = append(out, n)
		}
	}
	sort.Slice(out, func(i, j int) bool { return out[i] < out[j] })
	return out
}

// bump records a change and wakes every waiting change-feed request.
func (s *Store) bumpLocked() {
	s.version++
	close(s.changed)
	s.changed = make(chan struct{})
}

// SaveRequest is one client's save.
type SaveRequest struct {
	ProgramID string
	// BaseRevision is the store revision the document was edited from; 0
	// creates the program.
	BaseRevision uint64
	SaveID       string
	Document     []byte
	Client       Client
	MakeCurrent  bool
}

// SaveResult is the accepted revision. Replayed: the same save_id was already
// accepted (the client's first answer was lost) and nothing was written.
type SaveResult struct {
	Head     Head
	Replayed bool
	// Version identifies the store snapshot at or after this save.
	Version uint64
	// CurrentRefused is why MakeCurrent did not switch the cell's current
	// program (the robot runs another one); the save itself was accepted.
	CurrentRefused string
}

type documentIdentity struct {
	Schema   string `json:"schema"`
	Identity struct {
		ID       string `json:"id"`
		Name     string `json:"name"`
		Revision uint64 `json:"revision"`
	} `json:"identity"`
	Robot struct {
		Model       string `json:"model"`
		Description *struct {
			SHA256 string `json:"robotDescriptionSha256"`
		} `json:"description"`
	} `json:"robot"`
	Program *struct {
		ProgramID string `json:"programId"`
	} `json:"program"`
}

// parseDocument unwraps an export envelope and reads the identity the store
// files the program by. The project itself stays byte-for-byte as sent.
func parseDocument(raw []byte) ([]byte, documentIdentity, error) {
	var identity documentIdentity
	if len(raw) > MaxDocumentBytes {
		return nil, identity, ErrTooLarge
	}
	var envelope struct {
		Schema  string          `json:"schema"`
		Project json.RawMessage `json:"project"`
	}
	if err := json.Unmarshal(raw, &envelope); err != nil {
		return nil, identity, &InvalidError{"program document is not a JSON object"}
	}
	if envelope.Schema == exportSchema {
		if len(envelope.Project) == 0 {
			return nil, identity, &InvalidError{"project export has no project"}
		}
		raw = envelope.Project
	}
	if err := json.Unmarshal(raw, &identity); err != nil {
		return nil, identity, &InvalidError{"program document identity is unreadable: " + err.Error()}
	}
	switch {
	case identity.Schema != projectSchema:
		return nil, identity, &InvalidError{fmt.Sprintf("program document schema is %q, not %s", identity.Schema, projectSchema)}
	case strings.TrimSpace(identity.Identity.Name) == "":
		return nil, identity, &InvalidError{"program document has no identity.name"}
	case strings.TrimSpace(identity.Robot.Model) == "":
		return nil, identity, &InvalidError{"program document has no robot.model"}
	}
	return raw, identity, nil
}

// Save writes a new revision of a program if, and only if, the client edited
// the program's current revision.
func (s *Store) Save(request SaveRequest) (SaveResult, error) {
	if !ValidID(request.ProgramID) {
		return SaveResult{}, ErrInvalidID
	}
	document, identity, err := parseDocument(request.Document)
	if err != nil {
		return SaveResult{}, err
	}
	if identity.Identity.ID != request.ProgramID {
		return SaveResult{}, &InvalidError{fmt.Sprintf("document identity.id %q does not match program %q", identity.Identity.ID, request.ProgramID)}
	}
	ref := ""
	if identity.Program != nil {
		ref = identity.Program.ProgramID
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	head := s.heads[request.ProgramID]
	if head != nil && request.SaveID != "" && head.SaveID == request.SaveID && head.Revision == request.BaseRevision+1 {
		return SaveResult{Head: *head, Replayed: true, Version: s.version}, nil
	}
	if err := s.checkBaseLocked(head, request.BaseRevision); err != nil {
		s.auditLocked("save", request.ProgramID, request.BaseRevision, 0, request.Client, "conflict", nil)
		return SaveResult{}, err
	}
	if reason := s.lockedLocked(request.ProgramID, ref, head); reason != "" {
		s.auditLocked("save", request.ProgramID, request.BaseRevision, 0, request.Client, "executing", nil)
		return SaveResult{}, &LockedError{reason}
	}
	if (head == nil || head.Deleted) && s.liveLocked() >= MaxPrograms {
		return SaveResult{}, fmt.Errorf("%w: %d programs (MaxPrograms)", ErrFull, MaxPrograms)
	}
	if s.bytes+int64(len(document)) > MaxStoreBytes {
		return SaveResult{}, fmt.Errorf("%w: kept revisions reach %d bytes (MaxStoreBytes)", ErrFull, MaxStoreBytes)
	}
	now := s.stamp()
	sum := sha256.Sum256(document)
	next := Head{
		ProgramID: request.ProgramID, Revision: request.BaseRevision + 1, CreatedAt: now, UpdatedAt: now,
		UpdatedBy: request.Client, SaveID: request.SaveID, SHA256: hex.EncodeToString(sum[:]), Bytes: len(document),
		Name: identity.Identity.Name, RobotModel: identity.Robot.Model, ProgramRef: ref, ProjectRevision: identity.Identity.Revision,
	}
	if identity.Robot.Description != nil {
		next.DescriptionSHA256 = identity.Robot.Description.SHA256
	}
	if head != nil && !head.Deleted {
		next.CreatedAt = head.CreatedAt
	}
	if err := s.writeRevisionLocked(next, document); err != nil {
		return SaveResult{}, err
	}
	s.heads[next.ProgramID] = &next
	action := "save"
	if head == nil || head.Deleted {
		action = "create"
	}
	s.auditLocked(action, next.ProgramID, request.BaseRevision, next.Revision, request.Client, "ok", &next)
	result := SaveResult{Head: next}
	if request.MakeCurrent {
		// The revision is written either way; only the switch can be refused.
		if err := s.setCurrentLocked(next.ProgramID, request.Client); err != nil {
			result.CurrentRefused = err.Error()
		}
	}
	s.bumpLocked()
	result.Version = s.version
	return result, nil
}

func (s *Store) checkBaseLocked(head *Head, base uint64) error {
	current := uint64(0)
	if head != nil {
		current = head.Revision
	}
	if base == current {
		return nil
	}
	if head == nil {
		return &ConflictError{Base: base}
	}
	copy := *head
	return &ConflictError{Base: base, Current: &copy}
}

// lockedLocked is the reason a change to this program is refused now: the
// robot has it loaded or playing.
func (s *Store) lockedLocked(id, ref string, head *Head) string {
	if s.executing == nil {
		return ""
	}
	running, reason := s.executing()
	if running == "" {
		return ""
	}
	if running == id || (ref != "" && running == ref) || (head != nil && head.ProgramRef != "" && running == head.ProgramRef) {
		return reason
	}
	return ""
}

func (s *Store) liveLocked() int {
	n := 0
	for _, head := range s.heads {
		if !head.Deleted {
			n++
		}
	}
	return n
}

type revisionFile struct {
	Schema   string          `json:"schema"`
	Meta     Head            `json:"meta"`
	Document json.RawMessage `json:"document,omitempty"`
}

func (s *Store) writeRevisionLocked(head Head, document []byte) error {
	if err := os.MkdirAll(s.programDir(head.ProgramID), 0o700); err != nil {
		return err
	}
	data, err := json.Marshal(revisionFile{Schema: RevisionSchema, Meta: head, Document: document})
	if err != nil {
		return err
	}
	path := s.revisionPath(head.ProgramID, head.Revision)
	var replaced int64
	if info, err := os.Stat(path); err == nil {
		// An orphan from a crash between this file and its head: never
		// acknowledged, so it is overwritten.
		replaced = info.Size()
	}
	if err := atomicWrite(path, data); err != nil {
		return fmt.Errorf("program store write: %w", err)
	}
	s.bytes += int64(len(data)) - replaced
	meta, _ := json.MarshalIndent(head, "", "  ")
	if err := atomicWrite(filepath.Join(s.programDir(head.ProgramID), "head.json"), append(meta, '\n')); err != nil {
		return fmt.Errorf("program store head: %w", err)
	}
	for _, rev := range s.revisionFiles(head.ProgramID) {
		if rev+KeepRevisions > head.Revision {
			break
		}
		old := s.revisionPath(head.ProgramID, rev)
		if info, err := os.Stat(old); err == nil && os.Remove(old) == nil {
			s.bytes -= info.Size()
		}
	}
	return nil
}

// Get reads a program at a revision (0: its latest). A deleted program's
// latest revision is a tombstone and answers ErrNotFound.
func (s *Store) Get(id string, revision uint64) (Head, []byte, error) {
	if !ValidID(id) {
		return Head{}, nil, ErrInvalidID
	}
	s.mu.Lock()
	head := s.heads[id]
	s.mu.Unlock()
	if head == nil || (revision == 0 && head.Deleted) {
		return Head{}, nil, ErrNotFound
	}
	if revision == 0 {
		revision = head.Revision
	}
	raw, err := os.ReadFile(s.revisionPath(id, revision))
	if err != nil {
		return Head{}, nil, ErrNotFound
	}
	var file revisionFile
	if json.Unmarshal(raw, &file) != nil || file.Meta.Revision != revision || file.Meta.Deleted {
		return Head{}, nil, ErrNotFound
	}
	return file.Meta, file.Document, nil
}

// Revisions lists the kept revision numbers of a program, oldest first.
func (s *Store) Revisions(id string) ([]uint64, error) {
	if !ValidID(id) {
		return nil, ErrInvalidID
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.heads[id] == nil {
		return nil, ErrNotFound
	}
	return s.revisionFiles(id), nil
}

// Delete writes a tombstone revision; the kept revisions stay on disk until
// pruned like any other, and a save from the tombstone's revision restores it.
func (s *Store) Delete(id string, base uint64, client Client) (Head, error) {
	if !ValidID(id) {
		return Head{}, ErrInvalidID
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	head := s.heads[id]
	if head == nil || head.Deleted {
		return Head{}, ErrNotFound
	}
	if err := s.checkBaseLocked(head, base); err != nil {
		s.auditLocked("delete", id, base, 0, client, "conflict", nil)
		return Head{}, err
	}
	if reason := s.lockedLocked(id, head.ProgramRef, head); reason != "" {
		s.auditLocked("delete", id, base, 0, client, "executing", nil)
		return Head{}, &LockedError{reason}
	}
	now := s.stamp()
	tomb := Head{ProgramID: id, Revision: head.Revision + 1, Deleted: true, CreatedAt: head.CreatedAt, UpdatedAt: now,
		UpdatedBy: client, Name: head.Name, RobotModel: head.RobotModel, DescriptionSHA256: head.DescriptionSHA256, ProgramRef: head.ProgramRef}
	if err := s.writeRevisionLocked(tomb, nil); err != nil {
		return Head{}, err
	}
	s.heads[id] = &tomb
	if s.current != nil && s.current.ProgramID == id {
		s.current = nil
		_ = s.writeStateLocked()
	}
	s.auditLocked("delete", id, base, tomb.Revision, client, "ok", &tomb)
	s.bumpLocked()
	return tomb, nil
}

// SetCurrent makes a program the cell's current one. Refused while the robot
// runs a different program: the Deck follows the current program, and must
// not be switched away from the one it is running.
func (s *Store) SetCurrent(id string, client Client) (Current, error) {
	if !ValidID(id) {
		return Current{}, ErrInvalidID
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.setCurrentLocked(id, client); err != nil {
		return Current{}, err
	}
	s.bumpLocked()
	return *s.current, nil
}

func (s *Store) setCurrentLocked(id string, client Client) error {
	head := s.heads[id]
	if head == nil || head.Deleted {
		return ErrNotFound
	}
	if s.current != nil && s.current.ProgramID == id {
		return nil
	}
	if s.executing != nil {
		if running, reason := s.executing(); running != "" && running != id && running != head.ProgramRef {
			s.auditLocked("current", id, 0, 0, client, "executing", nil)
			return &LockedError{reason + "; the cell's current program changes after Stop"}
		}
	}
	s.current = &Current{ProgramID: id, SetAt: s.stamp(), SetBy: client}
	if err := s.writeStateLocked(); err != nil {
		return err
	}
	s.auditLocked("current", id, 0, head.Revision, client, "ok", nil)
	return nil
}

func (s *Store) writeStateLocked() error {
	data, _ := json.MarshalIndent(map[string]any{"schema": Schema, "current": s.current}, "", "  ")
	return atomicWrite(filepath.Join(s.dir, "state.json"), append(data, '\n'))
}

// Touch records a change-feed request: the client watches a program (or none)
// and says whether it is editing it. A change in who watches or edits wakes
// the feed for everyone else. It returns the version before and after, so the
// caller's own wait is not woken by its own presence.
func (s *Store) Touch(key string, client Client, watch string, editing bool) (before, after uint64) {
	if key == "" {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	before = s.version
	defer func() { after = s.version }()
	now := s.now()
	p := s.presence[key]
	if watch == "" {
		if p != nil {
			delete(s.presence, key)
			s.bumpLocked()
		}
		return
	}
	changed := p == nil || p.ProgramID != watch || p.Editing != editing || p.Label != client.Label
	if p == nil {
		p = &Presence{key: key}
		s.presence[key] = p
	}
	p.ProgramID, p.Client, p.Editing, p.seen = watch, client, editing, now
	p.LastSeen = now.UTC().Format(time.RFC3339)
	if changed {
		s.bumpLocked()
	}
	return
}

// prunePresenceLocked drops clients that stopped asking.
func (s *Store) prunePresenceLocked() {
	now := s.now()
	for key, p := range s.presence {
		if now.Sub(p.seen) > PresenceTTL {
			delete(s.presence, key)
			s.bumpLocked()
		}
	}
}

// Snapshot is the store as a client sees it.
type Snapshot struct {
	StoreID  string     `json:"store_id"`
	Instance string     `json:"instance"`
	Version  uint64     `json:"version"`
	Current  *Current   `json:"current"`
	Programs []Head     `json:"programs"`
	Presence []Presence `json:"presence"`
	// Executing is what the robot runs: the store program it maps to (if
	// any), the program_id dense execution loaded, and the reason edits wait.
	Executing *ExecutingView `json:"executing"`
}

type ExecutingView struct {
	ProgramID  string `json:"program_id,omitempty"`
	ProgramRef string `json:"program_ref"`
	Reason     string `json:"reason"`
}

// Snapshot reads the whole store for a list or a change-feed answer.
func (s *Store) Snapshot() Snapshot {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.prunePresenceLocked()
	out := Snapshot{StoreID: s.id, Instance: s.instance, Version: s.version, Programs: []Head{}, Presence: []Presence{}}
	if s.current != nil {
		c := *s.current
		out.Current = &c
	}
	for _, head := range s.heads {
		if !head.Deleted {
			out.Programs = append(out.Programs, *head)
		}
	}
	sort.Slice(out.Programs, func(i, j int) bool {
		if out.Programs[i].UpdatedAt != out.Programs[j].UpdatedAt {
			return out.Programs[i].UpdatedAt > out.Programs[j].UpdatedAt
		}
		return out.Programs[i].ProgramID < out.Programs[j].ProgramID
	})
	for _, p := range s.presence {
		out.Presence = append(out.Presence, *p)
	}
	sort.Slice(out.Presence, func(i, j int) bool { return out.Presence[i].key < out.Presence[j].key })
	if s.executing != nil {
		if ref, reason := s.executing(); ref != "" {
			view := &ExecutingView{ProgramRef: ref, Reason: reason}
			for _, head := range s.heads {
				if !head.Deleted && (head.ProgramID == ref || head.ProgramRef == ref) {
					view.ProgramID = head.ProgramID
					break
				}
			}
			out.Executing = view
		}
	}
	return out
}

// PresenceKey is the store-internal key of a presence entry, for marking the
// asking client.
func (p Presence) PresenceKey() string { return p.key }

// Notify wakes the change feed for a change the store does not see itself:
// the robot loading a program or releasing one.
func (s *Store) Notify() {
	s.mu.Lock()
	s.bumpLocked()
	s.mu.Unlock()
}

// Head returns a program's latest metadata, deleted or not.
func (s *Store) Head(id string) (Head, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	head := s.heads[id]
	if head == nil {
		return Head{}, false
	}
	return *head, true
}

// Wait blocks until the store changes after `since`, `stale` reports true
// (checked once a second: what the robot runs is not a store event), the
// timeout passes or ctx ends. It returns the version it saw.
func (s *Store) Wait(ctx context.Context, since uint64, timeout time.Duration, stale func() bool) uint64 {
	if timeout > WaitLimit {
		timeout = WaitLimit
	}
	deadline := time.NewTimer(timeout)
	defer deadline.Stop()
	tick := time.NewTicker(time.Second)
	defer tick.Stop()
	for {
		s.mu.Lock()
		s.prunePresenceLocked()
		version, changed := s.version, s.changed
		s.mu.Unlock()
		if version != since || (stale != nil && stale()) {
			return version
		}
		select {
		case <-changed:
		case <-tick.C:
		case <-deadline.C:
			return version
		case <-ctx.Done():
			return version
		}
	}
}

type auditEntry struct {
	At           string `json:"at"`
	Action       string `json:"action"`
	Result       string `json:"result"`
	ProgramID    string `json:"program_id"`
	BaseRevision uint64 `json:"base_revision"`
	Revision     uint64 `json:"revision,omitempty"`
	Client       Client `json:"client"`
	Name         string `json:"name,omitempty"`
	SHA256       string `json:"sha256,omitempty"`
	Bytes        int    `json:"bytes,omitempty"`
}

// auditLocked appends one line to audit.jsonl. Auditing never refuses a
// change: a write failure is the audit's loss, reported on stderr.
func (s *Store) auditLocked(action, id string, base, revision uint64, client Client, result string, head *Head) {
	entry := auditEntry{At: s.stamp(), Action: action, Result: result, ProgramID: id, BaseRevision: base, Revision: revision, Client: client}
	if head != nil {
		entry.Name, entry.SHA256, entry.Bytes = head.Name, head.SHA256, head.Bytes
	}
	line, _ := json.Marshal(entry)
	path := filepath.Join(s.dir, "audit.jsonl")
	if info, err := os.Stat(path); err == nil && info.Size() > auditRotateBytes {
		_ = os.Rename(path, path+".1")
	}
	f, err := os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o600)
	if err != nil {
		fmt.Fprintf(os.Stderr, "program store audit: %v\n", err)
		return
	}
	defer f.Close()
	if _, err := f.Write(append(line, '\n')); err != nil {
		fmt.Fprintf(os.Stderr, "program store audit: %v\n", err)
	}
}

// atomicWrite replaces path with data so that a crash leaves the old file or
// the new one, never a torn one.
func atomicWrite(path string, data []byte) error {
	dir := filepath.Dir(path)
	tmp, err := os.CreateTemp(dir, "."+filepath.Base(path)+".tmp-*")
	if err != nil {
		return err
	}
	name := tmp.Name()
	_, werr := tmp.Write(data)
	if werr == nil {
		werr = tmp.Sync()
	}
	if cerr := tmp.Close(); werr == nil {
		werr = cerr
	}
	if werr == nil {
		werr = os.Chmod(name, 0o600)
	}
	if werr == nil {
		werr = os.Rename(name, path)
	}
	if werr != nil {
		os.Remove(name)
		return werr
	}
	if d, err := os.Open(dir); err == nil {
		_ = d.Sync()
		d.Close()
	}
	return nil
}

func randomID() string {
	b := make([]byte, 8)
	_, _ = rand.Read(b)
	return hex.EncodeToString(b)
}
