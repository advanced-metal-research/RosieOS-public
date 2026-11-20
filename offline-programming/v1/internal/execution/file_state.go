package execution

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
)

const idempotencyStateSchema = "offline-programming.execution-idempotency.v1"

// JSONLAuditSink is an append-only, fsync-before-success execution audit.
// The gateway therefore cannot report review or command success before the
// corresponding audit record is durable on the Offline Programming host.
type JSONLAuditSink struct {
	mu   sync.Mutex
	path string
}

func NewJSONLAuditSink(path string) (*JSONLAuditSink, error) {
	path = strings.TrimSpace(path)
	if path == "" || !filepath.IsAbs(path) {
		return nil, errors.New("execution: audit path must be absolute")
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o700); err != nil {
		return nil, fmt.Errorf("execution: create audit directory: %w", err)
	}
	return &JSONLAuditSink{path: path}, nil
}

func (sink *JSONLAuditSink) Append(
	ctx context.Context,
	envelope GatewayAuditEnvelope,
) error {
	if sink == nil || sink.path == "" {
		return errors.New("execution: audit sink is unavailable")
	}
	select {
	case <-ctx.Done():
		return ctx.Err()
	default:
	}
	raw, err := json.Marshal(envelope)
	if err != nil {
		return err
	}
	raw = append(raw, '\n')
	sink.mu.Lock()
	defer sink.mu.Unlock()
	file, err := os.OpenFile(
		sink.path,
		os.O_CREATE|os.O_APPEND|os.O_WRONLY,
		0o600,
	)
	if err != nil {
		return err
	}
	_, writeErr := file.Write(raw)
	syncErr := file.Sync()
	closeErr := file.Close()
	return errors.Join(writeErr, syncErr, closeErr)
}

type fileIdempotencyState struct {
	Schema    string          `json:"schema"`
	Committed map[string]bool `json:"committed"`
}

// FileIdempotencyStore persists committed identities. Reservations are
// process-local and intentionally disappear after a crash; a command is only
// durable after its exact application receipt has been validated.
type FileIdempotencyStore struct {
	mu        sync.Mutex
	path      string
	reserved  map[string]struct{}
	committed map[string]struct{}
}

func NewFileIdempotencyStore(path string) (*FileIdempotencyStore, error) {
	path = strings.TrimSpace(path)
	if path == "" || !filepath.IsAbs(path) {
		return nil, errors.New(
			"execution: idempotency state path must be absolute",
		)
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o700); err != nil {
		return nil, fmt.Errorf(
			"execution: create idempotency directory: %w",
			err,
		)
	}
	store := &FileIdempotencyStore{
		path:      path,
		reserved:  make(map[string]struct{}),
		committed: make(map[string]struct{}),
	}
	raw, err := os.ReadFile(path)
	if errors.Is(err, os.ErrNotExist) {
		return store, nil
	}
	if err != nil {
		return nil, fmt.Errorf("execution: read idempotency state: %w", err)
	}
	var state fileIdempotencyState
	if json.Unmarshal(raw, &state) != nil ||
		state.Schema != idempotencyStateSchema ||
		state.Committed == nil {
		return nil, errors.New("execution: idempotency state is invalid")
	}
	for key, committed := range state.Committed {
		if !committed || strings.TrimSpace(key) == "" {
			return nil, errors.New("execution: idempotency state is invalid")
		}
		store.committed[key] = struct{}{}
	}
	return store, nil
}

func (store *FileIdempotencyStore) Reserve(
	ctx context.Context,
	key string,
) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	key = strings.TrimSpace(key)
	if store == nil || key == "" {
		return false, errors.New("execution: invalid idempotency reservation")
	}
	store.mu.Lock()
	defer store.mu.Unlock()
	if _, ok := store.committed[key]; ok {
		return true, nil
	}
	if _, ok := store.reserved[key]; ok {
		return false, errors.New("execution: idempotency key is already reserved")
	}
	store.reserved[key] = struct{}{}
	return false, nil
}

func (store *FileIdempotencyStore) Commit(
	ctx context.Context,
	key string,
) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	store.mu.Lock()
	defer store.mu.Unlock()
	if _, ok := store.reserved[key]; !ok {
		return errors.New("execution: idempotency key was not reserved")
	}
	delete(store.reserved, key)
	store.committed[key] = struct{}{}
	if err := store.persistLocked(); err != nil {
		delete(store.committed, key)
		store.reserved[key] = struct{}{}
		return err
	}
	return nil
}

func (store *FileIdempotencyStore) Abort(
	_ context.Context,
	key string,
) error {
	if store == nil {
		return nil
	}
	store.mu.Lock()
	defer store.mu.Unlock()
	delete(store.reserved, key)
	return nil
}

func (store *FileIdempotencyStore) persistLocked() error {
	state := fileIdempotencyState{
		Schema:    idempotencyStateSchema,
		Committed: make(map[string]bool, len(store.committed)),
	}
	for key := range store.committed {
		state.Committed[key] = true
	}
	raw, err := json.Marshal(state)
	if err != nil {
		return err
	}
	raw = append(raw, '\n')
	temp, err := os.CreateTemp(filepath.Dir(store.path), ".execution-idempotency-*")
	if err != nil {
		return err
	}
	tempPath := temp.Name()
	removeTemp := true
	defer func() {
		if removeTemp {
			_ = os.Remove(tempPath)
		}
	}()
	if err := temp.Chmod(0o600); err != nil {
		_ = temp.Close()
		return err
	}
	if _, err := temp.Write(raw); err != nil {
		_ = temp.Close()
		return err
	}
	if err := temp.Sync(); err != nil {
		_ = temp.Close()
		return err
	}
	if err := temp.Close(); err != nil {
		return err
	}
	if err := os.Rename(tempPath, store.path); err != nil {
		return err
	}
	removeTemp = false
	return nil
}
