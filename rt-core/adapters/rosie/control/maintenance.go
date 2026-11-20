//go:build linux

package control

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"os"
	"strings"
	"sync"
	"time"
	"unicode/utf8"
)

// The cell's maintenance notice. While an update replaces or reconfigures this
// controller, the deploy tooling writes the notice file on the servo host
// (root, 0644) and removes it when it ends. While it is active, operations that
// take authority or start motion (Acquire, Arm, Enable, Home, anchor and
// reference restores, fault reset, jogs, trajectory and program starts, IoArm)
// are refused with the catalogued reason "busy", wrapping the notice's own
// reason text. Stop, Release, Halt, EndJog, Renew, IoDisarm, Discard and
// Prepare always pass: a notice never stops, disarms or retires a session by
// itself.
//
// GET /v1/maintenance serves the same state the guard enforces, with the grant
// holder's controller name and principal (never the session token), so an OLP
// whose own host carries no notice (a Steam Deck running OLP locally) still
// blocks its operators while the servo host is being updated.

// DefaultMaintenanceFile is under the installer's 0755 /etc/rosie-rt-core, which
// rt-control (rosie-ctl) can traverse.
const DefaultMaintenanceFile = "/etc/rosie-rt-core/maintenance.json"

const (
	maintenanceFileSchema   = "rosie.rt-control.maintenance.v1"
	maintenanceFileMaxBytes = 64 << 10
	// The OLP notice's display bound (offline-programming maintenance.go).
	maintenanceTextMaxBytes = 400
)

// maintenanceFile is the deploy tool's notice (steamdeck/real/v4
// rt_core_maintenance.go writes the relay's notice with this schema).
type maintenanceFile struct {
	Schema    string `json:"schema,omitempty"`
	Active    *bool  `json:"active,omitempty"`
	Reason    string `json:"reason,omitempty"`
	Phase     string `json:"phase,omitempty"`
	StartedAt string `json:"started_at"`
	ETA       string `json:"eta,omitempty"`
	SetBy     string `json:"set_by,omitempty"`
	Release   string `json:"release,omitempty"`
}

// maintenanceState is what the guard needs, whether the notice is active and
// its reason, plus what GET /v1/maintenance shows. Malformed: the file exists
// but is not a notice (still active).
type maintenanceState struct {
	Active, Malformed                             bool
	Reason, Phase, StartedAt, ETA, SetBy, Release string
}

// maintenanceSource stats the file per check and parses it only when its size
// or modification time changes.
type maintenanceSource struct {
	path    string
	mu      sync.Mutex
	known   bool
	size    int64
	modTime time.Time
	state   maintenanceState
}

// SetMaintenanceFile enables the notice; empty disables it (library and test
// controllers have none).
func (c *Controller) SetMaintenanceFile(path string) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if path == "" {
		c.maintenance = nil
		return
	}
	c.maintenance = &maintenanceSource{path: path}
}

func (s *maintenanceSource) current() maintenanceState {
	s.mu.Lock()
	defer s.mu.Unlock()
	info, err := os.Stat(s.path)
	if errors.Is(err, os.ErrNotExist) {
		s.known, s.state = false, maintenanceState{}
		return s.state
	}
	if err != nil {
		// Present but unreadable: an update is in progress; fail closed.
		s.known = false
		return maintenanceState{Active: true, Malformed: true}
	}
	if s.known && info.Size() == s.size && info.ModTime().Equal(s.modTime) {
		return s.state
	}
	s.known, s.size, s.modTime = true, info.Size(), info.ModTime()
	s.state = readMaintenanceFile(s.path)
	return s.state
}

// readMaintenanceFile applies the OLP notice contract: no file is inactive,
// active=false is inactive, anything that is not a notice is active and
// malformed. An overrun or stale notice stays active; nothing here clears it.
func readMaintenanceFile(path string) maintenanceState {
	file, err := os.Open(path)
	if errors.Is(err, os.ErrNotExist) {
		return maintenanceState{}
	}
	if err != nil {
		return maintenanceState{Active: true, Malformed: true}
	}
	defer file.Close()
	raw, err := io.ReadAll(io.LimitReader(file, maintenanceFileMaxBytes+1))
	var notice maintenanceFile
	if err != nil || len(raw) > maintenanceFileMaxBytes || json.Unmarshal(raw, &notice) != nil {
		return maintenanceState{Active: true, Malformed: true}
	}
	if notice.Active != nil && !*notice.Active {
		return maintenanceState{}
	}
	started, timeErr := time.Parse(time.RFC3339, strings.TrimSpace(notice.StartedAt))
	malformed := timeErr != nil || (notice.Schema != "" && notice.Schema != maintenanceFileSchema)
	// The reason is the deploy's own text, never a default here: it is the
	// cause a refusal carries, and every adapter-authored reason is catalogued.
	state := maintenanceState{Active: true, Malformed: malformed, Reason: strings.TrimSpace(notice.Reason),
		Phase: strings.TrimSpace(notice.Phase), SetBy: strings.TrimSpace(notice.SetBy), Release: strings.TrimSpace(notice.Release)}
	if timeErr == nil {
		state.StartedAt = started.UTC().Format(time.RFC3339)
	}
	if eta, err := time.Parse(time.RFC3339, strings.TrimSpace(notice.ETA)); err == nil {
		state.ETA = eta.UTC().Format(time.RFC3339)
	}
	return state
}

// maintenanceRefuses names the operations an active notice refuses.
func maintenanceRefuses(operation string) bool {
	switch operation {
	case OperationAcquire, OperationArm, OperationEnable, OperationHome, OperationRestoreAnchor, OperationRestoreReferenceVerifiedDelta,
		OperationResetFault, OperationJog, OperationBeginJog, OperationBeginPathJog, OperationStartTrajectory, OperationStartProgram, OperationIoArm, OperationIoTest:
		return true
	}
	return false
}

// checkMaintenance returns the refusal for operation while a notice is active:
// "busy", wrapping the notice's reason text when it has one, as busy already
// wraps native diagnostics.
func (c *Controller) checkMaintenance(operation string) error {
	if !maintenanceRefuses(operation) {
		return nil
	}
	c.mu.Lock()
	source := c.maintenance
	c.mu.Unlock()
	if source == nil {
		return nil
	}
	state := source.current()
	if !state.Active {
		return nil
	}
	if state.Reason == "" {
		return errors.New(ReasonBusy)
	}
	return fmt.Errorf("%s: %w", ReasonBusy, errors.New(state.Reason))
}

// Maintenance is GET /v1/maintenance: the notice exactly as checkMaintenance
// sees it now and the current grant's holder. It reads no session and takes
// no authority, so it answers while a grant is held and during a notice.
func (c *Controller) Maintenance() MaintenanceObservation {
	c.mu.Lock()
	source := c.maintenance
	var holder *MaintenanceHolder
	if c.grant.Session != "" {
		record := c.sessions[c.grant.Session]
		holder = &MaintenanceHolder{Controller: c.grant.Controller, Remote: record.principal.ID != "", Identity: record.principal.ID,
			Generation: c.grant.Generation}
		if !record.granted.IsZero() {
			holder.Since = record.granted.UTC().Format(time.RFC3339)
		}
	}
	c.mu.Unlock()
	observation := MaintenanceObservation{Holder: holder}
	if source == nil {
		return observation
	}
	state := source.current()
	if !state.Active {
		return observation
	}
	observation.Active, observation.Malformed = true, state.Malformed
	observation.Reason, observation.Phase = clipMaintenanceText(state.Reason), clipMaintenanceText(state.Phase)
	observation.StartedAt, observation.ETA = state.StartedAt, state.ETA
	observation.SetBy, observation.Release = clipMaintenanceText(state.SetBy), clipMaintenanceText(state.Release)
	return observation
}

// clipMaintenanceText bounds operator text without splitting a UTF-8 sequence.
func clipMaintenanceText(text string) string {
	if len(text) <= maintenanceTextMaxBytes {
		return text
	}
	cut := maintenanceTextMaxBytes - len("…")
	for cut > 0 && !utf8.RuneStart(text[cut]) {
		cut--
	}
	return text[:cut] + "…"
}

// logGrantLocked names a grant's holder in the journal when it changes hands,
// so a deployer waiting for the cell can see who to ask (renewals are not
// logged). Controller is the caller-chosen name; the principal is "local" for
// the Unix socket or the remote certificate identity.
func (c *Controller) logGrantLocked(kind string) {
	if kind == "grant_renewed" {
		return
	}
	principal := "local"
	if record, ok := c.sessions[c.grant.Session]; ok && record.principal.ID != "" {
		principal = "remote:" + record.principal.ID
	}
	log.Printf("rt-control: %s generation=%d controller=%q principal=%s lease_ms=%d", kind, c.grant.Generation, c.grant.Controller, principal, c.grant.LeaseMS)
}
