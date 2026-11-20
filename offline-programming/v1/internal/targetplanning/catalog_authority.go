package targetplanning

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"

	"rosieos/offline-programming/v1/internal/catalog"
)

// CatalogProgramAuthority is the server-only read boundary for the shared
// Robot V4 program. It resolves the exact immutable revision selected for
// physical planning from the Motion Server-owned catalog.
type CatalogProgramAuthority struct {
	gateway catalog.Gateway
}

func NewCatalogProgramAuthority(
	gateway catalog.Gateway,
) (*CatalogProgramAuthority, error) {
	if gateway == nil || !gateway.Available() {
		return nil, errors.New(
			"target planning: Motion Server program catalog is unavailable",
		)
	}
	return &CatalogProgramAuthority{gateway: gateway}, nil
}

func (authority *CatalogProgramAuthority) SavedProgram(
	ctx context.Context,
	programID string,
	revision uint64,
) (json.RawMessage, error) {
	if authority == nil || authority.gateway == nil ||
		!authority.gateway.Available() {
		return nil, ErrUnavailable
	}
	response, err := authority.gateway.Get(ctx, programID, &revision)
	if err != nil {
		return nil, fmt.Errorf("%w: %v", ErrUnavailable, err)
	}
	if response.Status == http.StatusNotFound {
		return nil, ErrProgramNotSaved
	}
	if response.Status != http.StatusOK {
		return nil, fmt.Errorf(
			"%w: program catalog returned HTTP %d",
			ErrUnavailable,
			response.Status,
		)
	}
	var entry struct {
		Schema  string          `json:"schema"`
		Owner   string          `json:"owner"`
		Program json.RawMessage `json:"program"`
	}
	if err := json.Unmarshal(response.Body, &entry); err != nil ||
		entry.Schema != "robot.v4.program-entry.v1" ||
		entry.Owner != "motion-server" ||
		len(entry.Program) == 0 {
		return nil, fmt.Errorf(
			"%w: catalog program entry is invalid",
			ErrUnavailable,
		)
	}
	var identity struct {
		ProgramID       string `json:"program_id"`
		ProgramRevision uint64 `json:"program_revision"`
	}
	if err := json.Unmarshal(entry.Program, &identity); err != nil ||
		identity.ProgramID != programID ||
		identity.ProgramRevision != revision {
		return nil, ErrProgramNotSaved
	}
	return append(json.RawMessage(nil), entry.Program...), nil
}
