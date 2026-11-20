package denseexec

import "context"

// CellSpec exposes commissioned address metadata, never credentials or mutable
// model/configuration pins. Add accepts only a matching server-side binding.
type CellSpec struct {
	ID                  string `json:"id"`
	Label               string `json:"label"`
	Address             string `json:"address"`
	Host                string `json:"host"`
	CredentialRef       string `json:"credential_ref"`
	ModelID             string `json:"model_id"`
	Backend             string `json:"backend"`
	ConfigurationDigest string `json:"configuration_digest"`
	Source              string `json:"source"`
}
type CellProbe struct {
	CellSpec
	Reachable bool   `json:"reachable"`
	Error     string `json:"error,omitempty"`
}
type CellsBody struct {
	Cells      []CellProbe `json:"cells"`
	ActiveCell string      `json:"active_cell"`
}

// CellDirectory is implemented by the native Picker. The facade has no
// selection, controller factory, endpoint store or independent authority.
type CellDirectory interface {
	RegisteredCells() []CellSpec
	DescribeCells(context.Context) CellsBody
	RegisterCell(CellSpec) error
	SelectCell(context.Context, string) error
	SelectedCell() string
}

type Registry struct{ directory CellDirectory }

func NewRegistry(directory CellDirectory) *Registry { return &Registry{directory: directory} }
func (r *Registry) List() []CellSpec {
	if r.directory == nil {
		return []CellSpec{}
	}
	return r.directory.RegisteredCells()
}
func (r *Registry) Add(spec CellSpec) error {
	if r.directory == nil {
		return httpError(409, "cell_not_commissioned", "not a commissioned cell")
	}
	return r.directory.RegisterCell(spec)
}

// Remove refuses GUI decommissioning, preventing loss of commissioned pins.
func (r *Registry) Remove(id string) error {
	return httpError(409, "cell_commissioning_required", "commissioned cells cannot be removed here; disconnect to work offline")
}
