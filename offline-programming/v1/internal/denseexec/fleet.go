package denseexec

import "context"

// Fleet serves main's Cells dialog through the same Picker used by motion
// admission. It cannot construct, retain or select a second controller.
type Fleet struct {
	backend   Backend
	directory CellDirectory
	registry  *Registry
}

func NewFleet(backend Backend) *Fleet {
	directory, _ := backend.(CellDirectory)
	return &Fleet{backend: backend, directory: directory, registry: NewRegistry(directory)}
}
func (f *Fleet) Registry() *Registry { return f.registry }
func (f *Fleet) ActiveID() string {
	if f.directory == nil {
		return ""
	}
	return f.directory.SelectedCell()
}
func (f *Fleet) Select(ctx context.Context, id string) error {
	if f.directory == nil {
		return httpError(409, "cell_catalogue_unavailable", "configure a commissioned cell catalogue")
	}
	return f.directory.SelectCell(ctx, id)
}
func (f *Fleet) Cells(ctx context.Context) CellsBody {
	if f.directory == nil {
		return CellsBody{Cells: []CellProbe{}}
	}
	return f.directory.DescribeCells(ctx)
}
func (f *Fleet) Capabilities(ctx context.Context) Capabilities {
	caps := Capabilities{Backend: "rt_core", Reason: "cell_not_selected"}
	if f.backend != nil {
		caps = f.backend.Capabilities(ctx)
	}
	caps.ActiveCell, caps.Cells = f.ActiveID(), f.registry.List()
	return caps
}
