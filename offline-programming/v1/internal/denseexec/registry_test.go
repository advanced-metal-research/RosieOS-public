package denseexec

import (
	"context"
	"errors"
	"testing"
)

type facadeDirectory struct {
	Backend
	cells                                []CellSpec
	selected                             string
	describes, selections, registrations int
	refusal                              error
}

func (d *facadeDirectory) RegisteredCells() []CellSpec { return d.cells }
func (d *facadeDirectory) DescribeCells(context.Context) CellsBody {
	d.describes++
	return CellsBody{Cells: []CellProbe{{CellSpec: d.cells[0], Reachable: false, Error: "rt_core_identity_mismatch"}}, ActiveCell: d.selected}
}
func (d *facadeDirectory) RegisterCell(CellSpec) error { d.registrations++; return d.refusal }
func (d *facadeDirectory) SelectCell(_ context.Context, id string) error {
	d.selections++
	if d.refusal != nil {
		return d.refusal
	}
	d.selected = id
	return nil
}
func (d *facadeDirectory) SelectedCell() string { return d.selected }
func (d *facadeDirectory) Capabilities(context.Context) Capabilities {
	return Capabilities{Backend: "rt_core"}
}

// TestFleetDelegatesWithoutAuthority ports the registry contract to native
// commissioning and proves the facade neither caches nor bypasses refusals.
func TestFleetDelegatesWithoutAuthority(t *testing.T) {
	ctx := context.Background()
	d := &facadeDirectory{cells: []CellSpec{{ID: "local-simulation"}}}
	f := NewFleet(d)
	if len(f.Registry().List()) != 1 || d.describes != 0 {
		t.Fatal("metadata performed discovery")
	}
	body := f.Cells(ctx)
	if d.describes != 1 || body.Cells[0].Reachable || body.Cells[0].Error != "rt_core_identity_mismatch" {
		t.Fatalf("native discovery: %+v", body)
	}
	d.refusal = errors.New("cell_configuration_mismatch")
	if f.Select(ctx, "local-simulation") != d.refusal || f.ActiveID() != "" {
		t.Fatal("selection refusal bypassed")
	}
	if f.Registry().Add(CellSpec{ID: "unknown"}) != d.refusal || d.registrations != 1 {
		t.Fatal("registration refusal bypassed")
	}
	d.refusal = nil
	if err := f.Select(ctx, "local-simulation"); err != nil {
		t.Fatal(err)
	}
	if f.Capabilities(ctx).ActiveCell != "local-simulation" {
		t.Fatal("selection not shared")
	}
	if err := f.Select(ctx, ""); err != nil || d.selections != 3 || f.ActiveID() != "" {
		t.Fatal("clear not delegated")
	}
	if err := f.Registry().Remove("local-simulation"); err == nil {
		t.Fatal("GUI decommissioning accepted")
	}
}

func TestFleetUnconfiguredFailsClosed(t *testing.T) {
	f := NewFleet(nil)
	if f.Capabilities(context.Background()).Available || len(f.Cells(context.Background()).Cells) != 0 {
		t.Fatal("unconfigured available")
	}
	if f.Select(context.Background(), "unknown") == nil || f.Registry().Add(CellSpec{}) == nil {
		t.Fatal("unconfigured mutation accepted")
	}
}
