//go:build linux

package targets

import (
	"context"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

type pickerControlStatusBackend struct {
	*pickerBackend
	body          *denseexec.StatusBody
	full, compact int
}

func (b *pickerControlStatusBackend) Status(context.Context) *denseexec.StatusBody {
	b.full++
	return b.body
}

func (b *pickerControlStatusBackend) ControlStatus(context.Context) *denseexec.StatusBody {
	b.compact++
	return b.body
}

func TestPickerControlStatusPreservesSelectionAndFullDiagnosticRead(t *testing.T) {
	b := &pickerControlStatusBackend{pickerBackend: &pickerBackend{}, body: &denseexec.StatusBody{Available: true}}
	identity := &denseexec.TargetIdentity{CellID: "bench", SelectionGeneration: 7}
	p := &Picker{backend: b, identity: identity}
	control := p.ControlStatus(context.Background())
	if !control.Available || control.Target == nil || *control.Target != *identity || control.Target == identity || b.compact != 1 || b.full != 0 {
		t.Fatalf("control status did not forward compact read with copied identity: %+v, full=%d compact=%d", control, b.full, b.compact)
	}
	if b.body.Target != nil {
		t.Fatal("picker mutated backend status")
	}
	full := p.Status(context.Background())
	if full.Target == nil || *full.Target != *identity || b.full != 1 || b.compact != 1 {
		t.Fatal("full Status no longer uses the diagnostic reader")
	}
	legacy := &Picker{backend: &pickerBackend{state: "legacy"}, identity: identity}
	if got := legacy.ControlStatus(context.Background()); got.Session.State != "legacy" || got.Target.CellID != "bench" {
		t.Fatal("legacy backend fallback lost status or identity")
	}
	if got := (&Picker{}).ControlStatus(context.Background()); got.Available || got.Target != nil {
		t.Fatal("no selection became available")
	}
}
