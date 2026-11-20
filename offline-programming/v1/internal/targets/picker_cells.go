//go:build linux

package targets

import (
	"context"
	"net/url"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// Clear retires authority under selection admission before invalidating its
// generation. An unacknowledged Stop or Release preserves the previous cell.
func (p *Picker) Clear(ctx context.Context) error {
	if !p.admission.TryLock() {
		return refusal("cell_switch_busy")
	}
	defer p.admission.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed {
		return refusal("cell_picker_closed")
	}
	if p.generation == ^uint64(0) {
		return refusal("target_changed")
	}
	if p.backend != nil {
		stop := p.backend.Stop
		if b, ok := p.backend.(interface {
			StopSelection(context.Context) (*denseexec.StopResult, error)
		}); ok {
			stop = b.StopSelection
		}
		result, err := stop(ctx)
		if err != nil || result == nil || !result.Stopped {
			return refusal("cell_previous_stop_failed")
		}
		p.backend.Close(ctx)
	}
	p.backend, p.selected, p.identity = nil, nil, nil
	p.selectedMu.Lock()
	p.selectedRTC = nil
	p.selectedMu.Unlock()
	p.generation++
	return nil
}

func (p *Picker) SelectedCell() string {
	p.mu.RLock()
	defer p.mu.RUnlock()
	if p.selected == nil {
		return ""
	}
	return p.selected.CellID
}

// SelectCell resolves the catalogue's single model and uses native selection,
// preventing the Cells facade from choosing a model or bypassing discovery.
func (p *Picker) SelectCell(ctx context.Context, id string) error {
	if id == "" {
		return p.Clear(ctx)
	}
	p.mu.RLock()
	model := ""
	for _, cell := range p.options.Catalogue.Cells {
		if cell.CellID == id && len(cell.Models) == 1 {
			model = cell.Models[0]
		}
	}
	p.mu.RUnlock()
	if model == "" {
		return refusal("cell_id_unknown")
	}
	_, err := p.Select(ctx, Selection{CellID: id, ModelID: model})
	return err
}

func cellSpec(cell Cell) denseexec.CellSpec {
	spec := denseexec.CellSpec{ID: cell.CellID, Label: cell.Label, Address: cell.Execution.Address, CredentialRef: cell.CellID, Source: "commissioned"}
	if len(cell.Models) == 1 {
		spec.ModelID = cell.Models[0]
	}
	spec.ConfigurationDigest = cell.Execution.Binding.ConfigurationSHA256
	if cell.Execution.Local {
		spec.Host = "localhost"
		spec.CredentialRef = ""
	} else if u, err := url.Parse(spec.Address); err == nil {
		spec.Host = u.Hostname()
	}
	return spec
}
func (p *Picker) RegisteredCells() []denseexec.CellSpec {
	p.mu.RLock()
	defer p.mu.RUnlock()
	specs := make([]denseexec.CellSpec, 0, len(p.options.Catalogue.Cells))
	for _, cell := range p.options.Catalogue.Cells {
		specs = append(specs, cellSpec(cell))
	}
	return specs
}

// DescribeCells reuses authenticated native discovery for every entry; a TCP
// connection alone cannot establish reachability, backend or cell identity.
func (p *Picker) DescribeCells(ctx context.Context) denseexec.CellsBody {
	p.mu.RLock()
	defer p.mu.RUnlock()
	body := denseexec.CellsBody{Cells: []denseexec.CellProbe{}}
	if p.selected != nil {
		body.ActiveCell = p.selected.CellID
	}
	for _, cell := range p.options.Catalogue.Cells {
		cfg, view, err := p.discover(ctx, cell)
		probe := denseexec.CellProbe{CellSpec: cellSpec(cell), Reachable: err == nil}
		probe.Backend = view.Backend
		probe.ConfigurationDigest = cfg.Binding.ConfigurationSHA256
		if err != nil {
			probe.Error = reason(err)
		}
		body.Cells = append(body.Cells, probe)
	}
	return body
}

// RegisterCell admits only an already commissioned server catalogue binding.
// GUI input cannot replace endpoints, credential references, models or digests.
func (p *Picker) RegisterCell(input denseexec.CellSpec) error {
	p.mu.RLock()
	defer p.mu.RUnlock()
	for _, cell := range p.options.Catalogue.Cells {
		pinned := cellSpec(cell)
		if input.ID != pinned.ID {
			continue
		}
		if cell.unavailableReason != "" || input.Address != pinned.Address || input.Host != pinned.Host || input.CredentialRef != pinned.CredentialRef ||
			(input.ModelID != "" && input.ModelID != pinned.ModelID) || (input.ConfigurationDigest != "" && input.ConfigurationDigest != pinned.ConfigurationDigest) || input.Backend != "" || input.Source != "" {
			return &denseexec.Error{Status: 409, Code: "cell_binding_pinned", Detail: "not a commissioned cell binding; endpoint, credentials, model and configuration pins cannot be overwritten"}
		}
		return nil
	}
	return &denseexec.Error{Status: 409, Code: "cell_not_commissioned", Detail: "not a commissioned cell; provision its listener and PKI reference in the server catalogue first"}
}
