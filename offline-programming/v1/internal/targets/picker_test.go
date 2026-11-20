//go:build linux

package targets

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/rt-core/sdk/control"
)

// TestPickerClearRetiresSelection proves disconnect cannot preserve an old
// generation or clear a controller whose required Stop was refused.
func TestPickerClearRetiresSelection(t *testing.T) {
	ctx := context.Background()
	b := &pickerBackend{}
	p := NewPicker(PickerOptions{
		Catalogue: CellCatalogue{Cells: []Cell{{CellID: "local-simulation", Models: []string{"rosie_1400_v3"}, RequestedMode: "simulation", Execution: CellExecution{Local: true}}}},
		Local:     denseexec.RTCoreConfig{Socket: "/private/control"},
		Describe: func(_ context.Context, cfg denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			d := control.Description{}
			d.Backend = "simulation"
			return cfg, d, nil
		},
		NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) { return b, nil },
	})
	if _, ok := p.SelectedRTCore(); ok {
		t.Fatal("rt-control reported before any selection")
	}
	if err := p.SelectCell(ctx, "local-simulation"); err != nil {
		t.Fatal(err)
	}
	if selected, ok := p.SelectedRTCore(); !ok || selected.CellID != "local-simulation" || selected.Host != "localhost" || selected.Config.Socket != "/private/control" {
		t.Fatalf("selected rt-control: %+v %t", selected, ok)
	}
	generation := p.Status(ctx).Target.SelectionGeneration
	p.admission.RLock()
	if err := p.Clear(ctx); err == nil || err.Error() != "cell_switch_busy: cell_switch_busy" {
		if reason(err) != "cell_switch_busy" {
			t.Fatalf("admission: %v", err)
		}
	}
	p.admission.RUnlock()
	b.stopError = errors.New("unacknowledged stop")
	if err := p.Clear(ctx); reason(err) != "cell_previous_stop_failed" {
		t.Fatalf("stop: %v", err)
	}
	if p.SelectedCell() != "local-simulation" || b.closes != 0 || p.Status(ctx).Target.SelectionGeneration != generation {
		t.Fatal("failed stop cleared selection")
	}
	b.stopError = nil
	b.state = "playing"
	if err := p.Clear(ctx); err != nil {
		t.Fatal(err)
	}
	if p.SelectedCell() != "" || p.Status(ctx).Target != nil || b.stops != 2 || b.closes != 1 || b.arms != 0 {
		t.Fatal("clear did not retire without arming")
	}
	if _, ok := p.SelectedRTCore(); ok {
		t.Fatal("cleared selection still names its rt-control")
	}
	admitted := false
	if err := p.WithTarget(generation, "local-simulation", func() { admitted = true }); reason(err) != "target_changed" || admitted {
		t.Fatal("old generation admitted")
	}
	if p.generation != generation+1 {
		t.Fatalf("generation %d", p.generation)
	}
	if err := p.WithPreview(func() { admitted = true }); err != nil || !admitted {
		t.Fatalf("offline preview: %v", err)
	}
}

// TestPickerRegistryPins ports registration to immutable commissioned bindings,
// preventing arbitrary endpoints and credential substitution through the facade.
func TestPickerRegistryPins(t *testing.T) {
	p := NewPicker(PickerOptions{Catalogue: CellCatalogue{Cells: []Cell{{
		CellID: "bench-relay", Models: []string{"rosie_1400_v3"},
		Execution: CellExecution{RTCoreConfig: denseexec.RTCoreConfig{Address: "https://127.0.0.1:8443"}},
	}}}})
	pinned := p.RegisteredCells()[0]
	input := denseexec.CellSpec{ID: pinned.ID, Address: pinned.Address, Host: pinned.Host, CredentialRef: pinned.CredentialRef}
	if err := p.RegisterCell(input); err != nil {
		t.Fatal(err)
	}
	for _, change := range []func(*denseexec.CellSpec){
		func(s *denseexec.CellSpec) { s.ID = "unknown" },
		func(s *denseexec.CellSpec) { s.Address = "https://unknown:8443" },
		func(s *denseexec.CellSpec) { s.CredentialRef = "/private/replacement" },
		func(s *denseexec.CellSpec) { s.ModelID = "rosie_1420_v1" },
		func(s *denseexec.CellSpec) { s.ConfigurationDigest = "different" },
	} {
		changed := input
		change(&changed)
		if err := p.RegisterCell(changed); err == nil || !strings.Contains(err.Error(), "not a commissioned cell") {
			t.Fatalf("admitted %+v: %v", changed, err)
		}
	}
	if p.RegisteredCells()[0] != pinned || p.SelectedCell() != "" {
		t.Fatal("registration changed native binding or selection")
	}
}

type pickerBackend struct {
	denseexec.Backend
	state               string
	stops, closes, arms int
	stopError           error
}

// TestReviewRound2DescribedPosture preserves the native backend discriminator
// in public discovery, preventing catalogue labels from implying simulation.
func TestReviewRound2DescribedPosture(t *testing.T) {
	for _, backend := range []string{"simulation", "ethercat"} {
		mode := "real"
		if backend == "simulation" {
			mode = "simulation"
		}
		p := NewPicker(PickerOptions{Catalogue: CellCatalogue{Cells: []Cell{{CellID: "bench-relay", RequestedMode: mode}}}, Describe: func(_ context.Context, cfg denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			d := control.Description{}
			d.Backend = backend
			return cfg, d, nil
		}})
		v := p.List(context.Background()).Targets[0]
		if v.Backend != backend || v.Simulation == nil || *v.Simulation != (backend == "simulation") || v.Reason != "" {
			t.Fatalf("described posture: %+v", v)
		}
	}
}

func (b *pickerBackend) Available() bool { return true }
func (b *pickerBackend) Status(context.Context) *denseexec.StatusBody {
	return &denseexec.StatusBody{Session: &denseexec.SessionInfo{State: b.state}}
}
func (b *pickerBackend) Stop(context.Context) (*denseexec.StopResult, error) {
	b.stops++
	if b.stopError == nil {
		b.state = "stopped"
	}
	return &denseexec.StopResult{Stopped: b.stopError == nil}, b.stopError
}
func (b *pickerBackend) Close(context.Context) { b.closes++ }
func (b *pickerBackend) Arm(context.Context, bool) (*denseexec.ArmResult, error) {
	b.arms++
	return &denseexec.ArmResult{}, nil
}

func TestPickerSelectionRefusalsAndStopBeforeReplacement(t *testing.T) {
	ctx := context.Background()
	local := Cell{CellID: "local-simulation", Label: "Local simulation", Models: []string{"rosie_1400_v3"}, RequestedMode: "simulation", Execution: CellExecution{Local: true}}
	remote := Cell{CellID: "bench-relay", Label: "Detached bench", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation", Execution: CellExecution{RTCoreConfig: denseexec.RTCoreConfig{Address: "https://127.0.0.1:8443", CAFile: "/private/ca", CertificateFile: "/private/client", KeyFile: "/private/key"}}}
	var made []*pickerBackend
	probes := 0
	p := NewPicker(PickerOptions{Catalogue: CellCatalogue{Cells: []Cell{local, remote}}, Local: denseexec.RTCoreConfig{Socket: "/private/control.sock"}, Describe: func(_ context.Context, c denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
		probes++
		d := control.Description{}
		d.Backend = "simulation"
		return c, d, nil
	}, NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) {
		if len(made) > 0 && made[0].stops != 1 {
			t.Fatal("replacement preceded acknowledged Stop and Release")
		}
		b := &pickerBackend{}
		made = append(made, b)
		return b, nil
	}})
	list := p.List(ctx)
	raw, _ := json.Marshal(list)
	if probes != 2 || len(made) != 0 || list.Selected != nil {
		t.Fatal("listing granted control")
	}
	for _, secret := range []string{"/private", "ca_file", "certificate_file", "key_file", "socket"} {
		if strings.Contains(string(raw), secret) {
			t.Fatalf("disclosed %s", secret)
		}
	}
	if _, err := p.Select(ctx, Selection{local.CellID, local.Models[0]}); err != nil {
		t.Fatal(err)
	}
	if made[0].arms != 0 {
		t.Fatal("selection armed")
	}
	refuse := func(s Selection, code string) {
		t.Helper()
		_, err := p.Select(ctx, s)
		var e *denseexec.Error
		if !errors.As(err, &e) || e.Code != code {
			t.Fatalf("want %s: %v", code, err)
		}
	}
	refuse(Selection{remote.CellID, "rosie_1400_v3"}, "cell_model_mismatch")
	made[0].state = "playing"
	refuse(Selection{remote.CellID, remote.Models[0]}, "cell_switch_while_playing")
	if made[0].stops != 0 || len(made) != 1 {
		t.Fatal("refused switch touched authority")
	}
	made[0].state = "done"
	p.options.Catalogue.Cells[1].RequestedMode = "real"
	refuse(Selection{remote.CellID, remote.Models[0]}, "cell_mode_mismatch")
	p.options.Catalogue.Cells[1].RequestedMode = "simulation"
	if _, err := p.Select(ctx, Selection{remote.CellID, remote.Models[0]}); err != nil {
		t.Fatal(err)
	}
	if made[0].stops != 1 || made[0].closes != 1 || len(made) != 2 || p.List(ctx).Selected.CellID != remote.CellID {
		t.Fatal("switch did not replace and remember")
	}
	if made[1].arms != 0 {
		t.Fatal("switch armed")
	}
	identity := p.Status(ctx).Target
	if identity == nil || identity.CellID != "bench-relay" || identity.ModelID != "rosie_1420_v1" || identity.Host != "127.0.0.1:8443" {
		t.Fatalf("status target: %+v", identity)
	}
	made[1].stopError = errors.New("uncertain receipt")
	refuse(Selection{local.CellID, local.Models[0]}, "cell_previous_stop_failed")
	if p.List(ctx).Selected.CellID != remote.CellID || made[1].closes != 0 {
		t.Fatal("uncertain stop replaced selection")
	}
}

func TestPickerUnreachableReasonHidesCredentialPaths(t *testing.T) {
	p := NewPicker(PickerOptions{Catalogue: CellCatalogue{Cells: []Cell{{CellID: "bench-relay"}}}, Describe: func(_ context.Context, c denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
		return c, control.Description{}, &denseexec.Error{Code: "session_principal_mismatch", Detail: "/private/key.pem"}
	}})
	list := p.List(context.Background())
	raw, _ := json.Marshal(list)
	if list.Targets[0].Backend != "unreachable" || list.Targets[0].Reason != "session_principal_mismatch" || strings.Contains(string(raw), "/private") {
		t.Fatalf("list: %s", raw)
	}
}
