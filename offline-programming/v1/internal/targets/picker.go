//go:build linux

package targets

import (
	"context"
	"errors"
	"net/url"
	"slices"
	"sync"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/rt-core/sdk/control"
)

type CellView struct {
	CellID        string   `json:"cell_id"`
	Label         string   `json:"label"`
	Role          string   `json:"role"`
	Models        []string `json:"models"`
	RequestedMode string   `json:"requested_mode"`
	Backend       string   `json:"backend"`
	Simulation    *bool    `json:"simulation,omitempty"`
	Host          string   `json:"host"`
	Reason        string   `json:"reason,omitempty"`
}
type Selection struct {
	CellID  string `json:"cell_id"`
	ModelID string `json:"model_id"`
}
type TargetList struct {
	Targets  []CellView `json:"targets"`
	Selected *Selection `json:"selected"`
}

type PickerOptions struct {
	CartesianResolver        denseexec.CartesianResolver
	CartesianResolverFactory denseexec.CartesianResolverFactory
	Catalogue                CellCatalogue
	Local                    denseexec.RTCoreConfig
	Source                   denseexec.Config
	Describe                 func(context.Context, denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error)
	NewBackend               func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error)
	// RobotGeometry reads a model's geometry from the robot store for the
	// live hold readout (denseexec.RobotGeometry).
	RobotGeometry func(model string) (denseexec.RobotGeometry, error)
}

// Picker serializes selection with execution admission. Selection only describes
// a target; Stop and Release of the old session must succeed before replacement.
type Picker struct {
	admission  sync.RWMutex
	mu         sync.RWMutex
	generation uint64
	options    PickerOptions
	selected   *Selection
	identity   *denseexec.TargetIdentity
	backend    denseexec.Backend
	closed     bool
	// The selected cell's resolved rt-control binding, behind its own lock so a
	// reader never waits for a Select that is Stopping the previous cell.
	selectedMu  sync.Mutex
	selectedRTC *SelectedRTCore
}

// SelectedRTCore is the selected cell's rt-control as selection resolved it.
type SelectedRTCore struct {
	CellID string
	Host   string
	Config denseexec.RTCoreConfig
}

func NewPicker(options PickerOptions) *Picker {
	if options.Describe == nil {
		options.Describe = denseexec.DiscoverRTCore
	}
	if options.NewBackend == nil {
		options.NewBackend = func(cfg denseexec.RTCoreConfig, source denseexec.Config) (denseexec.Backend, error) {
			return denseexec.NewRTCore(cfg, source, nil)
		}
	}
	return &Picker{options: options}
}

func refusal(code string) error { return &denseexec.Error{Status: 409, Code: code, Detail: code} }
func reason(err error) string {
	var e *denseexec.Error
	if errors.As(err, &e) {
		return e.Code
	}
	return "cell_unreachable"
}

func (p *Picker) discover(ctx context.Context, cell Cell) (denseexec.RTCoreConfig, CellView, error) {
	cfg := cell.Execution.RTCoreConfig
	v := CellView{CellID: cell.CellID, Label: cell.Label, Role: cell.Role, Models: cell.Models, RequestedMode: cell.RequestedMode, Backend: "unreachable"}
	if cell.unavailableReason != "" {
		return cfg, v, refusal(cell.unavailableReason)
	}
	if cell.Execution.Local {
		cfg = p.options.Local
		cfg.ExpectedBackend = "simulation"
		if cfg.Address != "" || cfg.Socket == "" {
			return cfg, v, refusal("cell_local_binding_missing")
		}
		v.Host = "localhost"
	} else if u, err := url.Parse(cfg.Address); err == nil {
		v.Host = u.Host
	}
	resolved, d, err := p.options.Describe(ctx, cfg)
	if d.Backend != "" {
		v.Backend = d.Backend
	}
	if err != nil {
		return cfg, v, err
	}
	// The native description encodes simulation in its backend discriminator.
	simulation := d.Backend == "simulation"
	v.Simulation = &simulation
	mode := "simulation"
	if d.Backend == "ethercat" {
		mode = "real"
	}
	if cell.RequestedMode != mode {
		return cfg, v, refusal("cell_mode_mismatch")
	}
	return resolved, v, nil
}

func (p *Picker) List(ctx context.Context) TargetList {
	p.mu.RLock()
	defer p.mu.RUnlock()
	list := TargetList{Targets: []CellView{}}
	if p.selected != nil {
		selection := *p.selected
		list.Selected = &selection
	}
	for _, cell := range p.options.Catalogue.Cells {
		_, view, err := p.discover(ctx, cell)
		if err != nil {
			view.Reason = reason(err)
			if view.Backend == "" {
				view.Backend = "unreachable"
			}
		}
		list.Targets = append(list.Targets, view)
	}
	return list
}

func (p *Picker) Select(ctx context.Context, selection Selection) (CellView, error) {
	if !p.admission.TryLock() {
		return CellView{}, refusal("cell_switch_busy")
	}
	defer p.admission.Unlock()
	if !p.mu.TryLock() {
		return CellView{}, refusal("cell_switch_busy")
	}
	defer p.mu.Unlock()
	if p.closed {
		return CellView{}, refusal("cell_picker_closed")
	}
	if p.generation == ^uint64(0) {
		return CellView{}, refusal("target_changed")
	}
	if p.backend != nil {
		status := p.backend.Status(ctx)
		if status.Session != nil && (status.Session.State == "playing" || status.Session.State == "paused") {
			return CellView{}, refusal("cell_switch_while_playing")
		}
	}
	for _, cell := range p.options.Catalogue.Cells {
		if cell.CellID != selection.CellID {
			continue
		}
		if !slices.Contains(cell.Models, selection.ModelID) {
			return CellView{}, refusal("cell_model_mismatch")
		}
		cfg, view, err := p.discover(ctx, cell)
		if err != nil {
			return view, err
		}
		if p.backend != nil {
			stop := p.backend.Stop
			if selection, ok := p.backend.(interface {
				StopSelection(context.Context) (*denseexec.StopResult, error)
			}); ok {
				stop = selection.StopSelection
			}
			if _, err := stop(ctx); err != nil {
				return view, refusal("cell_previous_stop_failed")
			}
		}
		cfg.CartesianModel = selection.ModelID
		next, err := p.options.NewBackend(cfg, p.options.Source)
		if err != nil || next == nil {
			return view, refusal("cell_backend_unavailable")
		}
		if c, ok := next.(interface {
			SetCartesianResolver(denseexec.CartesianResolver)
		}); ok {
			c.SetCartesianResolver(p.options.CartesianResolver)
		}
		if c, ok := next.(interface {
			SetCartesianResolverFactory(denseexec.CartesianResolverFactory)
		}); ok && p.options.CartesianResolverFactory != nil {
			c.SetCartesianResolverFactory(p.options.CartesianResolverFactory)
		}
		if c, ok := next.(interface {
			SetRobotGeometrySource(func(string) (denseexec.RobotGeometry, error))
		}); ok && p.options.RobotGeometry != nil {
			c.SetRobotGeometrySource(p.options.RobotGeometry)
		}
		if p.backend != nil {
			p.backend.Close(ctx)
		}
		p.backend, p.selected = next, &selection
		p.generation++
		p.identity = &denseexec.TargetIdentity{SelectionGeneration: p.generation, CellID: cell.CellID, ModelID: selection.ModelID, Label: view.Label, Backend: view.Backend, Simulation: view.Simulation, Host: view.Host}
		p.selectedMu.Lock()
		p.selectedRTC = &SelectedRTCore{CellID: cell.CellID, Host: view.Host, Config: cfg}
		p.selectedMu.Unlock()
		return view, nil
	}
	return CellView{}, refusal("cell_id_unknown")
}

func (p *Picker) Available() bool {
	p.mu.RLock()
	defer p.mu.RUnlock()
	return !p.closed && p.backend != nil && p.backend.Available()
}
func (p *Picker) Close(ctx context.Context) {
	p.admission.Lock()
	defer p.admission.Unlock()
	p.mu.Lock()
	defer p.mu.Unlock()
	p.closed = true
	if p.backend != nil {
		p.backend.Close(ctx)
	}
	p.selectedMu.Lock()
	p.selectedRTC = nil
	p.selectedMu.Unlock()
}

// UnattendedLoopAllowed is the selected cell's
// cell_deploy.olp.unattended_loop_allowed, as its backend runs it; false
// before any selection and after Close.
func (p *Picker) UnattendedLoopAllowed() bool {
	p.mu.RLock()
	defer p.mu.RUnlock()
	backend, ok := p.backend.(interface{ UnattendedLoopAllowed() bool })
	return !p.closed && ok && backend.UnattendedLoopAllowed()
}

// ControllerID is the name every backend of this picker acquires rt-control
// grants with, so a grant holder rt-control reports can be told apart from a
// controller that is not this server.
func (p *Picker) ControllerID() string { return p.options.Source.ControllerID }

// SelectedRTCore reports which rt-control the selected cell is; false before
// any selection and after Close.
func (p *Picker) SelectedRTCore() (SelectedRTCore, bool) {
	p.selectedMu.Lock()
	defer p.selectedMu.Unlock()
	if p.selectedRTC == nil {
		return SelectedRTCore{}, false
	}
	return *p.selectedRTC, true
}

// WithTarget holds selection stable through an energising request. Missing or
// stale client identity cannot redirect an old browser action to another cell.
func (p *Picker) WithTarget(generation uint64, cellID string, action func()) error {
	p.admission.RLock()
	defer p.admission.RUnlock()
	p.mu.RLock()
	valid := !p.closed && p.identity != nil && generation != 0 &&
		p.identity.SelectionGeneration == generation && p.identity.CellID == cellID
	p.mu.RUnlock()
	if !valid {
		return refusal("target_changed")
	}
	action()
	return nil
}

// WithPreview excludes selection while a preview request is admitted. An old
// browser cannot restart a second controller after a machine is selected.
func (p *Picker) WithPreview(action func()) error {
	p.admission.RLock()
	defer p.admission.RUnlock()
	p.mu.RLock()
	defer p.mu.RUnlock()
	if p.closed || p.selected != nil {
		return refusal("target_changed")
	}
	action()
	return nil
}
func (p *Picker) Capabilities(ctx context.Context) denseexec.Capabilities {
	p.mu.RLock()
	defer p.mu.RUnlock()
	if p.backend == nil {
		return denseexec.Capabilities{Backend: "rt_core", Reason: "cell_not_selected"}
	}
	return p.backend.Capabilities(ctx)
}
func (p *Picker) Status(ctx context.Context) *denseexec.StatusBody {
	return p.status(ctx, false)
}

// ControlStatus forwards the compact read before the backend contacts native
// control, while preserving the same target identity and selection lock.
func (p *Picker) ControlStatus(ctx context.Context) *denseexec.StatusBody {
	return p.status(ctx, true)
}

func (p *Picker) status(ctx context.Context, currentOnly bool) *denseexec.StatusBody {
	p.mu.RLock()
	defer p.mu.RUnlock()
	if p.backend == nil {
		return &denseexec.StatusBody{Backend: "rt_core", Available: false}
	}
	read := p.backend.Status
	if observer, ok := p.backend.(denseexec.ControlStatusReader); currentOnly && ok {
		read = observer.ControlStatus
	}
	status := *read(ctx)
	if p.identity != nil {
		identity := *p.identity
		status.Target = &identity
	}
	return &status
}

func withBackend[T any](p *Picker, f func(denseexec.Backend) (T, error)) (T, error) {
	p.mu.RLock()
	defer p.mu.RUnlock()
	if p.closed || p.backend == nil {
		var zero T
		return zero, refusal("cell_not_selected")
	}
	return f(p.backend)
}
func (p *Picker) Load(c context.Context, r denseexec.LoadRequest) (*denseexec.LoadResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.LoadResult, error) { return b.Load(c, r) })
}
func (p *Picker) Play(c context.Context, s string) (*denseexec.StatusBody, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.StatusBody, error) { return b.Play(c, s) })
}
func (p *Picker) Pause(c context.Context, s string) (*denseexec.StatusBody, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.StatusBody, error) { return b.Pause(c, s) })
}
func (p *Picker) Stop(c context.Context) (*denseexec.StopResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.StopResult, error) { return b.Stop(c) })
}
func (p *Picker) Arm(c context.Context, a bool) (*denseexec.ArmResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.ArmResult, error) { return b.Arm(c, a) })
}
func (p *Picker) GoHome(c context.Context) (*denseexec.GoHomeResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.GoHomeResult, error) { return b.GoHome(c) })
}
func (p *Picker) Heartbeat(s string) (*denseexec.HeartbeatResult, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.HeartbeatResult, error) { return b.Heartbeat(s) })
}
func (p *Picker) TelemetryWindow(c context.Context, seconds float64, stride int) (*denseexec.TelemetryWindow, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.TelemetryWindow, error) {
		return b.TelemetryWindow(c, seconds, stride)
	})
}

// CheckTestProgram runs the selected machine's Load checks for an uploaded
// test program without acquiring authority (denseexec CheckTestProgram).
func (p *Picker) CheckTestProgram(c context.Context, digest string) ([]string, error) {
	return withBackend(p, func(b denseexec.Backend) ([]string, error) {
		checker, ok := b.(interface {
			CheckTestProgram(context.Context, string) ([]string, error)
		})
		if !ok {
			return nil, refusal("capability_unimplemented")
		}
		return checker.CheckTestProgram(c, digest)
	})
}

// Cell and Description are the selected machine's side of the robot
// description contract (denseexec.CellTarget), forwarded as-is: the picker
// is which machine, never which robot.
func (p *Picker) Cell(c context.Context) (*denseexec.CellBody, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.CellBody, error) {
		target, ok := b.(interface {
			Cell(context.Context) (*denseexec.CellBody, error)
		})
		if !ok {
			return nil, refusal("capability_unimplemented")
		}
		return target.Cell(c)
	})
}
func (p *Picker) Description(c context.Context) (*denseexec.DescriptionBody, error) {
	return withBackend(p, func(b denseexec.Backend) (*denseexec.DescriptionBody, error) {
		target, ok := b.(interface {
			Description(context.Context) (*denseexec.DescriptionBody, error)
		})
		if !ok {
			return nil, refusal("capability_unimplemented")
		}
		return target.Description(c)
	})
}
func (p *Picker) Home(c context.Context) error {
	_, err := withBackend(p, func(b denseexec.Backend) (bool, error) {
		h, ok := b.(interface{ Home(context.Context) error })
		if !ok {
			return false, refusal("capability_unimplemented")
		}
		return true, h.Home(c)
	})
	return err
}
func (p *Picker) HomeAxes(c context.Context, mask uint32) error {
	_, err := withBackend(p, func(b denseexec.Backend) (bool, error) {
		h, ok := b.(interface {
			HomeAxes(context.Context, uint32) error
		})
		if !ok {
			return false, refusal("capability_unimplemented")
		}
		return true, h.HomeAxes(c, mask)
	})
	return err
}

// JointJog forwards the selected session under the selection lock, preventing
// a hold from opening or updating a lane on a replacement backend.
func (p *Picker) JointJog(c context.Context, action string, in denseexec.JointJogIntent) (denseexec.JointJogState, error) {
	return withBackend(p, func(b denseexec.Backend) (denseexec.JointJogState, error) {
		j, ok := b.(denseexec.JointJogBackend)
		if !ok {
			return denseexec.JointJogState{}, refusal("dense_target_unavailable")
		}
		return j.JointJog(c, action, in)
	})
}

// Trace pins the selected backend just like joystick jog. A replacement cell
// cannot inherit a pending trace request or the old session's path.
func (p *Picker) Trace(c context.Context, action string, in denseexec.TraceIntent) (denseexec.JointJogState, error) {
	return withBackend(p, func(b denseexec.Backend) (denseexec.JointJogState, error) {
		trace, ok := b.(denseexec.TraceBackend)
		if !ok {
			return denseexec.JointJogState{}, refusal("dense_target_unavailable")
		}
		return trace.Trace(c, action, in)
	})
}

// SetCartesianResolver installs computation for the selected and future cells;
// selection remains the only source of model identity.
func (p *Picker) SetCartesianResolver(resolve denseexec.CartesianResolver) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.options.CartesianResolver = resolve
	if c, ok := p.backend.(interface {
		SetCartesianResolver(denseexec.CartesianResolver)
	}); ok {
		c.SetCartesianResolver(resolve)
	}
}

// SetCartesianResolverFactory starts a separate resolver for every selected
// backend; a replaced backend owns cleanup of its original process.
func (p *Picker) SetCartesianResolverFactory(factory denseexec.CartesianResolverFactory) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.options.CartesianResolverFactory = factory
	if c, ok := p.backend.(interface {
		SetCartesianResolverFactory(denseexec.CartesianResolverFactory)
	}); ok {
		c.SetCartesianResolverFactory(factory)
	}
}

// SetRobotGeometrySource reads robot geometry for the selected and future
// cells (the live hold readout); it grants nothing.
func (p *Picker) SetRobotGeometrySource(source func(string) (denseexec.RobotGeometry, error)) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.options.RobotGeometry = source
	if c, ok := p.backend.(interface {
		SetRobotGeometrySource(func(string) (denseexec.RobotGeometry, error))
	}); ok {
		c.SetRobotGeometrySource(source)
	}
}

// CartesianJog uses the selected backend under the same lock as joint jog,
// preventing a late browser request from adopting a replacement controller.
func (p *Picker) CartesianJog(c context.Context, action string, in denseexec.CartesianJogIntent) (denseexec.JointJogState, error) {
	return withBackend(p, func(b denseexec.Backend) (denseexec.JointJogState, error) {
		j, ok := b.(denseexec.CartesianJogBackend)
		if !ok {
			return denseexec.JointJogState{}, refusal("dense_target_unavailable")
		}
		return j.CartesianJog(c, action, in)
	})
}

// JointJogState reports only the selected session. Missing joint support stays
// inactive with a refusal reason, preventing readiness from a previous target.
func (p *Picker) JointJogState() denseexec.JointJogState {
	state, err := withBackend(p, func(b denseexec.Backend) (denseexec.JointJogState, error) {
		j, ok := b.(denseexec.JointJogBackend)
		if !ok {
			return denseexec.JointJogState{}, refusal("dense_target_unavailable")
		}
		return j.JointJogState(), nil
	})
	if err != nil {
		state.Reason = reason(err)
	}
	return state
}

// JointMove keeps the selected backend stable until completion or cancellation;
// Stop and Disarm retain concurrent access to the same session.
func (p *Picker) JointMove(c context.Context, in denseexec.JointMoveIntent) (denseexec.JointMoveState, error) {
	return withBackend(p, func(b denseexec.Backend) (denseexec.JointMoveState, error) {
		j, ok := b.(denseexec.JointMoveBackend)
		if !ok {
			return denseexec.JointMoveState{}, refusal("dense_target_unavailable")
		}
		return j.JointMove(c, in)
	})
}

var _ denseexec.Backend = (*Picker)(nil)
var _ denseexec.JointJogBackend = (*Picker)(nil)
var _ denseexec.TraceBackend = (*Picker)(nil)
var _ denseexec.JointMoveBackend = (*Picker)(nil)

// CartesianMove pins the selected backend through completion while Stop and
// Disarm retain concurrent access; a replacement cannot inherit the move.
func (p *Picker) CartesianMove(c context.Context, in denseexec.CartesianMoveIntent) (denseexec.JointMoveState, error) {
	return withBackend(p, func(b denseexec.Backend) (denseexec.JointMoveState, error) {
		m, ok := b.(denseexec.CartesianMoveBackend)
		if !ok {
			return denseexec.JointMoveState{}, refusal("dense_target_unavailable")
		}
		return m.CartesianMove(c, in)
	})
}
