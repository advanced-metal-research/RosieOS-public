// The cell's own maintenance notice. rt-control on the servo host serves the
// notice it enforces (GET /v1/maintenance: while it is active, acquire and
// motion starts are refused "busy"). This server merges it with its own host's
// notice (maintenance.go), so clients of an OLP whose host carries no notice --
// a Steam Deck running OLP locally -- block while the servo host is updated,
// and the relay's clients block if only the servo host's copy was written.
package main

import (
	"context"
	"crypto/tls"
	"crypto/x509"
	"errors"
	"fmt"
	"net/url"
	"os"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
)

const (
	// One read per UI poll period (MAINTENANCE_POLL_MS, 2 s) however many
	// clients poll; the gate reuses the same answer.
	cellMaintenanceMaxAge = 2 * time.Second
	// Bounds one read, so an unreachable servo host delays one request per
	// period rather than every request. Unmeasured; a LAN read takes ms.
	cellMaintenanceTimeout = 1500 * time.Millisecond
)

// MaintenanceCell is the selected cell's notice as its rt-control answered.
// While rt-control does not answer after it reported an active notice, the
// notice is that last answer and stays active: a full deploy restarts
// rt-control under the very notice that announced it.
type MaintenanceCell struct {
	Cell string `json:"cell,omitempty"`
	Host string `json:"host,omitempty"`
	// Reachable: rt-control answered this read. Supported: it serves the route
	// (an rt-control older than the route answers unknown_endpoint).
	Reachable  bool   `json:"reachable"`
	Supported  bool   `json:"supported"`
	Error      string `json:"error,omitempty"`
	CheckedAt  string `json:"checked_at"`
	AnsweredAt string `json:"answered_at,omitempty"`
	control.MaintenanceObservation
}

// cellRTControl is the rt-control the notice is read from.
type cellRTControl struct {
	CellID, Host string
	Config       denseexec.RTCoreConfig
}

type cellMaintenanceClient interface {
	Maintenance(context.Context) (control.MaintenanceObservation, error)
	Close() error
}

// CellMaintenance reads the selected cell's notice, at most once per
// cellMaintenanceMaxAge, over one client per selected cell.
type CellMaintenance struct {
	target  func() (cellRTControl, bool)
	dial    func(denseexec.RTCoreConfig) (cellMaintenanceClient, error)
	now     func() time.Time
	maxAge  time.Duration
	timeout time.Duration

	mu      sync.Mutex
	key     string
	client  cellMaintenanceClient
	view    *MaintenanceCell
	checked time.Time
}

func newCellMaintenance(target func() (cellRTControl, bool)) *CellMaintenance {
	return &CellMaintenance{target: target, dial: dialCellMaintenance, now: time.Now, maxAge: cellMaintenanceMaxAge, timeout: cellMaintenanceTimeout}
}

// cellMaintenanceOf follows the picker's selection; a fixed --rt-core-config
// backend always reads its one rt-control; anything else reads none.
func cellMaintenanceOf(backend denseexec.Backend, fixed *denseexec.RTCoreConfig) *CellMaintenance {
	if picker, ok := backend.(*targets.Picker); ok {
		return newCellMaintenance(func() (cellRTControl, bool) {
			selected, ok := picker.SelectedRTCore()
			return cellRTControl{CellID: selected.CellID, Host: selected.Host, Config: selected.Config}, ok
		})
	}
	if fixed != nil {
		cfg := *fixed
		host := "localhost"
		if cfg.Address != "" {
			if u, err := url.Parse(cfg.Address); err == nil && u.Host != "" {
				host = u.Host
			} else {
				host = cfg.Address
			}
		}
		return newCellMaintenance(func() (cellRTControl, bool) {
			return cellRTControl{CellID: cfg.Binding.PairID, Host: host, Config: cfg}, true
		})
	}
	return nil
}

// Current is the selected cell's notice; nil with no cell selected.
func (m *CellMaintenance) Current(ctx context.Context) *MaintenanceCell {
	if m == nil {
		return nil
	}
	target, selected := m.target()
	m.mu.Lock()
	defer m.mu.Unlock()
	key := ""
	if selected {
		cfg := target.Config
		key = strings.Join([]string{target.CellID, cfg.Socket, cfg.Address, cfg.CAFile, cfg.CertificateFile, cfg.KeyFile}, "\x00")
	}
	if key != m.key {
		if m.client != nil {
			_ = m.client.Close()
		}
		m.key, m.client, m.view, m.checked = key, nil, nil, time.Time{}
	}
	if !selected {
		return nil
	}
	now := m.now()
	if m.view != nil && now.Sub(m.checked) < m.maxAge {
		view := *m.view
		return &view
	}
	next := &MaintenanceCell{Cell: target.CellID, Host: target.Host, CheckedAt: now.UTC().Format(time.RFC3339)}
	observation, err := m.read(ctx, target.Config)
	var rejected *control.Rejected
	switch {
	case err == nil:
		next.Reachable, next.Supported, next.AnsweredAt = true, true, next.CheckedAt
		next.MaintenanceObservation = observation
	case errors.As(err, &rejected) && strings.HasPrefix(rejected.Reason, "unknown_endpoint"):
		// An rt-control from before the route: it answers, it just cannot say.
		next.Reachable, next.AnsweredAt = true, next.CheckedAt
		next.Error = "this rt-control has no maintenance route (unknown_endpoint); its busy refusals still apply"
	default:
		next.Error = "rt-control did not answer: " + err.Error()
		if m.view != nil && m.view.Active {
			next.MaintenanceObservation, next.AnsweredAt, next.Supported = m.view.MaintenanceObservation, m.view.AnsweredAt, true
			next.Holder = nil
		}
	}
	m.view, m.checked = next, now
	view := *next
	return &view
}

func (m *CellMaintenance) read(ctx context.Context, cfg denseexec.RTCoreConfig) (control.MaintenanceObservation, error) {
	if m.client == nil {
		client, err := m.dial(cfg)
		if err != nil {
			return control.MaintenanceObservation{}, err
		}
		m.client = client
	}
	ctx, cancel := context.WithTimeout(ctx, m.timeout)
	defer cancel()
	return m.client.Maintenance(ctx)
}

// Close releases the client.
func (m *CellMaintenance) Close() {
	if m == nil {
		return
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.client != nil {
		_ = m.client.Close()
	}
	m.key, m.client, m.view = "", nil, nil
}

// dialCellMaintenance opens a read-only client with the cell's own
// credentials: the Unix socket, or the remote mutual-TLS listener with the
// same CA and client certificate dense execution uses. It never acquires.
func dialCellMaintenance(cfg denseexec.RTCoreConfig) (cellMaintenanceClient, error) {
	if (cfg.Socket == "") == (cfg.Address == "") {
		return nil, errors.New("rt_core requires exactly one Unix socket or authenticated HTTPS address")
	}
	if cfg.Socket != "" {
		return control.Dial(control.UnixPath(cfg.Socket))
	}
	pem, err := os.ReadFile(cfg.CAFile)
	if err != nil {
		return nil, err
	}
	roots := x509.NewCertPool()
	if !roots.AppendCertsFromPEM(pem) {
		return nil, fmt.Errorf("rt_core CA %s is invalid", cfg.CAFile)
	}
	cert, err := tls.LoadX509KeyPair(cfg.CertificateFile, cfg.KeyFile)
	if err != nil {
		return nil, err
	}
	return control.DialTLS(cfg.Address, &tls.Config{RootCAs: roots, Certificates: []tls.Certificate{cert}})
}

// mergeMaintenance is what clients see: active while either notice is,
// both reasons when they differ, the later ETA and the earlier start. With
// only the cell's notice active, the view is the cell's.
func mergeMaintenance(view MaintenanceView, cell *MaintenanceCell, now time.Time, staleAfter time.Duration) MaintenanceView {
	view.Cell = cell
	if cell == nil || !cell.Active {
		return view
	}
	where := "servo host"
	if cell.Cell != "" {
		where = "servo host (" + cell.Cell + ")"
	}
	cellReason := firstNonEmpty(cell.Reason, "Update in progress on the "+where)
	var problems []string
	if view.Error != "" {
		problems = append(problems, view.Error)
	}
	if cell.Malformed {
		problems = append(problems, "the "+where+" notice could not be fully read")
	}
	if !cell.Reachable {
		problems = append(problems, "rt-control on the "+where+" is not answering; its last notice is shown")
	}
	if !view.Active {
		view.Active = true
		view.Reason, view.Phase = cellReason, cell.Phase
		view.StartedAt, view.ETA = cell.StartedAt, cell.ETA
		view.SetBy, view.Release = cell.SetBy, cell.Release
		view.Malformed = cell.Malformed
	} else {
		if cellReason != view.Reason {
			view.Reason = clipMaintenanceText(view.Reason + " · " + where + ": " + cellReason)
		}
		if cell.Phase != "" && cell.Phase != view.Phase {
			if view.Phase == "" {
				view.Phase = cell.Phase
			} else {
				view.Phase = clipMaintenanceText(view.Phase + " · " + where + ": " + cell.Phase)
			}
		}
		view.ETA = laterRFC3339(view.ETA, cell.ETA)
		view.StartedAt = earlierRFC3339(view.StartedAt, cell.StartedAt)
		view.SetBy = firstNonEmpty(view.SetBy, cell.SetBy)
		view.Release = firstNonEmpty(view.Release, cell.Release)
		view.Malformed = view.Malformed || cell.Malformed
	}
	view.Error = strings.Join(problems, "; ")
	view.Overrun, view.Stale = false, false
	if eta, err := time.Parse(time.RFC3339, view.ETA); err == nil {
		view.Overrun, view.Stale = now.After(eta), now.Sub(eta) > staleAfter
	} else if started, err := time.Parse(time.RFC3339, view.StartedAt); err == nil {
		view.Stale = now.Sub(started) > staleAfter
	}
	return view
}

func laterRFC3339(a, b string) string {
	ta, errA := time.Parse(time.RFC3339, a)
	tb, errB := time.Parse(time.RFC3339, b)
	if errA != nil {
		return b
	}
	if errB != nil || !tb.After(ta) {
		return a
	}
	return b
}

func earlierRFC3339(a, b string) string {
	ta, errA := time.Parse(time.RFC3339, a)
	tb, errB := time.Parse(time.RFC3339, b)
	if errA != nil {
		return b
	}
	if errB != nil || !tb.Before(ta) {
		return a
	}
	return b
}
