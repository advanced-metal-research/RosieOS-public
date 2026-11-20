// Who controls the robot through this server. Every client of one OLP server
// (browser tabs, Steam Deck pendants, scripts) shares the one rt-control grant
// the server holds, so rt-control's own exclusivity (a second Acquire is
// refused control_already_owned) only separates servers, never two clients of
// the same server. Without this, a second client's Connect re-selected the
// cell, which Stops the selected backend: it silently disarmed whoever was
// jogging; its Arm, jog or Move drove a robot someone else had armed.
//
// The rule. The client whose control request was last admitted is the owner.
// While the robot is in use -- this server holds the grant, the machine is
// armed, or one of the owner's requests (a Move, Home, Load) is still running
// -- every request that could take control or start motion from any other
// client is refused 409 "in_use_by_other_controller" with who holds it.
// While nothing is in use, the next client to act becomes the owner. A
// disarmed holder whose last request is controlTakeoverIdle old may be
// replaced, but only by an explicit POST .../control/take; an armed holder or
// a running request never. Stop, Pause, Disarm, Disconnect, the heartbeat
// and ending a hold always pass, from anyone: any operator can stop the robot.
//
// The same request is refused when the selected cell's rt-control says its
// grant is held by a controller that is not this server (another OLP, a Deck
// running OLP locally, rtctl): naming the holder instead of the bare
// control_already_owned rt-control would answer. This server cannot stop that
// holder's motion; the view says so.
package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net"
	"net/http"
	"strings"
	"sync"
	"time"

	"rosieos/rt-core/sdk/control"
)

const (
	controlAPIPath    = "/api/offline-programming/v1/control"
	controlViewSchema = "offline-programming.control.v1"
	// Optional, opaque, chosen by the client: one per browser tab
	// (sessionStorage, so a reload stays the same controller) and one per
	// pendant process. A client that sends none is identified by what its
	// requests already carry (maintenance_clients.go), as before.
	clientIDHeader = "X-Rosie-Client-Id"
	// A disarmed holder whose last control request finished this long ago
	// may be replaced by an explicit Take control. Owner's choice in the
	// request (2026-10-07: "holder idle and disarmed beyond a timeout"), not
	// measured; long enough to cover the gap between Home and Arm.
	controlTakeoverIdle = 30 * time.Second
	// The refusal code both clients key their pop-up on.
	controlRefusal = "in_use_by_other_controller"
)

// controlClient is who sent a request: its identity key and the words the
// connected-clients list uses for it.
type controlClient struct {
	key   string
	named bool
	// who: the journal's words for the sender (clientIdentity.String), when
	// the gate has read them.
	who string
	MaintenanceClient
}

func (c controlClient) describe() string {
	if c.who != "" {
		return c.who
	}
	return c.Label + " (" + c.Address + ")"
}

// controlClientOf names a request's client. A valid X-Rosie-Client-Id makes
// each tab or pendant its own controller; without one, the peer address,
// Origin/Referer and User-Agent do, which cannot tell two tabs of one browser
// apart (they then count as one controller, the behaviour before this).
func controlClientOf(r *http.Request) controlClient {
	kind, label, origin := classifyClient(r)
	host, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		host = r.RemoteAddr
	}
	ip := net.ParseIP(host)
	client := controlClient{MaintenanceClient: MaintenanceClient{Kind: kind, Label: label, Address: host, Loopback: ip != nil && ip.IsLoopback(), Origin: origin}}
	if id := strings.TrimSpace(r.Header.Get(clientIDHeader)); validClientID(id) {
		client.key, client.named = "id:"+id, true
	} else {
		client.key = "peer:" + kind + "\x00" + label + "\x00" + host + "\x00" + origin
	}
	return client
}

func validClientID(id string) bool {
	if len(id) < 8 || len(id) > 64 {
		return false
	}
	for _, c := range id {
		if !(c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z' || c >= '0' && c <= '9' || c == '-' || c == '_' || c == '.' || c == ':') {
			return false
		}
	}
	return true
}

// controlHolder is the owner: the client whose control request was last
// admitted, while it matters (the robot in use).
type controlHolder struct {
	client controlClient
	since  time.Time
	// The last admitted request: its action and when it finished (or was
	// admitted, while it runs).
	action     string
	lastAction time.Time
	// Admitted requests still running; a running Move or Home is in use
	// even before the controller reports the grant.
	inFlight int
	running  string
}

type controlState struct {
	mu    sync.Mutex
	now   func() time.Time
	owner *controlHolder
}

func newControlState() *controlState { return &controlState{now: time.Now} }

// controlPosture is the selected machine as this server sees it, read from
// its own status body (JSON, so it does not depend on the platform's rt-core
// types, as maintenanceMachine).
type controlPosture struct {
	Armed     bool
	LeaseHeld bool
	Session   string
}

func (p controlPosture) inUse() bool { return p.Armed || p.LeaseHeld }

func controlPostureOf(status *denseStatusJSON) controlPosture {
	if status == nil {
		return controlPosture{}
	}
	posture := controlPosture{Armed: status.Armed != nil && *status.Armed}
	if status.Session != nil {
		posture.Session, posture.LeaseHeld = status.Session.State, status.Session.Lease.Held
		if posture.Session == "stopped" {
			posture.LeaseHeld = false
		}
	}
	if status.RTCore != nil && status.RTCore.Status != nil && status.RTCore.Status.Core != nil && status.RTCore.Status.Core.Armed != 0 && posture.LeaseHeld {
		posture.Armed = true
	}
	return posture
}

// denseStatusJSON is the part of the status body the maintenance and control
// views read.
type denseStatusJSON struct {
	Target *struct {
		CellID string `json:"cell_id"`
		Label  string `json:"label"`
		Host   string `json:"host"`
	} `json:"target"`
	Session *struct {
		State          string `json:"state"`
		HeartbeatAgeMS *int64 `json:"heartbeat_age_ms"`
		Lease          struct {
			Held bool `json:"held"`
		} `json:"lease"`
	} `json:"session"`
	Armed  *bool `json:"armed"`
	RTCore *struct {
		Status *struct {
			Core *struct {
				Armed          uint32 `json:"armed"`
				LeaseValid     uint32 `json:"lease_valid"`
				AxisEnableMask uint32 `json:"axis_enable_mask"`
			} `json:"core"`
		} `json:"status"`
	} `json:"rt_core"`
}

// denseStatus reads the selected machine's status once; nil without a backend
// or when the body cannot be read.
func (server *Server) denseStatus(ctx context.Context) *denseStatusJSON {
	if server.denseExecution == nil {
		return nil
	}
	status := server.denseExecution.Status(ctx)
	if status == nil {
		return nil
	}
	raw, err := json.Marshal(status)
	if err != nil {
		return nil
	}
	var body denseStatusJSON
	if json.Unmarshal(raw, &body) != nil {
		return nil
	}
	return &body
}

// servoHolderElsewhere is the selected cell's rt-control grant holder when it
// is not this server (rt-control's answer, at most cellMaintenanceMaxAge
// old); nil when nobody holds it, it is this server, or this server cannot
// tell (no maintenance route, or a backend that does not name itself).
func (server *Server) servoHolderElsewhere(ctx context.Context) *control.MaintenanceHolder {
	named, ok := server.denseExecution.(interface{ ControllerID() string })
	if !ok || named.ControllerID() == "" || server.cellMaintenance == nil {
		return nil
	}
	cell := server.cellMaintenance.Current(ctx)
	if cell == nil || cell.Holder == nil || cell.Holder.Controller == named.ControllerID() {
		return nil
	}
	holder := *cell.Holder
	return &holder
}

// ControlOwner is the holder as clients see it.
type ControlOwner struct {
	Kind     string `json:"kind"`
	Label    string `json:"label"`
	Address  string `json:"address"`
	Loopback bool   `json:"loopback"`
	Origin   string `json:"origin,omitempty"`
	// Since: when it took control. LastAction: its last control request
	// ("arm", "jog/begin", ...) and when that finished; Running: one still
	// running.
	Since        string `json:"since"`
	LastAction   string `json:"last_action,omitempty"`
	LastActionAt string `json:"last_action_at,omitempty"`
	Running      string `json:"running,omitempty"`
	IdleMS       int64  `json:"idle_ms"`
}

// ControlView is GET .../control (and the "control" member of the
// maintenance view and of every refusal), as the requester sees it.
type ControlView struct {
	Schema string `json:"schema"`
	Now    string `json:"now"`
	// State: "free" (nobody is using the robot), "yours" (the requester is),
	// "other" (another client of this server is), "other_server" (the
	// cell's rt-control grant is held by a controller that is not this
	// server).
	State string `json:"state"`
	// You: how this server names the requester; Named: it sent a client id.
	You   MaintenanceClient `json:"you"`
	Named bool              `json:"named"`
	// Owner: the client using the robot through this server (states "yours"
	// and "other").
	Owner *ControlOwner `json:"owner,omitempty"`
	// The machine: armed, this server holds its grant, the program session.
	Armed        bool   `json:"armed"`
	LeaseHeld    bool   `json:"lease_held"`
	SessionState string `json:"session_state,omitempty"`
	// ServoHolder: rt-control's grant holder (state "other_server").
	ServoHolder *control.MaintenanceHolder `json:"servo_holder,omitempty"`
	// TakeControl: POST .../control/take would succeed for the requester now.
	// TakeControlNote says why not, or after how long it will.
	TakeControl     bool   `json:"take_control"`
	TakeControlNote string `json:"take_control_note,omitempty"`
	TakeoverIdleMS  int64  `json:"takeover_idle_ms"`
	// StopAvailable: this server's Stop reaches the robot (false only for
	// "other_server": its holder's Stop or the hardware stop does).
	StopAvailable bool   `json:"stop_available"`
	Detail        string `json:"detail"`
}

// controlSnapshot copies the holder for a view.
func (c *controlState) snapshot() (controlHolder, bool) {
	if c == nil {
		return controlHolder{}, false
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.owner == nil {
		return controlHolder{}, false
	}
	return *c.owner, true
}

// controlViewOf applies the rule to one requester (pure; controlView reads
// the inputs).
func controlViewOf(you controlClient, holder *controlHolder, posture controlPosture, servo *control.MaintenanceHolder, now time.Time) ControlView {
	view := ControlView{Schema: controlViewSchema, Now: now.UTC().Format(time.RFC3339), State: "free", You: you.MaintenanceClient, Named: you.named,
		Armed: posture.Armed, LeaseHeld: posture.LeaseHeld, SessionState: posture.Session, StopAvailable: true,
		TakeoverIdleMS: controlTakeoverIdle.Milliseconds()}
	if servo != nil {
		view.State, view.ServoHolder, view.StopAvailable = "other_server", servo, false
		view.TakeControlNote = "Control cannot be taken from another server; its operator releases it by disarming."
		view.Detail = fmt.Sprintf("The robot's servo host grant is held by %s (%s)%s, not by this server. This server cannot stop it: use that controller's Stop or the hardware stop.",
			servo.Controller, servoPrincipal(servo), sinceText(servo.Since))
		return view
	}
	if holder == nil || !(posture.inUse() || holder.inFlight > 0) {
		view.Detail = "Nobody is using the robot through this server."
		return view
	}
	owner := ownerOf(holder, now)
	view.Owner = &owner
	if holder.client.key == you.key {
		view.State, view.Detail = "yours", "You are using the robot."
		return view
	}
	view.State = "other"
	idle := now.Sub(holder.lastAction)
	switch {
	case posture.Armed:
		view.TakeControlNote = "It is armed: control is never taken from an armed controller. Ask its operator to disarm, or press Stop (which disarms it and releases control)."
	case holder.inFlight > 0:
		view.TakeControlNote = fmt.Sprintf("Its %s is still running.", holder.running)
	case idle < controlTakeoverIdle:
		view.TakeControlNote = fmt.Sprintf("It is disarmed; control can be taken once it has been idle for %s (%s more).", maintenanceDuration(controlTakeoverIdle), maintenanceDuration(controlTakeoverIdle-idle))
	default:
		view.TakeControl = true
		view.TakeControlNote = fmt.Sprintf("It is disarmed and has been idle for %s.", maintenanceDuration(idle))
	}
	view.Detail = fmt.Sprintf("The robot is in use by %s (%s)%s: %s. Stop always works.", owner.Label, ownerWhere(owner), sinceText(owner.Since), activityText(owner, posture))
	return view
}

func ownerOf(holder *controlHolder, now time.Time) ControlOwner {
	owner := ControlOwner{Kind: holder.client.Kind, Label: holder.client.Label, Address: holder.client.Address, Loopback: holder.client.Loopback,
		Origin: holder.client.Origin, Since: holder.since.UTC().Format(time.RFC3339), LastAction: holder.action, Running: holder.running}
	if !holder.lastAction.IsZero() {
		owner.LastActionAt = holder.lastAction.UTC().Format(time.RFC3339)
		owner.IdleMS = max(0, now.Sub(holder.lastAction).Milliseconds())
	}
	if holder.inFlight > 0 {
		owner.IdleMS = 0
	}
	return owner
}

func ownerWhere(owner ControlOwner) string {
	if owner.Loopback {
		return "from " + owner.Address + ", this host or a tunnel"
	}
	return "from " + owner.Address
}

func servoPrincipal(holder *control.MaintenanceHolder) string {
	if holder.Remote {
		return "remote " + firstNonEmpty(holder.Identity, "client")
	}
	return "local socket on the servo host"
}

func sinceText(iso string) string {
	if iso == "" {
		return ""
	}
	return " since " + iso
}

func activityText(owner ControlOwner, posture controlPosture) string {
	var parts []string
	if posture.Armed {
		parts = append(parts, "ARMED")
	} else if posture.LeaseHeld {
		parts = append(parts, "holds the control session, disarmed")
	}
	if owner.Running != "" {
		parts = append(parts, "running "+owner.Running)
	} else if owner.LastAction != "" {
		parts = append(parts, "last request "+owner.LastAction)
	}
	if posture.Session == "playing" || posture.Session == "loaded" {
		parts = append(parts, "program "+posture.Session)
	}
	if len(parts) == 0 {
		return "a request is running"
	}
	return strings.Join(parts, ", ")
}

// controlView is the rule applied to one requester now.
func (server *Server) controlView(ctx context.Context, you controlClient, posture controlPosture) ControlView {
	var holder *controlHolder
	if snapshot, ok := server.control.snapshot(); ok {
		holder = &snapshot
	}
	return controlViewOf(you, holder, posture, server.servoHolderElsewhere(ctx), server.control.now())
}

// controlAction classifies a mutating request: take=true for one that takes
// control or starts motion. Protective requests and everything that is not a
// robot request pass; anything else on the robot routes takes control,
// including routes added later (deny by default, as the maintenance gate).
// The body of an arm or target request is read and restored.
func controlAction(w http.ResponseWriter, r *http.Request) (action string, take bool, err error) {
	if r.Method == http.MethodGet || r.Method == http.MethodHead || r.Method == http.MethodOptions {
		return "", false, nil
	}
	action, robot := maintenanceRobotAction(r.URL.Path)
	if !robot || action == "" {
		return action, false, nil
	}
	// The cell registry is configuration; it grants no authority, and the
	// selected cell cannot be removed (dense_cells.go).
	if action == "cells" || strings.HasPrefix(action, "cells/") {
		return action, false, nil
	}
	if action == "arm" || action == "target" || action == "io/test" {
		raw, readErr := io.ReadAll(http.MaxBytesReader(w, r.Body, maxDenseRequestBytes))
		if readErr != nil {
			return action, false, readErr
		}
		r.Body = io.NopCloser(bytes.NewReader(raw))
		if maintenanceProtectiveBody(action, raw) {
			return action, false, nil
		}
	}
	return action, true, nil
}

// controlGate admits or refuses control requests (the rule above).
func (server *Server) controlGate(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		action, take, err := controlAction(w, r)
		if err != nil {
			writeDenseError(w, http.StatusBadRequest, "dense_request_invalid", "request body is invalid")
			return
		}
		if !take {
			next.ServeHTTP(w, r)
			return
		}
		identity := clientIdentityOf(r)
		if refused, err := server.clientRefusal(w, r, action, identity); err != nil {
			writeDenseError(w, http.StatusBadRequest, "dense_request_invalid", "request body is invalid")
			return
		} else if refused != nil {
			logClientRefusal(r, identity, refused)
			writeJSON(w, http.StatusConflict, refused)
			return
		}
		if server.control == nil {
			next.ServeHTTP(w, r)
			return
		}
		client := controlClientOf(r)
		client.who = identity.String()
		release, refusal := server.admitControl(r.Context(), client, action, false)
		if refusal != nil {
			log.Printf("control: refused %s %s from %s: %s", r.Method, r.URL.Path, client.who, refusal.Detail)
			writeJSON(w, http.StatusConflict, map[string]any{"error": controlRefusal, "detail": refusal.Detail, "control": refusal})
			return
		}
		defer release()
		next.ServeHTTP(w, r)
	})
}

// admitControl decides one request. Admitted: the requester is the owner
// with this request counted as running until release is called. Refused: the
// view to answer with. takeover: an explicit Take control (it may replace a
// disarmed idle holder; a request never does).
func (server *Server) admitControl(ctx context.Context, client controlClient, action string, takeover bool) (release func(), refusal *ControlView) {
	// The cell's own grant holder is a different server: never taken.
	// A select names the cell to observe next; the holder read belongs to
	// the one selected now.
	if action != "select" {
		if servo := server.servoHolderElsewhere(ctx); servo != nil {
			view := controlViewOf(client, nil, controlPosture{}, servo, server.control.now())
			return nil, &view
		}
	}
	c := server.control
	// The status read is I/O: it runs outside the lock, and a holder that
	// changed meanwhile is decided again.
	for attempt := 0; ; attempt++ {
		c.mu.Lock()
		owner := c.owner
		if owner == nil || owner.client.key == client.key {
			release = c.admitLocked(client, action, owner)
			c.mu.Unlock()
			return release, nil
		}
		c.mu.Unlock()
		posture := controlPostureOf(server.denseStatus(ctx))
		c.mu.Lock()
		if c.owner != owner && attempt < 3 {
			c.mu.Unlock()
			continue
		}
		now := c.now()
		free := owner.inFlight == 0 && !posture.inUse()
		if c.owner == owner && (free || takeover && owner.inFlight == 0 && !posture.Armed && now.Sub(owner.lastAction) >= controlTakeoverIdle) {
			if takeover && !free {
				log.Printf("control: %s took control from %s, disarmed and idle %s", client.describe(), owner.client.describe(), maintenanceDuration(now.Sub(owner.lastAction)))
			}
			release = c.admitLocked(client, action, owner)
			c.mu.Unlock()
			return release, nil
		}
		snapshot := *c.owner
		c.mu.Unlock()
		view := controlViewOf(client, &snapshot, posture, nil, now)
		if view.State != "other" {
			// The holder changed between reads more often than the bound:
			// refuse rather than guess, with what was seen.
			view.State, view.Detail = "other", "Another client is taking control of the robot right now; try again."
		}
		return nil, &view
	}
}

// admitLocked makes client the owner and counts its request; hold c.mu.
func (c *controlState) admitLocked(client controlClient, action string, previous *controlHolder) func() {
	now := c.now()
	holder := previous
	if holder == nil || holder.client.key != client.key {
		from := "free"
		if previous != nil {
			from = previous.client.Label
		}
		log.Printf("control: %s now controls the robot through this server (was %s)", client.describe(), from)
		holder = &controlHolder{client: client, since: now}
		c.owner = holder
	}
	holder.client = client
	holder.action, holder.lastAction = action, now
	if action != "take" {
		holder.inFlight++
		holder.running = action
	}
	var once sync.Once
	return func() {
		once.Do(func() {
			if action == "take" {
				return
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			holder.inFlight--
			holder.lastAction = c.now()
			if holder.inFlight == 0 {
				holder.running = ""
			}
		})
	}
}

// controlRoute is GET .../control: the rule as the requester sees it now.
func (server *Server) controlRoute(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Cache-Control", "no-store")
	writeJSON(w, http.StatusOK, server.controlView(r.Context(), controlClientOf(r), controlPostureOf(server.denseStatus(r.Context()))))
}

// controlTake is POST .../control/take: the deliberate handover. It succeeds
// when the robot is free or already the requester's, or when its holder is
// disarmed, has no request running and has been idle controlTakeoverIdle;
// otherwise it is refused like any control request. It moves nothing.
func (server *Server) controlTake(w http.ResponseWriter, r *http.Request) {
	client := controlClientOf(r)
	identity := clientIdentityOf(r)
	client.who = identity.String()
	if refused, _ := server.clientRefusal(w, r, "take control", identity); refused != nil {
		logClientRefusal(r, identity, refused)
		writeJSON(w, http.StatusConflict, refused)
		return
	}
	release, refusal := server.admitControl(r.Context(), client, "take", true)
	if refusal != nil {
		log.Printf("control: refused take control from %s: %s", client.who, refusal.Detail)
		writeJSON(w, http.StatusConflict, map[string]any{"error": controlRefusal, "detail": refusal.Detail, "control": refusal})
		return
	}
	release()
	writeJSON(w, http.StatusOK, server.controlView(r.Context(), client, controlPostureOf(server.denseStatus(r.Context()))))
}
