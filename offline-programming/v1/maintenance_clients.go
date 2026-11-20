package main

import (
	"net/http"
	"net/url"
	"sort"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// Who is using this OLP server, for the maintenance overlay and the deploy
// tool's idle wait ("who to wait for"). It is built only from what requests
// already carry -- the peer address, Origin or Referer, User-Agent and the
// optional X-Rosie-Client label the Steam Deck sends -- so it identifies no
// one and requires nothing new of a client. Behind an SSH tunnel every peer is
// loopback; that is shown as such, and a client that names nothing is listed
// as unknown.

const (
	// A client is listed while it has made a request this recently: every
	// client polls at least every 2 s (maintenance poll; the Deck's status
	// poll is 250 ms). Unverified bound.
	maintenanceClientWindow = 60 * time.Second
	// A client whose heartbeat is this recent holds the OLP execution
	// session: the server's deadman stops the machine after 5 s of silence.
	maintenanceHeartbeatHolder = 5 * time.Second
	maintenanceClientLimit     = 64
)

type MaintenanceClient struct {
	Kind          string `json:"kind"` // browser, pendant, tool, unknown
	Label         string `json:"label"`
	Address       string `json:"address"`
	Loopback      bool   `json:"loopback"` // on this host or through a tunnel: the address does not tell machines apart
	Origin        string `json:"origin,omitempty"`
	FirstSeen     string `json:"first_seen"`
	LastSeen      string `json:"last_seen"`
	LastAction    string `json:"last_action,omitempty"` // the last robot request: "arm", "jog/begin", ...
	LastActionAt  string `json:"last_action_at,omitempty"`
	SessionHolder bool   `json:"session_holder"` // sends the execution-session heartbeat
	// What a pendant says about itself on its latest request
	// (client_identity.go): its process id, the cell its launcher was
	// started for and whether its window had input focus.
	PID        int    `json:"pid,omitempty"`
	ExpectCell string `json:"expect_cell,omitempty"`
	Focus      string `json:"focus,omitempty"`
	// Controller: the client using the robot through this server (control.go);
	// You: the client that asked for this list.
	Controller bool `json:"controller,omitempty"`
	You        bool `json:"you,omitempty"`
}

// MaintenanceMachine is the selected cell as this server sees it.
type MaintenanceMachine struct {
	Cell           string `json:"cell,omitempty"`
	Label          string `json:"label,omitempty"`
	Host           string `json:"host,omitempty"`
	SessionState   string `json:"session_state,omitempty"`
	HeartbeatAgeMS *int64 `json:"heartbeat_age_ms,omitempty"`
	Armed          *bool  `json:"armed,omitempty"`
	LeaseValid     *bool  `json:"lease_valid,omitempty"`
	AxisEnableMask uint32 `json:"axis_enable_mask"`
}

type clientRecord struct {
	MaintenanceClient
	key                            string
	first, last, action, heartbeat time.Time
}

type clientRegistry struct {
	mu      sync.Mutex
	now     func() time.Time
	clients map[string]*clientRecord
}

func newClientRegistry() *clientRegistry {
	return &clientRegistry{now: time.Now, clients: map[string]*clientRecord{}}
}

// classifyClient names a request's client from its headers only.
func classifyClient(r *http.Request) (kind, label, origin string) {
	origin = r.Header.Get("Origin")
	if origin == "" {
		if referer, err := url.Parse(r.Header.Get("Referer")); err == nil && referer.Host != "" {
			origin = referer.Scheme + "://" + referer.Host
		}
	}
	agent := r.Header.Get("User-Agent")
	if named := strings.TrimSpace(r.Header.Get("X-Rosie-Client")); named != "" {
		named = clipClientText(named)
		if strings.HasPrefix(named, "rosie-pendant") {
			return "pendant", named, origin
		}
		return "tool", named, origin
	}
	switch {
	case strings.HasPrefix(agent, "curl/"):
		return "tool", "curl (deploy tool or a script)", origin
	case strings.HasPrefix(agent, "Go-http-client"):
		return "tool", "Go HTTP client", origin
	case strings.Contains(agent, "Mozilla/"):
		label := "Browser (" + browserFamily(agent) + ")"
		if origin != "" {
			label += " on " + origin
		}
		return "browser", label, origin
	case agent != "":
		return "unknown", "Unknown client (" + clipClientText(agent) + ")", origin
	}
	return "unknown", "Unknown client (sends no User-Agent)", origin
}

func browserFamily(agent string) string {
	browser := "unknown browser"
	switch {
	case strings.Contains(agent, "Edg/"):
		browser = "Edge"
	case strings.Contains(agent, "Chrome/"):
		browser = "Chrome"
	case strings.Contains(agent, "Firefox/"):
		browser = "Firefox"
	case strings.Contains(agent, "Safari/"):
		browser = "Safari"
	}
	system := ""
	switch {
	case strings.Contains(agent, "Mac OS X"):
		system = " on macOS"
	case strings.Contains(agent, "Windows"):
		system = " on Windows"
	case strings.Contains(agent, "Android"):
		system = " on Android"
	case strings.Contains(agent, "Linux"):
		system = " on Linux"
	}
	return browser + system
}

func clipClientText(text string) string {
	text = strings.Map(func(r rune) rune {
		if r < 0x20 || r == 0x7f {
			return -1
		}
		return r
	}, text)
	if len(text) > 80 {
		return text[:79] + "…"
	}
	return text
}

// robotAction names a mutating robot request, or "" for anything else.
func robotAction(r *http.Request) string {
	if r.Method == http.MethodGet || r.Method == http.MethodHead || r.Method == http.MethodOptions {
		return ""
	}
	if r.URL.Path == "/api/offline-programming/v1/targets/select" {
		return "select cell"
	}
	if rest, ok := strings.CutPrefix(r.URL.Path, denseExecutionPath+"/"); ok {
		return rest
	}
	return ""
}

func (registry *clientRegistry) observe(r *http.Request) {
	// One record per controller identity (control.go): a tab or pendant that
	// sends a client id is its own line.
	client := controlClientOf(r)
	now := registry.now()
	action := robotAction(r)
	registry.mu.Lock()
	defer registry.mu.Unlock()
	record := registry.clients[client.key]
	if record == nil {
		registry.pruneLocked(now)
		if len(registry.clients) >= maintenanceClientLimit {
			return
		}
		record = &clientRecord{MaintenanceClient: client.MaintenanceClient, key: client.key, first: now}
		registry.clients[client.key] = record
	}
	record.last = now
	identity := clientIdentityOf(r)
	record.PID, record.ExpectCell, record.Focus = identity.PID, identity.ExpectCell, identity.Focus
	if action == "heartbeat" {
		record.heartbeat = now
	} else if action != "" {
		record.LastAction, record.action = action, now
	}
}

func (registry *clientRegistry) pruneLocked(now time.Time) {
	for key, record := range registry.clients {
		if now.Sub(record.last) > maintenanceClientWindow {
			delete(registry.clients, key)
		}
	}
}

// list is every client seen within the window, most recent first; the keys
// mark the controller and the asking client.
func (registry *clientRegistry) list(marks ...string) []MaintenanceClient {
	controller, you := "", ""
	if len(marks) > 0 {
		controller = marks[0]
	}
	if len(marks) > 1 {
		you = marks[1]
	}
	if registry == nil {
		return []MaintenanceClient{}
	}
	now := registry.now()
	registry.mu.Lock()
	defer registry.mu.Unlock()
	registry.pruneLocked(now)
	out := make([]MaintenanceClient, 0, len(registry.clients))
	records := make([]*clientRecord, 0, len(registry.clients))
	for _, record := range registry.clients {
		records = append(records, record)
	}
	sort.Slice(records, func(i, j int) bool { return records[i].last.After(records[j].last) })
	for _, record := range records {
		client := record.MaintenanceClient
		client.FirstSeen, client.LastSeen = record.first.UTC().Format(time.RFC3339), record.last.UTC().Format(time.RFC3339)
		if !record.action.IsZero() {
			client.LastActionAt = record.action.UTC().Format(time.RFC3339)
		}
		client.SessionHolder = !record.heartbeat.IsZero() && now.Sub(record.heartbeat) <= maintenanceHeartbeatHolder
		client.Controller = controller != "" && record.key == controller
		client.You = you != "" && record.key == you
		out = append(out, client)
	}
	return out
}

func (server *Server) trackClients(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if strings.HasPrefix(r.URL.Path, "/api/") {
			server.clients.observe(r)
			// The controller attributes a hold to the client whose press began
			// it, and a Stop or Disarm to whoever sent it (jog end reasons).
			client := denseexec.Client{Label: controlClientOf(r).Label}
			if id := strings.TrimSpace(r.Header.Get(clientIDHeader)); validClientID(id) {
				client.ID = id
			}
			r = r.WithContext(denseexec.WithClient(r.Context(), client))
		}
		next.ServeHTTP(w, r)
	})
}

// maintenanceMachine is the selected cell's posture from this server's own
// status body (its JSON, so it does not depend on the platform's rt-core types).
func maintenanceMachine(body *denseStatusJSON) *MaintenanceMachine {
	if body == nil || body.Target == nil {
		return nil
	}
	machine := &MaintenanceMachine{Cell: body.Target.CellID, Label: body.Target.Label, Host: body.Target.Host, Armed: body.Armed}
	if body.Session != nil {
		machine.SessionState, machine.HeartbeatAgeMS = body.Session.State, body.Session.HeartbeatAgeMS
	}
	if body.RTCore != nil && body.RTCore.Status != nil && body.RTCore.Status.Core != nil {
		core := body.RTCore.Status.Core
		armed, lease := core.Armed != 0, core.LeaseValid != 0
		machine.Armed, machine.LeaseValid, machine.AxisEnableMask = &armed, &lease, core.AxisEnableMask
	}
	return machine
}
