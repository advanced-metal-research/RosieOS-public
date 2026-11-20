package discovery

import (
	"context"
	"encoding/json"
	"io"
	"net"
	"net/http"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

// ResultSchema names the discovered-cells route's body.
const ResultSchema = "offline-programming.discovered-cells.v1"

// Here is what the browsing server makes of a discovered cell, from its own
// catalogue and the repository manifests only. Nothing here comes from the
// advertisement except the pin it is compared with.
type Here struct {
	// Configured: the cell is in this server's commissioned catalogue.
	Configured bool `json:"configured"`
	// Pin compares the advertised configuration pin with this server's
	// catalogue pin, or with the repository manifest's pin when the cell is
	// not configured here: "match", "differs" or "unknown".
	Pin string `json:"pin"`
	// KnownManifest: rt-core/config/cells has a manifest for this cell id.
	KnownManifest bool `json:"known_manifest"`
	// CommissionedAddress and CommissionedHost are the rt-control listener
	// the manifest or catalogue commissions; the Add form is filled from
	// these, never from the network.
	CommissionedAddress string `json:"commissioned_address,omitempty"`
	CommissionedHost    string `json:"commissioned_host,omitempty"`
	Reason              string `json:"reason,omitempty"`
}

// Cell is one discovered cell with this server's view of it.
type Cell struct {
	AdvertisedCell
	Here Here `json:"here"`
}

// Server is one discovered OLP server.
type Server struct {
	Instance    string `json:"instance"`
	Hostname    string `json:"hostname"`
	Address     string `json:"address"`
	Port        int    `json:"port"`
	Reach       string `json:"reach"`
	BuildID     string `json:"build_id"`
	StartedAt   string `json:"started_at"`
	Maintenance bool   `json:"maintenance"`
	Truncated   bool   `json:"truncated,omitempty"`
	Dropped     int    `json:"dropped_cells,omitempty"`
	// Source is "mdns" or "tailscale".
	Source string `json:"source"`
	// Health: "ok", "unhealthy", "unreachable", or "not_probed" for a server
	// that listens on loopback (reached through its tunnel only).
	Health       string `json:"health"`
	HealthDetail string `json:"health_detail,omitempty"`
	Cells        []Cell `json:"cells"`
}

// SourceStatus says what one source searched and whether it worked.
type SourceStatus struct {
	Name       string   `json:"name"`
	Interfaces []string `json:"interfaces,omitempty"`
	Found      int      `json:"found"`
	Error      string   `json:"error,omitempty"`
}

// Result is the discovered-cells route's body.
type Result struct {
	Schema     string         `json:"schema"`
	Enabled    bool           `json:"enabled"`
	Reason     string         `json:"reason,omitempty"`
	SearchedAt string         `json:"searched_at,omitempty"`
	Sources    []SourceStatus `json:"sources"`
	Servers    []Server       `json:"servers"`
}

// Options configures a Service.
type Options struct {
	// Interfaces are searched with mDNS; empty turns mDNS off.
	Interfaces []Interface
	// InterfaceError is why the configured interfaces could not be used.
	InterfaceError string
	// Self is this process's instance, left out of results.
	Self string
	// Tailscale enables the tailnet search (peers' advertisement listeners);
	// nil leaves it off.
	Tailscale *TailscaleOptions
	// Annotate says what this server knows of a cell id. Nil: nothing.
	Annotate func(AdvertisedCell) Here
	// HealthClient probes a network-reachable server's health route.
	HealthClient *http.Client
	Window       time.Duration
	// CacheFor serves a recent search without a new one (default 10 s);
	// MinInterval bounds forced searches (default 2 s).
	CacheFor, MinInterval time.Duration

	// Test seams.
	Browse          func(context.Context, BrowseOptions) ([]Found, error)
	BrowseTailscale func(context.Context, TailscaleOptions) ([]Found, error)
	Now             func() time.Time
}

// Service runs searches for the discovered-cells route, one at a time.
type Service struct {
	opts   Options
	mu     sync.Mutex
	last   *Result
	lastAt time.Time
}

func NewService(opts Options) *Service {
	if opts.Browse == nil {
		opts.Browse = Browse
	}
	if opts.BrowseTailscale == nil {
		opts.BrowseTailscale = BrowseTailscale
	}
	if opts.Now == nil {
		opts.Now = time.Now
	}
	if opts.CacheFor <= 0 {
		opts.CacheFor = 10 * time.Second
	}
	if opts.MinInterval <= 0 {
		opts.MinInterval = 2 * time.Second
	}
	if opts.HealthClient == nil {
		opts.HealthClient = &http.Client{Timeout: 800 * time.Millisecond}
	}
	return &Service{opts: opts}
}

// Enabled says whether any source is configured.
func (s *Service) Enabled() bool { return len(s.opts.Interfaces) > 0 || s.opts.Tailscale != nil }

func (s *Service) disabledReason() string {
	if s.opts.InterfaceError != "" {
		return "Network search is off on this server: " + s.opts.InterfaceError
	}
	return "Network search is off on this server: set OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES (operator LAN) or OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT (tailnet) (offline-programming/v1/README.md, Finding cells on the network)"
}

// Search returns the cached result while it is fresh, else searches. force
// searches again unless the last search is younger than MinInterval.
func (s *Service) Search(ctx context.Context, force bool) Result {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.opts.Now()
	if !s.Enabled() {
		return Result{Schema: ResultSchema, Reason: s.disabledReason(), Sources: []SourceStatus{}, Servers: []Server{}}
	}
	if s.last != nil {
		age := now.Sub(s.lastAt)
		if age < s.opts.MinInterval || (!force && age < s.opts.CacheFor) {
			return *s.last
		}
	}
	result := s.search(ctx)
	result.SearchedAt = s.opts.Now().UTC().Format(time.RFC3339)
	s.last, s.lastAt = &result, s.opts.Now()
	return result
}

func (s *Service) search(ctx context.Context) Result {
	result := Result{Schema: ResultSchema, Enabled: true, Sources: []SourceStatus{}, Servers: []Server{}}
	type sourced struct {
		Found
		source string
	}
	var all []sourced
	var wg sync.WaitGroup
	var mu sync.Mutex
	collect := func(name string, ifaces []string, found []Found, err error) {
		mu.Lock()
		defer mu.Unlock()
		status := SourceStatus{Name: name, Interfaces: ifaces, Found: len(found)}
		if err != nil {
			status.Error = err.Error()
		}
		result.Sources = append(result.Sources, status)
		for _, f := range found {
			all = append(all, sourced{Found: f, source: name})
		}
	}
	if len(s.opts.Interfaces) > 0 {
		names := make([]string, len(s.opts.Interfaces))
		for i, iface := range s.opts.Interfaces {
			names[i] = iface.Name
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			found, err := s.opts.Browse(ctx, BrowseOptions{Interfaces: s.opts.Interfaces, Window: s.opts.Window, Self: s.opts.Self})
			collect("mdns", names, found, err)
		}()
	}
	if s.opts.Tailscale != nil {
		wg.Add(1)
		go func() {
			defer wg.Done()
			found, err := s.opts.BrowseTailscale(ctx, *s.opts.Tailscale)
			var kept []Found
			for _, f := range found {
				if f.Instance != s.opts.Self {
					kept = append(kept, f)
				}
			}
			collect("tailscale", nil, kept, err)
		}()
	}
	wg.Wait()
	sort.Slice(result.Sources, func(i, j int) bool { return result.Sources[i].Name < result.Sources[j].Name })

	// mDNS first: one server heard both ways is listed once, as heard on the LAN.
	sort.SliceStable(all, func(i, j int) bool { return all[i].source == "mdns" && all[j].source != "mdns" })
	seen := map[string]bool{}
	for _, f := range all {
		if seen[f.Instance] {
			continue
		}
		seen[f.Instance] = true
		server := Server{
			Instance: f.Instance, Hostname: f.Hostname, Address: f.Address, Port: f.Port, Reach: f.Reach,
			BuildID: f.BuildID, StartedAt: f.StartedAt, Maintenance: f.Maintenance, Truncated: f.Truncated,
			Dropped: f.Dropped, Source: f.source, Cells: []Cell{},
		}
		for _, c := range f.Cells {
			here := Here{Pin: "unknown"}
			if s.opts.Annotate != nil {
				here = s.opts.Annotate(c)
			}
			server.Cells = append(server.Cells, Cell{AdvertisedCell: c, Here: here})
		}
		result.Servers = append(result.Servers, server)
	}
	var probes sync.WaitGroup
	for i := range result.Servers {
		probes.Add(1)
		go func() {
			defer probes.Done()
			result.Servers[i].Health, result.Servers[i].HealthDetail = s.probe(ctx, result.Servers[i])
		}()
	}
	probes.Wait()
	sort.SliceStable(result.Servers, func(i, j int) bool {
		a, b := result.Servers[i], result.Servers[j]
		if a.Hostname != b.Hostname {
			return a.Hostname < b.Hostname
		}
		return a.Instance < b.Instance
	})
	return result
}

// probe reads a network-reachable server's health route at the address the
// answer came from and the advertised port, on a fixed path. A loopback
// server is not probed: only its tunnel reaches it.
func (s *Service) probe(ctx context.Context, server Server) (string, string) {
	if server.Reach != "network" || server.Port == 0 || net.ParseIP(server.Address) == nil {
		return "not_probed", "listens on loopback; reach it through its tunnel"
	}
	url := "http://" + net.JoinHostPort(server.Address, strconv.Itoa(server.Port)) + HealthPath
	request, err := http.NewRequestWithContext(ctx, http.MethodGet, url, nil)
	if err != nil {
		return "unreachable", err.Error()
	}
	response, err := s.opts.HealthClient.Do(request)
	if err != nil {
		return "unreachable", "health route did not answer"
	}
	defer response.Body.Close()
	var body struct {
		OK bool `json:"ok"`
	}
	if response.StatusCode != http.StatusOK || json.NewDecoder(io.LimitReader(response.Body, 64<<10)).Decode(&body) != nil || !body.OK {
		return "unhealthy", "health route answered " + response.Status
	}
	return "ok", ""
}

// Handler serves GET discovered-cells; ?refresh=1 asks for a new search.
func (s *Service) Handler() http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		force := r.URL.Query().Get("refresh") == "1"
		ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
		defer cancel()
		writeJSON(w, s.Search(ctx, force))
	}
}

func writeJSON(w http.ResponseWriter, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Cache-Control", "no-store")
	encoder := json.NewEncoder(w)
	_ = encoder.Encode(value)
}

// ParseInterfaceList splits OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES.
func ParseInterfaceList(value string) []string {
	var names []string
	for _, name := range strings.Split(value, ",") {
		if name = strings.TrimSpace(name); name != "" {
			names = append(names, name)
		}
	}
	return names
}
