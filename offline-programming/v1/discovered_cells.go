package main

import (
	"fmt"
	"io"
	"log"
	"net"
	"net/http"
	"net/url"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/discovery"
	"rosieos/offline-programming/v1/internal/targets"
)

// cellDiscovery advertises this server's catalogue on the operator LAN and
// lists the OLP servers it hears (internal/discovery). Both halves are
// information only: an advertised cell is annotated from this server's own
// catalogue and the repository manifests, and adding one still goes through
// the commissioned-only Add cell and an authenticated Describe on Connect.
type cellDiscovery struct {
	instance string
	hostname string
	port     int
	reach    string
	cells    []targets.Cell
	// selected reports the selected cell and the capabilities digest its
	// last authenticated Describe returned. Nil: no picker.
	selected    func() (cellID, capabilities string, ok bool)
	maintenance func() bool
	resolve     func(cellID string) (targets.CellConfiguration, error)
	service     *discovery.Service
	responder   *discovery.Responder
	tailnet     *discovery.TailnetListener
	stop        chan struct{}
	stopped     sync.Once

	manifestMu sync.Mutex
	manifests  map[string]manifestLookup
}

type manifestLookup struct {
	binding targets.CellConfiguration
	err     error
}

// maxManifestLookups bounds the per-id cache; ids come from the network.
const maxManifestLookups = 64

type cellDiscoveryConfig struct {
	Interfaces string // OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES
	Advertise  string // OFFLINE_PROGRAMMING_DISCOVERY_ADVERTISE: "0" browses only
	// OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT: serve the advertisement on
	// this host's tailnet address at that port (unless Advertise is "0") and
	// search tailnet peers' listeners on the same port.
	TailnetPort string
	Listen      string
	Repo        string
	Catalogue   *targets.CellCatalogue
	Picker      *targets.Picker
	Maintenance *MaintenanceSource
}

func cellDiscoveryConfigFromEnv() cellDiscoveryConfig {
	return cellDiscoveryConfig{
		Interfaces:  os.Getenv("OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES"),
		Advertise:   strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_DISCOVERY_ADVERTISE")),
		TailnetPort: strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT")),
	}
}

// newCellDiscovery never fails the server: a bad interface or a socket that
// cannot bind turns discovery off and says why on the route and in the log.
func newCellDiscovery(cfg cellDiscoveryConfig, stdout io.Writer) *cellDiscovery {
	host, _ := os.Hostname()
	host = strings.Split(host, ".")[0]
	d := &cellDiscovery{
		instance:  discovery.NewInstance(host),
		hostname:  host,
		reach:     "network",
		stop:      make(chan struct{}),
		manifests: map[string]manifestLookup{},
	}
	if _, loopback := loopbackListenHost(cfg.Listen); loopback {
		d.reach = "loopback"
	}
	if _, portText, err := net.SplitHostPort(cfg.Listen); err == nil {
		d.port, _ = strconv.Atoi(portText)
	}
	if cfg.Catalogue != nil {
		d.cells = cfg.Catalogue.Cells
	}
	if cfg.Picker != nil {
		picker := cfg.Picker
		d.selected = func() (string, string, bool) {
			selected, ok := picker.SelectedRTCore()
			return selected.CellID, selected.Config.CapabilitiesDigest, ok
		}
	}
	if cfg.Maintenance != nil {
		source := cfg.Maintenance
		d.maintenance = func() bool { return source.Current().Active }
	}
	repo := cfg.Repo
	d.resolve = func(id string) (targets.CellConfiguration, error) { return targets.ResolveCellConfiguration(repo, id) }

	options := discovery.Options{Self: d.instance, Annotate: d.annotate}
	names := discovery.ParseInterfaceList(cfg.Interfaces)
	ifaces, err := discovery.ResolveInterfaces(names)
	if err != nil {
		options.InterfaceError = err.Error()
		fmt.Fprintf(stdout, "discovery: off (%v)\n", err)
		ifaces = nil
	}
	options.Interfaces = ifaces
	tailnetPort := 0
	if cfg.TailnetPort != "" {
		port, perr := strconv.Atoi(cfg.TailnetPort)
		if perr != nil || port < 1024 || port > 65535 || port == d.port {
			fmt.Fprintf(stdout, "discovery: tailnet off (OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT %q is not a port 1024-65535 other than the API's)\n", cfg.TailnetPort)
		} else {
			tailnetPort = port
			options.Tailscale = &discovery.TailscaleOptions{Port: port}
		}
	}
	if tailnetPort != 0 && cfg.Advertise != "0" {
		// Its own listener on the tailnet address: the advertisement only,
		// never the OLP API's handler, client list or maintenance view.
		d.tailnet = &discovery.TailnetListener{Port: tailnetPort, Record: d.record, Logf: log.Printf}
		d.tailnet.Start()
		fmt.Fprintf(stdout, "discovery: advertisement only on this host's tailnet address, port %d\n", tailnetPort)
	}
	if len(ifaces) > 0 && cfg.Advertise != "0" {
		conn, err := discovery.ListenMulticast(ifaces)
		if err != nil {
			fmt.Fprintf(stdout, "discovery: not advertising (%v)\n", err)
		} else {
			d.responder = discovery.NewResponder(discovery.ResponderOptions{Interfaces: ifaces, Record: d.record, Logf: log.Printf})
			go d.responder.Serve(conn)
			go d.reannounce()
			fmt.Fprintf(stdout, "discovery: advertising %s as %s on %s (reach %s)\n", discovery.ServiceType, d.instance, strings.Join(names, ","), d.reach)
		}
	}
	d.service = discovery.NewService(options)
	if d.service.Enabled() {
		sources := []string{}
		if len(ifaces) > 0 {
			sources = append(sources, "mDNS on "+strings.Join(names, ","))
		}
		if options.Tailscale != nil {
			sources = append(sources, fmt.Sprintf("Tailscale peers' advertisement listeners (port %d)", options.Tailscale.Port))
		}
		fmt.Fprintf(stdout, "discovery: Cells search uses %s\n", strings.Join(sources, " and "))
	} else if err == nil {
		fmt.Fprintln(stdout, "discovery: off (set OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES to the operator LAN interface)")
	}
	return d
}

// record is this server's advertisement now, read for every answer.
func (d *cellDiscovery) record() discovery.Advertisement {
	identity := maintenanceServerIdentity()
	a := discovery.Advertisement{
		Schema: discovery.Schema, Instance: d.instance, Hostname: d.hostname, Port: d.port, Reach: d.reach,
		BuildID: identity.BuildID, StartedAt: identity.StartedAt, Cells: []discovery.AdvertisedCell{},
	}
	if d.maintenance != nil {
		a.Maintenance = d.maintenance()
	}
	selectedID, capabilities := "", ""
	if d.selected != nil {
		if id, caps, ok := d.selected(); ok {
			selectedID, capabilities = id, caps
		}
	}
	for _, cell := range d.cells {
		model := ""
		if len(cell.Models) == 1 {
			model = cell.Models[0]
		}
		advertised := discovery.AdvertisedCell{
			CellID: cell.CellID, Label: cell.Label, Model: model, Mode: cell.RequestedMode,
			ConfigurationSHA256: cell.Execution.Binding.ConfigurationSHA256,
		}
		if cell.CellID == selectedID {
			advertised.Selected, advertised.CapabilitiesDigest = true, capabilities
		}
		a.Cells = append(a.Cells, advertised)
	}
	return a
}

// reannounce multicasts again when the record changes (a selection, a
// maintenance notice), at most every 5 s, so caches on the LAN follow.
func (d *cellDiscovery) reannounce() {
	d.responder.Announce()
	last := strings.Join(d.record().TXT(), "\x00")
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()
	for {
		select {
		case <-d.stop:
			return
		case <-ticker.C:
			if now := strings.Join(d.record().TXT(), "\x00"); now != last {
				last = now
				d.responder.Announce()
			}
		}
	}
}

// annotate says what this server knows of an advertised cell, from its own
// catalogue first and the repository manifest second. The commissioned
// address comes from those, never from the advertisement.
func (d *cellDiscovery) annotate(c discovery.AdvertisedCell) discovery.Here {
	here := discovery.Here{Pin: "unknown"}
	comparePin := func(local string) {
		switch {
		case c.ConfigurationSHA256 == "" || local == "":
			here.Pin = "unknown"
		case c.ConfigurationSHA256 == local:
			here.Pin = "match"
		default:
			here.Pin = "differs"
		}
	}
	for _, cell := range d.cells {
		if cell.CellID != c.CellID {
			continue
		}
		// A remote catalogue entry is admitted only bound to its manifest.
		here.Configured, here.KnownManifest = true, !cell.Execution.Local
		if cell.Execution.Local {
			here.CommissionedHost = "localhost"
			here.Reason = "Local simulation on this server"
		} else {
			here.CommissionedAddress = cell.Execution.Address
			if u, err := url.Parse(cell.Execution.Address); err == nil {
				here.CommissionedHost = u.Hostname()
			}
		}
		local := cell.Execution.Binding.ConfigurationSHA256
		comparePin(local)
		if !cell.Execution.Local && local == "" {
			here.Reason = "In this server's catalogue, but its native configuration is missing here (cell_configuration_missing)"
		} else if len(cell.Models) == 1 && cell.Models[0] != c.Model {
			here.Reason = fmt.Sprintf("Advertised model %s differs from this server's %s", c.Model, cell.Models[0])
		}
		return here
	}
	lookup := d.lookupManifest(c.CellID)
	if lookup.err != nil {
		if lookup.err.Error() == "cell_id_unknown" {
			here.Reason = "No manifest for this cell id in rt-core/config/cells"
			return here
		}
		here.KnownManifest = true
		here.Reason = "Its manifest does not resolve here: " + lookup.err.Error()
		return here
	}
	here.KnownManifest = true
	here.CommissionedAddress = lookup.binding.Address
	if u, err := url.Parse(lookup.binding.Address); err == nil {
		here.CommissionedHost = u.Hostname()
	}
	comparePin(lookup.binding.SHA256)
	here.Reason = "Not in this server's cell catalogue: commission it here (an olp-cells.json entry and its PKI files) before Add can register it"
	if lookup.binding.ModelID != c.Model {
		here.Reason = fmt.Sprintf("Advertised model %s differs from the manifest's %s. ", c.Model, lookup.binding.ModelID) + here.Reason
	}
	return here
}

// lookupManifest compiles a cell manifest once per id; unknown ids are
// refused by name before any file is read.
func (d *cellDiscovery) lookupManifest(id string) manifestLookup {
	d.manifestMu.Lock()
	defer d.manifestMu.Unlock()
	if cached, ok := d.manifests[id]; ok {
		return cached
	}
	binding, err := d.resolve(id)
	lookup := manifestLookup{binding: binding, err: err}
	if len(d.manifests) < maxManifestLookups {
		d.manifests[id] = lookup
	}
	return lookup
}

func (d *cellDiscovery) Close() {
	if d == nil {
		return
	}
	d.stopped.Do(func() {
		close(d.stop)
		if d.responder != nil {
			d.responder.Close()
		}
		if d.tailnet != nil {
			d.tailnet.Close()
		}
	})
}

func (server *Server) discoveredCells(w http.ResponseWriter, r *http.Request) {
	if server.discovery == nil {
		discovery.NewService(discovery.Options{}).Handler()(w, r)
		return
	}
	server.discovery.service.Handler()(w, r)
}
