package main

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/discovery"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
)

const (
	discoveryPinHere  = "87183b30e1b52c0f7d1a4a8f1c7a2f0e5b9d3c1a0f2e4d6c8b0a1928374655aa"
	discoveryPinOther = "a4a0721b00000000000000000000000000000000000000000000000000000000"
	discoveryCaps     = "f02e9f057e6ae580ecfc2737e6a683bae4cfffe5c7935540684f28722eac084d"
)

func discoveryTestState(t *testing.T) *cellDiscovery {
	t.Helper()
	d := newCellDiscovery(cellDiscoveryConfig{
		Listen: "127.0.0.1:8794",
		Catalogue: &targets.CellCatalogue{Schema: targets.CellCatalogueSchema, Cells: []targets.Cell{{
			CellID: "example-arm-1420", Label: "rosie_1420_v1", Role: "arm", Models: []string{"rosie_1420_v1"}, RequestedMode: "real",
			Execution: targets.CellExecution{RTCoreConfig: denseexec.RTCoreConfig{
				Address: "https://127.0.0.1:18443",
				Binding: control.Binding{PairID: "example-pair", Revision: 1, ConfigurationSHA256: discoveryPinHere},
			}},
		}}},
	}, new(strings.Builder))
	d.selected = func() (string, string, bool) { return "example-arm-1420", discoveryCaps, true }
	d.resolve = func(id string) (targets.CellConfiguration, error) {
		if id == "example-bench-1400" {
			return targets.CellConfiguration{ModelID: "bench_nine_motors_1to1", SHA256: discoveryPinOther, Address: "https://127.0.0.1:28443"}, nil
		}
		return targets.CellConfiguration{}, errors.New("cell_id_unknown")
	}
	t.Cleanup(d.Close)
	return d
}

func TestDiscoveryAdvertisesTheCatalogueAndTheSelectedCellsCapabilities(t *testing.T) {
	d := discoveryTestState(t)
	a := d.record()
	if a.Reach != "loopback" || a.Port != 8794 || len(a.Cells) != 1 || a.Instance == "" {
		t.Fatalf("record: %+v", a)
	}
	c := a.Cells[0]
	if c.CellID != "example-arm-1420" || c.Model != "rosie_1420_v1" || c.Mode != "real" || c.ConfigurationSHA256 != discoveryPinHere || !c.Selected || c.CapabilitiesDigest != discoveryCaps {
		t.Fatalf("advertised cell: %+v", c)
	}
	d.selected = func() (string, string, bool) { return "", "", false }
	if c := d.record().Cells[0]; c.Selected || c.CapabilitiesDigest != "" {
		t.Fatalf("an unselected cell has no described capabilities: %+v", c)
	}
	if _, _, err := discovery.ParseTXT(a.TXT(), a.Port); err != nil {
		t.Fatalf("own record does not survive TXT: %v", err)
	}
}

// The annotation is the browsing server's own knowledge: the address the Add
// form is filled with comes from its catalogue or manifest, never the network.
func TestDiscoveryAnnotatesFromThisServersCatalogueAndManifestsOnly(t *testing.T) {
	d := discoveryTestState(t)
	cases := []struct {
		name string
		cell discovery.AdvertisedCell
		want discovery.Here
		why  string
	}{
		{"configured, same pin", discovery.AdvertisedCell{CellID: "example-arm-1420", Model: "rosie_1420_v1", Mode: "real", ConfigurationSHA256: discoveryPinHere},
			discovery.Here{Configured: true, Pin: "match", KnownManifest: true, CommissionedAddress: "https://127.0.0.1:18443", CommissionedHost: "127.0.0.1"}, ""},
		{"configured, other pin", discovery.AdvertisedCell{CellID: "example-arm-1420", Model: "rosie_1420_v1", Mode: "real", ConfigurationSHA256: discoveryPinOther},
			discovery.Here{Configured: true, Pin: "differs", KnownManifest: true, CommissionedAddress: "https://127.0.0.1:18443", CommissionedHost: "127.0.0.1"}, ""},
		{"configured, other model", discovery.AdvertisedCell{CellID: "example-arm-1420", Model: "rosie_1400_v3", Mode: "real"},
			discovery.Here{Configured: true, Pin: "unknown", KnownManifest: true, CommissionedAddress: "https://127.0.0.1:18443", CommissionedHost: "127.0.0.1"}, "differs from this server's rosie_1420_v1"},
		{"manifest only", discovery.AdvertisedCell{CellID: "example-bench-1400", Model: "bench_nine_motors_1to1", Mode: "real", ConfigurationSHA256: discoveryPinOther},
			discovery.Here{Pin: "match", KnownManifest: true, CommissionedAddress: "https://127.0.0.1:28443", CommissionedHost: "127.0.0.1"}, "Not in this server's cell catalogue"},
		{"unknown id", discovery.AdvertisedCell{CellID: "made-up", Model: "rosie_1420_v1", Mode: "real", ConfigurationSHA256: discoveryPinHere},
			discovery.Here{Pin: "unknown"}, "No manifest"},
	}
	for _, tc := range cases {
		got := d.annotate(tc.cell)
		reason := got.Reason
		got.Reason = ""
		if got != tc.want || (tc.why != "" && !strings.Contains(reason, tc.why)) || (tc.why == "" && reason != "") {
			t.Errorf("%s: got %+v (reason %q), want %+v with reason containing %q", tc.name, got, reason, tc.want, tc.why)
		}
	}
}

func TestDiscoveredCellsRouteServesTheAnnotatedSearch(t *testing.T) {
	d := discoveryTestState(t)
	heard := discovery.Advertisement{
		Schema: discovery.Schema, Instance: "steamdeck-pendant-olp-abcdef", Hostname: "steamdeck-pendant", Port: 8794, Reach: "loopback",
		Cells: []discovery.AdvertisedCell{{CellID: "example-arm-1420", Model: "rosie_1420_v1", Mode: "real", ConfigurationSHA256: discoveryPinOther}},
	}
	d.service = discovery.NewService(discovery.Options{
		Interfaces: []discovery.Interface{{Name: "wlan0"}},
		Self:       d.instance,
		Annotate:   d.annotate,
		Browse: func(context.Context, discovery.BrowseOptions) ([]discovery.Found, error) {
			return []discovery.Found{{Advertisement: heard, Address: "192.168.50.77"}}, nil
		},
	})
	handler := NewServer(ServerOptions{Discovery: d}).Handler()
	recorder := httptest.NewRecorder()
	handler.ServeHTTP(recorder, httptest.NewRequest(http.MethodGet, "/api/offline-programming/v1/discovered-cells?refresh=1", nil))
	var body discovery.Result
	if recorder.Code != http.StatusOK || json.Unmarshal(recorder.Body.Bytes(), &body) != nil {
		t.Fatalf("route: %d %s", recorder.Code, recorder.Body.String())
	}
	if !body.Enabled || len(body.Servers) != 1 || body.Servers[0].Address != "192.168.50.77" || len(body.Servers[0].Cells) != 1 {
		t.Fatalf("body: %+v", body)
	}
	if here := body.Servers[0].Cells[0].Here; !here.Configured || here.Pin != "differs" || here.CommissionedAddress != "https://127.0.0.1:18443" {
		t.Fatalf("annotation: %+v", here)
	}
	if recorder.Header().Get("X-Content-Type-Options") == "" {
		t.Fatalf("route bypassed the server's security headers: %v", recorder.Header())
	}
}

func TestDiscoveryRoutesWithoutDiscoverySayItIsOff(t *testing.T) {
	handler := NewServer(ServerOptions{}).Handler()
	recorder := httptest.NewRecorder()
	handler.ServeHTTP(recorder, httptest.NewRequest(http.MethodGet, "/api/offline-programming/v1/discovered-cells", nil))
	var body discovery.Result
	if recorder.Code != http.StatusOK || json.Unmarshal(recorder.Body.Bytes(), &body) != nil || body.Enabled || !strings.Contains(body.Reason, "OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES") {
		t.Fatalf("discovered-cells without discovery: %d %s", recorder.Code, recorder.Body.String())
	}
	// The advertisement is served only by the tailnet listener, never by the API.
	recorder = httptest.NewRecorder()
	handler.ServeHTTP(recorder, httptest.NewRequest(http.MethodGet, "/api/offline-programming/v1/advertisement", nil))
	if recorder.Code != http.StatusNotFound {
		t.Fatalf("advertisement on the OLP API: %d", recorder.Code)
	}

	// A tailnet port equal to the API's, or out of range, leaves the tailnet off.
	for _, bad := range []string{"8794", "80", "x"} {
		d := newCellDiscovery(cellDiscoveryConfig{TailnetPort: bad, Listen: "127.0.0.1:8794"}, new(strings.Builder))
		if d.tailnet != nil || d.service.Enabled() {
			t.Fatalf("tailnet port %q accepted", bad)
		}
		d.Close()
	}

	// A configured interface that does not exist turns discovery off and says
	// so; it never fails the server.
	d := newCellDiscovery(cellDiscoveryConfig{Interfaces: "no-such-interface0", Listen: "127.0.0.1:8794"}, new(strings.Builder))
	defer d.Close()
	result := d.service.Search(context.Background(), true)
	if result.Enabled || !strings.Contains(result.Reason, "no-such-interface0") {
		t.Fatalf("bad interface: %+v", result)
	}
}
