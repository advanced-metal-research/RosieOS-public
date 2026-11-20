package discovery

import (
	"context"
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"net/http/httptest"
	"strconv"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

const (
	pinA  = "87183b30e1b52c0f7d1a4a8f1c7a2f0e5b9d3c1a0f2e4d6c8b0a1928374655aa"
	capsA = "f02e9f057e6ae580ecfc2737e6a683bae4cfffe5c7935540684f28722eac084d"
)

func sampleAdvertisement() Advertisement {
	return Advertisement{
		Schema: Schema, Instance: "example-relay-olp-3f2a1c", Hostname: "example-relay", Port: 8794, Reach: "loopback",
		BuildID: "git:9dcf3158", StartedAt: "2026-10-08T07:12:00Z",
		Cells: []AdvertisedCell{{
			CellID: "example-arm-1420", Label: "rosie_1420_v1 (Rosie 1430)", Model: "rosie_1420_v1", Mode: "real",
			ConfigurationSHA256: pinA, CapabilitiesDigest: capsA, Selected: true,
		}},
	}
}

func TestDNSMessageRoundTripAndCompressedNames(t *testing.T) {
	msg := dnsMessage{ID: 7, Flags: flagResponse, Answers: []dnsRecord{
		{Name: serviceName, Type: typePTR, Class: classIN, TTL: 120, Target: "x." + serviceName},
		{Name: "x." + serviceName, Type: typeSRV, Class: classIN, TTL: 120, Port: 8794, Target: "x.local."},
		{Name: "x." + serviceName, Type: typeTXT, Class: classIN, TTL: 120, TXT: []string{"v=1", "id=x"}},
		{Name: "x.local.", Type: typeA, Class: classIN, TTL: 120, A: [4]byte{192, 168, 50, 121}},
	}}
	raw, err := msg.pack()
	if err != nil {
		t.Fatal(err)
	}
	got, err := unpack(raw)
	if err != nil || len(got.Answers) != 4 || got.Answers[1].Port != 8794 || got.Answers[2].TXT[1] != "id=x" || got.Answers[3].A != [4]byte{192, 168, 50, 121} {
		t.Fatalf("round trip: %+v %v", got, err)
	}

	// avahi compresses: a PTR whose target points back into the question name.
	compressed := []byte{0, 0, 0x84, 0, 0, 0, 0, 1, 0, 0, 0, 0}
	nameOffset := len(compressed)
	compressed, _ = appendName(compressed, serviceName)
	compressed = append(compressed, 0, 12, 0, 1, 0, 0, 0, 120)
	rdata := []byte{1, 'y', 0xC0, byte(nameOffset)}
	compressed = append(compressed, 0, byte(len(rdata)))
	compressed = append(compressed, rdata...)
	got, err = unpack(compressed)
	if err != nil || got.Answers[0].Target != "y."+serviceName {
		t.Fatalf("compressed PTR: %+v %v", got, err)
	}

	// A pointer to itself (or forwards) is a loop and must be refused, not followed.
	loop := []byte{0, 0, 0x84, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0xC0, 12, 0, 12, 0, 1}
	if _, err := unpack(loop); err == nil {
		t.Fatal("self-pointing name accepted")
	}
}

func TestTXTRoundTripDropsMalformedCellsAndTruncatesToOnePacket(t *testing.T) {
	a := sampleAdvertisement()
	got, dropped, err := ParseTXT(a.TXT(), 8794)
	if err != nil || dropped != 0 || len(got.Cells) != 1 || got.Cells[0] != a.Cells[0] || got.Instance != a.Instance || got.Reach != "loopback" {
		t.Fatalf("round trip: %+v dropped=%d err=%v", got, dropped, err)
	}

	txt := a.TXT()
	txt = append(txt,
		"c1=../etc|rosie_1420_v1|real|||0|bad id",
		"c2=bench-relay|rosie_1420_v1|real|NOTHEX|||0|bad pin",
		"c3=other-relay|rosie_1420_v1|armed||0|bad mode",
		"c4=example-arm-1420|rosie_1420_v1|real|||0|duplicate",
		"c5=example-minibench|bench_one_motor_1to1|simulation|||0|ok <b>\x07label</b>",
	)
	got, dropped, err = ParseTXT(txt, 8794)
	if err != nil || dropped != 4 || len(got.Cells) != 2 || got.Cells[1].CellID != "example-minibench" || strings.ContainsRune(got.Cells[1].Label, '\x07') {
		t.Fatalf("malformed cells: %+v dropped=%d err=%v", got.Cells, dropped, err)
	}

	if _, _, err := ParseTXT([]string{"v=2", "id=x"}, 1); err == nil {
		t.Fatal("unknown TXT version accepted")
	}

	many := sampleAdvertisement()
	many.Cells = nil
	for i := 0; i < 12; i++ {
		many.Cells = append(many.Cells, AdvertisedCell{CellID: fmt.Sprintf("cell-%d", i), Label: strings.Repeat("L", 64), Model: "rosie_1420_v1", Mode: "real", ConfigurationSHA256: pinA, CapabilitiesDigest: capsA})
	}
	encoded := many.TXT()
	size := 0
	for _, s := range encoded {
		if len(s) > 255 {
			t.Fatalf("TXT string over 255 bytes: %d", len(s))
		}
		size += len(s) + 1
	}
	if size > maxTXTBytes || encoded[len(encoded)-1] != "more=1" {
		t.Fatalf("TXT not bounded to one packet: %d bytes, last %q", size, encoded[len(encoded)-1])
	}
	got, _, _ = ParseTXT(encoded, 1)
	if !got.Truncated || len(got.Cells) == 0 || len(got.Cells) >= 12 {
		t.Fatalf("truncation not reported: truncated=%v cells=%d", got.Truncated, len(got.Cells))
	}
}

// A fake mDNS network on loopback: the responder's socket stands in for the
// group and Browse queries it from an ephemeral port, as on a LAN.
func startResponder(t *testing.T, record func() Advertisement) *net.UDPAddr {
	t.Helper()
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	r := NewResponder(ResponderOptions{Record: record, Group: conn.LocalAddr().(*net.UDPAddr)})
	go r.Serve(conn)
	t.Cleanup(r.Close)
	return conn.LocalAddr().(*net.UDPAddr)
}

func TestBrowseFindsAResponderAndDropsItself(t *testing.T) {
	group := startResponder(t, sampleAdvertisement)
	found, err := Browse(context.Background(), BrowseOptions{Group: group, Window: 300 * time.Millisecond})
	if err != nil || len(found) != 1 {
		t.Fatalf("browse: %+v %v", found, err)
	}
	f := found[0]
	if f.Address != "127.0.0.1" || f.Instance != "example-relay-olp-3f2a1c" || f.Port != 8794 || len(f.Cells) != 1 || f.Cells[0].ConfigurationSHA256 != pinA {
		t.Fatalf("found: %+v", f)
	}

	found, err = Browse(context.Background(), BrowseOptions{Group: group, Window: 300 * time.Millisecond, Self: "example-relay-olp-3f2a1c"})
	if err != nil || len(found) != 0 {
		t.Fatalf("own advertisement not dropped: %+v %v", found, err)
	}
}

func TestOneShotQueryGetsLegacyUnicastReply(t *testing.T) {
	group := startResponder(t, sampleAdvertisement)
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()
	query, _ := dnsMessage{ID: 0x4242, Questions: []dnsQuestion{{Name: serviceName, Type: typePTR, Class: classIN}}}.pack()
	if _, err := conn.WriteTo(query, group); err != nil {
		t.Fatal(err)
	}
	_ = conn.SetReadDeadline(time.Now().Add(time.Second))
	buf := make([]byte, maxPacket)
	n, _, err := conn.ReadFrom(buf)
	if err != nil {
		t.Fatal(err)
	}
	reply, err := unpack(buf[:n])
	if err != nil || reply.ID != 0x4242 || len(reply.Questions) != 1 {
		t.Fatalf("legacy reply must echo ID and question: %+v %v", reply, err)
	}
	for _, rec := range append(reply.Answers, reply.Extra...) {
		if rec.TTL > legacyTTL || rec.Class&classTopBit != 0 {
			t.Fatalf("legacy reply record %+v: TTL must be <= %d s and no cache-flush bit", rec, legacyTTL)
		}
	}

	// A query for any other name gets no answer at all.
	other, _ := dnsMessage{ID: 1, Questions: []dnsQuestion{{Name: "_http._tcp.local.", Type: typePTR, Class: classIN}}}.pack()
	_, _ = conn.WriteTo(other, group)
	_ = conn.SetReadDeadline(time.Now().Add(200 * time.Millisecond))
	if _, _, err := conn.ReadFrom(buf); err == nil {
		t.Fatal("responder answered a name it does not own")
	}
}

func TestResponderIgnoresQueriersOffTheAdvertisedSubnets(t *testing.T) {
	_, lan, _ := net.ParseCIDR("192.168.50.0/24")
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	r := NewResponder(ResponderOptions{
		Record:     sampleAdvertisement,
		Group:      conn.LocalAddr().(*net.UDPAddr),
		Interfaces: []Interface{{Name: "wlan0", IPv4: net.IPv4(192, 168, 50, 121).To4(), Net: lan}},
	})
	go r.Serve(conn)
	defer r.Close()
	// 127.0.0.1 stands for the servo host's link: not an advertised subnet.
	found, err := Browse(context.Background(), BrowseOptions{Group: conn.LocalAddr().(*net.UDPAddr), Window: 300 * time.Millisecond})
	if err != nil || len(found) != 0 {
		t.Fatalf("answered a querier off the advertised subnets: %+v %v", found, err)
	}
}

func TestSpoofedTXTForAnotherInstanceIsRefused(t *testing.T) {
	a := sampleAdvertisement()
	a.Instance = "someone-else-olp-000000"
	reply, _ := dnsMessage{Flags: flagResponse, Answers: []dnsRecord{
		{Name: serviceName, Type: typePTR, Class: classIN, TTL: 120, Target: "example-relay-olp-3f2a1c." + serviceName},
		{Name: "example-relay-olp-3f2a1c." + serviceName, Type: typeTXT, Class: classIN, TTL: 120, TXT: a.TXT()},
	}}.pack()
	if got := parseAnswer(reply, "192.168.50.9"); len(got) != 0 {
		t.Fatalf("TXT naming another instance accepted: %+v", got)
	}
	goodbye, _ := dnsMessage{Flags: flagResponse, Answers: []dnsRecord{
		{Name: serviceName, Type: typePTR, Class: classIN, TTL: 0, Target: "example-relay-olp-3f2a1c." + serviceName},
		{Name: "example-relay-olp-3f2a1c." + serviceName, Type: typeTXT, Class: classIN, TTL: 0, TXT: sampleAdvertisement().TXT()},
	}}.pack()
	if got := parseAnswer(goodbye, "192.168.50.9"); len(got) != 0 {
		t.Fatalf("goodbye (TTL 0) listed as present: %+v", got)
	}
}

func TestTailscaleProbesOnlineIPv4PeersAndValidates(t *testing.T) {
	var hits atomic.Int32
	good := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		hits.Add(1)
		if r.URL.Path != AdvertisementPath {
			http.NotFound(w, r)
			return
		}
		a := sampleAdvertisement()
		a.Cells = append(a.Cells, AdvertisedCell{CellID: "x", Model: "BAD MODEL", Mode: "real"})
		_ = json.NewEncoder(w).Encode(a)
	}))
	defer good.Close()
	_, portText, _ := net.SplitHostPort(good.Listener.Addr().String())
	port, _ := strconv.Atoi(portText)
	status := (`{"Peer":{
		"a":{"HostName":"example-relay","TailscaleIPs":["fd7a:115c::1","127.0.0.1"],"Online":true},
		"b":{"HostName":"asleep","TailscaleIPs":["127.0.0.2"],"Online":false},
		"c":{"HostName":"v6only","TailscaleIPs":["fd7a:115c::2"],"Online":true}}}`)
	found, err := BrowseTailscale(context.Background(), TailscaleOptions{
		Status: func(context.Context) ([]byte, error) { return []byte(status), nil },
		Port:   port,
	})
	// Port stays the advertised OLP API port, not the probed listener's.
	if err != nil || len(found) != 1 || found[0].Address != "127.0.0.1" || found[0].Dropped != 1 || len(found[0].Cells) != 1 || found[0].Port != 8794 {
		t.Fatalf("tailscale: %+v %v", found, err)
	}
	if hits.Load() != 1 {
		t.Fatalf("probed %d peers, want only the one online IPv4 peer", hits.Load())
	}
}

func TestServiceAnnotatesCachesAndProbesHealth(t *testing.T) {
	health := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != HealthPath {
			http.NotFound(w, r)
			return
		}
		_, _ = w.Write([]byte(`{"ok":true}`))
	}))
	defer health.Close()
	_, portText, _ := net.SplitHostPort(health.Listener.Addr().String())
	port, _ := strconv.Atoi(portText)

	lan := sampleAdvertisement()
	reachable := sampleAdvertisement()
	reachable.Instance, reachable.Hostname, reachable.Reach, reachable.Port = "deck-olp-aaaaaa", "steamdeck-pendant", "network", port
	var browses atomic.Int32
	now := time.Date(2026, 10, 8, 12, 0, 0, 0, time.UTC)
	service := NewService(Options{
		Interfaces: []Interface{{Name: "wlan0"}},
		Self:       "me-olp-000000",
		Tailscale:  &TailscaleOptions{},
		Browse: func(context.Context, BrowseOptions) ([]Found, error) {
			browses.Add(1)
			return []Found{{Advertisement: lan, Address: "192.168.50.121"}, {Advertisement: reachable, Address: "127.0.0.1"}}, nil
		},
		BrowseTailscale: func(context.Context, TailscaleOptions) ([]Found, error) {
			self := sampleAdvertisement()
			self.Instance = "me-olp-000000"
			return []Found{{Advertisement: lan, Address: "100.64.0.10"}, {Advertisement: self, Address: "100.1.1.1"}}, nil
		},
		Annotate: func(c AdvertisedCell) Here {
			return Here{Configured: true, Pin: "match", KnownManifest: true, CommissionedAddress: "https://127.0.0.1:18443", CommissionedHost: "127.0.0.1"}
		},
		Now: func() time.Time { return now },
	})
	result := service.Search(context.Background(), false)
	if !result.Enabled || len(result.Servers) != 2 || len(result.Sources) != 2 {
		t.Fatalf("result: %+v", result)
	}
	byInstance := map[string]Server{}
	for _, s := range result.Servers {
		byInstance[s.Instance] = s
	}
	j2 := byInstance["example-relay-olp-3f2a1c"]
	if j2.Source != "mdns" || j2.Address != "192.168.50.121" || j2.Health != "not_probed" || !j2.Cells[0].Here.Configured {
		t.Fatalf("a server heard on both sources is listed once, from mDNS, unprobed on loopback: %+v", j2)
	}
	if deck := byInstance["deck-olp-aaaaaa"]; deck.Health != "ok" {
		t.Fatalf("network-reachable server health: %+v", deck)
	}

	service.Search(context.Background(), false)
	now = now.Add(time.Second)
	service.Search(context.Background(), true)
	if browses.Load() != 1 {
		t.Fatalf("a fresh result and a forced search inside MinInterval must not search again: %d searches", browses.Load())
	}
	now = now.Add(3 * time.Second)
	service.Search(context.Background(), true)
	if browses.Load() != 2 {
		t.Fatalf("forced search after MinInterval did not search: %d", browses.Load())
	}

	recorder := httptest.NewRecorder()
	service.Handler()(recorder, httptest.NewRequest(http.MethodGet, DiscoveredCellsPath, nil))
	var body Result
	if err := json.Unmarshal(recorder.Body.Bytes(), &body); err != nil || body.Schema != ResultSchema || len(body.Servers) != 2 {
		t.Fatalf("route body: %s %v", recorder.Body.String(), err)
	}
}

func TestServiceOffSaysHowToTurnItOn(t *testing.T) {
	result := NewService(Options{}).Search(context.Background(), true)
	if result.Enabled || !strings.Contains(result.Reason, "OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES") || result.Servers == nil {
		t.Fatalf("disabled result: %+v", result)
	}
	recorder := httptest.NewRecorder()
	AdvertisementHandler(func() Advertisement { return Advertisement{} })(recorder, httptest.NewRequest(http.MethodGet, AdvertisementPath, nil))
	if recorder.Code != http.StatusNotFound {
		t.Fatalf("advertisement with discovery off: %d", recorder.Code)
	}
}

func TestNewInstanceIsAValidDNSLabel(t *testing.T) {
	for _, host := range []string{"example-relay", "Operator's MacBook Pro.local", "", strings.Repeat("x", 80)} {
		instance := NewInstance(host)
		if !instancePattern.MatchString(instance) || len(instance) > 63 {
			t.Fatalf("instance %q from %q", instance, host)
		}
	}
}
