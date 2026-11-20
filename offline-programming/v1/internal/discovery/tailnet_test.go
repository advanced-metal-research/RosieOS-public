package discovery

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

// The tailnet listener is the advertisement and nothing else: every other
// path (the OLP API's included), method or query is refused.
func TestAdvertisementHandlerServesOnlyTheAdvertisement(t *testing.T) {
	handler := AdvertisementHandler(sampleAdvertisement)
	get := func(method, target string) *httptest.ResponseRecorder {
		recorder := httptest.NewRecorder()
		handler(recorder, httptest.NewRequest(method, target, nil))
		return recorder
	}
	ok := get(http.MethodGet, AdvertisementPath)
	var a Advertisement
	if ok.Code != http.StatusOK || json.Unmarshal(ok.Body.Bytes(), &a) != nil || a.Instance != "example-relay-olp-3f2a1c" || ok.Header().Get("X-Content-Type-Options") != "nosniff" {
		t.Fatalf("advertisement: %d %s", ok.Code, ok.Body.String())
	}
	for _, path := range []string{"/", HealthPath, DiscoveredCellsPath, "/api/offline-programming/v1/targets", "/api/offline-programming/v1/dense-execution/status", AdvertisementPath + "/", AdvertisementPath + "?x=1"} {
		if code := get(http.MethodGet, path).Code; code != http.StatusNotFound {
			t.Errorf("GET %s: %d, want 404", path, code)
		}
	}
	for _, method := range []string{http.MethodPost, http.MethodPut, http.MethodDelete} {
		if code := get(method, AdvertisementPath).Code; code != http.StatusMethodNotAllowed {
			t.Errorf("%s: %d, want 405", method, code)
		}
	}
}

func freePort(t *testing.T) int {
	t.Helper()
	listener, err := net.Listen("tcp4", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	defer listener.Close()
	return listener.Addr().(*net.TCPAddr).Port
}

func waitBound(t *testing.T, l *TailnetListener) string {
	t.Helper()
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		if bound := l.Bound(); bound != "" {
			return bound
		}
		time.Sleep(10 * time.Millisecond)
	}
	t.Fatal("listener never bound")
	return ""
}

// Tailscale can come up after the OLP: the listener keeps trying until the
// address exists, binds only that address, and closes promptly.
func TestTailnetListenerWaitsForTheAddressAndBindsOnlyIt(t *testing.T) {
	port := freePort(t)
	var attempts atomic.Int32
	l := &TailnetListener{
		Port: port, Record: sampleAdvertisement, RetryEvery: 20 * time.Millisecond,
		Address: func() (net.IP, error) {
			if attempts.Add(1) < 3 {
				return nil, errors.New("no tailnet yet")
			}
			return net.IPv4(127, 0, 0, 1), nil
		},
	}
	l.Start()
	bound := waitBound(t, l)
	if bound != fmt.Sprintf("127.0.0.1:%d", port) || attempts.Load() < 3 {
		t.Fatalf("bound %q after %d attempts", bound, attempts.Load())
	}
	response, err := http.Get("http://" + bound + AdvertisementPath)
	if err != nil {
		t.Fatal(err)
	}
	body, _ := io.ReadAll(response.Body)
	response.Body.Close()
	if response.StatusCode != http.StatusOK || !strings.Contains(string(body), `"instance":"example-relay-olp-3f2a1c"`) {
		t.Fatalf("GET: %d %s", response.StatusCode, body)
	}
	started := time.Now()
	l.Close()
	if time.Since(started) > 2*time.Second {
		t.Fatalf("Close took %s", time.Since(started))
	}
	if _, err := http.Get("http://" + bound + AdvertisementPath); err == nil {
		t.Fatal("still serving after Close")
	}
}

// The Tailscale search reaches a peer's advertisement listener, not its OLP:
// a peer with only an OLP API on another port is not found.
func TestTailscaleSearchFindsAPeersAdvertisementListener(t *testing.T) {
	port := freePort(t)
	l := &TailnetListener{Port: port, Record: sampleAdvertisement, Address: func() (net.IP, error) { return net.IPv4(127, 0, 0, 1), nil }}
	l.Start()
	defer l.Close()
	waitBound(t, l)
	status := `{"Peer":{"example-relay":{"HostName":"example-relay","TailscaleIPs":["127.0.0.1"],"Online":true}}}`
	found, err := BrowseTailscale(context.Background(), TailscaleOptions{
		Status: func(context.Context) ([]byte, error) { return []byte(status), nil },
		Port:   port,
	})
	if err != nil || len(found) != 1 || found[0].Instance != "example-relay-olp-3f2a1c" || found[0].Address != "127.0.0.1" {
		t.Fatalf("found %+v %v", found, err)
	}
	found, err = BrowseTailscale(context.Background(), TailscaleOptions{
		Status: func(context.Context) ([]byte, error) { return []byte(status), nil },
		Port:   freePort(t),
	})
	if err != nil || len(found) != 0 {
		t.Fatalf("a peer with no listener on the port was listed: %+v %v", found, err)
	}
}
