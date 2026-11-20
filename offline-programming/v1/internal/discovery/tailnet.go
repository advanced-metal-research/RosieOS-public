package discovery

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"strconv"
	"sync"
	"time"
)

// DefaultTailnetPort is the advertisement listener's port on every OLP host
// of the fleet. The Tailscale search probes this port on each peer, so one
// value serves the whole tailnet.
const DefaultTailnetPort = 8798

// tailnetPrefix is Tailscale's IPv4 range (CGNAT 100.64.0.0/10).
var tailnetPrefix = &net.IPNet{IP: net.IPv4(100, 64, 0, 0).To4(), Mask: net.CIDRMask(10, 32)}

// TailnetAddress finds this host's Tailscale IPv4 on its interfaces
// (tailscale0 on Linux, a utun on macOS). The address is fixed per node, so
// binding it keeps the listener off every other network.
func TailnetAddress() (net.IP, error) {
	ifaces, err := net.Interfaces()
	if err != nil {
		return nil, err
	}
	for _, iface := range ifaces {
		if iface.Flags&net.FlagUp == 0 {
			continue
		}
		addrs, err := iface.Addrs()
		if err != nil {
			continue
		}
		for _, addr := range addrs {
			if ipnet, ok := addr.(*net.IPNet); ok && ipnet.IP.To4() != nil && tailnetPrefix.Contains(ipnet.IP.To4()) {
				return ipnet.IP.To4(), nil
			}
		}
	}
	return nil, errors.New("discovery: no Tailscale IPv4 (100.64.0.0/10) on any interface")
}

// AdvertisementHandler serves this server's record and nothing else: GET on
// AdvertisementPath only, no query, no cookies, no client bookkeeping. It is
// the whole surface of the tailnet listener, never the OLP API's handler.
func AdvertisementHandler(record func() Advertisement) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("X-Content-Type-Options", "nosniff")
		w.Header().Set("Cache-Control", "no-store")
		if r.URL.Path != AdvertisementPath || r.URL.RawQuery != "" {
			http.NotFound(w, r)
			return
		}
		if r.Method != http.MethodGet {
			w.Header().Set("Allow", http.MethodGet)
			http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
			return
		}
		a := record()
		if a.Instance == "" {
			http.NotFound(w, r)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(a)
	}
}

// TailnetListener serves the advertisement on this host's tailnet address.
// Tailscale may come up after the OLP (boot order) or restart; the listener
// retries every RetryEvery until it can bind, and again if serving stops.
type TailnetListener struct {
	Port       int
	Record     func() Advertisement
	Logf       func(string, ...any)
	RetryEvery time.Duration
	// Address finds the address to bind; nil is TailnetAddress. Tests bind loopback.
	Address func() (net.IP, error)

	mu      sync.Mutex
	server  *http.Server
	bound   string
	stopped bool
	stop    chan struct{}
	done    chan struct{}
}

// Start begins binding in the background and returns at once.
func (l *TailnetListener) Start() {
	if l.Address == nil {
		l.Address = TailnetAddress
	}
	if l.Logf == nil {
		l.Logf = func(string, ...any) {}
	}
	if l.RetryEvery <= 0 {
		l.RetryEvery = 10 * time.Second
	}
	l.stop, l.done = make(chan struct{}), make(chan struct{})
	go l.run()
}

// Bound is the address served now, empty while not bound.
func (l *TailnetListener) Bound() string {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.bound
}

func (l *TailnetListener) run() {
	defer close(l.done)
	lastErr := ""
	for {
		l.mu.Lock()
		if l.stopped {
			l.mu.Unlock()
			return
		}
		l.mu.Unlock()
		err := l.serveOnce()
		if err != nil && err.Error() != lastErr {
			l.Logf("discovery: tailnet advertisement listener: %v (retrying every %s)", err, l.RetryEvery)
			lastErr = err.Error()
		}
		select {
		case <-l.stop:
			return
		case <-time.After(l.RetryEvery):
		}
	}
}

func (l *TailnetListener) serveOnce() error {
	ip, err := l.Address()
	if err != nil {
		return err
	}
	address := net.JoinHostPort(ip.String(), strconv.Itoa(l.Port))
	listener, err := net.Listen("tcp4", address)
	if err != nil {
		return err
	}
	server := &http.Server{
		Handler:           AdvertisementHandler(l.Record),
		ReadHeaderTimeout: 2 * time.Second,
		ReadTimeout:       2 * time.Second,
		WriteTimeout:      2 * time.Second,
		IdleTimeout:       10 * time.Second,
		MaxHeaderBytes:    4 << 10,
	}
	l.mu.Lock()
	if l.stopped {
		l.mu.Unlock()
		listener.Close()
		return nil
	}
	l.server, l.bound = server, address
	l.mu.Unlock()
	l.Logf("discovery: advertisement only on http://%s%s (tailnet)", address, AdvertisementPath)
	err = server.Serve(listener)
	l.mu.Lock()
	l.server, l.bound = nil, ""
	l.mu.Unlock()
	if errors.Is(err, http.ErrServerClosed) {
		return nil
	}
	return fmt.Errorf("serve %s: %w", address, err)
}

// Close stops serving and waits for the loop to end.
func (l *TailnetListener) Close() {
	l.mu.Lock()
	if l.stopped {
		l.mu.Unlock()
		return
	}
	l.stopped = true
	server := l.server
	l.mu.Unlock()
	if l.stop != nil {
		close(l.stop)
	}
	if server != nil {
		ctx, cancel := context.WithTimeout(context.Background(), time.Second)
		_ = server.Shutdown(ctx)
		cancel()
	}
	if l.done != nil {
		<-l.done
	}
}
