package main

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"
)

// configFrom reads the camera's settings file values (systemd passes the
// same EnvironmentFile to both units): CELL_CAMERA_LISTEN and
// CELL_CAMERA_PATH_PREFIX name the camera, CELL_CAMERA_VIDEO_* the sender.
func configFrom(env func(string) string) (Config, error) {
	get := func(key, fallback string) string {
		if v := strings.TrimSpace(env("CELL_CAMERA_" + key)); v != "" {
			return v
		}
		return fallback
	}
	var c Config
	address := get("VIDEO_ADDRESS", "")
	c.Address = net.ParseIP(address)
	if c.Address == nil || c.Address.IsUnspecified() || c.Address.IsMulticast() {
		return c, fmt.Errorf("CELL_CAMERA_VIDEO_ADDRESS must be the host's Tailscale address, got %q", address)
	}
	port, err := strconv.Atoi(get("VIDEO_PORT", "40400"))
	if err != nil || port < 1024 || port > 65535 {
		return c, fmt.Errorf("CELL_CAMERA_VIDEO_PORT must be 1024..65535, got %q", get("VIDEO_PORT", ""))
	}
	c.Port = port
	c.Listen = get("VIDEO_LISTEN", "")
	host, _, err := net.SplitHostPort(c.Listen)
	if ip := net.ParseIP(host); err != nil || ip == nil || !ip.IsLoopback() {
		return c, fmt.Errorf("CELL_CAMERA_VIDEO_LISTEN must be a loopback host:port (the OLP relays it), got %q", c.Listen)
	}
	camera := get("LISTEN", "127.0.0.1:8088")
	c.Camera = "http://" + camera + strings.TrimRight(get("PATH_PREFIX", "/camera"), "/")
	return c, nil
}

// Handler is the WHEP interface (see sender.go).
func (s *Sender) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("POST /whep", func(w http.ResponseWriter, r *http.Request) {
		if !strings.HasPrefix(r.Header.Get("Content-Type"), "application/sdp") {
			http.Error(w, "expected an application/sdp offer", http.StatusBadRequest)
			return
		}
		body, err := io.ReadAll(io.LimitReader(r.Body, maxOfferBytes+1))
		if err != nil {
			http.Error(w, "reading the offer: "+err.Error(), http.StatusBadRequest)
			return
		}
		client := r.Header.Get("X-Rosie-Client")
		if client == "" {
			client = "unnamed client"
		}
		if len(client) > 80 {
			client = client[:80]
		}
		id, answer, err := s.Offer(string(body), client)
		var refused *Refusal
		switch {
		case errors.As(err, &refused):
			http.Error(w, refused.Reason, refused.Status)
			return
		case err != nil:
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}
		logf("viewer %s (%s) answered", id, client)
		w.Header().Set("Content-Type", "application/sdp")
		w.Header().Set("Location", "/whep/"+id)
		w.WriteHeader(http.StatusCreated)
		io.WriteString(w, answer)
	})
	mux.HandleFunc("DELETE /whep/{id}", func(w http.ResponseWriter, r *http.Request) {
		if !s.End(r.PathValue("id")) {
			http.Error(w, "no such video session", http.StatusNotFound)
			return
		}
		io.WriteString(w, "ended\n")
	})
	mux.HandleFunc("GET /status", func(w http.ResponseWriter, _ *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(s.Status())
	})
	return mux
}

// hostHas reports whether ip is assigned to one of this host's interfaces.
func hostHas(ip net.IP) bool {
	addrs, err := net.InterfaceAddrs()
	if err != nil {
		return false
	}
	for _, a := range addrs {
		if n, ok := a.(*net.IPNet); ok && n.IP.Equal(ip) {
			return true
		}
	}
	return false
}

func run(check bool) error {
	cfg, err := configFrom(os.Getenv)
	if err != nil {
		return err
	}
	if cfg.Address.IsLoopback() {
		return fmt.Errorf("CELL_CAMERA_VIDEO_ADDRESS %s is loopback: viewers could not reach it", cfg.Address)
	}
	if check {
		// install.sh runs this before it changes anything on the host.
		if !hostHas(cfg.Address) {
			return fmt.Errorf("CELL_CAMERA_VIDEO_ADDRESS %s is not an address of this host (tailscale ip -4)", cfg.Address)
		}
		fmt.Printf("cell-video settings ok: WebRTC on %s udp/%d, WHEP on %s, camera %s\n", cfg.Address, cfg.Port, cfg.Listen, cfg.Camera)
		return nil
	}
	conn, err := net.ListenUDP("udp", &net.UDPAddr{IP: cfg.Address, Port: cfg.Port})
	if err != nil {
		return fmt.Errorf("WebRTC port %s:%d: %w (is the address this host's Tailscale address?)", cfg.Address, cfg.Port, err)
	}
	sender, err := New(cfg, conn)
	if err != nil {
		return err
	}
	go func() {
		for now := range time.Tick(time.Second) {
			sender.Sweep(now)
		}
	}()
	logf("WHEP on http://%s/whep, WebRTC on %s udp/%d, camera %s", cfg.Listen, cfg.Address, cfg.Port, cfg.Camera)
	server := &http.Server{Addr: cfg.Listen, Handler: sender.Handler(), ReadHeaderTimeout: 5 * time.Second}
	return server.ListenAndServe()
}

func main() {
	check := len(os.Args) == 2 && os.Args[1] == "-check"
	if len(os.Args) > 1 && !check {
		fmt.Fprintln(os.Stderr, "usage: cell-video [-check]  (settings from CELL_CAMERA_* in the environment)")
		os.Exit(2)
	}
	if err := run(check); err != nil {
		fmt.Fprintln(os.Stderr, "cell-video:", err)
		os.Exit(2)
	}
}
