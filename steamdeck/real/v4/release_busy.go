package main

// Read the dashboard's minimal busy projection before a managed release.
// This is a cooperative notice, not an execution lease or host-write fence.

import (
	"encoding/json"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/url"
	"strings"
	"time"
)

type releaseBusyView struct {
	Schema             string    `json:"schema"`
	GeneratedAt        time.Time `json:"generatedAt"`
	AuthorityCeiling   string    `json:"authorityCeiling"`
	HostWriteAssurance string    `json:"hostWriteAssurance"`
	Cells              map[string]struct {
		State       string `json:"state"`
		Reservation string `json:"reservation"`
		Controller  string `json:"controller"`
	} `json:"cells"`
}

// releaseBusyPreflight refuses a candidate when the configured bench view is
// busy, unknown, stale, malformed, or unavailable.
func releaseBusyPreflight(rawURL, cellID string) error {
	u, err := url.Parse(rawURL)
	if err != nil || u == nil || (u.Scheme != "http" && u.Scheme != "https") ||
		u.Host == "" || u.User != nil || u.RawQuery != "" || u.Fragment != "" || u.Path != "/api/busy" {
		return fmt.Errorf("--busy-status-url must be a dashboard /api/busy HTTP URL")
	}
	host := strings.ToLower(u.Hostname())
	ip := net.ParseIP(host)
	allowed := host == "localhost" || strings.HasSuffix(host, ".ts.net") ||
		(ip != nil && (ip.IsLoopback() || ipInTailnet(ip)))
	if !allowed {
		return fmt.Errorf("busy status host must be local or on the tailnet")
	}
	client := &http.Client{Timeout: 3 * time.Second,
		Transport: &http.Transport{Proxy: nil},
		CheckRedirect: func(*http.Request, []*http.Request) error {
			return fmt.Errorf("busy status redirects are not allowed")
		}}
	response, err := client.Get(rawURL)
	if err != nil {
		return fmt.Errorf("busy status unavailable: %w", err)
	}
	defer response.Body.Close()
	if response.StatusCode != http.StatusOK {
		return fmt.Errorf("busy status returned HTTP %d", response.StatusCode)
	}
	raw, err := io.ReadAll(io.LimitReader(response.Body, 64*1024+1))
	if err != nil || len(raw) > 64*1024 {
		return fmt.Errorf("busy status body is unreadable or too large")
	}
	var view releaseBusyView
	if json.Unmarshal(raw, &view) != nil || view.Schema != "rosie.bench-busy.v1" ||
		view.AuthorityCeiling != "coordination_only" || view.HostWriteAssurance != "cooperative" {
		return fmt.Errorf("busy status contract differs")
	}
	age := time.Since(view.GeneratedAt)
	if view.GeneratedAt.IsZero() || age < 0 || age > 10*time.Second {
		return fmt.Errorf("busy status is stale or has an invalid clock")
	}
	cell, ok := view.Cells[cellID]
	if !ok || cell.State != "free" || cell.Reservation != "none" || cell.Controller != "idle" {
		return fmt.Errorf("cell %s is not confirmed free (state=%q reservation=%q controller=%q)",
			cellID, cell.State, cell.Reservation, cell.Controller)
	}
	return nil
}

func ipInTailnet(ip net.IP) bool {
	addr := ip.To4()
	return addr != nil && addr[0] == 100 && addr[1] >= 64 && addr[1] <= 127
}
