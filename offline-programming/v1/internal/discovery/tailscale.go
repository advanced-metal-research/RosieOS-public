package discovery

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"os/exec"
	"sort"
	"strconv"
	"sync"
	"time"
)

// TailscaleOptions configures the tailnet search: the online peers in
// `tailscale status --json`, each asked for its advertisement on its
// advertisement-only listener (TailnetListener, DefaultTailnetPort). Tailscale
// carries no multicast, so mDNS cannot cross it; and an OLP API listens on
// loopback, so the probe never reaches one and never registers as its client.
type TailscaleOptions struct {
	// Status returns `tailscale status --json`. Nil runs the tailscale CLI.
	Status func(ctx context.Context) ([]byte, error)
	Client *http.Client
	// Port is the advertisement listener probed on each peer (DefaultTailnetPort).
	Port int
	// MaxPeers bounds one search (default 64); Concurrency bounds probes in
	// flight (default 16).
	MaxPeers, Concurrency int
}

type tailscaleStatus struct {
	Peer map[string]struct {
		HostName     string   `json:"HostName"`
		TailscaleIPs []string `json:"TailscaleIPs"`
		Online       bool     `json:"Online"`
	} `json:"Peer"`
}

// maxAdvertisementBytes bounds a probed JSON advertisement.
const maxAdvertisementBytes = 64 << 10

func runTailscaleStatus(ctx context.Context) ([]byte, error) {
	ctx, cancel := context.WithTimeout(ctx, 3*time.Second)
	defer cancel()
	return exec.CommandContext(ctx, "tailscale", "status", "--json").Output()
}

// BrowseTailscale probes each online peer's advertisement route.
func BrowseTailscale(ctx context.Context, opts TailscaleOptions) ([]Found, error) {
	status := opts.Status
	if status == nil {
		status = runTailscaleStatus
	}
	raw, err := status(ctx)
	if err != nil {
		return nil, fmt.Errorf("discovery: tailscale status: %w", err)
	}
	var parsed tailscaleStatus
	if err := json.Unmarshal(raw, &parsed); err != nil {
		return nil, fmt.Errorf("discovery: tailscale status: %w", err)
	}
	port := opts.Port
	if port <= 0 {
		port = DefaultTailnetPort
	}
	maxPeers := opts.MaxPeers
	if maxPeers <= 0 {
		maxPeers = 64
	}
	concurrency := opts.Concurrency
	if concurrency <= 0 {
		concurrency = 16
	}
	client := opts.Client
	if client == nil {
		// A peer without a listener refuses at once; an offline or filtered
		// one is cut at 700 ms. 40 peers at 16 in flight: about 2 s at worst.
		client = &http.Client{Timeout: 700 * time.Millisecond, CheckRedirect: func(*http.Request, []*http.Request) error { return http.ErrUseLastResponse }}
	}
	var addresses []string
	for _, peer := range parsed.Peer {
		if !peer.Online {
			continue
		}
		for _, ip := range peer.TailscaleIPs {
			if parsedIP := net.ParseIP(ip); parsedIP != nil && parsedIP.To4() != nil {
				addresses = append(addresses, parsedIP.To4().String())
				break
			}
		}
	}
	sort.Strings(addresses)
	if len(addresses) > maxPeers {
		addresses = addresses[:maxPeers]
	}
	results := make([]*Found, len(addresses))
	sem := make(chan struct{}, concurrency)
	var wg sync.WaitGroup
	for i, address := range addresses {
		wg.Add(1)
		go func() {
			defer wg.Done()
			sem <- struct{}{}
			defer func() { <-sem }()
			if f, err := fetchAdvertisement(ctx, client, address, port); err == nil {
				results[i] = f
			}
		}()
	}
	wg.Wait()
	var out []Found
	for _, f := range results {
		if f != nil {
			out = append(out, *f)
		}
	}
	return out, nil
}

func fetchAdvertisement(ctx context.Context, client *http.Client, address string, port int) (*Found, error) {
	url := "http://" + net.JoinHostPort(address, strconv.Itoa(port)) + AdvertisementPath
	request, err := http.NewRequestWithContext(ctx, http.MethodGet, url, nil)
	if err != nil {
		return nil, err
	}
	response, err := client.Do(request)
	if err != nil {
		return nil, err
	}
	defer response.Body.Close()
	if response.StatusCode != http.StatusOK {
		return nil, errors.New(response.Status)
	}
	var a Advertisement
	if err := json.NewDecoder(io.LimitReader(response.Body, maxAdvertisementBytes)).Decode(&a); err != nil {
		return nil, err
	}
	dropped, err := a.Validate()
	if err != nil {
		return nil, err
	}
	return &Found{Advertisement: a, Address: address, Dropped: dropped}, nil
}
