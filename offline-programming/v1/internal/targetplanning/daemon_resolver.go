package targetplanning

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/url"
	"strings"
	"time"
)

const maxDaemonStateBytes = 4 << 20

type DaemonNATSResolverOptions struct {
	StateURL string
	Client   *http.Client
}

type DaemonNATSResolver struct {
	stateURL string
	client   *http.Client
}

func NewDaemonNATSResolver(
	options DaemonNATSResolverOptions,
) (*DaemonNATSResolver, error) {
	stateURL, err := normalizeDaemonStateURL(options.StateURL)
	if err != nil {
		return nil, err
	}
	client := options.Client
	if client == nil {
		client = &http.Client{
			Timeout: 4 * time.Second,
			CheckRedirect: func(
				_ *http.Request,
				_ []*http.Request,
			) error {
				return http.ErrUseLastResponse
			},
		}
	}
	return &DaemonNATSResolver{stateURL: stateURL, client: client}, nil
}

func (resolver *DaemonNATSResolver) ResolveNATS(
	ctx context.Context,
	serviceRef string,
) (string, error) {
	if resolver == nil || resolver.client == nil {
		return "", errors.New("daemon-v1 resolver is unavailable")
	}
	peer, role, service, ok := parseDaemonServiceRef(serviceRef)
	if !ok {
		return "", errors.New("daemon-v1 NATS service reference is invalid")
	}
	request, err := http.NewRequestWithContext(
		ctx,
		http.MethodGet,
		resolver.stateURL,
		nil,
	)
	if err != nil {
		return "", err
	}
	request.Header.Set("Accept", "application/json")
	response, err := resolver.client.Do(request)
	if err != nil {
		return "", err
	}
	defer response.Body.Close()
	if response.StatusCode < 200 || response.StatusCode >= 300 {
		return "", fmt.Errorf(
			"daemon-v1 state returned HTTP %d",
			response.StatusCode,
		)
	}
	raw, err := io.ReadAll(
		io.LimitReader(response.Body, maxDaemonStateBytes+1),
	)
	if err != nil {
		return "", err
	}
	if len(raw) == 0 || len(raw) > maxDaemonStateBytes {
		return "", errors.New("daemon-v1 state is empty or exceeds its bound")
	}
	var state daemonState
	if err := json.Unmarshal(raw, &state); err != nil {
		return "", errors.New("daemon-v1 state is invalid")
	}
	for _, node := range state.Nodes {
		if node.Peer != peer ||
			(node.Role != role && node.Node != role) {
			continue
		}
		for _, endpoint := range node.ServiceEndpoints {
			if endpoint.Name != service ||
				endpoint.Protocol != "nats" ||
				!readyDaemonEndpoint(endpoint.Status) ||
				(endpoint.AdvertisedBy != "" &&
					endpoint.AdvertisedBy != peer) {
				continue
			}
			if endpoint.URL != serviceRef &&
				strings.HasPrefix(endpoint.URL, "daemon-v1://") {
				continue
			}
			if validConcreteNATSURL(endpoint.URL) {
				return endpoint.URL, nil
			}
			host := preferredDaemonHost(node.AdvertisedAddresses)
			if host == "" {
				continue
			}
			port := daemonListenPort(endpoint.Listen)
			return "nats://" + net.JoinHostPort(host, port), nil
		}
	}
	return "", errors.New(
		"fresh exact daemon-v1 NATS service endpoint was not found",
	)
}

type daemonState struct {
	Nodes []daemonStateNode `json:"nodes"`
}

type daemonStateNode struct {
	Peer                string                  `json:"peer"`
	Role                string                  `json:"role"`
	Node                string                  `json:"node"`
	AdvertisedAddresses []string                `json:"advertised_addresses"`
	ServiceEndpoints    []daemonServiceEndpoint `json:"service_endpoints"`
}

type daemonServiceEndpoint struct {
	Name         string `json:"name"`
	Protocol     string `json:"protocol"`
	URL          string `json:"url"`
	Listen       string `json:"listen"`
	Status       string `json:"status"`
	AdvertisedBy string `json:"advertised_by"`
}

func normalizeDaemonStateURL(raw string) (string, error) {
	raw = strings.TrimSpace(raw)
	if raw == "" {
		return "", errors.New("explicit daemon-v1 state URL is required")
	}
	parsed, err := url.Parse(raw)
	if err != nil ||
		(parsed.Scheme != "http" && parsed.Scheme != "https") ||
		parsed.User != nil ||
		strings.TrimSpace(parsed.Hostname()) == "" {
		return "", errors.New("daemon-v1 state URL must be concrete HTTP(S)")
	}
	parsed.RawQuery = ""
	parsed.Fragment = ""
	parsed.Path = strings.TrimRight(parsed.Path, "/")
	if !strings.HasSuffix(parsed.Path, "/api/state") {
		parsed.Path += "/api/state"
	}
	return parsed.String(), nil
}

func parseDaemonServiceRef(
	raw string,
) (peer string, role string, service string, ok bool) {
	const prefix = "daemon-v1://"
	if !strings.HasPrefix(raw, prefix) ||
		strings.ContainsAny(raw, " \t\r\n@?#") {
		return "", "", "", false
	}
	parts := strings.Split(strings.TrimPrefix(raw, prefix), "/")
	if len(parts) != 3 ||
		!concreteDaemonToken(parts[0]) ||
		!concreteDaemonToken(parts[1]) ||
		!concreteDaemonToken(parts[2]) ||
		parts[0] == parts[1] ||
		parts[1] == "service" {
		return "", "", "", false
	}
	return parts[0], parts[1], parts[2], true
}

func concreteDaemonToken(value string) bool {
	if value == "" || len(value) > 96 {
		return false
	}
	for _, char := range value {
		if (char >= 'a' && char <= 'z') ||
			(char >= 'A' && char <= 'Z') ||
			(char >= '0' && char <= '9') ||
			char == '-' || char == '_' || char == '.' {
			continue
		}
		return false
	}
	switch strings.ToLower(value) {
	case "manifest", "service":
		return false
	default:
		return true
	}
}

func readyDaemonEndpoint(status string) bool {
	switch strings.ToLower(strings.TrimSpace(status)) {
	case "ready", "running", "advertised":
		return true
	default:
		return false
	}
}

func preferredDaemonHost(addresses []string) string {
	for _, raw := range addresses {
		raw = strings.TrimSpace(raw)
		if raw == "" {
			continue
		}
		if parsed, err := url.Parse(raw); err == nil &&
			parsed.Hostname() != "" {
			if host := validDaemonHost(parsed.Hostname()); host != "" {
				return host
			}
		}
		host := raw
		if parsedHost, _, err := net.SplitHostPort(raw); err == nil {
			host = parsedHost
		}
		if value := validDaemonHost(host); value != "" {
			return value
		}
	}
	return ""
}

func validDaemonHost(raw string) string {
	host := strings.Trim(strings.TrimSpace(raw), "[]")
	if host == "" ||
		strings.ContainsAny(host, " /@?#\\") ||
		strings.EqualFold(host, "motion-server") ||
		strings.EqualFold(host, "realtime-core") ||
		strings.EqualFold(host, "manifest") {
		return ""
	}
	return host
}

func daemonListenPort(listen string) string {
	_, port, err := net.SplitHostPort(strings.TrimSpace(listen))
	if err == nil && port != "" {
		return port
	}
	return "4222"
}
