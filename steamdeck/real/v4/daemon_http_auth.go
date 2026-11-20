package main

import (
	"errors"
	"net"
	"net/http"
	"os"
	"strings"
)

const (
	robotV4DaemonWriteTokenFileEnv  = "ROSIE_DAEMON_WRITE_TOKEN_FILE"
	robotV4MinDaemonWriteTokenBytes = 32
)

func robotV4AttachDaemonWriteAuthorization(request *http.Request) error {
	if robotV4DaemonRequestIsLoopback(request) {
		return nil
	}
	token, err := robotV4ReadDaemonWriteToken()
	if err != nil {
		return err
	}
	request.Header.Set("Authorization", "Bearer "+token)
	return nil
}

func robotV4DaemonMutationClient(client *http.Client) *http.Client {
	copy := *client
	copy.CheckRedirect = func(*http.Request, []*http.Request) error {
		return http.ErrUseLastResponse
	}
	return &copy
}

// robotV4DaemonRequestIsLoopback admits localhost and loopback IPs, including
// IPv4-mapped 127.0.0.0/8, so mapped remote hosts still require a write token.
func robotV4DaemonRequestIsLoopback(request *http.Request) bool {
	if request == nil || request.URL == nil {
		return false
	}
	host := strings.TrimSuffix(strings.ToLower(request.URL.Hostname()), ".")
	if host == "localhost" {
		return true
	}
	if index := strings.IndexByte(host, '%'); index >= 0 {
		host = host[:index]
	}
	ip := net.ParseIP(host)
	return ip != nil && ip.IsLoopback()
}

func robotV4ReadDaemonWriteToken() (string, error) {
	path, configured := os.LookupEnv(robotV4DaemonWriteTokenFileEnv)
	if !configured || strings.TrimSpace(path) == "" {
		return "", errors.New(robotV4DaemonWriteTokenFileEnv + " is required for non-loopback daemon writes")
	}
	token, err := os.ReadFile(path)
	if err != nil {
		return "", errors.New("daemon write token configuration is unavailable")
	}
	if len(token) > 0 && token[len(token)-1] == '\n' {
		token = token[:len(token)-1]
		if len(token) > 0 && token[len(token)-1] == '\r' {
			token = token[:len(token)-1]
		}
	}
	if len(token) < robotV4MinDaemonWriteTokenBytes {
		return "", errors.New("daemon write token configuration is unavailable")
	}
	for _, value := range token {
		if value < 0x21 || value > 0x7e {
			return "", errors.New("daemon write token configuration is unavailable")
		}
	}
	return string(token), nil
}

func robotV4RedactDaemonWriteAuthorization(message, authorization string) string {
	if authorization == "" {
		return message
	}
	redacted := strings.ReplaceAll(message, authorization, "Bearer [REDACTED]")
	if token, found := strings.CutPrefix(authorization, "Bearer "); found && token != "" {
		redacted = strings.ReplaceAll(redacted, token, "[REDACTED]")
	}
	return redacted
}
