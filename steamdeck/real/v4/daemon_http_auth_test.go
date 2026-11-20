package main

import (
	"io"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestDaemonHTTPAuthLoopbackClassification(t *testing.T) {
	robotV4UnsetDaemonWriteTokenForTest(t)
	for _, test := range []struct {
		host     string
		loopback bool
	}{
		{"127.0.0.1", true},
		{"127.5.6.7", true},
		{"::1", true},
		{"::ffff:127.0.0.1", true},
		{"::ffff:127.5.6.7", true},
		{"::ffff:127.0.0.0", true},
		{"::ffff:127.255.255.255", true},
		{"::ffff:126.255.255.255", false},
		{"::ffff:128.0.0.0", false},
		{"::ffff:10.0.0.1", false},
		{"10.0.0.1", false},
		{"fe80::1", false},
		{"fd00::1", false},
		{"::127.0.0.1", false},
		{"::", false},
		{"localhost", true},
	} {
		t.Run(test.host, func(t *testing.T) {
			request, err := http.NewRequest(http.MethodPost, "http://"+net.JoinHostPort(test.host, "8787")+"/api/processes/desired", nil)
			if err != nil {
				t.Fatal(err)
			}
			if got := robotV4DaemonRequestIsLoopback(request); got != test.loopback {
				t.Fatalf("loopback=%v, want %v", got, test.loopback)
			}
			err = robotV4AttachDaemonWriteAuthorization(request)
			if test.loopback {
				if err != nil {
					t.Fatalf("loopback write required token: %v", err)
				}
			} else if err == nil || !strings.Contains(err.Error(), robotV4DaemonWriteTokenFileEnv) {
				t.Fatalf("remote write did not require token configuration: %v", err)
			}
			if got := request.Header.Get("Authorization"); got != "" {
				t.Fatalf("unexpected authorization header %q", got)
			}
		})
	}
	if robotV4DaemonRequestIsLoopback(nil) || robotV4DaemonRequestIsLoopback(&http.Request{}) {
		t.Fatal("missing request URL classified as loopback")
	}
}

type robotV4DaemonAuthRoundTripper func(*http.Request) (*http.Response, error)

func (roundTrip robotV4DaemonAuthRoundTripper) RoundTrip(request *http.Request) (*http.Response, error) {
	return roundTrip(request)
}

func robotV4UnsetDaemonWriteTokenForTest(t *testing.T) {
	t.Helper()
	previous, configured := os.LookupEnv(robotV4DaemonWriteTokenFileEnv)
	if err := os.Unsetenv(robotV4DaemonWriteTokenFileEnv); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if configured {
			_ = os.Setenv(robotV4DaemonWriteTokenFileEnv, previous)
		} else {
			_ = os.Unsetenv(robotV4DaemonWriteTokenFileEnv)
		}
	})
}

func TestPostDaemonJSONRemoteRequiresConfiguredWriteTokenBeforeSending(t *testing.T) {
	robotV4UnsetDaemonWriteTokenForTest(t)
	requests := 0
	client := &http.Client{Transport: robotV4DaemonAuthRoundTripper(func(*http.Request) (*http.Response, error) {
		requests++
		return nil, nil
	})}

	_, err := postDaemonJSON(client, "http://build-standby.test:8787/api/deploy/build-request", []byte(`{}`))
	if err == nil || !strings.Contains(err.Error(), robotV4DaemonWriteTokenFileEnv) {
		t.Fatalf("expected missing token configuration error, got %v", err)
	}
	if requests != 0 {
		t.Fatalf("remote mutation was sent before authorization configuration: requests=%d", requests)
	}
}

func TestPostDaemonJSONRemoteAddsBearerWithoutLeakingIt(t *testing.T) {
	token := "0123456789abcdef0123456789abcdef"
	tokenPath := filepath.Join(t.TempDir(), "daemon-write-token")
	if err := os.WriteFile(tokenPath, []byte(token+"\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	t.Setenv(robotV4DaemonWriteTokenFileEnv, tokenPath)
	observedAuthorization := ""
	client := &http.Client{Transport: robotV4DaemonAuthRoundTripper(func(request *http.Request) (*http.Response, error) {
		observedAuthorization = request.Header.Get("Authorization")
		return &http.Response{
			StatusCode: http.StatusAccepted,
			Status:     "202 Accepted",
			Header:     make(http.Header),
			Body:       io.NopCloser(strings.NewReader(`{"ok":true}`)),
			Request:    request,
		}, nil
	})}

	response, err := postDaemonJSON(client, "http://build-standby.test:8787/api/deploy/build-request", []byte(`{}`))
	if err != nil {
		t.Fatalf("authorized daemon POST failed: %v", err)
	}
	if string(response) != `{"ok":true}` {
		t.Fatalf("unexpected response %q", response)
	}
	if observedAuthorization != "Bearer "+token {
		t.Fatalf("authorization header mismatch: %q", observedAuthorization)
	}
	client.Transport = robotV4DaemonAuthRoundTripper(func(request *http.Request) (*http.Response, error) {
		return &http.Response{
			StatusCode: http.StatusUnauthorized,
			Status:     "401 Unauthorized",
			Header:     make(http.Header),
			Body:       io.NopCloser(strings.NewReader("reflected=" + request.Header.Get("Authorization"))),
			Request:    request,
		}, nil
	})
	_, err = postDaemonJSON(client, "http://build-standby.test:8787/api/deploy/build-request", []byte(`{}`))
	if err == nil || strings.Contains(err.Error(), token) {
		t.Fatalf("daemon error did not redact reflected bearer token: %v", err)
	}

	if err := os.WriteFile(tokenPath, []byte("short-secret"), 0o600); err != nil {
		t.Fatal(err)
	}
	_, err = postDaemonJSON(client, "http://build-standby.test:8787/api/deploy/build-request", []byte(`{}`))
	if err == nil {
		t.Fatal("short daemon write token was accepted")
	}
	if strings.Contains(err.Error(), "short-secret") || strings.Contains(err.Error(), tokenPath) {
		t.Fatalf("daemon token error exposed secret material or token path: %v", err)
	}
}

func TestPostDaemonJSONRefusesRedirectWithoutForwardingBearer(t *testing.T) {
	token := "0123456789abcdef0123456789abcdef"
	tokenPath := filepath.Join(t.TempDir(), "daemon-write-token")
	if err := os.WriteFile(tokenPath, []byte(token+"\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	t.Setenv(robotV4DaemonWriteTokenFileEnv, tokenPath)
	requests := 0
	client := &http.Client{Transport: robotV4DaemonAuthRoundTripper(func(request *http.Request) (*http.Response, error) {
		requests++
		if requests > 1 {
			t.Fatalf("bearer followed redirect to %s with authorization=%q", request.URL, request.Header.Get("Authorization"))
		}
		return &http.Response{
			StatusCode: http.StatusTemporaryRedirect,
			Status:     "307 Temporary Redirect",
			Header: http.Header{
				"Location": []string{"http://redirected.daemon.test:9090/collect"},
			},
			Body:    io.NopCloser(strings.NewReader("redirect")),
			Request: request,
		}, nil
	})}

	_, err := postDaemonJSON(client, "http://daemon.test:8787/api/deploy/build-request", []byte(`{}`))
	if err == nil || !strings.Contains(err.Error(), "307 Temporary Redirect") {
		t.Fatalf("daemon mutation redirect did not fail closed: %v", err)
	}
	if strings.Contains(err.Error(), token) {
		t.Fatalf("daemon redirect error exposed bearer token: %v", err)
	}
	if requests != 1 {
		t.Fatalf("daemon redirect issued %d requests, want exactly one", requests)
	}
}

func TestDaemonHTTPHelpersKeepLoopbackWritesAndRemoteReadsCompatible(t *testing.T) {
	robotV4UnsetDaemonWriteTokenForTest(t)
	loopbackClient := &http.Client{Transport: robotV4DaemonAuthRoundTripper(func(request *http.Request) (*http.Response, error) {
		if request.Header.Get("Authorization") != "" {
			t.Errorf("loopback request unexpectedly carried authorization")
		}
		return &http.Response{
			StatusCode: http.StatusAccepted,
			Status:     "202 Accepted",
			Header:     make(http.Header),
			Body:       io.NopCloser(strings.NewReader(`{"ok":true}`)),
			Request:    request,
		}, nil
	})}
	if _, err := postDaemonJSON(loopbackClient, "http://127.0.0.1:8787/api/processes/desired", []byte(`{}`)); err != nil {
		t.Fatalf("loopback daemon write compatibility failed: %v", err)
	}

	remoteReads := 0
	client := &http.Client{Transport: robotV4DaemonAuthRoundTripper(func(request *http.Request) (*http.Response, error) {
		remoteReads++
		if request.Header.Get("Authorization") != "" {
			t.Errorf("read-only request unexpectedly carried authorization")
		}
		return &http.Response{
			StatusCode: http.StatusOK,
			Status:     "200 OK",
			Header:     make(http.Header),
			Body:       io.NopCloser(strings.NewReader(`{"ok":true}`)),
			Request:    request,
		}, nil
	})}
	if _, err := getDaemonJSON(client, "http://build-standby.test:8787/api/state"); err != nil {
		t.Fatalf("read-only daemon helper required write authorization: %v", err)
	}
	if remoteReads != 1 {
		t.Fatalf("expected one remote read, got %d", remoteReads)
	}
}
