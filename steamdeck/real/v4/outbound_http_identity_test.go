package main

import (
	"errors"
	"net/http"
	"strings"
	"testing"
)

type robotV4OutboundHTTPTripwire struct {
	calls int
}

func (transport *robotV4OutboundHTTPTripwire) RoundTrip(*http.Request) (*http.Response, error) {
	transport.calls++
	return nil, errors.New("outbound HTTP tripwire reached")
}

func TestRobotV4BuildCacheRelayRejectsSemanticDestinationBeforeSourceFetch(t *testing.T) {
	tripwire := &robotV4OutboundHTTPTripwire{}
	client := &http.Client{Transport: tripwire}
	err := robotV4RelayBuildPeerCacheBlob(
		client,
		"http://jetson-test:8787",
		"http://realtime-core:8787",
		strings.Repeat("a", 64),
	)
	if err == nil || !strings.Contains(err.Error(), "semantic role/placeholder") {
		t.Fatalf("error = %v, want semantic destination rejection", err)
	}
	if tripwire.calls != 0 {
		t.Fatalf("semantic destination validation occurred after %d source request(s)", tripwire.calls)
	}
}
