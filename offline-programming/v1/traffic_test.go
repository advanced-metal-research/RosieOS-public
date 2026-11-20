package main

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

// The meter counts what reaches the socket per client and route: the body
// after gzip and the header bytes, so a client's total matches the bytes it
// was sent, not the JSON before compression.
func TestTrafficMeterCountsWireBytesPerClientAndRoute(t *testing.T) {
	m := newTrafficMeter(time.Unix(0, 0))
	payload := strings.Repeat("x", 10000)
	h := m.meter(gzipResponses(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.Header().Set("Content-Type", "text/plain")
		io.WriteString(w, payload)
	})))
	var sizes []int
	for _, req := range []struct{ client, path, gzip string }{
		{"rosie-pendant steamdeck-viewer", "/api/offline-programming/v1/dense-execution/status", ""},
		{"rosie-pendant steamdeck-viewer", "/api/offline-programming/v1/dense-execution/status", ""},
		{"rosie-pendant steamdeck-pendant", "/api/offline-programming/v1/cell-cameras/default/frame.jpg", "gzip"},
	} {
		r := httptest.NewRequest("GET", req.path, nil)
		r.Header.Set("X-Rosie-Client", req.client)
		if req.gzip != "" {
			r.Header.Set("Accept-Encoding", "gzip")
		}
		rec := httptest.NewRecorder()
		h.ServeHTTP(rec, r)
		sizes = append(sizes, rec.Body.Len())
	}
	if sizes[2] >= 1000 {
		t.Fatalf("gzip did not compress the third reply (%d bytes)", sizes[2])
	}
	report := m.report(time.Unix(10, 0))
	byKey := map[string]trafficCount{}
	for _, c := range report.Counts {
		byKey[c.Client+"|"+c.Route] = c
	}
	status := byKey["rosie-pendant steamdeck-viewer|GET dense-execution/status"]
	if status.Requests != 2 || status.OutBytes < 2*10000 || status.OutBytes > 2*10000+400 {
		t.Fatalf("status count %+v, want 2 requests of 10000 B plus headers", status)
	}
	frame := byKey["rosie-pendant steamdeck-pendant|GET cell-cameras/default/frame.jpg"]
	if frame.Requests != 1 || frame.OutBytes < int64(sizes[2]) || frame.OutBytes > int64(sizes[2])+400 {
		t.Fatalf("frame count %+v, want the gzipped %d B plus headers", frame, sizes[2])
	}
	if status.InBytes <= 0 || report.OutBytes != status.OutBytes+frame.OutBytes {
		t.Fatalf("totals %+v", report)
	}
}

func TestTrafficRouteHidesIdentifiersAndNamesTheUI(t *testing.T) {
	for path, want := range map[string]string{
		"/api/offline-programming/v1/programs/0f3a9c2e-1d/revision":    "GET programs/*/revision",
		"/api/offline-programming/v1/cell-cameras/default/frame.jpg":   "GET cell-cameras/default/frame.jpg",
		"/api/offline-programming/v1/cell-camera/video.h264":           "GET cell-camera/video.h264",
		"/api/offline-programming/v1/a/b/c/d/e/f":                      "GET a/b/c/d/...",
		"/offline-programming/v1/ui/cameras.html":                      "ui",
		"/api/offline-programming/v1/dense-execution/telemetry/window": "GET dense-execution/telemetry/window",
	} {
		if got := trafficRoute(httptest.NewRequest("GET", path, nil)); got != want {
			t.Errorf("trafficRoute(%s) = %q, want %q", path, got, want)
		}
	}
}

// A stream shows in the counts while it runs: the Deck's video lasts hours.
func TestTrafficMeterCountsAStreamWhileItRuns(t *testing.T) {
	m := newTrafficMeter(time.Unix(0, 0))
	wrote, finish := make(chan struct{}), make(chan struct{})
	h := m.meter(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		io.WriteString(w, strings.Repeat("v", 5000))
		close(wrote)
		<-finish
	}))
	done := make(chan struct{})
	go func() {
		h.ServeHTTP(httptest.NewRecorder(), httptest.NewRequest("GET", "/api/offline-programming/v1/cell-camera/video.h264", nil))
		close(done)
	}()
	<-wrote
	counts := m.report(time.Unix(1, 0)).Counts
	close(finish)
	<-done
	if len(counts) != 1 || counts[0].Route != "GET cell-camera/video.h264" || counts[0].Requests != 1 || counts[0].OutBytes < 5000 {
		t.Fatalf("while streaming: %+v", counts)
	}
}

// The table is bounded: past trafficMaxKeys new keys fold into "other".
func TestTrafficMeterBoundsItsTable(t *testing.T) {
	m := newTrafficMeter(time.Unix(0, 0))
	for i := 0; i < trafficMaxKeys+50; i++ {
		m.add(fmt.Sprintf("client-%d", i), "GET x", 1, 1, 1)
	}
	r := m.report(time.Unix(1, 0))
	if len(r.Counts) != trafficMaxKeys+1 || r.OutBytes != trafficMaxKeys+50 {
		t.Fatalf("%d keys, %d bytes", len(r.Counts), r.OutBytes)
	}
}

func TestTrafficEndpointAnswersJSON(t *testing.T) {
	server := &Server{traffic: newTrafficMeter(time.Unix(0, 0))}
	server.traffic.add("c", "GET r", 1, 3, 4)
	rec := httptest.NewRecorder()
	server.trafficCounts(rec, httptest.NewRequest("GET", trafficAPIPath, nil))
	var got trafficReport
	if err := json.Unmarshal(rec.Body.Bytes(), &got); err != nil || got.OutBytes != 4 || len(got.Counts) != 1 {
		t.Fatalf("%v %+v", err, got)
	}
}
