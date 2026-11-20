//go:build linux

package main

import (
	"bytes"
	"compress/gzip"
	"context"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/rt-core/sdk/control"
)

type controlStatusBackend struct {
	denseexec.Backend
	body *denseexec.StatusBody
}

func (b *controlStatusBackend) Status(context.Context) *denseexec.StatusBody { return b.body }

type currentControlStatusBackend struct {
	*controlStatusBackend
	fullReads, controlReads int
}

func (b *currentControlStatusBackend) Status(ctx context.Context) *denseexec.StatusBody {
	b.fullReads++
	return b.controlStatusBackend.Status(ctx)
}

func (b *currentControlStatusBackend) ControlStatus(context.Context) *denseexec.StatusBody {
	b.controlReads++
	return denseexec.ControlStatusBody(b.body)
}

func TestControlStatusHTTPSelectsCurrentReadBeforeBackendIO(t *testing.T) {
	b := &currentControlStatusBackend{controlStatusBackend: &controlStatusBackend{body: controlStatusFixture(t)}}
	server := NewServer(ServerOptions{DenseExecution: b}).Handler()
	for _, query := range []string{"?view=control", "", "?view=unknown"} {
		response := httptest.NewRecorder()
		server.ServeHTTP(response, httptest.NewRequest(http.MethodGet, denseExecutionPath+"/status"+query, nil))
		var got denseexec.StatusBody
		if err := json.Unmarshal(response.Body.Bytes(), &got); err != nil {
			t.Fatal(err)
		}
		if response.Code != http.StatusOK || b.controlReads != 1 {
			t.Fatalf("query %q: status=%d full=%d current=%d", query, response.Code, b.fullReads, b.controlReads)
		}
		if query == "?view=control" {
			if b.fullReads != 0 || len(got.RTCore.Status.Execution.Handles) != 0 {
				t.Fatal("control view fetched full history before filtering it")
			}
		} else if len(got.RTCore.Status.Execution.Handles) != 10000 {
			t.Fatal("ordinary diagnostic view lost history")
		}
	}
	if b.fullReads != 2 {
		t.Fatalf("diagnostic reads=%d, want 2", b.fullReads)
	}
}

func controlStatusFixture(t testing.TB) *denseexec.StatusBody {
	t.Helper()
	raw, err := os.ReadFile("../../steamdeck/real/v5/tests/fixtures/example-live/status.json")
	if err != nil {
		t.Fatal(err)
	}
	var body denseexec.StatusBody
	if err := json.Unmarshal(raw, &body); err != nil {
		t.Fatal(err)
	}
	body.RTCore.Status.Execution.Handles = nil
	for i := uint64(1); i <= 10000; i++ {
		body.RTCore.Status.Execution.Handles = append(body.RTCore.Status.Execution.Handles, control.HandleRecord{Handle: i, State: "consumed"})
	}
	return &body
}

func TestControlStatusHTTPPreservesFullDiagnosticsAndNegotiatesGzip(t *testing.T) {
	body := controlStatusFixture(t)
	server := NewServer(ServerOptions{DenseExecution: &controlStatusBackend{body: body}}).Handler()
	full, _ := json.Marshal(body)
	compact, _ := json.Marshal(denseexec.ControlStatusBody(body))
	for _, test := range []struct {
		query, encoding string
		compressed      bool
	}{
		{"", "", false},
		{"", "gzip", true},
		{"?view=control", "", false},
		{"?view=control", "br, gzip", true},
		{"?view=control", "gzip;q=0", false},
		{"?view=control", "*;q=1, gzip;q=0", false},
		{"?view=control", "gzip; q=0.5", true},
		{"?view=control", "gzip;q=NaN", false},
		{"?view=control", "gzip;q=2", false},
		{"?view=control", "*;q=0.5", true},
	} {
		t.Run(test.query+"/"+test.encoding, func(t *testing.T) {
			request := httptest.NewRequest(http.MethodGet, denseExecutionPath+"/status"+test.query, nil)
			if test.encoding != "" {
				request.Header.Set("Accept-Encoding", test.encoding)
			}
			response := httptest.NewRecorder()
			server.ServeHTTP(response, request)
			if response.Code != http.StatusOK || response.Header().Get("Cache-Control") != "no-store" {
				t.Fatalf("status=%d headers=%v", response.Code, response.Header())
			}
			if test.query != "" && response.Header().Get("Vary") != "Accept-Encoding" {
				t.Fatalf("missing encoding variation: %v", response.Header())
			}
			if (response.Header().Get("Content-Encoding") == "gzip") != test.compressed {
				t.Fatalf("wrong compression negotiation: %v", response.Header())
			}
			decoded := response.Body.Bytes()
			if test.compressed {
				zr, err := gzip.NewReader(bytes.NewReader(decoded))
				if err != nil {
					t.Fatal(err)
				}
				decoded, err = io.ReadAll(zr)
				zr.Close()
				if err != nil {
					t.Fatal(err)
				}
				// Absolute regression budget on the checked-in six-axis fixture,
				// not a threshold derived from this response's own size.
				if test.query != "" && response.Body.Len() > 12000 {
					t.Fatalf("control response exceeded 12 KB wire budget: %d", response.Body.Len())
				}
				t.Logf("full=%d compact=%d compressed=%d bytes", len(full), len(compact), response.Body.Len())
			}
			want := compact
			if test.query == "" {
				want = full
			}
			if !bytes.Equal(bytes.TrimSpace(decoded), want) {
				t.Fatal("response lost current status fields or changed the full-history contract")
			}
		})
	}
	// Repeated compact and full calls share one backend snapshot.
	if after, _ := json.Marshal(body); !bytes.Equal(after, full) {
		t.Fatal("HTTP projection mutated the backend observation")
	}
}

func TestControlStatusHTTPUnavailable(t *testing.T) {
	response := httptest.NewRecorder()
	request := httptest.NewRequest(http.MethodGet, denseExecutionPath+"/status?view=control", nil)
	request.Header.Set("Accept-Encoding", "gzip")
	NewServer(ServerOptions{}).Handler().ServeHTTP(response, request)
	zr, err := gzip.NewReader(response.Body)
	if err != nil {
		t.Fatal(err)
	}
	defer zr.Close()
	var body denseexec.StatusBody
	if err := json.NewDecoder(zr).Decode(&body); err != nil || body.Available {
		t.Fatalf("unavailable response: %+v, %v", body, err)
	}
}

func BenchmarkControlStatusHTTP(b *testing.B) {
	body := controlStatusFixture(b)
	server := NewServer(ServerOptions{DenseExecution: &controlStatusBackend{body: body}}).Handler()
	r := httptest.NewRequest(http.MethodGet, denseExecutionPath+"/status?view=control", nil)
	r.Header.Set("Accept-Encoding", "gzip")
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		server.ServeHTTP(httptest.NewRecorder(), r)
	}
}
