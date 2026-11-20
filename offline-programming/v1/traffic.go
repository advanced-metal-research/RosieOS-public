package main

import (
	"net/http"
	"sort"
	"strings"
	"sync"
	"time"
)

// Traffic counters: how many requests and bytes each client and each kind of
// request moved through this server since it started. On 2026-10-15 a
// conference hotspot ran out of data while a Rosie 1430 cell was teleoperated,
// and nothing could say which client or which request used it: its relay had
// sent 110 GB to Tailscale peers in under three days, attributed afterwards
// only by estimate (camera frames at about 235 KB, status polls at 40 Hz with
// 513 B of headers each). GET trafficAPIPath answers the counts; `profile`
// (steamdeck/real/v4) samples them.
//
// The meter is the outermost handler, so it counts what reaches the socket:
// response bodies after gzip, plus the response header bytes. The request
// side counts the request line, headers and declared body. TLS, TCP, SSH and
// Tailscale framing are not counted. Bytes count as they are written, so a
// long stream (the Deck's video, hours long) shows while it runs, not when it
// ends. Cost: one map lookup under a mutex per request and per write.

const trafficAPIPath = "/api/offline-programming/v1/traffic"

// trafficMaxKeys bounds the table: a client or route beyond it is counted
// under "other" rather than growing the map without limit.
const trafficMaxKeys = 512

type trafficKey struct{ client, route string }

type trafficCount struct {
	Client   string `json:"client"`
	Route    string `json:"route"`
	Requests int64  `json:"requests"`
	InBytes  int64  `json:"in_bytes"`
	OutBytes int64  `json:"out_bytes"`
}

type trafficMeter struct {
	mu     sync.Mutex
	since  time.Time
	counts map[trafficKey]*trafficCount
}

func newTrafficMeter(now time.Time) *trafficMeter {
	return &trafficMeter{since: now, counts: map[trafficKey]*trafficCount{}}
}

// trafficRoute names a request's kind: its path with the API prefix removed,
// at most four segments, a segment that looks like an identifier (digits or
// over 24 characters) replaced by "*", except the file names the server serves
// (trafficFileNames). Browser pages and scripts are "ui".
func trafficRoute(r *http.Request) string {
	path := r.URL.Path
	const api = "/api/offline-programming/v1/"
	if !strings.HasPrefix(path, api) {
		return "ui"
	}
	parts := strings.Split(strings.Trim(path[len(api):], "/"), "/")
	if len(parts) > 4 {
		parts = append(parts[:4], "...")
	}
	for i, p := range parts {
		if len(p) > 24 || strings.ContainsAny(p, "0123456789") && !trafficFileNames[p] {
			parts[i] = "*"
		}
	}
	return r.Method + " " + strings.Join(parts, "/")
}

var trafficFileNames = map[string]bool{"frame.jpg": true, "video.h264": true, "depth.z16": true}

func trafficHeaderBytes(h http.Header) int64 {
	n := int64(0)
	for key, values := range h {
		for _, v := range values {
			n += int64(len(key) + len(v) + 4) // "Key: value\r\n"
		}
	}
	return n
}

func (m *trafficMeter) add(client, route string, requests, in, out int64) {
	m.mu.Lock()
	defer m.mu.Unlock()
	key := trafficKey{client, route}
	c := m.counts[key]
	if c == nil {
		if len(m.counts) >= trafficMaxKeys {
			key = trafficKey{"other", "other"}
			if c = m.counts[key]; c == nil {
				c = &trafficCount{Client: key.client, Route: key.route}
				m.counts[key] = c
			}
		} else {
			c = &trafficCount{Client: client, Route: route}
			m.counts[key] = c
		}
	}
	c.Requests += requests
	c.InBytes += in
	c.OutBytes += out
}

type trafficReport struct {
	Since    time.Time      `json:"since"`
	Now      time.Time      `json:"now"`
	InBytes  int64          `json:"in_bytes"`
	OutBytes int64          `json:"out_bytes"`
	Counts   []trafficCount `json:"counts"`
}

func (m *trafficMeter) report(now time.Time) trafficReport {
	m.mu.Lock()
	defer m.mu.Unlock()
	r := trafficReport{Since: m.since, Now: now, Counts: make([]trafficCount, 0, len(m.counts))}
	for _, c := range m.counts {
		r.Counts = append(r.Counts, *c)
		r.InBytes += c.InBytes
		r.OutBytes += c.OutBytes
	}
	sort.Slice(r.Counts, func(i, j int) bool {
		a, b := r.Counts[i], r.Counts[j]
		if a.OutBytes+a.InBytes != b.OutBytes+b.InBytes {
			return a.OutBytes+a.InBytes > b.OutBytes+b.InBytes
		}
		return a.Client+a.Route < b.Client+b.Route
	})
	return r
}

// trafficWriter adds what the handler writes to its request's count as it
// goes; Flush passes through so streamed replies keep streaming.
type trafficWriter struct {
	http.ResponseWriter
	meter         *trafficMeter
	client, route string
	status        int
}

func (w *trafficWriter) WriteHeader(status int) {
	if w.status == 0 {
		w.status = status
		w.meter.add(w.client, w.route, 0, 0, int64(len("HTTP/1.1 200 OK\r\n\r\n"))+trafficHeaderBytes(w.Header()))
	}
	w.ResponseWriter.WriteHeader(status)
}

func (w *trafficWriter) Write(b []byte) (int, error) {
	if w.status == 0 {
		w.WriteHeader(http.StatusOK)
	}
	n, err := w.ResponseWriter.Write(b)
	w.meter.add(w.client, w.route, 0, 0, int64(n))
	return n, err
}

func (w *trafficWriter) Flush() {
	if f, ok := w.ResponseWriter.(http.Flusher); ok {
		f.Flush()
	}
}

func (w *trafficWriter) Unwrap() http.ResponseWriter { return w.ResponseWriter }

// meter wraps the whole handler chain.
func (m *trafficMeter) meter(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		in := int64(len(r.Method)+len(r.URL.RequestURI())+len(" HTTP/1.1\r\n\r\n")) + trafficHeaderBytes(r.Header)
		if r.ContentLength > 0 {
			in += r.ContentLength
		}
		_, label, _ := classifyClient(r)
		counted := &trafficWriter{ResponseWriter: w, meter: m, client: label, route: trafficRoute(r)}
		m.add(counted.client, counted.route, 1, in, 0)
		next.ServeHTTP(counted, r)
	})
}

func (server *Server) trafficCounts(writer http.ResponseWriter, _ *http.Request) {
	writeJSON(writer, http.StatusOK, server.traffic.report(time.Now()))
}
