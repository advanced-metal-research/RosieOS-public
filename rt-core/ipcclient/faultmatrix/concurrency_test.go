//go:build linux

package faultmatrix

import (
	"context"
	"encoding/json"
	"errors"
	"net"
	"net/http"
	"path/filepath"
	ctl "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/internal/controltest"
	"rosieos/rt-core/internal/unixhttp"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

func adapterFixture(t *testing.T) (*fixture, *ctl.Controller, ctl.Fence) {
	f := compileFixture(t, "simulation.json", nil)
	f.launch(false)
	binding := ctl.Binding{PairID: "fault-matrix", Revision: 1, ConfigurationSHA256: f.compiled.SHA256}
	a, err := ctl.New(f.c, binding)
	must(t, err)
	t.Cleanup(func() { _ = a.Close() })
	grant, err := a.Acquire("matrix", binding)
	must(t, err)
	fence := ctl.Fence{Session: grant.Session, Generation: grant.Generation}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				_, _ = a.Renew(fence)
			}
		}
	}()
	t.Cleanup(func() { cancel(); <-done })
	return f, a, fence
}
func armAdapter(t *testing.T, f *fixture, a *ctl.Controller, fence ctl.Fence) {
	t.Helper()
	_, err := a.Enable(fence, 3)
	must(t, err)
	_, err = a.Arm(fence)
	must(t, err)
	f.until(ipc.HostMonotonicNS(), publicationNS, enabled)
}

func concurrentOne(t *testing.T, r row) {
	f := start(t)
	ready := f.arm()
	h := f.prepare(801, .03, 3*time.Second)
	lateHandle := f.prepare(802, .01, time.Second)
	gate := make(chan struct{})
	started := make(chan error, 1)
	stopped := make(chan error, 1)
	go func() { <-gate; _, err := f.c.StartTrajectory(context.Background(), h); started <- err }()
	go func() { <-gate; _, err := f.c.Stop(context.Background()); stopped <- err }()
	at := ipc.HostMonotonicNS()
	close(gate)
	must(t, <-stopped)
	startErr := <-started
	if startErr != nil {
		var rejected *ipc.RejectedError
		if !errors.Is(startErr, ipc.ErrRevoked) && !errors.As(startErr, &rejected) {
			t.Fatalf("unexpected race result: %v", startErr)
		}
	}
	s := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.Armed == 0 && s.ControlGeneration > ready.ControlGeneration })
	inhibited(t, s, 0)
	// This handle was never submitted by the racing Start. Its rejection must
	// therefore come from Stop's generation fence, not prior consumption.
	_, err := f.c.StartTrajectory(context.Background(), lateHandle)
	reject(t, err, ipc.ReasonUnknownHandle)
	f.hold(s, 400_000_000, 0)
}

func concurrentTwo(t *testing.T, r row) {
	f, a, fence := adapterFixture(t)
	armAdapter(t, f, a, fence)
	before := f.status()
	h, err := a.Prepare(fence, 3, []ipc.Point{{Position: []float64{0, 0}}, {Time: 3 * time.Second, Position: []float64{.03, .03}}})
	must(t, err)
	lateHandle, err := a.Prepare(fence, 3, []ipc.Point{{Position: []float64{0, 0}}, {Time: time.Second, Position: []float64{.01, .01}}})
	must(t, err)
	listener, err := net.Listen("unix", filepath.Join(f.socketDir, "http"))
	must(t, err)
	server := &http.Server{Handler: ctl.NewHandler(a)}
	go func() { _ = server.Serve(listener) }()
	defer server.Close()
	request := func(operation string, handle uint64) ctl.Response {
		transport := unixhttp.NewClient(listener.Addr().String()).Transport.(*http.Transport)
		transport.DisableKeepAlives = true
		defer transport.CloseIdleConnections()
		client := http.Client{Transport: transport, Timeout: watchdog}
		body, _ := json.Marshal(ctl.Request{Schema: "rosie.rt-control.request.v1", Operation: operation, Fence: fence, Handle: handle})
		var out ctl.Response
		if _, err := (controltest.Client{HTTP: &client, Base: "http://local"}).Decode(context.Background(), http.MethodPost, "/v1/control", body, http.Header{"Content-Type": {"application/json"}}, &out); err != nil {
			return ctl.Response{Error: err.Error()}
		}
		return out
	}
	gate := make(chan struct{})
	started := make(chan ctl.Response, 1)
	stopped := make(chan ctl.Response, 1)
	go func() { <-gate; started <- request("start_trajectory", h) }()
	go func() { <-gate; stopped <- request("stop", 0) }()
	at := ipc.HostMonotonicNS()
	close(gate)
	stop := <-stopped
	if stop.Error != "" {
		t.Fatalf("Stop HTTP: %+v", stop)
	}
	first := <-started
	if first.Error != "" && first.Error != "control_session_stale" && first.Error != "fence" && first.Error != "handle_retired" {
		t.Fatalf("unexpected raced Start rejection: %+v", first)
	}
	t.Logf("raced Start: %+v", first)
	s := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.Armed == 0 && s.ControlGeneration > before.ControlGeneration })
	inhibited(t, s, 0)
	late := request("start_trajectory", lateHandle)
	if late.Error != "control_session_stale" {
		t.Fatalf("late Start not generation fenced: %+v", late)
	}
	f.hold(s, 400_000_000, 0)
}
