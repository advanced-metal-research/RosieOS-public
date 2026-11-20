//go:build linux

package control

import (
	"context"
	"errors"
	"net/http"
	"net/http/httptest"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Real TLS/native simulator composition: every press consumes a WSS socket.
// Releasing and pressing again must not depend on the 30-second idle keeper.
// Zero vectors exercise transport/lifecycle only, not physical motion.
func TestRemoteJogRepeatedWarmSessions(t *testing.T) {
	s, _, _, c := remoteJogService(t, true)
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	remoteJogReady(t, s, c, ctx)
	c.stopKeepAlive()
	must(t, c.warmConnections(ctx, true))
	var previous uint64
	for attempt := 0; attempt < 4; attempt++ {
		j, err := c.NewRemoteJogSession(ctx, 511, 100_000_000, nil)
		if err != nil {
			t.Fatalf("hold %d could not start: %v", attempt+1, err)
		}
		if j.generation <= previous {
			t.Fatal("new hold reused a retired generation")
		}
		previous = j.generation
		must(t, j.Update(make([]float64, 9), ipcclient.HostMonotonicNS()))
		poll(t, time.Second, func() bool {
			o, err := j.Observe(ctx)
			return err == nil && o.Open == 1 && o.HasInput == 1 && o.ObservedSourceSequence == 1
		})
		must(t, j.End(ctx))
		poll(t, time.Second, func() bool {
			o, err := c.Status(ctx)
			return err == nil && o.Jog != nil && o.Jog.Open == 0 && o.Core.Armed == 1 && o.Core.SafetyFaultMask == 0
		})
	}
	_, err := c.Stop(ctx)
	must(t, err)
	_, err = c.Release(ctx)
	must(t, err)
}

func TestWarmJogPreparationCancellationIsUnsent(t *testing.T) {
	entered := make(chan struct{}, 1)
	var unexpected atomic.Int32
	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Method != "GET" || r.URL.Path != "/v1/jog/ingress" {
			unexpected.Add(1)
		}
		entered <- struct{}{}
		<-r.Context().Done()
	}))
	defer server.Close()
	c := newClient(server.URL, server.Client().Transport.(*http.Transport).Clone())
	defer c.Close()
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() {
		conn, err := c.takeWarmJogConnection(ctx)
		if conn != nil {
			conn.Close()
		}
		done <- err
	}()
	select {
	case <-entered:
	case <-time.After(time.Second):
		cancel()
		t.Fatal("preparation never reached read-only ingress")
	}
	cancel()
	select {
	case err := <-done:
		if !errors.Is(err, ErrConnectionUnsent) || !errors.Is(err, context.Canceled) || errors.Is(err, ErrTransport) {
			t.Fatalf("preparation incorrectly implied a sent motion request: %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("cancelled preparation did not return")
	}
	if unexpected.Load() != 0 || c.Grant().Session != "" {
		t.Fatal("preparation touched motion or authority")
	}
}
