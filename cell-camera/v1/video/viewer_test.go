package main

import (
	"bytes"
	"context"
	"errors"
	"net"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"

	deck "rosieos/cell-video/viewer"
)

// The Deck's client (viewer, run by cell-video-view) against this sender and
// a fake camera: units arrive whole, a key frame first, and ending the
// session removes the viewer and stops the camera read.
func TestTheDeckViewerReceivesWholeUnitsFromTheSender(t *testing.T) {
	camera := &fakeCamera{}
	cameraServer := httptest.NewServer(camera)
	defer cameraServer.Close()
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()
	sender, err := New(Config{Camera: cameraServer.URL + "/camera", Address: net.IPv4(127, 0, 0, 1), Port: conn.LocalAddr().(*net.UDPAddr).Port}, conn)
	if err != nil {
		t.Fatal(err)
	}
	defer sender.Close()
	whep := httptest.NewServer(sender.Handler())
	defer whep.Close()

	var mu sync.Mutex
	var units [][]byte
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() {
		_, err := deck.Run(ctx, deck.Options{WHEP: whep.URL + "/whep", Header: http.Header{"X-Rosie-Client": {"rosie-pendant-v5 (steamdeck-pendant)"}}, Loopback: true},
			nil, func(u []byte) error {
				mu.Lock()
				units = append(units, append([]byte(nil), u...))
				mu.Unlock()
				return nil
			})
		done <- err
	}()
	deadline := time.Now().Add(10 * time.Second)
	for time.Now().Before(deadline) {
		mu.Lock()
		n := len(units)
		mu.Unlock()
		if n >= 30 {
			break
		}
		time.Sleep(50 * time.Millisecond)
	}
	mu.Lock()
	got := units
	mu.Unlock()
	if len(got) < 30 {
		t.Fatalf("received %d units; camera served %d", len(got), camera.served.Load())
	}
	// The fake camera's units are a key frame every 30th (unit(true, n)), the
	// rest slices; each arrives exactly as the camera served it.
	if !bytes.HasPrefix(got[0], []byte{0, 0, 0, 1, 0x67}) {
		t.Fatalf("the first unit must be a key frame: % x", got[0][:8])
	}
	for i, u := range got {
		n := int(u[len(u)-1])
		if want := unit(n%30 == 0, n); !bytes.Equal(u, want) {
			t.Fatalf("unit %d (camera unit %d) differs from what the camera served: %d bytes, want %d", i, n, len(u), len(want))
		}
	}
	if st := sender.Status(); len(st.Viewers) != 1 || st.Viewers[0].Client != "rosie-pendant-v5 (steamdeck-pendant)" || !st.Viewers[0].Connected {
		t.Fatalf("sender status %+v", st)
	}

	cancel()
	if err := <-done; err != nil {
		t.Fatalf("run ended with %v", err)
	}
	deadline = time.Now().Add(5 * time.Second)
	for (camera.open.Load() > 0 || len(sender.Status().Viewers) > 0) && time.Now().Before(deadline) {
		time.Sleep(50 * time.Millisecond)
	}
	if st := sender.Status(); len(st.Viewers) != 0 || st.Ended != 1 || camera.open.Load() != 0 {
		t.Fatalf("after the viewer ended: %+v, camera streams open %d", st, camera.open.Load())
	}
}

// A cell without a sender (the OLP answers 404) is ErrNoVideo: the pendant
// shows JPEG frames.
func TestTheDeckViewerReportsACellWithoutVideo(t *testing.T) {
	olp := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		http.Error(w, `{"error":"cell_video_not_configured"}`, http.StatusNotFound)
	}))
	defer olp.Close()
	_, err := deck.Run(context.Background(), deck.Options{WHEP: olp.URL + "/whep", Loopback: true}, nil, func([]byte) error { return nil })
	if err == nil || !errors.Is(err, deck.ErrNoVideo) {
		t.Fatalf("got %v", err)
	}
}
