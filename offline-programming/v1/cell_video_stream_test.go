package main

import (
	"bytes"
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"sync/atomic"
	"testing"
	"time"
)

func h264Unit(key bool, n byte) []byte {
	if key {
		return append([]byte{0, 0, 0, 1, 0x67, 0x42, 0, 0, 0, 1, 0x68, 0xce, 0, 0, 0, 1, 0x65}, bytes.Repeat([]byte{n}, 900)...)
	}
	return append([]byte{0, 0, 0, 1, 0x41}, bytes.Repeat([]byte{n}, 300)...)
}

func framedUnit(unit []byte) []byte {
	var head [4]byte
	binary.BigEndian.PutUint32(head[:], uint32(len(unit)))
	return append(head[:], unit...)
}

// readFramed reads the relay's stream back into units.
func readFramed(t *testing.T, body []byte) [][]byte {
	t.Helper()
	var units [][]byte
	for len(body) > 0 {
		if len(body) < 4 {
			t.Fatalf("%d stray bytes", len(body))
		}
		n := binary.BigEndian.Uint32(body[:4])
		units = append(units, body[4:4+n])
		body = body[4+n:]
	}
	return units
}

// The relay passes the camera service's stream through unchanged, with no
// timeout of its own, and asks for key frames on the client's behalf.
func TestCellVideoStreamRelaysTheCameraStream(t *testing.T) {
	var sent [][]byte
	for i := 0; i < 40; i++ {
		sent = append(sent, h264Unit(i%15 == 0, byte(i)))
	}
	var keyframes atomic.Int32
	service := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		switch {
		case r.Method == http.MethodPost && r.URL.Path == "/camera/video/keyframe":
			keyframes.Add(1)
			w.WriteHeader(http.StatusNoContent)
		case r.URL.Path == "/camera/video.h264":
			for _, u := range sent {
				w.Write(framedUnit(u))
				w.(http.Flusher).Flush()
				time.Sleep(5 * time.Millisecond) // paced, as a camera is (15 a second)
			}
		default:
			http.NotFound(w, r)
		}
	}))
	defer service.Close()
	camera, err := NewCellCamera(service.URL + "/camera")
	if err != nil {
		t.Fatal(err)
	}
	olp := httptest.NewServer(NewServer(ServerOptions{DistDir: t.TempDir(), CellCamera: camera}).Handler())
	defer olp.Close()

	resp, err := http.Get(olp.URL + cellVideoStreamPath)
	if err != nil {
		t.Fatal(err)
	}
	body, _ := io.ReadAll(resp.Body)
	resp.Body.Close()
	if resp.StatusCode != http.StatusOK || resp.Header.Get("Content-Type") != "video/h264" || resp.Header.Get("Content-Encoding") != "" {
		t.Fatalf("stream: %d %q %q", resp.StatusCode, resp.Header.Get("Content-Type"), resp.Header.Get("Content-Encoding"))
	}
	got := readFramed(t, body)
	if len(got) != len(sent) {
		t.Fatalf("relayed %d units, camera sent %d", len(got), len(sent))
	}
	for i := range sent {
		if !bytes.Equal(got[i], sent[i]) {
			t.Fatalf("unit %d changed in the relay", i)
		}
	}

	resp, err = http.Post(olp.URL+cellVideoKeyframePath, "text/plain", nil)
	if err != nil {
		t.Fatal(err)
	}
	resp.Body.Close()
	if resp.StatusCode != http.StatusOK || keyframes.Load() != 1 {
		t.Fatalf("key frame request: %d, camera asked %d times", resp.StatusCode, keyframes.Load())
	}
}

// A slow client never builds a queue in the relay: once the relay's queue
// is full, units are dropped until the next key frame, and the camera is
// asked for one so the wait is short.
func TestCellVideoStreamSkipsToAKeyFrameForASlowClient(t *testing.T) {
	var keyframes atomic.Int32
	service := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		keyframes.Add(1)
		w.WriteHeader(http.StatusNoContent)
	}))
	defer service.Close()
	camera, _ := NewCellCamera(service.URL + "/camera")

	pr, pw := io.Pipe()
	units := make(chan []byte, 2)
	done := make(chan struct{})
	go func() {
		camera.readUnits(context.Background(), pr, units)
		close(units) // as serveVideo does
		close(done)
	}()
	write := func(u []byte) {
		if _, err := pw.Write(framedUnit(u)); err != nil {
			t.Fatal(err)
		}
	}
	k1, p1, p2, p3, k2, p4 := h264Unit(true, 1), h264Unit(false, 2), h264Unit(false, 3), h264Unit(false, 4), h264Unit(true, 5), h264Unit(false, 6)
	write(k1)
	write(p1)
	write(p2) // the queue is full: dropped, and from here until a key frame
	write(p2) // (written twice so the reader has surely handled the first)
	if a, b := <-units, <-units; !bytes.Equal(a, k1) || !bytes.Equal(b, p1) {
		t.Fatal("the queued units must come first, in order")
	}
	write(p3) // room again, but not a key frame: still dropped
	write(k2)
	write(p4)
	pw.Close()
	<-done
	var rest [][]byte
	for u := range units {
		rest = append(rest, u)
	}
	if len(rest) != 2 || !bytes.Equal(rest[0], k2) || !bytes.Equal(rest[1], p4) {
		t.Fatalf("after the drop the stream must resume at the key frame: %d units", len(rest))
	}
	deadline := time.Now().Add(2 * time.Second)
	for keyframes.Load() == 0 && time.Now().Before(deadline) {
		time.Sleep(10 * time.Millisecond)
	}
	if keyframes.Load() != 1 {
		t.Fatalf("the camera was asked for %d key frames, want 1", keyframes.Load())
	}
}

// A client that stops reading but keeps its connection open is dropped after
// cellVideoWriteStall, and so is the relay's stream from the camera: the
// camera's encoder and request slot are freed. Through the whole handler
// chain, so every wrapper must pass the write deadline on (Unwrap).
func TestCellVideoStreamReleasesTheCameraWhenTheClientStalls(t *testing.T) {
	defer func(stall time.Duration) { cellVideoWriteStall = stall }(cellVideoWriteStall)
	cellVideoWriteStall = 200 * time.Millisecond
	released := make(chan struct{})
	service := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/camera/video.h264" {
			w.WriteHeader(http.StatusNoContent)
			return
		}
		defer close(released)
		unit := framedUnit(append([]byte{0, 0, 0, 1, 0x41}, bytes.Repeat([]byte{7}, 64<<10)...))
		for {
			if _, err := w.Write(unit); err != nil {
				return
			}
			w.(http.Flusher).Flush()
			select {
			case <-r.Context().Done():
				return
			case <-time.After(time.Millisecond):
			}
		}
	}))
	defer service.Close()
	camera, _ := NewCellCamera(service.URL + "/camera")
	olp := httptest.NewServer(NewServer(ServerOptions{DistDir: t.TempDir(), CellCamera: camera}).Handler())
	defer olp.Close()

	// Sends the request and never reads: the connection stays open.
	conn, err := net.Dial("tcp", olp.Listener.Addr().String())
	if err != nil {
		t.Fatal(err)
	}
	defer conn.Close()
	fmt.Fprintf(conn, "GET %s HTTP/1.1\r\nHost: olp\r\n\r\n", cellVideoStreamPath)
	select {
	case <-released:
	case <-time.After(10 * time.Second):
		t.Fatal("the camera stream is still open 10 s after the client stalled")
	}
}

// A cell camera without video, or no cell camera, is 404: the pendant shows
// JPEG frames. Neither route is a robot action.
func TestCellVideoStreamAbsent(t *testing.T) {
	service := httptest.NewServer(http.NotFoundHandler())
	defer service.Close()
	camera, _ := NewCellCamera(service.URL + "/camera")
	for _, server := range []*Server{{}, {cellCamera: camera}} {
		rec := httptest.NewRecorder()
		server.cellVideoStream(rec, httptest.NewRequest(http.MethodGet, cellVideoStreamPath, nil))
		if rec.Code != http.StatusNotFound {
			t.Fatalf("no video: %d %s", rec.Code, rec.Body.String())
		}
	}
	for _, path := range []string{cellVideoStreamPath, cellVideoKeyframePath} {
		if _, robot := maintenanceRobotAction(path); robot {
			t.Errorf("%s counts as a robot action", path)
		}
	}
}
