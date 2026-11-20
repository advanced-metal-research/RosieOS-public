package main

import (
	"bufio"
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"net/http"
	"sync"
	"time"
)

// The cell camera's low-data video for a client that cannot take WebRTC: the
// Steam Deck pendant, whose Tailscale runs in userspace-networking mode and so
// has no route for the sender's UDP (a Steam Deck, 2026-10-19). The camera
// service's H.264 stream (cell-camera/v1 video.py, about 800 kbit/s) is relayed
// through this server, so it reaches the Deck through the tunnel that already
// carries the JPEG frames: about 0.36 GB an hour instead of 12.7 GB.
//
//	GET  cellVideoStreamPath    the stream: per access unit a 4-byte big-endian
//	                            length and the Annex-B unit, a key frame first;
//	                            404 when the cell camera has no video
//	POST cellVideoKeyframePath  asks the encoder for a key frame (the client's
//	                            decoder lost its place)
//
// The stream shares the tunnel with jog replies, so it never queues here: when
// the client reads slower than the camera sends, units are dropped until the
// next key frame. This bounds only this server's queue. The SSH tunnel buffers
// what sshd has read before this server sees the client slow down (OpenSSH's
// default channel window, CHAN_TCP_WINDOW_DEFAULT in channels.h, is 2 MB,
// about 20 s of video; unmeasured on the cells), and jog replies share that
// TCP connection: cell-camera/v1/README.md "Next steps".
const (
	cellVideoStreamPath   = cellCameraAPIPath + "/video.h264"
	cellVideoKeyframePath = cellVideoAPIPath + "/keyframe"
	// Unverified bounds. About half a second of units at 15 fps waits for a
	// slow client; beyond that it skips to a key frame. A client that takes
	// no bytes for cellVideoWriteStall is gone. The first unit comes within
	// the camera's own first-unit wait.
	cellVideoQueuedUnits = 8
	cellVideoMaxUnit     = 4 << 20
	cellVideoKeyInterval = time.Second
)

// A variable only so a test need not wait 10 s for a stalled client.
var cellVideoWriteStall = 10 * time.Second

func (server *Server) cellVideoStream(w http.ResponseWriter, r *http.Request) {
	camera := server.cellCamera
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_not_configured"})
		return
	}
	camera.serveVideo(w, r)
}

func (server *Server) cellVideoKeyframe(w http.ResponseWriter, r *http.Request) {
	camera := server.cellCamera
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_not_configured"})
		return
	}
	if err := camera.askKeyframe(r.Context()); err != nil {
		writeJSON(w, http.StatusBadGateway, map[string]string{"error": "cell_camera_unavailable", "detail": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]bool{"asked": true})
}

// askKeyframe relays a key frame request to the camera service.
func (camera *CellCamera) askKeyframe(ctx context.Context) error {
	request, err := http.NewRequestWithContext(ctx, http.MethodPost, camera.origin+"/video/keyframe", nil)
	if err != nil {
		return err
	}
	response, err := camera.client.Do(request)
	if err != nil {
		return err
	}
	io.Copy(io.Discard, io.LimitReader(response.Body, cellCameraStatusLimit))
	response.Body.Close()
	if response.StatusCode != http.StatusOK && response.StatusCode != http.StatusNoContent {
		return fmt.Errorf("camera service answered %s", response.Status)
	}
	return nil
}

// serveVideo relays the camera service's stream until the client or the
// camera ends it.
func (camera *CellCamera) serveVideo(w http.ResponseWriter, r *http.Request) {
	ctx, cancel := context.WithCancel(r.Context())
	defer cancel()
	request, err := http.NewRequestWithContext(ctx, http.MethodGet, camera.origin+"/video.h264", nil)
	if err != nil {
		writeJSON(w, http.StatusInternalServerError, map[string]string{"error": err.Error()})
		return
	}
	// No client timeout: the stream lasts as long as someone watches.
	response, err := (&http.Client{Transport: camera.client.Transport}).Do(request)
	if err != nil {
		writeJSON(w, http.StatusBadGateway, map[string]string{"error": "cell_camera_unavailable", "detail": err.Error()})
		return
	}
	defer response.Body.Close()
	switch {
	case response.StatusCode == http.StatusNotFound:
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_video_not_configured"})
		return
	case response.StatusCode != http.StatusOK:
		writeJSON(w, http.StatusServiceUnavailable, map[string]string{"error": "cell_camera_unavailable", "detail": "camera service: " + response.Status})
		return
	}

	units := make(chan []byte, cellVideoQueuedUnits)
	var reader sync.WaitGroup
	reader.Add(1)
	go func() {
		defer reader.Done()
		defer close(units)
		camera.readUnits(ctx, response.Body, units)
	}()
	// A client that stops reading fails a write here. net/http then cancels
	// r.Context(), but stop the reader and close the camera stream here
	// rather than rely on that: a reader left running holds the camera's
	// encoder and one of its request slots.
	defer func() {
		cancel()
		response.Body.Close()
		reader.Wait()
	}()

	// The server's 5-minute write timeout is for requests; this one lasts
	// while the client keeps taking bytes. A wrapper in the handler chain
	// without Unwrap makes this fail: refuse now rather than cut every
	// stream at minute 5.
	control := http.NewResponseController(w)
	if err := control.SetWriteDeadline(time.Now().Add(cellVideoWriteStall)); err != nil {
		writeJSON(w, http.StatusInternalServerError, map[string]string{"error": "cell_video_no_write_deadline", "detail": err.Error()})
		return
	}
	w.Header().Set("Content-Type", "video/h264")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(http.StatusOK)
	var head [4]byte
	for unit := range units {
		_ = control.SetWriteDeadline(time.Now().Add(cellVideoWriteStall))
		binary.BigEndian.PutUint32(head[:], uint32(len(unit)))
		if _, err := w.Write(head[:]); err != nil {
			return
		}
		if _, err := w.Write(unit); err != nil {
			return
		}
		if err := control.Flush(); err != nil {
			return
		}
	}
}

// readUnits reads framed units from the camera into units. When the queue is
// full (the client is slow) it drops units until the next key frame and asks
// the camera for one, at most once per cellVideoKeyInterval.
func (camera *CellCamera) readUnits(ctx context.Context, body io.Reader, units chan<- []byte) {
	r := bufio.NewReaderSize(body, 256<<10)
	var head [4]byte
	skipping := false
	var lastKey time.Time
	for {
		if _, err := io.ReadFull(r, head[:]); err != nil {
			return
		}
		n := binary.BigEndian.Uint32(head[:])
		if n == 0 || n > cellVideoMaxUnit {
			return
		}
		unit := make([]byte, n)
		if _, err := io.ReadFull(r, unit); err != nil {
			return
		}
		if skipping && !annexBKeyUnit(unit) {
			continue
		}
		select {
		case units <- unit:
			skipping = false
		case <-ctx.Done():
			return
		default:
			skipping = true
			if time.Since(lastKey) >= cellVideoKeyInterval {
				lastKey = time.Now()
				go camera.askKeyframe(ctx)
			}
		}
	}
}

// annexBKeyUnit reports whether an Annex-B access unit holds an IDR slice.
func annexBKeyUnit(unit []byte) bool {
	for i := 0; i+3 < len(unit); i++ {
		if unit[i] == 0 && unit[i+1] == 0 && unit[i+2] == 1 && unit[i+3]&0x1f == 5 {
			return true
		}
	}
	return false
}
