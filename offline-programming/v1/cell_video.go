package main

import (
	"fmt"
	"io"
	"net"
	"net/http"
	"net/url"
	"regexp"
	"strings"
	"time"
)

// The cell camera's low-data video: WHEP signalling relayed to the camera's
// WebRTC sender (cell-camera/v1/video, `cell-video`) on this host. Only the
// set-up exchange passes here, a few kilobytes per viewer; the video itself
// goes from the sender to the viewer as UDP over Tailscale, outside the tunnel
// that carries this server's jog replies (one browser viewer of the JPEG
// stream used about 25 GB an hour, a Rosie 1430 cell, 2026-10-16). A client without
// WebRTC, or a cell without the sender, keeps the JPEG frames at
// cellCameraAPIPath/frame.jpg.
//
//	GET    cellVideoAPIPath           200 with the sender's status, or 404 when the cell has no video
//	POST   cellVideoAPIPath/whep      the viewer's SDP offer; 201 with the answer and a Location here
//	DELETE cellVideoAPIPath/whep/{id} ends that viewer's session
const (
	cellVideoAPIPath = "/api/offline-programming/v1/cell-camera/video"
	// CellVideoOriginEnv names the sender's WHEP origin, for example
	// http://127.0.0.1:8090. Unset: the cell camera has no video.
	CellVideoOriginEnv = "OFFLINE_PROGRAMMING_CELL_VIDEO_ORIGIN"
	// Unverified bound: the sender answers after gathering one host
	// candidate, bounded at 2 s, plus a key frame request.
	cellVideoTimeout    = 6 * time.Second
	cellVideoOfferLimit = 64 << 10
)

var cellVideoSession = regexp.MustCompile(`^[0-9a-f]{16}$`)

// CellVideo relays one WebRTC sender. A nil *CellVideo means no video.
type CellVideo struct {
	origin string
	client *http.Client
}

// NewCellVideo accepts an http origin on this host; empty is no video.
func NewCellVideo(origin string) (*CellVideo, error) {
	origin = strings.TrimRight(strings.TrimSpace(origin), "/")
	if origin == "" {
		return nil, nil
	}
	u, err := url.Parse(origin)
	if err != nil || u.Scheme != "http" || u.Host == "" || u.User != nil || u.RawQuery != "" || u.Fragment != "" || (u.Path != "" && u.Path != "/") {
		return nil, fmt.Errorf("%s must be an http origin without a path, credentials or a query, got %q", CellVideoOriginEnv, origin)
	}
	if ip := net.ParseIP(u.Hostname()); u.Hostname() != "localhost" && (ip == nil || !ip.IsLoopback()) {
		return nil, fmt.Errorf("%s must name the video sender on this host (a loopback address), got %q", CellVideoOriginEnv, u.Host)
	}
	return &CellVideo{origin: origin, client: &http.Client{Timeout: cellVideoTimeout}}, nil
}

func (server *Server) cellVideoStatus(w http.ResponseWriter, r *http.Request) {
	video := server.cellVideo
	if video == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_video_not_configured"})
		return
	}
	video.relay(w, r, http.MethodGet, "/status", nil, "")
}

func (server *Server) cellVideoOffer(w http.ResponseWriter, r *http.Request) {
	video := server.cellVideo
	if video == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_video_not_configured"})
		return
	}
	if !strings.HasPrefix(r.Header.Get("Content-Type"), "application/sdp") {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "cell_video_offer_invalid", "detail": "expected an application/sdp offer"})
		return
	}
	offer, err := io.ReadAll(io.LimitReader(r.Body, cellVideoOfferLimit+1))
	if err != nil || len(offer) == 0 || len(offer) > cellVideoOfferLimit {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "cell_video_offer_invalid", "detail": "an SDP offer of 1 to 64 KiB"})
		return
	}
	_, label, _ := classifyClient(r)
	video.relay(w, r, http.MethodPost, "/whep", offer, label)
}

func (server *Server) cellVideoEnd(w http.ResponseWriter, r *http.Request) {
	video := server.cellVideo
	id := r.PathValue("id")
	if video == nil || !cellVideoSession.MatchString(id) {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_video_session_unknown"})
		return
	}
	video.relay(w, r, http.MethodDelete, "/whep/"+id, nil, "")
}

// relay forwards one request to the sender and its reply to the client. The
// sender's Location (/whep/<id>) becomes this server's path for the session.
func (v *CellVideo) relay(w http.ResponseWriter, r *http.Request, method, path string, body []byte, client string) {
	var reader io.Reader
	if body != nil {
		reader = strings.NewReader(string(body))
	}
	request, err := http.NewRequestWithContext(r.Context(), method, v.origin+path, reader)
	if err != nil {
		writeJSON(w, http.StatusInternalServerError, map[string]string{"error": "cell_video_request", "detail": err.Error()})
		return
	}
	if body != nil {
		request.Header.Set("Content-Type", "application/sdp")
	}
	if client != "" {
		request.Header.Set("X-Rosie-Client", client)
	}
	response, err := v.client.Do(request)
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]string{"error": "cell_video_unavailable", "detail": err.Error()})
		return
	}
	defer response.Body.Close()
	reply, err := io.ReadAll(io.LimitReader(response.Body, cellVideoOfferLimit+1))
	if err != nil || len(reply) > cellVideoOfferLimit {
		writeJSON(w, http.StatusBadGateway, map[string]string{"error": "cell_video_reply_invalid"})
		return
	}
	if location := response.Header.Get("Location"); location != "" {
		id := strings.TrimPrefix(location, "/whep/")
		if !cellVideoSession.MatchString(id) {
			writeJSON(w, http.StatusBadGateway, map[string]string{"error": "cell_video_reply_invalid", "detail": "unexpected session location"})
			return
		}
		w.Header().Set("Location", cellVideoAPIPath+"/whep/"+id)
	}
	if kind := response.Header.Get("Content-Type"); kind != "" {
		w.Header().Set("Content-Type", kind)
	}
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(response.StatusCode)
	_, _ = w.Write(reply)
}
