package main

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/url"
	"regexp"
	"strconv"
	"strings"
	"time"
)

// The cell camera is a read-only view of the camera service that runs beside
// this server on the cell host. Clients (the Steam Deck pendant's Jog > Camera
// slot, a browser) already reach this server through one tunnel; relaying the
// camera here means they need no second route and never learn where the
// camera lives. Nothing here reaches the robot: these are GETs of JPEG frames,
// a RealSense's raw depth frames and a status document, and they are not
// robot actions.
//
// The camera service's contract (amr-camera, 2026-10): GET <origin>/status is
// a JSON document, and GET <origin>/frame.jpg?after=<sequence> waits up to 3 s
// for a frame newer than <sequence> and answers it with X-Frame-Sequence, or
// 503 while the camera is starting or gone. The service captures only while
// someone asks for frames. A RealSense with depth configured also answers GET
// <origin>/depth.z16?after=<sequence> the same way, with width x height
// unsigned 16-bit depth units and the headers that read them
// (cell-camera/v1/cell_camera/server.py); 404 when it serves no depth.
const (
	cellCameraAPIPath = "/api/offline-programming/v1/cell-camera"
	// The wrist camera's own route and setting from before the camera list
	// (2026-10-18), kept so pendants and OLP units of that time keep
	// working: both mean the list's camera wristCameraID.
	wristCameraAPIPath   = "/api/offline-programming/v1/wrist-camera"
	WristCameraOriginEnv = "OFFLINE_PROGRAMMING_WRIST_CAMERA_ORIGIN"
	wristCameraID        = "wrist"
	// CellCameraOriginEnv names the camera service, for example
	// http://127.0.0.1:8088/camera. Unset: the cell has no camera.
	CellCameraOriginEnv = "OFFLINE_PROGRAMMING_CELL_CAMERA_ORIGIN"
	// CellExtraCamerasEnv names further cameras on the cell host, other
	// angles of the same cell: comma-separated <id>=<origin>, for example
	// side=http://127.0.0.1:8089/camera. The camera above stays the cell
	// camera every client already shows; these are listed beside it at
	// cellCamerasAPIPath. Unset: the cell camera is the only one.
	CellExtraCamerasEnv = "OFFLINE_PROGRAMMING_CELL_EXTRA_CAMERAS"
	cellCamerasAPIPath  = "/api/offline-programming/v1/cell-cameras"
	// cellCameraDefaultID is the cell camera's id in the list; an extra
	// camera cannot take it.
	cellCameraDefaultID = "default"
	// Unverified bounds: the service's long poll is 3 s, so 5 s covers it; a
	// 1920x1200 JPEG at quality 88 is about 0.4 MiB, so 8 MiB refuses a
	// runaway body without touching a real frame. A raw depth frame is 2
	// bytes a pixel: a D435i's largest depth mode, 1280x720, is 1.8 MiB.
	cellCameraTimeout     = 5 * time.Second
	cellCameraFrameLimit  = 8 << 20
	cellCameraStatusLimit = 64 << 10
)

// CellCamera relays one camera service. The zero value is not usable; a nil
// *CellCamera means the cell names no camera.
type CellCamera struct {
	origin string
	client *http.Client
}

// NewCellCamera accepts an http origin on this host, with an optional path.
// An empty origin is no camera, not an error.
func NewCellCamera(origin string) (*CellCamera, error) {
	origin = strings.TrimRight(strings.TrimSpace(origin), "/")
	if origin == "" {
		return nil, nil
	}
	u, err := url.Parse(origin)
	if err != nil || u.Scheme != "http" || u.Host == "" || u.User != nil || u.RawQuery != "" || u.Fragment != "" {
		return nil, fmt.Errorf("%s must be an http URL without credentials or a query, got %q", CellCameraOriginEnv, origin)
	}
	// The relay fetches whatever this names, so it stays on the cell host.
	if ip := net.ParseIP(u.Hostname()); u.Hostname() != "localhost" && (ip == nil || !ip.IsLoopback()) {
		return nil, fmt.Errorf("%s must name a camera service on this host (a loopback address), got %q", CellCameraOriginEnv, u.Host)
	}
	return &CellCamera{origin: origin, client: &http.Client{Timeout: cellCameraTimeout}}, nil
}

// Origin is the camera service this relay reads.
func (c *CellCamera) Origin() string { return c.origin }

// NamedCellCamera is one further camera of the cell, by the id clients ask
// for it with.
type NamedCellCamera struct {
	ID     string
	Camera *CellCamera
}

var cellCameraID = regexp.MustCompile(`^[a-z0-9][a-z0-9-]{0,31}$`)

// NewCellExtraCameras reads CellExtraCamerasEnv's value. Every origin meets
// NewCellCamera's rules; an id is a short lower-case name, used once, and
// never the cell camera's. An empty value is no extra cameras.
func NewCellExtraCameras(value string) ([]NamedCellCamera, error) {
	var out []NamedCellCamera
	for _, entry := range strings.Split(value, ",") {
		if entry = strings.TrimSpace(entry); entry == "" {
			continue
		}
		id, origin, ok := strings.Cut(entry, "=")
		if !ok || !cellCameraID.MatchString(id) || id == cellCameraDefaultID {
			return nil, fmt.Errorf("%s entries are <id>=<origin> with a lower-case id other than %q, got %q", CellExtraCamerasEnv, cellCameraDefaultID, entry)
		}
		for _, seen := range out {
			if seen.ID == id {
				return nil, fmt.Errorf("%s names camera %q twice", CellExtraCamerasEnv, id)
			}
		}
		camera, err := NewCellCamera(origin)
		if err != nil || camera == nil {
			return nil, fmt.Errorf("%s camera %q: origin %q is not a camera service on this host (%v)", CellExtraCamerasEnv, id, origin, err)
		}
		out = append(out, NamedCellCamera{ID: id, Camera: camera})
	}
	return out, nil
}

// cameraByID is the cell camera for cellCameraDefaultID, else the extra
// camera of that id; nil when the cell has none by that id.
func (server *Server) cameraByID(id string) *CellCamera {
	if id == cellCameraDefaultID {
		return server.cellCamera
	}
	for _, extra := range server.cellExtraCameras {
		if extra.ID == id {
			return extra.Camera
		}
	}
	return nil
}

type cellCameraEntry struct {
	ID      string `json:"id"`
	Default bool   `json:"default"`
}

// cellCameraList names the cell's cameras, the cell camera first. It asks no
// camera anything: a client reads one camera's status and frames at
// cellCamerasAPIPath/<id> and cellCamerasAPIPath/<id>/frame.jpg, and a
// RealSense's raw depth at cellCamerasAPIPath/<id>/depth.z16.
func (server *Server) cellCameraList(w http.ResponseWriter, _ *http.Request) {
	cameras := []cellCameraEntry{}
	if server.cellCamera != nil {
		cameras = append(cameras, cellCameraEntry{ID: cellCameraDefaultID, Default: true})
	}
	for _, extra := range server.cellExtraCameras {
		cameras = append(cameras, cellCameraEntry{ID: extra.ID})
	}
	writeJSON(w, http.StatusOK, map[string]any{"cameras": cameras})
}

func (server *Server) cellCameraStatusByID(w http.ResponseWriter, r *http.Request) {
	camera := server.cameraByID(r.PathValue("id"))
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_unknown"})
		return
	}
	camera.serveStatus(w, r)
}

func (server *Server) cellCameraFrameByID(w http.ResponseWriter, r *http.Request) {
	camera := server.cameraByID(r.PathValue("id"))
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_unknown"})
		return
	}
	camera.serveFrame(w, r)
}

func (server *Server) cellCameraDepthByID(w http.ResponseWriter, r *http.Request) {
	camera := server.cameraByID(r.PathValue("id"))
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_unknown"})
		return
	}
	camera.serveDepth(w, r)
}

type cellCameraView struct {
	Configured bool            `json:"configured"`
	Status     json.RawMessage `json:"status,omitempty"`
	Error      string          `json:"error,omitempty"`
}

// cellCameraStatus says whether the cell names a camera and, when it does,
// what the camera service reports about itself.
func (server *Server) cellCameraStatus(w http.ResponseWriter, r *http.Request) {
	camera := server.cellCamera
	if camera == nil {
		writeJSON(w, http.StatusOK, cellCameraView{})
		return
	}
	camera.serveStatus(w, r)
}

// serveStatus answers what this camera's service reports about itself.
func (camera *CellCamera) serveStatus(w http.ResponseWriter, r *http.Request) {
	view := cellCameraView{Configured: true}
	raw, _, err := camera.get(r, "/status", cellCameraStatusLimit)
	if err == nil && !json.Valid(raw) {
		err = fmt.Errorf("camera status is not JSON")
	}
	if err != nil {
		view.Error = err.Error()
	} else {
		view.Status = raw
	}
	writeJSON(w, http.StatusOK, view)
}

// cellCameraFrame answers the newest frame after ?after=<sequence>.
func (server *Server) cellCameraFrame(w http.ResponseWriter, r *http.Request) {
	camera := server.cellCamera
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_not_configured"})
		return
	}
	camera.serveFrame(w, r)
}

// serveFrame answers this camera's newest frame after ?after=<sequence>.
func (camera *CellCamera) serveFrame(w http.ResponseWriter, r *http.Request) {
	after := r.URL.Query().Get("after")
	if after == "" {
		after = "0"
	}
	if _, err := strconv.ParseUint(after, 10, 63); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid_frame_cursor"})
		return
	}
	asked := time.Now()
	frame, header, err := camera.get(r, "/frame.jpg?after="+after, cellCameraFrameLimit)
	upstream := time.Since(asked)
	if err == nil && !strings.HasPrefix(header.Get("Content-Type"), "image/jpeg") {
		err = fmt.Errorf("camera answered %q, not a JPEG frame", header.Get("Content-Type"))
	}
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]string{"error": "cell_camera_unavailable", "detail": err.Error()})
		return
	}
	w.Header().Set("Content-Type", "image/jpeg")
	w.Header().Set("Content-Length", strconv.Itoa(len(frame)))
	w.Header().Set("Cache-Control", "no-store")
	if sequence := header.Get("X-Frame-Sequence"); sequence != "" {
		w.Header().Set("X-Frame-Sequence", sequence)
	}
	// Where the frame's time went, for a latency profile: the camera
	// service's own stages, then this relay's round trip to it (`relay`
	// includes the service's `wait`). The age is the service's, at its send.
	if age, err := strconv.ParseFloat(header.Get("X-Source-Age-Ms"), 64); err == nil && age >= 0 {
		w.Header().Set("X-Source-Age-Ms", strconv.FormatFloat(age, 'f', 2, 64))
	}
	timing := fmt.Sprintf("relay;dur=%.2f", float64(upstream.Microseconds())/1e3)
	if stages := header.Get("Server-Timing"); cellCameraTimingOK(stages) {
		timing = stages + ", " + timing
	}
	w.Header().Set("Server-Timing", timing)
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(frame)
}

// cellCameraDepth answers the cell camera's newest raw depth frame after
// ?after=<sequence>.
func (server *Server) cellCameraDepth(w http.ResponseWriter, r *http.Request) {
	camera := server.cellCamera
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_not_configured"})
		return
	}
	camera.serveDepth(w, r)
}

// cellCameraDepthHeaders are the depth reply's headers, copied only as
// numbers: what a client needs to read the frame and how old it is.
var cellCameraDepthHeaders = []string{"X-Frame-Sequence", "X-Depth-Width", "X-Depth-Height", "X-Depth-Scale",
	"X-Source-Age-Ms", "X-Camera-Frame-Number", "X-Camera-Timestamp-Ms"}

// serveDepth answers this camera's newest raw depth frame after
// ?after=<sequence>. A body that is not exactly width x height 16-bit units
// is refused, never passed on as depth a client would misread.
func (camera *CellCamera) serveDepth(w http.ResponseWriter, r *http.Request) {
	after := r.URL.Query().Get("after")
	if after == "" {
		after = "0"
	}
	if _, err := strconv.ParseUint(after, 10, 63); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]string{"error": "invalid_frame_cursor"})
		return
	}
	asked := time.Now()
	frame, header, err := camera.get(r, "/depth.z16?after="+after, cellCameraFrameLimit)
	upstream := time.Since(asked)
	var refused *cellCameraRefusal
	if errors.As(err, &refused) && refused.status == http.StatusNotFound {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_no_depth", "detail": err.Error()})
		return
	}
	if err == nil {
		err = cellCameraDepthValid(header, len(frame))
	}
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]string{"error": "cell_camera_unavailable", "detail": err.Error()})
		return
	}
	w.Header().Set("Content-Type", "application/octet-stream")
	w.Header().Set("Content-Length", strconv.Itoa(len(frame)))
	w.Header().Set("Cache-Control", "no-store")
	for _, key := range cellCameraDepthHeaders {
		if value := header.Get(key); value != "" {
			w.Header().Set(key, value)
		}
	}
	w.Header().Set("Server-Timing", fmt.Sprintf("relay;dur=%.2f", float64(upstream.Microseconds())/1e3))
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(frame)
}

// cellCameraDepthValid checks a depth reply before it is relayed: raw bytes,
// numeric headers, and a body of width x height 16-bit units.
func cellCameraDepthValid(header http.Header, size int) error {
	if kind := header.Get("Content-Type"); kind != "application/octet-stream" {
		return fmt.Errorf("camera answered %q, not a raw depth frame", kind)
	}
	for _, key := range cellCameraDepthHeaders {
		if value := header.Get(key); value != "" {
			if number, err := strconv.ParseFloat(value, 64); err != nil || number < 0 || len(value) > 32 {
				return fmt.Errorf("camera depth header %s is not a number: %q", key, value)
			}
		}
	}
	width, errW := strconv.Atoi(header.Get("X-Depth-Width"))
	height, errH := strconv.Atoi(header.Get("X-Depth-Height"))
	scale, errS := strconv.ParseFloat(header.Get("X-Depth-Scale"), 64)
	if errW != nil || errH != nil || errS != nil || width <= 0 || height <= 0 || scale <= 0 {
		return fmt.Errorf("camera depth frame does not say its size and scale")
	}
	if size != width*height*2 {
		return fmt.Errorf("camera depth frame is %d bytes, not %dx%d 16-bit units", size, width, height)
	}
	return nil
}

// cellCameraRefusal is the camera service answering something other than
// 200, kept so a caller can tell "no such route" from "not ready".
type cellCameraRefusal struct {
	status int
	text   string
}

func (e *cellCameraRefusal) Error() string { return e.text }

// get reads one camera-service document; a client that leaves cancels it.
func (c *CellCamera) get(r *http.Request, path string, limit int64) ([]byte, http.Header, error) {
	request, err := http.NewRequestWithContext(r.Context(), http.MethodGet, c.origin+path, nil)
	if err != nil {
		return nil, nil, err
	}
	response, err := c.client.Do(request)
	if err != nil {
		return nil, nil, fmt.Errorf("camera service unreachable: %w", err)
	}
	defer response.Body.Close()
	body, err := io.ReadAll(io.LimitReader(response.Body, limit+1))
	if err != nil {
		return nil, nil, fmt.Errorf("camera service read: %w", err)
	}
	if int64(len(body)) > limit {
		return nil, nil, fmt.Errorf("camera service answered more than %d bytes", limit)
	}
	if response.StatusCode != http.StatusOK {
		return nil, nil, &cellCameraRefusal{response.StatusCode,
			fmt.Sprintf("camera service: %s: %s", response.Status, strings.TrimSpace(string(body[:min(len(body), 200)])))}
	}
	return body, response.Header, nil
}

// cellCameraTimingOK admits only a plain Server-Timing value from the camera
// service: this relay copies it into its own reply.
func cellCameraTimingOK(value string) bool {
	if value == "" || len(value) > 512 {
		return false
	}
	for _, c := range value {
		if !(c >= 'a' && c <= 'z' || c >= '0' && c <= '9' || strings.ContainsRune(";=., _-", c)) {
			return false
		}
	}
	return true
}

// wristCameraStatus and wristCameraFrame answer the old wrist-camera route
// from the list's "wrist" camera.
func (server *Server) wristCameraStatus(w http.ResponseWriter, r *http.Request) {
	camera := server.cameraByID(wristCameraID)
	if camera == nil {
		writeJSON(w, http.StatusOK, cellCameraView{})
		return
	}
	camera.serveStatus(w, r)
}

func (server *Server) wristCameraFrame(w http.ResponseWriter, r *http.Request) {
	camera := server.cameraByID(wristCameraID)
	if camera == nil {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "cell_camera_not_configured"})
		return
	}
	camera.serveFrame(w, r)
}
