package main

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
)

// A stand-in for the camera service's contract: status JSON, and a frame
// newer than ?after= with its sequence, or 503 while the camera starts.
func fakeCameraService(t *testing.T, sequence *int, frame []byte) *httptest.Server {
	t.Helper()
	return httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		switch r.URL.Path {
		case "/camera/status":
			w.Header().Set("Content-Type", "application/json")
			_, _ = w.Write([]byte(`{"camera":"test","active":true}`))
		case "/camera/frame.jpg":
			if *sequence == 0 {
				http.Error(w, "Camera unavailable; reconnecting automatically", http.StatusServiceUnavailable)
				return
			}
			if r.URL.Query().Get("after") != "41" {
				t.Errorf("after = %q, want the client's cursor", r.URL.Query().Get("after"))
			}
			w.Header().Set("Content-Type", "image/jpeg")
			w.Header().Set("X-Frame-Sequence", "42")
			w.Header().Set("X-Source-Age-Ms", "46.5")
			w.Header().Set("Server-Timing", r.Header.Get("X-Test-Server-Timing"))
			_, _ = w.Write(frame)
		default:
			http.NotFound(w, r)
		}
	}))
}

func cellCameraServer(t *testing.T, origin string) http.Handler {
	t.Helper()
	camera, err := NewCellCamera(origin)
	if err != nil {
		t.Fatal(err)
	}
	return NewServer(ServerOptions{DistDir: t.TempDir(), CellCamera: camera}).Handler()
}

func TestCellCameraRelaysTheNewestFrameAndStatus(t *testing.T) {
	sequence, frame := 42, []byte("\xff\xd8 a frame \xff\xd9")
	service := fakeCameraService(t, &sequence, frame)
	defer service.Close()
	handler := cellCameraServer(t, service.URL+"/camera/")

	response := httptest.NewRecorder()
	handler.ServeHTTP(response, httptest.NewRequest(http.MethodGet, cellCameraAPIPath+"/frame.jpg?after=41", nil))
	if response.Code != http.StatusOK || response.Header().Get("Content-Type") != "image/jpeg" ||
		response.Header().Get("X-Frame-Sequence") != "42" || response.Header().Get("Cache-Control") != "no-store" ||
		response.Body.String() != string(frame) {
		t.Fatalf("frame: %d %v %q", response.Code, response.Header(), response.Body.String())
	}
	// A latency profile reads the frame's age and the relay's own cost; a
	// camera service without stage timings still gets the relay's.
	if response.Header().Get("X-Source-Age-Ms") != "46.50" || !strings.HasPrefix(response.Header().Get("Server-Timing"), "relay;dur=") {
		t.Fatalf("timing headers: %v", response.Header())
	}

	response = httptest.NewRecorder()
	handler.ServeHTTP(response, httptest.NewRequest(http.MethodGet, cellCameraAPIPath, nil))
	var view cellCameraView
	if err := json.Unmarshal(response.Body.Bytes(), &view); err != nil || !view.Configured || view.Error != "" ||
		!strings.Contains(string(view.Status), `"active":true`) {
		t.Fatalf("status: %s (%v)", response.Body.String(), err)
	}
}

// A camera that is starting, gone, or not a camera is "unavailable", never a
// stale or foreign body passed through as a frame.
func TestCellCameraRefusals(t *testing.T) {
	sequence := 0
	service := fakeCameraService(t, &sequence, nil)
	defer service.Close()
	handler := cellCameraServer(t, service.URL+"/camera")
	get := func(handler http.Handler, path string) (int, string) {
		response := httptest.NewRecorder()
		handler.ServeHTTP(response, httptest.NewRequest(http.MethodGet, path, nil))
		return response.Code, response.Body.String()
	}
	if code, body := get(handler, cellCameraAPIPath+"/frame.jpg"); code != http.StatusServiceUnavailable || !strings.Contains(body, "cell_camera_unavailable") {
		t.Fatalf("starting camera: %d %s", code, body)
	}
	if code, body := get(handler, cellCameraAPIPath+"/frame.jpg?after=-1"); code != http.StatusBadRequest || !strings.Contains(body, "invalid_frame_cursor") {
		t.Fatalf("bad cursor: %d %s", code, body)
	}
	// Something else on that port answers 200 with a page, not a frame.
	page := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.Header().Set("Content-Type", "text/html")
		_, _ = w.Write([]byte("<html>not a camera</html>"))
	}))
	defer page.Close()
	if code, body := get(cellCameraServer(t, page.URL), cellCameraAPIPath+"/frame.jpg"); code != http.StatusServiceUnavailable || !strings.Contains(body, "not a JPEG frame") {
		t.Fatalf("wrong service: %d %s", code, body)
	}
	service.Close()
	if code, body := get(handler, cellCameraAPIPath+"/frame.jpg"); code != http.StatusServiceUnavailable || !strings.Contains(body, "unreachable") {
		t.Fatalf("stopped camera: %d %s", code, body)
	}

	none := NewServer(ServerOptions{DistDir: t.TempDir()}).Handler()
	if code, body := get(none, cellCameraAPIPath+"/frame.jpg"); code != http.StatusNotFound || !strings.Contains(body, "cell_camera_not_configured") {
		t.Fatalf("no camera: %d %s", code, body)
	}
	if code, body := get(none, cellCameraAPIPath); code != http.StatusOK || !strings.Contains(body, `"configured":false`) {
		t.Fatalf("no camera status: %d %s", code, body)
	}
}

// The relay fetches what the origin names, so it stays on the cell host.
func TestCellCameraOriginStaysOnTheHost(t *testing.T) {
	if camera, err := NewCellCamera(""); camera != nil || err != nil {
		t.Fatalf("empty origin: %v %v", camera, err)
	}
	for _, origin := range []string{"http://127.0.0.1:8088/camera", "http://localhost:8088", "http://[::1]:8088/camera/"} {
		if camera, err := NewCellCamera(origin); err != nil || camera == nil || strings.HasSuffix(camera.Origin(), "/") {
			t.Errorf("%s: %v %v", origin, camera, err)
		}
	}
	for _, origin := range []string{"https://127.0.0.1:8088", "http://192.168.1.10:8088/camera", "http://example.com/camera",
		"http://user@127.0.0.1:8088", "http://127.0.0.1:8088/camera?x=1", "127.0.0.1:8088", "file:///etc/passwd"} {
		if camera, err := NewCellCamera(origin); err == nil {
			t.Errorf("%s accepted: %v", origin, camera)
		}
	}
}

// The camera service's stage timings are copied in front of the relay's own
// only when they are a plain timing list.
func TestCellCameraTimingIsCopiedOnlyWhenPlain(t *testing.T) {
	for value, want := range map[string]bool{
		"capture;dur=4.10, encode;dur=18.00, held;dur=0.50, wait;dur=20.00": true,
		"":                               false,
		"capture;desc=\"x\"":             false,
		"capture;dur=1\r\nX-Injected: 1": false,
		strings.Repeat("a;dur=1, ", 100): false,
	} {
		if got := cellCameraTimingOK(value); got != want {
			t.Errorf("cellCameraTimingOK(%q) = %v, want %v", value, got, want)
		}
	}
}

// The wrist camera (2026-10-18) is the list's "wrist" camera; its
// old route still answers it, for pendants built before the list.
func TestWristCameraIsIndependent(t *testing.T) {
	cell := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "image/jpeg")
		w.Write([]byte("cell"))
	}))
	defer cell.Close()
	wrist := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "image/jpeg")
		w.Write([]byte("wrist"))
	}))
	defer wrist.Close()
	c, _ := NewCellCamera(cell.URL)
	v, _ := NewCellCamera(wrist.URL)
	for _, configured := range []bool{true, false} {
		options := ServerOptions{DistDir: t.TempDir(), CellCamera: c}
		if configured {
			options.CellExtraCameras = []NamedCellCamera{{ID: wristCameraID, Camera: v}}
		}
		handler := NewServer(options).Handler()
		// The old wrist route and the list's entry are the same camera.
		for _, path := range []string{cellCameraAPIPath, wristCameraAPIPath, cellCamerasAPIPath + "/" + wristCameraID} {
			response := httptest.NewRecorder()
			handler.ServeHTTP(response, httptest.NewRequest("GET", path+"/frame.jpg", nil))
			if path != cellCameraAPIPath && !configured {
				if response.Code != 404 {
					t.Fatal(response.Code)
				}
				continue
			}
			want := "cell"
			if path != cellCameraAPIPath {
				want = "wrist"
			}
			if response.Body.String() != want {
				t.Fatal(response.Body.String())
			}
		}
	}
}

// A cell with a second camera lists both, the cell camera first, and relays
// each by its id; the cell camera's own routes are unchanged, so a client
// that knows one camera still gets it.
func TestCellCamerasListAndRelayEachByID(t *testing.T) {
	sequence := 42
	main := fakeCameraService(t, &sequence, []byte("\xff\xd8 main \xff\xd9"))
	defer main.Close()
	side := fakeCameraService(t, &sequence, []byte("\xff\xd8 side \xff\xd9"))
	defer side.Close()
	camera, err := NewCellCamera(main.URL + "/camera")
	if err != nil {
		t.Fatal(err)
	}
	extras, err := NewCellExtraCameras(" side=" + side.URL + "/camera/ ,")
	if err != nil || len(extras) != 1 || extras[0].ID != "side" {
		t.Fatalf("extras: %+v %v", extras, err)
	}
	handler := NewServer(ServerOptions{DistDir: t.TempDir(), CellCamera: camera, CellExtraCameras: extras}).Handler()
	get := func(path string) (int, string) {
		response := httptest.NewRecorder()
		handler.ServeHTTP(response, httptest.NewRequest(http.MethodGet, path, nil))
		return response.Code, response.Body.String()
	}
	if code, body := get(cellCamerasAPIPath); code != http.StatusOK ||
		strings.TrimSpace(body) != `{"cameras":[{"id":"default","default":true},{"id":"side","default":false}]}` {
		t.Fatalf("list: %d %s", code, body)
	}
	for path, want := range map[string]string{
		cellCameraAPIPath + "/frame.jpg?after=41":          " main ",
		cellCamerasAPIPath + "/default/frame.jpg?after=41": " main ",
		cellCamerasAPIPath + "/side/frame.jpg?after=41":    " side ",
	} {
		if code, body := get(path); code != http.StatusOK || !strings.Contains(body, want) {
			t.Errorf("%s: %d %q", path, code, body)
		}
	}
	if code, body := get(cellCamerasAPIPath + "/side"); code != http.StatusOK || !strings.Contains(body, `"configured":true`) || !strings.Contains(body, `"camera":"test"`) {
		t.Fatalf("side status: %d %s", code, body)
	}
	for _, path := range []string{cellCamerasAPIPath + "/top", cellCamerasAPIPath + "/top/frame.jpg"} {
		if code, body := get(path); code != http.StatusNotFound || !strings.Contains(body, "cell_camera_unknown") {
			t.Errorf("%s: %d %s", path, code, body)
		}
	}
	// A cell with no camera lists none; it has no "default" to ask for.
	none := NewServer(ServerOptions{DistDir: t.TempDir()}).Handler()
	response := httptest.NewRecorder()
	none.ServeHTTP(response, httptest.NewRequest(http.MethodGet, cellCamerasAPIPath, nil))
	if strings.TrimSpace(response.Body.String()) != `{"cameras":[]}` {
		t.Fatalf("no cameras: %s", response.Body.String())
	}
	response = httptest.NewRecorder()
	none.ServeHTTP(response, httptest.NewRequest(http.MethodGet, cellCamerasAPIPath+"/default", nil))
	if response.Code != http.StatusNotFound {
		t.Fatalf("default without a camera: %d", response.Code)
	}
}

// Extra cameras meet the cell camera's rules, each under its own id.
func TestCellExtraCamerasRefusals(t *testing.T) {
	if extras, err := NewCellExtraCameras(""); err != nil || len(extras) != 0 {
		t.Fatalf("empty: %+v %v", extras, err)
	}
	for _, bad := range []string{
		"http://127.0.0.1:8089/camera",         // no id
		"Side=http://127.0.0.1:8089/camera",    // not lower-case
		"default=http://127.0.0.1:8089/camera", // the cell camera's id
		"side=http://192.168.1.10:8089/camera", // not on this host
		"side=",                                // no origin
		"side=http://127.0.0.1:8089/camera,side=http://127.0.0.1:8090/camera", // twice
	} {
		if extras, err := NewCellExtraCameras(bad); err == nil {
			t.Errorf("%q accepted: %+v", bad, extras)
		}
	}
}

// A stand-in RealSense service: raw depth with its headers, 404 when the
// camera serves no depth, 503 while it starts.
func fakeDepthService(t *testing.T, mode string) *httptest.Server {
	t.Helper()
	return httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/camera/depth.z16" {
			http.NotFound(w, r)
			return
		}
		switch mode {
		case "none":
			http.Error(w, "This camera serves no depth", http.StatusNotFound)
			return
		case "starting":
			http.Error(w, "Depth unavailable; reconnecting automatically", http.StatusServiceUnavailable)
			return
		}
		if r.URL.Query().Get("after") != "6" {
			t.Errorf("after = %q, want the client's cursor", r.URL.Query().Get("after"))
		}
		w.Header().Set("Content-Type", "application/octet-stream")
		w.Header().Set("X-Frame-Sequence", "7")
		w.Header().Set("X-Depth-Width", "4")
		w.Header().Set("X-Depth-Height", "2")
		w.Header().Set("X-Depth-Scale", "0.001")
		w.Header().Set("X-Source-Age-Ms", "31.2")
		w.Header().Set("X-Camera-Frame-Number", "812")
		w.Header().Set("X-Camera-Timestamp-Ms", "1700000000123.5")
		body := make([]byte, 4*2*2)
		if mode == "short" {
			body = body[:15]
		}
		if mode == "header" {
			w.Header().Set("X-Depth-Scale", "1e-3\r\nX-Evil: 1")
		}
		_, _ = w.Write(body)
	}))
}

// The OLP relays a RealSense's raw depth as the service sent it, with the
// numbers that read it, and refuses a body that does not match them.
func TestCellCameraRelaysRawDepth(t *testing.T) {
	get := func(handler http.Handler, path string) *httptest.ResponseRecorder {
		response := httptest.NewRecorder()
		handler.ServeHTTP(response, httptest.NewRequest(http.MethodGet, path, nil))
		return response
	}
	service := fakeDepthService(t, "ok")
	defer service.Close()
	handler := cellCameraServer(t, service.URL+"/camera")
	response := get(handler, cellCameraAPIPath+"/depth.z16?after=6")
	if response.Code != http.StatusOK || response.Header().Get("Content-Type") != "application/octet-stream" || response.Body.Len() != 16 ||
		response.Header().Get("Cache-Control") != "no-store" {
		t.Fatalf("depth: %d %v %d bytes", response.Code, response.Header(), response.Body.Len())
	}
	for key, want := range map[string]string{"X-Frame-Sequence": "7", "X-Depth-Width": "4", "X-Depth-Height": "2", "X-Depth-Scale": "0.001",
		"X-Source-Age-Ms": "31.2", "X-Camera-Frame-Number": "812", "X-Camera-Timestamp-Ms": "1700000000123.5"} {
		if got := response.Header().Get(key); got != want {
			t.Errorf("%s = %q, want %q", key, got, want)
		}
	}
	if !strings.HasPrefix(response.Header().Get("Server-Timing"), "relay;dur=") {
		t.Errorf("no relay timing: %v", response.Header())
	}
	// The same camera by id in the list.
	if response := get(handler, cellCamerasAPIPath+"/"+cellCameraDefaultID+"/depth.z16?after=6"); response.Code != http.StatusOK || response.Body.Len() != 16 {
		t.Fatalf("depth by id: %d %s", response.Code, response.Body.String())
	}
	if response := get(handler, cellCamerasAPIPath+"/side/depth.z16"); response.Code != http.StatusNotFound || !strings.Contains(response.Body.String(), "cell_camera_unknown") {
		t.Fatalf("unknown camera: %d %s", response.Code, response.Body.String())
	}
	if response := get(handler, cellCameraAPIPath+"/depth.z16?after=x"); response.Code != http.StatusBadRequest {
		t.Fatalf("bad cursor: %d %s", response.Code, response.Body.String())
	}

	for mode, want := range map[string]struct {
		code int
		text string
	}{
		"none":     {http.StatusNotFound, "cell_camera_no_depth"},
		"starting": {http.StatusServiceUnavailable, "cell_camera_unavailable"},
		"short":    {http.StatusServiceUnavailable, "not 4x2 16-bit units"},
		"header":   {http.StatusServiceUnavailable, "not a number"},
	} {
		service := fakeDepthService(t, mode)
		response := get(cellCameraServer(t, service.URL+"/camera"), cellCameraAPIPath+"/depth.z16?after=6")
		service.Close()
		if response.Code != want.code || !strings.Contains(response.Body.String(), want.text) {
			t.Errorf("%s: %d %s", mode, response.Code, response.Body.String())
		}
	}
	// A JPEG camera's service is not a depth camera.
	sequence := 42
	jpeg := fakeCameraService(t, &sequence, []byte("\xff\xd8\xff\xd9"))
	defer jpeg.Close()
	if response := get(cellCameraServer(t, jpeg.URL+"/camera"), cellCameraAPIPath+"/depth.z16"); response.Code != http.StatusNotFound {
		t.Fatalf("camera without depth: %d %s", response.Code, response.Body.String())
	}
	if response := get(NewServer(ServerOptions{DistDir: t.TempDir()}).Handler(), cellCameraAPIPath+"/depth.z16"); response.Code != http.StatusNotFound ||
		!strings.Contains(response.Body.String(), "cell_camera_not_configured") {
		t.Fatalf("no camera: %d %s", response.Code, response.Body.String())
	}
}
