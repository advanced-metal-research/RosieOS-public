package main

import (
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
)

// The relay passes a viewer's offer to the sender with the viewer's name,
// returns the answer with a Location on this server, and ends the session.
func TestCellVideoRelaysWHEPToTheSender(t *testing.T) {
	var gotClient, gotOffer, ended string
	sender := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		switch {
		case r.Method == http.MethodPost && r.URL.Path == "/whep":
			body, _ := io.ReadAll(r.Body)
			gotClient, gotOffer = r.Header.Get("X-Rosie-Client"), string(body)
			w.Header().Set("Content-Type", "application/sdp")
			w.Header().Set("Location", "/whep/0123456789abcdef")
			w.WriteHeader(http.StatusCreated)
			io.WriteString(w, "v=0 answer")
		case r.Method == http.MethodDelete:
			ended = r.URL.Path
			io.WriteString(w, "ended")
		case r.URL.Path == "/status":
			io.WriteString(w, `{"viewers":[]}`)
		}
	}))
	defer sender.Close()
	video, err := NewCellVideo(sender.URL)
	if err != nil {
		t.Fatal(err)
	}
	server := &Server{cellVideo: video}

	req := httptest.NewRequest(http.MethodPost, cellVideoAPIPath+"/whep", strings.NewReader("v=0 offer m=video"))
	req.Header.Set("Content-Type", "application/sdp")
	req.Header.Set("X-Rosie-Client", "rosie-pendant steamdeck-viewer")
	rec := httptest.NewRecorder()
	server.cellVideoOffer(rec, req)
	if rec.Code != http.StatusCreated || rec.Body.String() != "v=0 answer" || rec.Header().Get("Location") != cellVideoAPIPath+"/whep/0123456789abcdef" {
		t.Fatalf("offer: %d %q %q", rec.Code, rec.Body.String(), rec.Header().Get("Location"))
	}
	if gotClient != "rosie-pendant steamdeck-viewer" || gotOffer != "v=0 offer m=video" {
		t.Fatalf("sender got client %q offer %q", gotClient, gotOffer)
	}

	del := httptest.NewRequest(http.MethodDelete, cellVideoAPIPath+"/whep/0123456789abcdef", nil)
	del.SetPathValue("id", "0123456789abcdef")
	rec = httptest.NewRecorder()
	server.cellVideoEnd(rec, del)
	if rec.Code != http.StatusOK || ended != "/whep/0123456789abcdef" {
		t.Fatalf("end: %d, sender saw %q", rec.Code, ended)
	}

	bad := httptest.NewRequest(http.MethodDelete, cellVideoAPIPath+"/whep/../status", nil)
	bad.SetPathValue("id", "../status")
	rec = httptest.NewRecorder()
	server.cellVideoEnd(rec, bad)
	if rec.Code != http.StatusNotFound {
		t.Fatalf("a malformed session id reached the sender: %d", rec.Code)
	}
	rec = httptest.NewRecorder()
	server.cellVideoStatus(rec, httptest.NewRequest(http.MethodGet, cellVideoAPIPath, nil))
	if rec.Code != http.StatusOK || !strings.Contains(rec.Body.String(), "viewers") {
		t.Fatalf("status: %d %s", rec.Code, rec.Body.String())
	}
}

func TestCellVideoAbsentOrMisconfigured(t *testing.T) {
	server := &Server{}
	rec := httptest.NewRecorder()
	server.cellVideoStatus(rec, httptest.NewRequest(http.MethodGet, cellVideoAPIPath, nil))
	if rec.Code != http.StatusNotFound {
		t.Fatalf("no video: %d", rec.Code)
	}
	for _, origin := range []string{"http://10.0.0.5:8090", "https://127.0.0.1:8090", "http://127.0.0.1:8090/whep", "http://u:p@127.0.0.1:8090"} {
		if _, err := NewCellVideo(origin); err == nil {
			t.Errorf("origin %q was accepted", origin)
		}
	}
	if v, err := NewCellVideo(""); v != nil || err != nil {
		t.Fatalf("empty origin: %v %v", v, err)
	}
}

// Video set-up is not a robot action: neither the control gate nor a
// maintenance notice stops a viewer.
func TestCellVideoIsNotARobotAction(t *testing.T) {
	for _, path := range []string{cellVideoAPIPath + "/whep", cellVideoAPIPath + "/whep/0123456789abcdef"} {
		if _, robot := maintenanceRobotAction(path); robot {
			t.Errorf("%s counts as a robot action", path)
		}
	}
}
