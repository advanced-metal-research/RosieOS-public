package main

import (
	"bytes"
	"context"
	"encoding/json"
	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/programstore"
	"io"
	"math"
	"net/http"
	"path/filepath"
	"testing"
)

func TestIOSettingsPersistenceAndValidation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "settings", "cell.json")
	s, e := readIOSettings(path, "cell")
	if e != nil || s.Revision != 0 {
		t.Fatal(s, e)
	}
	s.Labels["O1"] = "Red light"
	s.StartDelay = .35
	s.StopDelay = .7
	s.Revision = 1
	if e = saveIOSettings(path, s); e != nil {
		t.Fatal(e)
	}
	got, e := readIOSettings(path, "cell")
	if e != nil || got.Labels["O1"] != "Red light" || got.StartDelay != .35 || got.StopDelay != .7 || got.Revision != 1 {
		t.Fatal(got, e)
	}
	if _, e = readIOSettings(path, "other-cell"); e == nil {
		t.Fatal("cross-cell settings accepted")
	}
	for _, delay := range []float64{-1, 31, math.Inf(1), math.NaN()} {
		bad := s
		bad.StartDelay = delay
		if validateIOSettings(bad) == nil {
			t.Fatal("invalid delay accepted", delay)
		}
	}
	s.Cell = "../../escape"
	if validateIOSettings(s) == nil {
		t.Fatal("invalid cell accepted")
	}
}

type ioSettingsBackend struct{ maintenanceBackend }

func (*ioSettingsBackend) Status(context.Context) *denseexec.StatusBody {
	var s denseexec.StatusBody
	_ = json.Unmarshal([]byte(`{"available":true,"target":{"cell_id":"cell"},"rt_core":{"description":{"io":{"torch_qualified":false,"inputs":[{"name":"I1","bit":0}],"outputs":[{"name":"O1","bit":0,"class":"process"},{"name":"weld","bit":4,"class":"torch"}]}}}}`), &s)
	return &s
}
func TestIOSettingsAPIAllowsRequestedOutputButRejectsStaleAndReservedAssignments(t *testing.T) {
	server, _ := newCellProgramsServer(t, t.TempDir(), &ioSettingsBackend{})
	defer server.Close()
	path := server.URL + "/api/offline-programming/v1/io-settings"
	post := func(s ioSettings, want int) {
		t.Helper()
		b, _ := json.Marshal(s)
		r, e := http.Post(path, "application/json", bytes.NewReader(b))
		if e != nil {
			t.Fatal(e)
		}
		defer r.Body.Close()
		if r.StatusCode != want {
			raw, _ := io.ReadAll(r.Body)
			t.Fatalf("status %d want %d: %s", r.StatusCode, want, raw)
		}
	}
	s := ioSettings{Cell: "cell", Labels: map[string]string{"O1": "Red light"}, WeldOutput: "O1"}
	post(s, 200)
	s.Revision = 1
	s.WeldOutput = "weld"
	s.Cell = "other-cell"
	post(s, 409)
	s.Cell = "cell"
	post(s, 200)
	post(s, 409)
	s.Revision = 2
	s.WeldOutput = "O6"
	post(s, 409)
	s.WeldOutput = "I1"
	post(s, 409)
	s.WeldOutput = "weld"
	s.Labels["O6"] = "E-stop"
	post(s, 400)
	delete(s.Labels, "O6")
	s.StartDelay = 31
	post(s, 400)
	r, e := http.Get(path)
	if e != nil {
		t.Fatal(e)
	}
	defer r.Body.Close()
	var got struct {
		Settings   ioSettings `json:"settings"`
		Controller struct {
			Qualified bool `json:"torch_qualified"`
		} `json:"controller_io"`
	}
	if json.NewDecoder(r.Body).Decode(&got) != nil || got.Settings.Revision != 2 || got.Controller.Qualified {
		t.Fatal("save lost revision or qualified hardware", got)
	}
}

// Planning binds the selection but must not reclassify or qualify hardware.
func TestIOSettingsBindsOutputWithoutQualifyingController(t *testing.T) {
	store, err := programstore.Open(t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	defer store.Close()
	server := NewServer(ServerOptions{DenseExecution: &ioSettingsBackend{}, ProgramStore: store})
	for _, requested := range []string{"", "O1", "weld"} {
		err = saveIOSettings(filepath.Join(store.Dir(), "io-settings", "cell.json"), ioSettings{Cell: "cell", WeldOutput: requested})
		if err != nil {
			t.Fatal(err)
		}
		outputs := map[string]any{"torch_qualified": false, "outputs": []map[string]any{{"name": "weld", "class": "torch"}, {"name": "O1", "class": "process"}}}
		if err = server.applyOperatorWeldAssignment(context.Background(), outputs); err != nil {
			t.Fatal(err)
		}
		if outputs["torch_qualified"] != false || outputs["weld_output"] != requested {
			t.Fatalf("request %q changed qualification incorrectly: %v", requested, outputs)
		}
		if outputs["outputs"].([]map[string]any)[1]["class"] != "process" {
			t.Fatal("request reclassified hardware")
		}
	}
}
