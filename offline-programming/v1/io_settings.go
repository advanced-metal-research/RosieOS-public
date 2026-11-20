package main

// Operator metadata is shared by the cell's program store, independently of
// deployment directories. It never rewrites the commissioned native I/O map.
import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"sync"
)

type ioSettings struct {
	Cell       string            `json:"cell"`
	Revision   uint64            `json:"revision"`
	Labels     map[string]string `json:"labels"`
	WeldOutput string            `json:"weld_output"`
	StartDelay float64           `json:"start_delay_s"`
	StopDelay  float64           `json:"stop_delay_s"`
}

var ioSettingsMu sync.Mutex
var ioSettingKey = regexp.MustCompile(`^[A-Za-z0-9_-]{1,80}$`)

func validateIOSettings(s ioSettings) error {
	if !ioSettingKey.MatchString(s.Cell) || len(s.Labels) > 32 {
		return errors.New("invalid cell or too many labels")
	}
	for k, v := range s.Labels {
		if !ioSettingKey.MatchString(k) || len(v) > 64 || strings.ContainsAny(v, "\r\n\x00") {
			return errors.New("labels must be single-line text, at most 64 bytes")
		}
	}
	for _, v := range []float64{s.StartDelay, s.StopDelay} {
		if math.IsNaN(v) || math.IsInf(v, 0) || v < 0 || v > 30 {
			return errors.New("weld delays must be 0–30 seconds")
		}
	}
	return nil
}
func readIOSettings(path, cell string) (ioSettings, error) {
	s := ioSettings{Cell: cell, Labels: map[string]string{}}
	b, e := os.ReadFile(path)
	if os.IsNotExist(e) {
		return s, nil
	}
	if e != nil {
		return s, e
	}
	if len(b) > 16384 {
		return s, errors.New("settings file too large")
	}
	if e = json.Unmarshal(b, &s); e != nil {
		return s, e
	}
	if s.Cell != cell {
		return s, errors.New("settings cell mismatch")
	}
	return s, validateIOSettings(s)
}
func saveIOSettings(path string, s ioSettings) error {
	b, e := json.MarshalIndent(s, "", "  ")
	if e != nil {
		return e
	}
	if e = os.MkdirAll(filepath.Dir(path), 0700); e != nil {
		return e
	}
	f, e := os.CreateTemp(filepath.Dir(path), ".io-settings-*")
	if e != nil {
		return e
	}
	defer os.Remove(f.Name())
	if _, e = f.Write(append(b, '\n')); e == nil {
		e = f.Sync()
	}
	closeErr := f.Close()
	if e != nil {
		return e
	}
	if closeErr != nil {
		return closeErr
	}
	return os.Rename(f.Name(), path)
}
func (server *Server) ioSettingsRoute(w http.ResponseWriter, r *http.Request) {
	if server.programs == nil || server.denseExecution == nil {
		writeDenseError(w, 409, "io_settings_unavailable", "Connect to a cell with shared program storage")
		return
	}
	var observed struct {
		Target struct {
			Cell string `json:"cell_id"`
		} `json:"target"`
		RT struct {
			Description struct {
				IO json.RawMessage `json:"io"`
			} `json:"description"`
		} `json:"rt_core"`
	}
	raw, e := json.Marshal(server.denseExecution.Status(r.Context()))
	if e != nil || json.Unmarshal(raw, &observed) != nil || !ioSettingKey.MatchString(observed.Target.Cell) {
		writeDenseError(w, 409, "io_settings_unavailable", "Cell identity unavailable")
		return
	}
	cell := observed.Target.Cell
	path := filepath.Join(server.programs.store.Dir(), "io-settings", cell+".json")
	ioSettingsMu.Lock()
	defer ioSettingsMu.Unlock()
	current, e := readIOSettings(path, cell)
	if _, stat := os.Stat(path); os.IsNotExist(stat) {
		current, e = readIOSettings(filepath.Join(server.repoRoot, "offline-programming", "v1", "config", "operator-io", cell+".json"), cell)
	}
	if e != nil {
		writeDenseError(w, 500, "io_settings_read_failed", e.Error())
		return
	}
	var definition struct {
		Inputs []struct {
			Name string `json:"name"`
		} `json:"inputs"`
		Outputs []struct {
			Name  string `json:"name"`
			Class string `json:"class"`
		} `json:"outputs"`
	}
	if json.Unmarshal(observed.RT.Description.IO, &definition) != nil {
		writeDenseError(w, 409, "io_settings_unavailable", "Controller I/O description unavailable")
		return
	}
	keys := map[string]bool{}
	assignable := map[string]bool{}
	for _, c := range definition.Inputs {
		keys[c.Name] = true
	}
	for _, c := range definition.Outputs {
		keys[c.Name] = true
		if c.Class == "torch" || c.Class == "process" {
			assignable[c.Name] = true
		}
	}
	if r.Method == http.MethodPost {
		var next ioSettings
		if e = decodeBoundedJSON(w, r, 16384, &next); e != nil {
			writeDenseError(w, 400, "io_settings_invalid", e.Error())
			return
		}
		if e = validateIOSettings(next); e != nil {
			writeDenseError(w, 400, "io_settings_invalid", e.Error())
			return
		}
		if next.Cell != cell || next.Revision != current.Revision {
			writeDenseError(w, 409, "io_settings_changed", "Reload the current cell settings before saving")
			return
		}
		// Save the requested channel without reclassifying or qualifying hardware.
		// Planning resolves this name against the unchanged native output policy.
		if next.WeldOutput != "" && !assignable[next.WeldOutput] {
			writeDenseError(w, 409, "io_assignment_invalid", "Select an available process or weld output; reserved and unknown channels cannot be assigned")
			return
		}
		for k := range next.Labels {
			if !keys[k] {
				writeDenseError(w, 400, "io_settings_invalid", fmt.Sprintf("Unknown channel %s", k))
				return
			}
		}
		next.Revision++
		if e = saveIOSettings(path, next); e != nil {
			writeDenseError(w, 500, "io_settings_save_failed", e.Error())
			return
		}
		current = next
	}
	writeJSON(w, 200, map[string]any{"settings": current, "controller_io": json.RawMessage(observed.RT.Description.IO)})
}

// Bind newly planned weld boundaries to the saved channel. This does not
// reclassify hardware or qualify a torch; both planner and native Load retain
// the described channel policy. Existing immutable plans keep their schedule.
func (server *Server) applyOperatorWeldAssignment(rctx context.Context, outputs map[string]any) error {
	if server.programs == nil {
		return nil
	}
	s := server.denseStatus(rctx)
	if s == nil || s.Target == nil || !ioSettingKey.MatchString(s.Target.CellID) {
		return errors.New("cell identity unavailable for weld setup")
	}
	cell := s.Target.CellID
	path := filepath.Join(server.programs.store.Dir(), "io-settings", cell+".json")
	if _, err := os.Stat(path); os.IsNotExist(err) {
		path = filepath.Join(server.repoRoot, "offline-programming", "v1", "config", "operator-io", cell+".json")
	}
	// Cells without operator setup retain their existing commissioned behavior.
	if _, err := os.Stat(path); os.IsNotExist(err) {
		return nil
	}
	config, err := readIOSettings(path, cell)
	if err != nil {
		return err
	}
	// Explicit empty means unassigned, distinct from legacy cells without settings.
	outputs["weld_output"] = config.WeldOutput
	return nil
}
