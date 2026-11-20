// Dense execution HTTP surface uses the selected native controller. Motion
// requires the displayed selection identity; cell management grants no authority.
package main

import (
	"bytes"
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"io"
	"log"
	"net/http"
	"strconv"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
)

// Unverified request bound; oversized browser input is refused before admission.
const maxDenseRequestBytes = 64 << 10

// Load may carry the client's kept trajectory (dense_blob_base64): the
// planner fetch's own 64 MiB frame cap as base64, plus the identity fields.
var maxDenseLoadRequestBytes = int64(maxDenseRequestBytes + base64.StdEncoding.EncodedLen(denseexec.MaxFrameBytes))

// denseTargetRequests fences every mutating execution route, including future
// jog/move routes, against the identity displayed when the client issued it.
// Only explicit inhibition and renewal routes bypass selection identity.
func (server *Server) denseTargetRequests(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		picker, selected := server.denseExecution.(*targets.Picker)
		if selected && r.Method == http.MethodPost && strings.HasPrefix(r.URL.Path, "/api/offline-programming/v1/local-simulator/jog/") && !strings.HasSuffix(r.URL.Path, "/stop") && !strings.HasSuffix(r.URL.Path, "/halt") {
			if err := picker.WithPreview(func() { next.ServeHTTP(w, r) }); err != nil {
				writeDenseFailure(w, "target", err)
			}
			return
		}
		if !selected || r.Method != http.MethodPost || !strings.HasPrefix(r.URL.Path, denseExecutionPath+"/") {
			next.ServeHTTP(w, r)
			return
		}
		path := strings.TrimPrefix(r.URL.Path, denseExecutionPath)
		safe := path == "/target" || path == "/cells" || path == "/stop" || path == "/pause" || path == "/heartbeat" || path == "/jog/stop" || path == "/jog/end" || path == "/trace/end" || path == "/run-program/finish"
		if path == "/arm" {
			raw, err := io.ReadAll(http.MaxBytesReader(w, r.Body, maxDenseRequestBytes))
			if err != nil {
				writeDenseError(w, http.StatusBadRequest, "dense_arm_invalid", "arm request is invalid")
				return
			}
			r.Body = io.NopCloser(bytes.NewReader(raw))
			var input struct {
				Armed *bool `json:"armed"`
			}
			safe = json.Unmarshal(raw, &input) == nil && input.Armed != nil && !*input.Armed
		}
		if safe {
			next.ServeHTTP(w, r)
			return
		}
		generation, _ := strconv.ParseUint(r.Header.Get("X-RT-Target-Generation"), 10, 64)
		if err := picker.WithTarget(generation, r.Header.Get("X-RT-Target-Cell"), func() { next.ServeHTTP(w, r) }); err != nil {
			writeDenseFailure(w, "target", err)
		}
	})
}

func (server *Server) denseExecutionCapabilities(writer http.ResponseWriter, request *http.Request) {
	writeJSON(writer, http.StatusOK, struct {
		denseexec.Capabilities
		TestPrograms *testProgramsCapability `json:"test_programs,omitempty"`
	}{server.denseCells.Capabilities(request.Context()), server.testProgramsCapability()})
}

func (server *Server) denseExecutionStatus(writer http.ResponseWriter, request *http.Request) {
	status := &denseexec.StatusBody{Available: false}
	currentOnly := request.URL.Query().Get("view") == "control"
	if server.denseExecution != nil {
		read := server.denseExecution.Status
		if observer, ok := server.denseExecution.(denseexec.ControlStatusReader); currentOnly && ok {
			read = observer.ControlStatus
		}
		status = read(request.Context())
	}
	if currentOnly {
		writer.Header().Set("Vary", "Accept-Encoding")
		writeJSON(writer, http.StatusOK, denseexec.ControlStatusBody(status))
		return
	}
	writeJSON(writer, http.StatusOK, status)
}

// denseExecutionTelemetryWindow answers the newest seconds of the connected
// machine's telemetry ring, thinned by stride, for the operator's strip. The
// read is bounded by the ring (30 s) and by the link: one second is 4.9 MB
// from the cell before thinning, so the handler allows the fetch time it needs.
func (server *Server) denseExecutionTelemetryWindow(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	query := request.URL.Query()
	seconds := 3.0
	if raw := strings.TrimSpace(query.Get("seconds")); raw != "" {
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil || !(v > 0) || v > 30 {
			writeDenseError(writer, http.StatusBadRequest, "telemetry_window_invalid", "seconds must be a number in (0, 30]")
			return
		}
		seconds = v
	}
	stride := 1
	if raw := strings.TrimSpace(query.Get("stride")); raw != "" {
		v, err := strconv.Atoi(raw)
		if err != nil || v < 1 || v > 100 {
			writeDenseError(writer, http.StatusBadRequest, "telemetry_window_invalid", "stride must be an integer in [1, 100]")
			return
		}
		stride = v
	}
	// Unverified fetch budget: 30 s of ring over a 0.8 MB/s link is about 180 s.
	ctx, cancel := context.WithTimeout(request.Context(), 240*time.Second)
	defer cancel()
	window, err := server.denseExecution.TelemetryWindow(ctx, seconds, stride)
	if err != nil {
		writeDenseFailure(writer, "telemetry window", err)
		return
	}
	writeJSON(writer, http.StatusOK, window)
}

func (server *Server) denseExecutionLoad(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	var input denseexec.LoadRequest
	if err := decodeBoundedJSON(writer, request, maxDenseLoadRequestBytes, &input); err != nil {
		writeDenseError(writer, http.StatusBadRequest, "dense_load_invalid", "load request is not valid JSON: "+err.Error())
		return
	}
	result, err := server.denseExecution.Load(request.Context(), input)
	if err != nil {
		writeDenseFailure(writer, "load", err)
		return
	}
	server.programs.noteLoaded(input.ProgramID, result.SessionID)
	kept := ""
	if input.DenseBlobBase64 != "" {
		kept = " (client sent its kept copy)"
	}
	if input.Source != "" {
		log.Printf("dense-execution: load %s (source %s) -> session %s", input.TrajectoryDigest, input.Source, result.SessionID)
	} else {
		log.Printf("dense-execution: load %s%s -> session %s, bytes from %s", input.TrajectoryDigest, kept, result.SessionID, result.BlobSource)
	}
	writeJSON(writer, http.StatusOK, result)
}

func (server *Server) denseExecutionPlay(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	server.denseSessionCommand(writer, request, "play", server.denseExecution.Play)
}

func (server *Server) denseExecutionPause(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	server.denseSessionCommand(writer, request, "pause", server.denseExecution.Pause)
}

// denseExecutionHome establishes native Home, keeping it distinct from travel
// to a saved Go Home pose so a button cannot substitute one motion for another.
func (server *Server) denseExecutionHome(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	controller, ok := server.denseExecution.(interface{ Home(context.Context) error })
	if !ok {
		writeDenseError(writer, http.StatusConflict, "capability_unimplemented", "native Home is unavailable on this backend")
		return
	}
	// An optional body names the axes to home ({"axes": [index, ...]}); no
	// body, or no axes, homes every configured axis as before.
	var input struct {
		Axes []int `json:"axes"`
	}
	if request.ContentLength != 0 {
		if err := decodeBoundedJSON(writer, request, maxDenseRequestBytes, &input); err != nil && !errors.Is(err, io.EOF) {
			writeDenseError(writer, http.StatusBadRequest, "home_axes_invalid", `request body must be {"axes": [index, ...]} or empty`)
			return
		}
	}
	if len(input.Axes) > 0 {
		var mask uint32
		for _, axis := range input.Axes {
			if axis < 0 || axis > 15 {
				writeDenseError(writer, http.StatusBadRequest, "home_axes_invalid", "axis indices must be in [0, 15]")
				return
			}
			mask |= 1 << uint(axis)
		}
		axes, ok := server.denseExecution.(interface {
			HomeAxes(context.Context, uint32) error
		})
		if !ok {
			writeDenseError(writer, http.StatusConflict, "capability_unimplemented", "per-axis Home is unavailable on this backend")
			return
		}
		if err := axes.HomeAxes(request.Context(), mask); err != nil {
			writeDenseFailure(writer, "home", err)
			return
		}
	} else if err := controller.Home(request.Context()); err != nil {
		writeDenseFailure(writer, "home", err)
		return
	}
	writeJSON(writer, http.StatusOK, server.denseExecution.Status(request.Context()))
}

// denseSessionCommand is play and pause: {session_id} in, status out.
func (server *Server) denseSessionCommand(
	writer http.ResponseWriter,
	request *http.Request,
	name string,
	command func(ctx context.Context, sessionID string) (*denseexec.StatusBody, error),
) {
	if !server.denseExecutionReady(writer) {
		return
	}
	sessionID, ok := decodeDenseSessionID(writer, request)
	if !ok {
		return
	}
	log.Printf("dense-execution: %s session %s requested by %s", name, sessionID, clientIdentityOf(request))
	status, err := command(request.Context(), sessionID)
	if err != nil {
		writeDenseFailure(writer, name, err)
		return
	}
	writeJSON(writer, http.StatusOK, status)
}

func (server *Server) denseExecutionStop(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	log.Printf("dense-execution: stop requested by %s", clientIdentityOf(request))
	result, err := server.denseExecution.Stop(request.Context())
	if err != nil {
		var denseErr *denseexec.Error
		if errors.As(err, &denseErr) && result != nil {
			writeJSON(writer, denseErr.Status, map[string]any{
				"error":    denseErr.Code,
				"detail":   denseErr.Detail,
				"ok":       false,
				"stopped":  result.Stopped,
				"attempts": result.Attempts,
				"status":   result.Status,
			})
			return
		}
		writeDenseFailure(writer, "stop", err)
		return
	}
	writeJSON(writer, http.StatusOK, result)
}

func (server *Server) denseExecutionArm(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	var input struct {
		Armed *bool `json:"armed"`
		// Program is the pendant's Run-page "Arm for program": an arm that
		// stays latched with no held dead-man (L2), after which Once, loops
		// and go-to-start run without L2 and losing window focus no longer
		// stops them. Owner decision (2026-11-13): admitted only on a cell
		// whose manifest sets cell_deploy.olp.unattended_loop_allowed.
		Program bool `json:"program,omitempty"`
	}
	if err := decodeBoundedJSON(writer, request, maxDenseRequestBytes, &input); err != nil || input.Armed == nil {
		writeDenseError(writer, http.StatusBadRequest, "dense_arm_invalid", `arm request needs {"armed": true|false}`)
		return
	}
	// The ordinary line stays as it was; a program arm says so.
	program := ""
	if input.Program {
		program = " program=true"
	}
	log.Printf("dense-execution: arm=%t%s requested by %s", *input.Armed, program, clientIdentityOf(request))
	if *input.Armed && input.Program && !server.unattendedLoopAllowed() {
		writeDenseError(writer, http.StatusConflict, "program_arm_not_allowed",
			"Arm for program is not allowed on this cell: its manifest does not set cell_deploy.olp.unattended_loop_allowed. Hold L2 to arm; nothing was sent.")
		return
	}
	// Arm first confirms an unconfirmed Stop and resets faults whose recovery
	// is a reset (denseexec settleForArm); a refusal keeps the steps taken.
	result, err := server.denseExecution.Arm(request.Context(), *input.Armed)
	var denseErr *denseexec.Error
	if err != nil && result != nil && len(result.Steps) > 0 && errors.As(err, &denseErr) {
		log.Printf("dense-execution: arm failed: %s: %s", denseErr.Code, denseErr.Detail)
		writeJSON(writer, denseErr.Status, map[string]any{"error": denseErr.Code, "detail": denseErr.Detail, "steps": result.Steps})
		return
	}
	if err != nil {
		writeDenseFailure(writer, "arm", err)
		return
	}
	writeJSON(writer, http.StatusOK, result)
}

// denseExecutionGoHome: POST {} -> the status body plus ok. The session
// the run happens in (existing, or a fresh go_home one) is in
// status.session.id, which heartbeat needs.
func (server *Server) denseExecutionGoHome(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	log.Printf("dense-execution: go-home requested by %s", clientIdentityOf(request))
	result, err := server.denseExecution.GoHome(request.Context())
	if err != nil {
		writeDenseFailure(writer, "go-home", err)
		return
	}
	writeJSON(writer, http.StatusOK, result)
}

func (server *Server) denseExecutionHeartbeat(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	sessionID, ok := decodeDenseSessionID(writer, request)
	if !ok {
		return
	}
	var result *denseexec.HeartbeatResult
	var err error
	if from, ok := server.denseExecution.(interface {
		HeartbeatFrom(context.Context, string) (*denseexec.HeartbeatResult, error)
	}); ok {
		// The client is named so an attended loop knows its starter's beat.
		result, err = from.HeartbeatFrom(request.Context(), sessionID)
	} else {
		result, err = server.denseExecution.Heartbeat(sessionID)
	}
	if err != nil {
		writeDenseFailure(writer, "heartbeat", err)
		return
	}
	writeJSON(writer, http.StatusOK, result)
}

// denseExecutionDriveHealth answers the selected cell's drive health for the
// drive page (ui/drives.html, docs/decisions/0001): per joint the drive's
// heatsink temperature and the last second of torque and following error,
// from one ordinary status read. Read-only; it touches no robot state.
func (server *Server) denseExecutionDriveHealth(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	health, err := denseexec.DriveHealthFrom(server.denseExecution.Status(request.Context()))
	if err != nil {
		writeDenseError(writer, http.StatusServiceUnavailable, "drive_health_unavailable", err.Error())
		return
	}
	writeJSON(writer, http.StatusOK, health)
}

// denseExecutionReady writes the 503 and returns false when the target is
// not configured.
// unattendedLoopAllowed is the selected cell's
// cell_deploy.olp.unattended_loop_allowed; false when no cell is selected or
// the backend cannot say, so a program arm fails closed.
func (server *Server) unattendedLoopAllowed() bool {
	backend, ok := server.denseExecution.(interface{ UnattendedLoopAllowed() bool })
	return ok && backend.UnattendedLoopAllowed()
}

func (server *Server) denseExecutionReady(writer http.ResponseWriter) bool {
	if server.denseExecution == nil || !server.denseExecution.Available() {
		writeDenseError(writer, http.StatusServiceUnavailable, "dense_target_unavailable",
			"no cell selected: choose a commissioned cell from Cells")
		return false
	}
	return true
}

func decodeDenseSessionID(writer http.ResponseWriter, request *http.Request) (string, bool) {
	var input struct {
		SessionID string `json:"session_id"`
	}
	if err := decodeBoundedJSON(writer, request, maxDenseRequestBytes, &input); err != nil || input.SessionID == "" {
		writeDenseError(writer, http.StatusBadRequest, "dense_session_required", `request needs {"session_id": "..."}`)
		return "", false
	}
	return input.SessionID, true
}

func writeDenseError(writer http.ResponseWriter, status int, code, detail string) {
	writeJSON(writer, status, map[string]any{"error": code, "detail": detail})
}

// writeDenseFailure maps a manager error onto {"error", "detail"}.
func writeDenseFailure(writer http.ResponseWriter, action string, err error) {
	var denseErr *denseexec.Error
	switch {
	case errors.As(err, &denseErr):
		log.Printf("dense-execution: %s failed: %s: %s", action, denseErr.Code, denseErr.Detail)
		payload := map[string]any{"error": denseErr.Code, "detail": denseErr.Detail}
		if denseErr.LimitViolation != nil {
			payload["limit_violation"] = denseErr.LimitViolation
		}
		writeJSON(writer, denseErr.Status, payload)
	case errors.Is(err, denseexec.ErrUnavailable):
		writeDenseError(writer, http.StatusServiceUnavailable, "dense_target_unavailable", err.Error())
	default:
		log.Printf("dense-execution: %s failed: %v", action, err)
		writeDenseError(writer, http.StatusInternalServerError, "dense_internal", err.Error())
	}
}

// denseExecutionCell: the selected target's machine_planning_calibration.json and robot identity
// (docs/ROBOT-DESCRIPTION-CONTRACT.md). Not to be confused with /cells, the
// registry of targets: this is what one target says about itself, and the
// bytes the next plan request carries to the planner.
func (server *Server) denseExecutionCell(writer http.ResponseWriter, request *http.Request) {
	if !server.denseExecutionReady(writer) {
		return
	}
	target, ok := server.denseExecution.(CellTarget)
	if !ok || target == nil {
		writeDenseError(writer, http.StatusConflict, "robot_cell_unavailable",
			"the execution backend does not say which robot it is")
		return
	}
	result, err := target.Cell(request.Context())
	if err != nil {
		writeDenseFailure(writer, "cell", err)
		return
	}
	writeJSON(writer, http.StatusOK, result)
}
