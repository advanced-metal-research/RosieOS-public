package main

import (
	"bytes"
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"net/http"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/robotconfig"
	"rosieos/offline-programming/v1/internal/seam"
)

// WeldPlanAPIPath plans a rails program: pack, plan, and fetch the trajectory
// the robot will actually play, in one request.
//
// It used to be one route on the weld planner's authoring server, which packed
// the plan request, posted it to the motion server and compiled the result to
// spreadsheet rows. That server is gone (D24) and so are the rows. What is left
// is the middle: a `.weldplan` from the seam worker, a plan from the motion
// server, and the dense blob brought back here rather than left where it was
// filed.
const WeldPlanAPIPath = "/api/offline-programming/v1/weld-plan"

// WeldPlanExportAPIPath packs the same `.weldplan` and hands back the bytes
// instead of planning them.
//
// The container is what a planner is actually given -- program.v2, the
// placement inside it, the cell descriptor, the tooling and the STEP, hashed
// together -- so it is the one artifact that reproduces a plan somewhere
// else. Exporting the project JSON instead exports what the OLP holds, which
// is a different document and cannot be handed to the motion server at all.
const WeldPlanExportAPIPath = "/api/offline-programming/v1/weld-plan/export"

const (
	weldPlanMaxRequestBytes = int64(24 << 20)
	weldPlanMaxResponse     = int64(256 << 20)
	weldPlanTimeout         = 30 * time.Minute
)

type weldPlanRequest struct {
	TraceFromHeldTarget  bool `json:"trace_from_held_target"`
	RequireMeasuredStart bool `json:"require_measured_start"`
	// PlanFromProgramStart is the operator's explicit alternative to the
	// measured start: plan from the program's own start (the model reset
	// pose) with no capture, then Go to plan start before Load and Play.
	// Chosen when the measured pose cannot be used (a capture refused, a
	// first leg the planner cannot verify from where the robot stands); it is
	// never a silent fallback, and asking for both is refused.
	PlanFromProgramStart bool `json:"plan_from_program_start"`
	// Program is a `robot.v4.program.v2` document.
	Program json.RawMessage `json:"program"`
	// Tooling is the fitted gun, as `tooling.json` inside the container.
	Tooling         json.RawMessage `json:"tooling"`
	STEPBase64      string          `json:"step_base64"`
	STEPFilename    string          `json:"step_filename"`
	WorkpieceAbsent bool            `json:"workpiece_absent"`
	// CellID names the URDF under `robot_description`. The browser never learns a path;
	// this process reads the file and the seam worker parses it.
	CellID           string `json:"cell_id"`
	FreeSpaceBackend string `json:"free_space_backend"`
	// The cell's own settings are NOT taken from this request while a cell is
	// connected: they are the cell's, so the server fetches them from the
	// target when it packs (docs/ROBOT-DESCRIPTION-CONTRACT.md); a browser
	// cannot state what the robot it is talking to has been calibrated to.
	// MachinePlanningCalibrationBase64 is for a plan made with NO cell to ask -- a machine_planning_calibration.json
	// the person uploaded, or pulled earlier -- and is read only then. It is
	// labelled as the target's (docs/PLANNING-CONTEXT.md, Offline): the gate
	// is the hash, and the cell that plays the plan compares it.
	MachinePlanningCalibrationBase64 string `json:"machine_planning_calibration_base64"`
	ProgramID                        string `json:"program_id"`
	ProgramDigest                    string `json:"program_digest"`
	ManifestRevision                 int    `json:"manifest_revision"`
	PlanRevision                     int    `json:"plan_revision"`
	// Fixtures are the operator's workcell bodies (a stand under the part),
	// in the cell's world frame, passed to the seam worker verbatim: it packs
	// them as `fixtures.json` and the planner avoids them instead of its own
	// placeholder. Absent, the seam worker keeps the cell's placeholder.
	Fixtures json.RawMessage `json:"fixtures"`
}

type weldPlanResponse struct {
	// Result is the planner's own document, unchanged: seam trajectories as
	// Hermite wires, the connecting moves, and the verdicts behind both.
	Result json.RawMessage `json:"result"`
	// WeldplanSHA256 names the request this answers, which is what Weld
	// Preflight and the dense daemon both fence on.
	WeldplanSHA256 string `json:"weldplan_sha256"`
	// ProgramDigest is echoed from the request. A trajectory kept beside it
	// can then say which program produced it, rather than being read against
	// whichever one happens to be open when someone replays it.
	ProgramDigest string `json:"program_digest,omitempty"`
	// DenseBlobBase64 is the trajectory the robot plays, brought back at plan
	// time rather than left on the motion server. That store is a cache under
	// `~/.cache`; a project that needed it later and found it evicted would
	// have nothing, and nothing else has a copy.
	DenseBlobBase64 string          `json:"dense_blob_base64,omitempty"`
	Dense           json.RawMessage `json:"dense,omitempty"`
	DenseError      json.RawMessage `json:"dense_error,omitempty"`
}

// weldPlanOutcome is one /weld-plan or /weld-plan/export answer, logged as
// one line when the handler returns, as dense-execution logs Load and Play:
// which client asked, for which program, what it was answered and where the
// time went. Before this a refused plan was visible only on the client that
// asked (2026-10-10: no plan line at all in the relay's OLP journal).
type weldPlanOutcome struct {
	route, client, program string
	started                time.Time
	pack, plan, fetch      time.Duration
	status                 int
	code, message          string
}

func newWeldPlanOutcome(route string, request *http.Request) *weldPlanOutcome {
	_, label, _ := classifyClient(request)
	return &weldPlanOutcome{route: route, client: label, started: time.Now()}
}

// refuse writes a refusal and records it for the log line.
func (o *weldPlanOutcome) refuse(writer http.ResponseWriter, status int, payload map[string]any) {
	o.status, o.code, o.message = status, fmt.Sprint(payload["code"]), fmt.Sprint(payload["error"])
	writeJSON(writer, status, payload)
}

func (o *weldPlanOutcome) log() {
	program := o.program
	if program == "" {
		program = "-"
	}
	line := fmt.Sprintf("%s: program %s from %s -> %d %s", o.route, program, o.client, o.status, o.code)
	if o.message != "" {
		message := o.message
		if len(message) > 400 {
			message = message[:400] + "…"
		}
		line += ": " + message
	}
	ms := func(d time.Duration) int64 { return d.Milliseconds() }
	log.Printf("%s (pack %d ms, plan %d ms, fetch %d ms, total %d ms)", line, ms(o.pack), ms(o.plan), ms(o.fetch), ms(time.Since(o.started)))
}

// weldPlanFailure is the status and code a pack failure answers with: a
// measured-start refusal under its own code (409 while the robot is still
// moving, so a client can tell "let go and retry" from the rest; 400
// otherwise), anything else as the seam worker's.
func weldPlanFailure(err error) (int, string) {
	var refusal *planStartRefusal
	if errors.As(err, &refusal) {
		if refusal.code == planStartMoving {
			return http.StatusConflict, refusal.code
		}
		return http.StatusBadRequest, refusal.code
	}
	status, code := seamHTTPError(err)
	return status, string(code)
}

// weldPlanExport packs a plan request and returns it as a file.
func (server *Server) weldPlanExport(writer http.ResponseWriter, request *http.Request) {
	outcome := newWeldPlanOutcome("weld-plan/export", request)
	defer outcome.log()
	if server.seams == nil {
		outcome.refuse(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "code": "seam_worker_unavailable",
			"error": "the seam worker is unavailable, so no plan request can be packed",
		})
		return
	}
	request.Body = http.MaxBytesReader(writer, request.Body, weldPlanMaxRequestBytes)
	var body weldPlanRequest
	if err := json.NewDecoder(request.Body).Decode(&body); err != nil {
		outcome.refuse(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "weld_plan_invalid",
			"error": "request is not valid JSON: " + err.Error(),
		})
		return
	}
	outcome.program = body.ProgramID
	ctx, cancel := context.WithTimeout(request.Context(), weldPlanTimeout)
	defer cancel()

	packStarted := time.Now()
	weldplan, sha, err := server.packWeldPlan(ctx, body)
	outcome.pack = time.Since(packStarted)
	if err != nil {
		status, code := weldPlanFailure(err)
		outcome.refuse(writer, status, map[string]any{"ok": false, "code": code, "error": err.Error()})
		return
	}
	outcome.status, outcome.code = http.StatusOK, "exported "+sha
	name := strings.TrimSpace(body.ProgramID)
	if name == "" {
		name = "program"
	}
	writer.Header().Set("Content-Type", "application/vnd.gradient.amr-weld-planner-v1.plan-request")
	writer.Header().Set("Content-Disposition", fmt.Sprintf("attachment; filename=%q", name+".weldplan"))
	writer.Header().Set("X-Weldplan-SHA256", sha)
	writer.WriteHeader(http.StatusOK)
	_, _ = writer.Write(weldplan)
}

func (server *Server) weldPlan(writer http.ResponseWriter, request *http.Request) {
	outcome := newWeldPlanOutcome("weld-plan", request)
	defer outcome.log()
	if server.seams == nil {
		outcome.refuse(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "code": "seam_worker_unavailable",
			"error": "the seam worker is unavailable, so no plan request can be packed",
		})
		return
	}
	if strings.TrimSpace(server.motionOrigin) == "" {
		outcome.refuse(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "code": "motion_origin_unavailable",
			"error": "no weld planner motion origin configured; set " +
				"OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN",
		})
		return
	}

	request.Body = http.MaxBytesReader(writer, request.Body, weldPlanMaxRequestBytes)
	var body weldPlanRequest
	if err := json.NewDecoder(request.Body).Decode(&body); err != nil {
		outcome.refuse(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "weld_plan_invalid",
			"error": "request is not valid JSON: " + err.Error(),
		})
		return
	}
	outcome.program = body.ProgramID
	step, err := base64.StdEncoding.DecodeString(body.STEPBase64)
	if err != nil || (len(step) == 0) != body.WorkpieceAbsent {
		outcome.refuse(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "weld_plan_invalid",
			"error": "step_base64 is required and must be base64",
		})
		return
	}
	ctx, cancel := context.WithTimeout(request.Context(), weldPlanTimeout)
	defer cancel()

	packStarted := time.Now()
	weldplan, weldplanSHA, err := server.packWeldPlan(ctx, body)
	outcome.pack = time.Since(packStarted)
	if err != nil {
		status, code := weldPlanFailure(err)
		outcome.refuse(writer, status, map[string]any{"ok": false, "code": code, "error": err.Error()})
		return
	}

	planStarted := time.Now()
	result, err := server.postMotionPlan(ctx, weldplan, body)
	outcome.plan = time.Since(planStarted)
	if err != nil {
		outcome.refuse(writer, http.StatusBadGateway, map[string]any{
			"ok": false, "code": "motion_plan_failed", "error": err.Error(),
		})
		return
	}

	var verdict planVerdict
	if err := json.Unmarshal(result, &verdict); err != nil {
		outcome.refuse(writer, http.StatusBadGateway, map[string]any{
			"ok": false, "code": "motion_plan_invalid_response", "error": err.Error(),
		})
		return
	}
	if code, message := verdict.failure(); code != "" {
		payload := map[string]any{"ok": false, "code": code, "error": message}
		if refusal, ok := parseDenseRefusal(verdict.DenseError); ok {
			payload["dense_error"] = refusal
		}
		outcome.refuse(writer, http.StatusBadGateway, payload)
		return
	}

	answer := weldPlanResponse{
		Result:         result,
		WeldplanSHA256: weldplanSHA,
		ProgramDigest:  body.ProgramDigest,
		Dense:          verdict.Dense,
		DenseError:     verdict.DenseError,
	}
	digest := denseDigestOf(verdict.Dense)
	if digest != "" {
		fetchStarted := time.Now()
		blob, err := server.fetchDenseBlob(ctx, digest)
		outcome.fetch = time.Since(fetchStarted)
		if err != nil {
			outcome.refuse(writer, http.StatusBadGateway, map[string]any{
				"ok": false, "code": "dense_blob_unreachable",
				"error": "the plan filed a trajectory this could not fetch: " + err.Error(),
			})
			return
		}
		answer.DenseBlobBase64 = base64.StdEncoding.EncodeToString(blob)
	}
	outcome.status, outcome.code = http.StatusOK, "planned "+digest
	writeJSON(writer, http.StatusOK, answer)
}

// packWeldPlan builds the `.weldplan` a planner is handed: program.v2, the
// placement inside it, the planning context, the tooling and the STEP, hashed
// together into deterministic bytes. Shared, because exporting the container
// and planning it use the same contract. Connected taught requests each capture
// a fresh immutable start, so a later export is a new observed question.
//
// The robot goes in as the planning context and nothing else
// (docs/PLANNING-CONTEXT.md): the description read here, the cell's
// deviation judged here, both composed here. The worker packs what it is
// handed and derives the descriptor from the context's own URDF; it is
// given no URDF, no settings and no frames of its own to combine.
func (server *Server) packWeldPlan(
	ctx context.Context, body weldPlanRequest,
) (weldplan []byte, sha256Hex string, err error) {
	step, decodeErr := base64.StdEncoding.DecodeString(body.STEPBase64)
	if decodeErr != nil || (len(step) == 0) != body.WorkpieceAbsent {
		return nil, "", errors.New("step_base64 is required and must be base64")
	}
	// The target, asked once: its identity and machine_planning_calibration.json decide which
	// description this plan is for and how the cell deviates from it.
	target := server.packCellTarget()
	if body.TraceFromHeldTarget {
		body.RequireMeasuredStart = true
	}
	if body.RequireMeasuredStart && body.PlanFromProgramStart {
		return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest, Message: "A plan starts either from the robot's measured pose or from the program's start, not both."}
	}
	if body.RequireMeasuredStart && (target == nil || !target.Available() || !hasMotionNodes(body.Program)) {
		return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest, Message: "Measured-start planning requires a connected native machine and a motion node; no model-reset fallback is permitted."}
	}
	// An output row is scheduled against the connected cell's outputs
	// (weldplan/io_schedule.py), read here from its Describe; with no cell it
	// is named here rather than packed and refused (or, before 2026-10-11 on
	// a weld-only program, dropped) by the planner.
	var ioOutputs map[string]any
	if hasIONodes(body.Program) {
		if target == nil || !target.Available() {
			if err := taughtProgramRefusal(body.Program); err != nil {
				return nil, "", err
			}
		}
		outputs, err := cellIOOutputs(ctx, target, body.Program)
		if err != nil {
			return nil, "", err
		}
		if err := server.applyOperatorWeldAssignment(ctx, outputs); err != nil {
			return nil, "", err
		}
		ioOutputs = outputs
	}

	var cell *denseexec.CellBody
	if target != nil && target.Available() {
		// A selected machine's native envelope must not silently disappear
		// into wider model-only planning if the connection fails.
		reply, err := target.Cell(ctx)
		if err != nil || reply == nil {
			return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest,
				Message: fmt.Sprintf("Cannot read the connected machine's planning limits. Reconnect it before planning: %v", err)}
		}
		if len(reply.ExecutionLimits) == 0 {
			return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest,
				Message: "The connected machine did not provide its native planning limits. Reconnect it before planning."}
		}
		cell = reply
	}
	if cell == nil && strings.TrimSpace(body.MachinePlanningCalibrationBase64) != "" {
		// Offline: the machine_planning_calibration.json the person supplied stands in for the cell's.
		// No identity comes with it, so the description is the workstation's.
		cell = &denseexec.CellBody{Robot: denseexec.RobotCell{ModelID: body.CellID},
			MachinePlanningCalibrationBase64: strings.TrimSpace(body.MachinePlanningCalibrationBase64)}
	}
	description, cellErr := server.descriptionFor(ctx, body.CellID, target, cell)
	if cellErr != nil {
		return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest, Message: cellErr.Error()}
	}
	settings, cellErr := cellSettings(description, cell)
	if cellErr != nil {
		// The caller's cell is wrong, answered as one: a 400, not a worker
		// failure. Nothing was packed and nothing was planned.
		return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest, Message: cellErr.Error()}
	}
	// Which machine this plan is for, when a machine was asked: recorded in
	// the context's identity and carried into the plan's header. It covers
	// the description and the calibration and more besides, so it is a record
	// and never the fence (docs/ROBOT-DESCRIPTION-CONTRACT.md).
	if cell != nil && settings.Identity.MachinePlanningCalibrationSource == robotconfig.SourceTarget {
		settings.Identity.MachineConfigurationSHA256 = cell.Robot.MachineConfigurationSHA256
	}
	// The planning context: the robot this plan is for, compiled once here
	// and packed beside the program (docs/PLANNING-CONTEXT.md). The planner
	// reads it and never a description directory of its own.
	var execution robotconfig.ExecutionLimits
	if cell != nil {
		execution = cell.ExecutionLimits
	}
	compiled, cellErr := robotconfig.Compile(description, settings, execution)
	if cellErr != nil {
		return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest, Message: cellErr.Error()}
	}
	var start *measuredPlanStart
	if cell != nil && target != nil && hasMotionNodes(body.Program) && !body.PlanFromProgramStart {
		start, cellErr = captureMeasuredPlanStart(ctx, target, cell.Robot, body.TraceFromHeldTarget)
		var refusal *planStartRefusal
		if errors.As(cellErr, &refusal) {
			return nil, "", refusal
		}
		if cellErr != nil {
			return nil, "", &seam.WorkerError{Code: seam.ErrorInvalidRequest, Message: cellErr.Error()}
		}
	}
	operation := map[string]any{
		"operation":        string(seam.OperationBuildPlanRequest),
		"step_base64":      body.STEPBase64,
		"step_filename":    body.STEPFilename,
		"workpiece_absent": body.WorkpieceAbsent,
		"program":          body.Program,
		"tooling":          body.Tooling,
		"planning_context": compiled.Wire(),
		"start_state":      start,
		"io_outputs":       ioOutputs,
		"request_id":       body.ProgramID,
		"fixtures":         fixturesForWorker(body.Fixtures),
	}
	packed, runErr := server.seams.ExecuteJSON(ctx, mustJSON(operation))
	if runErr != nil {
		return nil, "", runErr
	}
	var container struct {
		WeldplanBase64 string `json:"weldplan_base64"`
		SHA256         string `json:"sha256"`
	}
	if err := json.Unmarshal(packed.JSON, &container); err != nil {
		return nil, "", fmt.Errorf("the seam worker packed something this cannot read: %w", err)
	}
	bytes, decoded := base64.StdEncoding.DecodeString(container.WeldplanBase64)
	if decoded != nil || len(bytes) == 0 {
		return nil, "", errors.New("the packed plan request is not readable")
	}
	return bytes, container.SHA256, nil
}

// fixturesForWorker hands the operator's fixture list on as JSON, or nothing:
// an absent or null list must reach the seam worker as absent, so its
// placeholder rule ("no list, keep the cell's stand") still applies.
func fixturesForWorker(raw json.RawMessage) any {
	trimmed := strings.TrimSpace(string(raw))
	if trimmed == "" || trimmed == "null" {
		return nil
	}
	return raw
}

// descriptionFor is the description this plan is made against: the one the
// TARGET holds, whenever a target is connected and says which
// (docs/PLANNING-CONTEXT.md).
//
// The workstation's own copy is used when it hashes to what the cell
// advertises. When it does not -- another revision, or no copy at all -- the
// cell's is fetched, verified against the advertised identity and kept by
// that identity, so the plan is made against the robot that will play it and
// `robot_cell_mismatch` cannot happen on this path. A local copy the cell
// does not hold is never planned against silently: that plan would be
// refused at upload, after the planning was paid for.
//
// With no target, or one that does not say, the workstation's copy is what
// there is, and the plan says `machine_planning_calibration_source: "empty"`.
func (server *Server) descriptionFor(
	ctx context.Context, modelID string, target CellTarget, cell *denseexec.CellBody,
) (robotconfig.Description, error) {
	local, localErr := server.robotStore.Load(modelID)
	if cell == nil || !cell.Robot.Valid || cell.Robot.RobotDescriptionSHA256 == "" {
		return local, localErr
	}
	if cell.Robot.ModelID != modelID {
		return robotconfig.Description{}, fmt.Errorf(
			"the target is a %s and this program is for a %s; plans do not cross robots",
			cell.Robot.ModelID, modelID)
	}
	advertised := cell.Robot.RobotDescriptionSHA256
	if localErr == nil && local.Identity == advertised {
		return local, nil
	}
	if cached, ok := robotconfig.Cached(server.robotCache, modelID, advertised); ok {
		return cached, nil
	}
	held := "no copy of it"
	if localErr == nil {
		held = "revision " + local.Identity
	}
	served, err := target.Description(ctx)
	if err != nil {
		return robotconfig.Description{}, fmt.Errorf(
			"the target holds description %s of %s and this workstation has %s; "+
				"the target could not serve its own (%v), so no plan can be made against it",
			advertised, modelID, held, err)
	}
	adopted, err := robotconfig.Adopt(server.robotCache, modelID, advertised, served.Files)
	if err != nil {
		return robotconfig.Description{}, err
	}
	return adopted, nil
}

// cellSettings is what this cell deviates from its model by, resolved here and
// nowhere else: the target's own `machine_planning_calibration.json`, fetched from it at pack time and
// checked against the description before a single number leaves this process.
//
// Fetched rather than taken from the request: the cell is the source of truth
// for its own settings, and a browser that had held them since the tab was
// opened would plan against a calibration that has since been redone. Planning
// without a selected target uses the model's empty document; an unreachable
// selected target is refused by packWeldPlan instead of dropping its limits.
func cellSettings(description robotconfig.Description, cell *denseexec.CellBody) (robotconfig.Settings, error) {
	if cell == nil || (!cell.Robot.Configured && cell.MachinePlanningCalibrationBase64 == "") {
		return robotconfig.Resolve(description, nil, robotconfig.SourceEmpty)
	}
	if cell.Robot.ModelID != "" && cell.Robot.ModelID != description.ModelID {
		return robotconfig.Settings{}, fmt.Errorf(
			"the target is a %s and this program is for a %s; plans do not cross robots",
			cell.Robot.ModelID, description.ModelID)
	}
	bytes, decodeErr := base64.StdEncoding.DecodeString(cell.MachinePlanningCalibrationBase64)
	if decodeErr != nil {
		return robotconfig.Settings{}, fmt.Errorf("the target's machine_planning_calibration.json is not base64: %w", decodeErr)
	}
	return robotconfig.Resolve(description, bytes, robotconfig.SourceTarget)
}

// packCellTarget is the target packing asks, nil when there is none: a test's
// stand-in, else the execution backend when it can say what robot it is
// (the rt-core backend does; docs/PLANNING-CONTEXT.md).
func (server *Server) packCellTarget() CellTarget {
	if server.cellTarget != nil {
		return server.cellTarget
	}
	if target, ok := server.denseExecution.(CellTarget); ok && target != nil {
		return target
	}
	return nil
}

func (server *Server) postMotionPlan(
	ctx context.Context, weldplan []byte, body weldPlanRequest,
) (json.RawMessage, error) {
	query := "run_weld_trajopt=true&run_connecting_trajopt=true&verify=true&dense=true"
	backend := strings.TrimSpace(body.FreeSpaceBackend)
	if backend == "" {
		backend = "bspline"
	}
	query += "&free_space_backend=" + backend
	if body.ProgramID != "" {
		query += fmt.Sprintf("&program_id=%s&manifest_revision=%d&plan_revision=%d",
			body.ProgramID, max(body.ManifestRevision, 1), max(body.PlanRevision, 1))
	}
	endpoint := strings.TrimSuffix(server.motionOrigin, "/") + "/api/motion/plan?" + query

	post, err := http.NewRequestWithContext(ctx, http.MethodPost, endpoint, bytes.NewReader(weldplan))
	if err != nil {
		return nil, err
	}
	post.Header.Set("Content-Type", "application/octet-stream")
	response, err := http.DefaultClient.Do(post)
	if err != nil {
		return nil, fmt.Errorf("the motion server is unreachable at %s (%w)", server.motionOrigin, err)
	}
	defer response.Body.Close()
	payload, err := io.ReadAll(io.LimitReader(response.Body, weldPlanMaxResponse))
	if err != nil {
		return nil, err
	}
	if response.StatusCode != http.StatusOK {
		detail := strings.TrimSpace(string(payload))
		if len(detail) > 600 {
			detail = detail[:600]
		}
		return nil, fmt.Errorf("the motion server refused the plan request: %s", detail)
	}
	return payload, nil
}

func (server *Server) fetchDenseBlob(ctx context.Context, digest string) ([]byte, error) {
	endpoint := strings.TrimSuffix(server.motionOrigin, "/") + "/api/motion/dense/" + digest
	get, err := http.NewRequestWithContext(ctx, http.MethodGet, endpoint, nil)
	if err != nil {
		return nil, err
	}
	response, err := http.DefaultClient.Do(get)
	if err != nil {
		return nil, err
	}
	defer response.Body.Close()
	if response.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("%s answered %d", endpoint, response.StatusCode)
	}
	return io.ReadAll(io.LimitReader(response.Body, weldPlanMaxResponse))
}

// planSeamVerdict is one seam's outcome inside a motion plan result. An
// `error` is the planner declining to plan that weld, with the reason.
// planVerdict is the part of a motion result that says whether it planned.
type planVerdict struct {
	ConnectingError json.RawMessage   `json:"connecting_trajectories_error"`
	Dense           json.RawMessage   `json:"dense"`
	DenseError      json.RawMessage   `json:"dense_error"`
	Seams           []planSeamVerdict `json:"seams"`
}

// failure is why a 200 from the motion server is still not a plan, as a code
// and a sentence, or empty codes when it planned. The motion server answers 200
// for a well-formed request whatever became of it, so a caller reading only the
// status installs a plan with nothing in it -- "I press Plan and nothing
// happens".
//
// MOST SPECIFIC CAUSE FIRST. A weld that did not plan breaks everything after
// it too: its transits have no end to start from, so the join fails and there
// is no dense trajectory. Checked in the other order, the operator was told
// "the welds planned but nothing connects them" about a program where two of
// four welds had no path at all -- which sends them looking at transits.
func (verdict planVerdict) failure() (string, string) {
	// A seam the planner refused, or ran and found no path through, is not a
	// planned seam -- and the second carries no `error`, because nothing went
	// wrong: the arm cannot do that weld as authored.
	if refused := refusedSeams(verdict.Seams); len(refused) > 0 {
		return "motion_plan_seam_refused", fmt.Sprintf("%d of %d weld(s) could not be planned: %s",
			len(refused), len(verdict.Seams), strings.Join(refused, "; "))
	}
	// This stage also plans ordinary taught moves with no weld seams. Preserve
	// its exact failed-leg evidence without claiming that nonexistent welds
	// succeeded, or displaying JSON quotes around a plain planner sentence.
	if len(verdict.ConnectingError) > 0 && string(verdict.ConnectingError) != "null" {
		detail := strings.TrimSpace(string(verdict.ConnectingError))
		var sentence string
		if json.Unmarshal(verdict.ConnectingError, &sentence) == nil && strings.TrimSpace(sentence) != "" {
			detail = sentence
		}
		if len(verdict.Seams) == 0 {
			return "motion_plan_unjoined", "Program motion could not be planned: " + detail
		}
		return "motion_plan_unjoined", "The welds planned, but connecting motion could not be planned: " + detail
	}
	// Dense was asked for, so a plan without it planned nothing that can be
	// run or simulated, and `dense_error` is the field that says why. A
	// glued-on JSON object is not a sentence: render `detail · reason`,
	// and the segment when the refusal names one.
	if len(verdict.Dense) == 0 || string(verdict.Dense) == "null" {
		if refusal, ok := parseDenseRefusal(verdict.DenseError); ok {
			return "motion_plan_no_trajectory", denseRefusalSentence(refusal)
		}
		detail := strings.TrimSpace(string(verdict.DenseError))
		if detail == "" || detail == "null" {
			return "motion_plan_no_trajectory",
				"the motion server returned no trajectory and no reason"
		}
		return "motion_plan_no_trajectory", "the plan produced no trajectory: " + detail
	}
	return "", ""
}

// denseRefusal is `{reason, detail, segment_index}` from the motion server.
type denseRefusal struct {
	Reason       string `json:"reason"`
	Detail       string `json:"detail"`
	SegmentIndex *int   `json:"segment_index"`
}

func parseDenseRefusal(raw json.RawMessage) (denseRefusal, bool) {
	trimmed := strings.TrimSpace(string(raw))
	if trimmed == "" || trimmed == "null" {
		return denseRefusal{}, false
	}
	var refusal denseRefusal
	if err := json.Unmarshal(raw, &refusal); err != nil {
		return denseRefusal{}, false
	}
	if strings.TrimSpace(refusal.Reason) == "" {
		return denseRefusal{}, false
	}
	return refusal, true
}

func denseRefusalSentence(refusal denseRefusal) string {
	var parts []string
	if detail := strings.TrimSpace(refusal.Detail); detail != "" {
		parts = append(parts, detail)
	}
	if reason := strings.TrimSpace(refusal.Reason); reason != "" {
		parts = append(parts, reason)
	}
	if refusal.SegmentIndex != nil {
		parts = append(parts, fmt.Sprintf("segment %d", *refusal.SegmentIndex))
	}
	return strings.Join(parts, " · ")
}

type planSeamVerdict struct {
	ID    string `json:"id"`
	Error string `json:"error"`
	// Crossed is false for a seam the search ran on and found no path
	// through. Nil from a result that does not carry it.
	Crossed        *bool          `json:"crossed"`
	FeasibleRate   float64        `json:"feasible_rate"`
	FrontierCauses map[string]int `json:"frontier_causes"`
}

// refusedSeams names the welds the planner would not plan, each with the
// reason it gave. Named rather than counted: "1 of 3 refused" sends an
// operator looking, and the reason usually says what is missing.
func refusedSeams(seams []planSeamVerdict) []string {
	var refused []string
	for _, seam := range seams {
		switch {
		case strings.TrimSpace(seam.Error) != "":
			refused = append(refused, fmt.Sprintf("%s: %s", seam.ID, seam.Error))
		case seam.Crossed != nil && !*seam.Crossed:
			refused = append(refused, fmt.Sprintf("%s: no path the arm can follow (%s)", seam.ID, blockedBy(seam)))
		}
	}
	return refused
}

// blockedBy says how close a seam came and what stopped it, largest cause
// first: "1.65% of poses feasible, joint limit 43593, no ik 41668".
func blockedBy(seam planSeamVerdict) string {
	type cause struct {
		name  string
		count int
	}
	var causes []cause
	for name, count := range seam.FrontierCauses {
		if count <= 0 {
			continue
		}
		at := len(causes)
		for at > 0 && (causes[at-1].count < count || (causes[at-1].count == count && causes[at-1].name > name)) {
			at--
		}
		causes = append(causes, cause{})
		copy(causes[at+1:], causes[at:])
		causes[at] = cause{name, count}
	}
	parts := []string{fmt.Sprintf("%.2f%% of poses feasible", seam.FeasibleRate*100)}
	for _, entry := range causes {
		parts = append(parts, fmt.Sprintf("%s %d", entry.name, entry.count))
	}
	return strings.Join(parts, ", ")
}

func denseDigestOf(dense json.RawMessage) string {
	if len(dense) == 0 {
		return ""
	}
	var summary struct {
		TrajectoryDigest string `json:"trajectory_digest"`
	}
	if err := json.Unmarshal(dense, &summary); err != nil {
		return ""
	}
	return strings.TrimSpace(summary.TrajectoryDigest)
}

func mustJSON(value any) []byte {
	encoded, err := json.Marshal(value)
	if err != nil {
		// Every caller passes a map of strings and already-valid JSON.
		panic(err)
	}
	return encoded
}
