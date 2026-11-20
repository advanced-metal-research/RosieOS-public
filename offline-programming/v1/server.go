package main

import (
	"compress/gzip"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/cad"
	"rosieos/offline-programming/v1/internal/catalog"
	"rosieos/offline-programming/v1/internal/connected"
	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/execution"
	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/programstore"
	"rosieos/offline-programming/v1/internal/robotconfig"
	"rosieos/offline-programming/v1/internal/seam"
	"rosieos/offline-programming/v1/internal/simulator"
	"rosieos/offline-programming/v1/internal/targetplanning"
	"rosieos/rt-core/cartesian"
)

const (
	uiBasePath           = "/offline-programming/v1/ui/"
	maxCADRequestBytes   = 12 << 20
	maxPlanRequestBytes  = 8 << 20
	connectedTargetsPath = "/api/offline-programming/v1/connected-targets"
	executionGatewayPath = "/api/offline-programming/v1/execution-gateway"
	executionPath        = "/api/offline-programming/v1/execution"
	programsPath         = "/api/offline-programming/v1/programs"
	denseExecutionPath   = "/api/offline-programming/v1/dense-execution"
)

type CADWorker interface {
	ExecuteJSON(context.Context, []byte) (cad.Result, error)
}

// SeamWorker is the rails half of the CAD surface: joints, the seams they
// default to, and torch clearance along one. Its sibling `CADWorker` serves
// the classic flow. Both are one bounded subprocess per request.
type SeamWorker interface {
	ExecuteJSON(context.Context, []byte) (seam.Result, error)
}

type ServerOptions struct {
	DistDir          string
	Worker           CADWorker
	Seams            SeamWorker
	MotionOrigin     string
	RepoRoot         string
	LocalSimulator   *simulator.Service
	JogManager       *simulator.JogManager
	ConnectedTargets *ConnectedRegistry
	ExecutionGateway *execution.Gateway
	ProgramCatalog   catalog.Gateway
	TargetPlanning   *targetplanning.Service
	DenseExecution   denseexec.Backend
	// RobotDescriptionRoot is where the robot descriptions live. Empty means
	// <RepoRoot>/robot_description, which is what a workstation has; a
	// deployment that ships them elsewhere says so here rather than in the
	// package that reads them.
	RobotDescriptionRoot string
	// RobotDescriptionCache is where descriptions fetched from a cell are
	// kept, by identity. Empty means <user cache>/rosieos-olp/robot-descriptions.
	RobotDescriptionCache string
	// CellTarget is what packing asks for the target's own machine_planning_calibration.json.
	// Unset, that is the active dense-execution target; a test stands its
	// own in, because reaching the real one opens NATS.
	CellTarget CellTarget
	// Maintenance is the update notice (maintenance.go). Nil: never active.
	Maintenance *MaintenanceSource
	// CellMaintenance reads the selected cell's notice from its rt-control
	// (cell_maintenance.go). Nil: follow a cell picker's selection, else none.
	CellMaintenance *CellMaintenance
	// Discovery advertises this server and lists the OLP servers it hears
	// (discovered_cells.go). Nil: the routes answer that search is off.
	Discovery *cellDiscovery
	// TestPrograms holds uploaded test programs (dense_test_program.go).
	// Nil: test programs are off and their route refuses.
	TestPrograms *denseexec.TestPrograms
	// CellCamera relays the camera service on the cell host (cell_camera.go).
	// Nil: the cell names no camera.
	CellCamera *CellCamera
	// CellExtraCameras relays further cameras of the same cell, listed
	// beside CellCamera (cell_camera.go). Empty: the cell camera is the
	// only one.
	CellExtraCameras []NamedCellCamera
	// CellVideo relays WebRTC set-up to the camera's video sender (cell_video.go). Nil: no video.
	CellVideo *CellVideo
	// ProgramStore is the cell's shared program store (cell_programs.go).
	// Nil: its routes answer program_store_disabled and programs stay on
	// each client. StoreLabel names the host to clients (the relay's host name);
	// empty means this host's name.
	ProgramStore *programstore.Store
	StoreLabel   string
}

// CellTarget is the cell a plan is being packed for, as pack time needs it:
// whether it can be reached and what it says about itself
// (docs/ROBOT-DESCRIPTION-CONTRACT.md).
type CellTarget interface {
	Available() bool
	Cell(ctx context.Context) (*denseexec.CellBody, error)
	// Description is the robot description the target holds, fetched when
	// this workstation's copy is not the one the cell advertises
	// (docs/PLANNING-CONTEXT.md).
	Description(ctx context.Context) (*denseexec.DescriptionBody, error)
}

type Server struct {
	distDir          string
	worker           CADWorker
	seams            SeamWorker
	motionOrigin     string
	repoRoot         string
	robotStore       robotconfig.Store
	robotCache       string
	localSimulator   *simulator.Service
	jogManager       *simulator.JogManager
	connectedTargets *ConnectedRegistry
	executionGateway *execution.Gateway
	programCatalog   catalog.Gateway
	targetPlanning   *targetplanning.Service
	denseExecution   denseexec.Backend
	denseCells       *denseexec.Fleet
	cellTarget       CellTarget
	maintenance      *MaintenanceSource
	cellMaintenance  *CellMaintenance
	clients          *clientRegistry
	// traffic counts requests and bytes per client and route (traffic.go).
	traffic          *trafficMeter
	control          *controlState
	discovery        *cellDiscovery
	testPrograms     *denseexec.TestPrograms
	cellCamera       *CellCamera
	cellExtraCameras []NamedCellCamera
	cellVideo        *CellVideo
	programs         *cellPrograms
}

func NewServer(options ServerOptions) *Server {
	if controller, ok := options.DenseExecution.(interface {
		SetCartesianResolverFactory(denseexec.CartesianResolverFactory)
	}); ok && options.RepoRoot != "" {
		controller.SetCartesianResolverFactory(cartesianResolver(options.RepoRoot))
	}
	if controller, ok := options.DenseExecution.(interface {
		SetRobotGeometrySource(func(string) (denseexec.RobotGeometry, error))
	}); ok {
		store := robotconfig.At(robotDescriptionRoot(options))
		controller.SetRobotGeometrySource(func(model string) (denseexec.RobotGeometry, error) {
			description, err := store.Load(model)
			if err != nil {
				return denseexec.RobotGeometry{}, err
			}
			geometry, err := denseexec.RobotGeometryFromURDF(string(description.URDFRaw))
			if err != nil {
				return geometry, err
			}
			// The existing model loader uses the description's arm joint order
			// and TCP frame. Unsupported geometry keeps the monitoring readout;
			// a speed-bounded return explicitly refuses without kinematics.
			geometry.Kinematics, _ = cartesian.LoadKinematics(description.Dir)
			return geometry, nil
		})
	}
	gateway := options.ExecutionGateway
	if gateway == nil {
		gateway = execution.NewGateway(execution.GatewayConfig{})
	}
	cellMaintenance := options.CellMaintenance
	if cellMaintenance == nil {
		cellMaintenance = cellMaintenanceOf(options.DenseExecution, nil)
	}
	targetPlanning := options.TargetPlanning
	if targetPlanning == nil {
		targetPlanning = targetplanning.NewService(targetplanning.Options{
			Targets: options.ConnectedTargets,
		})
	}
	return &Server{
		distDir:          options.DistDir,
		worker:           options.Worker,
		seams:            options.Seams,
		motionOrigin:     options.MotionOrigin,
		repoRoot:         options.RepoRoot,
		robotStore:       robotconfig.At(robotDescriptionRoot(options)),
		robotCache:       robotDescriptionCache(options),
		localSimulator:   options.LocalSimulator,
		jogManager:       options.JogManager,
		connectedTargets: options.ConnectedTargets,
		executionGateway: gateway,
		programCatalog:   options.ProgramCatalog,
		targetPlanning:   targetPlanning,
		denseExecution:   options.DenseExecution,
		denseCells:       denseexec.NewFleet(options.DenseExecution),
		cellTarget:       options.CellTarget,
		maintenance:      options.Maintenance,
		cellMaintenance:  cellMaintenance,
		clients:          newClientRegistry(),
		traffic:          newTrafficMeter(time.Now()),
		control:          newControlState(),
		discovery:        options.Discovery,
		testPrograms:     options.TestPrograms,
		cellCamera:       options.CellCamera,
		cellExtraCameras: options.CellExtraCameras,
		cellVideo:        options.CellVideo,
		programs:         newCellPrograms(options.ProgramStore, options.StoreLabel, options.DenseExecution),
	}
}

func (server *Server) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("GET "+denseExecutionPath+"/cells", server.denseExecutionCells)
	mux.HandleFunc("POST "+denseExecutionPath+"/cells", server.denseExecutionAddCell)
	mux.HandleFunc("DELETE "+denseExecutionPath+"/cells/{id}", server.denseExecutionRemoveCell)
	mux.HandleFunc("POST "+denseExecutionPath+"/target", server.denseExecutionSelectTarget)
	mux.HandleFunc("GET /api/offline-programming/v1/health", server.health)
	mux.HandleFunc("GET "+trafficAPIPath, server.trafficCounts)
	mux.HandleFunc("GET /api/offline-programming/v1/discovered-cells", server.discoveredCells)
	mux.HandleFunc("GET "+cellCameraAPIPath, server.cellCameraStatus)
	mux.HandleFunc("GET "+cellCameraAPIPath+"/frame.jpg", server.cellCameraFrame)
	mux.HandleFunc("GET "+cellCameraAPIPath+"/depth.z16", server.cellCameraDepth)
	mux.HandleFunc("GET "+cellCamerasAPIPath, server.cellCameraList)
	mux.HandleFunc("GET "+cellVideoAPIPath, server.cellVideoStatus)
	mux.HandleFunc("POST "+cellVideoAPIPath+"/whep", server.cellVideoOffer)
	mux.HandleFunc("DELETE "+cellVideoAPIPath+"/whep/{id}", server.cellVideoEnd)
	mux.HandleFunc("GET "+cellVideoStreamPath, server.cellVideoStream)
	mux.HandleFunc("POST "+cellVideoKeyframePath, server.cellVideoKeyframe)
	mux.HandleFunc("GET "+cellCamerasAPIPath+"/{id}", server.cellCameraStatusByID)
	mux.HandleFunc("GET "+cellCamerasAPIPath+"/{id}/frame.jpg", server.cellCameraFrameByID)
	mux.HandleFunc("GET "+cellCamerasAPIPath+"/{id}/depth.z16", server.cellCameraDepthByID)
	// The wrist camera route (2026-10-18), kept for pendants built
	// before the camera list: it is the list's "wrist" camera.
	mux.HandleFunc("GET "+wristCameraAPIPath, server.wristCameraStatus)
	mux.HandleFunc("GET "+wristCameraAPIPath+"/frame.jpg", server.wristCameraFrame)
	mux.HandleFunc("GET "+maintenanceAPIPath, server.maintenanceNotice)
	mux.HandleFunc("GET "+controlAPIPath, server.controlRoute)
	mux.HandleFunc("POST "+controlAPIPath+"/take", server.controlTake)
	mux.HandleFunc("GET /api/offline-programming/v1/capabilities", server.capabilities)
	mux.HandleFunc("POST /api/offline-programming/v1/cadquery/topology", server.cadQuery)
	mux.HandleFunc("GET "+RobotsAPIPath, server.robots)
	mux.HandleFunc("POST "+RobotsAPIPath+"/capture-model", server.captureModel)
	mux.HandleFunc("POST "+SeamAPIPath, server.seamWorker)
	mux.HandleFunc("POST "+WeldPlanAPIPath, server.weldPlan)
	mux.HandleFunc("POST "+WeldPlanExportAPIPath, server.weldPlanExport)
	mux.HandleFunc("GET "+simulator.HTTPStatusPath, server.localSimulatorStatus)
	mux.HandleFunc("POST "+simulator.HTTPStartPath, server.localSimulatorStart)
	mux.HandleFunc("POST "+simulator.HTTPStopPath, server.localSimulatorStop)
	mux.HandleFunc("POST "+simulator.HTTPPlansPath, server.localSimulatorPlan)
	mux.HandleFunc("POST "+simulator.HTTPJogStartPath, server.localSimulatorJogStart)
	mux.HandleFunc("POST "+simulator.HTTPJogStopPath, server.localSimulatorJogStop)
	mux.HandleFunc("POST "+simulator.HTTPJogIntentPath, server.localSimulatorJogIntent)
	mux.HandleFunc("GET "+simulator.HTTPJogStatePath, server.localSimulatorJogState)
	mux.HandleFunc("POST "+simulator.HTTPJogHomePath, server.localSimulatorJogHome)
	mux.HandleFunc("POST "+simulator.HTTPJogHaltPath, server.localSimulatorJogHalt)
	mux.HandleFunc("POST "+simulator.HTTPJogSetZeroPath, server.localSimulatorJogSetZero)
	mux.HandleFunc("POST "+simulator.HTTPJogStepPath, server.localSimulatorJogStep)
	mux.HandleFunc("GET "+connectedTargetsPath, server.connectedDiscovery)
	mux.HandleFunc("GET "+connectedTargetsPath+"/{profile}/status", server.connectedStatus)
	mux.HandleFunc("GET "+connectedTargetsPath+"/{profile}/capabilities", server.connectedCapabilities)
	mux.HandleFunc("GET "+executionGatewayPath+"/capabilities", server.executionGatewayCapabilities)
	mux.HandleFunc("POST "+executionGatewayPath+"/review", server.executionGatewayReview)
	mux.HandleFunc("POST "+executionPath+"/review", server.executionGatewayReview)
	mux.HandleFunc("POST "+executionPath+"/start", server.executionStart)
	mux.HandleFunc("POST "+executionPath+"/{execution_id}/stop", server.executionStop)
	mux.HandleFunc("GET "+executionPath+"/{execution_id}", server.executionSession)
	mux.HandleFunc("GET "+programsPath, server.programCatalogList)
	mux.HandleFunc("POST "+programsPath, server.programCatalogCreate)
	mux.HandleFunc("GET "+programsPath+"/{program_id}", server.programCatalogGet)
	mux.HandleFunc("PUT "+programsPath+"/{program_id}", server.programCatalogUpdate)
	mux.HandleFunc("GET "+programsPath+"/{program_id}/revisions", server.programCatalogRevisions)
	server.registerCellPrograms(mux)
	mux.HandleFunc("POST /api/offline-programming/v1/targets/{profile}/plans", server.targetPlan)
	mux.HandleFunc("GET /api/offline-programming/v1/plans/{plan_id}", server.acceptedPlan)
	mux.HandleFunc("GET "+denseExecutionPath+"/capabilities", server.denseExecutionCapabilities)
	mux.HandleFunc("GET /api/offline-programming/v1/targets", server.cellTargets)
	mux.HandleFunc("POST /api/offline-programming/v1/targets/select", server.selectCellTarget)
	mux.HandleFunc("GET "+denseExecutionPath+"/status", server.denseExecutionStatus)
	mux.HandleFunc("POST "+denseExecutionPath+"/io/test", server.denseManualIO)
	mux.HandleFunc("POST "+denseExecutionPath+"/io/test/start", server.denseBeginManualIO)
	mux.HandleFunc("GET /api/offline-programming/v1/io-settings", server.ioSettingsRoute)
	mux.HandleFunc("POST /api/offline-programming/v1/io-settings", server.ioSettingsRoute)
	mux.HandleFunc("GET "+denseExecutionPath+"/telemetry/window", server.denseExecutionTelemetryWindow)
	mux.HandleFunc("GET "+denseExecutionPath+"/drive-health", server.denseExecutionDriveHealth)
	mux.HandleFunc("GET "+denseExecutionPath+"/cell", server.denseExecutionCell)
	mux.HandleFunc("POST "+denseExecutionPath+"/load", server.denseExecutionLoad)
	mux.HandleFunc("GET "+denseExecutionPath+"/test-program", server.denseTestPrograms)
	mux.HandleFunc("POST "+denseExecutionPath+"/test-program", server.denseTestProgramUpload)
	mux.HandleFunc("POST "+denseExecutionPath+"/play", server.denseExecutionPlay)
	mux.HandleFunc("POST "+denseExecutionPath+"/pause", server.denseExecutionPause)
	mux.HandleFunc("POST "+denseExecutionPath+"/stop", server.denseExecutionStop)
	mux.HandleFunc("POST "+denseExecutionPath+"/arm", server.denseExecutionArm)
	mux.HandleFunc("POST "+denseExecutionPath+"/reset-fault", server.denseExecutionResetFault)
	mux.HandleFunc("POST "+denseExecutionPath+"/go-home", server.denseExecutionGoHome)
	mux.HandleFunc("POST "+denseExecutionPath+"/home", server.denseExecutionHome)
	mux.HandleFunc("POST "+denseExecutionPath+"/heartbeat", server.denseExecutionHeartbeat)
	mux.HandleFunc("GET "+denseExecutionPath+"/jog/state", server.denseJointJog)
	mux.HandleFunc("POST "+denseExecutionPath+"/jog/{action}", server.denseJointJog)
	mux.HandleFunc("POST "+denseExecutionPath+"/move", server.denseJointMove)
	mux.HandleFunc("POST "+denseExecutionPath+"/recover-to-zero", server.denseExecutionRecoverToZero)
	mux.HandleFunc("POST "+denseExecutionPath+"/run-program", server.denseExecutionRunProgram)
	mux.HandleFunc("POST "+denseExecutionPath+"/run-program/finish", server.denseExecutionFinishLoop)
	mux.HandleFunc("POST "+denseExecutionPath+"/cartesian/move", server.denseCartesianMove)
	mux.HandleFunc("GET "+denseExecutionPath+"/trace/state", server.denseTrace)
	mux.HandleFunc("POST "+denseExecutionPath+"/trace/{action}", server.denseTrace)
	mux.HandleFunc("GET "+denseExecutionPath+"/cartesian/state", server.denseCartesianJog)
	mux.HandleFunc("POST "+denseExecutionPath+"/cartesian/{action}", server.denseCartesianJog)
	mux.HandleFunc("POST /api/offline-programming/v1/connect", disabledCapability("connected targets are read-only; active connect is disabled"))
	mux.HandleFunc("POST /api/offline-programming/v1/execute", disabledCapability("legacy execute endpoint is retired; only reviewed execution sessions are supported"))
	mux.HandleFunc("POST /api/offline-programming/v1/teleop", disabledCapability("tele-op is reserved and disabled"))
	mux.HandleFunc("GET /{$}", func(writer http.ResponseWriter, request *http.Request) {
		http.Redirect(writer, request, uiBasePath, http.StatusTemporaryRedirect)
	})
	if strings.TrimSpace(server.distDir) != "" {
		files := http.FileServer(http.Dir(server.distDir))
		mux.Handle("GET "+uiBasePath, http.StripPrefix(uiBasePath, noDirectoryListing(files, server.distDir)))
	}
	return server.traffic.meter(securityHeaders(gzipResponses(server.trackClients(server.maintenanceGate(server.controlGate(server.denseTargetRequests(mux)))))))
}

// denseJointJog requires explicit target/session fencing and keeps unsupported
// backends closed, preventing real-target intents from reaching a simulator pad.
func (server *Server) denseJointJog(writer http.ResponseWriter, request *http.Request) {
	controller, ok := server.denseExecution.(denseexec.JointJogBackend)
	if !ok {
		writeDenseError(writer, http.StatusConflict, "dense_target_unavailable", "Select an rt-core target and ARM before joint jog")
		return
	}
	if request.Method == http.MethodGet {
		writeJSON(writer, http.StatusOK, controller.JointJogState())
		return
	}
	var input denseexec.JointJogIntent
	if err := decodeBoundedJSON(writer, request, maxDenseRequestBytes, &input); err != nil {
		writeDenseError(writer, http.StatusBadRequest, "jog_invalid_vector", err.Error())
		return
	}
	state, err := controller.JointJog(request.Context(), request.PathValue("action"), input)
	if err != nil {
		writeDenseFailure(writer, "jog", err)
		return
	}
	writeJSON(writer, http.StatusOK, state)
}

// denseJointMove returns done only after correlated native completion. Request
// cancellation reaches the controller's Stop path, preventing orphaned moves.
func (server *Server) denseJointMove(writer http.ResponseWriter, request *http.Request) {
	controller, ok := server.denseExecution.(denseexec.JointMoveBackend)
	if !ok {
		writeDenseError(writer, http.StatusConflict, "dense_target_unavailable", "Select an rt-core target and ARM before moving a joint")
		return
	}
	var input denseexec.JointMoveIntent
	if err := decodeBoundedJSON(writer, request, maxDenseRequestBytes, &input); err != nil {
		writeDenseError(writer, http.StatusBadRequest, "joint_move_invalid", err.Error())
		return
	}
	state, err := controller.JointMove(request.Context(), input)
	if err != nil {
		writeDenseFailure(writer, "move", err)
		return
	}
	writeJSON(writer, http.StatusOK, state)
}

func (server *Server) executionGatewayCapabilities(writer http.ResponseWriter, _ *http.Request) {
	writeJSON(writer, http.StatusOK, server.executionGateway.Capabilities())
}

func (server *Server) executionGatewayReview(
	writer http.ResponseWriter,
	request *http.Request,
) {
	var input execution.ExecutionReviewInput
	if err := decodeBoundedJSON(writer, request, maxPlanRequestBytes, &input); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "execution_review_invalid",
			"error": "execution review request is invalid",
		})
		return
	}
	if server.executionGateway.Capabilities().InfrastructureReady {
		if err := server.bindAuthoritativeExecutionInput(&input); err != nil {
			writeJSON(writer, http.StatusConflict, map[string]any{
				"ok": false, "code": "authoritative_execution_evidence_unavailable",
				"error": err.Error(),
			})
			return
		}
	}
	review, err := server.executionGateway.Review(request.Context(), input)
	if err != nil {
		status := http.StatusConflict
		if errors.Is(err, execution.ErrUnavailable) {
			status = http.StatusServiceUnavailable
		}
		writeJSON(writer, status, review)
		return
	}
	writeJSON(writer, http.StatusOK, review)
}

func (server *Server) executionStart(writer http.ResponseWriter, request *http.Request) {
	var start execution.StartRequest
	if err := decodeBoundedJSON(writer, request, maxPlanRequestBytes, &start); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "execution_start_invalid",
			"error": "execution start request is invalid",
		})
		return
	}
	if err := server.bindAuthoritativeExecutionInput(&start.Review); err != nil {
		writeJSON(writer, http.StatusConflict, map[string]any{
			"ok": false, "code": "authoritative_execution_evidence_unavailable",
			"error": err.Error(),
		})
		return
	}
	session, err := server.executionGateway.Start(request.Context(), start)
	if err != nil {
		status, code := executionHTTPError(err)
		writeJSON(writer, status, map[string]any{
			"ok": false, "code": code, "error": err.Error(),
		})
		return
	}
	writeJSON(writer, http.StatusOK, session)
}

func (server *Server) bindAuthoritativeExecutionInput(
	input *execution.ExecutionReviewInput,
) error {
	if server == nil || input == nil ||
		server.targetPlanning == nil ||
		server.connectedTargets == nil {
		return errors.New(
			"server-owned target plan and observation authority is unavailable",
		)
	}
	planID := strings.TrimSpace(input.Identity.PlanID)
	if planID == "" {
		planID = strings.TrimSpace(input.AcceptedPlan.PlanID)
	}
	profileID := strings.TrimSpace(input.Identity.Target.ProfileID)
	if planID == "" || profileID == "" {
		return errors.New("selected target and accepted plan identity are required")
	}
	stored, ok := server.targetPlanning.GetStored(planID)
	if !ok ||
		stored.Plan.Target.TargetID != profileID ||
		stored.Request.Target.ProfileID != profileID {
		return errors.New(
			"selected plan is not an exact server-stored target plan",
		)
	}
	target, ok := server.connectedTargets.PlanningTarget(profileID)
	if !ok ||
		target.Binding.ManifestRevision !=
			stored.Plan.Target.ManifestRevision {
		return errors.New("selected target manifest revision changed")
	}
	input.Identity = execution.ExecutionIdentity{
		ProjectID:        stored.Request.ProjectID,
		ProjectRevision:  stored.Request.ProjectRevision,
		ProgramID:        stored.Plan.ProgramID,
		ProgramRevision:  stored.Plan.ProgramRevision,
		PlanID:           stored.Plan.PlanID,
		PlanRevision:     stored.Plan.PlanRevision,
		ProgramDigest:    stored.Plan.ProgramDigest,
		TrajectoryDigest: stored.Plan.TrajectoryDigest,
		RobotModel:       stored.Plan.Target.RobotModel,
		Target: execution.TargetRevisionIdentity{
			ProfileID:        target.Discovery.ProfileID,
			ManifestRef:      target.Discovery.ManifestRef,
			CellID:           target.Discovery.CellID,
			ManifestRevision: target.Binding.ManifestRevision,
		},
	}
	input.Discovery = target.Discovery
	input.PlanRequest = stored.Request
	input.AcceptedPlan = stored.Plan
	if err := server.connectedTargets.BindLatestExecutionEvidence(input); err != nil {
		return err
	}
	runtime := input.RuntimePlan
	if runtime.Identity.PlanID != stored.Plan.PlanID ||
		runtime.Identity.PlanRevision != stored.Plan.PlanRevision ||
		runtime.Identity.ProgramID != stored.Plan.ProgramID ||
		runtime.Identity.ProgramRevision != stored.Plan.ProgramRevision ||
		runtime.Identity.RobotModel != string(stored.Plan.Target.RobotModel) {
		return errors.New(
			"target runtime plan differs from the server-stored accepted plan",
		)
	}
	input.Identity.ProgramHash = runtime.Identity.ProgramHash
	input.Evidence = planning.NormalizedPlanEvidence{
		Schema:          planning.NormalizedPlanEvidenceSchema,
		RequestID:       stored.Request.RequestID,
		GeneratedAt:     stored.Plan.CreatedAt,
		PlannerOwner:    runtime.PlannerOwner,
		PlannerBackend:  runtime.PlannerBackend,
		State:           planning.PlanServerValidated,
		Accepted:        runtime.Accepted.Known && runtime.Accepted.Value,
		ValidationState: runtime.ValidationState,
		SampleCount:     len(stored.Plan.Trajectory),
		Identity: planning.PlanIdentity{
			ProjectID:       stored.Request.ProjectID,
			ProjectRevision: stored.Request.ProjectRevision,
			ProgramID:       stored.Plan.ProgramID,
			ProgramRevision: stored.Plan.ProgramRevision,
			PlanID:          stored.Plan.PlanID,
			PlanRevision:    stored.Plan.PlanRevision,
			ProgramHash:     runtime.Identity.ProgramHash,
			RobotModel:      stored.Plan.Target.RobotModel,
			Target:          stored.Request.Target,
		},
		Collision: planning.CollisionEvidence{
			State:      executionCheckState(runtime.Safety.Collision),
			CheckState: runtime.Safety.Collision.State,
			Points:     []planning.CollisionPoint{},
		},
		Singularity: planning.SingularityEvidence{
			State:      executionCheckState(runtime.Safety.Singularity),
			CheckState: runtime.Safety.Singularity.State,
			Points:     []planning.SingularityPoint{},
		},
		Limits: planning.LimitEvidence{
			State:      executionCheckState(runtime.Safety.Limits),
			CheckState: runtime.Safety.Limits.State,
			Points:     []planning.UnreachablePoint{},
		},
		Runtime: planning.RuntimeEvidence{
			State: executionCheckState(runtime.Safety.Runtime),
			Playable: runtime.Safety.Runtime.Available &&
				runtime.Safety.Runtime.Clear,
		},
		CorrelationReasons: append(
			[]string(nil),
			runtime.CorrelationReasons...,
		),
	}
	return nil
}

func executionCheckState(
	evidence connected.SafetyEvidence,
) planning.CheckState {
	if evidence.Available && evidence.Clear {
		return planning.CheckClear
	}
	if evidence.Available {
		return planning.CheckBlocked
	}
	return planning.CheckUnavailable
}

func (server *Server) executionStop(writer http.ResponseWriter, request *http.Request) {
	var stop struct {
		RequestID string `json:"request_id"`
		Reason    string `json:"reason"`
	}
	if err := decodeBoundedJSON(writer, request, 64<<10, &stop); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "execution_stop_invalid",
			"error": "execution stop request is invalid",
		})
		return
	}
	sessionID := strings.TrimSpace(request.PathValue("execution_id"))
	err := server.executionGateway.Stop(request.Context(), execution.StopRequest{
		RequestID: stop.RequestID,
		SessionID: sessionID,
		Reason:    stop.Reason,
	})
	if err != nil {
		status, code := executionHTTPError(err)
		writeJSON(writer, status, map[string]any{
			"ok": false, "code": code, "error": err.Error(),
		})
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{
		"schema":       "offline-programming.execution-stop.v1",
		"execution_id": sessionID,
		"stopped":      true,
	})
}

func (server *Server) executionSession(writer http.ResponseWriter, request *http.Request) {
	session, ok := server.executionGateway.Session(
		request.PathValue("execution_id"),
	)
	if !ok {
		writeJSON(writer, http.StatusNotFound, map[string]any{
			"ok": false, "code": "execution_not_found",
			"error": "execution session was not found",
		})
		return
	}
	writeJSON(writer, http.StatusOK, session)
}

func executionHTTPError(err error) (int, string) {
	switch {
	case errors.Is(err, execution.ErrSessionNotFound):
		return http.StatusNotFound, "execution_not_found"
	case errors.Is(err, execution.ErrReviewToken):
		return http.StatusConflict, "execution_review_token_invalid"
	case errors.Is(err, execution.ErrReviewDenied):
		return http.StatusConflict, "execution_review_denied"
	case errors.Is(err, execution.ErrIdempotencyReplay):
		return http.StatusConflict, "execution_idempotency_conflict"
	case errors.Is(err, execution.ErrUnavailable):
		return http.StatusServiceUnavailable, "execution_unavailable"
	case errors.Is(err, execution.ErrTransportReceipt):
		return http.StatusBadGateway, "execution_receipt_invalid"
	default:
		return http.StatusBadGateway, "execution_failed"
	}
}

func (server *Server) health(writer http.ResponseWriter, _ *http.Request) {
	local := server.localStatus()
	connectedCount := len(server.connectedDiscoveryEnvelope().Targets)
	targetPlanning := server.targetPlanningAvailable()
	executionCapabilities := server.executionGateway.Capabilities()
	identity := maintenanceServerIdentity()
	writeJSON(writer, http.StatusOK, map[string]any{
		"ok":                true,
		"schema":            "offline-programming.server-status.v1",
		"build_id":          identity.BuildID,
		"started_at":        identity.StartedAt,
		"maintenance":       map[string]any{"active": server.maintenance.Current().Active},
		"mode":              "offline_preview",
		"network_required":  false,
		"connected":         false,
		"connected_targets": connectedCount,
		"local_simulator":   local.Availability,
		"local_ready":       local.Readiness.Ready,
		"execution_enabled": executionCapabilities.InfrastructureReady,
		"target_planning":   targetPlanning,
		"teleop_enabled":    false,
		"program_catalog": map[string]any{
			"available": server.programCatalogAvailable(),
			"owner":     "motion-server",
		},
		"cell_programs": map[string]any{"enabled": server.programs != nil},
	})
}

func (server *Server) capabilities(writer http.ResponseWriter, _ *http.Request) {
	local := server.localStatus()
	connectedCount := len(server.connectedDiscoveryEnvelope().Targets)
	targetPlanning := server.targetPlanningAvailable()
	executionCapabilities := server.executionGateway.Capabilities()
	executionEnabled := executionCapabilities.InfrastructureReady
	writeJSON(writer, http.StatusOK, map[string]any{
		"schema": "offline-programming.capabilities.v1",
		"weld_planner": map[string]any{
			// Rails authoring is served from inside this process now, so this
			// is whether the seam worker is wired rather than whether a bridge
			// to another application is configured. Booleans only, as before:
			// no binding of any kind enters browser JSON.
			"enabled": server.seams != nil,
		},
		"targets": map[string]any{
			"offline_preview": map[string]any{
				"enabled": true, "movement": false,
			},
			"local_simulator": map[string]any{
				"enabled":           local.Availability.Available,
				"ready":             local.Readiness.Ready,
				"movement":          false,
				"explicit_start":    true,
				"canonical_planner": local.Availability.CanonicalPlanner,
				"process_bounded":   local.Availability.ProcessBounded,
				"availability":      local.Availability,
			},
			"real_connected": map[string]any{
				"enabled":               connectedCount > 0,
				"target_count":          connectedCount,
				"read_only":             true,
				"movement":              executionEnabled,
				"planning":              targetPlanning,
				"planning_authority":    "selected_robot_v4_motion_server",
				"raw_browser_transport": false,
			},
		},
		"planning": map[string]any{
			"enabled":                  local.Availability.Available || targetPlanning,
			"local_simulator":          local.Availability.Available,
			"physical_target":          targetPlanning,
			"server_mediated":          targetPlanning,
			"saved_program_required":   true,
			"browser_mints_trajectory": false,
			"reason":                   local.Availability.Code,
		},
		"connected_observation": map[string]any{
			"enabled":       connectedCount > 0,
			"read_only":     true,
			"auto_connect":  false,
			"server_routes": targetPlanning || executionEnabled,
		},
		"execution": map[string]any{
			"enabled":      executionEnabled,
			"server_owned": true,
			"reviewed":     true,
			"capabilities": executionCapabilities,
		},
		"teleop": map[string]any{"enabled": false, "reason": "reserved"},
		"program_catalog": map[string]any{
			"enabled":               server.programCatalogAvailable(),
			"owner":                 "motion-server",
			"same_origin_browser":   true,
			"optimistic_revisions":  true,
			"immutable_history":     true,
			"raw_browser_transport": false,
		},
	})
}

func (server *Server) targetPlanningAvailable() bool {
	if server == nil || server.targetPlanning == nil ||
		server.connectedTargets == nil {
		return false
	}
	for _, target := range server.connectedDiscoveryEnvelope().Targets {
		if server.targetPlanning.Available(target.ProfileID) {
			return true
		}
	}
	return false
}

type programSaveRequest struct {
	Schema          string          `json:"schema"`
	CatalogRevision uint64          `json:"catalog_revision"`
	Program         json.RawMessage `json:"program"`
}

type programSaveIdentity struct {
	SchemaVersion       int    `json:"schema_version"`
	ProgramID           string `json:"program_id"`
	ProgramRevision     uint64 `json:"program_revision"`
	BaseProgramRevision uint64 `json:"base_program_revision"`
	ProgramDigest       string `json:"program_digest"`
}

func (server *Server) programCatalogList(writer http.ResponseWriter, request *http.Request) {
	if server.programCatalog == nil {
		writeCatalogUnavailable(writer)
		return
	}
	response, err := server.programCatalog.List(request.Context())
	server.writeCatalogResult(writer, response, err)
}

func (server *Server) programCatalogGet(writer http.ResponseWriter, request *http.Request) {
	if server.programCatalog == nil {
		writeCatalogUnavailable(writer)
		return
	}
	programID := request.PathValue("program_id")
	if !catalog.ValidProgramID(programID) {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "invalid_program_id", "error": "program identity is invalid",
		})
		return
	}
	var revision *uint64
	if raw := strings.TrimSpace(request.URL.Query().Get("revision")); raw != "" {
		value, err := parsePositiveRevision(raw)
		if err != nil {
			writeJSON(writer, http.StatusBadRequest, map[string]any{
				"ok": false, "code": "invalid_program_revision", "error": err.Error(),
			})
			return
		}
		revision = &value
	}
	response, err := server.programCatalog.Get(request.Context(), programID, revision)
	server.writeCatalogResult(writer, response, err)
}

func (server *Server) programCatalogRevisions(writer http.ResponseWriter, request *http.Request) {
	if server.programCatalog == nil {
		writeCatalogUnavailable(writer)
		return
	}
	programID := request.PathValue("program_id")
	if !catalog.ValidProgramID(programID) {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "invalid_program_id", "error": "program identity is invalid",
		})
		return
	}
	response, err := server.programCatalog.Revisions(request.Context(), programID)
	server.writeCatalogResult(writer, response, err)
}

func (server *Server) programCatalogCreate(writer http.ResponseWriter, request *http.Request) {
	server.programCatalogSave(writer, request, "")
}

func (server *Server) programCatalogUpdate(writer http.ResponseWriter, request *http.Request) {
	server.programCatalogSave(writer, request, request.PathValue("program_id"))
}

func (server *Server) programCatalogSave(
	writer http.ResponseWriter,
	request *http.Request,
	expectedProgramID string,
) {
	if server.programCatalog == nil {
		writeCatalogUnavailable(writer)
		return
	}
	if expectedProgramID != "" && !catalog.ValidProgramID(expectedProgramID) {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "invalid_program_id", "error": "program identity is invalid",
		})
		return
	}
	request.Body = http.MaxBytesReader(writer, request.Body, catalog.MaxRequestBytes)
	decoder := json.NewDecoder(request.Body)
	decoder.DisallowUnknownFields()
	var save programSaveRequest
	if err := decoder.Decode(&save); err != nil || requireJSONEOF(decoder) != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "invalid_program_save", "error": "program save request is invalid",
		})
		return
	}
	if save.Schema != "offline-programming.program-save.v1" || len(save.Program) == 0 {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "invalid_program_save", "error": "program save schema and program are required",
		})
		return
	}
	var identity programSaveIdentity
	if err := json.Unmarshal(save.Program, &identity); err != nil ||
		identity.SchemaVersion != 1 ||
		!catalog.ValidProgramID(identity.ProgramID) ||
		identity.ProgramRevision == 0 ||
		!validSHA256(identity.ProgramDigest) {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "invalid_program", "error": "shared Robot V4 program identity is invalid",
		})
		return
	}
	if expectedProgramID == "" {
		if identity.BaseProgramRevision != 0 {
			writeJSON(writer, http.StatusConflict, map[string]any{
				"ok": false, "code": "catalog_conflict",
				"error": "new programs must use base revision 0",
			})
			return
		}
	} else if identity.ProgramID != expectedProgramID {
		writeJSON(writer, http.StatusConflict, map[string]any{
			"ok": false, "code": "catalog_conflict",
			"error": "route program identity does not match the saved program",
		})
		return
	} else if identity.BaseProgramRevision == 0 ||
		identity.ProgramRevision <= identity.BaseProgramRevision {
		writeJSON(writer, http.StatusConflict, map[string]any{
			"ok": false, "code": "catalog_conflict",
			"error": "updated program revision must be greater than its explicit base revision",
		})
		return
	}
	mutation := catalog.MutationContext{
		RequestID:      request.Header.Get("X-Request-ID"),
		CorrelationID:  request.Header.Get("X-Correlation-ID"),
		IdempotencyKey: request.Header.Get("Idempotency-Key"),
	}
	if err := mutation.Validate(); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "invalid_mutation_identity", "error": err.Error(),
		})
		return
	}
	upstream, err := json.Marshal(map[string]any{
		"schema":           "robot.v4.program_catalog.v1",
		"owner":            "motion-server",
		"client":           "offline-programming",
		"catalog_revision": save.CatalogRevision,
		"programs":         []json.RawMessage{save.Program},
	})
	if err != nil {
		writeJSON(writer, http.StatusInternalServerError, map[string]any{
			"ok": false, "code": "catalog_encoding_failed", "error": "program catalog request could not be encoded",
		})
		return
	}
	response, err := server.programCatalog.Save(request.Context(), upstream, mutation)
	server.writeCatalogResult(writer, response, err)
}

func (server *Server) writeCatalogResult(
	writer http.ResponseWriter,
	response catalog.Response,
	err error,
) {
	if err != nil {
		status := http.StatusBadGateway
		code := "catalog_invalid_response"
		if errors.Is(err, catalog.ErrUnavailable) {
			status = http.StatusServiceUnavailable
			code = "catalog_unavailable"
		}
		writeJSON(writer, status, map[string]any{
			"ok": false, "code": code, "error": "Motion Server program catalog is unavailable",
		})
		return
	}
	status := response.Status
	if status < 200 || status > 599 {
		status = http.StatusBadGateway
	}
	writer.Header().Set("Content-Type", "application/json; charset=utf-8")
	writer.Header().Set("Cache-Control", "no-store")
	writer.WriteHeader(status)
	_, _ = writer.Write(append(response.Body, '\n'))
}

func (server *Server) programCatalogAvailable() bool {
	return server.programCatalog != nil && server.programCatalog.Available()
}

func writeCatalogUnavailable(writer http.ResponseWriter) {
	writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
		"ok": false, "code": "catalog_unavailable",
		"error": "Motion Server program catalog is not configured",
	})
}

func parsePositiveRevision(value string) (uint64, error) {
	var revision uint64
	if _, err := fmt.Sscan(value, &revision); err != nil || revision == 0 ||
		fmt.Sprint(revision) != value {
		return 0, errors.New("program revision must be a positive integer")
	}
	return revision, nil
}

func validSHA256(value string) bool {
	if len(value) != 71 || !strings.HasPrefix(value, "sha256:") {
		return false
	}
	for _, char := range value[7:] {
		if (char < '0' || char > '9') && (char < 'a' || char > 'f') {
			return false
		}
	}
	return true
}

func (server *Server) localSimulatorStatus(writer http.ResponseWriter, request *http.Request) {
	if server.localSimulator == nil {
		writeJSON(writer, http.StatusOK, simulator.StatusEnvelope{Status: server.localStatus()})
		return
	}
	status := server.localSimulator.Status()
	if status.Phase == simulator.PhaseReady {
		refreshContext, cancel := context.WithTimeout(request.Context(), 2*time.Second)
		defer cancel()
		refreshed, _ := server.localSimulator.Refresh(refreshContext)
		status = refreshed
	}
	writeJSON(writer, http.StatusOK, simulator.StatusEnvelope{Status: status})
}

func (server *Server) localSimulatorStart(writer http.ResponseWriter, request *http.Request) {
	if server.localSimulator == nil {
		writeSimulatorError(
			writer,
			http.StatusServiceUnavailable,
			server.localStatus(),
			&simulator.ServiceError{
				Code:      simulator.ErrorBackendUnavailable,
				Operation: "start",
				Message:   "canonical local simulator service is unavailable",
			},
		)
		return
	}
	if !emptyRequestBody(request) {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": simulator.ErrorInvalidConfiguration,
			"error": "local simulator start accepts no request body",
		})
		return
	}
	status, err := server.localSimulator.Start(request.Context())
	if err != nil {
		httpStatus, serviceError := simulatorHTTPError(err)
		writeSimulatorError(writer, httpStatus, status, serviceError)
		return
	}
	writeJSON(writer, http.StatusOK, simulator.StatusEnvelope{Status: status})
}

// Jog endpoints: the UI's pendant seat on the isolated simulation cell.
// The session speaks the Steam Deck's exact wire pathway to the real
// robot-v4-cartesiand runtime — no physical output can exist here.
func (server *Server) localSimulatorJogStart(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "code": "local_simulation_jog_unavailable", "error": "jog runtime is not configured",
		})
		return
	}
	state, err := server.jogManager.Start()
	if err != nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": err.Error(),
		})
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{"ok": true, "state": state})
}

func (server *Server) localSimulatorJogStop(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "jog runtime is not configured",
		})
		return
	}
	server.jogManager.Stop()
	writeJSON(writer, http.StatusOK, map[string]any{"ok": true})
}

func (server *Server) localSimulatorJogIntent(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "jog runtime is not configured",
		})
		return
	}
	var intent simulator.JogIntent
	if err := json.NewDecoder(io.LimitReader(request.Body, 4096)).Decode(&intent); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "invalid jog intent: " + err.Error(),
		})
		return
	}
	state, err := server.jogManager.Intent(intent)
	if err != nil {
		writeJSON(writer, http.StatusConflict, map[string]any{
			"ok": false, "error": err.Error(), "state": state,
		})
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{"ok": true, "state": state})
}

func (server *Server) localSimulatorJogHome(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "jog runtime is not configured",
		})
		return
	}
	state, err := server.jogManager.GoHome()
	if err != nil {
		writeJSON(writer, http.StatusConflict, map[string]any{
			"ok": false, "error": err.Error(),
		})
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{"ok": true, "state": state})
}

func (server *Server) localSimulatorJogHalt(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "jog runtime is not configured",
		})
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{
		"ok": true, "state": server.jogManager.Halt(),
	})
}

func (server *Server) localSimulatorJogSetZero(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "jog runtime is not configured",
		})
		return
	}
	var body struct {
		AxisMask uint32 `json:"axisMask"`
	}
	if err := json.NewDecoder(io.LimitReader(request.Body, 512)).Decode(&body); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "invalid set-zero request: " + err.Error(),
		})
		return
	}
	state, err := server.jogManager.SetZero(body.AxisMask)
	if err != nil {
		writeJSON(writer, http.StatusConflict, map[string]any{
			"ok": false, "error": err.Error(),
		})
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{"ok": true, "state": state})
}

func (server *Server) localSimulatorJogStep(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "jog runtime is not configured",
		})
		return
	}
	var body struct {
		Joint    int     `json:"joint"`
		DeltaDeg float64 `json:"deltaDeg"`
	}
	if err := json.NewDecoder(io.LimitReader(request.Body, 512)).Decode(&body); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "invalid step request: " + err.Error(),
		})
		return
	}
	if err := server.jogManager.StepJoint(body.Joint, body.DeltaDeg*math.Pi/180); err != nil {
		writeJSON(writer, http.StatusConflict, map[string]any{
			"ok": false, "error": err.Error(),
		})
		return
	}
	writeJSON(writer, http.StatusAccepted, map[string]any{"ok": true})
}

func (server *Server) localSimulatorJogState(writer http.ResponseWriter, request *http.Request) {
	if server.jogManager == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "jog runtime is not configured",
		})
		return
	}
	writeJSON(writer, http.StatusOK, map[string]any{
		"ok": true, "state": server.jogManager.State(),
	})
}

func (server *Server) localSimulatorStop(writer http.ResponseWriter, request *http.Request) {
	if server.localSimulator == nil {
		writeJSON(writer, http.StatusOK, simulator.StatusEnvelope{Status: server.localStatus()})
		return
	}
	if !emptyRequestBody(request) {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": simulator.ErrorInvalidConfiguration,
			"error": "local simulator stop accepts no request body",
		})
		return
	}
	status, err := server.localSimulator.Stop(request.Context())
	if err != nil {
		httpStatus, serviceError := simulatorHTTPError(err)
		writeSimulatorError(writer, httpStatus, status, serviceError)
		return
	}
	writeJSON(writer, http.StatusOK, simulator.StatusEnvelope{Status: status})
}

func (server *Server) localSimulatorPlan(writer http.ResponseWriter, request *http.Request) {
	if server.localSimulator == nil {
		writeSimulatorError(
			writer,
			http.StatusServiceUnavailable,
			server.localStatus(),
			&simulator.ServiceError{
				Code:      simulator.ErrorBackendUnavailable,
				Operation: "plan",
				Message:   "canonical local simulator service is unavailable",
			},
		)
		return
	}
	request.Body = http.MaxBytesReader(writer, request.Body, maxPlanRequestBytes)
	decoder := json.NewDecoder(request.Body)
	decoder.DisallowUnknownFields()
	var incoming struct {
		Schema          string                  `json:"schema"`
		RequestID       string                  `json:"request_id"`
		ProjectID       string                  `json:"project_id"`
		ProjectRevision uint64                  `json:"project_revision"`
		Target          planning.TargetIdentity `json:"target"`
		SubmittedAt     time.Time               `json:"submitted_at"`
		Program         json.RawMessage         `json:"program"`
	}
	if err := decoder.Decode(&incoming); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": simulator.ErrorInvalidPlan,
			"error": "invalid bounded shared-program planning request",
		})
		return
	}
	if err := requireJSONEOF(decoder); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": simulator.ErrorInvalidPlan,
			"error": "local plan request contains trailing JSON",
		})
		return
	}
	planRequest, err := planning.NewNormalizedPlanRequestFromRobotV4Program(
		incoming.Schema,
		incoming.RequestID,
		incoming.ProjectID,
		incoming.ProjectRevision,
		incoming.Target,
		incoming.SubmittedAt,
		incoming.Program,
	)
	if err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": simulator.ErrorInvalidPlan,
			"error": err.Error(),
		})
		return
	}
	result, err := server.localSimulator.SubmitPlan(request.Context(), planRequest)
	if err == nil || errors.Is(err, simulator.ErrPlanRejected) {
		writeJSON(writer, http.StatusOK, simulator.PlanEnvelope{Result: result})
		return
	}
	httpStatus, serviceError := simulatorHTTPError(err)
	writeSimulatorPlanError(writer, httpStatus, result, serviceError)
}

func (server *Server) targetPlan(writer http.ResponseWriter, request *http.Request) {
	profileID := strings.TrimSpace(request.PathValue("profile"))
	if server.targetPlanning == nil || server.connectedTargets == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "code": "target_planning_unavailable",
			"error": "server-owned target planning transport is not configured",
		})
		return
	}
	target, ok := server.connectedTargets.PlanningTarget(profileID)
	if !ok {
		writeJSON(writer, http.StatusNotFound, map[string]any{
			"ok": false, "code": "target_not_found",
			"error": "manifest-backed connected target was not found",
		})
		return
	}
	request.Body = http.MaxBytesReader(writer, request.Body, maxPlanRequestBytes)
	decoder := json.NewDecoder(request.Body)
	decoder.DisallowUnknownFields()
	var incoming struct {
		Schema          string          `json:"schema"`
		RequestID       string          `json:"request_id"`
		CorrelationID   string          `json:"correlation_id"`
		IdempotencyKey  string          `json:"idempotency_key"`
		ProjectID       string          `json:"project_id"`
		ProjectRevision uint64          `json:"project_revision"`
		SubmittedAt     time.Time       `json:"submitted_at"`
		Program         json.RawMessage `json:"program"`
	}
	if err := decoder.Decode(&incoming); err != nil ||
		incoming.Schema != targetplanning.RequestSchema {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "target_plan_invalid",
			"error": "invalid bounded target planning request",
		})
		return
	}
	if err := requireJSONEOF(decoder); err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "target_plan_invalid",
			"error": "target plan request contains trailing JSON",
		})
		return
	}
	planRequest, err := planning.NewNormalizedPlanRequestFromRobotV4Program(
		planning.NormalizedPlanRequestSchema,
		incoming.RequestID,
		incoming.ProjectID,
		incoming.ProjectRevision,
		planning.TargetIdentity{
			ProfileID:   profileID,
			ManifestRef: target.Discovery.ManifestRef,
		},
		incoming.SubmittedAt,
		incoming.Program,
	)
	if err != nil {
		writeJSON(writer, http.StatusBadRequest, map[string]any{
			"ok": false, "code": "target_plan_invalid",
			"error": err.Error(),
		})
		return
	}
	plan, err := server.targetPlanning.Submit(
		request.Context(),
		profileID,
		planRequest,
		targetplanning.MutationIdentity{
			CorrelationID:  incoming.CorrelationID,
			IdempotencyKey: incoming.IdempotencyKey,
		},
	)
	if err != nil {
		status, code := targetPlanningHTTPError(err)
		writeJSON(writer, status, map[string]any{
			"ok": false, "code": code, "error": err.Error(),
		})
		return
	}
	writeJSON(writer, http.StatusOK, targetplanning.PlanEnvelope{
		Schema: targetplanning.EnvelopeSchema,
		Plan:   plan,
	})
}

func (server *Server) acceptedPlan(writer http.ResponseWriter, request *http.Request) {
	planID := strings.TrimSpace(request.PathValue("plan_id"))
	if server.targetPlanning == nil || planID == "" || len(planID) > 128 {
		writeJSON(writer, http.StatusNotFound, map[string]any{
			"ok": false, "code": "accepted_plan_not_found",
			"error": "accepted plan was not found",
		})
		return
	}
	plan, ok := server.targetPlanning.Get(planID)
	if !ok {
		writeJSON(writer, http.StatusNotFound, map[string]any{
			"ok": false, "code": "accepted_plan_not_found",
			"error": "accepted plan was not found",
		})
		return
	}
	writeJSON(writer, http.StatusOK, targetplanning.PlanEnvelope{
		Schema: targetplanning.EnvelopeSchema,
		Plan:   plan,
	})
}

func targetPlanningHTTPError(err error) (int, string) {
	switch {
	case errors.Is(err, targetplanning.ErrTargetNotFound):
		return http.StatusNotFound, "target_not_found"
	case errors.Is(err, targetplanning.ErrInvalidRequest):
		return http.StatusBadRequest, "target_plan_invalid"
	case errors.Is(err, targetplanning.ErrIdempotencyReplay):
		return http.StatusConflict, "target_plan_idempotency_conflict"
	case errors.Is(err, targetplanning.ErrProgramNotSaved):
		return http.StatusConflict, "target_plan_program_not_saved"
	case errors.Is(err, targetplanning.ErrProgramMismatch):
		return http.StatusConflict, "target_plan_program_mismatch"
	case errors.Is(err, targetplanning.ErrRejected):
		return http.StatusUnprocessableEntity, "target_plan_rejected"
	case errors.Is(err, targetplanning.ErrUnavailable):
		return http.StatusServiceUnavailable, "target_planning_unavailable"
	case errors.Is(err, targetplanning.ErrIdentityMismatch):
		return http.StatusBadGateway, "target_plan_identity_mismatch"
	default:
		return http.StatusBadGateway, "target_plan_failed"
	}
}

func (server *Server) connectedDiscovery(writer http.ResponseWriter, _ *http.Request) {
	writeJSON(writer, http.StatusOK, server.connectedDiscoveryEnvelope())
}

func (server *Server) connectedStatus(writer http.ResponseWriter, request *http.Request) {
	status, ok := server.connectedTargets.Status(request.PathValue("profile"), time.Now().UTC())
	if !ok {
		writeJSON(writer, http.StatusNotFound, map[string]any{
			"ok": false, "code": "target_not_found",
			"error": "manifest-backed connected target was not found",
		})
		return
	}
	writeJSON(writer, http.StatusOK, status)
}

func (server *Server) connectedCapabilities(writer http.ResponseWriter, request *http.Request) {
	report, ok := server.connectedTargets.Capabilities(request.PathValue("profile"), time.Now().UTC())
	if !ok {
		writeJSON(writer, http.StatusNotFound, map[string]any{
			"ok": false, "code": "target_not_found",
			"error": "manifest-backed connected target was not found",
		})
		return
	}
	writeJSON(writer, http.StatusOK, report)
}

func (server *Server) cadQuery(writer http.ResponseWriter, request *http.Request) {
	if server.worker == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok":    false,
			"code":  cad.ErrorUnavailable,
			"error": "CadQuery worker unavailable",
		})
		return
	}
	request.Body = http.MaxBytesReader(writer, request.Body, maxCADRequestBytes)
	raw, err := io.ReadAll(request.Body)
	if err != nil {
		writeJSON(writer, http.StatusRequestEntityTooLarge, map[string]any{
			"ok":    false,
			"code":  cad.ErrorPayloadTooLarge,
			"error": "CAD request exceeds the 12 MiB limit",
		})
		return
	}
	result, err := server.worker.ExecuteJSON(request.Context(), raw)
	if err != nil {
		status, code := cadHTTPError(err)
		writeJSON(writer, status, map[string]any{
			"ok":    false,
			"code":  code,
			"error": err.Error(),
		})
		return
	}
	if !json.Valid(result.JSON) {
		writeJSON(writer, http.StatusBadGateway, map[string]any{
			"ok":    false,
			"code":  cad.ErrorInvalidResponse,
			"error": "CadQuery worker returned invalid JSON",
		})
		return
	}
	writer.Header().Set("Content-Type", "application/json; charset=utf-8")
	writer.Header().Set("Cache-Control", "no-store")
	writer.Header().Set("X-Offline-CAD-Cache", "miss")
	if result.CacheHit {
		writer.Header().Set("X-Offline-CAD-Cache", "hit")
	}
	writer.WriteHeader(http.StatusOK)
	_, _ = writer.Write(result.JSON)
}

func cadHTTPError(err error) (int, cad.ErrorCode) {
	var workerErr *cad.WorkerError
	if errors.As(err, &workerErr) {
		switch workerErr.Code {
		case cad.ErrorInvalidRequest:
			return http.StatusBadRequest, workerErr.Code
		case cad.ErrorPayloadTooLarge:
			return http.StatusRequestEntityTooLarge, workerErr.Code
		case cad.ErrorBusy:
			return http.StatusTooManyRequests, workerErr.Code
		case cad.ErrorTimeout:
			return http.StatusGatewayTimeout, workerErr.Code
		case cad.ErrorCanceled:
			return http.StatusRequestTimeout, workerErr.Code
		case cad.ErrorUnavailable:
			return http.StatusServiceUnavailable, workerErr.Code
		case cad.ErrorWorkerFailed, cad.ErrorInvalidResponse:
			return http.StatusBadGateway, workerErr.Code
		}
	}
	return http.StatusInternalServerError, cad.ErrorWorkerFailed
}

func simulatorHTTPError(err error) (int, *simulator.ServiceError) {
	var serviceError *simulator.ServiceError
	if !errors.As(err, &serviceError) {
		serviceError = &simulator.ServiceError{
			Code:      simulator.ErrorBackendUnavailable,
			Operation: "unknown",
			Message:   "local simulator failed closed",
		}
	}
	switch serviceError.Code {
	case simulator.ErrorInvalidConfiguration, simulator.ErrorInvalidLifecycle,
		simulator.ErrorInvalidPlan:
		return http.StatusBadRequest, serviceError
	case simulator.ErrorNotReady, simulator.ErrorBusy:
		return http.StatusConflict, serviceError
	case simulator.ErrorResourceLimit:
		return http.StatusRequestEntityTooLarge, serviceError
	case simulator.ErrorUncorrelatedReport:
		return http.StatusBadGateway, serviceError
	case simulator.ErrorBackendUnavailable, simulator.ErrorUnsafeRuntime:
		return http.StatusServiceUnavailable, serviceError
	case simulator.ErrorPlanRejected:
		return http.StatusOK, serviceError
	default:
		return http.StatusInternalServerError, serviceError
	}
}

func writeSimulatorError(
	writer http.ResponseWriter,
	statusCode int,
	status simulator.Status,
	err *simulator.ServiceError,
) {
	writeJSON(writer, statusCode, map[string]any{
		"ok":     false,
		"code":   err.Code,
		"error":  err.Message,
		"status": status,
	})
}

func writeSimulatorPlanError(
	writer http.ResponseWriter,
	statusCode int,
	result planning.NormalizedPlanResult,
	err *simulator.ServiceError,
) {
	payload := map[string]any{
		"ok":    false,
		"code":  err.Code,
		"error": err.Message,
	}
	if result.Schema != "" {
		payload["result"] = result
	}
	writeJSON(writer, statusCode, payload)
}

func (server *Server) localStatus() simulator.Status {
	if server.localSimulator != nil {
		return server.localSimulator.Status()
	}
	now := time.Now().UTC()
	return simulator.Status{
		Schema:         simulator.ServiceStatusSchema,
		ServiceID:      "offline-programming-local",
		StatusSequence: 1,
		Phase:          simulator.PhaseFailed,
		ObservedAt:     now,
		Availability: simulator.BackendAvailability{
			Schema:            simulator.AvailabilitySchema,
			Available:         false,
			Code:              simulator.AvailabilityCanonicalBinary,
			Reason:            "canonical local simulator service is unavailable",
			CanonicalPlanner:  false,
			ProcessBounded:    false,
			PersistentProcess: false,
			ExplicitOptIn:     true,
		},
		Topology: simulator.NewPCTopology(
			"local-simulator",
			planning.RobotModelRosie1400V3,
			"127.0.0.1",
		),
		Readiness: simulator.Readiness{
			Ready:   false,
			Reasons: []string{"backend_unavailable"},
		},
		LastErrorCode: simulator.ErrorBackendUnavailable,
		LastError:     "canonical local simulator service is unavailable",
	}
}

func (server *Server) connectedDiscoveryEnvelope() connectedDiscoveryEnvelope {
	if server.connectedTargets == nil {
		return (&ConnectedRegistry{}).Discovery()
	}
	return server.connectedTargets.Discovery()
}

func emptyRequestBody(request *http.Request) bool {
	if request.Body == nil || request.Body == http.NoBody {
		return true
	}
	raw, err := io.ReadAll(io.LimitReader(request.Body, 2))
	return err == nil && len(strings.TrimSpace(string(raw))) == 0
}

func requireJSONEOF(decoder *json.Decoder) error {
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); errors.Is(err, io.EOF) {
		return nil
	} else {
		return err
	}
}

func decodeBoundedJSON(
	writer http.ResponseWriter,
	request *http.Request,
	maximum int64,
	out any,
) error {
	request.Body = http.MaxBytesReader(writer, request.Body, maximum)
	decoder := json.NewDecoder(request.Body)
	if err := decoder.Decode(out); err != nil {
		return err
	}
	return requireJSONEOF(decoder)
}

func disabledCapability(reason string) http.HandlerFunc {
	return func(writer http.ResponseWriter, _ *http.Request) {
		writeJSON(writer, http.StatusForbidden, map[string]any{
			"ok":      false,
			"enabled": false,
			"error":   reason,
		})
	}
}

func noDirectoryListing(next http.Handler, root string) http.Handler {
	return http.HandlerFunc(func(writer http.ResponseWriter, request *http.Request) {
		if strings.HasSuffix(request.URL.Path, "/") {
			path := filepath.Join(root, filepath.FromSlash(strings.TrimPrefix(request.URL.Path, "/")))
			if info, err := os.Stat(path); err == nil && info.IsDir() && !fileExists(filepath.Join(path, "index.html")) {
				http.NotFound(writer, request)
				return
			}
		}
		next.ServeHTTP(writer, request)
	})
}

// gzipResponses compresses JSON and text responses for clients that accept
// it. The catalogue listing is 2.5 MB of JSON and the app fetches it at
// every boot; over a Tailscale link from a laptop that alone was most of
// the wait, and it compresses about ten to one. Event streams (the weld
// planner's job progress, proxied through here) are passed untouched: a
// buffered compressor would hold their events back, which is the opposite
// of what a progress stream is for. Streamed responses that do get
// compressed keep working because Flush reaches the compressor first.
func gzipResponses(next http.Handler) http.Handler {
	return http.HandlerFunc(func(writer http.ResponseWriter, request *http.Request) {
		if !acceptsGzip(request.Header.Values("Accept-Encoding")) {
			next.ServeHTTP(writer, request)
			return
		}
		compressed := &gzipResponseWriter{ResponseWriter: writer}
		defer compressed.Close()
		next.ServeHTTP(compressed, request)
	})
}

type gzipResponseWriter struct {
	http.ResponseWriter
	writer      *gzip.Writer
	wroteHeader bool
	compress    bool
}

// Unwrap lets http.ResponseController reach the connection (the cell
// camera's video stream clears the server's write timeout with it).
func (writer *gzipResponseWriter) Unwrap() http.ResponseWriter { return writer.ResponseWriter }

func compressibleContentType(value string) bool {
	value = strings.ToLower(value)
	if strings.HasPrefix(value, "text/event-stream") {
		return false
	}
	return strings.HasPrefix(value, "application/json") ||
		strings.HasPrefix(value, "text/") ||
		strings.HasPrefix(value, "application/javascript") ||
		strings.HasPrefix(value, "image/svg+xml")
}

func (writer *gzipResponseWriter) WriteHeader(status int) {
	if writer.wroteHeader {
		return
	}
	writer.wroteHeader = true
	header := writer.Header()
	if header.Get("Vary") != "Accept-Encoding" {
		header.Add("Vary", "Accept-Encoding")
	}
	if status != http.StatusNoContent && status != http.StatusNotModified &&
		header.Get("Content-Encoding") == "" && compressibleContentType(header.Get("Content-Type")) {
		header.Set("Content-Encoding", "gzip")
		header.Del("Content-Length")
		writer.compress = true
		writer.writer = gzip.NewWriter(writer.ResponseWriter)
	}
	writer.ResponseWriter.WriteHeader(status)
}

func (writer *gzipResponseWriter) Write(data []byte) (int, error) {
	if !writer.wroteHeader {
		if writer.Header().Get("Content-Type") == "" {
			writer.Header().Set("Content-Type", http.DetectContentType(data))
		}
		writer.WriteHeader(http.StatusOK)
	}
	if writer.compress {
		return writer.writer.Write(data)
	}
	return writer.ResponseWriter.Write(data)
}

func (writer *gzipResponseWriter) Flush() {
	if writer.compress && writer.writer != nil {
		_ = writer.writer.Flush()
	}
	if flusher, ok := writer.ResponseWriter.(http.Flusher); ok {
		flusher.Flush()
	}
}

func (writer *gzipResponseWriter) Close() {
	if writer.compress && writer.writer != nil {
		_ = writer.writer.Close()
	}
}

func securityHeaders(next http.Handler) http.Handler {
	return http.HandlerFunc(func(writer http.ResponseWriter, request *http.Request) {
		writer.Header().Set("X-Content-Type-Options", "nosniff")
		writer.Header().Set("Referrer-Policy", "no-referrer")
		writer.Header().Set("Permissions-Policy", "camera=(), microphone=(), geolocation=(), payment=(), usb=()")
		writer.Header().Set("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; worker-src 'self'; object-src 'none'; base-uri 'self'; frame-ancestors 'none'")
		if strings.HasSuffix(request.URL.Path, "service-worker.js") {
			writer.Header().Set("Cache-Control", "no-cache")
			writer.Header().Set("Service-Worker-Allowed", uiBasePath)
		}
		next.ServeHTTP(writer, request)
	})
}

func writeJSON(writer http.ResponseWriter, status int, value any) {
	raw, err := json.Marshal(value)
	if err != nil {
		http.Error(writer, fmt.Sprintf("encode response: %v", err), http.StatusInternalServerError)
		return
	}
	writer.Header().Set("Content-Type", "application/json; charset=utf-8")
	writer.Header().Set("Cache-Control", "no-store")
	writer.WriteHeader(status)
	_, _ = writer.Write(append(raw, '\n'))
}

// SeamAPIPath is where rails mode reaches the seam worker. One route, because
// the operation is named in the body: the browser posts the worker's own
// request shape and this forwards it, the way the CadQuery route does.
const SeamAPIPath = "/api/offline-programming/v1/seam"

func (server *Server) seamWorker(writer http.ResponseWriter, request *http.Request) {
	if server.seams == nil {
		writeJSON(writer, http.StatusServiceUnavailable, map[string]any{
			"ok":    false,
			"code":  seam.ErrorUnavailable,
			"error": "seam worker unavailable",
		})
		return
	}
	request.Body = http.MaxBytesReader(writer, request.Body, maxCADRequestBytes)
	raw, err := io.ReadAll(request.Body)
	if err != nil {
		writeJSON(writer, http.StatusRequestEntityTooLarge, map[string]any{
			"ok":    false,
			"code":  seam.ErrorPayloadTooLarge,
			"error": "seam request exceeds the 12 MiB limit",
		})
		return
	}
	result, err := server.seams.ExecuteJSON(request.Context(), raw)
	if err != nil {
		status, code := seamHTTPError(err)
		writeJSON(writer, status, map[string]any{"ok": false, "code": code, "error": err.Error()})
		return
	}
	if !json.Valid(result.JSON) {
		writeJSON(writer, http.StatusBadGateway, map[string]any{
			"ok":    false,
			"code":  seam.ErrorInvalidResponse,
			"error": "seam worker returned invalid JSON",
		})
		return
	}
	writer.Header().Set("Content-Type", "application/json; charset=utf-8")
	writer.Header().Set("Cache-Control", "no-store")
	// The part the answer is about, so a caller holding several can tell them
	// apart without re-reading the STEP it sent.
	writer.Header().Set("X-Offline-Seam-Part", result.Fingerprint)
	writer.WriteHeader(http.StatusOK)
	_, _ = writer.Write(result.JSON)
}

func seamHTTPError(err error) (int, seam.ErrorCode) {
	var workerErr *seam.WorkerError
	if errors.As(err, &workerErr) {
		switch workerErr.Code {
		case seam.ErrorInvalidRequest:
			return http.StatusBadRequest, workerErr.Code
		case seam.ErrorPayloadTooLarge:
			return http.StatusRequestEntityTooLarge, workerErr.Code
		case seam.ErrorBusy:
			return http.StatusTooManyRequests, workerErr.Code
		case seam.ErrorTimeout:
			return http.StatusGatewayTimeout, workerErr.Code
		case seam.ErrorCanceled:
			return http.StatusRequestTimeout, workerErr.Code
		case seam.ErrorUnavailable:
			return http.StatusServiceUnavailable, workerErr.Code
		case seam.ErrorWorkerFailed, seam.ErrorInvalidResponse:
			return http.StatusBadGateway, workerErr.Code
		}
	}
	return http.StatusInternalServerError, seam.ErrorWorkerFailed
}

// Where this server keeps the descriptions it fetched from cells.
func robotDescriptionCache(options ServerOptions) string {
	if root := strings.TrimSpace(options.RobotDescriptionCache); root != "" {
		return root
	}
	cache, err := os.UserCacheDir()
	if err != nil {
		return ""
	}
	return filepath.Join(cache, "rosieos-olp", "robot-descriptions")
}

// Where this server reads robot descriptions from.
func robotDescriptionRoot(options ServerOptions) string {
	if root := strings.TrimSpace(options.RobotDescriptionRoot); root != "" {
		return root
	}
	return filepath.Join(options.RepoRoot, "robot_description", "robots")
}
