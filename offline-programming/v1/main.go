package main

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"io/fs"
	"net"
	"net/http"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"runtime"
	"slices"
	"strings"
	"syscall"
	"time"

	"rosieos/offline-programming/v1/internal/cad"
	"rosieos/offline-programming/v1/internal/catalog"
	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/execution"
	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/programstore"
	"rosieos/offline-programming/v1/internal/seam"
	"rosieos/offline-programming/v1/internal/simulator"
	"rosieos/offline-programming/v1/internal/targetplanning"
	"rosieos/offline-programming/v1/internal/targets"
)

const usage = `usage: offline-programming [v1] <command> [flags]

Commands:
  install   Download Go modules and install UI dependencies
  clean     Remove generated build, staging, and deploy state
  lint      Run Go vet and strict TypeScript checks
  build     Build the Go wrapper/server and production PWA
  test      Run scoped Go and TypeScript unit tests
  ui        Start the Vite development UI
  serve     Serve the built PWA and local-only bounded APIs
  deploy    Build and stage a local deploy bundle
  compile-context
            Write the planning context for a robot model to a directory
  maintenance
            Set, clear or show the update notice that blocks robot actions
  help      Show this help

Examples:
  ./rosie.sh offline-programming v1 install
  ./rosie.sh offline-programming v1 test
  ./rosie.sh offline-programming v1 ui --host 127.0.0.1 --port 5189
  ./rosie.sh offline-programming v1 serve --listen 127.0.0.1:8794
  ./rosie.sh offline-programming v1 serve --listen 127.0.0.1:8794 --enable-local-simulator
  ./rosie.sh offline-programming v1 serve --program-catalog-origin http://127.0.0.1:8787
  ./rosie.sh offline-programming v1 serve --program-catalog-origin http://127.0.0.1:8787 --enable-connected-planning --daemon-state-url http://127.0.0.1:8788
`

var runExternalCommand = runExternal

func main() {
	os.Exit(Run(os.Args[1:], os.Stdout, os.Stderr))
}

// Run is the testable command entrypoint.
func Run(args []string, stdout, stderr io.Writer) int {
	if len(args) > 0 && args[0] == "v1" {
		args = args[1:]
	}
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, usage)
		return 0
	}

	command := args[0]
	rest := args[1:]
	var err error
	switch command {
	case "install":
		err = runInstall(stdout, stderr)
	case "clean":
		err = runClean(stdout)
	case "lint":
		err = runLint(stdout, stderr)
	case "build":
		err = runBuild(stdout, stderr)
	case "test":
		err = runTests(rest, stdout, stderr)
	case "ui":
		err = runUI(rest, stdout, stderr)
	case "serve":
		err = runServe(rest, stdout, stderr)
	case "deploy":
		err = runDeploy(rest, stdout, stderr)
	case "compile-context":
		err = runCompileContext(rest, stdout, stderr)
	case "maintenance":
		err = runMaintenance(rest, stdout, stderr)
	default:
		fmt.Fprintf(stderr, "offline-programming v1: unknown command %q\n\n%s", command, usage)
		return 2
	}
	if err != nil {
		fmt.Fprintf(stderr, "offline-programming v1 %s: %v\n", command, err)
		return 1
	}
	return 0
}

func runInstall(stdout, stderr io.Writer) error {
	root, err := projectRoot()
	if err != nil {
		return err
	}
	fmt.Fprintln(stdout, "install: downloading Go modules")
	if err := runExternal(root, stdout, stderr, "go", "mod", "download"); err != nil {
		return err
	}
	fmt.Fprintln(stdout, "install: installing UI dependencies")
	return runExternal(filepath.Join(root, "ui"), stdout, stderr, "npm", "install")
}

func runClean(stdout io.Writer) error {
	root, err := projectRoot()
	if err != nil {
		return err
	}
	for _, path := range []string{
		filepath.Join(root, "bin"),
		filepath.Join(root, "deploy"),
		filepath.Join(root, "test.log"),
		filepath.Join(root, "ui", "dist"),
		filepath.Join(root, "ui", ".staged-public"),
	} {
		if err := removeGeneratedPath(root, path); err != nil {
			return err
		}
		fmt.Fprintf(stdout, "clean: removed %s\n", path)
	}
	return nil
}

func runLint(stdout, stderr io.Writer) error {
	root, err := projectRoot()
	if err != nil {
		return err
	}
	fmt.Fprintln(stdout, "lint: go vet ./...")
	if err := runExternal(root, stdout, stderr, "go", "vet", "./..."); err != nil {
		return err
	}
	if !dirExists(filepath.Join(root, "ui", "node_modules")) {
		return errors.New("ui/node_modules is missing; run install first")
	}
	fmt.Fprintln(stdout, "lint: strict TypeScript")
	return runExternal(filepath.Join(root, "ui"), stdout, stderr, "npm", "run", "lint")
}

func runBuild(stdout, stderr io.Writer) error {
	root, err := projectRoot()
	if err != nil {
		return err
	}
	if !dirExists(filepath.Join(root, "ui", "node_modules")) {
		return errors.New("ui/node_modules is missing; run install first")
	}
	binDir := filepath.Join(root, "bin")
	if err := os.MkdirAll(binDir, 0o755); err != nil {
		return err
	}
	fmt.Fprintln(stdout, "build: Go wrapper/server")
	if err := runExternal(root, stdout, stderr, "go", "build", "-o", filepath.Join(binDir, binaryName()), "."); err != nil {
		return err
	}
	fmt.Fprintln(stdout, "build: staged canonical assets and PWA")
	return runExternal(filepath.Join(root, "ui"), stdout, stderr, "npm", "run", "build")
}

func runTests(args []string, stdout, stderr io.Writer) error {
	flags := flag.NewFlagSet("test", flag.ContinueOnError)
	flags.SetOutput(stderr)
	filter := flags.String("filter", "", "run go, ui, all, or the real live planner acceptance test")
	if err := flags.Parse(args); err != nil {
		return err
	}
	if flags.NArg() != 0 {
		return errors.New("test accepts no positional arguments")
	}
	root, err := projectRoot()
	if err != nil {
		return err
	}
	switch strings.ToLower(strings.TrimSpace(*filter)) {
	case "", "all":
		fmt.Fprintln(stdout, "test: Go")
		if err := runExternalCommand(root, stdout, stderr, "go", "test", "./..."); err != nil {
			return err
		}
		if !dirExists(filepath.Join(root, "ui", "node_modules")) {
			return errors.New("ui/node_modules is missing; run install first")
		}
		fmt.Fprintln(stdout, "test: TypeScript")
		return runExternalCommand(filepath.Join(root, "ui"), stdout, stderr, "npm", "test")
	case "go":
		return runExternalCommand(root, stdout, stderr, "go", "test", "./...")
	case "ui", "typescript":
		if !dirExists(filepath.Join(root, "ui", "node_modules")) {
			return errors.New("ui/node_modules is missing; run install first")
		}
		return runExternalCommand(filepath.Join(root, "ui"), stdout, stderr, "npm", "test")
	case "live", "live-planner":
		fmt.Fprintln(
			stdout,
			"test: live Robot V4 Motion Server/Tesseract planner acceptance",
		)
		return runExternalCommand(
			root,
			stdout,
			stderr,
			"go",
			"test",
			"./internal/simulator",
			"-run",
			"^TestPCNativeBackendLiveCanonicalPlanner$",
			"-count=1",
			"-v",
			"-offline-live-planner",
		)
	default:
		return fmt.Errorf("--filter must be go, ui, all, or live; got %q", *filter)
	}
}

func runUI(args []string, stdout, stderr io.Writer) error {
	flags := flag.NewFlagSet("ui", flag.ContinueOnError)
	flags.SetOutput(stderr)
	host := flags.String("host", "127.0.0.1", "Vite host")
	port := flags.Int("port", 5189, "Vite port")
	if err := flags.Parse(args); err != nil {
		return err
	}
	if flags.NArg() != 0 || strings.TrimSpace(*host) == "" || *port < 1 || *port > 65535 {
		return errors.New("ui requires a valid --host and --port and accepts no positional arguments")
	}
	root, err := projectRoot()
	if err != nil {
		return err
	}
	uiDir := filepath.Join(root, "ui")
	if !dirExists(filepath.Join(uiDir, "node_modules")) {
		return errors.New("ui/node_modules is missing; run install first")
	}
	fmt.Fprintf(stdout, "ui: http://%s:%d%s\n", *host, *port, uiBasePath)
	return runExternal(uiDir, stdout, stderr, "npm", "run", "dev", "--", "--host", *host, "--port", fmt.Sprint(*port))
}

func runServe(args []string, stdout, stderr io.Writer) error {
	flags := flag.NewFlagSet("serve", flag.ContinueOnError)
	flags.SetOutput(stderr)
	listen := flags.String("listen", "127.0.0.1:8794", "HTTP listen address")
	dist := flags.String("dist", "", "built UI directory")
	enableLocalSimulator := flags.Bool(
		"enable-local-simulator",
		true,
		"start the required loopback-only canonical local simulator (enabled by default)",
	)
	programCatalogOrigin := flags.String(
		"program-catalog-origin",
		strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_PROGRAM_CATALOG_ORIGIN")),
		"server-only HTTP(S) origin for the Motion Server-owned Robot V4 program catalog",
	)
	enableConnectedPlanning := flags.Bool(
		"enable-connected-planning",
		false,
		"explicitly enable server-mediated planning on manifest-backed Robot V4 targets",
	)
	enableConnectedExecution := flags.Bool(
		"enable-connected-execution",
		false,
		"explicitly enable reviewed Robot V4 execution on manifest-authorized targets",
	)
	programStoreDir := flags.String(
		"program-store",
		strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_PROGRAM_STORE")),
		"absolute directory of the cell's shared program store (cell_programs.go); empty keeps programs on each client",
	)
	programStoreLabel := flags.String(
		"program-store-label",
		strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_PROGRAM_STORE_LABEL")),
		"the host name clients show for the program store (\"synced with <host>\"); empty means this host's name",
	)
	executionStateDir := flags.String(
		"execution-state-dir",
		strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_EXECUTION_STATE_DIR")),
		"absolute directory for durable execution audit and idempotency state",
	)
	// The authoring origin went with the authoring server. What is left is a
	// single origin, and it is server-only: `denseexec` fetches the dense
	// trajectory blob from it, and no browser ever learns it.
	weldPlannerMotionOrigin := flags.String(
		"weld-planner-motion-origin",
		strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN")),
		"server-only HTTP(S) origin for the weld planner motion server",
	)
	daemonStateURL := flags.String(
		"daemon-state-url",
		strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_DAEMON_STATE_URL")),
		"server-only HTTP(S) URL for exact daemon-v1 service resolution",
	)
	// Unverified repository default; native execution still requires an explicit binding.
	executionBackend := flags.String("execution-backend", envOrDefault("OFFLINE_PROGRAMMING_EXECUTION_BACKEND", "rt_core"), "execution backend: rt_core")
	rtCoreConfig := flags.String("rt-core-config", strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_RT_CORE_CONFIG")), "server-only JSON configuration for the public rt_core backend")
	enableTestPrograms := flags.Bool("enable-test-programs", os.Getenv("OFFLINE_PROGRAMMING_TEST_PROGRAMS") == "1",
		"accept uploaded tool-generated dense test programs (.rdt) for Load and Play (off unless OFFLINE_PROGRAMMING_TEST_PROGRAMS=1)")
	if err := flags.Parse(args); err != nil {
		return err
	}
	if flags.NArg() != 0 || strings.TrimSpace(*listen) == "" {
		return errors.New("serve accepts no positional arguments and requires --listen")
	}
	if err := simulator.ValidateJogBackend(os.Getenv("OFFLINE_PROGRAMMING_JOG_BACKEND")); err != nil {
		return err
	}
	cellsPath := strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_CELLS"))
	validationConfig := *rtCoreConfig
	if cellsPath != "" {
		validationConfig = cellsPath
	}
	if err := validateExecutionBackend(*executionBackend, validationConfig, *enableConnectedPlanning || *enableConnectedExecution); err != nil {
		return err
	}
	root, err := projectRoot()
	if err != nil {
		return err
	}
	distDir := strings.TrimSpace(*dist)
	if distDir == "" {
		distDir = filepath.Join(root, "ui", "dist")
	} else if !filepath.IsAbs(distDir) {
		distDir = filepath.Join(root, distDir)
	}
	if !fileExists(filepath.Join(distDir, "index.html")) {
		return fmt.Errorf("built UI not found at %s; run build first", distDir)
	}
	repo, err := repositoryRoot(root)
	if err != nil {
		return err
	}
	// The robot models this server knows are the descriptions in this tree.
	robotmodel.SetRoot(repo)
	var cells *targets.CellCatalogue
	if cellsPath != "" {
		if *executionBackend != "rt_core" {
			return errors.New("cell_backend_required: rt_core")
		}
		catalogue, err := targets.ReadCellCatalogue(cellsPath, repo)
		if err != nil {
			return err
		}
		cells = &catalogue
	}
	worker, err := cad.NewWorker(cad.DefaultConfig(repo))
	if err != nil {
		return fmt.Errorf("initialize bounded CadQuery worker: %w", err)
	}
	// The rails half. It runs in `weld_planner/v1` so the worker resolves the
	// pixi environment its OCCT imports expect; a checkout where that has not
	// been provisioned answers `unavailable` rather than failing to start,
	// because classic authoring does not need it.
	seams, err := seam.New(seam.Config{
		WeldPlannerRoot: filepath.Join(repo, "weld_planner", "v1"),
	})
	if err != nil {
		return fmt.Errorf("initialize bounded seam worker: %w", err)
	}
	// Spares start now, so the first click is warm too, and end on the way out.
	seams.Warm()
	defer seams.Close()
	loopbackHost, loopback := loopbackListenHost(*listen)
	if *enableLocalSimulator && !loopback {
		return errors.New("--enable-local-simulator requires an explicit loopback --listen address")
	}
	if loopbackHost == "" {
		loopbackHost = "127.0.0.1"
	}
	topology := simulator.NewPCTopology(
		"local-simulator",
		planning.RobotModelRosie1400V3,
		loopbackHost,
	)
	backend := simulator.NewPCNativeBackend(simulator.PCBackendConfig{
		Enabled:   *enableLocalSimulator,
		RepoRoot:  repo,
		RunBinary: os.Getenv("OFFLINE_PROGRAMMING_JOG_BINARY"),
		ProofTTL:  5 * time.Second,
	})
	jogManager := simulator.NewJogManager(repo)
	localSimulator, err := simulator.NewService(simulator.Options{
		ServiceID:             "offline-programming-local",
		Topology:              topology,
		Limits:                simulator.DefaultResourceLimits(),
		ProofTTL:              5 * time.Second,
		PlannerRequestSubject: "",
	}, backend)
	if err != nil {
		return fmt.Errorf("initialize local simulator service: %w", err)
	}
	var programCatalog catalog.Gateway
	if origin := strings.TrimSpace(*programCatalogOrigin); origin != "" {
		programCatalog, err = catalog.NewHTTPGateway(origin, nil)
		if err != nil {
			return fmt.Errorf("initialize Motion Server program catalog: %w", err)
		}
	}
	connectedTargets := LoadConnectedRegistry(repo)
	var targetPlanning *targetplanning.Service
	var resolver *targetplanning.DaemonNATSResolver
	if *enableConnectedPlanning || *enableConnectedExecution {
		resolver, err = targetplanning.NewDaemonNATSResolver(
			targetplanning.DaemonNATSResolverOptions{
				StateURL: *daemonStateURL,
			},
		)
		if err != nil {
			return fmt.Errorf(
				"initialize connected Robot V4 service resolver: %w",
				err,
			)
		}
	}
	if *enableConnectedPlanning {
		if programCatalog == nil {
			return errors.New(
				"--enable-connected-planning requires --program-catalog-origin",
			)
		}
		programAuthority, authorityErr :=
			targetplanning.NewCatalogProgramAuthority(programCatalog)
		if authorityErr != nil {
			return fmt.Errorf(
				"initialize connected planning program authority: %w",
				authorityErr,
			)
		}
		adapter, adapterErr := targetplanning.NewNATSAdapter(
			targetplanning.NATSAdapterOptions{Resolver: resolver},
		)
		if adapterErr != nil {
			return fmt.Errorf(
				"initialize connected planning adapter: %w",
				adapterErr,
			)
		}
		targetPlanning = targetplanning.NewService(targetplanning.Options{
			Targets:  connectedTargets,
			Programs: programAuthority,
			Adapter:  adapter,
		})
	}

	var executionGateway *execution.Gateway
	if *enableConnectedExecution {
		if !*enableConnectedPlanning {
			return errors.New(
				"--enable-connected-execution requires --enable-connected-planning",
			)
		}
		stateDir := strings.TrimSpace(*executionStateDir)
		if stateDir == "" || !filepath.IsAbs(stateDir) {
			return errors.New(
				"--enable-connected-execution requires an absolute --execution-state-dir",
			)
		}
		allowlist, allowlistErr := connectedExecutionAllowlist(connectedTargets)
		if allowlistErr != nil {
			return allowlistErr
		}
		controller, controllerErr := execution.NewNATSController(
			execution.NATSControllerOptions{
				Targets:  connectedTargets,
				Resolver: resolver,
			},
		)
		if controllerErr != nil {
			return fmt.Errorf(
				"initialize connected execution controller: %w",
				controllerErr,
			)
		}
		audit, auditErr := execution.NewJSONLAuditSink(
			filepath.Join(stateDir, "execution-audit.jsonl"),
		)
		if auditErr != nil {
			return fmt.Errorf("initialize execution audit: %w", auditErr)
		}
		idempotency, idempotencyErr := execution.NewFileIdempotencyStore(
			filepath.Join(stateDir, "execution-idempotency.json"),
		)
		if idempotencyErr != nil {
			return fmt.Errorf(
				"initialize execution idempotency state: %w",
				idempotencyErr,
			)
		}
		bootstrap, bootstrapErr := newExecutionBootstrap()
		if bootstrapErr != nil {
			return fmt.Errorf(
				"initialize execution controller identity: %w",
				bootstrapErr,
			)
		}
		executionGateway = execution.NewGateway(execution.GatewayConfig{
			Bootstrap: bootstrap,
			Features: execution.FeatureGates{
				GatewayEnabled:            true,
				ConnectedExecutionEnabled: true,
				ServerControllerAuthority: true,
				SessionMonitoringEnabled:  true,
				PlanPreloadEnabled:        true,
				ControlledStartArmEnabled: true,
				PlanPlayEnabled:           true,
				ProtectiveTailEnabled:     true,
				AuditRequired:             true,
			},
			Allowlist:   allowlist,
			Evidence:    connectedTargets,
			Observer:    connectedTargets,
			Transport:   controller,
			Authority:   controller,
			Audit:       audit,
			Idempotency: idempotency,
		})
		if !executionGateway.Capabilities().InfrastructureReady {
			return errors.New(
				"connected execution infrastructure failed closed during initialization",
			)
		}
	}

	var connectedMonitor *ConnectedNATSMonitor
	if resolver != nil {
		connectedMonitor, err = NewConnectedNATSMonitor(
			connectedTargets,
			resolver,
		)
		if err != nil {
			return fmt.Errorf(
				"initialize connected observation monitor: %w",
				err,
			)
		}
		connectedMonitor.Start(context.Background())
		defer connectedMonitor.Close()
	}
	signalContext, stopSignals := signal.NotifyContext(
		context.Background(),
		os.Interrupt,
		syscall.SIGTERM,
	)
	defer stopSignals()
	defer func() {
		stopContext, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()
		_, _ = localSimulator.Stop(stopContext)
		if jogManager != nil {
			jogManager.Stop()
		}
	}()
	defer func() {
		if executionGateway != nil {
			stopContext, cancel := context.WithTimeout(
				context.Background(),
				20*time.Second,
			)
			defer cancel()
			_ = executionGateway.Close(stopContext)
		}
	}()
	if *enableLocalSimulator {
		fmt.Fprintln(stdout, "local simulation: starting canonical Motion Server/Tesseract runtime")
		go func() {
			startContext, cancel := context.WithTimeout(
				signalContext,
				simulator.DefaultResourceLimits().PlanTimeout+30*time.Second,
			)
			defer cancel()
			status, startErr := localSimulator.Start(startContext)
			if startErr != nil {
				fmt.Fprintf(
					stderr,
					"local simulation: startup failed: %v\n",
					startErr,
				)
				return
			}
			if !status.Readiness.Ready {
				fmt.Fprintln(stderr, "local simulation: startup finished without ready proof")
				return
			}
			fmt.Fprintln(stdout, "local simulation: ready · Motion Server/Tesseract · no physical output")
		}()
		go func() {
			fmt.Fprintln(stdout, "local jog: starting isolated Motion Server cartesiand against local simulation")
			state, startErr := jogManager.Start()
			if startErr != nil {
				fmt.Fprintf(stderr, "local jog: startup failed: %v\n", startErr)
				return
			}
			fmt.Fprintf(
				stdout,
				"local jog: ready · isolated cartesiand · %s · no physical output\n",
				state.DaemonAddress,
			)
		}()
	}
	var testPrograms *denseexec.TestPrograms
	if *enableTestPrograms {
		testPrograms = denseexec.NewTestPrograms()
		fmt.Fprintln(stdout, "test programs: ON (uploaded .rdt programs load and play through dense execution after the cell's checks)")
	} else {
		fmt.Fprintln(stdout, "test programs: off (OFFLINE_PROGRAMMING_TEST_PROGRAMS=1 turns them on)")
	}
	denseExecution, err := newDenseExecution(denseExecutionFlags{
		Cells:        cells,
		Backend:      *executionBackend,
		RTCoreConfig: *rtCoreConfig,
		MotionOrigin: *weldPlannerMotionOrigin,
		TestPrograms: testPrograms,
	}, stdout)
	if err != nil {
		return fmt.Errorf("initialize dense execution: %w", err)
	}
	defer func() {
		closeContext, cancel := context.WithTimeout(context.Background(), 15*time.Second)
		defer cancel()
		denseExecution.Close(closeContext)
	}()
	if *executionBackend == "rt_core" && os.Getenv("OFFLINE_PROGRAMMING_JOG_BACKEND") != "rt_core" {
		jogManager = nil
	}
	maintenance, err := maintenanceSourceFromEnv()
	if err != nil {
		return err
	}
	var fixedRTCore *denseexec.RTCoreConfig
	if _, picker := denseExecution.(*targets.Picker); !picker && *rtCoreConfig != "" {
		raw, err := os.ReadFile(*rtCoreConfig)
		if err != nil {
			return err
		}
		cfg, err := denseexec.DecodeRTCoreConfig(raw)
		if err != nil {
			return err
		}
		fixedRTCore = &cfg
	}
	cellMaintenance := cellMaintenanceOf(denseExecution, fixedRTCore)
	defer cellMaintenance.Close()
	if view := maintenance.Current(); view.Active {
		fmt.Fprintf(stdout, "maintenance: notice ACTIVE at %s (%s); robot actions are refused until it is cleared\n", maintenance.Path(), view.Reason)
	} else {
		fmt.Fprintf(stdout, "maintenance: notice file %s (none active)\n", maintenance.Path())
	}
	if cellMaintenance != nil {
		fmt.Fprintln(stdout, "maintenance: also reading the selected cell's notice from its rt-control (GET /v1/maintenance)")
	}
	discoveryConfig := cellDiscoveryConfigFromEnv()
	discoveryConfig.Listen, discoveryConfig.Repo, discoveryConfig.Catalogue, discoveryConfig.Maintenance = *listen, repo, cells, maintenance
	discoveryConfig.Picker, _ = denseExecution.(*targets.Picker)
	discoveryState := newCellDiscovery(discoveryConfig, stdout)
	defer discoveryState.Close()
	wristCamera, err := NewCellCamera(os.Getenv(WristCameraOriginEnv))
	if err != nil {
		return fmt.Errorf("wrist camera: %w", err)
	}
	cellCamera, err := NewCellCamera(os.Getenv(CellCameraOriginEnv))
	if err != nil {
		return err
	}
	if cellCamera != nil {
		fmt.Fprintf(stdout, "cell camera: relaying %s (read-only frames and status)\n", cellCamera.Origin())
	}
	cellExtraCameras, err := NewCellExtraCameras(os.Getenv(CellExtraCamerasEnv))
	if err != nil {
		return err
	}
	// An OLP unit rendered before the camera list names the wrist camera on
	// its own: it is the list's "wrist" camera.
	if wristCamera != nil && !slices.ContainsFunc(cellExtraCameras, func(c NamedCellCamera) bool { return c.ID == wristCameraID }) {
		cellExtraCameras = append(cellExtraCameras, NamedCellCamera{ID: wristCameraID, Camera: wristCamera})
	}
	for _, extra := range cellExtraCameras {
		fmt.Fprintf(stdout, "cell camera %s: relaying %s (read-only frames and status)\n", extra.ID, extra.Camera.Origin())
	}
	cellVideo, err := NewCellVideo(os.Getenv(CellVideoOriginEnv))
	if err != nil {
		return err
	}
	if cellVideo != nil {
		fmt.Fprintf(stdout, "cell camera video: relaying WebRTC set-up to %s (the video itself goes from that sender over UDP)\n", cellVideo.origin)
	}
	var programStore *programstore.Store
	if dir := strings.TrimSpace(*programStoreDir); dir != "" {
		programStore, err = programstore.Open(dir)
		if err != nil {
			return err
		}
		defer programStore.Close()
		fmt.Fprintf(stdout, "cell programs: store %s (id %s, %d programs)\n", dir, programStore.ID(), len(programStore.Snapshot().Programs))
	} else {
		fmt.Fprintln(stdout, "cell programs: no store (--program-store); programs stay on each client")
	}
	server := NewServer(ServerOptions{
		DistDir:          distDir,
		DenseExecution:   denseExecution,
		Worker:           worker,
		Seams:            seams,
		MotionOrigin:     *weldPlannerMotionOrigin,
		RepoRoot:         repo,
		LocalSimulator:   localSimulator,
		JogManager:       jogManager,
		ConnectedTargets: connectedTargets,
		ExecutionGateway: executionGateway,
		ProgramCatalog:   programCatalog,
		TargetPlanning:   targetPlanning,
		Maintenance:      maintenance,
		CellMaintenance:  cellMaintenance,
		Discovery:        discoveryState,
		TestPrograms:     testPrograms,
		CellCamera:       cellCamera,
		CellExtraCameras: cellExtraCameras,
		CellVideo:        cellVideo,
		ProgramStore:     programStore,
		StoreLabel:       *programStoreLabel,
	})
	httpServer := &http.Server{
		Addr:              *listen,
		Handler:           server.Handler(),
		ReadHeaderTimeout: 5 * time.Second,
		ReadTimeout:       20 * time.Second,
		// Local canonical planning remains request-bounded even though the
		// Motion Server and collision world stay warm for the app session.
		WriteTimeout: 5 * time.Minute,
		IdleTimeout:  60 * time.Second,
	}
	fmt.Fprintf(stdout, "offline-programming v1 listening on http://%s%s\n", *listen, uiBasePath)
	serveResult := make(chan error, 1)
	go func() {
		serveResult <- httpServer.ListenAndServe()
	}()
	select {
	case err = <-serveResult:
		if errors.Is(err, http.ErrServerClosed) {
			return nil
		}
		return err
	case <-signalContext.Done():
		shutdownContext, cancel := context.WithTimeout(
			context.Background(),
			20*time.Second,
		)
		shutdownErr := httpServer.Shutdown(shutdownContext)
		cancel()
		serveErr := <-serveResult
		if errors.Is(serveErr, http.ErrServerClosed) {
			serveErr = nil
		}
		return errors.Join(shutdownErr, serveErr)
	}
}

func runDeploy(args []string, stdout, stderr io.Writer) error {
	flags := flag.NewFlagSet("deploy", flag.ContinueOnError)
	flags.SetOutput(stderr)
	if err := flags.Parse(args); err != nil {
		return err
	}
	if flags.NArg() != 0 {
		return errors.New("deploy accepts no positional targets; use Rosie --host")
	}
	root, err := projectRoot()
	if err != nil {
		return err
	}
	if err := runBuild(stdout, stderr); err != nil {
		return err
	}
	target := filepath.Join(root, "deploy", "local")
	if err := os.RemoveAll(target); err != nil {
		return err
	}
	if err := copyFile(filepath.Join(root, "bin", binaryName()), filepath.Join(target, "bin", binaryName())); err != nil {
		return err
	}
	if err := copyDir(filepath.Join(root, "ui", "dist"), filepath.Join(target, "ui")); err != nil {
		return err
	}
	manifest := map[string]any{
		"name":           "offline-programming-v1",
		"built_at_utc":   time.Now().UTC().Format(time.RFC3339),
		"binary":         filepath.ToSlash(filepath.Join("bin", binaryName())),
		"ui_entrypoint":  filepath.ToSlash(filepath.Join("ui", "index.html")),
		"default_mode":   "offline_preview",
		"real_execution": false,
	}
	raw, err := json.MarshalIndent(manifest, "", "  ")
	if err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(target, "manifest.json"), append(raw, '\n'), 0o644); err != nil {
		return err
	}
	fmt.Fprintf(stdout, "deploy: local bundle ready at %s\n", target)
	return nil
}

func projectRoot() (string, error) {
	if repo := strings.TrimSpace(os.Getenv("ROSIE_REPO")); repo != "" {
		candidate := filepath.Join(repo, "offline-programming", "v1")
		if fileExists(filepath.Join(candidate, "go.mod")) {
			return candidate, nil
		}
	}
	wd, err := os.Getwd()
	if err != nil {
		return "", err
	}
	for {
		if fileExists(filepath.Join(wd, "go.mod")) && dirExists(filepath.Join(wd, "ui")) &&
			filepath.Base(wd) == "v1" && filepath.Base(filepath.Dir(wd)) == "offline-programming" {
			return wd, nil
		}
		candidate := filepath.Join(wd, "offline-programming", "v1")
		if fileExists(filepath.Join(candidate, "go.mod")) {
			return candidate, nil
		}
		parent := filepath.Dir(wd)
		if parent == wd {
			return "", fmt.Errorf("could not find offline-programming/v1 from %s", wd)
		}
		wd = parent
	}
}

func repositoryRoot(productRoot string) (string, error) {
	repo := filepath.Clean(filepath.Join(productRoot, "..", ".."))
	hasCadQuery := dirExists(filepath.Join(repo, "cadquery", "v1"))
	hasGradientMod := fileExists(filepath.Join(repo, "go.mod"))
	hasTesseract := dirExists(filepath.Join(repo, "tesseract", "v1"))
	hasMotionServer := dirExists(filepath.Join(repo, "motion-server", "v1"))
	// The predecessor monorepo had a root go.mod; RosieOS is a split checkout without one.
	if !hasCadQuery || (!hasGradientMod && !hasTesseract) {
		return "", fmt.Errorf("repository root not found above %s", productRoot)
	}
	if !hasMotionServer {
		return "", fmt.Errorf(
			"repository root %s is missing motion-server/v1 (canonical Motion Server is at repo root, not robot/v4/motion-server)",
			repo,
		)
	}
	return repo, nil
}

func runExternal(dir string, stdout, stderr io.Writer, name string, args ...string) error {
	cmd := exec.Command(name, args...)
	cmd.Dir = dir
	cmd.Stdout = stdout
	cmd.Stderr = stderr
	cmd.Stdin = os.Stdin
	return cmd.Run()
}

func removeGeneratedPath(root, path string) error {
	rel, err := filepath.Rel(root, path)
	if err != nil || rel == "." || rel == ".." || strings.HasPrefix(rel, ".."+string(filepath.Separator)) {
		return fmt.Errorf("refusing to clean path outside product root: %s", path)
	}
	return os.RemoveAll(path)
}

func copyFile(src, dst string) error {
	if err := os.MkdirAll(filepath.Dir(dst), 0o755); err != nil {
		return err
	}
	in, err := os.Open(src)
	if err != nil {
		return err
	}
	defer in.Close()
	info, err := in.Stat()
	if err != nil {
		return err
	}
	out, err := os.OpenFile(dst, os.O_CREATE|os.O_TRUNC|os.O_WRONLY, info.Mode().Perm())
	if err != nil {
		return err
	}
	_, copyErr := io.Copy(out, in)
	closeErr := out.Close()
	if copyErr != nil {
		return copyErr
	}
	return closeErr
}

func copyDir(src, dst string) error {
	return filepath.WalkDir(src, func(path string, entry fs.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		rel, err := filepath.Rel(src, path)
		if err != nil {
			return err
		}
		target := filepath.Join(dst, rel)
		if entry.IsDir() {
			return os.MkdirAll(target, 0o755)
		}
		return copyFile(path, target)
	})
}

func binaryName() string {
	if runtime.GOOS == "windows" {
		return "offline-programming-v1.exe"
	}
	return "offline-programming-v1"
}

func isHelp(value string) bool {
	return value == "help" || value == "-h" || value == "--help"
}

func loopbackListenHost(value string) (string, bool) {
	host, _, err := net.SplitHostPort(strings.TrimSpace(value))
	if err != nil {
		return "", false
	}
	host = strings.Trim(strings.TrimSpace(host), "[]")
	if strings.EqualFold(host, "localhost") {
		return "localhost", true
	}
	address := net.ParseIP(host)
	return host, address != nil && address.IsLoopback()
}

func connectedExecutionAllowlist(
	registry *ConnectedRegistry,
) (*execution.StaticManifestAllowlist, error) {
	if registry == nil {
		return nil, errors.New(
			"connected execution target registry is unavailable",
		)
	}
	entries := []execution.ManifestAllowance{}
	for _, descriptor := range registry.Discovery().Targets {
		target, ok := registry.PlanningTarget(descriptor.ProfileID)
		if !ok ||
			!containsExact(target.Binding.LeaderControllerRoles,
				"offline-programming") {
			continue
		}
		identity := execution.TargetRevisionIdentity{
			ProfileID:        target.Discovery.ProfileID,
			ManifestRef:      target.Discovery.ManifestRef,
			CellID:           target.Discovery.CellID,
			ManifestRevision: target.Binding.ManifestRevision,
		}
		for _, model := range []planning.RobotModel{
			planning.RobotModelRosie1400V3,
			planning.RobotModelRosie1420V1,
		} {
			entries = append(entries, execution.ManifestAllowance{
				Target:     identity,
				RobotModel: model,
			})
		}
	}
	if len(entries) == 0 {
		return nil, errors.New(
			"no Robot V4 manifest explicitly authorizes offline-programming execution",
		)
	}
	return execution.NewStaticManifestAllowlist(entries)
}

// denseExecutionFlags is the dense execution target as the flags name it.
type denseExecutionFlags struct {
	Cells        *targets.CellCatalogue
	Backend      string
	RTCoreConfig string
	MotionOrigin string
	TestPrograms *denseexec.TestPrograms
}

// newDenseExecution builds the dense execution manager and prints one
// `dense execution:` line either way, the same habit as the `rails:` line:
// a target configured by two of its three variables looks exactly like an
// unconfigured one from the UI (every action answers 503), and the
// startup line is where that shows.
func newDenseExecution(flags denseExecutionFlags, stdout io.Writer) (denseexec.Backend, error) {
	if flags.Backend != "" && flags.Backend != "rt_core" {
		return nil, validateExecutionBackend(flags.Backend, flags.RTCoreConfig, false)
	}
	if flags.Cells != nil {
		var local denseexec.RTCoreConfig
		if flags.RTCoreConfig != "" {
			raw, err := os.ReadFile(flags.RTCoreConfig)
			if err != nil {
				return nil, err
			}
			local, err = denseexec.DecodeRTCoreConfig(raw)
			if err != nil {
				return nil, err
			}
		}
		instance, err := randomLowerHex(6)
		if err != nil {
			return nil, err
		}
		fmt.Fprintln(stdout, "dense execution: choose a machine; joint and Cartesian jog share its armed control session")
		return targets.NewPicker(targets.PickerOptions{Catalogue: *flags.Cells, Local: local, Source: denseexec.Config{ControllerID: "offline-programming:server-" + instance, MotionOrigin: flags.MotionOrigin, TestPrograms: flags.TestPrograms}}), nil
	}
	if flags.Backend == "" {
		flags.Backend = "rt_core"
	}
	if err := validateExecutionBackend(flags.Backend, flags.RTCoreConfig, false); err != nil {
		return nil, err
	}
	instance, err := randomLowerHex(6)
	if err != nil {
		return nil, err
	}
	if flags.Backend == "rt_core" {
		raw, err := os.ReadFile(flags.RTCoreConfig)
		if err != nil {
			return nil, err
		}
		cfg, err := denseexec.DecodeRTCoreConfig(raw)
		if err != nil {
			return nil, err
		}
		controller, err := denseexec.NewRTCore(cfg, denseexec.Config{ControllerID: "offline-programming:server-" + instance, MotionOrigin: flags.MotionOrigin, TestPrograms: flags.TestPrograms}, nil)
		if err == nil {
			fmt.Fprintln(stdout, "dense execution: rt_core public API selected")
		}
		return controller, err
	}
	return nil, &denseexec.Error{Status: 400, Code: "backend_retired", Detail: "select rt_core"}
}

func envOrDefault(name, fallback string) string {
	if value := strings.TrimSpace(os.Getenv(name)); value != "" {
		return value
	}
	return fallback
}

// validateExecutionBackend prevents the selected public route from activating
// legacy connected commands or falling back when configuration is incomplete.
func validateExecutionBackend(backend, config string, connected bool) error {
	switch backend {
	case "legacy_v4":
		return &denseexec.Error{Status: 400, Code: "backend_retired", Detail: "select rt_core"}
	case "rt_core":
		if config == "" {
			return errors.New("rt_core requires --rt-core-config")
		}
		if connected {
			return errors.New("rt_core does not support the legacy connected planning/execution gateway")
		}
	default:
		return &denseexec.Error{Status: 400, Code: "backend_retired", Detail: fmt.Sprintf("unsupported execution backend %q; select rt_core", backend)}
	}
	return nil
}

func newExecutionBootstrap() (execution.ControllerBootstrap, error) {
	instance, err := randomLowerHex(6)
	if err != nil {
		return execution.ControllerBootstrap{}, err
	}
	bootID, err := randomLowerHex(12)
	if err != nil {
		return execution.ControllerBootstrap{}, err
	}
	prefix, err := randomLowerHex(4)
	if err != nil {
		return execution.ControllerBootstrap{}, err
	}
	now := time.Now().UTC()
	bootstrap := execution.ControllerBootstrap{
		Controller: targets.ControllerIdentity{
			ControllerID:     "offline-programming:server-" + instance,
			ControllerBootID: "op-" + bootID,
		},
		CommandIDPrefix:        "opg-" + prefix,
		BootstrappedAt:         now,
		ReviewTTL:              10 * time.Second,
		LeaseTTL:               2 * time.Second,
		MaxPlanEvidenceAge:     10 * time.Minute,
		StatusFreshnessCeiling: 30 * time.Second,
	}
	if err := bootstrap.Validate(); err != nil {
		return execution.ControllerBootstrap{}, err
	}
	return bootstrap, nil
}

func randomLowerHex(byteCount int) (string, error) {
	if byteCount < 1 || byteCount > 24 {
		return "", errors.New("random execution identity size is invalid")
	}
	raw := make([]byte, byteCount)
	if _, err := rand.Read(raw); err != nil {
		return "", err
	}
	return hex.EncodeToString(raw), nil
}

func containsExact(values []string, wanted string) bool {
	for _, value := range values {
		if value == wanted {
			return true
		}
	}
	return false
}

func fileExists(path string) bool {
	info, err := os.Stat(path)
	return err == nil && info.Mode().IsRegular()
}

func dirExists(path string) bool {
	info, err := os.Stat(path)
	return err == nil && info.IsDir()
}
