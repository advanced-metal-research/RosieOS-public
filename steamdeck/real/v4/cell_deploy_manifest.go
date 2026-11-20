package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"net"
	"net/url"
	"os"
	"path"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
)

// The cell manifest's top-level "cell_deploy" block holds the relay-host facts
// that the build, OLP and servo-host maintenance commands need beyond the
// rt_core binding and the connections map. Until 2026-10-07 these lived only
// in hand-made scripts under ~/.rosie/deployments/rosieos-<robot>-*/ on the relay,
// one edited copy per deployment with hosts, users, link-local addresses,
// paths, ports and unit names inlined. A cell that lacks the block cannot use
// these commands; nothing falls back to one arm's values.
//
// Everything else is derived from what the manifest already holds:
//   - relay (build host, deployment tree, OLP host, dump copy target): the
//     via_peer of the realtime-core node's connection, its ssh_user and host;
//   - servo host: that connection's user@host as the relay reaches it;
//   - cell id: the manifest file name; execution address: rt_core.remote_listen;
//   - configuration pin: rt_core.configuration_sha256;
//   - EtherCAT slave count: the machine config's axes when bus.unknown_slaves
//     refuses any slave that is not an axis (else expected_ethercat_slaves);
//   - robot model: the machine config's robot_description directory name.

type cellDeployConfig struct {
	DeploymentsDir   string `json:"deployments_dir"`
	DeploymentPrefix string `json:"deployment_prefix"`
	// ReleasePlanOnly permits release --plan and --dry-run, but refuses apply.
	// It does not constrain separate commands or direct host access.
	ReleasePlanOnly bool     `json:"release_plan_only,omitempty"`
	ScratchDir      string   `json:"scratch_dir"`
	DumpDir         string   `json:"dump_dir"`
	Home            string   `json:"home"`
	Path            []string `json:"path"`
	IgHDepsDir      string   `json:"igh_deps_dir"`
	BuildJobs       int      `json:"build_jobs"`
	DumpCopyRate    int64    `json:"dump_copy_rate_bytes_per_s"`
	// ExpectedSlaves is required only when the machine config holds unknown
	// slaves (a bench with spare drives); with bus.unknown_slaves "refuse" the
	// slaves are exactly the axes and a different value is refused.
	ExpectedSlaves int           `json:"expected_ethercat_slaves,omitempty"`
	OLP            cellDeployOLP `json:"olp"`
	// DCLock bounds every DC slave's ESC 0x092C (system time difference) in
	// verify (cell_deploy_dc_lock.go). Absent: verify refuses a DC machine.
	DCLock *cellDeployDCLock `json:"dc_lock,omitempty"`
	// Camera is the cell camera service this manifest installs on the relay
	// (cell_deploy_camera.go; `camera install`). Absent: none.
	Camera *cellDeployCamera `json:"camera,omitempty"`
	// ExtraCameras are further cameras of the same cell on the relay, other
	// angles beside Camera: each its own service instance, device and port.
	// The OLP lists them after the cell camera, by instance. Needs Camera.
	ExtraCameras []cellDeployCamera `json:"extra_cameras,omitempty"`
}

type cellDeployOLP struct {
	Unit           string `json:"unit"`
	Description    string `json:"description"`
	Listen         string `json:"listen"`
	PlannerUnit    string `json:"planner_unit"`
	PlannerOrigin  string `json:"planner_origin"`
	PlannerPixiEnv string `json:"planner_pixi_env"`
	// Planner hands the planner unit and its env to the deploy commands
	// (cell_deploy_planner.go). Absent: the unit is provisioned by hand and
	// olp install only requires it to be active.
	Planner           *cellDeployOLPPlanner `json:"planner,omitempty"`
	ControlTunnelUnit string                `json:"control_tunnel_unit,omitempty"`
	// DiscoveryInterfaces are the operator-LAN interfaces the OLP advertises
	// on and searches for other OLP servers (Cells → Search local network,
	// OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES). Absent: discovery off. Never
	// the link to the servo host: nothing may be multicast onto it.
	DiscoveryInterfaces []string `json:"discovery_interfaces,omitempty"`
	// DiscoveryTailnet serves the OLP's advertisement (and nothing else) on
	// the host's tailnet address at listen_port, and makes the OLP's Cells
	// search ask tailnet peers on that port
	// (OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT). Absent: no tailnet listener.
	DiscoveryTailnet *cellDeployOLPTailnet `json:"discovery_tailnet,omitempty"`
	// TestPrograms lets operators load tool-generated dense test programs
	// (.rdt, tuningreport inertia-plan) through the OLP's dense Load and Play
	// after the cell's checks (OFFLINE_PROGRAMMING_TEST_PROGRAMS=1). Absent or
	// false: the OLP refuses every upload.
	TestPrograms bool `json:"test_programs,omitempty"`
	// CameraOrigin is a camera service on the relay that this manifest does
	// not install, which the OLP relays to its clients as the cell camera, for
	// example http://127.0.0.1:8088/camera
	// (OFFLINE_PROGRAMMING_CELL_CAMERA_ORIGIN; offline-programming/v1
	// cell_camera.go). A cell whose camera is installed from here names it in
	// cell_deploy.camera instead, and the origin follows from that block. It
	// must be on the relay itself: the OLP fetches what this names.
	CameraOrigin string `json:"camera_origin,omitempty"`
	// ProgramStore is the cell's shared program store on the relay
	// (cell_deploy_program_store.go). Absent: programs stay on each client.
	ProgramStore *cellDeployOLPProgramStore `json:"program_store,omitempty"`
	// UnattendedLoopAllowed lets this cell's loops (Run N times, Loop until
	// stopped) continue without any client's heartbeat (owner decision
	// 2026-10-11 23:32 PDT). The OLP reads it from the manifest in its own
	// tree (offline-programming/v1 internal/targets/cells.go), so it reaches
	// the cell with an olp install; nothing here renders it. Absent: false.
	UnattendedLoopAllowed bool `json:"unattended_loop_allowed,omitempty"`
	// Replaces are services on the relay, not from this repository, whose job
	// the OLP now does (one cell: the hand-installed servo dashboard, replaced by
	// the drive health page). olp install stops and disables them only after
	// the new OLP passed its checks, so a refused or restored install leaves
	// them as they were; they stay installed as the rollback. olp status
	// reports their state.
	Replaces []string `json:"replaces,omitempty"`
}

type cellDeployOLPTailnet struct {
	ListenPort int `json:"listen_port"`
}

// cellDeployInterface is a Linux interface name (IFNAMSIZ 16 with the NUL).
var cellDeployInterface = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]{0,14}$`)

var (
	cellDeployAbsPath = regexp.MustCompile(`^(/[A-Za-z0-9._@+-]+)+$`)
	cellDeployToken   = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]*$`)
	cellDeployUnit    = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9@._-]*\.service$`)
)

// cellDeployReleaseToken is replaced by the deployment's short commit in the
// OLP unit description.
const cellDeployReleaseToken = "{release}"

func cellDeployCleanAbs(field, value string) error {
	if !cellDeployAbsPath.MatchString(value) || path.Clean(value) != value || strings.Contains(value, "/../") || strings.HasSuffix(value, "/..") {
		return fmt.Errorf("cell_deploy %s must be an absolute normalized path of [A-Za-z0-9._@+-] segments, got %q", field, value)
	}
	return nil
}

func (c cellDeployConfig) validate() error {
	for field, value := range map[string]string{"deployments_dir": c.DeploymentsDir, "scratch_dir": c.ScratchDir, "dump_dir": c.DumpDir, "home": c.Home, "igh_deps_dir": c.IgHDepsDir} {
		if err := cellDeployCleanAbs(field, value); err != nil {
			return err
		}
	}
	if !cellDeployToken.MatchString(c.DeploymentPrefix) {
		return fmt.Errorf("cell_deploy deployment_prefix must be a file-name token, got %q", c.DeploymentPrefix)
	}
	if len(c.Path) == 0 {
		return fmt.Errorf("cell_deploy path must list the relay's toolchain directories (go, node, pixi) and the system bin directories")
	}
	for i, p := range c.Path {
		if err := cellDeployCleanAbs(fmt.Sprintf("path[%d]", i), p); err != nil {
			return err
		}
	}
	if c.ExpectedSlaves < 0 || c.ExpectedSlaves > 64 {
		return fmt.Errorf("cell_deploy expected_ethercat_slaves must be 1..64 when present, got %d", c.ExpectedSlaves)
	}
	if c.BuildJobs < 1 || c.BuildJobs > 64 {
		return fmt.Errorf("cell_deploy build_jobs must be 1..64, got %d", c.BuildJobs)
	}
	// A copy must stay far below the servo host's link and SD card rates while
	// the controller runs; the bounds only reject unit mistakes (bytes vs MB).
	if c.DumpCopyRate < 64*1024 || c.DumpCopyRate > 64*1024*1024 {
		return fmt.Errorf("cell_deploy dump_copy_rate_bytes_per_s must be between 65536 and 67108864, got %d", c.DumpCopyRate)
	}
	for _, dir := range []string{c.ScratchDir, c.DumpDir} {
		if dir == c.DeploymentsDir || strings.HasPrefix(dir+"/", c.DeploymentsDir+"/") {
			return fmt.Errorf("cell_deploy scratch_dir and dump_dir must be outside deployments_dir (deployments are created root-owned by the build)")
		}
	}
	if c.DCLock != nil {
		if err := c.DCLock.validate(); err != nil {
			return err
		}
	}
	if c.Camera != nil {
		if err := c.Camera.validate(); err != nil {
			return err
		}
		// One camera, named once: an origin written by hand beside the block
		// could drift from the service the block installs.
		if c.OLP.CameraOrigin != "" && c.OLP.CameraOrigin != c.Camera.Origin() {
			return fmt.Errorf("cell_deploy olp.camera_origin %q differs from the camera block's %q; drop olp.camera_origin", c.OLP.CameraOrigin, c.Camera.Origin())
		}
	}
	if err := c.validateExtraCameras(); err != nil {
		return err
	}
	o := c.OLP
	if !cellDeployUnit.MatchString(o.Unit) {
		return fmt.Errorf("cell_deploy olp.unit must be a systemd .service name, got %q", o.Unit)
	}
	for field, unit := range map[string]string{"olp.planner_unit": o.PlannerUnit, "olp.control_tunnel_unit": o.ControlTunnelUnit} {
		if unit != "" && !cellDeployUnit.MatchString(unit) {
			return fmt.Errorf("cell_deploy %s must be a systemd .service name, got %q", field, unit)
		}
	}
	if o.PlannerUnit == "" {
		return fmt.Errorf("cell_deploy olp.planner_unit is required: the OLP unit orders itself after the planner")
	}
	if strings.Count(o.Description, cellDeployReleaseToken) != 1 || strings.ContainsAny(o.Description, "\r\n%\\") {
		return fmt.Errorf("cell_deploy olp.description must be one line holding %s exactly once, without %% or backslashes", cellDeployReleaseToken)
	}
	host, port, err := net.SplitHostPort(o.Listen)
	if n, perr := strconv.ParseUint(port, 10, 16); err != nil || perr != nil || n == 0 || host == "" || strings.ContainsAny(host, " \t") {
		return fmt.Errorf("cell_deploy olp.listen must be host:port, got %q", o.Listen)
	}
	u, err := url.Parse(o.PlannerOrigin)
	if err != nil || (u.Scheme != "http" && u.Scheme != "https") || u.Host == "" || (u.Path != "" && u.Path != "/") || u.User != nil || u.RawQuery != "" {
		return fmt.Errorf("cell_deploy olp.planner_origin must be an http(s) origin, got %q", o.PlannerOrigin)
	}
	if !cellDeployToken.MatchString(o.PlannerPixiEnv) {
		return fmt.Errorf("cell_deploy olp.planner_pixi_env must be a pixi environment name, got %q", o.PlannerPixiEnv)
	}
	if o.Planner != nil {
		if err := o.Planner.validate(c); err != nil {
			return err
		}
	}
	if t := o.DiscoveryTailnet; t != nil {
		if _, apiPort, _ := net.SplitHostPort(o.Listen); t.ListenPort < 1024 || t.ListenPort > 65535 || strconv.Itoa(t.ListenPort) == apiPort {
			return fmt.Errorf("cell_deploy olp.discovery_tailnet.listen_port must be 1024-65535 and not olp.listen's port, got %d", t.ListenPort)
		}
	}
	if o.ProgramStore != nil {
		if err := o.ProgramStore.validate(c); err != nil {
			return err
		}
	}
	if o.CameraOrigin != "" && !cellDeployLoopbackHTTP(o.CameraOrigin) {
		return fmt.Errorf("cell_deploy olp.camera_origin must be an http URL on the relay itself (a loopback address), got %q", o.CameraOrigin)
	}
	for _, name := range o.DiscoveryInterfaces {
		if !cellDeployInterface.MatchString(name) {
			return fmt.Errorf("cell_deploy olp.discovery_interfaces must hold network interface names, got %q", name)
		}
	}
	for _, unit := range o.Replaces {
		if !cellDeployUnit.MatchString(unit) || unit == o.Unit || unit == o.PlannerUnit || unit == o.ControlTunnelUnit {
			return fmt.Errorf("cell_deploy olp.replaces must hold systemd .service names other than the OLP's own units, got %q", unit)
		}
	}
	return nil
}

// cellDeployStripNotes removes "_"-prefixed documentation keys from an object
// (and from its "olp" member) so the remaining keys can be decoded strictly: a
// misspelt field must fail validation, not silently fall back to empty.
func cellDeployStripNotes(raw json.RawMessage) (json.RawMessage, error) {
	var object map[string]json.RawMessage
	if err := json.Unmarshal(raw, &object); err != nil || object == nil {
		return nil, fmt.Errorf("cell_deploy must be a JSON object")
	}
	for key, value := range object {
		if strings.HasPrefix(key, "_") {
			delete(object, key)
			continue
		}
		if key == "olp" || key == "camera" || key == "planner" || key == "program_store" || key == "dc_lock" || key == "realsense" {
			stripped, err := cellDeployStripNotes(value)
			if err != nil {
				return nil, fmt.Errorf("cell_deploy %s: %w", key, err)
			}
			object[key] = stripped
		}
		if key == "extra_cameras" {
			var list []json.RawMessage
			if err := json.Unmarshal(value, &list); err != nil {
				return nil, fmt.Errorf("cell_deploy extra_cameras must be a list of camera blocks")
			}
			for i, entry := range list {
				stripped, err := cellDeployStripNotes(entry)
				if err != nil {
					return nil, fmt.Errorf("cell_deploy extra_cameras[%d]: %w", i, err)
				}
				list[i] = stripped
			}
			joined, err := json.Marshal(list)
			if err != nil {
				return nil, err
			}
			object[key] = joined
		}
	}
	return json.Marshal(object)
}

// decodeCellDeployConfig reads the block from a whole cell manifest.
func decodeCellDeployConfig(manifest []byte) (cellDeployConfig, error) {
	var root map[string]json.RawMessage
	if err := json.Unmarshal(manifest, &root); err != nil {
		return cellDeployConfig{}, fmt.Errorf("cell manifest is not a JSON object: %w", err)
	}
	raw, ok := root["cell_deploy"]
	if !ok {
		return cellDeployConfig{}, fmt.Errorf("cell manifest has no cell_deploy block; add one (steamdeck/real/v4/README.md, \"Adding a new robot cell\")")
	}
	stripped, err := cellDeployStripNotes(raw)
	if err != nil {
		return cellDeployConfig{}, err
	}
	var c cellDeployConfig
	decoder := json.NewDecoder(bytes.NewReader(stripped))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&c); err != nil {
		return cellDeployConfig{}, fmt.Errorf("decode cell_deploy: %w", err)
	}
	return c, c.validate()
}

// cellDeployContext is everything a rendered relay script needs.
type cellDeployContext struct {
	CellID      string
	Manifest    string // repository-relative when inside the repository
	Deploy      cellDeployConfig
	RTCore      robotV4RTCoreConfig
	MachineAxes int
	Slaves      int
	// DCSlaves are the EtherCAT positions rt-core configures a distributed
	// clock on: every axis's slave_position unless the machine sets
	// bus_bringup.no_dc (engine/bus_bringup.hpp, ecrt_slave_config_dc).
	DCSlaves    []int
	RobotModel  string
	RelayPeer   string
	RelayUser   string
	RelayHost   string
	ServoPeer   string
	ServoTarget string // user@host as the relay reaches it
}

// ExecutionAddress is the OLP's authenticated route to rt-control (on a Rosie
// 1430 cell the relay's loopback tunnel to the servo host).
func (ctx cellDeployContext) ExecutionAddress() string {
	return "https://" + ctx.RTCore.RemoteListen
}

func (ctx cellDeployContext) DeploymentDir(commit string) string {
	return path.Join(ctx.Deploy.DeploymentsDir, ctx.Deploy.DeploymentPrefix+cellDeployShort(commit))
}

// MachineInRTCore is the machine config path relative to rt-core/, as
// `rtctl compile --config` takes it from that directory.
func (ctx cellDeployContext) MachineInRTCore() string {
	return strings.TrimPrefix(ctx.RTCore.MachineConfig, "rt-core/")
}

func (ctx cellDeployContext) FullAxisMask() uint64 {
	return uint64(1)<<uint(ctx.MachineAxes) - 1
}

func cellDeployShort(commit string) string {
	if len(commit) > 8 {
		return commit[:8]
	}
	return commit
}

// loadCellDeployContext resolves the selected manifest (selectRealtimeCoreManifest
// must have run). It reads no host.
func loadCellDeployContext() (cellDeployContext, error) {
	node, servoTarget, _, viaPeer, err := rtCoreRelayRoute()
	if err != nil {
		return cellDeployContext{}, err
	}
	relay, err := robotV4ManifestManagementConnection(viaPeer)
	if err != nil {
		return cellDeployContext{}, err
	}
	if relay.Transport != "ssh" || relay.SSHUser == "" || relay.Host == "" {
		return cellDeployContext{}, fmt.Errorf("relay %s must be a direct ssh connection with ssh_user and host (it hosts builds, deployments and the OLP)", viaPeer)
	}
	manifestPath := resolveRobotV4ManifestPath(selectedRobotV4ManifestPath())
	raw, err := os.ReadFile(manifestPath)
	if err != nil {
		return cellDeployContext{}, err
	}
	deploy, err := decodeCellDeployConfig(raw)
	if err != nil {
		return cellDeployContext{}, fmt.Errorf("%s: %w", manifestPath, err)
	}
	repo, err := repoRoot()
	if err != nil {
		return cellDeployContext{}, err
	}
	machine, err := cellDeployMachineFacts(filepath.Join(repo, filepath.FromSlash(node.RTCore.MachineConfig)))
	if err != nil {
		return cellDeployContext{}, err
	}
	axes, model, unknownSlaves := machine.Axes, machine.Model, machine.UnknownSlaves
	slaves := axes
	switch {
	case unknownSlaves == "refuse" && deploy.ExpectedSlaves != 0 && deploy.ExpectedSlaves != axes:
		return cellDeployContext{}, fmt.Errorf("cell_deploy expected_ethercat_slaves %d contradicts %s: it refuses unknown slaves, so the slaves are its %d axes", deploy.ExpectedSlaves, node.RTCore.MachineConfig, axes)
	case unknownSlaves != "refuse" && deploy.ExpectedSlaves < axes:
		return cellDeployContext{}, fmt.Errorf("%s does not refuse unknown slaves (%q): cell_deploy expected_ethercat_slaves must name the bus's slave count (at least its %d axes)", node.RTCore.MachineConfig, unknownSlaves, axes)
	case unknownSlaves != "refuse":
		slaves = deploy.ExpectedSlaves
	}
	shown := manifestPath
	if abs, err := filepath.Abs(manifestPath); err == nil {
		if rel, err := filepath.Rel(repo, abs); err == nil && !strings.HasPrefix(rel, "..") {
			shown = filepath.ToSlash(rel)
		}
	}
	return cellDeployContext{
		CellID: strings.TrimSuffix(filepath.Base(manifestPath), ".json"), Manifest: shown, Deploy: deploy, RTCore: *node.RTCore,
		MachineAxes: axes, Slaves: slaves, DCSlaves: machine.DCSlaves, RobotModel: model, RelayPeer: viaPeer, RelayUser: relay.SSHUser, RelayHost: relay.Host,
		ServoPeer: node.Peer, ServoTarget: servoTarget,
	}, nil
}

// cellDeployMachine is what the deploy commands read from a machine config.
type cellDeployMachine struct {
	Axes          int
	Model         string
	UnknownSlaves string
	DCSlaves      []int
}

// cellDeployMachineFacts reads the axis count, robot model, unknown-slave
// policy and DC slave positions from a machine config. The model is the
// robot_description directory name, which is what robot-v4-cartesiand
// --resolve-only and the OLP target picker take.
func cellDeployMachineFacts(machinePath string) (cellDeployMachine, error) {
	raw, err := os.ReadFile(machinePath)
	if err != nil {
		return cellDeployMachine{}, err
	}
	var machine struct {
		Robot struct {
			RobotDescription struct {
				Path string `json:"path"`
			} `json:"robot_description"`
		} `json:"robot"`
		Axes []struct {
			SlavePosition *int `json:"slave_position"`
		} `json:"axes"`
		Bus struct {
			UnknownSlaves string `json:"unknown_slaves"`
		} `json:"bus"`
		BusBringup struct {
			NoDC bool `json:"no_dc"`
		} `json:"bus_bringup"`
	}
	if err := json.Unmarshal(raw, &machine); err != nil {
		return cellDeployMachine{}, fmt.Errorf("machine config %s: %w", machinePath, err)
	}
	model := path.Base(machine.Robot.RobotDescription.Path)
	if len(machine.Axes) == 0 || len(machine.Axes) > 32 || !cellDeployToken.MatchString(model) || !strings.HasPrefix(machine.Robot.RobotDescription.Path, "robot_description/robots/") {
		return cellDeployMachine{}, fmt.Errorf("machine config %s needs 1..32 axes and robot.robot_description.path robot_description/robots/<model>", machinePath)
	}
	out := cellDeployMachine{Axes: len(machine.Axes), Model: model, UnknownSlaves: machine.Bus.UnknownSlaves}
	if !machine.BusBringup.NoDC {
		for i, a := range machine.Axes {
			if a.SlavePosition == nil || *a.SlavePosition < 0 || *a.SlavePosition > 65535 {
				return cellDeployMachine{}, fmt.Errorf("machine config %s axis %d needs slave_position 0..65535 (the DC lock check reads that slave)", machinePath, i)
			}
			out.DCSlaves = append(out.DCSlaves, *a.SlavePosition)
		}
	}
	return out, nil
}

// cellDeployLoopbackHTTP accepts an http URL, with an optional path, whose
// host is this machine, in characters that are safe in a unit's Environment=.
func cellDeployLoopbackHTTP(raw string) bool {
	u, err := url.Parse(raw)
	if err != nil || u.Scheme != "http" || u.Host == "" || u.User != nil || u.RawQuery != "" || u.Fragment != "" ||
		strings.ContainsAny(raw, " \t\"'\\$%") {
		return false
	}
	ip := net.ParseIP(u.Hostname())
	return u.Hostname() == "localhost" || (ip != nil && ip.IsLoopback())
}
