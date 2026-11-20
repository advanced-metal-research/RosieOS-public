package main

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"

	"rosieos/robotdesc"
	"rosieos/rt-core/cartesian"
	"rosieos/rt-core/tools/rtctl"
)

// JointConfig is one joint's unit conversion and bounds, every bound in joint
// units (rad, rad/s, rad/s^2) exactly as the compiled configuration hands it
// to the runtime. The report converts to degrees for reading; it never invents
// a bound.
type JointConfig struct {
	Name            string            `json:"name"`
	GearRatio       float64           `json:"gear_ratio"`
	MotorCountsRev  float64           `json:"motor_counts_per_rev"`
	Sign            float64           `json:"sign"`
	CountsPerRad    float64           `json:"counts_per_rad"`
	FollowingError  float64           `json:"following_error_rad"`
	MaxTargetLead   float64           `json:"max_target_lead_rad"`
	CompletionTol   float64           `json:"completion_tolerance_rad"`
	VelocityLimit   float64           `json:"velocity_limit_rad_s"`
	AccelerationLim float64           `json:"acceleration_limit_rad_s2"`
	JogAcceleration float64           `json:"jog_acceleration_rad_s2"`
	LowerRad        float64           `json:"lower_rad"`
	UpperRad        float64           `json:"upper_rad"`
	URDFJoint       string            `json:"urdf_joint"`
	DriveSettings   map[string]string `json:"drive_settings,omitempty"`
}

// CellConfig is everything the analysis takes from configuration: the
// compiled configuration (rtctl's compiler, the numbers the runtime enforces)
// and the kinematics of the robot description it pins (rt-core/cartesian, the
// resolver's FK). Nothing is re-derived from the machine config by hand.
type CellConfig struct {
	ManifestPath        string                `json:"manifest,omitempty"`
	PinnedConfiguration string                `json:"pinned_configuration_sha256,omitempty"`
	MachinePath         string                `json:"machine_config,omitempty"`
	MachineSHA256       string                `json:"machine_config_sha256,omitempty"`
	CompiledPath        string                `json:"compiled_configuration,omitempty"`
	ConfigurationSHA256 string                `json:"configuration_sha256"`
	DescriptionPath     string                `json:"robot_description"`
	DescriptionSHA256   string                `json:"robot_description_sha256"`
	Notes               []string              `json:"notes,omitempty"`
	CycleNs             float64               `json:"cycle_ns"`
	LagAllowanceS       float64               `json:"tracking_lag_allowance_s"`
	JogJerk             float64               `json:"jog_jerk_rad_s3"`
	ToolFrame           string                `json:"tool_frame"`
	Joints              []JointConfig         `json:"joints"`
	Kin                 *cartesian.Kinematics `json:"-"`
	// ModelID and CalibrationSHA256 are the robot binding's model and
	// machine_planning_calibration.json identity: with the description and
	// configuration they name the robot a generated program is for.
	ModelID           string `json:"model_id,omitempty"`
	CalibrationSHA256 string `json:"machine_planning_calibration_sha256,omitempty"`
	// PairRevision is the manifest's rt_core.pair_revision: the control
	// binding revision a program's manifest_revision must equal for native
	// preparation to accept it.
	PairRevision uint64 `json:"pair_revision,omitempty"`
}

// findRepoRoot walks up from dir to the checkout holding rt-core and the
// robot descriptions; manifest and machine paths are relative to it.
func findRepoRoot(dir string) (string, error) {
	d, _ := filepath.Abs(dir)
	for {
		if st, err := os.Stat(filepath.Join(d, "robot_description")); err == nil && st.IsDir() {
			if st, err := os.Stat(filepath.Join(d, "rt-core")); err == nil && st.IsDir() {
				return d, nil
			}
		}
		parent := filepath.Dir(d)
		if parent == d {
			return "", fmt.Errorf("no RosieOS checkout above %s (pass --repo)", dir)
		}
		d = parent
	}
}

func readJSON(path string, v any) ([]byte, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	if err = json.Unmarshal(b, v); err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	return b, nil
}

// machineFromManifest finds the node whose rt_core block pins a machine
// config. A manifest with none, or with two different ones, is refused: the
// report must name exactly one configuration.
func machineFromManifest(path string) (machine, pinned string, pairRevision uint64, err error) {
	var m struct {
		Nodes []struct {
			RTCore *struct {
				MachineConfig       string `json:"machine_config"`
				ConfigurationSHA256 string `json:"configuration_sha256"`
				PairRevision        uint64 `json:"pair_revision"`
			} `json:"rt_core"`
		} `json:"nodes"`
	}
	if _, err = readJSON(path, &m); err != nil {
		return "", "", 0, err
	}
	for _, n := range m.Nodes {
		if n.RTCore == nil || n.RTCore.MachineConfig == "" {
			continue
		}
		if machine != "" && machine != n.RTCore.MachineConfig {
			return "", "", 0, fmt.Errorf("%s pins two machine configs (%s, %s); pass --machine", path, machine, n.RTCore.MachineConfig)
		}
		machine, pinned, pairRevision = n.RTCore.MachineConfig, n.RTCore.ConfigurationSHA256, n.RTCore.PairRevision
	}
	if machine == "" {
		return "", "", 0, fmt.Errorf("%s: no node has rt_core.machine_config", path)
	}
	return machine, pinned, pairRevision, nil
}

var configurationName = regexp.MustCompile(`^[0-9a-f]{64}$`)

// loadCellConfig takes the configuration from rtctl's compiler: either it
// compiles the machine config (the manifest's, or --machine) against the
// checkout's drive profiles and robot description, exactly as `realtime-core
// config` does, or it reads a compiled directory (`rtctl compile --out`
// output, named by its configuration sha256, e.g. the one deployed on the
// cell) so a report can be judged by the configuration that actually ran.
func loadCellConfig(manifestPath, machinePath, compiledDir, repo string) (*CellConfig, error) {
	cfg := &CellConfig{}
	var err error
	if repo == "" {
		start := manifestPath
		for _, p := range []string{machinePath, compiledDir} {
			if start == "" {
				start = p
			}
		}
		if repo, err = findRepoRoot(filepath.Dir(start)); err != nil {
			return nil, err
		}
	}
	if manifestPath != "" {
		cfg.ManifestPath = manifestPath
		rel, pinned, pairRevision, err := machineFromManifest(manifestPath)
		if err != nil {
			return nil, err
		}
		cfg.PinnedConfiguration, cfg.PairRevision = pinned, pairRevision
		if machinePath == "" && compiledDir == "" {
			machinePath = filepath.Join(repo, rel)
		}
	}
	var argv []string
	var axisProfiles string
	var robot *rtctl.RobotBinding
	switch {
	case compiledDir != "":
		cfg.CompiledPath = compiledDir
		if name := filepath.Base(filepath.Clean(compiledDir)); configurationName.MatchString(name) {
			cfg.ConfigurationSHA256 = name
		}
		if _, err = readJSON(filepath.Join(compiledDir, "argv.json"), &argv); err != nil {
			return nil, err
		}
		raw, err := os.ReadFile(filepath.Join(compiledDir, "axes.conf"))
		if err != nil {
			return nil, err
		}
		axisProfiles = string(raw)
		if _, err = os.Stat(filepath.Join(compiledDir, "robot.json")); err == nil {
			robot = &rtctl.RobotBinding{}
			if _, err = readJSON(filepath.Join(compiledDir, "robot.json"), robot); err != nil {
				return nil, err
			}
		}
	case machinePath != "":
		mb, err := os.ReadFile(machinePath)
		if err != nil {
			return nil, err
		}
		sum := sha256.Sum256(mb)
		cfg.MachinePath, cfg.MachineSHA256 = machinePath, hex.EncodeToString(sum[:])
		compiled, err := rtctl.Compile(mb, filepath.Join(repo, "rt-core", "config", "drives"))
		if err != nil {
			return nil, fmt.Errorf("%s: rtctl compile: %w", machinePath, err)
		}
		argv, axisProfiles, robot, cfg.ConfigurationSHA256 = compiled.Argv, compiled.AxisProfiles, compiled.Robot, compiled.SHA256
	default:
		return nil, errors.New("need --manifest, --machine or --compiled")
	}
	if cfg.PinnedConfiguration != "" && cfg.ConfigurationSHA256 != "" && cfg.PinnedConfiguration != cfg.ConfigurationSHA256 {
		cfg.Notes = append(cfg.Notes, fmt.Sprintf("configuration %.12s is not the cell pin %.12s: the bounds below may not be the ones that ran", cfg.ConfigurationSHA256, cfg.PinnedConfiguration))
	}
	cycle, axes, err := rtctl.CompiledAxes(argv, axisProfiles)
	if err != nil {
		return nil, err
	}
	cfg.CycleNs = float64(cycle)
	if robot == nil {
		return nil, errors.New("the configuration binds no robot description; the report needs it for kinematics")
	}
	cfg.DescriptionPath, cfg.DescriptionSHA256 = robot.Description.Path, robot.Description.RobotDescriptionSHA256
	cfg.ModelID, cfg.CalibrationSHA256 = robot.ModelID, robot.MachinePlanningCalibrationSHA256
	desc := filepath.Join(repo, robot.Description.Path)
	if got, err := robotdesc.RobotDescriptionSHA256(desc); err != nil || got != cfg.DescriptionSHA256 {
		cfg.Notes = append(cfg.Notes, fmt.Sprintf("robot description %s in this checkout is %.19s (%v), the configuration pins %.19s: kinematics are the checkout's", robot.Description.Path, got, err, cfg.DescriptionSHA256))
	}
	if cfg.Kin, err = cartesian.LoadKinematics(desc); err != nil {
		return nil, err
	}
	if len(robot.Axes) != len(axes) {
		return nil, fmt.Errorf("robot binding has %d axes, the compiled configuration %d", len(robot.Axes), len(axes))
	}
	var chain []string
	for i, a := range axes {
		b := robot.Axes[i]
		urdf, _ := b.URDFJoint.(string)
		motor, err := strconv.ParseFloat(b.MotorEncoderCountsPerRev, 64)
		if b.Name != a.Name || err != nil || motor <= 0 {
			return nil, fmt.Errorf("robot binding axis %d (%s, %q counts) does not match compiled axis %s", i, b.Name, b.MotorEncoderCountsPerRev, a.Name)
		}
		if b.CartesianParticipates {
			chain = append(chain, urdf)
		}
		jc := JointConfig{Name: a.Name, URDFJoint: urdf, MotorCountsRev: motor, GearRatio: a.CountsPerRev * a.GearRatio / motor, Sign: a.Sign,
			CountsPerRad: a.CountsPerRad(), FollowingError: a.FollowingError, MaxTargetLead: a.MaxTargetLead, CompletionTol: a.CompletionTolerance,
			VelocityLimit: a.MaxVelocity, AccelerationLim: a.MaxAcceleration, JogAcceleration: a.JogAcceleration, LowerRad: a.Lower, UpperRad: a.Upper}
		// The drive writes as compiled (startup SDOs, per-joint tuning) are
		// carried into the report so an iteration records what was set.
		for _, sdo := range strings.Split(a.StartupSDOs, ";") {
			if f := strings.Split(sdo, "|"); len(f) == 5 {
				if jc.DriveSettings == nil {
					jc.DriveSettings = map[string]string{}
				}
				jc.DriveSettings[strings.TrimPrefix(f[0], "a6ec_")] = f[4]
			}
		}
		if i == 0 {
			cfg.LagAllowanceS, cfg.JogJerk = a.TrackingLagAllowanceMs/1000, a.JogJerk
		} else if a.TrackingLagAllowanceMs/1000 != cfg.LagAllowanceS || a.JogJerk != cfg.JogJerk {
			return nil, fmt.Errorf("compiled axis %s carries its own tracking_lag_allowance or jog_jerk; both are one machine-level value", a.Name)
		}
		cfg.Joints = append(cfg.Joints, jc)
	}
	if strings.Join(chain, ",") != strings.Join(cfg.Kin.Joints, ",") {
		return nil, fmt.Errorf("compiled Cartesian axes %v are not the description's chain %v", chain, cfg.Kin.Joints)
	}
	cfg.ToolFrame = cfg.Kin.ToolFrame
	return cfg, nil
}

// q converts drive counts to joint radians the way OLP's logical_position
// does: (counts + native home offset) / (counts per rad x sign).
func (j JointConfig) q(counts, homeOff int64) float64 {
	return float64(counts+homeOff) / (j.CountsPerRad * j.Sign)
}

func (j JointConfig) v(countsPerS int64) float64 {
	return float64(countsPerS) / (j.CountsPerRad * j.Sign)
}
