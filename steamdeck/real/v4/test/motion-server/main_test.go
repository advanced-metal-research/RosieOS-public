package motion_server

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"testing"
)

func TestMain(m *testing.M) {
	// Offline solver fixtures share the native launch selection without acquiring a runtime.
	if err := os.Setenv("MOTION_SERVER_BACKEND", "rt_core"); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(2)
	}
	repoRoot, err := os.Getwd()
	if err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: getwd: %v\n", err)
		os.Exit(2)
	}
	for {
		if _, statErr := os.Stat(filepath.Join(repoRoot, "steamdeck", "real", "v4", "go.mod")); statErr == nil {
			break
		}
		parent := filepath.Dir(repoRoot)
		if parent == repoRoot {
			fmt.Fprintln(os.Stderr, "motion-server tests: repository root not found")
			os.Exit(2)
		}
		repoRoot = parent
	}
	plannerURDFPath := filepath.Join(
		repoRoot, "robot_description", "robots", "rosie_1400_v3", "robot.urdf",
	)
	plannerURDF, err := os.ReadFile(plannerURDFPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: read planner URDF: %v\n", err)
		os.Exit(2)
	}
	plannerDigest := sha256.Sum256(plannerURDF)
	generatedSHA256 := hex.EncodeToString(plannerDigest[:])
	numerators := []uint16{100, 100, 100, 18, 100, 10, 10, 10, 10}
	denominators := []uint16{1, 1, 1, 1, 11, 1, 1, 1, 1}
	zeros := []float64{0, 0, 0, 0, 0, 0, 0, 0, 0}
	// High but explicit fixture limits keep legacy solver/slew tests focused
	// on their own math; production values always arrive in the real status.
	speed := []float64{20, 20, 20, 20, 20, 20, 20, 20, 20}
	lower := []float64{-10, -10, -10, -10, -10, -10, -10, -10, -10}
	upper := []float64{10, 10, 10, 10, 10, 10, 10, 10, 10}
	payload, err := json.Marshal(map[string]any{
		"schema":    "robot.v4.rtcore.status.v1",
		"subject":   "robot/v4/rtcore.MotionServerExplicitTest.status",
		"host":      "MotionServerExplicitTest",
		"component": "rtcore",
		"status": map[string]any{
			"name":                                              "robot-v4-rtcore",
			"state":                                             "running",
			"rtcore_mode":                                       "sim",
			"runtime_mode_epoch":                                uint64(7001),
			"pair_config_jetson_id":                             "jetson-motion-test",
			"pair_config_robot_model_id":                        "rosie_1400_v3",
			"pair_config_revision":                              uint64(1),
			"pair_config_sha256":                                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
			"pair_config_generated_urdf_sha256":                 generatedSHA256,
			"pair_config_activation_epoch":                      uint64(7002),
			"pair_config_home_epoch":                            uint64(7003),
			"pair_config_home_profile_key":                      "0123456789abcdef",
			"pair_config_activation":                            "live-qualified",
			"pair_config_mapped_axis_mask":                      uint32(0x1ff),
			"home_valid_axis_mask":                              uint32(0x1ff),
			"ethercat_live":                                     false,
			"pair_config_drive_ratio_converged":                 true,
			"pair_config_ratio_reconciliation_simulated":        true,
			"pair_config_ratio_reconciliation_state":            "simulated",
			"pair_config_ratio_reconciliation_abort_code":       uint32(0),
			"pair_config_ratio_reconciliation_failed_axis_mask": uint32(0),
			"pair_config_gear_numerator":                        numerators,
			"pair_config_gear_denominator":                      denominators,
			"pair_config_observed_gear_numerator":               numerators,
			"pair_config_observed_gear_denominator":             denominators,
			"pair_config_home_position_rad":                     zeros,
			"pair_config_speed_limit_rad_s":                     speed,
			"pair_config_lower_limit_rad":                       lower,
			"pair_config_upper_limit_rad":                       upper,
		},
	})
	if err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: encode explicit RTCore status: %v\n", err)
		os.Exit(2)
	}
	if err := os.Setenv("ROBOT_V4_ALLOW_EXPLICIT_TEST_AUTHORITY_INJECTION", "1"); err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: set injection guard: %v\n", err)
		os.Exit(2)
	}
	if err := os.Setenv("ROBOT_V4_EXPLICIT_TEST_RTCORE_STATUS_PAYLOAD", string(payload)); err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: set explicit RTCore status: %v\n", err)
		os.Exit(2)
	}
	if err := os.Setenv("ROBOT_V4_TESSERACT_URDF_PATH", plannerURDFPath); err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: set explicit planner URDF: %v\n", err)
		os.Exit(2)
	}
	if err := os.Setenv("ROBOT_V4_LIVE_AXIS_COUNT", "9"); err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: set live axis count: %v\n", err)
		os.Exit(2)
	}
	if err := os.Setenv(
		"ROBOT_V4_ROBOT_AXES",
		"0:J1:J1:arm:-1:1;1:J2:J2:arm:1:1;2:J3:J3:arm:-1:1;3:J4:J4:arm:-1:1;4:J5:J5:arm:-1:1;5:J6:J6:arm:-1:1;6:J7:J7:external:1:0;7:J8:J8:external:1:0;8:J9:J9:external:1:0",
	); err != nil {
		fmt.Fprintf(os.Stderr, "motion-server tests: set exact axis wiring: %v\n", err)
		os.Exit(2)
	}
	os.Exit(m.Run())
}
