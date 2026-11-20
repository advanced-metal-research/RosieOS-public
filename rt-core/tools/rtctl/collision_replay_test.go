package rtctl

import (
	"bufio"
	"bytes"
	"compress/gzip"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

// TestCollisionRecordedFeedback replays the checked-in observations through the
// exact native evaluator. Archived status has no torque: zero is an explicit
// unavailable-input placeholder, never claimed as measured torque evidence.
func TestCollisionRecordedFeedback(t *testing.T) {
	root := testRoot(t)
	var defaults []struct {
		Name   string `json:"name"`
		Config struct {
			Torque     int    `json:"torque_abs_max_raw"`
			Error      int    `json:"following_error_counts_max"`
			Cycles     int    `json:"sustained_cycles"`
			Unverified string `json:"_unverified"`
			Source     string `json:"_source"`
		} `json:"collision_watchdog"`
	}
	if err := json.Unmarshal(readTestFile(t, "tools/rtctl/testdata/collision-watchdog-unverified.json"), &defaults); err != nil {
		t.Fatal(err)
	}
	if len(defaults) != 9 {
		t.Fatal("expected nine per-axis defaults")
	}
	for i, d := range defaults {
		if d.Name != fmt.Sprintf("J%d", i+1) || d.Config.Torque != 1000 || d.Config.Cycles != 10 || d.Config.Unverified == "" || d.Config.Source == "" {
			t.Fatal(d)
		}
	}
	binary := filepath.Join(root, "build/cycle-machine-tests")
	if _, err := os.Stat(binary); err != nil {
		cmd := exec.Command("make", "build/cycle-machine-tests")
		cmd.Dir = root
		if out, err := cmd.CombinedOutput(); err != nil {
			t.Fatalf("build: %v\n%s", err, out)
		}
	}
	for _, name := range []string{"bd-program-b04-184908.jsonl.gz", "bd-program-ffA.jsonl.gz", "bd-program-ffB.jsonl.gz", "bd-move-b04.jsonl.gz", "bd-halt-b04-185015.jsonl.gz"} {
		t.Run(name, func(t *testing.T) {
			f, err := os.Open(filepath.Join(root, "tools/rtctl/testdata/bench-evidence/2026-09-15", name))
			if err != nil {
				t.Fatal(err)
			}
			defer f.Close()
			gz, err := gzip.NewReader(f)
			if err != nil {
				t.Fatal(err)
			}
			defer gz.Close()
			var input bytes.Buffer
			var counts []float64
			samples, torqueSamples, driveSamples := 0, 0, 0
			peakError := int64(0)
			emit := func(axis int, torque *int, error int32, valid bool, command int64) {
				if axis < 0 || axis >= 9 {
					t.Fatal("invalid axis", axis)
				}
				tq := 0
				if torque != nil {
					tq = *torque
					torqueSamples++
				}
				flag := 0
				if valid {
					flag = 1
					driveSamples++
					command = int64(error)
				}
				peakError = max(peakError, int64(math.Abs(float64(command))))
				c := defaults[axis].Config
				fmt.Fprintf(&input, "%d %d %d %d %d %d %d %d\n", axis, c.Torque, c.Error, c.Cycles, tq, error, flag, command)
				samples++
			}
			scan := bufio.NewScanner(gz)
			scan.Buffer(make([]byte, 65536), 4*1024*1024)
			for scan.Scan() {
				var row struct {
					Kind, Operation string
					Response        json.RawMessage
				}
				if err := json.Unmarshal(scan.Bytes(), &row); err != nil {
					t.Fatal(err)
				}
				if row.Operation == "describe" {
					var desc struct {
						Axes []struct {
							Counts float64 `json:"counts_per_unit"`
						}
					}
					if err := json.Unmarshal(row.Response, &desc); err != nil {
						t.Fatal(err)
					}
					for _, axis := range desc.Axes {
						counts = append(counts, axis.Counts)
					}
				}
				if row.Kind == "status" {
					var s struct {
						Core struct {
							Axes []struct {
								Index    int
								Fresh    uint32  `json:"pdo_fresh"`
								Target   float64 `json:"logical_target"`
								Position float64 `json:"logical_position"`
								Torque   *int    `json:"torque_raw"`
								Error    int32   `json:"following_error_counts"`
								Valid    bool    `json:"following_error_valid"`
							}
						}
					}
					if err := json.Unmarshal(row.Response, &s); err != nil {
						t.Fatal(err)
					}
					for _, a := range s.Core.Axes {
						if a.Fresh == 1 {
							if a.Index >= len(counts) {
								t.Fatal("missing scaling")
							}
							emit(a.Index, a.Torque, a.Error, a.Valid, int64(math.Round((a.Target-a.Position)*counts[a.Index])))
						}
					}
				}
				if row.Kind == "position" && (row.Operation == "move_cycle" || row.Operation == "halt_cycle") {
					var a struct {
						Axis   int
						Error  int32 `json:"following_error_counts"`
						Valid  bool  `json:"following_error_valid"`
						Torque *int  `json:"torque_raw"`
					}
					if err := json.Unmarshal(row.Response, &a); err != nil {
						t.Fatal(err)
					}
					if a.Valid {
						emit(a.Axis, a.Torque, a.Error, true, 0)
					}
				}
			}
			if err := scan.Err(); err != nil {
				t.Fatal(err)
			}
			if samples < 100 {
				t.Fatalf("insufficient recorded observations: %d", samples)
			}
			cmd := exec.Command(binary, "--collision-replay")
			cmd.Dir = root
			cmd.Stdin = &input
			out, err := cmd.CombinedOutput()
			if err != nil {
				t.Fatalf("replay: %v\n%s", err, out)
			}
			t.Logf("%s observations=%d drive_error_samples=%d torque_samples=%d peak_error_counts=%d; unavailable torque excluded from evidence", out, samples, driveSamples, torqueSamples, peakError)
		})
	}
}

func TestCollisionDefaultsCompile(t *testing.T) {
	var defaults []any = asArray(decode(readTestFile(t, "tools/rtctl/testdata/collision-watchdog-unverified.json")))
	for _, v := range defaults {
		d := asObject(v)
		f := collisionFixture()
		f.put("machine.axes.0.name", canonical(d.get("name")))
		f.put("machine.axes.0.collision_watchdog", canonical(d.get("collision_watchdog")))
		if _, err := f.compile(); err != nil {
			t.Fatal(err)
		}
	}
}
