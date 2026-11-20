//go:build linux

package rtctl

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/sdk/control"
)

// TestCellDescribesCompiledResources composes the actual compiler, simulator,
// adapter and SDK. The flat program fixture must not invent robot geometry;
// the pinned robot fixture must work after its source tree is removed.
func TestCellDescribesCompiledResources(t *testing.T) {
	for _, model := range []string{"simulation-program", "robot", "robot_sii_mismatch", "cell-io", "cell-io-planet", "cell-io-picontrol"} {
		t.Run(model, func(t *testing.T) {
			isRobot := strings.HasPrefix(model, "robot")
			root := testRoot(t)
			build := buildTemp(t)
			piControlImage := filepath.Join(build, "picontrol.img")
			var compiled Compiled
			var err error
			if isRobot {
				f := newRobotFixture(t, false)
				profileBytes, err := os.ReadFile(filepath.Join(f.profiles, "test.json"))
				if err != nil {
					t.Fatal(err)
				}
				profile := asObject(decode(profileBytes))
				ethercat := asObject(profile.get("ethercat"))
				// Unverified simulated identity fixture: vendor 1, product 1, revision 12024.
				ethercat.set("revision_no", json.Number("12024"))
				writeTestFile(t, filepath.Join(f.profiles, "test.json"), canonical(profile), 0644)
				urdf := strings.Replace(robotFixtureURDF, "</robot>", `<link name="link"><visual><geometry><mesh filename="package://fixture/meshes/link.stl"/></geometry></visual></link></robot>`, 1)
				writeTestFile(t, f.urdf, urdf, 0644)
				writeTestFile(t, filepath.Join(f.dir, "meshes", "link.stl"), "solid fixture\nendsolid fixture\n", 0644)
				f.pin(t)
				compiled, err = simfixture.CompileRaw([]byte(canonical(f.machine)), f.profiles, Compile)
				if err != nil {
					t.Fatal(err)
				}
				// Source removal proves request-time repository independence.
				if err := os.RemoveAll(f.dir); err != nil {
					t.Fatal(err)
				}
				if err := os.RemoveAll(filepath.Join(f.root, "config")); err != nil {
					t.Fatal(err)
				}
			} else {
				file := "config/machines/simulation/simulation-program.json"
				if strings.HasPrefix(model, "cell-io") {
					file = "config/cell-io/simulation.json"
				}
				raw := readTestFile(t, file)
				if io := schema2CellIO(t, model, piControlImage); io != "" {
					machine := asObject(decode(raw))
					machine.set("io", decode([]byte(io)))
					raw = []byte(canonical(machine))
				}
				compiled, err = simfixture.CompileRaw(raw, filepath.Join(root, "config/drives"), Compile)
				if err != nil {
					t.Fatal(err)
				}
			}
			directory, args, err := writeCompiled(compiled, filepath.Join(build, "config"))
			if err != nil {
				t.Fatal(err)
			}
			manifestBytes, err := os.ReadFile(filepath.Join(directory, "resources.json"))
			if err != nil {
				t.Fatal(err)
			}
			var manifest adapter.ResourceManifest
			if err := json.Unmarshal(manifestBytes, &manifest); err != nil {
				t.Fatal(err)
			}
			runtime, err := os.MkdirTemp("", "cr-")
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() { os.RemoveAll(runtime) })
			nativeSocket, publicSocket := filepath.Join(runtime, "ipc.sock"), filepath.Join(runtime, "control.sock")
			args = append(args, "--socket-path", nativeSocket, "--telemetry-directory", filepath.Join(build, "telemetry"))
			if model == "robot_sii_mismatch" {
				// Measured separate bench identities: SII 12024 and CoE 22033.
				// Inject wrong SII while preserving simulated CoE readback 12024.
				args = append(args, "--sim-sii-revision", "22033")
			}
			simfixture.SpawnLogged(t, exec.Command(filepath.Join(root, "build/rosie-rt-core-sim"), args...), filepath.Join(build, "core.log"), busTestWatchdog, 4000, nil)
			mask := uint32(511)
			if isRobot {
				mask = 1
			}
			simfixture.WaitVerified(t, filepath.Join(runtime, "metrics.json"), busTestWatchdog, 10*time.Millisecond, func(v simfixture.Verified) bool {
				return v.TimeNS != 0 && (v.ConfigVerifiedMask == mask || model == "robot_sii_mismatch")
			})
			var axes []string
			for _, drive := range manifest.Drives {
				axes = append(axes, drive.Axis)
			}
			args = simfixture.ControlArgs(nativeSocket, publicSocket, compiled.SHA256, "resources", 1, "--compiled-config", directory)
			simfixture.SpawnLogged(t, exec.Command(filepath.Join(root, "build/rt-control"), args...), filepath.Join(build, "control.log"), busTestWatchdog, 4000, nil)
			client, err := control.Dial(control.UnixPath(publicSocket))
			if err != nil {
				t.Fatal(err)
			}
			defer client.Close()
			ctx, cancel := context.WithTimeout(context.Background(), busTestWatchdog)
			defer cancel()
			if model == "robot_sii_mismatch" {
				err = simfixture.Poll(ctx, 10*time.Millisecond, func() (bool, error) {
					state, err := client.Status(ctx)
					if err != nil || len(state.Core.Axes) != 1 || state.Core.Axes[0].RevisionValid == 0 {
						return false, nil
					}
					if state.Core.ConfigVerifiedMask != 0 || state.Core.Axes[0].Revision != 12024 {
						t.Fatalf("SII mismatch accepted or substituted for CoE: %+v", state.Core)
					}
					return true, nil
				})
				if err != nil {
					t.Fatal(err)
				}
			}
			var description control.Description
			err = simfixture.Poll(ctx, 10*time.Millisecond, func() (bool, error) { description, err = client.Describe(ctx); return err == nil, nil })
			if err != nil {
				t.Fatal(err)
			}
			if !reflect.DeepEqual(description.Robot, manifest.Robot) || description.ConfigurationSHA256 != compiled.SHA256 {
				t.Fatalf("describe does not match compiled artefact: %+v", description.Robot)
			}
			if len(description.Drives) != len(manifest.Drives) {
				t.Fatal("drive count mismatch")
			}
			for i, drive := range description.Drives {
				if model == "robot" {
					if drive.VerifiedIdentity == nil || *drive.VerifiedIdentity != (control.DriveIdentity{ExpectedVendorID: 1, ExpectedProductCode: 1, ObservedCoERevision: 12024}) {
						t.Fatalf("axis %d simulated readback: %+v", i, drive.VerifiedIdentity)
					}
				} else if drive.VerifiedIdentity != nil {
					t.Fatalf("axis %d invented an unavailable revision readback", i)
				}
				drive.VerifiedIdentity = nil
				if drive != manifest.Drives[i] {
					t.Fatal("drive configuration changed")
				}
			}
			if description.Robot == nil {
				if _, err := client.Resource(ctx, strings.Repeat("0", 64)); err == nil {
					t.Fatal("flat fixture invented resources")
				}
				if model == "cell-io-planet" || model == "cell-io-picontrol" {
					// The native core published the configured outputs; with no
					// wired feedback they are never valid observations.
					err = simfixture.Poll(ctx, 10*time.Millisecond, func() (bool, error) {
						state, err := client.Status(ctx)
						if err != nil || len(state.Core.IO.Outputs) != 2 {
							return false, nil
						}
						for _, output := range state.Core.IO.Outputs {
							if output.Valid || output.Readback || output.Commanded {
								t.Fatalf("unwired output observed or commanded: %+v", output)
							}
						}
						return state.Core.IO.Outputs[0].Name == "gas" && state.Core.IO.Outputs[1].Name == "weld" && state.Core.IO.Outputs[1].Bit == 13, nil
					})
					if err != nil {
						t.Fatal(err)
					}
				}
				if model == "cell-io-picontrol" {
					image, err := os.ReadFile(piControlImage)
					if err != nil || len(image) != 4096 || image[70] != 0 || image[71] != 0 {
						t.Fatalf("piControl output word not OFF: %v", err)
					}
				}
				return
			}
			// The description whole: manifest, definition, URDF, mesh, and the
			// machine's machine_planning_calibration.json beside them.
			if len(description.Robot.Resources) != 6 {
				t.Fatalf("expected manifest, definition, config, URDF, mesh and machine_planning_calibration.json: %+v", description.Robot.Resources)
			}
			served, _ := json.Marshal(struct {
				Robot  *control.RobotDescription  `json:"robot"`
				Drives []control.DriveDescription `json:"drives"`
			}{description.Robot, description.Drives})
			t.Logf("served simulation describe: %s", served)
			for _, resource := range description.Robot.Resources {
				reader, err := client.Resource(ctx, resource.SHA256)
				if err != nil {
					t.Fatal(err)
				}
				data, err := io.ReadAll(reader)
				reader.Close()
				if err != nil {
					t.Fatal(err)
				}
				if uint64(len(data)) != resource.Bytes || identityDigest(string(data)) != resource.SHA256 || !bytes.Equal(data, compiled.resourceData[resource.SHA256]) {
					t.Fatal("resource differs from compiled bytes")
				}
			}
			_, err = client.Resource(ctx, strings.Repeat("0", 64))
			var refusal *control.Rejected
			if !errors.As(err, &refusal) || refusal.Reason != "resource_unknown" {
				t.Fatalf("unknown digest: %v", err)
			}
			var stdout, stderr bytes.Buffer
			out := filepath.Join(build, "fetch")
			if code := RunControlCLI([]string{"resources", "fetch", description.Robot.Resources[0].SHA256, "--out", out, "--socket", publicSocket}, &stdout, &stderr); code != 0 {
				t.Fatalf("resources CLI: %d %s", code, &stderr)
			}
			for _, resource := range description.Robot.Resources {
				data, err := os.ReadFile(filepath.Join(out, resource.SHA256))
				if err != nil || identityDigest(string(data)) != resource.SHA256 {
					t.Fatalf("CLI digest: %s %v", resource.Path, err)
				}
			}
			stdout.Reset()
			stderr.Reset()
			if code := RunControlCLI([]string{"describe", "--json", "--socket", publicSocket}, &stdout, &stderr); code != 0 || !bytes.Contains(stdout.Bytes(), []byte(description.Robot.RobotDescriptionSHA256)) {
				t.Fatalf("describe CLI: %d %s", code, &stderr)
			}
		})
	}
}

// schema2CellIO replaces the simulation fixture's terminal with a schema 2 I/O
// profile: the PLANET output-only terminal at bus position 9, or a RevPi DIO
// image whose piControl device is a 4096-byte file. Unverified example leases.
func schema2CellIO(t *testing.T, model, image string) string {
	t.Helper()
	outputs := `"outputs":[{"name":"gas","bit":0,"safe_state":false,"expiry_ns":200000000,"class":"process","readback":"none_unverified"},` +
		`{"name":"weld","bit":13,"safe_state":false,"expiry_ns":200000000,"class":"process","readback":"none_unverified"}]`
	switch model {
	case "cell-io-planet":
		return `{"profile":"planet_iecs_1116_do","slave_position":9,"torch_qualified":false,"inputs":[],` + outputs + `}`
	case "cell-io-picontrol":
		writeTestFile(t, image, string(make([]byte, 4096)), 0644)
		return `{"profile":"revpi_dio","picontrol_device":"` + image + `","picontrol_output_offset":70,"picontrol_input_offset":0,"torch_qualified":false,"inputs":[],` + outputs + `}`
	}
	return ""
}

// TestCompiledResourceMeshRefusals rejects missing, escaping and unsupported
// mesh references before any compiled directory can be published.
func TestCompiledResourceMeshRefusals(t *testing.T) {
	for _, reference := range []string{"package://fixture/../escape.stl", "https://example.invalid/a.stl", "file:///etc/passwd", "package://fixture/meshes/missing.stl"} {
		t.Run(reference, func(t *testing.T) {
			f := newRobotFixture(t, false)
			urdf := strings.Replace(robotFixtureURDF, "</robot>", fmt.Sprintf(`<link name="l"><visual><geometry><mesh filename="%s"/></geometry></visual></link></robot>`, reference), 1)
			writeTestFile(t, f.urdf, urdf, 0644)
			f.pin(t)
			if _, err := f.compile(); err == nil {
				t.Fatal("invalid mesh admitted")
			}
		})
	}
}
