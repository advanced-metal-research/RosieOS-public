//go:build linux

package control

import (
	"crypto/sha256"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"testing"

	"rosieos/robotdesc"
	"rosieos/rt-core/ipcclient"
)

// describedSet is a robot served the way rtctl compiles one: the description's
// manifest, its rt-core definition and whatever else it registers, by
// description-relative path, plus the machine's machine_planning_calibration.json, under the identity
// over the registered files (docs/ROBOT-DESCRIPTION-CONTRACT.md).
func describedSet(id, definition, cell string, extra map[string]string) (*RobotDescription, map[string][]byte) {
	registered := []string{"rtcore_definition.json"}
	contents := map[string]string{"rtcore_definition.json": definition}
	for path, data := range extra {
		registered = append(registered, path)
		contents[path] = data
	}
	sort.Strings(registered)
	quoted := make([]string, len(registered))
	for i, path := range registered {
		quoted[i] = strconv.Quote(path)
	}
	manifest := `{"schema":"rosie.robot-manifest.v1","model_id":"` + id + `","files":[` + strings.Join(quoted, ",") + `]}`
	entries := []robotdesc.Entry{{Path: "robot_description_manifest.json", SHA256: sha256.Sum256([]byte(manifest))}}
	for _, path := range registered {
		entries = append(entries, robotdesc.Entry{Path: path, SHA256: sha256.Sum256([]byte(contents[path]))})
	}
	files := map[string][]byte{}
	robot := &RobotDescription{ModelID: id, RobotDescriptionSHA256: robotdesc.HashEntries(entries), MachinePlanningCalibrationSHA256: "sha256:" + resourceDigest([]byte(cell))}
	add := func(path, data string) {
		media := "application/json"
		switch {
		case strings.HasSuffix(path, ".urdf"):
			media = "application/xml"
		case strings.HasSuffix(path, ".dae"):
			media = "model/vnd.collada+xml"
		case strings.HasSuffix(path, ".stl"):
			media = "model/stl"
		}
		robot.Resources = append(robot.Resources, ResourceInfo{Path: path, SHA256: resourceDigest([]byte(data)), Bytes: uint64(len(data)), MediaType: media})
		files[resourceDigest([]byte(data))] = []byte(data)
	}
	add("robot_description_manifest.json", manifest)
	for _, path := range registered {
		add(path, contents[path])
	}
	add("machine_planning_calibration.json", cell)
	return robot, files
}

// definitionSHA is the served definition's digest, the resource most tests
// tamper with.
func definitionSHA(m ResourceManifest) string {
	for _, r := range m.Robot.Resources {
		if r.Path == "rtcore_definition.json" {
			return r.SHA256
		}
	}
	return ""
}

func machineIdentity(id, robotSHA256, extra string) string {
	return `{"machine":{"robot":{"robot_description":{"path":"robot_description/robots/` + id + `","sha256":"` + robotSHA256 + `"}` + extra + `},"axes":[{"robot_joint":"axis_0","slave_position":0}]},"profiles":[{}]}`
}

func resourceFixture(t *testing.T) (string, ipcclient.Description, ResourceManifest) {
	t.Helper()
	dir := remoteTestDir(t)
	robot, files := describedSet("fixture", `{"robot_id":"fixture","defaults":{"drive_profile":"test"},"joints":[{"name":"axis_0"}]}`, `{"schema":"rosie.machine-planning-calibration.v1","model_id":"fixture"}`+"\n", nil)
	identity := []byte(machineIdentity("fixture", robot.RobotDescriptionSHA256, ""))
	native := ipcclient.Description{ConfigurationSHA256: resourceDigest(identity), MachineSHA256: strings.Repeat("a", 64), Axes: []ipcclient.AxisDescription{{ID: "axis_0"}}}
	m := ResourceManifest{ConfigurationSHA256: native.ConfigurationSHA256, MachineSHA256: native.MachineSHA256, Robot: robot,
		Drives: []DriveDescription{{Axis: "axis_0", ConfigName: "test", ConfigSHA256: resourceDigest([]byte(`{}`))}}}
	if err := os.Mkdir(filepath.Join(dir, "resources"), 0755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "configuration.identity"), identity, 0644); err != nil {
		t.Fatal(err)
	}
	for digest, data := range files {
		if err := os.WriteFile(filepath.Join(dir, "resources", digest), data, 0644); err != nil {
			t.Fatal(err)
		}
	}
	writeResourceManifest(t, dir, m)
	return dir, native, m
}

func writeResourceManifest(t *testing.T, dir string, m ResourceManifest) {
	t.Helper()
	data, err := json.Marshal(m)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "resources.json"), data, 0644); err != nil {
		t.Fatal(err)
	}
}

// TestResourceAdmissionAndSnapshot refuses mismatched or unbounded artefacts;
// verified snapshots remain detached from caller and disk mutations.
func TestResourceAdmissionAndSnapshot(t *testing.T) {
	for _, name := range []string{"configuration", "drive", "robot", "hash", "missing", "oversize", "duplicate", "escape", "extra_file"} {
		t.Run(name, func(t *testing.T) {
			dir, native, m := resourceFixture(t)
			file := filepath.Join(dir, "resources", definitionSHA(m))
			switch name {
			case "configuration":
				native.ConfigurationSHA256 = strings.Repeat("b", 64)
			case "drive":
				m.Drives[0].ConfigSHA256 = strings.Repeat("b", 64)
			case "robot":
				m.Robot.RobotDescriptionSHA256 = "sha256:" + strings.Repeat("b", 64)
			case "hash":
				if err := os.WriteFile(file, []byte("bad"), 0644); err != nil {
					t.Fatal(err)
				}
			case "missing":
				if err := os.Remove(file); err != nil {
					t.Fatal(err)
				}
			case "oversize":
				m.Robot.Resources[0].Bytes = 33554433
			case "duplicate":
				m.Robot.Resources = append(m.Robot.Resources, m.Robot.Resources[0])
			case "escape":
				outside := filepath.Join(remoteTestDir(t), "outside")
				body, err := os.ReadFile(file)
				if err != nil {
					t.Fatal(err)
				}
				if err := os.WriteFile(outside, body, 0644); err != nil {
					t.Fatal(err)
				}
				if err := os.Remove(file); err != nil {
					t.Fatal(err)
				}
				if err := os.Symlink(outside, file); err != nil {
					t.Fatal(err)
				}
			case "extra_file":
				if err := os.WriteFile(filepath.Join(dir, "resources", strings.Repeat("b", 64)), []byte("secret"), 0644); err != nil {
					t.Fatal(err)
				}
			}
			writeResourceManifest(t, dir, m)
			set, err := loadCompiledResources(dir, native)
			if name != "extra_file" {
				if err == nil {
					t.Fatal("invalid artefact admitted")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			c := &Controller{resources: set}
			for _, sha := range []string{strings.Repeat("b", 64), "../configuration.identity", strings.Repeat("A", 64), ""} {
				r := httptest.NewRecorder()
				NewHandler(c).ServeHTTP(r, httptest.NewRequest("GET", "/v1/resources/"+sha, nil))
				var refusal Response
				if json.Unmarshal(r.Body.Bytes(), &refusal) != nil || r.Code != 404 || refusal.Error != ReasonResourceUnknown {
					t.Fatalf("unknown resource: %d %s", r.Code, r.Body)
				}
			}
		})
	}
	dir, native, m := resourceFixture(t)
	set, err := loadCompiledResources(dir, native)
	if err != nil {
		t.Fatal(err)
	}
	c := &Controller{resources: set}
	robot, drives := c.describeResources(nil)
	robot.ModelID = "mutated"
	robot.Resources[0].Path = "mutated"
	drives[0].ConfigName = "mutated"
	definition := definitionSHA(m)
	if err := os.Remove(filepath.Join(dir, "resources", definition)); err != nil {
		t.Fatal(err)
	}
	r := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(r, httptest.NewRequest("GET", "/v1/resources/"+definition, nil))
	if r.Code != 200 || resourceDigest(r.Body.Bytes()) != definition || r.Header().Get("ETag") != `"`+definition+`"` || r.Header().Get("Content-Type") != "application/json" {
		t.Fatalf("snapshot: %d %s", r.Code, r.Body)
	}
	robot, drives = c.describeResources(nil)
	if robot.ModelID != "fixture" || robot.Resources[0].Path != "robot_description_manifest.json" || drives[0].ConfigName != "test" || drives[0].VerifiedIdentity != nil {
		t.Fatal("snapshot aliases or guessed drive identity")
	}
	state := &ipcclient.Status{ConfigVerifiedMask: 1, Axes: []ipcclient.AxisStatus{{RevisionValid: 1, Revision: 12024, VendorID: 4194304, ProductCode: 1813}}}
	_, drives = c.describeResources(state)
	if drives[0].VerifiedIdentity == nil || *drives[0].VerifiedIdentity != (DriveIdentity{ExpectedVendorID: 4194304, ExpectedProductCode: 1813, ObservedCoERevision: 12024}) {
		t.Fatal("bring-up identity lost")
	}
	state.ConfigVerifiedMask = 0
	_, drives = c.describeResources(state)
	if drives[0].VerifiedIdentity != nil {
		t.Fatal("stale verified identity")
	}
}

// TestResourceDriveMetadataBinding rejects forged names and bus positions under
// an unchanged configuration pin, including definition overrides and I/O wrappers.
func TestResourceDriveMetadataBinding(t *testing.T) {
	for _, model := range []string{"flat", "default", "joint_override", "bench_override", "cell_io"} {
		t.Run(model, func(t *testing.T) {
			for _, field := range []string{"valid", "name", "position", "axis"} {
				t.Run(field, func(t *testing.T) {
					dir, native, m := resourceFixture(t)
					id := "fixture"
					definition := `{"robot_id":"fixture","defaults":{"drive_profile":"test"},"joints":[{"name":"axis_0"}]}`
					extra := ""
					if model == "joint_override" {
						definition = strings.Replace(definition, `"name":"axis_0"`, `"name":"axis_0","drive_profile":"override"`, 1)
						m.Drives[0].ConfigName = "override"
					}
					if model == "bench_override" {
						id = "bench_nine_motors_1to1"
						definition = strings.ReplaceAll(definition, `"fixture"`, `"bench_nine_motors_1to1"`)
						definition = strings.ReplaceAll(definition, `"test"`, `"a6ec_bench_1to1"`)
						m.Drives[0].ConfigName = "a6ec_bench_1to1_fast"
						extra = `,"bench_measurement_profile":"a6ec_bench_1to1_fast"`
					}
					robot, served := describedSet(id, definition, `{"schema":"rosie.machine-planning-calibration.v1","model_id":"`+id+`"}`+"\n", nil)
					m.Robot = robot
					identity := machineIdentity(id, robot.RobotDescriptionSHA256, extra)
					if model == "flat" {
						m.Robot = nil
						identity = `{"machine":{"axes":[{"name":"axis_0","profile":"test","slave_position":0}]},"profiles":[{}]}`
					}
					files := map[string]string{}
					for digest, data := range served {
						files["resources/"+digest] = string(data)
					}
					if model == "cell_io" {
						files["configuration.base.identity"] = identity
						identity = `{"base_sha256":"` + resourceDigest([]byte(identity)) + `","io":{},"io_profile":{}}`
					}
					files["configuration.identity"] = identity
					native.ConfigurationSHA256 = resourceDigest([]byte(identity))
					m.ConfigurationSHA256 = native.ConfigurationSHA256
					for name, body := range files {
						if err := os.WriteFile(filepath.Join(dir, name), []byte(body), 0644); err != nil {
							t.Fatal(err)
						}
					}
					switch field {
					case "name":
						m.Drives[0].ConfigName = "not-the-compiled-name"
					case "position":
						m.Drives[0].SlavePosition = 9
					case "axis":
						m.Drives[0].Axis = "not-the-compiled-axis"
					}
					writeResourceManifest(t, dir, m)
					_, err := loadCompiledResources(dir, native)
					if field == "valid" {
						if err != nil {
							t.Fatal(err)
						}
					} else if err == nil || err.Error() != ReasonResourceArtifactInvalid {
						t.Fatalf("forged %s under unchanged pin %s: %v", field, native.ConfigurationSHA256, err)
					}
				})
			}
		})
	}
}

// TestResourceRemoteTLS applies the same principal/pair authentication as other
// observations; a resource download never establishes control authority.
func TestResourceRemoteTLS(t *testing.T) {
	dir, native, m := resourceFixture(t)
	set, err := loadCompiledResources(dir, native)
	if err != nil {
		t.Fatal(err)
	}
	c := &Controller{resources: set, binding: Binding{PairID: "pair"}}
	h := NewRemoteHandler(c, NewHandler(c), JogClockLimits{})
	defer h.Close()
	p := newRemoteTestPKI(t, "pair")
	address := remoteTestServer(t, p, h)
	cert := p.certificate(t, "observer", "pair", false, false)
	for _, sha := range []string{definitionSHA(m), strings.Repeat("0", 64)} {
		response, err := p.client(t, &cert).Get("https://" + address + "/v1/resources/" + sha)
		if err != nil {
			t.Fatal(err)
		}
		body, err := io.ReadAll(response.Body)
		response.Body.Close()
		if err != nil {
			t.Fatal(err)
		}
		if sha == definitionSHA(m) {
			if response.StatusCode != http.StatusOK || resourceDigest(body) != sha {
				t.Fatal("authenticated resource failed")
			}
		} else if response.StatusCode != 404 || !strings.Contains(string(body), ReasonResourceUnknown) {
			t.Fatal("unknown remote resource accepted")
		}
	}
	if response, err := p.client(t, nil).Get("https://" + address + "/v1/resources/" + definitionSHA(m)); err == nil {
		response.Body.Close()
		t.Fatal("unauthenticated resource accepted")
	}
}
