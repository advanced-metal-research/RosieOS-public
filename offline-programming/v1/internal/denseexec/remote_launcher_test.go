//go:build linux

package denseexec

import (
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// remotePKI uses the production issuer in its isolated test root, preventing
// handcrafted certificates from hiding incompatibility with deployed identities.
func remotePKI(t *testing.T, pair string) (string, string) {
	t.Helper()
	root, err := filepath.Abs("../../../../rt-core")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp("", "rosie-rt-pki-test.")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(dir) })
	pki := filepath.Join(dir, "pki")
	pkiCommand(t, root, dir, "init", pki, pair)
	pkiCommand(t, root, dir, "issue-client", pki, "olp-dev")
	return dir, pki
}

func pkiCommand(t *testing.T, root, dir string, args ...string) {
	t.Helper()
	cmd := exec.Command("bash", append([]string{filepath.Join(root, "host/remote-pki.sh")}, args...)...)
	cmd.Env = append(os.Environ(), "TMPDIR="+filepath.Dir(dir), "ROSIE_RT_PKI_TEST_ROOT="+dir)
	if out, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("PKI: %v\n%s", err, out)
	}
}

// TestRemotePKIWithoutCallerTMPDIR keeps the fixture valid outside CI's TMPDIR setting.
func TestRemotePKIWithoutCallerTMPDIR(t *testing.T) {
	t.Setenv("TMPDIR", "")
	_, pki := remotePKI(t, "bench")
	if _, err := os.Stat(filepath.Join(pki, "ca.pem")); err != nil {
		t.Fatalf("CA certificate: %v", err)
	}
}

// TestRemoteLauncher exercises Bash against authenticated TLS discovery and
// verifies refusals cannot substitute a stale or locally compiled identity.
func TestRemoteLauncher(t *testing.T) {
	testRemoteLauncher(t, "success", "describe_failure", "backend_mismatch", "missing_credential", "implicit_jog")
}

func TestReviewLauncherRejectsBindingPairMismatch(t *testing.T) {
	testRemoteLauncher(t, "pair_mismatch")
}

func TestReviewLauncherRequiresVerifiedDiscovery(t *testing.T) {
	testRemoteLauncher(t, "hostile_curlrc")
}

func TestReviewLauncherNativeJogSummary(t *testing.T) {
	root, _ := filepath.Abs("../../../..")
	cmd := exec.Command("bash", "-c", `source "$1/offline-programming/v1/start-offline-programming.sh"; jog_backend_summary`, "bash", root)
	cmd.Env = append(os.Environ(), "ROSIEOS_OLP_CACHE="+t.TempDir(), "OFFLINE_PROGRAMMING_EXECUTION_BACKEND=rt_core", "OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core")
	out, err := cmd.CombinedOutput()
	if err != nil || strings.TrimSpace(string(out)) != "olp-start: jog backend: rt_core" {
		t.Fatalf("summary: %v %s", err, out)
	}
}

func testRemoteLauncher(t *testing.T, scenarios ...string) {
	_, pki := remotePKI(t, "bench-rosie1420")
	cert, err := tls.LoadX509KeyPair(filepath.Join(pki, "server.pem"), filepath.Join(pki, "server-key.pem"))
	if err != nil {
		t.Fatal(err)
	}
	ca, err := os.ReadFile(filepath.Join(pki, "ca.pem"))
	if err != nil {
		t.Fatal(err)
	}
	roots := x509.NewCertPool()
	if !roots.AppendCertsFromPEM(ca) {
		t.Fatal("CA")
	}
	for _, scenario := range scenarios {
		t.Run(scenario, func(t *testing.T) {
			server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				if r.URL.Path != "/v1/describe" || len(r.TLS.PeerCertificates) < 1 || len(r.TLS.VerifiedChains) < 1 {
					t.Error("unauthenticated Describe")
				}
				if scenario == "describe_failure" {
					http.Error(w, "unavailable", http.StatusServiceUnavailable)
					return
				}
				backend := "ethercat"
				if scenario == "backend_mismatch" {
					backend = "simulation"
				}
				json.NewEncoder(w).Encode(map[string]any{"ok": true, "data": map[string]any{
					"backend": backend, "contract_version": 2, "axes": make([]any, 6),
					"machine_sha256": strings.Repeat("a", 64), "configuration_sha256": strings.Repeat("b", 64),
					"deployment_sha256": strings.Repeat("c", 64), "capabilities_digest": strings.Repeat("d", 64),
				}})
			}))
			server.TLS = &tls.Config{Certificates: []tls.Certificate{cert}, ClientCAs: roots, ClientAuth: tls.RequireAndVerifyClientCert}
			server.StartTLS()
			defer server.Close()
			root, _ := filepath.Abs("../../../..")
			build, err := os.MkdirTemp(filepath.Join(root, "rt-core/build"), "launcher-")
			if err != nil {
				t.Fatal(err)
			}
			defer os.RemoveAll(build)
			cfg := documentedRemoteBinding(t)
			cfg["address"] = server.URL
			if scenario == "pair_mismatch" {
				cfg["binding"].(map[string]any)["pair_id"] = "different-pair"
			}
			cfg["ca_file"], cfg["certificate_file"], cfg["key_file"] = filepath.Join(pki, "ca.pem"), filepath.Join(pki, "clients/olp-dev.pem"), filepath.Join(pki, "clients/olp-dev-key.pem")
			if scenario == "hostile_curlrc" {
				_, wrongPKI := remotePKI(t, "untrusted")
				cfg["ca_file"] = filepath.Join(wrongPKI, "ca.pem")
				curlHome := t.TempDir()
				if err := os.WriteFile(filepath.Join(curlHome, ".curlrc"), []byte("insecure\n"), 0600); err != nil {
					t.Fatal(err)
				}
				t.Setenv("CURL_HOME", curlHome)
			}
			cfg["binding"].(map[string]any)["machine_sha256"] = "stale"
			if scenario == "missing_credential" {
				cfg["key_file"] = filepath.Join(build, "missing")
			}
			raw, _ := json.Marshal(cfg)
			binding := filepath.Join(build, "remote.json")
			if err := os.WriteFile(binding, raw, 0600); err != nil {
				t.Fatal(err)
			}
			cmd := exec.Command("bash", "testdata/remote-launcher.sh", root, build, binding, scenario)
			if out, err := cmd.CombinedOutput(); err != nil {
				t.Fatalf("Bash regression: %v\n%s", err, out)
			}
		})
	}
}

// Execute the documented format in TestRemoteLauncher so the operator example
// cannot drift from the authenticated launcher contract.
func documentedRemoteBinding(t *testing.T) map[string]any {
	t.Helper()
	raw, err := os.ReadFile("../../docs/RT_CORE_EXECUTION.md")
	if err != nil {
		t.Fatal(err)
	}
	_, section, ok := strings.Cut(string(raw), "## Remote cell\n")
	if !ok {
		t.Fatal("missing Remote cell section")
	}
	_, section, ok = strings.Cut(section, "```json\n")
	if !ok {
		t.Fatal("missing remote JSON example")
	}
	example, _, ok := strings.Cut(section, "\n```")
	if !ok {
		t.Fatal("unterminated remote JSON example")
	}
	var cfg map[string]any
	if err := json.Unmarshal([]byte(example), &cfg); err != nil {
		t.Fatal(err)
	}
	return cfg
}

func TestRemoteCellDocumentation(t *testing.T) {
	cfg := documentedRemoteBinding(t)
	raw, err := json.Marshal(cfg)
	if err != nil {
		t.Fatal(err)
	}
	decoded, err := DecodeRTCoreConfig(raw)
	if err != nil {
		t.Fatalf("documented config: %v", err)
	}
	if decoded.AxisMask != 63 || decoded.Binding.PairID != "bench-rosie1420" || decoded.Binding.Revision != 1 || decoded.ExpectedBackend != "ethercat" {
		t.Fatalf("documented six-axis binding: %+v", decoded)
	}
	if decoded.Socket != "" || decoded.Binding.MachineSHA256 != "" || decoded.Binding.ConfigurationSHA256 != "" || decoded.DeploymentSHA256 != "" || decoded.CapabilitiesDigest != "" {
		t.Fatal("remote example must obtain all identity hashes from live Describe")
	}
}
