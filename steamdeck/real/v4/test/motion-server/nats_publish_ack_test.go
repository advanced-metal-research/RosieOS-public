package motion_server

import (
	"bufio"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
	"time"
)

const motionServerNATSPublishTestSubject = "robot/v4/test.motion-server-publish"
const motionServerNATSPublishTestPayload = `{"command":"stop"}`

func buildMotionServerNATSPublishHarness(t *testing.T, repoRoot string) string {
	t.Helper()
	if runtime.GOOS == "windows" {
		t.Skip("motion-server C++ NATS publish harness runs on macOS/Linux")
	}

	workDir := t.TempDir()
	harnessPath := filepath.Join(workDir, "nats_publish_harness.cpp")
	binaryPath := filepath.Join(workDir, "nats-publish-harness")
	const harnessSource = `
#define main robot_v4_cartesiand_original_main
#include "robot_v4_cartesian_daemon.cpp"
#undef main

int main(int argc, char** argv) {
  if (argc == 3 && std::string(argv[1]) == "--http-get-test") {
    std::string body;
    if (!http_get_text(argv[2], &body)) {
      return 1;
    }
    std::cout << body;
    return 0;
  }
  if (argc == 4 &&
      std::string(argv[1]) == "--resolve-daemon-nats-test") {
    std::string target;
    if (!resolve_daemon_nats_service_ref_uncached(argv[3], argv[2], &target)) {
      return 1;
    }
    std::cout << target;
    return 0;
  }
  if (argc == 4 &&
      std::string(argv[1]) == "--resolve-daemon-udp-test") {
    std::string target;
    if (!resolve_daemon_udp_service_ref(argv[3], argv[2], &target)) {
      return 1;
    }
    std::cout << target;
    return 0;
  }
  if (argc != 4) {
    return 2;
  }
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif
  return publish_payload_to_nats(argv[1], "", argv[2], argv[3], "test") ? 0 : 1;
}
`
	if err := os.WriteFile(harnessPath, []byte(harnessSource), 0o644); err != nil {
		t.Fatalf("write motion-server NATS publish harness: %v", err)
	}
	cxx := strings.TrimSpace(os.Getenv("CXX"))
	if cxx == "" {
		cxx = "c++"
	}
	sourceDir := filepath.Join(repoRoot, "motion-server", "v1", "src")
	cmd := exec.Command(cxx,
		"-std=c++17",
		"-O2",
		"-Wall",
		"-Wextra",
		"-Wpedantic",
		"-I", sourceDir,
		"-o", binaryPath,
		harnessPath,
		"-pthread",
	)
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("build motion-server NATS publish harness: %v\n%s", err, string(output))
	}
	return binaryPath
}

func writeMotionServerNATSFragments(conn net.Conn, fragments ...string) error {
	for index, fragment := range fragments {
		if _, err := io.WriteString(conn, fragment); err != nil {
			return err
		}
		if index+1 < len(fragments) {
			time.Sleep(5 * time.Millisecond)
		}
	}
	return nil
}

func readMotionServerNATSPublishRequest(reader *bufio.Reader, payload string) error {
	connectLine, err := reader.ReadString('\n')
	if err != nil {
		return fmt.Errorf("read CONNECT: %w", err)
	}
	if !strings.HasPrefix(connectLine, "CONNECT ") || !strings.HasSuffix(connectLine, "\r\n") {
		return fmt.Errorf("unexpected CONNECT line %q", connectLine)
	}

	wantHeader := fmt.Sprintf("PUB %s %d\r\n",
		motionServerNATSPublishTestSubject, len(payload))
	publishLine, err := reader.ReadString('\n')
	if err != nil {
		return fmt.Errorf("read PUB header: %w", err)
	}
	if publishLine != wantHeader {
		return fmt.Errorf("PUB header = %q, want %q", publishLine, wantHeader)
	}

	body := make([]byte, len(payload)+2)
	if _, err := io.ReadFull(reader, body); err != nil {
		return fmt.Errorf("read PUB body: %w", err)
	}
	if got, want := string(body), payload+"\r\n"; got != want {
		return fmt.Errorf("PUB body = %q, want %q", got, want)
	}

	pingLine, err := reader.ReadString('\n')
	if err != nil {
		return fmt.Errorf("read client PING: %w", err)
	}
	if pingLine != "PING\r\n" {
		return fmt.Errorf("client protocol barrier = %q, want PING", pingLine)
	}
	return nil
}

func runMotionServerNATSPublishCase(
	t *testing.T,
	binary string,
	payload string,
	wantSuccess bool,
	handler func(net.Conn) error,
) time.Duration {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("start fake NATS listener: %v", err)
	}
	defer listener.Close()

	serverDone := make(chan error, 1)
	go func() {
		conn, acceptErr := listener.Accept()
		if acceptErr != nil {
			serverDone <- acceptErr
			return
		}
		defer conn.Close()
		_ = conn.SetDeadline(time.Now().Add(2 * time.Second))
		serverDone <- handler(conn)
	}()

	startedAt := time.Now()
	cmd := exec.Command(binary,
		"nats://"+listener.Addr().String(),
		motionServerNATSPublishTestSubject,
		payload,
	)
	output, runErr := cmd.CombinedOutput()
	elapsed := time.Since(startedAt)
	if gotSuccess := runErr == nil; gotSuccess != wantSuccess {
		t.Fatalf("publish success = %t, want %t; err=%v output=%s",
			gotSuccess, wantSuccess, runErr, string(output))
	}
	if !wantSuccess && cmd.ProcessState.ExitCode() != 1 {
		t.Fatalf("rejected publish exit code = %d, want 1; output=%s",
			cmd.ProcessState.ExitCode(), string(output))
	}

	select {
	case serverErr := <-serverDone:
		if serverErr != nil {
			t.Fatalf("fake NATS server: %v", serverErr)
		}
	case <-time.After(3 * time.Second):
		t.Fatal("timed out waiting for fake NATS server")
	}
	return elapsed
}

func TestRobotV4MotionServerNATSPublishRequiresPONGDryRun(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := buildMotionServerNATSPublishHarness(t, repoRoot)

	t.Run("fragmented PING and PONG accepted", func(t *testing.T) {
		runMotionServerNATSPublishCase(t, binary, motionServerNATSPublishTestPayload, true, func(conn net.Conn) error {
			if err := writeMotionServerNATSFragments(conn,
				"IN", `FO {"server_id":"motion-server-pong-test","max_payload":1048576}`, "\r", "\n"); err != nil {
				return fmt.Errorf("write fragmented INFO: %w", err)
			}
			reader := bufio.NewReader(conn)
			if err := readMotionServerNATSPublishRequest(reader, motionServerNATSPublishTestPayload); err != nil {
				return err
			}
			if err := writeMotionServerNATSFragments(conn, "PI", "NG\r", "\n"); err != nil {
				return fmt.Errorf("write fragmented server PING: %w", err)
			}
			pongLine, err := reader.ReadString('\n')
			if err != nil {
				return fmt.Errorf("read client PONG: %w", err)
			}
			if pongLine != "PONG\r\n" {
				return fmt.Errorf("server PING response = %q, want PONG", pongLine)
			}
			if err := writeMotionServerNATSFragments(conn,
				"+O", "K\r\n", "PO", "NG\r", "\n"); err != nil {
				return fmt.Errorf("write fragmented publish PONG: %w", err)
			}
			return nil
		})
	})

	t.Run("fragmented error before PONG rejected", func(t *testing.T) {
		runMotionServerNATSPublishCase(t, binary, motionServerNATSPublishTestPayload, false, func(conn net.Conn) error {
			if _, err := io.WriteString(conn, "INFO {\"max_payload\":1048576}\r\n"); err != nil {
				return err
			}
			if err := readMotionServerNATSPublishRequest(bufio.NewReader(conn), motionServerNATSPublishTestPayload); err != nil {
				return err
			}
			if _, err := io.WriteString(conn, "-ER"); err != nil {
				return err
			}
			time.Sleep(5 * time.Millisecond)
			_, err := io.WriteString(conn, "R 'Permissions Violation'\r\nPONG\r\n")
			return err
		})
	})

	t.Run("EOF before PONG rejected", func(t *testing.T) {
		runMotionServerNATSPublishCase(t, binary, motionServerNATSPublishTestPayload, false, func(conn net.Conn) error {
			if _, err := io.WriteString(conn, "INFO {\"max_payload\":1048576}\r\n"); err != nil {
				return err
			}
			return readMotionServerNATSPublishRequest(bufio.NewReader(conn), motionServerNATSPublishTestPayload)
		})
	})

	t.Run("missing PONG rejected on bounded deadline", func(t *testing.T) {
		elapsed := runMotionServerNATSPublishCase(t, binary, motionServerNATSPublishTestPayload, false, func(conn net.Conn) error {
			if _, err := io.WriteString(conn, "INFO {\"max_payload\":1048576}\r\n"); err != nil {
				return err
			}
			if err := readMotionServerNATSPublishRequest(bufio.NewReader(conn), motionServerNATSPublishTestPayload); err != nil {
				return err
			}
			if _, err := io.WriteString(conn, "+OK\r\n"); err != nil {
				return err
			}
			time.Sleep(650 * time.Millisecond)
			return nil
		})
		if elapsed < 300*time.Millisecond || elapsed > 2*time.Second {
			t.Fatalf("no-PONG deadline elapsed %s, want 300ms..2s", elapsed)
		}
	})

	t.Run("large plan payload gets extended acknowledgement deadline", func(t *testing.T) {
		payload := strings.Repeat("x", 96*1024)
		elapsed := runMotionServerNATSPublishCase(t, binary, payload, true, func(conn net.Conn) error {
			if _, err := io.WriteString(conn, "INFO {\"max_payload\":1048576}\r\n"); err != nil {
				return err
			}
			if err := readMotionServerNATSPublishRequest(bufio.NewReader(conn), payload); err != nil {
				return err
			}
			time.Sleep(650 * time.Millisecond)
			_, err := io.WriteString(conn, "PONG\r\n")
			return err
		})
		if elapsed < 600*time.Millisecond || elapsed > 2*time.Second {
			t.Fatalf("large publish elapsed %s, want 600ms..2s", elapsed)
		}
	})
}

func TestRobotV4MotionServerDaemonStateHTTPReadIsBounded(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := buildMotionServerNATSPublishHarness(t, repoRoot)

	t.Run("content length completes without waiting for server close", func(t *testing.T) {
		listener, err := net.Listen("tcp", "127.0.0.1:0")
		if err != nil {
			t.Fatalf("start fake daemon-state listener: %v", err)
		}
		defer listener.Close()
		serverDone := make(chan error, 1)
		go func() {
			conn, acceptErr := listener.Accept()
			if acceptErr != nil {
				serverDone <- acceptErr
				return
			}
			defer conn.Close()
			_ = conn.SetDeadline(time.Now().Add(3 * time.Second))
			reader := bufio.NewReader(conn)
			for {
				line, readErr := reader.ReadString('\n')
				if readErr != nil {
					serverDone <- readErr
					return
				}
				if line == "\r\n" {
					break
				}
			}
			body := `{"nodes":[]}`
			_, writeErr := fmt.Fprintf(conn,
				"HTTP/1.1 200 OK\r\nContent-Length: %d\r\nConnection: keep-alive\r\n\r\n%s",
				len(body), body)
			if writeErr != nil {
				serverDone <- writeErr
				return
			}
			// Keep the response socket open. The client must complete from the
			// declared body length instead of waiting for EOF.
			one := make([]byte, 1)
			_, readErr := conn.Read(one)
			if readErr != io.EOF {
				serverDone <- fmt.Errorf("wait for client close: %v", readErr)
				return
			}
			serverDone <- nil
		}()

		startedAt := time.Now()
		output, runErr := exec.Command(binary, "--http-get-test",
			"http://"+listener.Addr().String()+"/api/state").CombinedOutput()
		elapsed := time.Since(startedAt)
		if runErr != nil {
			t.Fatalf("bounded daemon-state HTTP read failed: %v output=%s", runErr, string(output))
		}
		if got, want := string(output), `{"nodes":[]}`; got != want {
			t.Fatalf("daemon-state body = %q, want %q", got, want)
		}
		if elapsed > 750*time.Millisecond {
			t.Fatalf("daemon-state content-length read took %s, want <=750ms", elapsed)
		}
		select {
		case serverErr := <-serverDone:
			if serverErr != nil {
				t.Fatalf("fake daemon-state server: %v", serverErr)
			}
		case <-time.After(2 * time.Second):
			t.Fatal("daemon-state client did not close after Content-Length body")
		}
	})

	t.Run("silent server fails within socket deadline", func(t *testing.T) {
		listener, err := net.Listen("tcp", "127.0.0.1:0")
		if err != nil {
			t.Fatalf("start silent daemon-state listener: %v", err)
		}
		defer listener.Close()
		serverDone := make(chan struct{}, 1)
		go func() {
			conn, acceptErr := listener.Accept()
			if acceptErr == nil {
				defer conn.Close()
				_ = conn.SetReadDeadline(time.Now().Add(3 * time.Second))
				_, _ = io.Copy(io.Discard, conn)
			}
			serverDone <- struct{}{}
		}()
		startedAt := time.Now()
		output, runErr := exec.Command(binary, "--http-get-test",
			"http://"+listener.Addr().String()+"/api/state").CombinedOutput()
		elapsed := time.Since(startedAt)
		if runErr == nil {
			t.Fatalf("silent daemon-state server unexpectedly succeeded: %s", string(output))
		}
		if elapsed < 750*time.Millisecond || elapsed > 2*time.Second {
			t.Fatalf("silent daemon-state deadline took %s, want 750ms..2s", elapsed)
		}
		select {
		case <-serverDone:
		case <-time.After(2 * time.Second):
			t.Fatal("silent daemon-state connection remained open after deadline")
		}
	})
}

func TestRobotV4MotionServerDaemonServicePeerIdentityIsCaseInsensitive(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := buildMotionServerNATSPublishHarness(t, repoRoot)
	const serviceRef = "daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp"
	const body = `{"nodes":[{"peer":"other-bench-servo","role":"realtime-core","source_addr":"100.64.0.11:55796","advertised_addresses":["192.168.58.11"],"service_endpoints":[{"name":"robot-v4-rtcore-udp","kind":"robot-hot-path","protocol":"udp","url":"daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp","listen":"0.0.0.0:49004","status":"running","advertised_by":"other-bench-servo"}]}]}`
	server := httptest.NewServer(http.HandlerFunc(func(writer http.ResponseWriter, request *http.Request) {
		if request.URL.Path != "/api/state" {
			http.NotFound(writer, request)
			return
		}
		_, _ = io.WriteString(writer, body)
	}))
	defer server.Close()

	output, err := exec.Command(
		binary,
		"--resolve-daemon-udp-test",
		server.URL+"/api/state",
		serviceRef,
	).CombinedOutput()
	if err != nil {
		t.Fatalf("resolve mixed-case daemon peer identity: %v output=%s", err, string(output))
	}
	if got, want := string(output), "100.64.0.11:49004"; got != want {
		t.Fatalf("resolved daemon UDP target = %q, want %q", got, want)
	}
}

func TestRobotV4MotionServerDaemonEndpointIdentityIsCaseInsensitive(t *testing.T) {
	binary := buildMotionServerNATSPublishHarness(t, findRepoRoot(t))
	const ref = "daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp"
	for _, tc := range []struct {
		name, provider, endpoint, role, service string
		accepted                                bool
	}{
		{"observed", "other-bench-servo", "daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp", "realtime-core", "robot-v4-rtcore-udp", true},
		{"provider-case", "other-bench-servo", ref, "realtime-core", "robot-v4-rtcore-udp", true},
		{"other-provider", "bench-servo", ref, "realtime-core", "robot-v4-rtcore-udp", false},
		{"other-url-peer", "other-bench-servo", "daemon-v1://bench-servo/realtime-core/robot-v4-rtcore-udp", "realtime-core", "robot-v4-rtcore-udp", false},
		{"other-url-role", "other-bench-servo", "daemon-v1://other-bench-servo/motion-server/robot-v4-rtcore-udp", "realtime-core", "robot-v4-rtcore-udp", false},
		{"role-case-is-not-host-case", "other-bench-servo", "daemon-v1://other-bench-servo/Realtime-Core/robot-v4-rtcore-udp", "realtime-core", "robot-v4-rtcore-udp", false},
		{"malformed-url", "other-bench-servo", "daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp/extra", "realtime-core", "robot-v4-rtcore-udp", false},
		{"other-url-service", "other-bench-servo", "daemon-v1://other-bench-servo/realtime-core/other", "realtime-core", "robot-v4-rtcore-udp", false},
		{"other-node-role", "other-bench-servo", ref, "motion-server", "robot-v4-rtcore-udp", false},
		{"other-endpoint-name", "other-bench-servo", ref, "realtime-core", "other", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := fmt.Sprintf(`{"nodes":[{"peer":"other-bench-servo","node":"other-bench-servo","role":%q,"source_addr":"100.64.0.11:55796","service_endpoints":[{"name":%q,"protocol":"udp","url":%q,"listen":"0.0.0.0:49004","status":"running","advertised_by":%q}]}]}`, tc.role, tc.service, tc.endpoint, tc.provider)
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) { _, _ = io.WriteString(w, body) }))
			defer server.Close()
			out, err := exec.Command(binary, "--resolve-daemon-udp-test", server.URL+"/api/state", ref).CombinedOutput()
			if tc.accepted {
				if err != nil || string(out) != "100.64.0.11:49004" {
					t.Fatalf("resolve: %v output=%s", err, out)
				}
			} else if err == nil {
				t.Fatalf("accepted mismatched endpoint: %s", out)
			}
		})
	}
}

func TestRobotV4MotionServerNATSPhysicalEndpointIdentity(t *testing.T) {
	binary := buildMotionServerNATSPublishHarness(t, findRepoRoot(t))
	const ref = "daemon-v1://other-bench-servo/realtime-core/robot-v4-nats"
	for _, tc := range []struct {
		name, peer, role, provider, endpoint string
		accepted                             bool
	}{
		{"mixed-physical-peer", "other-bench-servo", "other-bench-servo", "other-bench-servo", ref, true},
		{"mixed-url-provider", "other-bench-servo", "other-bench-servo", "other-bench-servo", "daemon-v1://other-bench-servo/realtime-core/robot-v4-nats", true},
		{"mixed-host-role", "other-bench-servo", "other-bench-servo", "other-bench-servo", ref, true},
		{"other-peer", "bench-servo", "bench-servo", "other-bench-servo", ref, false},
		{"other-provider", "other-bench-servo", "other-bench-servo", "bench-servo", ref, false},
		{"other-url-peer", "other-bench-servo", "other-bench-servo", "other-bench-servo", "daemon-v1://bench-servo/realtime-core/robot-v4-nats", false},
		{"other-url-role", "other-bench-servo", "other-bench-servo", "other-bench-servo", "daemon-v1://other-bench-servo/motion-server/robot-v4-nats", false},
		{"other-url-service", "other-bench-servo", "other-bench-servo", "other-bench-servo", "daemon-v1://other-bench-servo/realtime-core/other", false},
		{"other-host-role", "other-bench-servo", "other-relay", "other-bench-servo", ref, false},
		{"semantic-peer", "realtime-core", "realtime-core", "other-bench-servo", ref, false},
		{"missing-provider", "other-bench-servo", "other-bench-servo", "", ref, false},
		{"missing-url", "other-bench-servo", "other-bench-servo", "other-bench-servo", "", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := fmt.Sprintf(`{"nodes":[{"peer":%q,"role":%q,"source_addr":"100.64.0.11:55796","service_endpoints":[{"name":"robot-v4-nats","protocol":"nats","url":%q,"listen":"0.0.0.0:4222","advertised_by":%q}]}]}`, tc.peer, tc.role, tc.endpoint, tc.provider)
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				if r.URL.Path != "/nodes" {
					http.NotFound(w, r)
					return
				}
				_, _ = io.WriteString(w, body)
			}))
			defer server.Close()
			out, err := exec.Command(binary, "--resolve-daemon-nats-test", server.URL+"/api/state", ref).CombinedOutput()
			if tc.accepted {
				if err != nil || string(out) != "nats://100.64.0.11:4222" {
					t.Fatalf("resolve: %v output=%s", err, out)
				}
			} else if err == nil {
				t.Fatalf("accepted mismatched endpoint: %s", out)
			}
		})
	}
}

func TestRobotV4MotionServerRemoteLocalProjectionDoesNotResolveToLoopback(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := buildMotionServerNATSPublishHarness(t, repoRoot)
	const serviceRef = "daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp"
	const body = `{"health":{"host":"other-relay","node":"other-relay"},"nodes":[{"peer":"other-bench-servo","role":"realtime-core","source_addr":"local","advertised_addresses":["192.168.58.20"],"service_endpoints":[{"name":"robot-v4-rtcore-udp","kind":"robot-hot-path","protocol":"udp","url":"daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp","listen":"0.0.0.0:49004","status":"running","advertised_by":"other-bench-servo"}]}]}`
	server := httptest.NewServer(http.HandlerFunc(func(writer http.ResponseWriter, request *http.Request) {
		if request.URL.Path != "/api/state" {
			http.NotFound(writer, request)
			return
		}
		_, _ = io.WriteString(writer, body)
	}))
	defer server.Close()

	output, err := exec.Command(
		binary,
		"--resolve-daemon-udp-test",
		server.URL+"/api/state",
		serviceRef,
	).CombinedOutput()
	if err != nil {
		t.Fatalf("resolve remote local projection: %v output=%s", err, string(output))
	}
	if got, want := string(output), "192.168.58.20:49004"; got != want {
		t.Fatalf("resolved remote UDP target = %q, want %q", got, want)
	}
}

func TestRobotV4MotionServerLocalPeerStillResolvesToLoopback(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := buildMotionServerNATSPublishHarness(t, repoRoot)
	const serviceRef = "daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp"
	const body = `{"health":{"host":"other-bench-servo","node":"other-bench-servo"},"nodes":[{"peer":"other-bench-servo","role":"realtime-core","source_addr":"local","advertised_addresses":["192.168.58.20"],"service_endpoints":[{"name":"robot-v4-rtcore-udp","kind":"robot-hot-path","protocol":"udp","url":"daemon-v1://other-bench-servo/realtime-core/robot-v4-rtcore-udp","listen":"0.0.0.0:49004","status":"running","advertised_by":"other-bench-servo"}]}]}`
	server := httptest.NewServer(http.HandlerFunc(func(writer http.ResponseWriter, request *http.Request) {
		if request.URL.Path != "/api/state" {
			http.NotFound(writer, request)
			return
		}
		_, _ = io.WriteString(writer, body)
	}))
	defer server.Close()

	output, err := exec.Command(
		binary,
		"--resolve-daemon-udp-test",
		server.URL+"/api/state",
		serviceRef,
	).CombinedOutput()
	if err != nil {
		t.Fatalf("resolve local peer: %v output=%s", err, string(output))
	}
	if got, want := string(output), "127.0.0.1:49004"; got != want {
		t.Fatalf("resolved local UDP target = %q, want %q", got, want)
	}
}
