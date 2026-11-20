package build

import (
	"archive/tar"
	"bytes"
	"compress/gzip"
	"encoding/base64"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

func TestSSHBuildPeerCacheScriptOrdersColdPublication(t *testing.T) {
	script := SSHBuildPeerCacheScript(
		`$HOME/.rosie/build-cache/robot-v4-ssh-artifacts/test-artifact`,
		"source-recipe",
		"test-artifact",
		`printf '\177ELF' > "$DAEMON_V1_BUILD_OUTPUT" && chmod 0755 "$DAEMON_V1_BUILD_OUTPUT"`,
	)

	ordered := []string{
		`mkdir -p "$CACHE_ROOT"`,
		`if test -x "$BINARY"`,
		`LOCK="$CACHE_ROOT/.lock-source-recipe"`,
		`exec 9>"$LOCK"`,
		`flock 9`,
		`hit-after-wait`,
		`cache_hit=0`,
		`WORK=$(mktemp -d "$CACHE_ROOT/.build.source-recipe.XXXXXX")`,
		`PUBLISH="$WORK/publish"`,
		`install -m 0755 "$DAEMON_V1_BUILD_OUTPUT" "$PUBLISH/test-artifact"`,
		`rm -rf "$CACHE"`,
		`mv "$PUBLISH" "$CACHE"`,
	}
	previous := -1
	for _, fragment := range ordered {
		index := strings.Index(script, fragment)
		if index < 0 {
			t.Fatalf("cache script missing %q:\n%s", fragment, script)
		}
		if index <= previous {
			t.Fatalf("cache script fragment %q is out of order:\n%s", fragment, script)
		}
		previous = index
	}
	if finalBase64 := strings.LastIndex(script, `base64 "$BINARY"`); finalBase64 <= previous {
		t.Fatalf("cache script does not return the atomically published binary last:\n%s", script)
	}
	for _, unsafe := range []string{`mkdir "$CACHE"`, `install -m 0755 "$DAEMON_V1_BUILD_OUTPUT" "$BINARY"`} {
		if strings.Contains(script, unsafe) {
			t.Fatalf("cache script retains non-atomic publication %q:\n%s", unsafe, script)
		}
	}
}

func TestSSHBuildPeerCacheScriptDrainsInputBeforeReturningCachedBinary(t *testing.T) {
	script := SSHBuildPeerCacheScript(
		`$HOME/.rosie/build-cache/robot-v4-ssh-artifacts/test-artifact`,
		"source-recipe",
		"test-artifact",
		`printf '\177ELF' > "$DAEMON_V1_BUILD_OUTPUT" && chmod 0755 "$DAEMON_V1_BUILD_OUTPUT"`,
	)

	for _, hit := range []string{"hit", "hit-after-wait"} {
		branch := `cat >/dev/null; echo "robot-v4 ssh build cache: ` + hit
		if !strings.Contains(script, branch) {
			t.Fatalf("cache %s branch does not drain streamed source before returning the binary:\n%s", hit, script)
		}
	}
}

func TestSSHBuildPeerCacheSerializesConcurrentColdPublication(t *testing.T) {
	if _, err := exec.LookPath("flock"); err != nil {
		t.Skip("flock is unavailable on this test host")
	}

	cacheRoot := filepath.Join(t.TempDir(), "cache")
	identity := "source-recipe"
	artifactName := "test-artifact"
	counterPath := filepath.Join(t.TempDir(), "build-count")
	cachePath := filepath.Join(cacheRoot, identity)
	if err := os.MkdirAll(cachePath, 0o755); err != nil {
		t.Fatal(err)
	}

	script := SSHBuildPeerCacheScript(
		cacheRoot,
		identity,
		artifactName,
		`printf x >> "$ROBOT_V4_TEST_BUILD_COUNTER"; sleep 1; printf '\177ELFcache' > "$DAEMON_V1_BUILD_OUTPUT"; chmod 0755 "$DAEMON_V1_BUILD_OUTPUT"`,
	)

	var source bytes.Buffer
	gz := gzip.NewWriter(&source)
	tw := tar.NewWriter(gz)
	payload := []byte("source")
	if err := tw.WriteHeader(&tar.Header{Name: "source.txt", Mode: 0o644, Size: int64(len(payload))}); err != nil {
		t.Fatal(err)
	}
	if _, err := tw.Write(payload); err != nil {
		t.Fatal(err)
	}
	if err := tw.Close(); err != nil {
		t.Fatal(err)
	}
	if err := gz.Close(); err != nil {
		t.Fatal(err)
	}

	type processResult struct {
		stdout bytes.Buffer
		stderr bytes.Buffer
		cmd    *exec.Cmd
	}
	newProcess := func() *processResult {
		result := &processResult{cmd: exec.Command("sh", "-c", script)}
		result.cmd.Env = append(os.Environ(), "ROBOT_V4_TEST_BUILD_COUNTER="+counterPath)
		result.cmd.Stdin = strings.NewReader(base64.StdEncoding.EncodeToString(source.Bytes()))
		result.cmd.Stdout = &result.stdout
		result.cmd.Stderr = &result.stderr
		return result
	}
	first := newProcess()
	second := newProcess()
	if err := first.cmd.Start(); err != nil {
		t.Fatal(err)
	}
	if err := second.cmd.Start(); err != nil {
		_ = first.cmd.Process.Kill()
		t.Fatal(err)
	}
	if err := first.cmd.Wait(); err != nil {
		t.Fatalf("first cache request: %v\n%s", err, first.stderr.String())
	}
	if err := second.cmd.Wait(); err != nil {
		t.Fatalf("second cache request: %v\n%s", err, second.stderr.String())
	}

	for index, output := range [][]byte{first.stdout.Bytes(), second.stdout.Bytes()} {
		binary, err := decodeWhitespaceBase64(output)
		if err != nil {
			t.Fatalf("request %d returned invalid base64: %v", index+1, err)
		}
		if !bytes.HasPrefix(binary, []byte{'\x7f', 'E', 'L', 'F'}) {
			t.Fatalf("request %d returned non-ELF bytes %q", index+1, binary)
		}
	}
	buildCount, err := os.ReadFile(counterPath)
	if err != nil {
		t.Fatal(err)
	}
	if string(buildCount) != "x" {
		t.Fatalf("cold concurrent requests built %d times, want 1", len(buildCount))
	}
	if _, err := os.Stat(filepath.Join(cachePath, artifactName)); err != nil {
		t.Fatalf("atomically published artifact is missing: %v", err)
	}
	if _, err := os.Stat(filepath.Join(cachePath, "publish")); !os.IsNotExist(err) {
		t.Fatalf("legacy incomplete cache nested the publication: err=%v", err)
	}
	combinedDiagnostics := first.stderr.String() + second.stderr.String()
	if strings.Count(combinedDiagnostics, "cache_hit=0") != 1 || strings.Count(combinedDiagnostics, "cache_hit=1") != 1 {
		t.Fatalf("unexpected concurrent cache diagnostics:\n%s", combinedDiagnostics)
	}
	if !strings.Contains(combinedDiagnostics, "recovering incomplete publication") {
		t.Fatalf("stale cache recovery was not reported:\n%s", combinedDiagnostics)
	}
}

func decodeWhitespaceBase64(raw []byte) ([]byte, error) {
	return base64.StdEncoding.DecodeString(strings.Join(strings.Fields(string(raw)), ""))
}
