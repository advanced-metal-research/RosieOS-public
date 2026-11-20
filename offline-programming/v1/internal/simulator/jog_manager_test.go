package simulator

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestCanonicalJogBinaryCandidatesAreMotionServerNotTesseract(t *testing.T) {
	repo := t.TempDir()
	candidates := CanonicalJogBinaryCandidates(repo)
	if len(candidates) == 0 {
		t.Fatal("expected jog binary candidates")
	}
	want := filepath.Join(repo, filepath.FromSlash(defaultCanonicalBinary))
	if candidates[0] != want {
		t.Fatalf("first candidate = %q, want %q", candidates[0], want)
	}
	joined := strings.Join(candidates, "\n")
	if !strings.Contains(joined, filepath.FromSlash("motion-server/v1/bin/robot-v4-cartesiand")) {
		t.Fatalf("candidates missing motion-server/v1: %s", joined)
	}
	if strings.Contains(joined, filepath.FromSlash("tesseract/v1")) {
		t.Fatalf("jog binary search must not include tesseract/v1: %s", joined)
	}
	if strings.Contains(joined, filepath.FromSlash("robot/v4/motion-server")) {
		t.Fatalf("jog binary search must not use robot/v4/motion-server: %s", joined)
	}
}

func TestResolveCanonicalJogBinaryFindsRepoELF(t *testing.T) {
	root := t.TempDir()
	binary := filepath.Join(root, filepath.FromSlash(defaultCanonicalBinary))
	if err := os.MkdirAll(filepath.Dir(binary), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(binary, []byte{0x7f, 'E', 'L', 'F', 0x02}, 0o755); err != nil {
		t.Fatal(err)
	}
	resolved, err := ResolveCanonicalJogBinary(root)
	if err != nil {
		t.Fatalf("resolve: %v", err)
	}
	if resolved != binary {
		linked, linkErr := filepath.EvalSymlinks(binary)
		if linkErr != nil || resolved != linked {
			t.Fatalf("resolved %q, want %q", resolved, binary)
		}
	}
}

func TestJogManagerStartMissingBinaryNamesCartesiand(t *testing.T) {
	manager := NewJogManager(t.TempDir())
	_, err := manager.Start()
	if err == nil {
		t.Fatal("expected missing-binary error")
	}
	message := err.Error()
	if !strings.Contains(message, "robot-v4-cartesiand") {
		t.Fatalf("error %q should name robot-v4-cartesiand", message)
	}
	if strings.Contains(strings.ToLower(message), "tesseract/v1") {
		t.Fatalf("jog start must not mention tesseract/v1: %q", message)
	}
}
