package simulator

import (
	"os"
	"path/filepath"
	"testing"
)

func repoRootForJogTest(t *testing.T) string {
	t.Helper()
	directory, err := os.Getwd()
	if err != nil {
		t.Fatal(err)
	}
	for {
		hasCadQuery := false
		hasTesseract := false
		if _, statErr := os.Stat(filepath.Join(directory, "cadquery", "v1")); statErr == nil {
			hasCadQuery = true
		}
		if _, statErr := os.Stat(filepath.Join(directory, "tesseract", "v1")); statErr == nil {
			hasTesseract = true
		}
		if hasCadQuery && hasTesseract {
			return directory
		}
		parent := filepath.Dir(directory)
		if parent == directory {
			t.Fatal("repo root not found")
		}
		directory = parent
	}
}
