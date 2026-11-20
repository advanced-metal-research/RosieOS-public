package main

import (
	"encoding/base64"

	"io"

	"os/exec"

	"strings"
)

func posixShellQuote(value string) string {
	return "'" + strings.ReplaceAll(value, "'", "'\"'\"'") + "'"
}

func runCommandWithStdinCapture(dir, stdin string, stderr io.Writer, name string, args ...string) ([]byte, error) {
	cmd := exec.Command(name, args...)
	cmd.Dir = dir
	cmd.Stdin = strings.NewReader(stdin)
	cmd.Stderr = stderr
	return cmd.Output()
}

func decodeWhitespaceBase64(raw []byte) ([]byte, error) {
	text := strings.Map(func(r rune) rune {
		if r == '\n' || r == '\r' || r == '\t' || r == ' ' {
			return -1
		}
		return r
	}, string(raw))
	return base64.StdEncoding.DecodeString(text)
}

func firstNonEmpty(values ...string) string {
	for _, value := range values {
		if strings.TrimSpace(value) != "" {
			return strings.TrimSpace(value)
		}
	}
	return ""
}

var _ io.Writer
