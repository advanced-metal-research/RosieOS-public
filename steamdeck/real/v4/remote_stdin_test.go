package main

import (
	"bytes"
	"strings"
	"testing"
)

// A relay script over the command-line limit still runs, whole and once: it
// goes on stdin, a command in it that reads stdin cannot eat its tail, and its
// quotes survive. 1.5 MB cannot be exec'd as one argument on Linux (128 KiB
// per argument) or macOS (1 MiB in total), so the command-line form fails here.
func TestRemoteLargeScriptGoesOnStdin(t *testing.T) {
	var body strings.Builder
	for body.Len() < 1500*1024 {
		body.WriteString("# padding padding padding padding padding padding padding padding\n")
	}
	script := body.String() + "cat\necho \"quote:'x' tail-ok $(( 6 * 7 ))\"\n"
	remote := "bash -c " + shellQuote(script)
	argv, got, ok := remoteScriptViaStdin([]string{"tailscale", "ssh", "u@h", remote})
	if !ok || got != script || len(argv[len(argv)-1]) > 200 || argv[2] != "u@h" {
		t.Fatalf("not rewritten: ok=%t len=%d argv=%q", ok, len(got), argv[:3])
	}
	var out bytes.Buffer
	code, err := rtCoreRunRemoteProcess([]string{"bash", "-c", `eval "$1"`, "_", remote}, newRTCoreProgress(&out, nil))
	if err != nil || code != 0 || !strings.Contains(out.String(), "quote:'x' tail-ok 42") {
		t.Fatalf("code %d err %v output:\n%s", code, err, out.String())
	}
}

// Scripts under the limit keep the command-line form byte for byte.
func TestRemoteSmallScriptStaysOnCommandLine(t *testing.T) {
	argv := []string{"tailscale", "ssh", "u@h", "bash -c " + shellQuote("echo 'small'")}
	got, _, ok := remoteScriptViaStdin(argv)
	if ok || strings.Join(got, "\x00") != strings.Join(argv, "\x00") {
		t.Fatalf("small script rewritten: %q", got)
	}
}
