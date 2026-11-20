package rtctl

import (
	"bytes"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// Unverified CLI fixture: independent raw register observations from the owner
// sequence. No hardware is accessed; every unexpected command fails the mock.
const encoderMock = `#!/bin/sh
set -eu
echo "$*" >> "$ENCODER_LOG"
stage=$(cat "$ENCODER_STAGE")
case "$1" in
master)
  echo Master0
  if [ "$ENCODER_REFUSE" = owner ]; then echo '  Phase: Operation'; else echo '  Phase: Idle'; fi
  echo '  Active: no';;
slaves)
  echo '=== Master 0, Slave 0 ==='
  echo '  State: PREOP'
  echo '  Vendor Id: 0x00400000'
  echo '  Product code: 0x00000715'
  echo '  Revision number: 0x00002ef8';;
upload)
  [ "$2 $3 $4 $5 $6" = '--master 0 --position 0 --type' ]
  case "$8:$9" in
  0x603f:0)
    if [ "$stage" = 3 ]; then echo '0x0000 0'; else echo '0x7305 29445'; fi;;
  0x203f:0)
    [ "$7" = octet_string ]
    if [ "$stage" = 1 ] && [ "$ENCODER_REFUSE" = alarm ]; then printf '\020\017'
    elif [ "$stage" = 3 ]; then printf '\000\000'
    elif [ "$stage" = 2 ]; then printf '\220\017\000\000'
    else printf '\010\002'; fi;;
  0x2031:0x11) echo '0x0000 0';;
  0x6041:0) echo '0x0040 64';;
  *) exit 71;;
  esac;;
download)
  case "$stage:$8:$9:$10" in
  0:0x2031:0x11:4) echo 1 > "$ENCODER_STAGE";;
  1:0x2031:0x02:1) echo 2 > "$ENCODER_STAGE";;
  2:0x2031:0x01:1) echo 3 > "$ENCODER_STAGE";;
  *) exit 72;;
  esac;;
*) exit 73;;
esac
`

// TestRecoverEncoderCLI asserts the observed write/read sequence and refusal
// boundaries through a real subprocess mock of the ethercat executable.
func TestRecoverEncoderCLI(t *testing.T) {
	for _, refusal := range []string{"none", "alarm", "owner"} {
		t.Run(refusal, func(t *testing.T) {
			dir := buildTemp(t)
			writeTestFile(t, filepath.Join(dir, "ethercat"), strings.ReplaceAll(encoderMock, "$10", "${10}"), 0o755)
			stage, log := filepath.Join(dir, "stage"), filepath.Join(dir, "commands")
			writeTestFile(t, stage, "0\n", 0o644)
			t.Setenv("PATH", dir+string(os.PathListSeparator)+os.Getenv("PATH"))
			t.Setenv("ENCODER_STAGE", stage)
			t.Setenv("ENCODER_LOG", log)
			t.Setenv("ENCODER_REFUSE", refusal)
			var stdout, stderr bytes.Buffer
			code := RunCLI([]string{"recover-encoder", "--slave", "0", "--backend", "ethercat"}, testRoot(t), &stdout, &stderr)
			commands, err := os.ReadFile(log)
			if err != nil {
				t.Fatal(err)
			}
			count := strings.Count(string(commands), "download ")
			switch refusal {
			case "none":
				if code != 0 || count != 3 || !strings.Contains(stdout.String(), "recovery complete. Home is required") {
					t.Fatalf("code=%d writes=%d stdout=%s stderr=%s", code, count, &stdout, &stderr)
				}
				for _, observation := range []string{"initial: 0x603F=0x7305 0x203F=0x0208", "F31.10: 0x603F=0x7305 0x203F=0x0208", "F31.01 PREOP: 0x603F=0x7305 0x203F=0x0F90", "F31.00: 0x603F=0x0000 0x203F=0x0000"} {
					if !strings.Contains(stdout.String(), observation) {
						t.Errorf("missing %s", observation)
					}
				}
			case "alarm":
				if code != 1 || count != 1 || !strings.Contains(stderr.String(), "unexpected alarm after F31.10") {
					t.Fatalf("%d %d %s", code, count, &stderr)
				}
			case "owner":
				if code != 1 || count != 0 || !strings.Contains(stderr.String(), "owned") {
					t.Fatalf("%d %d %s", code, count, &stderr)
				}
			}
		})
	}
}

func TestRecoverEncoderArguments(t *testing.T) {
	for _, args := range [][]string{nil, {"--slave", "-1", "--backend", "ethercat"}, {"--slave", "65536", "--backend", "ethercat"}, {"--slave", "0", "--backend", "simulation"}} {
		var stdout, stderr bytes.Buffer
		if code := RunRecoverEncoderCLI(args, &stdout, &stderr); code != 2 {
			t.Fatalf("%v code=%d", args, code)
		}
	}
}
