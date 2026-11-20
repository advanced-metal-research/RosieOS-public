package rtctl

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
)

// Tokens that may appear in ROSIE_RT_CORE_ARGS. The value is double-quoted in
// the environment file and systemd splits the unbraced $ROSIE_RT_CORE_ARGS on
// whitespace only; '|' and ';' (the cell I/O argv separators) are literal there
// and inside shell double quotes. A token of separators alone is still refused
// (a lone ';' word is ExecStart's command separator in unit syntax).
var (
	environmentToken = regexp.MustCompile(`^[-A-Za-z0-9_./,:+|;]*[-A-Za-z0-9_./,:+][-A-Za-z0-9_./,:+|;]*$`)
)

// runControlEnv emits a compiler-bound systemd environment for the installer.
// Validation precedes publication so malformed bindings cannot replace control.env.
func runControlEnv(args []string, root string, stdout, stderr io.Writer) int {
	flags := flag.NewFlagSet("control-env", flag.ContinueOnError)
	flags.SetOutput(stderr)
	config := flags.String("config", "", "live machine JSON file")
	pair := flags.String("pair-id", "", "deployment pair token")
	revision := flags.String("pair-revision", "", "positive uint64 revision")
	out := flags.String("out", "", "private staging directory")
	final := flags.String("installed-out", "", "installed compiled directory")
	if err := flags.Parse(args); err != nil {
		return 2
	}
	if flags.NArg() != 0 || *config == "" || *out == "" || *final == "" {
		fmt.Fprintln(stderr, "control-env requires --config, --out and --installed-out")
		return 2
	}
	environment, err := compileControlEnv(*config, filepath.Join(root, "config/drives"), *pair, *revision, *out, *final)
	if err == nil {
		_, err = io.WriteString(stdout, environment)
	}
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	return 0
}

func compileControlEnv(config, profiles, pair, revision, out, final string) (environment string, err error) {
	defer validationBoundary(&err)
	if !regexp.MustCompile(`^[A-Za-z0-9_.:-]+$`).MatchString(pair) {
		return "", fmt.Errorf("pair-id must be a nonempty deployment token")
	}
	number, parseErr := strconv.ParseUint(revision, 10, 64)
	if parseErr != nil || number == 0 || !regexp.MustCompile(`^[1-9][0-9]*$`).MatchString(revision) {
		return "", fmt.Errorf("pair-revision must be a positive uint64")
	}
	data, err := os.ReadFile(config)
	if err != nil {
		return "", err
	}
	machine := asObject(decode(data))
	compiled, err := Compile(data, profiles)
	if err != nil {
		return "", err
	}
	if machine.optional("backend", nil) != "live" {
		return "", fmt.Errorf("installed hardware units require an explicitly live machine configuration")
	}
	// The dense axis map is not part of the binding: the adapter derives it
	// from the described rotary axes of this same compiled configuration.
	argv := append(append([]string(nil), compiled.Argv...), "--machine-sha256", compiled.MachineSHA256,
		"--deployment-sha256", compiled.DeploymentSHA256, "--axis-profiles",
		filepath.Join(final, compiled.SHA256, "axes.conf"), "--configuration-hash", compiled.SHA256)
	for _, arg := range argv {
		if !environmentToken.MatchString(arg) {
			return "", fmt.Errorf("compiled argv contains an unsafe environment token")
		}
	}
	directory, _, err := writeCompiled(compiled, out)
	if err != nil {
		return "", err
	}
	encoded, err := json.MarshalIndent(argv, "", "  ")
	if err != nil {
		return "", err
	}
	if err := os.WriteFile(filepath.Join(directory, "argv.json"), append(encoded, '\n'), 0o644); err != nil {
		return "", err
	}
	return fmt.Sprintf("ROSIE_RT_PAIR_ID=%s\nROSIE_RT_PAIR_REVISION=%s\nROSIE_RT_CONFIGURATION_SHA256=%s\nROSIE_RT_CORE_ARGS=\"%s\"\nROSIE_RT_COMPILED_CONFIG=%s\n", pair, revision, compiled.SHA256, strings.Join(argv, " "), filepath.Join(final, compiled.SHA256)), nil
}
