// Command protocolgen generates the current native protocol declarations.
package main

import (
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
)

func run(args []string, diagnostics io.Writer) error {
	flags := flag.NewFlagSet("protocolgen", flag.ContinueOnError)
	flags.SetOutput(diagnostics)
	root := flags.String("root", ".", "rt-core source root containing protocol/control.json")
	outputRoot := flags.String("output-root", "", "generated artifact root (defaults to --root)")
	check := flags.Bool("check", false, "report generated artifact drift without writing files")
	if err := flags.Parse(args); err != nil {
		return err
	}
	if flags.NArg() != 0 {
		return fmt.Errorf("unexpected arguments: %v", flags.Args())
	}
	data, err := os.ReadFile(filepath.Join(*root, "protocol", "control.json"))
	if err != nil {
		return err
	}
	schema, err := parseSchema(data)
	if err != nil {
		return fmt.Errorf("invalid protocol schema: %w", err)
	}
	outputs, err := generate(schema)
	if err != nil {
		return err
	}
	if *outputRoot == "" {
		*outputRoot = *root
	}
	for _, output := range outputs {
		path := filepath.Join(*outputRoot, filepath.FromSlash(output.path))
		if *check {
			existing, err := os.ReadFile(path)
			if err != nil || !equivalent(output.path, existing, output.data) {
				return fmt.Errorf("generated protocol drift: %s", path)
			}
			continue
		}
		if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
			return err
		}
		if err := os.WriteFile(path, output.data, 0o644); err != nil {
			return err
		}
	}
	return nil
}

func main() {
	if err := run(os.Args[1:], os.Stderr); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
