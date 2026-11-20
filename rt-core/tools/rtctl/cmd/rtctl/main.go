package main

import (
	"fmt"
	"os"

	"rosieos/rt-core/tools/rtctl"
)

func main() {
	if len(os.Args) > 1 && os.Args[1] == "telemetry" {
		os.Exit(rtctl.RunTelemetryCLI(os.Args[2:], os.Stdout, os.Stderr))
	}
	if len(os.Args) > 1 && rtctl.IsControlCommand(os.Args[1]) {
		os.Exit(rtctl.RunControlCLI(os.Args[1:], os.Stdout, os.Stderr))
	}
	root, err := rtctl.DefaultRoot()
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		if len(os.Args) > 1 && os.Args[1] == "hostcheck" {
			os.Exit(3)
		}
		os.Exit(1)
	}
	if len(os.Args) > 1 {
		switch os.Args[1] {
		case "hostcheck":
			os.Exit(rtctl.RunHostcheckCLI(os.Args[2:], root, os.Stdout, os.Stderr))
		case "inventory":
			os.Exit(rtctl.RunInventoryCLI(os.Args[2:], root, os.Stdout, os.Stderr))
		}
	}
	os.Exit(rtctl.RunCLI(os.Args[1:], root, os.Stdout, os.Stderr))
}
