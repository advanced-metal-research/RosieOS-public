// Command identity prints a robot description's identity: the sha256 over
// its manifest and every file the manifest registers, as "sha256:<hex>"
// (docs/ROBOT-DESCRIPTION-CONTRACT.md). It is the number a machine config
// pins as robot_description_sha256 and a plan's header carries.
//
//	go run rosieos/robotdesc/cmd/identity robot_description/robots/rosie_1400_v3
package main

import (
	"fmt"
	"os"

	"rosieos/robotdesc"
)

func main() {
	if len(os.Args) < 2 {
		fmt.Fprintln(os.Stderr, "usage: identity <description directory>...")
		os.Exit(2)
	}
	code := 0
	for _, dir := range os.Args[1:] {
		identity, err := robotdesc.RobotDescriptionSHA256(dir)
		if err != nil {
			fmt.Fprintf(os.Stderr, "%s: %v\n", dir, err)
			code = 1
			continue
		}
		fmt.Printf("%s %s\n", identity, dir)
	}
	os.Exit(code)
}
