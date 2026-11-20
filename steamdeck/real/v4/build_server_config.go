package main

import (
	"os"
	"strings"
)

const robotV4BuildServerEnv = "ROBOT_V4_BUILD_SERVER"

var robotV4BuildServerOverride string

// robotV4ConfiguredBuildServer is the operator's choice of build server: the
// explicit value, else ROBOT_V4_BUILD_SERVER, else the older Steam Deck-only
// ROBOT_V4_STEAMDECK_BUILD_HOST, lower-cased; "" when none is set. With no
// choice the cell manifest decides (robotV4PickBuildPeer), so no host is
// named in code.
func robotV4ConfiguredBuildServer(explicit string) string {
	for _, candidate := range []string{
		explicit,
		os.Getenv(robotV4BuildServerEnv),
		// Keep the old Steam Deck-only override working while all callers move
		// to the shared build-server setting.
		os.Getenv("ROBOT_V4_STEAMDECK_BUILD_HOST"),
	} {
		if value := strings.ToLower(strings.TrimSpace(candidate)); value != "" {
			return value
		}
	}
	return ""
}

func selectedRobotV4BuildServer() string {
	return robotV4ConfiguredBuildServer(robotV4BuildServerOverride)
}
