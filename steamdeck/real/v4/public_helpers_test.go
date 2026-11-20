package main

// Helpers shared by public and private tests. The site's own cells, pins and
// hosts are in site_helpers_test.go, which only private tests use and the
// public mirror leaves out.

const olpUITestCell = "example-arm-1420"

func olpUITestConfig(dir string) olpUIConfig {
	return olpUIConfig{Cell: olpUITestCell, Transport: "tailscale", Target: "operator@example-relay", OLPListen: "127.0.0.1:8794",
		Port: 5189, TunnelPort: 18894, UIDir: "/repo/offline-programming/v1/ui", StateDir: dir,
		TunnelBin: "/usr/local/bin/tailscale", NPM: "/opt/homebrew/bin/npm", PathEnv: "/opt/homebrew/bin:/usr/bin:/bin"}
}

const steamDeckCaptureTestXProp = `#!/bin/sh
case "$*" in
  *"_NET_WM_PID"*) printf '_NET_WM_PID(CARDINAL) = %s\n' "$ROBOT_V4_TEST_WINDOW_PID" ;;
  *"STEAM_GAME"*) printf 'STEAM_GAME(CARDINAL) = %s\n' "$ROBOT_V4_TEST_STEAM_APP_ID" ;;
  *) exit 1 ;;
esac
`
