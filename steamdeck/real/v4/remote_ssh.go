package main

import (
	"fmt"
	"strings"
)

// Dead-link detection for every ssh the deploy commands open: Mac to relay
// (relay sessions, the cell-lock holder, uploads, fetches, the supervised
// tunnels; plain ssh and `tailscale ssh`, which hands options after the
// destination to the real ssh) and relay to servo host inside the relay
// scripts. Without it a session whose path died stays open on the Mac for as
// long as the kernel's TCP keepalive allows (hours on macOS): on 2026-10-08
// an OLP-tier release of ae985a83 held the cell lock from 12:45 to 13:49 PDT
// because its fast-gate session hung half-open after the gate on the relay had
// finished at ~13:02; the release continued only when the coordinator sent
// SIGTERM to the Mac's ssh.
//
// Chosen, not measured (2026-10-08, unverified): ServerAlive 5 s x 3 notices
// a dead path in ~15 s; a healthy idle session answers every probe, so the
// lock holder stays up for hours. The values are the supervised tunnels'
// (tunnel_supervisor.go), which run them on the Mac and the Deck.
const (
	sshServerAliveInterval = 5
	sshServerAliveCount    = 3
	sshConnectTimeout      = 10
)

// sshAliveArgs are the keepalive and connect-timeout options, in the order
// the tunnels have always rendered them.
func sshAliveArgs(connectTimeout int) []string {
	return []string{
		"-o", fmt.Sprintf("ServerAliveInterval=%d", sshServerAliveInterval),
		"-o", fmt.Sprintf("ServerAliveCountMax=%d", sshServerAliveCount),
		"-o", fmt.Sprintf("ConnectTimeout=%d", connectTimeout),
	}
}

// Mac-to-relay sessions (relay scripts, the cell-lock holder, package uploads)
// cross the WAN, often through a relayed tailnet path. On 2026-10-13 (a Rosie 1430 cell,
// Mac with no UDP, 750-1000 ms pings and loss) 15 s without a probe answer
// killed a live 21 MB upload (ssh exit 255) and, on the retry, the lock holder
// during a 2m25s upload, so deploy refused "the cell lock is not held". Chosen,
// not measured: 12 probes (60 s) still ends a dead session within a minute,
// inside the relay holder's 120 s client timeout (cellLockClientTimeout).
// Tunnels and relay-to-servo hops on the cell's LAN keep sshAliveArgs.
const sshRelayServerAliveCount = 12

// sshRelayAliveArgs are sshAliveArgs with the WAN probe budget.
func sshRelayAliveArgs(connectTimeout int) []string {
	return []string{
		"-o", fmt.Sprintf("ServerAliveInterval=%d", sshServerAliveInterval),
		"-o", fmt.Sprintf("ServerAliveCountMax=%d", sshRelayServerAliveCount),
		"-o", fmt.Sprintf("ConnectTimeout=%d", connectTimeout),
	}
}

// sshAliveShell is sshAliveArgs as words for an ssh in a rendered script.
func sshAliveShell(connectTimeout int) string {
	return strings.Join(sshAliveArgs(connectTimeout), " ")
}
