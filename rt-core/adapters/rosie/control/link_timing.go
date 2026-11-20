//go:build linux

package control

// Unverified compiled LAN and internet idle policy: 90000 ms. Describe publishes
// this value for both the listener and reservation keepers; lease/input ceilings
// retain their independently compiled cell values.
const CompiledControlIdleTimeoutNS uint64 = 90000000000
