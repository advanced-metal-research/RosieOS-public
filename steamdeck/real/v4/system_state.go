package main

// System-state maintenance and legacy dry-run previews for the deploy CLI.
// Native observation is in system_state_reader_linux.go. Runtime mode commands
// fail closed because native status cannot prove the v4 output-switch contract.
//
// The runtime postures are simple, reversible operations:
//
//	clean  maintenance command: every manifest process desired stopped
//	sim    everything remains running; publish a NATS mode command to RTCore,
//	       which still receives commands but disables EtherCAT motion outputs
//	live   everything remains running; publish a NATS mode command to RTCore,
//	       which re-enables EtherCAT motion outputs while staying UNARMED
//
// Publishing goes to the local daemon and to the motion-server daemon when
// reachable: children on isolated links (RevPi) mirror intents from their
// parent index, so the parent is the reliable publish point for them.
