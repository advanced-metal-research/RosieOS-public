package main

import (
	"io"
)

type robotV4AutomaticTourRuntime struct {
	SessionForwarder               *robotV4TourSessionForwarder
	SessionForwarderStartAttempted bool
	SteamLaunchAttempted           bool
}

type robotV4TourSessionForwarder struct {
	document     robotV4ManifestDocument
	connection   robotV4ManifestConnection
	cell         string
	sessionID    string
	unit         string
	invocationID string
	stdout       io.Writer
	ready        bool
	indexed      bool
	stopped      bool
}
