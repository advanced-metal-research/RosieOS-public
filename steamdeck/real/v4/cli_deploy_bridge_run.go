package main

import (
	"time"
)

type robotV4BridgeRunBinding struct {
	RunID                 string
	BridgeBootID          string
	ControllerID          string
	ControllerBootID      string
	TestSessionID         string
	RobotCell             string
	RobotModelID          string
	PairRevision          uint64
	PairConfigSHA256      string
	GeneratedURDFSHA256   string
	HomeProfileKey        string
	PairActivateCommandID string
	PlannerURDFPath       string
	MappedAxisMask        uint32
	BridgeMonotonicMS     uint64
	DeadlineMonotonicMS   uint64
	ExpiresInMS           uint64
	ObservedAt            time.Time
}
