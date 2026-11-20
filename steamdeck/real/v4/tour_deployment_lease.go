package main

import (
	"os"

	"sync"
)

type robotV4TourDeploymentLeaseRecord struct {
	Schema                string `json:"schema"`
	Cell                  string `json:"cell"`
	TargetPeer            string `json:"target_peer"`
	DeploymentFingerprint string `json:"deployment_fingerprint"`
	TestSessionID         string `json:"test_session_id"`
	Owner                 string `json:"owner"`
	OriginalOwner         string `json:"original_owner,omitempty"`
	Generation            uint64 `json:"generation"`
	StartedAt             string `json:"started_at"`
	RecoveryStartedAt     string `json:"recovery_started_at,omitempty"`
	State                 string `json:"state"`
	Phase                 string `json:"phase"`
	ProofPath             string `json:"proof_path"`
	ProofSHA256           string `json:"proof_sha256,omitempty"`
	PublicationEventID    string `json:"publication_event_id,omitempty"`
	FinalPosture          string `json:"final_posture"`
	TourSucceededAt       string `json:"tour_succeeded_at,omitempty"`
	UpdatedAt             string `json:"updated_at"`
}

type robotV4TourDeploymentCellLock struct {
	mu       sync.Mutex
	path     string
	file     *os.File
	released bool
}

type robotV4TourDeploymentLease struct {
	mu           sync.Mutex
	path         string
	record       robotV4TourDeploymentLeaseRecord
	cellLock     *robotV4TourDeploymentCellLock
	recoveryOnly bool
	guardOnly    bool
	released     bool
}
