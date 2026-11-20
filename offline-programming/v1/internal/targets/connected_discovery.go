package targets

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"strings"
)

const ConnectedTargetDiscoverySchema = "offline-programming.connected-target-discovery.v1"

// ConnectedTargetServiceDescriptor is safe to expose to a read-only client.
// It describes which observations a later server adapter may provide without
// exposing network locations, subjects, credentials, or command routes.
type ConnectedTargetServiceDescriptor struct {
	Role           string `json:"role"`
	StatusStream   bool   `json:"status_stream"`
	PlanStream     bool   `json:"plan_stream"`
	ReadOnly       bool   `json:"read_only"`
	ServerMediated bool   `json:"server_mediated"`
}

type ConnectedTargetDiscoveryDescriptor struct {
	Schema                string                             `json:"schema"`
	ProfileID             string                             `json:"profile_id"`
	ManifestRef           string                             `json:"manifest_ref"`
	ManifestSchema        string                             `json:"manifest_schema"`
	CellID                string                             `json:"cell_id"`
	PeerStaleAfterSeconds uint64                             `json:"peer_stale_after_s"`
	RobotModelBound       bool                               `json:"robot_model_bound"`
	Services              []ConnectedTargetServiceDescriptor `json:"services"`
	Target                TargetDescriptor                   `json:"target"`
}

// ConnectedTargetServerBinding stays server-side. daemon-v1 refs are stable
// peer/role/service identities; concrete URLs must be resolved from fresh
// daemon state by the future connector.
type ConnectedTargetServerBinding struct {
	ServerTargetBinding
	ManifestRevision           string
	MotionServerNATSServiceRef string
	MotionServerUDPServiceRef  string
	RealtimeCoreUDPServiceRef  string
	LeaderControllerRoles      []string
}

type ConnectedManifestTarget struct {
	Discovery ConnectedTargetDiscoveryDescriptor `json:"discovery"`
	Binding   ConnectedTargetServerBinding       `json:"-"`
}

type connectedManifestMetadata struct {
	SchemaVersion         string   `json:"schema_version"`
	Name                  string   `json:"name"`
	PeerStaleAfterSeconds uint64   `json:"peer_stale_after_s"`
	LeaderControllerRoles []string `json:"leader_controller_roles"`
}

// ParseConnectedTargetManifest builds the additive discovery contract around
// the existing strict Robot V4 target parser. The deployment manifests do not
// bind a robot model, so RobotModelBound deliberately remains false.
func ParseConnectedTargetManifest(manifestRef string, raw []byte, repositoryRoot ...string) (ConnectedManifestTarget, error) {
	target, err := ParseRobotV4Manifest(manifestRef, raw, repositoryRoot...)
	if err != nil {
		return ConnectedManifestTarget{}, err
	}

	decoder := json.NewDecoder(bytes.NewReader(raw))
	var metadata connectedManifestMetadata
	if err := decoder.Decode(&metadata); err != nil {
		return ConnectedManifestTarget{}, fmt.Errorf("targets: decode connected manifest metadata: %w", err)
	}
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		return ConnectedManifestTarget{}, errors.New("targets: connected manifest contains multiple JSON values")
	}
	if metadata.SchemaVersion != target.Descriptor.ManifestSchema ||
		strings.TrimSpace(metadata.Name) != target.Descriptor.CellID {
		return ConnectedManifestTarget{}, errors.New("targets: connected manifest identity changed during parsing")
	}
	if metadata.PeerStaleAfterSeconds == 0 || metadata.PeerStaleAfterSeconds > 300 {
		return ConnectedManifestTarget{}, errors.New("targets: peer freshness window must be between 1 and 300 seconds")
	}
	leaderRoles := make([]string, 0, len(metadata.LeaderControllerRoles))
	seenLeaderRoles := make(map[string]struct{}, len(metadata.LeaderControllerRoles))
	for _, role := range metadata.LeaderControllerRoles {
		role = strings.TrimSpace(role)
		if !safeIdentity(role) || strings.Contains(role, ":") {
			return ConnectedManifestTarget{}, errors.New("targets: leader controller role is invalid")
		}
		if _, duplicate := seenLeaderRoles[role]; duplicate {
			return ConnectedManifestTarget{}, errors.New("targets: leader controller roles must be unique")
		}
		seenLeaderRoles[role] = struct{}{}
		leaderRoles = append(leaderRoles, role)
	}

	motionRole := "motion-server"
	realtimeRole := "realtime-core"
	return ConnectedManifestTarget{
		Discovery: ConnectedTargetDiscoveryDescriptor{
			Schema:                ConnectedTargetDiscoverySchema,
			ProfileID:             target.Descriptor.ProfileID,
			ManifestRef:           target.Descriptor.ManifestRef,
			ManifestSchema:        target.Descriptor.ManifestSchema,
			CellID:                target.Descriptor.CellID,
			PeerStaleAfterSeconds: metadata.PeerStaleAfterSeconds,
			RobotModelBound:       false,
			Services: []ConnectedTargetServiceDescriptor{
				{
					Role:           motionRole,
					StatusStream:   true,
					PlanStream:     true,
					ReadOnly:       true,
					ServerMediated: true,
				},
				{
					Role:           realtimeRole,
					StatusStream:   true,
					PlanStream:     false,
					ReadOnly:       true,
					ServerMediated: true,
				},
			},
			Target: target.Descriptor,
		},
		Binding: ConnectedTargetServerBinding{
			ServerTargetBinding: target.Binding,
			ManifestRevision: fmt.Sprintf(
				"sha256:%x",
				sha256.Sum256(raw),
			),
			MotionServerNATSServiceRef: daemonServiceRef(
				target.Binding.MotionServerPeer,
				motionRole,
				"robot-v4-nats",
			),
			MotionServerUDPServiceRef: daemonServiceRef(
				target.Binding.MotionServerPeer,
				motionRole,
				"robot-v4-cartesian-udp",
			),
			RealtimeCoreUDPServiceRef: daemonServiceRef(
				target.Binding.RealtimeCorePeer,
				realtimeRole,
				"robot-v4-rtcore-udp",
			),
			LeaderControllerRoles: leaderRoles,
		},
	}, nil
}

func daemonServiceRef(peer, role, service string) string {
	if !safeIdentity(peer) || !safeIdentity(role) || !safeIdentity(service) {
		return ""
	}
	return "daemon-v1://" + peer + "/" + role + "/" + service
}
