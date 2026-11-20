// Package targets defines manifest-backed target identity and server-only
// control boundaries for Offline Programming V1.
package targets

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"strconv"
	"strings"
)

const TargetDescriptorSchema = "offline-programming.target-descriptor.v1"

type TargetKind string

const (
	TargetOfflinePreview TargetKind = "offline_preview"
	TargetLocalSimulator TargetKind = "local_simulator"
	TargetRealCell       TargetKind = "real_cell"
)

type TargetCapabilities struct {
	Monitoring          bool `json:"monitoring"`
	Planning            bool `json:"planning"`
	Simulation          bool `json:"simulation"`
	RealExecution       bool `json:"real_execution"`
	Teleop              bool `json:"teleop"`
	RawBrowserTransport bool `json:"raw_browser_transport"`
}

type AxisDescriptor struct {
	Index                 int     `json:"index"`
	Name                  string  `json:"name"`
	URDFJoint             string  `json:"urdf_joint"`
	Kind                  string  `json:"kind"`
	GearRatio             float64 `json:"gear_ratio,omitempty"`
	Sign                  float64 `json:"sign"`
	CartesianParticipates bool    `json:"cartesian_participates"`
}

type TargetDescriptor struct {
	Schema         string             `json:"schema"`
	ProfileID      string             `json:"profile_id"`
	Label          string             `json:"label"`
	Kind           TargetKind         `json:"kind"`
	ManifestRef    string             `json:"manifest_ref,omitempty"`
	ManifestSchema string             `json:"manifest_schema,omitempty"`
	CellID         string             `json:"cell_id,omitempty"`
	Axes           []AxisDescriptor   `json:"axes"`
	Capabilities   TargetCapabilities `json:"capabilities"`
}

type ServerTargetBinding struct {
	CommandSubject            string
	MotionServerPeer          string
	RealtimeCorePeer          string
	MotionServerStatusSubject string
	MotionServerPlanSubject   string
	RealtimeCoreStatusSubject string
}

type ManifestTarget struct {
	Descriptor TargetDescriptor    `json:"descriptor"`
	Binding    ServerTargetBinding `json:"-"`
}

type robotV4Manifest struct {
	SchemaVersion       string `json:"schema_version"`
	Name                string `json:"name"`
	RobotCommandSubject string `json:"robot_command_subject"`
	RobotAxes           struct {
		Axes []manifestAxis `json:"axes"`
	} `json:"robot_axes"`
	Nodes []manifestNode `json:"nodes"`
}

type manifestAxis struct {
	Index                 int      `json:"index"`
	Name                  string   `json:"name"`
	URDFJoint             string   `json:"urdf_joint"`
	Kind                  string   `json:"kind"`
	GearRatio             *float64 `json:"gear_ratio"`
	Sign                  float64  `json:"sign"`
	CartesianParticipates bool     `json:"cartesian_participates"`
}

type manifestNode struct {
	Node             string   `json:"node"`
	Peer             string   `json:"peer"`
	Role             string   `json:"role"`
	Host             string   `json:"host"`
	ColdPathSubjects []string `json:"cold_path_subjects"`
	RTCore           *struct {
		MachineConfig       string `json:"machine_config"`
		DriveProfiles       string `json:"drive_profiles,omitempty"`
		PairID              string `json:"pair_id"`
		PairRevision        uint64 `json:"pair_revision"`
		RemoteListen        string `json:"remote_listen"`
		ConfigurationSHA256 string `json:"configuration_sha256"`
	} `json:"rt_core"`
}

// ParseRobotV4Manifest validates the identity-bearing subset of a Robot V4
// manifest. The public descriptor intentionally omits hosts, NATS subjects,
// UDP endpoints, and command routes; those remain in the server-only binding.
func ParseRobotV4Manifest(manifestRef string, raw []byte, repositoryRoot ...string) (ManifestTarget, error) {
	if strings.TrimSpace(manifestRef) == "" {
		return ManifestTarget{}, errors.New("targets: manifest reference is required")
	}
	decoder := json.NewDecoder(bytes.NewReader(raw))
	var manifest robotV4Manifest
	if err := decoder.Decode(&manifest); err != nil {
		return ManifestTarget{}, fmt.Errorf("targets: decode robot v4 manifest: %w", err)
	}
	if manifest.SchemaVersion != "robot_v4_deployment_manifest_v1" {
		return ManifestTarget{}, fmt.Errorf("targets: unsupported manifest schema %q", manifest.SchemaVersion)
	}
	manifest.Name = strings.TrimSpace(manifest.Name)
	if !safeIdentity(manifest.Name) {
		return ManifestTarget{}, errors.New("targets: manifest name is not a concrete cell identity")
	}
	expectedCommandSubject := "robot/v4/robot." + manifest.Name + ".command"
	if manifest.RobotCommandSubject != expectedCommandSubject {
		return ManifestTarget{}, errors.New("targets: robot command subject does not match manifest cell")
	}
	if err := resolveManifestRatios(manifestRef, &manifest, repositoryRoot...); err != nil {
		return ManifestTarget{}, err
	}
	axes, err := normalizeAxes(manifest.RobotAxes.Axes)
	if err != nil {
		return ManifestTarget{}, err
	}
	motion, err := uniqueRole(manifest.Nodes, "motion-server")
	if err != nil {
		return ManifestTarget{}, err
	}
	realtime, err := uniqueRole(manifest.Nodes, "realtime-core")
	if err != nil {
		return ManifestTarget{}, err
	}
	if !concreteNodeIdentity(motion) || !concreteNodeIdentity(realtime) {
		return ManifestTarget{}, errors.New("targets: manifest nodes require concrete peer and host identities")
	}
	motionStatus := exactSubject(motion.ColdPathSubjects, "robot/v4/motion-server."+motion.Peer+".status")
	motionPlan := exactSubject(motion.ColdPathSubjects, "robot/v4/motion-server."+motion.Peer+".plan")
	realtimeStatus := exactSubject(realtime.ColdPathSubjects, "robot/v4/rtcore."+realtime.Host+".status")
	if motionStatus == "" || motionPlan == "" || realtimeStatus == "" {
		return ManifestTarget{}, errors.New("targets: manifest is missing exact cell status or plan subjects")
	}
	return ManifestTarget{
		Descriptor: TargetDescriptor{
			Schema:         TargetDescriptorSchema,
			ProfileID:      manifest.Name,
			Label:          manifest.Name,
			Kind:           TargetRealCell,
			ManifestRef:    strings.TrimSpace(manifestRef),
			ManifestSchema: manifest.SchemaVersion,
			CellID:         manifest.Name,
			Axes:           axes,
			Capabilities: TargetCapabilities{
				Monitoring:          true,
				Planning:            true,
				Simulation:          false,
				RealExecution:       false,
				Teleop:              false,
				RawBrowserTransport: false,
			},
		},
		Binding: ServerTargetBinding{
			CommandSubject:            expectedCommandSubject,
			MotionServerPeer:          motion.Peer,
			RealtimeCorePeer:          realtime.Peer,
			MotionServerStatusSubject: motionStatus,
			MotionServerPlanSubject:   motionPlan,
			RealtimeCoreStatusSubject: realtimeStatus,
		},
	}, nil
}

func OfflineDescriptor() TargetDescriptor {
	return TargetDescriptor{
		Schema:    TargetDescriptorSchema,
		ProfileID: "offline-preview",
		Label:     "Offline preview",
		Kind:      TargetOfflinePreview,
		Axes:      []AxisDescriptor{},
		Capabilities: TargetCapabilities{
			Monitoring:          false,
			Planning:            false,
			Simulation:          true,
			RealExecution:       false,
			Teleop:              false,
			RawBrowserTransport: false,
		},
	}
}

func LocalSimulatorDescriptor(profileID string, axes []AxisDescriptor) (TargetDescriptor, error) {
	if !safeIdentity(profileID) {
		return TargetDescriptor{}, errors.New("targets: local simulator profile id is invalid")
	}
	if len(axes) == 0 || len(axes) > 9 {
		return TargetDescriptor{}, errors.New("targets: local simulator axes are required")
	}
	return TargetDescriptor{
		Schema:    TargetDescriptorSchema,
		ProfileID: profileID,
		Label:     "Local simulator",
		Kind:      TargetLocalSimulator,
		Axes:      append([]AxisDescriptor(nil), axes...),
		Capabilities: TargetCapabilities{
			Monitoring:          true,
			Planning:            true,
			Simulation:          true,
			RealExecution:       false,
			Teleop:              false,
			RawBrowserTransport: false,
		},
	}, nil
}

func normalizeAxes(raw []manifestAxis) ([]AxisDescriptor, error) {
	if len(raw) == 0 || len(raw) > 9 {
		return nil, errors.New("targets: manifest must define between one and nine axes")
	}
	axes := make([]AxisDescriptor, len(raw))
	for index, axis := range raw {
		expectedName := "J" + strconv.Itoa(index+1)
		if axis.Index != index || axis.Name != expectedName || axis.URDFJoint != expectedName {
			return nil, fmt.Errorf("targets: axis %d is not in canonical J1-J9 order", index)
		}
		if axis.Kind != "arm" && axis.Kind != "external" {
			return nil, fmt.Errorf("targets: axis %s has unsupported kind %q", axis.Name, axis.Kind)
		}
		if (axis.GearRatio != nil && *axis.GearRatio <= 0) || (axis.Sign != -1 && axis.Sign != 1) {
			return nil, fmt.Errorf("targets: axis %s has invalid ratio or sign", axis.Name)
		}
		axes[index] = AxisDescriptor{
			Index:                 axis.Index,
			Name:                  axis.Name,
			URDFJoint:             axis.URDFJoint,
			Kind:                  axis.Kind,
			Sign:                  axis.Sign,
			CartesianParticipates: axis.CartesianParticipates,
		}
		if axis.GearRatio != nil {
			axes[index].GearRatio = *axis.GearRatio
		}
	}
	return axes, nil
}

func uniqueRole(nodes []manifestNode, role string) (manifestNode, error) {
	var match manifestNode
	count := 0
	for _, node := range nodes {
		if strings.TrimSpace(node.Role) == role {
			match = node
			count++
		}
	}
	if count != 1 {
		return manifestNode{}, fmt.Errorf("targets: manifest requires exactly one %s node", role)
	}
	return match, nil
}

func concreteNodeIdentity(node manifestNode) bool {
	return safeIdentity(node.Peer) &&
		safeIdentity(node.Host) &&
		node.Peer != node.Role &&
		node.Host != node.Role &&
		node.Peer != node.Node
}

func exactSubject(subjects []string, expected string) string {
	for _, subject := range subjects {
		if subject == expected {
			return subject
		}
	}
	return ""
}

func safeIdentity(value string) bool {
	value = strings.TrimSpace(value)
	if value == "" || len(value) > 96 {
		return false
	}
	for _, char := range value {
		if (char >= 'a' && char <= 'z') ||
			(char >= 'A' && char <= 'Z') ||
			(char >= '0' && char <= '9') ||
			char == '-' || char == '_' || char == '.' {
			continue
		}
		return false
	}
	return true
}
