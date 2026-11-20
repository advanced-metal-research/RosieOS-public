package planning

import (
	"encoding/json"
	"errors"
	"fmt"
	"strings"
)

const (
	MotionServerPlannerOwner   = "motion-server"
	MotionServerPlannerBackend = "motion_server_tesseract_trajectory_v1"

	MotionServerPlanTransport = "in_process"

	DefaultSamplesPerSegment = 5
)

// MotionServerSubmission is the bounded, cold-path handoff to the canonical
// motion-server planner owner. Payload is the authored spreadsheet request,
// never a planned path and never a UDP datagram.
type MotionServerSubmission struct {
	RequestID    string          `json:"request_id"`
	PlanRevision uint64          `json:"plan_revision"`
	Transport    string          `json:"transport"`
	Payload      json.RawMessage `json:"payload"`
}

// NewMotionServerSubmission hands the exact shared Robot V4 authoring
// document to the canonical planner. It never reconstructs or filters rows:
// SteamDeck/Offline unknown fields, commands, calibration rows, J7-J9 values,
// constraints, and provenance therefore reach the same Robot V4 parser.
func NewMotionServerSubmission(
	request NormalizedPlanRequest,
	planRevision uint64,
	plannerRequestSubject string,
) (MotionServerSubmission, error) {
	if err := request.Validate(); err != nil {
		return MotionServerSubmission{}, err
	}
	if planRevision == 0 {
		return MotionServerSubmission{}, errors.New("planning: plan revision must be nonzero")
	}
	if strings.ContainsAny(plannerRequestSubject, " \t\r\n") {
		return MotionServerSubmission{}, errors.New("planning: planner request subject contains whitespace")
	}
	if len(request.Program) == 0 {
		return MotionServerSubmission{}, errors.New(
			"planning: complete shared Robot V4 program is required",
		)
	}
	var topLevel map[string]json.RawMessage
	if err := json.Unmarshal(request.Program, &topLevel); err != nil {
		return MotionServerSubmission{}, fmt.Errorf(
			"planning: decode shared Robot V4 program envelope: %w",
			err,
		)
	}
	for _, forbidden := range []string{
		"planned_path",
		"trajectory",
		"accepted_plan",
		"preloaded_plan",
	} {
		if _, present := topLevel[forbidden]; present {
			return MotionServerSubmission{}, fmt.Errorf(
				"planning: authored program must not contain %q",
				forbidden,
			)
		}
	}
	return MotionServerSubmission{
		RequestID:    request.RequestID,
		PlanRevision: planRevision,
		Transport:    MotionServerPlanTransport,
		Payload:      append(json.RawMessage(nil), request.Program...),
	}, nil
}

func canonicalMoveType(value string) (string, string, error) {
	switch strings.ToLower(strings.TrimSpace(value)) {
	case "line", "line_tcp":
		return "line_tcp", "tcp_cartesian", nil
	case "curve", "curve_tcp":
		return "curve_tcp", "tcp_cartesian", nil
	case "joint", "joint_ptp":
		return "joint_ptp", "joint_positions", nil
	default:
		return "", "", fmt.Errorf("unsupported move type %q", value)
	}
}
