package planning

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"strings"
	"time"
)

const (
	// Generous memory-safety backstops, not authoring budgets (P2.11, ledger
	// L27). Sized on 2026-09-07 for the planner's wires at the 10 ms
	// certificate density on a cell limited to 0.09 rad/s: a two-seam
	// bracket program is 8747 rows, one per knot, so 65536 rows is ~11
	// minutes of motion; a full-fidelity wire document at that size measures
	// ~85 MB, so 96 MiB bounds the allocation with headroom while still
	// refusing hostile payloads. (The former 4096 was >5x the worst real
	// part at 0.2 s knots, exact Dog Box at 730 rows.) Mirrors maxItems in
	// robot/v4/contracts/programs/robot.v4.program.v1.schema.json.
	maxRobotV4ProgramRows  = 65536
	maxRobotV4ProgramBytes = 96 * 1024 * 1024
)

type robotV4ProgramDocument struct {
	SchemaVersion       int          `json:"schema_version"`
	ProgramID           string       `json:"program_id"`
	ProgramRevision     uint64       `json:"program_revision"`
	BaseProgramRevision uint64       `json:"base_program_revision"`
	RobotModel          RobotModel   `json:"robot_model"`
	RoboticCellType     string       `json:"robotic-cell-type"`
	ExecutionMode       string       `json:"execution_mode"`
	Rows                []robotV4Row `json:"rows"`
}

type robotV4Row struct {
	RowID             string              `json:"row_id"`
	Revision          uint64              `json:"revision"`
	Enabled           bool                `json:"enabled"`
	RowType           string              `json:"row_type"`
	Command           string              `json:"command"`
	MoveType          string              `json:"move_type"`
	MotionPhase       string              `json:"motion_phase"`
	ToolFrame         string              `json:"tool_frame"`
	WorkFrame         string              `json:"work_frame"`
	TargetTCPM        [6]float64          `json:"target_tcp_m"`
	TargetJointsRad   [JointCount]float64 `json:"target_joints_rad"`
	TCPVelocityMS     float64             `json:"tcp_velocity_m_s"`
	JointVelocityRadS float64             `json:"joint_velocity_rad_s"`
	WeldEnable        bool                `json:"weld_enable"`
	ArcState          string              `json:"arc_state"`
	RevPiIOSlot       *int                `json:"revpi_io_slot"`
	RevPiIOAction     string              `json:"revpi_io_action"`
}

// NewNormalizedPlanRequestFromRobotV4Program validates the complete shared
// authoring document and derives only a correlation index. The planner input
// remains the original JSON bytes, including fields unknown to this version.
func NewNormalizedPlanRequestFromRobotV4Program(
	schema string,
	requestID string,
	projectID string,
	projectRevision uint64,
	target TargetIdentity,
	submittedAt time.Time,
	program json.RawMessage,
) (NormalizedPlanRequest, error) {
	request := NormalizedPlanRequest{
		Schema:          schema,
		RequestID:       strings.TrimSpace(requestID),
		ProjectID:       strings.TrimSpace(projectID),
		ProjectRevision: projectRevision,
		Target:          target,
		SubmittedAt:     submittedAt,
		Program:         append(json.RawMessage(nil), program...),
	}
	document, err := decodeRobotV4ProgramDocument(program)
	if err != nil {
		return NormalizedPlanRequest{}, err
	}
	request.ProgramID = strings.TrimSpace(document.ProgramID)
	request.ProgramRevision = document.ProgramRevision
	request.BaseProgramRevision = document.BaseProgramRevision
	request.RobotModel = document.RobotModel
	request.Rows = planRowsFromRobotV4(document.Rows)
	if err := request.Validate(); err != nil {
		return NormalizedPlanRequest{}, err
	}
	return request, nil
}

func validateRobotV4ProgramDocument(
	raw json.RawMessage,
	request NormalizedPlanRequest,
) (robotV4ProgramDocument, error) {
	document, err := decodeRobotV4ProgramDocument(raw)
	if err != nil {
		return robotV4ProgramDocument{}, err
	}
	if document.ProgramID != request.ProgramID ||
		document.ProgramRevision != request.ProgramRevision ||
		document.BaseProgramRevision != request.BaseProgramRevision ||
		document.RobotModel != request.RobotModel {
		return robotV4ProgramDocument{}, errors.New(
			"planning: shared Robot V4 program identity does not match the request",
		)
	}
	return document, nil
}

func decodeRobotV4ProgramDocument(raw json.RawMessage) (robotV4ProgramDocument, error) {
	if len(raw) == 0 || len(raw) > maxRobotV4ProgramBytes {
		return robotV4ProgramDocument{}, errors.New(
			"planning: shared Robot V4 program is empty or exceeds the byte bound",
		)
	}
	decoder := json.NewDecoder(bytes.NewReader(raw))
	var document robotV4ProgramDocument
	if err := decoder.Decode(&document); err != nil {
		return robotV4ProgramDocument{}, fmt.Errorf(
			"planning: decode shared Robot V4 program: %w",
			err,
		)
	}
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		return robotV4ProgramDocument{}, errors.New(
			"planning: shared Robot V4 program contains multiple JSON values",
		)
	}
	if document.SchemaVersion != 1 {
		return robotV4ProgramDocument{}, errors.New(
			"planning: shared Robot V4 program schema_version must be 1",
		)
	}
	if strings.TrimSpace(document.ProgramID) == "" ||
		document.ProgramRevision == 0 ||
		document.BaseProgramRevision > document.ProgramRevision {
		return robotV4ProgramDocument{}, errors.New(
			"planning: shared Robot V4 program identity is invalid",
		)
	}
	if !supportedRobotModel(document.RobotModel) ||
		strings.TrimSpace(document.RoboticCellType) == "" {
		return robotV4ProgramDocument{}, errors.New(
			"planning: shared Robot V4 program robot identity is invalid",
		)
	}
	switch document.ExecutionMode {
	case "simulation", "dry_run", "production":
	default:
		return robotV4ProgramDocument{}, errors.New(
			"planning: shared Robot V4 program execution_mode is invalid",
		)
	}
	if len(document.Rows) < 1 || len(document.Rows) > maxRobotV4ProgramRows {
		return robotV4ProgramDocument{}, fmt.Errorf(
			"planning: shared Robot V4 program must contain 1-%d rows",
			maxRobotV4ProgramRows,
		)
	}
	seen := make(map[string]struct{}, len(document.Rows))
	for index, row := range document.Rows {
		if strings.TrimSpace(row.RowID) == "" || row.Revision == 0 {
			return robotV4ProgramDocument{}, fmt.Errorf(
				"planning: shared Robot V4 program row %d has incomplete identity",
				index,
			)
		}
		if _, duplicate := seen[row.RowID]; duplicate {
			return robotV4ProgramDocument{}, fmt.Errorf(
				"planning: shared Robot V4 program contains duplicate row_id %q",
				row.RowID,
			)
		}
		seen[row.RowID] = struct{}{}
		switch row.RowType {
		case "motion":
			if strings.TrimSpace(row.ToolFrame) == "" ||
				strings.TrimSpace(row.WorkFrame) == "" {
				return robotV4ProgramDocument{}, fmt.Errorf(
					"planning: motion row %q requires tool and work frames",
					row.RowID,
				)
			}
			if _, _, err := canonicalMoveType(row.MoveType); err != nil {
				return robotV4ProgramDocument{}, fmt.Errorf(
					"planning: motion row %q: %w",
					row.RowID,
					err,
				)
			}
		case "command", "comment", "capture", "calibration", "io":
		default:
			return robotV4ProgramDocument{}, fmt.Errorf(
				"planning: row %q has unsupported row_type %q",
				row.RowID,
				row.RowType,
			)
		}
		if !finiteArray(row.TargetTCPM[:]) ||
			!finiteArray(row.TargetJointsRad[:]) ||
			!finiteNonnegative(row.TCPVelocityMS) ||
			!finiteNonnegative(row.JointVelocityRadS) {
			return robotV4ProgramDocument{}, fmt.Errorf(
				"planning: row %q contains invalid numeric values",
				row.RowID,
			)
		}
	}
	return document, nil
}

func planRowsFromRobotV4(rows []robotV4Row) []PlanRow {
	result := make([]PlanRow, len(rows))
	for index, row := range rows {
		result[index] = PlanRow{
			RowID:             row.RowID,
			Revision:          row.Revision,
			Enabled:           row.Enabled,
			RowType:           row.RowType,
			Command:           strings.TrimSpace(row.Command),
			MoveType:          strings.TrimSpace(row.MoveType),
			MotionPhase:       strings.TrimSpace(row.MotionPhase),
			ToolFrame:         strings.TrimSpace(row.ToolFrame),
			WorkFrame:         strings.TrimSpace(row.WorkFrame),
			TargetTCPM:        row.TargetTCPM,
			TargetJointsRad:   row.TargetJointsRad,
			TCPVelocityMS:     row.TCPVelocityMS,
			JointVelocityRadS: row.JointVelocityRadS,
			WeldEnable:        row.WeldEnable,
			ArcState:          strings.TrimSpace(row.ArcState),
			RevPiIOSlot:       row.RevPiIOSlot,
			RevPiIOAction:     strings.TrimSpace(row.RevPiIOAction),
		}
	}
	return result
}
