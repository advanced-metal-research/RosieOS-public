package connected

import (
	"errors"
	"fmt"
	"strings"
)

const RTCoreOutputFrameSchema = "offline-programming.connected-rtcore-output.v1"

type rawRTCoreStatusEnvelope struct {
	Schema      string `json:"schema"`
	Subject     string `json:"subject"`
	Host        string `json:"host"`
	Component   string `json:"component"`
	PublishedAt string `json:"published_at"`
	Status      struct {
		Name                  string `json:"name"`
		State                 string `json:"state"`
		UpdatedAt             string `json:"updated_at"`
		RTCoreMode            string `json:"rtcore_mode"`
		SimulationMode        *bool  `json:"simulation_mode"`
		EtherCATLive          *bool  `json:"ethercat_live"`
		EtherCATOutputEnabled *bool  `json:"ethercat_output_enabled"`
		MotionOutputEnabled   *bool  `json:"motion_output_enabled"`
		PDOFresh              *bool  `json:"pdo_fresh"`
		ServosArmedRequested  *bool  `json:"servos_armed_requested"`
		ConfiguredAxisCount   int    `json:"configured_axis_count"`
		DS402EnabledAxes      int    `json:"ds402_enabled_axes"`
		AxisEnableMask        uint32 `json:"axis_enable_mask"`
		AxisErrorMask         uint32 `json:"axis_error_mask"`
		AxisFaultMask         uint32 `json:"axis_fault_mask"`
		RevPiOutputReadback   struct {
			Schema      string `json:"schema"`
			OutputCount int    `json:"output_count"`
			ValuesMask  uint16 `json:"values_mask"`
			ValidMask   uint16 `json:"valid_mask"`
			Fresh       bool   `json:"fresh"`
		} `json:"revpi_output_readback"`
	} `json:"status"`
}

func NormalizeNATSRTCoreStatusFrame(
	context ObservationContext,
	raw []byte,
) (RTCoreOutputObservation, error) {
	if len(raw) == 0 || len(raw) > MaxStatusFrameBytes {
		return RTCoreOutputObservation{}, ErrFrameTooLarge
	}
	if err := validateContext(
		context,
		context.Target.Binding.RealtimeCorePeer,
		context.Target.Binding.RealtimeCoreStatusSubject,
	); err != nil {
		return RTCoreOutputObservation{}, err
	}
	var envelope rawRTCoreStatusEnvelope
	if err := decodeSingleJSON(raw, &envelope); err != nil {
		return RTCoreOutputObservation{}, fmt.Errorf(
			"connected: decode RTCore status: %w",
			err,
		)
	}
	expectedHost := rtcoreSubjectHost(context.SourceSubject)
	if envelope.Schema != "robot.v4.rtcore.status.v1" ||
		envelope.Subject != context.SourceSubject ||
		envelope.Component != "rtcore" ||
		strings.TrimSpace(envelope.Host) != expectedHost ||
		strings.TrimSpace(envelope.Status.Name) != "robot-v4-rtcore" ||
		strings.TrimSpace(envelope.Status.State) != "running" {
		return RTCoreOutputObservation{}, errors.New(
			"connected: RTCore status identity does not match the selected manifest",
		)
	}
	publishedAt, publishedOK := parseSourceTime(
		envelope.PublishedAt,
		context.ReceivedAt,
	)
	sourceUpdatedAt, sourceOK := parseSourceTime(
		envelope.Status.UpdatedAt,
		context.ReceivedAt,
	)
	if !publishedOK || !sourceOK ||
		!context.ReceivedAt.Before(
			publishedAt.Add(context.FreshnessWindow),
		) ||
		!context.ReceivedAt.Before(
			sourceUpdatedAt.Add(context.FreshnessWindow),
		) {
		return RTCoreOutputObservation{}, errors.New(
			"connected: RTCore status timestamp is missing, future, or stale",
		)
	}
	freshUntil := publishedAt.Add(context.FreshnessWindow)
	if candidate := sourceUpdatedAt.Add(context.FreshnessWindow); candidate.Before(freshUntil) {
		freshUntil = candidate
	}

	status := envelope.Status
	reasons := make([]string, 0, 8)
	liveRuntime := status.RTCoreMode == "live" &&
		status.SimulationMode != nil &&
		!*status.SimulationMode &&
		status.EtherCATLive != nil &&
		*status.EtherCATLive &&
		status.PDOFresh != nil &&
		*status.PDOFresh &&
		status.EtherCATOutputEnabled != nil &&
		*status.EtherCATOutputEnabled
	if !liveRuntime {
		reasons = appendReason(reasons, "rtcore_live_output_not_proved")
	}
	if status.AxisErrorMask != 0 || status.AxisFaultMask != 0 {
		reasons = appendReason(reasons, "rtcore_axis_fault_or_error_present")
	}

	armRequested := booleanFact(status.ServosArmedRequested)
	actualArmed := BooleanFact{}
	if status.ConfiguredAxisCount > 0 &&
		status.ConfiguredAxisCount <= 16 &&
		status.ServosArmedRequested != nil {
		allAxes := uint32(1<<uint(status.ConfiguredAxisCount)) - 1
		switch {
		case !*status.ServosArmedRequested &&
			status.AxisEnableMask == 0 &&
			status.DS402EnabledAxes == 0:
			actualArmed = BooleanFact{Known: true, Value: false}
		case *status.ServosArmedRequested &&
			status.AxisEnableMask == allAxes &&
			status.DS402EnabledAxes == status.ConfiguredAxisCount:
			actualArmed = BooleanFact{Known: true, Value: true}
		default:
			reasons = appendReason(reasons, "rtcore_arm_state_transitional")
		}
	} else {
		reasons = appendReason(reasons, "rtcore_arm_state_unavailable")
	}

	weld := BooleanFact{}
	readback := status.RevPiOutputReadback
	allOutputs := uint16((1 << 14) - 1)
	if readback.Schema == "robot.v4.revpi-output-readback.v1" &&
		readback.OutputCount == 14 &&
		readback.Fresh &&
		readback.ValidMask&allOutputs == allOutputs {
		weld = BooleanFact{
			Known: true,
			Value: readback.ValuesMask&(1<<4) != 0,
		}
	} else {
		reasons = appendReason(reasons, "revpi_weld_output_readback_unavailable")
	}

	return RTCoreOutputObservation{
		Schema:          RTCoreOutputFrameSchema,
		Target:          targetIdentity(context.Target.Discovery),
		SourcePeer:      context.SourcePeer,
		SourceSubject:   context.SourceSubject,
		Sequence:        context.Sequence,
		ReceivedAt:      context.ReceivedAt.UTC(),
		SourceUpdatedAt: sourceUpdatedAt,
		FreshUntil:      freshUntil,
		ServiceName:     strings.TrimSpace(status.Name),
		ServiceState:    strings.TrimSpace(status.State),
		Output: RuntimeOutputFacts{
			ArmRequested:     armRequested,
			ActualServoArmed: actualArmed,
			EtherCATEnabled: BooleanFact{
				Known: status.EtherCATOutputEnabled != nil &&
					status.EtherCATLive != nil &&
					status.PDOFresh != nil,
				Value: liveRuntime,
			},
			MotionOutput: booleanFact(status.MotionOutputEnabled),
			WeldOutput:   weld,
		},
		Reasons: reasons,
	}, nil
}

func rtcoreSubjectHost(subject string) string {
	const prefix = "robot/v4/rtcore."
	const suffix = ".status"
	if !strings.HasPrefix(subject, prefix) || !strings.HasSuffix(subject, suffix) {
		return ""
	}
	return strings.TrimSuffix(strings.TrimPrefix(subject, prefix), suffix)
}
