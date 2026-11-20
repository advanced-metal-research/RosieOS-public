package main

import (
	"fmt"
	wire "rosieos/rt-core/ipcclient"
	"strings"
	"testing"
)

// Array bounds belong to fields, including renamed fields; a scalar attribute
// or undersized bound fails closed instead of bypassing layout admission.
func TestTelemetrySchemaArrayBound(t *testing.T) {
	for _, tc := range []struct {
		kind, attribute string
		size            int
		valid           bool
	}{
		{"u8[128]", `,"array_bound":128`, 128, true},
		{"u8[128]", "", 128, false},
		{"u8[128]", `,"array_bound":127`, 128, false},
		{"u8[128]", `,"array_bound":0`, 128, false},
		{"u8[128]", `,"array_bound":-1`, 128, false},
		{"u8[128]", `,"array_bound":999999999`, 128, false},
		{"u8", `,"array_bound":128`, 1, false},
	} {
		raw := fmt.Sprintf(`{"size":%d,"alignment":1,"fields":[{"name":"renamed","type":%q,"offset":0,"size":%d,"alignment":1%s}]}`, tc.size, tc.kind, tc.size, tc.attribute)
		_, err := parseWireRecord("RenamedRecord", []byte(raw), nil)
		if (err == nil) != tc.valid {
			t.Fatalf("%s: %v", raw, err)
		}
	}
}

// Pinned independent telemetry byte fixtures cover every configured-axis field
// and the bounded mark payload, preventing codec drift from changing old dumps.
func telemetryGoldenCases() []goldenCase {
	axis := func(i int) wire.CycleCaptureAxisV2 {
		a := wire.CycleCaptureAxisV2{Position: -int32(i + 1), Target: int32(100 + i), Statusword: 39, ErrorCode: 2, ManufacturerErrorCode: uint32(42 + i)}
		a.AbsoluteValue[0] = -int32((i + 1) * 2)
		a.AbsoluteValue[15] = -int32((i + 1) * 3)
		a.AbsoluteValid[0] = 1
		a.AbsoluteValid[15] = 1
		a.VelocityActualCountsPerS = -123456 - int32(i)
		a.FollowingErrorCounts = 789 + int32(i)
		a.VelocityActualValid = 1
		a.FollowingErrorValid = 1
		a.DiBits, a.DiValid, a.ExternalEnableActive, a.ExternalEnableValid = 65536, 1, 1, 1
		a.CoordinateCounts = -9007199254741121
		a.CoordinateEpoch = 9007199254741129
		a.PdoObservedTimeNs = 9007199254741137
		a.CollisionPeakFollowingErrorCounts = 9007199254741145
		a.CollisionTripCount = 9007199254741153
		a.CollisionSamples = 9007199254741161
		a.CollisionLastTripPeakFollowingErrorCounts = 9007199254741169
		a.AbsoluteSourceCounts = -185
		a.CoordinateReason = 189
		a.TargetVelocityCountsPerS = -193
		a.CollisionArmed = 1
		a.CollisionTorqueCycles = 201
		a.CollisionFollowingErrorCycles = 205
		a.CollisionPeakTorqueRaw = 209
		a.CollisionLastTripQuantity = 213
		a.CollisionTripSustainedCycles = 217
		a.CollisionLastTripPeakTorqueRaw = 221
		a.BrakeState = 225
		a.NativeHomePositionOffset = -229
		a.NativeHomeLastAbortCode = 233
		a.ExtPositionErrorCounts = -237
		a.ExtMultiTurnLo = -241
		a.ExtMultiTurnHi = -245
		a.MaxAbsVelocityActualCountsPerS = 249
		a.MaxAbsFollowingErrorCounts = 253
		a.TorqueRaw = -257
		a.ExtBusVoltageRaw = 259
		a.ExtLoadRateRaw = 261
		a.ExtIgbtTempRaw = -263
		a.ExtMotorTempRaw = -265
		a.ExtDriveNotReadyBits = 267
		a.ExtMotorNotRotatingCode = 269
		a.ExtValidMask = 1
		a.ModeDisplay = -8
		a.Ds402State = 1
		a.TorqueValid = 1
		a.CoordinateValid = 1
		a.CoordinateSourceValid = 1
		a.NativeHomeState = 1
		a.SlaveAlState = 1
		a.SlaveOnline = 1
		a.SlaveOperational = 1
		a.PdoFresh = 1
		a.TargetVelocityValid = 1
		a.CalibrationValid = 1; a.PositionState = 2; a.AuditReason = 13
		a.AuditLastVerifiedNs = 9007199254740993 + uint64(i)
		a.AuditStartedNs = 1000 + uint64(i); a.AuditCompletedNs = 2000 + uint64(i)
		a.AuditCoherenceErrorCounts = 50 + uint64(i)
		a.ExtValid = 1
		return a
	}
	record := wire.CycleCaptureRecordV2{TNs: 123, Seq: 7, Cycle: 8, WorkNs: 900, Axes: 16}
	record.CycleJitterNs = -9007199254741033
	record.ActiveTrajId = 9007199254741041
	record.ActiveCommandSeq = 9007199254741049
	record.JogGeneration = 9007199254741057
	record.AppGrantGeneration = 9007199254741065
	record.ControlGeneration = 9007199254741073
	record.ConfigurationEpoch = 9007199254741081
	record.HomeEpoch = 9007199254741089
	record.GrantDeadlineHostNs = 9007199254741097
	record.JogInputDeadlineHostNs = 9007199254741105
	record.JogSourceSequence = 9007199254741113
	record.BusSubmissionReturnCode = -9007199254741121
	record.Armed = 1
	record.AxisEnableMask = 141
	record.ActiveMode = 145
	record.State = 149
	record.CurrentPointIndex = 153
	record.QueueDepth = 157
	record.LastEventCode = 161
	record.UnderrunCount = 165
	record.StaleCommandFlag = 169
	record.MotionDone = 1
	record.CapabilityFlags = 177
	record.JogSessionOpen = 1
	record.JogHasInput = 1
	record.JogAxisMask = 189
	record.JogRamping = 1
	record.GrantActive = 1
	record.GrantAxisMask = 201
	record.SafetyFaultMask = 205
	record.ExecutionFaultReasons = 209
	record.SubmittedAxisMask = 213
	record.BusSubmissionOperation = 217
	record.PdoFreshAxisMask = 221
	record.ConfigVerifiedMask = 225
	record.HomeValidMask = 1
	record.ServiceModeAxisMask = 233
	record.WkcActual = 237
	record.WkcExpected = 241
	record.MasterState = 245
	record.IoConfigured = 1
	record.IoInputWord = 253
	record.IoOutputWord = 257
	record.IoInputValid = 1
	record.IoArmed = 1
	record.IoReason = 269
	for i := range record.Ax {
		record.Ax[i] = axis(i)
	}
	mark := wire.TelemetryMarkV1{Length: 128}
	header := wire.TelemetryBatchHeaderV1{Magic: 0x31425452, Version: 1, HeaderBytes: 312, CyclePeriodNs: 1000000, FirstSequence: 9007199254740993, LastSequence: 9007199254740994, Dropped: 7, AxisCount: 16, RecordCount: 2, RecordBytes: 5392}
	for i := range header.AdapterIncarnation {
		header.AdapterIncarnation[i] = 'a'
		header.DaemonIncarnation[i] = 'e'
	}
	for i := range header.MachineSha256 {
		header.MachineSha256[i] = 'b'
		header.DeploymentSha256[i] = 'c'
	}
	copy(header.RecordLayoutDigest[:], "1173439681672348f35f9652f2a4821d251638c70b60362ff59b37032a794b6b")
	for i := range mark.Label {
		mark.Label[i] = byte(i)
	}
	return []goldenCase{
		{name: "TelemetryBatchHeaderV1", size: 312, expected: header,
			decode: func(p []byte) (any, error) { return wire.DecodeTelemetryBatchHeaderV1(p) },
			encode: func(v any) []byte {
				p := wire.EncodeTelemetryBatchHeaderV1(v.(wire.TelemetryBatchHeaderV1))
				return p[:]
			}},
		{name: "TelemetryMarkV1", size: 136, expected: mark,
			decode: func(p []byte) (any, error) { return wire.DecodeTelemetryMarkV1(p) },
			encode: func(v any) []byte { p := wire.EncodeTelemetryMarkV1(v.(wire.TelemetryMarkV1)); return p[:] }},
		{name: "CycleCaptureAxisV2", size: 320, expected: axis(0),
			decode: func(p []byte) (any, error) { return wire.DecodeCycleCaptureAxisV2(p) },
			encode: func(v any) []byte { p := wire.EncodeCycleCaptureAxisV2(v.(wire.CycleCaptureAxisV2)); return p[:] }},
		{name: "CycleCaptureRecordV2", size: 5392, expected: record,
			decode: func(p []byte) (any, error) { return wire.DecodeCycleCaptureRecordV2(p) },
			encode: func(v any) []byte { p := wire.EncodeCycleCaptureRecordV2(v.(wire.CycleCaptureRecordV2)); return p[:] }},
	}
}

func TestTelemetryNestedLayoutChangesDigest(t *testing.T) {
	s := productionSchema(t)
	original, err := generate(s)
	if err != nil {
		t.Fatal(err)
	}
	for i := range s.wireRecords {
		if s.wireRecords[i].Name == "CycleCaptureAxisV2" {
			s.wireRecords[i].Fields = append([]wireField(nil), s.wireRecords[i].Fields...)
			s.wireRecords[i].Fields[0].Name = "changed_position"
		}
	}
	changed, err := generate(s)
	if err != nil {
		t.Fatal(err)
	}
	digest := func(files []artifact) string {
		for _, file := range files {
			if file.path == "ipcclient/protocol_generated.go" {
				for _, line := range strings.Split(string(file.data), "\n") {
					if strings.HasPrefix(line, "const CycleCaptureRecordV2LayoutDigest =") {
						return line
					}
				}
			}
		}
		return ""
	}
	if digest(original) == "" || digest(original) == digest(changed) {
		t.Fatal("nested layout change retained digest")
	}
}
