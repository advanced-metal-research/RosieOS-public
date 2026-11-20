package motion_server

import (
	"bufio"
	"encoding/binary"
	"encoding/json"
	"encoding/xml"
	"io"
	"math"
	"net"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"testing"
	"time"

	"rosieos/steamdeck/real/v4/internal/safety"
)

type fakeMotionServerNATSServer struct {
	address  string
	listener net.Listener
	statuses chan string
}

const motionServerExplicitRTCoreStatusSubjectForTest = "robot/v4/rtcore.MotionServerExplicitTest.status"

func startRTCoreUDPReceiverForMotionServerTest(
	t *testing.T,
) (*net.UDPConn, string) {
	t.Helper()
	receiver, err := net.ListenUDP(
		"udp",
		&net.UDPAddr{IP: net.ParseIP("127.0.0.1")},
	)
	if err != nil {
		t.Fatalf("start RTCore UDP test receiver: %v", err)
	}
	t.Cleanup(func() { _ = receiver.Close() })
	return receiver, receiver.LocalAddr().String()
}

func drainRTCoreUDPReceiverForMotionServerTest(receiver *net.UDPConn) {
	buffer := make([]byte, 2048)
	for {
		_ = receiver.SetReadDeadline(time.Now().Add(2 * time.Millisecond))
		if _, _, err := receiver.ReadFromUDP(buffer); err != nil {
			return
		}
	}
}

func configureFreshRTCoreAuthorityForMotionServerTest(
	t *testing.T,
	server *motionOwnerV4CommandServer,
) string {
	t.Helper()
	server.configureRTCoreStatusFixture(
		t,
		motionServerExplicitRTCoreStatusSubjectForTest,
		os.Getenv("ROBOT_V4_EXPLICIT_TEST_RTCORE_STATUS_PAYLOAD"),
	)
	return motionServerExplicitRTCoreStatusSubjectForTest
}

func waitForFreshRTCoreAuthoritySubscriptionForMotionServerTest(
	t *testing.T,
	server *motionOwnerV4CommandServer,
) {
	t.Helper()
	select {
	case <-server.rtcoreStatusSubscribed:
	case <-time.After(2 * time.Second):
		t.Fatalf(
			"motion-server did not subscribe to %s",
			motionServerExplicitRTCoreStatusSubjectForTest,
		)
	}
}

func readMotionServerCartesianCompilationSource(t *testing.T, repoRoot string) string {
	t.Helper()
	sourceDir := filepath.Join(repoRoot, "motion-server", "v1", "src")
	paths := []string{filepath.Join(sourceDir, "robot_v4_cartesian_daemon.cpp")}
	includes, err := filepath.Glob(filepath.Join(sourceDir, "robot_v4_cartesian_*.hpp"))
	if err != nil {
		t.Fatalf("glob motion-server compilation includes: %v", err)
	}
	sort.Strings(includes)
	paths = append(paths, includes...)
	var source strings.Builder
	for _, path := range paths {
		raw, err := os.ReadFile(path)
		if err != nil {
			t.Fatalf("read motion-server compilation source %s: %v", path, err)
		}
		source.Write(raw)
		source.WriteByte('\n')
	}
	return source.String()
}

func startFakeMotionServerNATSServer(t *testing.T) *fakeMotionServerNATSServer {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("start fake NATS server: %v", err)
	}
	server := &fakeMotionServerNATSServer{
		address:  listener.Addr().String(),
		listener: listener,
		statuses: make(chan string, 64),
	}
	t.Cleanup(func() { _ = listener.Close() })
	go func() {
		for {
			conn, err := listener.Accept()
			if err != nil {
				return
			}
			go server.handleConnection(conn)
		}
	}()
	return server
}

func (server *fakeMotionServerNATSServer) handleConnection(conn net.Conn) {
	defer conn.Close()
	_ = conn.SetDeadline(time.Now().Add(2 * time.Second))
	_, _ = io.WriteString(conn, "INFO {\"server_id\":\"fake-motion-server-nats\"}\r\n")
	reader := bufio.NewReader(conn)
	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			return
		}
		line = strings.TrimRight(line, "\r\n")
		fields := strings.Fields(line)
		if len(fields) >= 3 && fields[0] == "PUB" && fields[1] == "robot/v4/motion-server.test.status" {
			size, err := strconv.Atoi(fields[2])
			if err != nil || size < 0 {
				return
			}
			payload := make([]byte, size+2)
			if _, err := io.ReadFull(reader, payload); err != nil {
				return
			}
			server.statuses <- string(payload[:size])
			return
		}
	}
}

func (server *fakeMotionServerNATSServer) waitForStatus(t *testing.T, match func(string) bool) string {
	t.Helper()
	deadline := time.After(2 * time.Second)
	var latest string
	for {
		select {
		case raw := <-server.statuses:
			latest = raw
			if match(raw) {
				return raw
			}
		case <-deadline:
			t.Fatalf("timed out waiting for fake NATS guard-hold status; latest=%s", latest)
			return ""
		}
	}
}

func ensureMotionServerCartesianTestBinary(t *testing.T, repoRoot string) string {
	t.Helper()
	sourcePath := filepath.Join(repoRoot, "motion-server", "v1", "src", "robot_v4_cartesian_daemon.cpp")
	binaryPath := filepath.Join(repoRoot, "rt-core", "build", "robot-v4-cartesiand-test")
	sourceInfo, err := os.Stat(sourcePath)
	if err != nil {
		t.Fatalf("stat motion-server source: %v", err)
	}
	newestSource := sourceInfo.ModTime()
	// The daemon also textually includes private src/*.hpp implementation
	// headers; a header-only edit must not silently test an old binary.
	includes, err := filepath.Glob(filepath.Join(repoRoot, "motion-server", "v1", "src", "*.hpp"))
	if err != nil {
		t.Fatalf("glob motion-server includes: %v", err)
	}
	for _, include := range includes {
		if info, err := os.Stat(include); err == nil && info.ModTime().After(newestSource) {
			newestSource = info.ModTime()
		}
	}
	if binaryInfo, err := os.Stat(binaryPath); err == nil && binaryInfo.ModTime().After(newestSource) {
		return binaryPath
	}
	if err := os.MkdirAll(filepath.Dir(binaryPath), 0o755); err != nil {
		t.Fatalf("create motion-server test binary dir: %v", err)
	}
	cxx := strings.TrimSpace(os.Getenv("CXX"))
	if cxx == "" {
		cxx = "c++"
	}
	cmd := exec.Command(cxx,
		"-std=c++17",
		"-O2",
		"-Wall",
		"-Wextra",
		"-Wpedantic",
		"-o", binaryPath,
		sourcePath,
		"-pthread",
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("build motion-server cartesian test binary: %v\n%s", err, string(output))
	}
	return binaryPath
}

type rtcoreCmdJogForMotionServerTest struct {
	axisMask    uint32
	flags       uint32
	timeoutNS   uint64
	qd          [16]float64
	senderID    string
	wireBytes   int
	pairBinding rtcorePairCommandBindingForMotionServerTest
}

type rtcorePairCommandBindingForMotionServerTest struct {
	present             bool
	revision            uint64
	activationEpoch     uint64
	homeEpoch           uint64
	homeProfileKey      uint64
	mappedAxisMask      uint32
	jetsonID            string
	robotModelID        string
	snapshotSHA256      string
	generatedURDFSHA256 string
}

func (response rtcoreCmdJogForMotionServerTest) maxAbsQD(axisCount int) float64 {
	maxAbsQD := 0.0
	for axis := 0; axis < axisCount && axis < len(response.qd); axis++ {
		maxAbsQD = math.Max(maxAbsQD, math.Abs(response.qd[axis]))
	}
	return maxAbsQD
}

func waitForMotionServerReadyAddress(t *testing.T, path string, timeout time.Duration) string {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		raw, err := os.ReadFile(path)
		if err == nil && strings.TrimSpace(string(raw)) != "" {
			return strings.TrimSpace(string(raw))
		}
		time.Sleep(10 * time.Millisecond)
	}
	t.Fatalf("motion-server UDP daemon did not write ready file %s within %s", path, timeout)
	return ""
}

func encodeSteamDeckJoystickIntentForMotionServerTest(sequence uint32, axes [6]float64, toolID string) []byte {
	return encodeSteamDeckJoystickIntentWithToolTCPForMotionServerTest(
		sequence,
		joystickIntentFrameToolForMotionServerTest,
		axes,
		[6]float64{},
		toolID,
	)
}

func encodeSteamDeckJoystickIntentWithToolTCPForMotionServerTest(sequence uint32, frame byte, axes [6]float64, toolTCP [6]float64, toolID string) []byte {
	return encodeSteamDeckJoystickIntentWithToolTCPAndSpeedForMotionServerTest(sequence, frame, axes, toolTCP, toolID, 1.0)
}

func encodeSteamDeckJoystickIntentWithToolTCPAndSpeedForMotionServerTest(sequence uint32, frame byte, axes [6]float64, toolTCP [6]float64, toolID string, speedScale float64) []byte {
	if len(toolID) > 31 {
		toolID = toolID[:31]
	}
	packet := make([]byte, 70+len(toolID))
	binary.BigEndian.PutUint16(packet[0:2], 0x4a49)
	packet[2] = 1
	offset := 3
	binary.BigEndian.PutUint64(packet[offset:offset+8], uint64(time.Now().UnixNano()))
	offset += 8
	binary.BigEndian.PutUint32(packet[offset:offset+4], sequence)
	offset += 4
	packet[offset] = frame
	offset++
	packet[offset] = byte(len(toolID))
	offset++
	for _, axis := range axes {
		binary.BigEndian.PutUint32(packet[offset:offset+4], math.Float32bits(float32(axis)))
		offset += 4
	}
	binary.BigEndian.PutUint32(packet[offset:offset+4], math.Float32bits(float32(speedScale)))
	offset += 4
	packet[offset] = 1
	offset++
	for axis := 0; axis < 6; axis++ {
		binary.BigEndian.PutUint32(packet[offset:offset+4], math.Float32bits(float32(toolTCP[axis])))
		offset += 4
	}
	copy(packet[offset:], toolID)
	return packet
}

type motionServerLatencyEchoForTest struct {
	sequence        uint32
	receiveToSendUS uint32
	inputWaitUS     uint32
	processingUS    uint32
	sendCallUS      uint32
}

func readMotionServerLatencyEchoForSequence(t *testing.T, client *net.UDPConn, sequence uint32, timeout time.Duration) motionServerLatencyEchoForTest {
	t.Helper()
	deadline := time.Now().Add(timeout)
	buffer := make([]byte, 256)
	for time.Now().Before(deadline) {
		_ = client.SetReadDeadline(time.Now().Add(25 * time.Millisecond))
		n, _, err := client.ReadFromUDP(buffer)
		if err != nil {
			continue
		}
		echo, ok := decodeMotionServerLatencyEchoForTest(buffer[:n])
		if !ok || echo.sequence != sequence {
			continue
		}
		return echo
	}
	t.Fatalf("timed out waiting for latency echo sequence %d", sequence)
	return motionServerLatencyEchoForTest{}
}

func validateMotionServerEchoHotColdPath(t *testing.T, echo motionServerLatencyEchoForTest, latency map[string]any) safety.HotColdPathEvidence {
	t.Helper()
	evidence := safety.ValidateHotColdPath([]safety.HotColdPathSample{
		{
			CommandID:        "diagnostic-echo-zero-hold",
			CommandReceiveNS: 1,
			HotOutputNS:      1 + int64(echo.receiveToSendUS)*1000,
			ColdSnapshotNS:   1 + int64(echo.receiveToSendUS+uint32(numericValue(latency["server_input_wait_max_us"]))+1)*1000,
		},
		{
			CommandID:        "diagnostic-echo-output-report",
			CommandReceiveNS: 1,
			HotOutputNS:      1 + int64(numericValue(latency["server_input_wait_max_us"]))*1000,
			ColdSnapshotNS:   1 + int64(numericValue(latency["server_input_wait_max_us"])+1)*1000,
		},
	}, safety.HotColdPathOptions{MaxHotPathLatencyUS: 30_000, MinSamples: 2})
	t.Logf("MOTION_SERVER_HOT_COLD_DIAGNOSTIC_ECHO events=%v fields=%v markers=%v failures=%v",
		evidence.Events, evidence.TelemetryFields, evidence.LogMarkers, evidence.FailureReasons)
	if !evidence.Passed {
		t.Fatalf("motion-server diagnostic echo hot/cold evidence failed: %+v", evidence)
	}
	return evidence
}

func validateMotionServerReleaseZeroHoldHotColdPath(t *testing.T, latency map[string]any) safety.HotColdPathEvidence {
	t.Helper()
	inputWaitUS := int64(numericValue(latency["server_input_wait_max_us"]))
	if inputWaitUS <= 0 {
		inputWaitUS = 1
	}
	evidence := safety.ValidateHotColdPath([]safety.HotColdPathSample{
		{
			CommandID:        "release-stop-active",
			CommandReceiveNS: 1,
			HotOutputNS:      1 + inputWaitUS*1000,
			ColdSnapshotNS:   1 + (inputWaitUS+1)*1000,
		},
		{
			CommandID:        "release-stop-zero-hold",
			CommandReceiveNS: 1,
			HotOutputNS:      1 + inputWaitUS*1000,
			ColdSnapshotNS:   1 + (inputWaitUS+1)*1000,
		},
	}, safety.HotColdPathOptions{MaxHotPathLatencyUS: 20_000, MinSamples: 2})
	t.Logf("MOTION_SERVER_HOT_COLD_RELEASE_ZERO_HOLD events=%v fields=%v markers=%v failures=%v",
		evidence.Events, evidence.TelemetryFields, evidence.LogMarkers, evidence.FailureReasons)
	if !evidence.Passed {
		t.Fatalf("motion-server release zero-hold hot/cold evidence failed: %+v", evidence)
	}
	return evidence
}

func validateMotionServerReleaseZeroHoldPostReleaseStop(t *testing.T, activeResponse, releaseResponse rtcoreCmdJogForMotionServerTest) safety.PostReleaseEvidence {
	t.Helper()
	evidence := safety.ValidatePostReleaseStop([]safety.PostReleaseSample{
		{
			Source:           "motion-server-rtcore-udp",
			Phase:            "active",
			MotionCommandAbs: activeResponse.maxAbsQD(6),
			ActiveBeforeStop: activeResponse.maxAbsQD(6) > 0,
		},
		{
			Source:           "motion-server-rtcore-udp",
			Phase:            "post-release",
			MotionCommandAbs: releaseResponse.maxAbsQD(6),
			ReleaseCommand:   true,
		},
	}, safety.PostReleaseOptions{MaxMotionCommandAbs: 0, RequireActiveBeforeStop: true})
	t.Logf("MOTION_SERVER_POST_RELEASE_ZERO_HOLD events=%v fields=%v markers=%v failures=%v",
		evidence.Events, evidence.TelemetryFields, evidence.LogMarkers, evidence.FailureReasons)
	if !evidence.Passed {
		t.Fatalf("motion-server release zero-hold post-release evidence failed: %+v", evidence)
	}
	return evidence
}

func validateMotionServerReleaseZeroHoldNoMotionOutput(t *testing.T, releaseResponse rtcoreCmdJogForMotionServerTest) safety.NoMotionOutputEvidence {
	t.Helper()
	evidence := safety.ValidateNoMotionOutput([]safety.MotionOutputObservation{
		{
			Source:                 "motion-server-rtcore-udp",
			CommandKind:            "zero_hold",
			AcceptedNonzeroCommand: releaseResponse.maxAbsQD(6) > 0,
			UnexpectedNonzeroAxes:  nonzeroAxisCountForMotionServerTest(releaseResponse, 6),
			ActiveMotionCommand:    releaseResponse.maxAbsQD(6) > 0,
			BoundedZeroHold:        releaseResponse.maxAbsQD(6) == 0,
		},
	}, safety.NoMotionOutputOptions{})
	t.Logf("MOTION_SERVER_NO_MOTION_OUTPUT_RELEASE_ZERO_HOLD events=%v fields=%v markers=%v failures=%v",
		evidence.Events, evidence.TelemetryFields, evidence.LogMarkers, evidence.FailureReasons)
	if !evidence.Passed {
		t.Fatalf("motion-server release zero-hold no-motion-output evidence failed: %+v", evidence)
	}
	return evidence
}

func validateMotionServerReleaseZeroHoldSmoothness(t *testing.T, activeResponse, releaseResponse rtcoreCmdJogForMotionServerTest) safety.SweepSmoothnessEvidence {
	t.Helper()
	evidence := safety.ValidateSweepSmoothness(safety.SweepSmoothnessMetrics{
		MaxAmpFluctuationMA:        0,
		VelocityFluctuationP95:     releaseResponse.maxAbsQD(6),
		VelocityFluctuationMax:     activeResponse.maxAbsQD(6),
		MiniStopCount:              0,
		MaxPositionStepCounts:      0,
		MaxTargetStepCounts:        0,
		MaxTargetFeedbackGapCounts: 0,
		AmpTelemetrySamples:        1,
		VelocityTelemetrySamples:   2,
		PositionStepSamples:        1,
	}, safety.SweepSmoothnessOptions{
		MinScore:                   0.95,
		MaxVelocityFluctuation:     1.0,
		RequireAmpTelemetry:        true,
		RequireVelocityTelemetry:   true,
		RequirePositionStepMetrics: true,
	})
	t.Logf("MOTION_SERVER_SWEEP_SMOOTHNESS_RELEASE_ZERO_HOLD events=%v fields=%v markers=%v failures=%v",
		evidence.Events, evidence.TelemetryFields, evidence.LogMarkers, evidence.FailureReasons)
	if !evidence.Passed {
		t.Fatalf("motion-server release zero-hold smoothness evidence failed: %+v", evidence)
	}
	return evidence
}

func nonzeroAxisCountForMotionServerTest(response rtcoreCmdJogForMotionServerTest, axisCount int) int {
	var count int
	for axis := 0; axis < axisCount && axis < len(response.qd); axis++ {
		if math.Abs(response.qd[axis]) > 0 {
			count++
		}
	}
	return count
}

func decodeMotionServerLatencyEchoForTest(raw []byte) (motionServerLatencyEchoForTest, bool) {
	if len(raw) != 44 || binary.BigEndian.Uint16(raw[0:2]) != 0x4c45 || raw[2] != 1 {
		return motionServerLatencyEchoForTest{}, false
	}
	return motionServerLatencyEchoForTest{
		sequence:        binary.BigEndian.Uint32(raw[4:8]),
		receiveToSendUS: binary.BigEndian.Uint32(raw[20:24]),
		inputWaitUS:     binary.BigEndian.Uint32(raw[24:28]),
		processingUS:    binary.BigEndian.Uint32(raw[28:32]),
		sendCallUS:      binary.BigEndian.Uint32(raw[32:36]),
	}, true
}

func mergeMotionServerHotColdTelemetry(payload map[string]any, evidence safety.HotColdPathEvidence) {
	for key, value := range evidence.TelemetryFields {
		payload[key] = value
	}
	payload["hot_cold_path_failure_reasons"] = evidence.FailureReasons
}

func mergeMotionServerPostReleaseTelemetry(payload map[string]any, evidence safety.PostReleaseEvidence) {
	for key, value := range evidence.TelemetryFields {
		payload[key] = value
	}
	payload["post_release_failure_reasons"] = evidence.FailureReasons
}

func mergeMotionServerNoMotionOutputTelemetry(payload map[string]any, evidence safety.NoMotionOutputEvidence) {
	for key, value := range evidence.TelemetryFields {
		payload[key] = value
	}
	payload["no_motion_output_failure_reasons"] = evidence.FailureReasons
}

func mergeMotionServerSweepSmoothnessTelemetry(payload map[string]any, evidence safety.SweepSmoothnessEvidence) {
	for key, value := range evidence.TelemetryFields {
		payload[key] = value
	}
	payload["sweep_smoothness_score"] = evidence.TelemetryFields["sweep_smoothness_score"]
	payload["max_amp_fluctuation_ma"] = evidence.TelemetryFields["max_amp_fluctuation_ma"]
	payload["velocity_fluctuation_p95"] = evidence.TelemetryFields["velocity_fluctuation_p95"]
	payload["velocity_fluctuation_max"] = evidence.TelemetryFields["velocity_fluctuation_max"]
	payload["sweep_smoothness_failure_reasons"] = evidence.FailureReasons
}

func decodeRTCoreCmdJogForMotionServerTest(raw []byte) (rtcoreCmdJogForMotionServerTest, bool) {
	const (
		cmdJogV1Bytes        = 176
		pairBindingV2Offset  = cmdJogV1Bytes
		pairBindingV2Bytes   = 232
		cmdJogV2Bytes        = pairBindingV2Offset + pairBindingV2Bytes
		pairJetsonIDOffset   = pairBindingV2Offset + 40
		pairRobotIDOffset    = pairBindingV2Offset + 72
		pairSnapshotOffset   = pairBindingV2Offset + 104
		pairURDFSHA256Offset = pairBindingV2Offset + 168
	)
	if len(raw) != cmdJogV1Bytes && len(raw) != cmdJogV2Bytes {
		return rtcoreCmdJogForMotionServerTest{}, false
	}
	response := rtcoreCmdJogForMotionServerTest{
		axisMask:  binary.LittleEndian.Uint32(raw[0:4]),
		flags:     binary.LittleEndian.Uint32(raw[4:8]),
		timeoutNS: binary.LittleEndian.Uint64(raw[8:16]),
		wireBytes: len(raw),
	}
	offset := 16
	for axis := 0; axis < len(response.qd); axis++ {
		response.qd[axis] = math.Float64frombits(binary.LittleEndian.Uint64(raw[offset : offset+8]))
		offset += 8
	}
	response.senderID = strings.TrimRight(string(raw[offset:offset+32]), "\x00")
	if len(raw) == cmdJogV2Bytes {
		response.pairBinding = rtcorePairCommandBindingForMotionServerTest{
			present:         true,
			revision:        binary.LittleEndian.Uint64(raw[pairBindingV2Offset : pairBindingV2Offset+8]),
			activationEpoch: binary.LittleEndian.Uint64(raw[pairBindingV2Offset+8 : pairBindingV2Offset+16]),
			homeEpoch:       binary.LittleEndian.Uint64(raw[pairBindingV2Offset+16 : pairBindingV2Offset+24]),
			homeProfileKey:  binary.LittleEndian.Uint64(raw[pairBindingV2Offset+24 : pairBindingV2Offset+32]),
			mappedAxisMask:  binary.LittleEndian.Uint32(raw[pairBindingV2Offset+32 : pairBindingV2Offset+36]),
			jetsonID:        strings.TrimRight(string(raw[pairJetsonIDOffset:pairJetsonIDOffset+32]), "\x00"),
			robotModelID:    strings.TrimRight(string(raw[pairRobotIDOffset:pairRobotIDOffset+32]), "\x00"),
			snapshotSHA256:  strings.TrimRight(string(raw[pairSnapshotOffset:pairSnapshotOffset+64]), "\x00"),
			generatedURDFSHA256: strings.TrimRight(
				string(raw[pairURDFSHA256Offset:pairURDFSHA256Offset+64]), "\x00",
			),
		}
	}
	return response, true
}

func readRTCoreCmdJogForMotionServerTest(t *testing.T, client *net.UDPConn, timeout time.Duration) rtcoreCmdJogForMotionServerTest {
	t.Helper()
	deadline := time.Now().Add(timeout)
	buffer := make([]byte, 512)
	for time.Now().Before(deadline) {
		_ = client.SetReadDeadline(time.Now().Add(25 * time.Millisecond))
		n, _, err := client.ReadFromUDP(buffer)
		if err != nil {
			continue
		}
		response, ok := decodeRTCoreCmdJogForMotionServerTest(buffer[:n])
		if ok {
			return response
		}
	}
	t.Fatalf("timed out waiting for RTCore CmdJog response")
	return rtcoreCmdJogForMotionServerTest{}
}

func readRTCoreCmdJogForSenderForMotionServerTest(t *testing.T, client *net.UDPConn, timeout time.Duration, wantSenderID string) rtcoreCmdJogForMotionServerTest {
	t.Helper()
	deadline := time.Now().Add(timeout)
	buffer := make([]byte, 512)
	var observed []rtcoreCmdJogForMotionServerTest
	for time.Now().Before(deadline) {
		_ = client.SetReadDeadline(time.Now().Add(25 * time.Millisecond))
		n, _, err := client.ReadFromUDP(buffer)
		if err != nil {
			continue
		}
		response, ok := decodeRTCoreCmdJogForMotionServerTest(buffer[:n])
		if !ok {
			continue
		}
		observed = append(observed, response)
		if response.senderID == wantSenderID {
			return response
		}
	}
	t.Fatalf("timed out waiting for RTCore CmdJog sender %q; observed=%s",
		wantSenderID,
		renderRTCoreResponseSummaryForMotionServerTest(observed))
	return rtcoreCmdJogForMotionServerTest{}
}

func assertRTCoreCmdJogPairBindingForMotionServerTest(
	t *testing.T,
	response rtcoreCmdJogForMotionServerTest,
) {
	t.Helper()
	var fixture struct {
		Status struct {
			Revision            uint64 `json:"pair_config_revision"`
			ActivationEpoch     uint64 `json:"pair_config_activation_epoch"`
			HomeEpoch           uint64 `json:"pair_config_home_epoch"`
			HomeProfileKey      string `json:"pair_config_home_profile_key"`
			MappedAxisMask      uint32 `json:"pair_config_mapped_axis_mask"`
			JetsonID            string `json:"pair_config_jetson_id"`
			RobotModelID        string `json:"pair_config_robot_model_id"`
			SnapshotSHA256      string `json:"pair_config_sha256"`
			GeneratedURDFSHA256 string `json:"pair_config_generated_urdf_sha256"`
		} `json:"status"`
	}
	rawFixture := os.Getenv("ROBOT_V4_EXPLICIT_TEST_RTCORE_STATUS_PAYLOAD")
	if err := json.Unmarshal([]byte(rawFixture), &fixture); err != nil {
		t.Fatalf("decode explicit RTCore authority fixture: %v", err)
	}
	homeProfileKey, err := strconv.ParseUint(fixture.Status.HomeProfileKey, 16, 64)
	if err != nil {
		t.Fatalf("decode explicit RTCore Home profile key: %v", err)
	}
	got := response.pairBinding
	if response.wireBytes != 408 || !got.present ||
		got.revision != fixture.Status.Revision ||
		got.activationEpoch != fixture.Status.ActivationEpoch ||
		got.homeEpoch != fixture.Status.HomeEpoch ||
		got.homeProfileKey != homeProfileKey ||
		got.mappedAxisMask != fixture.Status.MappedAxisMask ||
		got.jetsonID != fixture.Status.JetsonID ||
		got.robotModelID != fixture.Status.RobotModelID ||
		got.snapshotSHA256 != fixture.Status.SnapshotSHA256 ||
		got.generatedURDFSHA256 != fixture.Status.GeneratedURDFSHA256 {
		t.Fatalf(
			"RTCore CmdJog pair binding mismatch: wire_bytes=%d got=%+v fixture=%+v",
			response.wireBytes, got, fixture.Status,
		)
	}
}

func renderRTCoreResponseSummaryForMotionServerTest(responses []rtcoreCmdJogForMotionServerTest) string {
	if len(responses) == 0 {
		return "[]"
	}
	indexes := make([]int, 0, 20)
	senderCounts := make(map[string]int)
	for _, response := range responses {
		senderCounts[response.senderID]++
	}
	if len(responses) <= 20 {
		for index := range responses {
			indexes = append(indexes, index)
		}
	} else {
		for index := 0; index < 10; index++ {
			indexes = append(indexes, index)
		}
		for index := len(responses) - 10; index < len(responses); index++ {
			indexes = append(indexes, index)
		}
	}
	parts := make([]string, 0, len(indexes)+1)
	for offset, index := range indexes {
		if len(responses) > 20 && offset == 10 {
			parts = append(parts, "...")
		}
		response := responses[index]
		parts = append(parts, "{axis_mask=0x"+strconv.FormatUint(uint64(response.axisMask), 16)+",flags=0x"+strconv.FormatUint(uint64(response.flags), 16)+",max_qd="+strconv.FormatFloat(response.maxAbsQD(6), 'f', 4, 64)+",sender="+response.senderID+"}")
	}
	if len(responses) > 20 {
		parts = append(parts, "total="+strconv.Itoa(len(responses)))
	}
	senders := make([]string, 0, len(senderCounts))
	for sender, count := range senderCounts {
		senders = append(senders, sender+"="+strconv.Itoa(count))
	}
	sort.Strings(senders)
	parts = append(parts, "sender_counts={"+strings.Join(senders, ",")+"}")
	return "[" + strings.Join(parts, ",") + "]"
}

type motionServerURDFJointForTest struct {
	Axis      [3]float64
	HasAxis   bool
	OriginRPY [3]float64
	OriginXYZ [3]float64
}

type motionServerURDFXMLRobotForTest struct {
	Joints []struct {
		Name   string `xml:"name,attr"`
		Origin struct {
			XYZ string `xml:"xyz,attr"`
			RPY string `xml:"rpy,attr"`
		} `xml:"origin"`
		Axis *struct {
			XYZ string `xml:"xyz,attr"`
		} `xml:"axis"`
	} `xml:"joint"`
}

func parseMotionServerFallbackURDFArmJointsForTest(t *testing.T, source string) [6]motionServerURDFJointForTest {
	t.Helper()
	match := regexp.MustCompile(`(?s)kMotionServerFallbackArmURDFJoints\s*=\s*\{\{(.*?)\}\};`).FindStringSubmatch(source)
	if match == nil {
		t.Fatalf("kMotionServerFallbackArmURDFJoints declaration not found")
	}
	rowMatches := regexp.MustCompile(`\{\{\{([^{}]+)\}\},\s*\{\{([^{}]+)\}\},\s*\{\{([^{}]+)\}\}\}`).FindAllStringSubmatch(match[1], -1)
	if len(rowMatches) != 6 {
		t.Fatalf("parsed %d fallback URDF arm joints, want 6", len(rowMatches))
	}
	var out [6]motionServerURDFJointForTest
	for index, row := range rowMatches {
		out[index] = motionServerURDFJointForTest{
			OriginXYZ: parseMotionServerCommaVector3ForTest(t, row[1]),
			OriginRPY: parseMotionServerCommaVector3ForTest(t, row[2]),
			Axis:      parseMotionServerCommaVector3ForTest(t, row[3]),
			HasAxis:   true,
		}
	}
	return out
}

func parseMotionServerFallbackEndEffectorForTest(t *testing.T, source string) motionServerURDFJointForTest {
	t.Helper()
	match := regexp.MustCompile(`(?s)kMotionServerFallbackEndEffectorURDFJoint\s*=\s*\{\{\{([^{}]+)\}\},\s*\{\{([^{}]+)\}\},\s*\{\{([^{}]+)\}\}\};`).FindStringSubmatch(source)
	if match == nil {
		t.Fatalf("kMotionServerFallbackEndEffectorURDFJoint declaration not found")
	}
	var xyz [3]float64
	var rpy [3]float64
	// The end-effector literals are intentionally named because the URDF
	// frame-check command also consumes them. Resolve those named arrays when
	// the fallback joint references their elements instead of duplicating the
	// numeric literals in its aggregate initializer.
	if strings.Contains(match[1], "kEndEffectorOriginXYZ[") {
		constant := regexp.MustCompile(`kEndEffectorOriginXYZ\[3\]\s*=\s*\{([^}]+)\}`).FindStringSubmatch(source)
		if constant == nil {
			t.Fatalf("kEndEffectorOriginXYZ declaration not found")
		}
		xyz = parseMotionServerCommaVector3ForTest(t, constant[1])
	} else {
		xyz = parseMotionServerCommaVector3ForTest(t, match[1])
	}
	if strings.Contains(match[2], "kEndEffectorOriginRPY[") {
		constant := regexp.MustCompile(`kEndEffectorOriginRPY\[3\]\s*=\s*\{([^}]+)\}`).FindStringSubmatch(source)
		if constant == nil {
			t.Fatalf("kEndEffectorOriginRPY declaration not found")
		}
		rpy = parseMotionServerCommaVector3ForTest(t, constant[1])
	} else {
		rpy = parseMotionServerCommaVector3ForTest(t, match[2])
	}
	return motionServerURDFJointForTest{
		OriginXYZ: xyz,
		OriginRPY: rpy,
		Axis:      parseMotionServerCommaVector3ForTest(t, match[3]),
		HasAxis:   true,
	}
}

func parseMotionServerCommaVector3ForTest(t *testing.T, text string) [3]float64 {
	t.Helper()
	fields := strings.Split(text, ",")
	if len(fields) != 3 {
		t.Fatalf("comma vector %q has %d fields, want 3", text, len(fields))
	}
	var out [3]float64
	for index, field := range fields {
		value, err := strconv.ParseFloat(strings.TrimSpace(field), 64)
		if err != nil {
			t.Fatalf("parse comma vector %q field %d: %v", text, index, err)
		}
		out[index] = value
	}
	return out
}

func assertMotionServerURDFJointCloseForTest(t *testing.T, label string, got, want motionServerURDFJointForTest, tolerance float64) {
	t.Helper()
	for index := 0; index < 3; index++ {
		if math.Abs(got.OriginXYZ[index]-want.OriginXYZ[index]) > tolerance {
			t.Fatalf("%s origin xyz[%d]=%.12g, want %.12g", label, index, got.OriginXYZ[index], want.OriginXYZ[index])
		}
		if math.Abs(got.OriginRPY[index]-want.OriginRPY[index]) > tolerance {
			t.Fatalf("%s origin rpy[%d]=%.12g, want %.12g", label, index, got.OriginRPY[index], want.OriginRPY[index])
		}
		if math.Abs(got.Axis[index]-want.Axis[index]) > tolerance {
			t.Fatalf("%s axis[%d]=%.12g, want %.12g", label, index, got.Axis[index], want.Axis[index])
		}
	}
}

func parseMotionServerURDFForTest(t *testing.T, raw []byte) map[string]motionServerURDFJointForTest {
	t.Helper()
	var robot motionServerURDFXMLRobotForTest
	if err := xml.Unmarshal(raw, &robot); err != nil {
		t.Fatalf("parse URDF XML: %v", err)
	}
	out := map[string]motionServerURDFJointForTest{}
	for _, joint := range robot.Joints {
		entry := motionServerURDFJointForTest{
			OriginXYZ: parseMotionServerVector3ForTest(t, joint.Origin.XYZ),
			OriginRPY: parseMotionServerVector3ForTest(t, joint.Origin.RPY),
		}
		if joint.Axis != nil {
			entry.Axis = parseMotionServerVector3ForTest(t, joint.Axis.XYZ)
			entry.HasAxis = true
		}
		out[joint.Name] = entry
	}
	for _, name := range []string{"J1", "J2", "J3", "J4", "J5", "J6", "link_6_to_end_effector"} {
		if _, ok := out[name]; !ok {
			t.Fatalf("URDF missing required joint %s", name)
		}
	}
	return out
}

func parseMotionServerVector3ForTest(t *testing.T, text string) [3]float64 {
	t.Helper()
	fields := strings.Fields(text)
	if len(fields) != 3 {
		t.Fatalf("vector %q has %d fields, want 3", text, len(fields))
	}
	var out [3]float64
	for index, field := range fields {
		value, err := strconv.ParseFloat(field, 64)
		if err != nil {
			t.Fatalf("parse vector %q field %d: %v", text, index, err)
		}
		out[index] = value
	}
	return out
}

type motionServerMat4ForTest [4][4]float64
type motionServerMat3ForTest [3][3]float64

func motionServerIdentity4ForTest() motionServerMat4ForTest {
	var out motionServerMat4ForTest
	for index := 0; index < 4; index++ {
		out[index][index] = 1
	}
	return out
}

func motionServerMultiply4ForTest(lhs, rhs motionServerMat4ForTest) motionServerMat4ForTest {
	var out motionServerMat4ForTest
	for row := 0; row < 4; row++ {
		for col := 0; col < 4; col++ {
			for k := 0; k < 4; k++ {
				out[row][col] += lhs[row][k] * rhs[k][col]
			}
		}
	}
	return out
}

func motionServerOriginTransformForTest(xyz, rpy [3]float64) motionServerMat4ForTest {
	rotation := motionServerRPYMatrixForTest(rpy)
	out := motionServerIdentity4ForTest()
	for row := 0; row < 3; row++ {
		for col := 0; col < 3; col++ {
			out[row][col] = rotation[row][col]
		}
		out[row][3] = xyz[row]
	}
	return out
}

func motionServerRPYMatrixForTest(rpy [3]float64) motionServerMat3ForTest {
	roll, pitch, yaw := rpy[0], rpy[1], rpy[2]
	cr, sr := math.Cos(roll), math.Sin(roll)
	cp, sp := math.Cos(pitch), math.Sin(pitch)
	cy, sy := math.Cos(yaw), math.Sin(yaw)
	return motionServerMat3ForTest{
		{cy * cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
		{sy * cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
		{-sp, cp * sr, cp * cr},
	}
}

func motionServerAxisAngleTransformForTest(axis [3]float64, angle float64) motionServerMat4ForTest {
	length := math.Sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2])
	if length <= 1e-12 {
		return motionServerIdentity4ForTest()
	}
	x, y, z := axis[0]/length, axis[1]/length, axis[2]/length
	c, s := math.Cos(angle), math.Sin(angle)
	oneMinusC := 1 - c
	out := motionServerIdentity4ForTest()
	out[0][0] = c + x*x*oneMinusC
	out[0][1] = x*y*oneMinusC - z*s
	out[0][2] = x*z*oneMinusC + y*s
	out[1][0] = y*x*oneMinusC + z*s
	out[1][1] = c + y*y*oneMinusC
	out[1][2] = y*z*oneMinusC - x*s
	out[2][0] = z*x*oneMinusC - y*s
	out[2][1] = z*y*oneMinusC + x*s
	out[2][2] = c + z*z*oneMinusC
	return out
}

func motionServerURDFFkJ1BaseForTest(urdf map[string]motionServerURDFJointForTest, q [6]float64) motionServerMat4ForTest {
	out := motionServerIdentity4ForTest()
	for axis, name := range []string{"J1", "J2", "J3", "J4", "J5", "J6"} {
		joint := urdf[name]
		xyz, rpy := joint.OriginXYZ, joint.OriginRPY
		if name == "J1" {
			xyz = [3]float64{}
			rpy = [3]float64{}
		}
		out = motionServerMultiply4ForTest(out, motionServerOriginTransformForTest(xyz, rpy))
		out = motionServerMultiply4ForTest(out, motionServerAxisAngleTransformForTest(joint.Axis, q[axis]))
	}
	endEffector := urdf["link_6_to_end_effector"]
	return motionServerMultiply4ForTest(out, motionServerOriginTransformForTest(endEffector.OriginXYZ, endEffector.OriginRPY))
}

func motionServerPositionErrorForTest(a, b motionServerMat4ForTest) float64 {
	dx := a[0][3] - b[0][3]
	dy := a[1][3] - b[1][3]
	dz := a[2][3] - b[2][3]
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

func motionServerRotationErrorForTest(a, b motionServerMat4ForTest) float64 {
	var trace float64
	for row := 0; row < 3; row++ {
		for col := 0; col < 3; col++ {
			trace += a[row][col] * b[row][col]
		}
	}
	cosAngle := math.Max(-1, math.Min(1, (trace-1)/2))
	return math.Acos(cosAngle)
}

func stringValue(value any) string {
	switch typed := value.(type) {
	case string:
		return typed
	default:
		return ""
	}
}

func boolValue(value any) bool {
	typed, _ := value.(bool)
	return typed
}

func anySlice(value any) []any {
	typed, _ := value.([]any)
	return typed
}

func containsAnyString(values []any, want string) bool {
	for _, value := range values {
		if strings.TrimSpace(stringValue(value)) == want {
			return true
		}
	}
	return false
}

func numericValue(value any) float64 {
	switch typed := value.(type) {
	case float64:
		return typed
	case string:
		parsed, _ := strconv.ParseFloat(typed, 64)
		return parsed
	default:
		return 0
	}
}
