package motion_server

import (
	"bytes"

	"encoding/binary"
	"encoding/json"

	"net"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

func compileAndRunLeaderHarness(t *testing.T, name string) {
	t.Helper()
	repoRoot := findRepoRoot(t)
	source := filepath.Join(
		repoRoot, "steamdeck", "real", "v4", "test", "motion-server", "testdata",
		name+".cpp",
	)
	executable := filepath.Join(t.TempDir(), name)
	cxx := strings.TrimSpace(os.Getenv("CXX"))
	if cxx == "" {
		cxx = "c++"
	}
	compile := exec.Command(
		cxx,
		"-std=c++17",
		"-O2",
		"-Wall",
		"-Wextra",
		"-Wpedantic",
		"-o", executable,
		source,
		"-pthread",
	)
	compile.Dir = repoRoot
	if output, err := compile.CombinedOutput(); err != nil {
		t.Fatalf("compile %s: %v\n%s", name, err, output)
	}
	run := exec.Command(executable)
	run.Dir = repoRoot
	if output, err := run.CombinedOutput(); err != nil {
		t.Fatalf("run %s: %v\n%s", name, err, output)
	}
}

func TestRobotV4LeaderAuthorityStateMachine(t *testing.T) {
	compileAndRunLeaderHarness(t, "leader_authority_harness")
}

func TestRobotV4LeaderAuthorityWireContract(t *testing.T) {
	compileAndRunLeaderHarness(t, "leader_wire_harness")
}

func TestRobotV4MotionServerQuiescenceBarrierWireContract(t *testing.T) {
	compileAndRunLeaderHarness(t, "motion_quiescence_harness")
}

func TestRobotV4MotionServerRelaysBridgeRunCredentialToPlanWeld(t *testing.T) {

	// Weld relay execution belonged to the retired output owner.
	binary := ensureMotionServerCartesianTestBinary(t, findRepoRoot(t))
	output, err := exec.Command(binary, "--nats-io-subject", "robot/v4/robot.fixture.command.io").CombinedOutput()
	if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 || !strings.Contains(string(output), "reason=backend_retired") {
		t.Fatalf("retired weld relay accepted: %v %s", err, output)
	}

}

func TestRobotV4MotionServerResolvesExactPairRevisionURDF(t *testing.T) {

	raw, err := os.ReadFile(filepath.Join(findRepoRoot(t), "motion-server/v1/src/rt_core_cartesian_runtime.hpp"))
	if err != nil {
		t.Fatal(err)
	}
	for _, required := range []string{"load_rt_core_kinematics(options.rt_core)", "sha256_lower_hex(urdf) != config.urdf_sha256", "rt_core_urdf_binding_invalid", "kinematics.urdf_path = config.urdf", "kinematics.urdf_sha256 = config.urdf_sha256"} {
		if !strings.Contains(string(raw), required) {
			t.Fatalf("explicit native URDF binding missing %q", required)
		}
	}

}

func TestRobotV4LeaderAuthorityLiveRequiresExplicitTrustedLANOptIn(t *testing.T) {

	raw, err := os.ReadFile(filepath.Join(findRepoRoot(t), "motion-server/v1/src/rt_core_cartesian_runtime.hpp"))
	if err != nil {
		t.Fatal(err)
	}
	source := string(raw)
	guard := strings.Index(source, "!options.trusted_lan_leader_authority")
	resources := strings.Index(source, "cartesian_rt::LocalClient client(options.rt_core)")
	if guard < 0 || resources < 0 || guard >= resources || !strings.Contains(source[guard:resources], "return 2;") {
		t.Fatal("native authority must refuse before client or listener acquisition")
	}

}

func TestRobotV4IdlePairRevokeDoesNotDisarmBridgeOwnedRun(t *testing.T) {

	// Legacy pair identity cannot select an output owner, even while idle.
	binary := ensureMotionServerCartesianTestBinary(t, findRepoRoot(t))
	output, err := exec.Command(binary, "--rtcore-target", "127.0.0.1:1").CombinedOutput()
	if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 || !strings.Contains(string(output), "reason=backend_retired") {
		t.Fatalf("retired pair output accepted: %v %s", err, output)
	}

}

type leaderV5StatusForTest struct {
	leaderID         string
	leaderBootID     string
	leaderLeaseID    string
	fenceEpoch       uint64
	expiresInMS      uint32
	fresh            bool
	pending          bool
	result           string
	requestID        string
	latestGeneration uint64
	activeGeneration uint64
	motionOwnerSeq   uint64
}

func leaderV5FixedASCII(packet []byte, offset, width int) string {
	field := packet[offset : offset+width]
	if end := bytes.IndexByte(field, 0); end >= 0 {
		field = field[:end]
	}
	return string(field)
}

func readLeaderV5Status(
	t *testing.T,
	client *net.UDPConn,
	timeout time.Duration,
	match func(leaderV5StatusForTest) bool,
) leaderV5StatusForTest {
	t.Helper()
	deadline := time.Now().Add(timeout)
	buffer := make([]byte, 2048)
	for time.Now().Before(deadline) {
		_ = client.SetReadDeadline(time.Now().Add(50 * time.Millisecond))
		n, _, err := client.ReadFromUDP(buffer)
		if err != nil || n != 928 {
			continue
		}
		packet := buffer[:n]
		if binary.BigEndian.Uint16(packet[0:2]) != 0x4f57 ||
			packet[2] != 5 {
			continue
		}
		status := leaderV5StatusForTest{
			leaderID:         leaderV5FixedASCII(packet, 592, 64),
			leaderBootID:     leaderV5FixedASCII(packet, 656, 64),
			leaderLeaseID:    leaderV5FixedASCII(packet, 720, 64),
			fenceEpoch:       binary.BigEndian.Uint64(packet[584:592]),
			expiresInMS:      binary.BigEndian.Uint32(packet[580:584]),
			fresh:            packet[576]&1 != 0,
			pending:          packet[576]&2 != 0,
			result:           leaderV5FixedASCII(packet, 784, 64),
			requestID:        leaderV5FixedASCII(packet, 848, 64),
			latestGeneration: binary.BigEndian.Uint64(packet[912:920]),
			activeGeneration: binary.BigEndian.Uint64(packet[920:928]),
			motionOwnerSeq:   binary.BigEndian.Uint64(packet[136:144]),
		}
		if match(status) {
			return status
		}
	}
	t.Fatalf("timed out waiting for matching MotionOwner v5 leader status")
	return leaderV5StatusForTest{}
}

func encodeLeaderJoystickV2ForTest(
	sequence uint32,
	frame byte,
	axes [6]float64,
	toolTCP [6]float64,
	toolID string,
	epoch uint64,
	leaseID string,
) []byte {
	packet := encodeSteamDeckJoystickIntentWithToolTCPForMotionServerTest(
		sequence, frame, axes, toolTCP, toolID,
	)
	packet[2] = 2
	tail := make([]byte, 9+len(leaseID))
	binary.BigEndian.PutUint64(tail[0:8], epoch)
	tail[8] = byte(len(leaseID))
	copy(tail[9:], leaseID)
	return append(packet, tail...)
}

func leaderCommandPayloadForTest(
	t *testing.T,
	robot string,
	command string,
	commandID string,
	senderID string,
	bootID string,
	requestID string,
	leaseID string,
	epoch uint64,
) string {
	t.Helper()
	return leaderCommandPayloadForGenerationTest(
		t, robot, command, commandID, senderID, bootID, requestID,
		leaseID, epoch, 1,
	)
}

func leaderCommandPayloadForGenerationTest(
	t *testing.T,
	robot string,
	command string,
	commandID string,
	senderID string,
	bootID string,
	requestID string,
	leaseID string,
	epoch uint64,
	campaignGeneration uint64,
) string {
	t.Helper()
	payload := map[string]any{
		"schema":              "robot.v4.robot-command.v1",
		"command":             command,
		"robot":               robot,
		"command_id":          commandID,
		"sender_id":           senderID,
		"controller_boot_id":  bootID,
		"request_id":          requestID,
		"campaign_generation": campaignGeneration,
	}
	switch command {
	case "leader_acquire":
		payload["ttl_ms"] = 1500
	case "leader_renew":
		payload["ttl_ms"] = 1500
		payload["lease_id"] = leaseID
		payload["fence_epoch"] = epoch
	case "leader_release":
		payload["lease_id"] = leaseID
		payload["fence_epoch"] = epoch
	}
	raw, err := json.Marshal(payload)
	if err != nil {
		t.Fatalf("marshal %s: %v", command, err)
	}
	return string(raw)
}

type motionServerLeaderLeaseForTest struct {
	robot      string
	toolID     string
	senderID   string
	bootID     string
	leaseID    string
	epoch      uint64
	generation uint64
}

func withLeaderCredentialV2ForTest(
	packet []byte,
	epoch uint64,
	leaseID string,
) []byte {
	packet[2] = 2
	tail := make([]byte, 9+len(leaseID))
	binary.BigEndian.PutUint64(tail[0:8], epoch)
	tail[8] = byte(len(leaseID))
	copy(tail[9:], leaseID)
	return append(packet, tail...)
}

func encodeLeaderPathScrubV2ForTest(
	direction int8,
	sequence uint64,
	revision uint64,
	cursorMS uint32,
	programID string,
	lease motionServerLeaderLeaseForTest,
) []byte {
	return withLeaderCredentialV2ForTest(
		pathScrubUDPPacketForTest(
			direction, sequence, revision, cursorMS, programID,
			lease.senderID,
		),
		lease.epoch,
		lease.leaseID,
	)
}

func acquireMotionServerLeaderForTest(
	t *testing.T,
	server *motionOwnerV4CommandServer,
	client *net.UDPConn,
	target *net.UDPAddr,
	robot string,
	toolID string,
	bootID string,
) motionServerLeaderLeaseForTest {
	t.Helper()
	senderID := "steamdeck:" + toolID
	if _, err := client.WriteToUDP(
		encodeSteamDeckJoystickIntentWithToolTCPForMotionServerTest(
			0,
			7,
			[6]float64{},
			[6]float64{},
			toolID,
		),
		target,
	); err != nil {
		t.Fatalf("establish leader fixture UDP peer: %v", err)
	}
	_ = server.waitForStatus(t, func(raw string) bool {
		return strings.Contains(raw, `"command":"io"`) &&
			strings.Contains(raw, `"on":false`) &&
			strings.Contains(raw, `"sender_id":"`+senderID+`"`)
	})
	select {
	case <-server.subscribed:
	case <-time.After(2 * time.Second):
		t.Fatalf("motion-server did not subscribe for leader fixture")
	}

	requestID := "acquire-" + toolID
	server.commands <- leaderCommandPayloadForTest(
		t, robot, "leader_acquire", requestID, senderID,
		bootID, requestID, "", 0,
	)
	status := readLeaderV5Status(
		t, client, 3*time.Second,
		func(status leaderV5StatusForTest) bool {
			return status.fresh &&
				status.leaderID == senderID &&
				status.leaderBootID == bootID &&
				status.requestID == requestID &&
				status.latestGeneration == 1
		},
	)
	if status.leaderLeaseID == "" || status.fenceEpoch == 0 ||
		status.expiresInMS == 0 || status.pending {
		t.Fatalf("invalid leader fixture grant: %+v", status)
	}
	lease := motionServerLeaderLeaseForTest{
		robot:      robot,
		toolID:     toolID,
		senderID:   senderID,
		bootID:     bootID,
		leaseID:    status.leaderLeaseID,
		epoch:      status.fenceEpoch,
		generation: status.latestGeneration,
	}
	return lease
}

func leaderCredentialFieldsForTest(
	lease motionServerLeaderLeaseForTest,
) map[string]any {
	return map[string]any{
		"controller_boot_id":  lease.bootID,
		"leader_lease_id":     lease.leaseID,
		"leader_fence_epoch":  lease.epoch,
		"campaign_generation": lease.generation,
	}
}

func robotV4EnvironmentWithoutForTest(prefix string) []string {
	environment := os.Environ()
	filtered := make([]string, 0, len(environment))
	for _, entry := range environment {
		if strings.HasPrefix(entry, prefix) {
			continue
		}
		filtered = append(filtered, entry)
	}
	return filtered
}
