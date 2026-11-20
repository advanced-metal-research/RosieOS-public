package motion_server

import (
	"bufio"
	"bytes"

	"encoding/binary"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net"

	"strconv"
	"strings"
	"sync"
	"testing"
	"time"
)

const (
	motionOwnerV4PacketBytes = 576
	motionOwnerV4Magic       = 0x4f57
	motionOwnerV4Version     = 4
	motionOwnerV5PacketBytes = 928
	motionOwnerV5Version     = 5
)

type motionOwnerV4RTCoreFeedbackFixture struct {
	conn        *net.UDPConn
	mu          sync.RWMutex
	joints      [9]float64
	pairBinding [240]byte
}

func startMotionOwnerV4RTCoreFeedbackFixture(
	t *testing.T,
	authorityPayload string,
) *motionOwnerV4RTCoreFeedbackFixture {
	t.Helper()
	var authority struct {
		Status struct {
			RuntimeModeEpoch    uint64 `json:"runtime_mode_epoch"`
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
	if err := json.Unmarshal([]byte(authorityPayload), &authority); err != nil {
		t.Fatalf("decode Motion Owner v4 RTCore authority fixture: %v", err)
	}
	homeProfileKey, err := strconv.ParseUint(authority.Status.HomeProfileKey, 16, 64)
	if err != nil || homeProfileKey == 0 {
		t.Fatalf("decode Motion Owner v4 home profile key %q: %v",
			authority.Status.HomeProfileKey, err)
	}
	if len(authority.Status.JetsonID) == 0 || len(authority.Status.JetsonID) > 31 ||
		len(authority.Status.RobotModelID) == 0 || len(authority.Status.RobotModelID) > 31 ||
		len(authority.Status.SnapshotSHA256) != 64 ||
		len(authority.Status.GeneratedURDFSHA256) != 64 {
		t.Fatalf("invalid Motion Owner v4 RTCore authority binding: %+v", authority.Status)
	}
	conn, err := net.ListenUDP(
		"udp",
		&net.UDPAddr{IP: net.ParseIP("127.0.0.1")},
	)
	if err != nil {
		t.Fatalf("start Motion Owner v4 RTCore feedback fixture: %v", err)
	}
	fixture := &motionOwnerV4RTCoreFeedbackFixture{conn: conn}
	binary.LittleEndian.PutUint64(fixture.pairBinding[0:8], authority.Status.Revision)
	binary.LittleEndian.PutUint64(fixture.pairBinding[8:16], authority.Status.ActivationEpoch)
	binary.LittleEndian.PutUint64(fixture.pairBinding[16:24], authority.Status.HomeEpoch)
	binary.LittleEndian.PutUint64(fixture.pairBinding[24:32], homeProfileKey)
	binary.LittleEndian.PutUint64(fixture.pairBinding[32:40], authority.Status.RuntimeModeEpoch)
	binary.LittleEndian.PutUint32(fixture.pairBinding[40:44], authority.Status.MappedAxisMask)
	copy(fixture.pairBinding[48:80], authority.Status.JetsonID)
	copy(fixture.pairBinding[80:112], authority.Status.RobotModelID)
	copy(fixture.pairBinding[112:176], authority.Status.SnapshotSHA256)
	copy(fixture.pairBinding[176:240], authority.Status.GeneratedURDFSHA256)
	t.Cleanup(func() { _ = conn.Close() })
	go func() {
		buffer := make([]byte, 2048)
		for {
			n, peer, err := conn.ReadFromUDP(buffer)
			if err != nil {
				return
			}
			if n == 0 {
				continue
			}
			if _, err := conn.WriteToUDP(fixture.feedbackPacket(), peer); err != nil {
				return
			}
		}
	}()
	return fixture
}

func (fixture *motionOwnerV4RTCoreFeedbackFixture) address() string {
	return fixture.conn.LocalAddr().String()
}

func (fixture *motionOwnerV4RTCoreFeedbackFixture) setJoints(
	t *testing.T,
	joints []float64,
) {
	t.Helper()
	if len(joints) != 9 {
		t.Fatalf("RTCore feedback fixture joint count = %d, want 9", len(joints))
	}
	fixture.mu.Lock()
	defer fixture.mu.Unlock()
	copy(fixture.joints[:], joints)
}

func (fixture *motionOwnerV4RTCoreFeedbackFixture) feedbackPacket() []byte {
	const countsPerRevolution = 131072.0
	gearRatio := [9]float64{
		100.0, 100.0, 100.0, 18.0, 100.0 / 11.0,
		10.0, 10.0, 10.0, 10.0,
	}
	sign := [9]float64{-1, 1, -1, -1, -1, -1, 1, 1, 1}
	fixture.mu.RLock()
	joints := fixture.joints
	fixture.mu.RUnlock()

	packet := make([]byte, 392)
	binary.LittleEndian.PutUint16(packet[0:2], 0x5346)
	packet[2] = 4
	packet[3] = 9
	binary.LittleEndian.PutUint32(packet[4:8], 0x1ff)
	binary.LittleEndian.PutUint64(packet[8:16], uint64(time.Now().UnixNano()))
	for axis, joint := range joints {
		counts := math.Round(
			joint * countsPerRevolution * gearRatio[axis] /
				(2 * math.Pi) * sign[axis],
		)
		binary.LittleEndian.PutUint32(
			packet[16+axis*4:20+axis*4],
			uint32(int32(counts)),
		)
		binary.LittleEndian.PutUint16(
			packet[114+axis*2:116+axis*2],
			0x27,
		)
		packet[132+axis] = 8
	}
	copy(packet[152:], fixture.pairBinding[:])
	return packet
}

type motionOwnerV4CommandServer struct {
	address                string
	listener               net.Listener
	subscribed             chan struct{}
	rtcoreStatusSubscribed chan struct{}
	commands               chan string
	statuses               chan string
	once                   sync.Once
	rtcoreStatusOnce       sync.Once
	writeMu                sync.Mutex
	rtcoreStatusFixtureMu  sync.RWMutex
	rtcoreStatusSubject    string
	rtcoreStatusFixture    string
}

func startMotionOwnerV4CommandServer(t *testing.T, subject string) *motionOwnerV4CommandServer {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("start Motion Owner v4 fake NATS server: %v", err)
	}
	server := &motionOwnerV4CommandServer{
		address:                listener.Addr().String(),
		listener:               listener,
		subscribed:             make(chan struct{}),
		rtcoreStatusSubscribed: make(chan struct{}),
		commands:               make(chan string, 8),
		statuses:               make(chan string, 64),
	}
	t.Cleanup(func() { _ = listener.Close() })
	go func() {
		for {
			conn, err := listener.Accept()
			if err != nil {
				return
			}
			go server.handleConnection(conn, subject)
		}
	}()
	return server
}

func (server *motionOwnerV4CommandServer) configureRTCoreStatusFixture(
	t *testing.T,
	subject string,
	payload string,
) {
	t.Helper()
	if subject == "" || payload == "" {
		t.Fatal("RTCore status fixture requires a subject and payload")
	}
	server.rtcoreStatusFixtureMu.Lock()
	server.rtcoreStatusSubject = subject
	server.rtcoreStatusFixture = payload
	server.rtcoreStatusFixtureMu.Unlock()
}

func (server *motionOwnerV4CommandServer) handleConnection(conn net.Conn, subject string) {
	defer conn.Close()
	_ = conn.SetDeadline(time.Now().Add(10 * time.Second))
	_, _ = io.WriteString(conn, "INFO {\"server_id\":\"motion-owner-v4-test\",\"max_payload\":1048576}\r\n")
	reader := bufio.NewReader(conn)
	done := make(chan struct{})
	defer close(done)
	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			return
		}
		line = strings.TrimRight(line, "\r\n")
		fields := strings.Fields(line)
		if len(fields) == 0 {
			continue
		}
		switch fields[0] {
		case "PING":
			server.writeMu.Lock()
			_, _ = io.WriteString(conn, "PONG\r\n")
			server.writeMu.Unlock()
		case "SUB":
			if len(fields) < 3 {
				continue
			}
			subscriptionSubject := fields[1]
			sid := fields[2]
			if subscriptionSubject == subject {
				server.once.Do(func() { close(server.subscribed) })
				go func() {
					for payload := range server.commands {
						server.writeMu.Lock()
						_, err := fmt.Fprintf(conn, "MSG %s %s %d\r\n%s\r\n",
							subject, sid, len(payload), payload)
						server.writeMu.Unlock()
						if err != nil {
							return
						}
					}
				}()
				continue
			}
			server.rtcoreStatusFixtureMu.RLock()
			rtcoreStatusSubject := server.rtcoreStatusSubject
			rtcoreStatusFixture := server.rtcoreStatusFixture
			server.rtcoreStatusFixtureMu.RUnlock()
			if subscriptionSubject != rtcoreStatusSubject ||
				rtcoreStatusFixture == "" {
				continue
			}
			sendRTCoreStatus := func() bool {
				server.writeMu.Lock()
				defer server.writeMu.Unlock()
				_, err := fmt.Fprintf(
					conn,
					"MSG %s %s %d\r\n%s\r\n",
					rtcoreStatusSubject,
					sid,
					len(rtcoreStatusFixture),
					rtcoreStatusFixture,
				)
				return err == nil
			}
			if !sendRTCoreStatus() {
				return
			}
			server.rtcoreStatusOnce.Do(
				func() { close(server.rtcoreStatusSubscribed) },
			)
			go func() {
				ticker := time.NewTicker(100 * time.Millisecond)
				defer ticker.Stop()
				for {
					select {
					case <-ticker.C:
						if !sendRTCoreStatus() {
							return
						}
					case <-done:
						return
					}
				}
			}()
		case "PUB":
			if len(fields) < 3 {
				return
			}
			size, err := strconv.Atoi(fields[len(fields)-1])
			if err != nil || size < 0 {
				return
			}
			payload := make([]byte, size+2)
			if _, err := io.ReadFull(reader, payload); err != nil {
				return
			}
			select {
			case server.statuses <- string(payload[:size]):
			default:
			}
		}
	}
}

func (server *motionOwnerV4CommandServer) waitForStatus(
	t *testing.T,
	match func(string) bool,
) string {
	return server.waitForStatusTimeout(t, 2*time.Second, match)
}

func (server *motionOwnerV4CommandServer) waitForStatusTimeout(
	t *testing.T,
	timeout time.Duration,
	match func(string) bool,
) string {
	t.Helper()
	deadline := time.After(timeout)
	var latest string
	for {
		select {
		case raw := <-server.statuses:
			latest = raw
			if match(raw) {
				return raw
			}
		case <-deadline:
			t.Fatalf("timed out waiting for Motion Server NATS status; latest=%s", latest)
			return ""
		}
	}
}

func motionOwnerV4FixedASCII(packet []byte, offset, width int) string {
	field := packet[offset : offset+width]
	if end := bytes.IndexByte(field, 0); end >= 0 {
		field = field[:end]
	}
	return string(field)
}

func readMotionOwnerV4PacketMatching(
	t *testing.T,
	client *net.UDPConn,
	timeout time.Duration,
	match func([]byte) bool,
) []byte {
	t.Helper()
	deadline := time.Now().Add(timeout)
	buffer := make([]byte, 2048)
	for time.Now().Before(deadline) {
		_ = client.SetReadDeadline(time.Now().Add(50 * time.Millisecond))
		n, _, err := client.ReadFromUDP(buffer)
		if err != nil || (n != motionOwnerV4PacketBytes && n != motionOwnerV5PacketBytes) {
			continue
		}
		packet := append([]byte(nil), buffer[:n]...)
		if binary.BigEndian.Uint16(packet[0:2]) != motionOwnerV4Magic ||
			(packet[2] != motionOwnerV4Version && packet[2] != motionOwnerV5Version) ||
			!match(packet) {
			continue
		}
		return packet
	}
	t.Fatalf("timed out waiting for matching Motion Owner v4 packet")
	return nil
}

func robotV4MotionOwnerCommandEnvelope(
	t *testing.T,
	robot string,
	command string,
	commandID string,
	senderID string,
	planRevision uint64,
	motionIntentSequence uint64,
	extra map[string]any,
) string {
	t.Helper()
	payload := map[string]any{
		"schema":            "robot.v4.robot-command.v1",
		"command":           command,
		"robot":             robot,
		"command_id":        commandID,
		"sender_id":         senderID,
		"plan_revision":     planRevision,
		"motion_intent_seq": motionIntentSequence,
	}
	for key, value := range extra {
		payload[key] = value
	}
	raw, err := json.Marshal(payload)
	if err != nil {
		t.Fatalf("marshal %s robot command: %v", command, err)
	}
	return string(raw)
}
