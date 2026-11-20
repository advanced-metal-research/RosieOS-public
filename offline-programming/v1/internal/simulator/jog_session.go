package simulator

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"

	"net"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"time"
)

const (
	// Subject naming must satisfy the daemon's concrete-peer rules
	// (robot_v4_cartesian_cli.hpp:207-226): robot/v4/motion-server.<host>.…
	// and robot/v4/robot.<host>.… with a plain concrete host token.
	jogRobotCell           = "olpsim1"
	jogRobotCommandSubject = "robot/v4/robot.olpsim1.command"
	jogStatusSubject       = "robot/v4/motion-server.olpsim1.status"
	jogSenderID            = "steamdeck:steamdeck-sim"
	// The daemon clamps lease TTL to [500, 2000] ms
	// (robot_v4_cartesian_leader_authority.hpp:379-381); renew well inside it.
	jogLeaseTTLMS      = 2_000
	jogLeaseRenewEvery = 600 * time.Millisecond

	jogIntentPeriod = 20 * time.Millisecond // pendant cadence (50 Hz)
	// An HTTP intent keeps axes alive this long; the UI re-posts while held.
	jogIntentHoldWindow = 250 * time.Millisecond
	jogAxisCount        = 9
)

type JogIntent struct {
	// Mode: "base" (cartesian world), "tool" (cartesian tool), "joint".
	Mode string `json:"mode"`
	// Axes: normalized -1..1. Cartesian: [X Y Z RX RY RZ]. Joint: the
	// selected joint's velocity in element 0 (or its packet slot).
	Axes [6]float64 `json:"axes"`
	// SelectedJoint: joint index for joint mode (0-based, full axis set).
	SelectedJoint int `json:"selectedJoint"`
	// SpeedScale 0..1, forwarded to the daemon's authoritative limits.
	SpeedScale float64 `json:"speedScale"`
}

type JogState struct {
	RTCore         json.RawMessage `json:"rtCore,omitempty"`
	Running        bool            `json:"running"`
	JointsRad      []float64       `json:"jointsRad"`
	AxisNames      []string        `json:"axisNames"`
	ActiveIntent   bool            `json:"activeIntent"`
	LastCommandAge string          `json:"lastCommandAge"`
	DaemonAddress  string          `json:"daemonAddress"`
	Error          string          `json:"error,omitempty"`
}

type jogSession struct {
	mu                sync.Mutex
	nativeDescription json.RawMessage
	nativeObservation json.RawMessage

	conn       *net.UDPConn
	daemonAddr *net.UDPAddr
	daemon     *exec.Cmd
	daemonDone chan struct{}
	temp       string
	cancel     context.CancelFunc

	sequence      uint32
	intent        JogIntent
	intentAt      time.Time
	joints        [jogAxisCount]float64
	axisNames     []string
	lastCmdAt     time.Time
	runError      string
	stopped       bool
	nativeFailure string
	stderr        *boundedBuffer

	broker      *jogNATSBroker
	leaseID     string
	fenceEpoch  uint64
	campaignGen uint64
}

// startJogSession refuses obsolete selection before files, sockets, or children.
func startJogSession(repoRoot, binary string, extraEnv []string) (*jogSession, error) {
	if err := ValidateJogBackend(os.Getenv("OFFLINE_PROGRAMMING_JOG_BACKEND")); err != nil {
		return nil, err
	}
	return startNativeJogSession(repoRoot, binary, extraEnv)
}

// ValidateJogBackend refuses retired selections before serve or session startup
// can acquire resources or mask the selection error with an unrelated failure.
func ValidateJogBackend(backend string) error {
	if backend != "" && backend != "rt_core" {
		return &JogBackendError{Backend: backend}
	}
	return nil
}

// JogBackendError identifies backend refusal before resource acquisition.
type JogBackendError struct{ Backend string }

func (e *JogBackendError) Error() string {
	return "backend_retired: jog requires rt_core; received " + e.Backend
}

func (session *jogSession) acquireLeaderLease() error {
	session.campaignGen = uint64(time.Now().UnixNano())
	payload := fmt.Sprintf(
		`{"schema":"robot.v4.robot-command.v1","command":"leader_acquire",`+
			`"robot":%q,"command_id":"olp-jog-acquire-%d","sender_id":%q,`+
			`"controller_id":%q,"controller_boot_id":"olp-jog-%d",`+
			`"request_id":"olp-jog-request-%d","campaign_generation":%d,"ttl_ms":%d}`,
		jogRobotCell, session.campaignGen, jogSenderID,
		jogSenderID, session.campaignGen,
		session.campaignGen, session.campaignGen, jogLeaseTTLMS)
	// The daemon's NATS subscriber reconnect loop polls every ~2 s; give the
	// campaign room to land.
	deadline := time.Now().Add(15 * time.Second)
	for time.Now().Before(deadline) {
		session.broker.PublishLocal(jogRobotCommandSubject, "", []byte(payload))
		time.Sleep(80 * time.Millisecond)
		session.mu.Lock()
		granted := session.leaseID != "" && session.fenceEpoch != 0
		session.mu.Unlock()
		if granted {
			return nil
		}
	}
	status := session.StatusJSON()
	return fmt.Errorf(
		"jog leader lease was not granted; leader_id=%s lease=%s result=%s request=%s kind=%s",
		jsonStringField(status, "leader_id"),
		jsonStringField(status, "leader_lease_id"),
		jsonStringField(status, "latest_leader_result"),
		jsonStringField(status, "latest_leader_request_id"),
		jsonStringField(status, "latest_command_kind"))
}

func (session *jogSession) leaseRenewLoop(ctx context.Context) {
	ticker := time.NewTicker(jogLeaseRenewEvery)
	defer ticker.Stop()
	renewal := 0
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}
		session.mu.Lock()
		leaseID := session.leaseID
		epoch := session.fenceEpoch
		campaign := session.campaignGen
		session.mu.Unlock()
		if leaseID == "" {
			continue
		}
		renewal++
		payload := fmt.Sprintf(
			`{"schema":"robot.v4.robot-command.v1","command":"leader_renew",`+
				`"robot":%q,"command_id":"olp-jog-renew-%d-%d","sender_id":%q,`+
				`"controller_id":%q,"controller_boot_id":"olp-jog-%d",`+
				`"request_id":"olp-jog-renew-%d","campaign_generation":%d,`+
				`"ttl_ms":%d,"lease_id":%q,"fence_epoch":%d}`,
			jogRobotCell, campaign, renewal, jogSenderID,
			jogSenderID, campaign, renewal, campaign,
			jogLeaseTTLMS, leaseID, epoch)
		session.broker.PublishLocal(jogRobotCommandSubject, "", []byte(payload))
	}
}

func (session *jogSession) readLeaseFromStatus() (string, uint64) {
	status := session.StatusJSON()
	leaseID := jsonStringField(status, "leader_lease_id")
	if leaseID == "" || leaseID == "none" {
		return "", 0
	}
	epoch := jsonUintField(status, "leader_fence_epoch")
	return leaseID, epoch
}

func jsonStringField(payload, field string) string {
	marker := fmt.Sprintf("%q:", field)
	index := strings.Index(payload, marker)
	if index < 0 {
		return ""
	}
	rest := payload[index+len(marker):]
	if len(rest) == 0 || rest[0] != '"' {
		return ""
	}
	end := strings.IndexByte(rest[1:], '"')
	if end < 0 {
		return ""
	}
	return rest[1 : 1+end]
}

func jsonUintField(payload, field string) uint64 {
	marker := fmt.Sprintf("%q:", field)
	index := strings.Index(payload, marker)
	if index < 0 {
		return 0
	}
	rest := payload[index+len(marker):]
	var value uint64
	for _, character := range rest {
		if character < '0' || character > '9' {
			break
		}
		value = value*10 + uint64(character-'0')
	}
	return value
}

func jsonRawField(payload, field string) string {
	marker := fmt.Sprintf("%q:", field)
	index := strings.Index(payload, marker)
	if index < 0 {
		return "?"
	}
	rest := payload[index+len(marker):]
	end := strings.IndexAny(rest, ",}")
	if end < 0 {
		return "?"
	}
	return rest[:end]
}

func trimForDiagnostics(text string, limit int) string {
	text = strings.TrimSpace(text)
	if len(text) > limit {
		return text[:limit] + "…"
	}
	return text
}

func waitForJogDaemonReady(
	readyFile string,
	daemonDone <-chan struct{},
	stderr *boundedBuffer,
	timeout time.Duration,
) (string, error) {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		select {
		case <-daemonDone:
			return "", fmt.Errorf(
				"jog daemon exited before ready: %s",
				strings.TrimSpace(stderr.String()))
		default:
		}
		raw, err := os.ReadFile(readyFile)
		if err == nil {
			address := strings.TrimSpace(string(raw))
			if address != "" {
				return address, nil
			}
		}
		time.Sleep(20 * time.Millisecond)
	}
	return "", errors.New("jog daemon did not report a ready UDP address in time")
}

func (session *jogSession) SetIntent(intent JogIntent) error {
	session.mu.Lock()
	failure := session.nativeFailure
	session.mu.Unlock()
	if failure != "" {
		return errors.New(failure)
	}
	if intent.Mode == "joint" {
		return errors.New("rt_core cartesiand supports base/tool Cartesian jog; joint jog is unavailable")
	}
	switch intent.Mode {
	case "base", "tool", "joint":
	default:
		return fmt.Errorf("jog mode %q must be base, tool, or joint", intent.Mode)
	}
	for _, value := range intent.Axes {
		if value < -1 || value > 1 || value != value {
			return errors.New("jog axes must be normalized to [-1, 1]")
		}
	}
	if intent.SelectedJoint < 0 || intent.SelectedJoint >= jogAxisCount {
		return fmt.Errorf("selected joint %d out of range", intent.SelectedJoint)
	}
	if intent.SpeedScale <= 0 || intent.SpeedScale > 1 {
		intent.SpeedScale = 1
	}
	session.mu.Lock()
	session.intent = intent
	session.intentAt = time.Now()
	session.mu.Unlock()
	return nil
}

func encodeJogIntentPacket(
	sequence uint32,
	intent JogIntent,
	forceDeadman bool,
	fenceEpoch uint64,
	leaseID string,
) []byte {
	switch intent.Mode {
	case "joint":
		axes := jointJogAxes(intent.SelectedJoint, intent.Axes[0])
		if forceDeadman {
			axes = [6]float64{}
		}
		return encodeJoystickIntentFenced(
			sequence, JoystickFrameJoint, axes,
			jointJogToolTCP(intent.SelectedJoint), intent.SpeedScale, forceDeadman,
			fenceEpoch, leaseID)
	case "tool":
		return encodeJoystickIntentFenced(
			sequence, JoystickFrameTool, intent.Axes, [6]float64{},
			intent.SpeedScale, forceDeadman, fenceEpoch, leaseID)
	default:
		return encodeJoystickIntentFenced(
			sequence, JoystickFrameBase, intent.Axes, [6]float64{},
			intent.SpeedScale, forceDeadman, fenceEpoch, leaseID)
	}
}

func (session *jogSession) StopMotion() {
	session.mu.Lock()
	session.intentAt = time.Now().Add(-jogIntentHoldWindow)
	session.mu.Unlock()
}

func (session *jogSession) State() JogState {
	session.mu.Lock()
	defer session.mu.Unlock()
	joints := make([]float64, jogAxisCount)
	copy(joints, session.joints[:])
	age := "never"
	if !session.lastCmdAt.IsZero() {
		age = time.Since(session.lastCmdAt).Round(time.Millisecond).String()
	}
	daemonAddress := ""
	if session.daemonAddr != nil {
		daemonAddress = session.daemonAddr.String()
	}
	diagnostic := session.runError
	if session.nativeFailure != "" {
		diagnostic = session.nativeFailure
	}
	return JogState{
		RTCore:         append(json.RawMessage(nil), session.nativeObservation...),
		Running:        !session.stopped,
		JointsRad:      joints,
		AxisNames:      append([]string(nil), session.axisNames...),
		ActiveIntent:   time.Since(session.intentAt) <= jogIntentHoldWindow,
		LastCommandAge: age,
		DaemonAddress:  daemonAddress,
		Error:          diagnostic,
	}
}

func (session *jogSession) StatusJSON() string {
	raw, err := os.ReadFile(filepath.Join(session.temp, "status.json"))
	if err != nil {
		return ""
	}
	return string(raw)
}

func (session *jogSession) Stop() {
	session.mu.Lock()
	if session.stopped {
		session.mu.Unlock()
		return
	}
	session.stopped = true
	session.mu.Unlock()
	if session.daemon != nil && session.daemon.Process != nil {
		stopNativeJogSession(session)
	}

	session.cancel()
	if session.broker != nil {
		session.broker.Close()
	}
	if session.conn != nil {
		session.conn.Close()
	}
	if session.daemon != nil && session.daemon.Process != nil {
		_ = session.daemon.Process.Kill()
	}
	select {
	case <-session.daemonDone:
	case <-time.After(2 * time.Second):
	}
	if session.temp != "" {
		os.RemoveAll(session.temp)
	}
}
