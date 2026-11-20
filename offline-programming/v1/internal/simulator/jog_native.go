//go:build linux

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
	"strconv"
	"syscall"
	"time"

	"rosieos/rt-core/sdk/control"
)

// Unverified local startup and observation bounds; only receipts establish readiness.
const nativeJogStartup = 15 * time.Second

// Unverified observation cadence; positive checks require native state changes.
const nativeJogPoll = 20 * time.Millisecond

// nativeJogArguments requires a complete local binding, preventing implicit access
// to the machine's installed control socket when a development stack is absent.
func nativeJogArguments(getenv func(string) string) ([]string, error) {
	args := []string{"--backend", "rt_core", "--trusted-lan-leader-authority", "--rt-axis-ids", "J1,J2,J3,J4,J5,J6"}
	for _, entry := range [][2]string{
		{"--rt-control-socket", "ROSIE_RT_CONTROL_SOCKET"}, {"--rt-jog-socket", "ROSIE_RT_JOG_SOCKET"},
		{"--rt-pair-id", "ROSIE_RT_PAIR_ID"}, {"--rt-pair-revision", "ROSIE_RT_PAIR_REVISION"},
		{"--rt-configuration-sha256", "ROSIE_RT_CONFIGURATION_SHA256"},
		{"--rt-urdf", "ROSIE_RT_URDF"}, {"--rt-urdf-sha256", "ROSIE_RT_URDF_SHA256"},
	} {
		value := getenv(entry[1])
		if value == "" {
			return nil, fmt.Errorf("local robot-v4-cartesiand rt_core jog requires %s; use start-offline-programming.sh", entry[1])
		}
		args = append(args, entry[0], value)
	}
	return args, nil
}

// startNativeJogSession retains the OLP pendant intent and leader lanes, while
// cartesiand alone resolves Cartesian input and owns the public native grant.
func startNativeJogSession(repoRoot, binary string, extraEnv []string) (*jogSession, error) {
	args, err := nativeJogArguments(os.Getenv)
	if err != nil {
		return nil, err
	}
	client, err := control.Dial(control.UnixPath(os.Getenv("ROSIE_RT_CONTROL_SOCKET")))
	if err != nil {
		return nil, err
	}
	defer client.Close()
	ctx, cancel := context.WithTimeout(context.Background(), nativeJogStartup)
	defer cancel()
	description, err := client.Describe(ctx)
	if err != nil {
		return nil, err
	}
	if description.Backend != "simulation" || description.ConfigurationSHA256 != os.Getenv("ROSIE_RT_CONFIGURATION_SHA256") {
		return nil, errors.New("local jog requires the bound simulation adapter")
	}
	if err := commissionLocalJog(ctx, client, description); err != nil {
		return nil, err
	}
	conn, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.ParseIP("127.0.0.1")})
	if err != nil {
		return nil, err
	}
	broker, err := startJogNATSBroker()
	if err != nil {
		conn.Close()
		return nil, err
	}
	temp, err := os.MkdirTemp("", "j-")
	if err != nil {
		conn.Close()
		broker.Close()
		return nil, err
	}
	runCtx, stop := context.WithCancel(context.Background())
	ready := filepath.Join(temp, "ready")
	args = append(args, "--udp-listen", "127.0.0.1:0", "--udp-ready-file", ready,
		"--status-output", filepath.Join(temp, "status.json"), "--nats-url", broker.URL(),
		"--robot-cell", jogRobotCell, "--robot-command-subject", jogRobotCommandSubject)
	command := exec.CommandContext(runCtx, binary, args...)
	command.Dir = repoRoot
	command.Env = append(os.Environ(), extraEnv...)
	stderr := &boundedBuffer{maximum: maxPlannerStderrBytes}
	command.Stdout, command.Stderr = stderr, stderr
	session := &jogSession{conn: conn, broker: broker, temp: temp, cancel: stop,
		daemon: command, daemonDone: make(chan struct{}), stderr: stderr,
		axisNames: []string{"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}}
	session.nativeDescription, _ = json.Marshal(description)
	if err := command.Start(); err != nil {
		close(session.daemonDone)
		session.Stop()
		return nil, err
	}
	go func() { _ = command.Wait(); close(session.daemonDone) }()
	address, err := waitForJogDaemonReady(ready, session.daemonDone, stderr, nativeJogStartup)
	if err != nil {
		session.Stop()
		return nil, err
	}
	session.daemonAddr, err = net.ResolveUDPAddr("udp", address)
	if err != nil {
		session.Stop()
		return nil, err
	}
	go session.nativeStatusLoop(runCtx)
	if err := session.acquireLeaderLease(); err != nil {
		session.Stop()
		return nil, err
	}
	go session.leaseRenewLoop(runCtx)
	go session.nativeSenderLoop(runCtx)
	return session, nil
}

// commissionLocalJog establishes required coordinates on the verified simulator
// before cartesiand can Enable. Home never borrows an active consumer's grant.
func commissionLocalJog(ctx context.Context, client *control.Client, description control.Description) error {
	var mask uint32
	for _, axis := range description.Axes {
		if axis.RequireHome {
			mask |= 1 << axis.Index
		}
	}
	status, err := client.Status(ctx)
	if err != nil || status.Core.HomeValidMask&mask == mask {
		return err
	}
	revision, err := strconv.ParseUint(os.Getenv("ROSIE_RT_PAIR_REVISION"), 10, 64)
	if err != nil {
		return err
	}
	_, err = client.Acquire(ctx, "offline-programming:local-jog-home", control.Binding{
		PairID: os.Getenv("ROSIE_RT_PAIR_ID"), Revision: revision, ConfigurationSHA256: description.ConfigurationSHA256, MachineSHA256: description.MachineSHA256})
	if err != nil {
		return err
	}
	// Unverified renewal cadence, bounded by the adapter's existing lease.
	renewals, err := client.StartRenewal(ctx, 100*time.Millisecond)
	if err == nil {
		_, err = client.Command(ctx, &control.Request{Operation: "home", AxisMask: mask})
	}
	if err == nil {
		ticker := time.NewTicker(nativeJogPoll)
		defer ticker.Stop()
		for {
			status, err = client.Status(ctx)
			if err != nil || status.Core.HomeValidMask&mask == mask {
				break
			}
			select {
			case <-ctx.Done():
				err = ctx.Err()
			case renewal, ok := <-renewals:
				if !ok {
					err = errors.New("local Home renewal closed")
				} else if renewal.Err != nil {
					err = renewal.Err
				}
			case <-ticker.C:
			}
			if err != nil {
				break
			}
		}
	}
	client.StopRenewal()
	cleanup, cancel := context.WithTimeout(context.Background(), nativeJogStartup)
	defer cancel()
	if _, stopErr := client.Stop(cleanup); stopErr != nil {
		return fmt.Errorf("local Home stop: %w", stopErr)
	}
	if _, releaseErr := client.Release(cleanup); releaseErr != nil {
		return fmt.Errorf("local Home release: %w", releaseErr)
	}
	return err
}

// nativeSenderLoop sends only held Cartesian intent. Native zero/deadman release
// is a Stop boundary, so legacy idle telemetry must never cancel commissioning.
func (session *jogSession) nativeSenderLoop(ctx context.Context) {
	ticker := time.NewTicker(jogIntentPeriod)
	defer ticker.Stop()
	active := false
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}
		session.mu.Lock()
		intent, at := session.intent, session.intentAt
		session.mu.Unlock()
		held := time.Since(at) <= jogIntentHoldWindow && intent.Axes != [6]float64{}
		if held && !active {
			if err := session.nativeCold("arm"); err != nil {
				session.mu.Lock()
				session.nativeFailure = err.Error()
				session.intentAt = time.Time{}
				session.mu.Unlock()
				return
			}
			active = true
		}
		if !held && !active {
			continue
		}
		session.mu.Lock()
		// Re-read after Arm so a release or expired browser intent cannot replay
		// the input captured before the lifecycle operation.
		intent = session.intent
		held = time.Since(session.intentAt) <= jogIntentHoldWindow && intent.Axes != [6]float64{}
		session.sequence++
		if !held {
			intent.Axes = [6]float64{}
		}
		packet := encodeJogIntentPacket(session.sequence, intent, true, session.fenceEpoch, session.leaseID)
		address := session.daemonAddr
		session.mu.Unlock()
		_, _ = session.conn.WriteToUDP(packet, address)
		active = held
	}
}

// nativeStatusLoop consumes actual native positions and the motion owner's
// leader fence; simulated UDP integration cannot masquerade as public feedback.
func (session *jogSession) nativeStatusLoop(ctx context.Context) {
	ticker := time.NewTicker(nativeJogPoll)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}
		var status struct {
			Lease   string `json:"leader_lease_id"`
			Epoch   uint64 `json:"leader_fence_epoch"`
			Refusal string `json:"refusal"`
			Native  struct {
				Core struct {
					Axes []struct {
						Position *float64 `json:"logical_position"`
					} `json:"axes"`
				} `json:"core"`
			} `json:"rt_core_status"`
		}
		rawStatus := []byte(session.StatusJSON())
		if json.Unmarshal(rawStatus, &status) != nil {
			session.mu.Lock()
			session.nativeObservation = nil
			session.mu.Unlock()
			continue
		}
		var public struct {
			Status json.RawMessage `json:"rt_core_status"`
		}
		_ = json.Unmarshal(rawStatus, &public)
		session.mu.Lock()
		session.nativeObservation, _ = json.Marshal(map[string]any{"status": public.Status, "description": session.nativeDescription, "reason": status.Refusal})
		session.leaseID, session.fenceEpoch = status.Lease, status.Epoch
		session.runError = status.Refusal
		for i, axis := range status.Native.Core.Axes {
			if i < jogAxisCount && axis.Position != nil {
				session.joints[i] = *axis.Position
				session.lastCmdAt = time.Now()
			}
		}
		session.mu.Unlock()
	}
}

// nativeCold requires an applied receipt and native readiness for Arm, or a
// terminal receipt for Home, preventing queue acceptance from authorizing input.
func (session *jogSession) nativeCold(command string) error {
	session.mu.Lock()
	id := "olp-native-" + command + "-" + strconv.FormatInt(time.Now().UnixNano(), 10)
	payload, err := json.Marshal(map[string]any{"schema": "robot.v4.robot-command.v1", "robot": jogRobotCell,
		"command": command, "command_id": id, "sender_id": jogSenderID,
		"controller_boot_id": fmt.Sprintf("olp-jog-%d", session.campaignGen),
		"leader_lease_id":    session.leaseID, "leader_fence_epoch": session.fenceEpoch, "armed": command == "arm"})
	session.mu.Unlock()
	if err != nil {
		return err
	}
	reply := "_INBOX." + id
	messages := session.broker.SubscribeLocal(reply)
	session.broker.PublishLocal(jogRobotCommandSubject, reply, payload)
	timer := time.NewTimer(nativeJogStartup)
	defer timer.Stop()
	poll := time.NewTicker(nativeJogPoll)
	defer poll.Stop()
	applied := false
	for {
		select {
		case <-poll.C:
			var status struct {
				ID       string `json:"latest_processed_cold_command_id"`
				Accepted bool   `json:"latest_processed_cold_accepted"`
				Result   string `json:"latest_processed_cold_result"`
				Armed    bool   `json:"servos_armed_requested"`
				Native   struct {
					Core struct {
						Axes []struct {
							Readiness string `json:"readiness"`
						} `json:"axes"`
					} `json:"core"`
				} `json:"rt_core_status"`
			}
			if json.Unmarshal([]byte(session.StatusJSON()), &status) != nil {
				continue
			}
			if status.ID == id {
				if !status.Accepted {
					return fmt.Errorf("native %s refused: %s", command, status.Result)
				}
				applied = status.Result == "applied"
			}
			if command == "arm" && applied && status.Armed && len(status.Native.Core.Axes) >= 6 {
				ready := true
				for _, axis := range status.Native.Core.Axes[:6] {
					ready = ready && axis.Readiness == "ready"
				}
				if ready {
					return nil
				}
			}
		case <-timer.C:
			return fmt.Errorf("native %s terminal receipt unavailable: %s", command, session.StatusJSON())
		case <-session.daemonDone:
			return fmt.Errorf("cartesiand exited: %s", session.stderr.String())
		case message := <-messages:
			var result struct {
				State string `json:"state"`
				Error string `json:"error"`
			}
			if json.Unmarshal(message.Payload, &result) != nil {
				continue
			}
			if result.State == "completed" {
				return nil
			}
			if result.State == "failed" || result.State == "rejected" || result.State == "cancelled" {
				return fmt.Errorf("native %s refused: %s", command, message.Payload)
			}
		}
	}
}

func stopNativeJogSession(session *jogSession) {
	_ = session.daemon.Process.Signal(syscall.SIGTERM)
	select {
	case <-session.daemonDone:
	case <-time.After(nativeJogStartup):
	}
}
