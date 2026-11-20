//go:build linux

package simulator

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"io"
	"net"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

func TestDevStackDenseProgram(t *testing.T) {
	if os.Getenv("DEV_STACK_SMOKE") != "1" {
		t.Skip("run motion-server/v1/tests/dev-stack-smoke.sh")
	}
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	broker, err := startJogNATSBroker()
	if err != nil {
		t.Fatal(err)
	}
	defer broker.Close()
	statuses := broker.SubscribeLocal("robot/v4/robot.dev-cell.status")
	t.Setenv("DEV_STACK_DENSE_NATS_URL", broker.URL())
	start := exec.Command("bash", filepath.Join(root, "dev-stack.sh"), "start", "daemon")
	if output, err := start.CombinedOutput(); err != nil {
		t.Fatalf("stack daemon start: %v\n%s", err, output)
	}
	t.Cleanup(func() {
		output, err := exec.Command("bash", filepath.Join(root, "dev-stack.sh"), "stop", "daemon").CombinedOutput()
		if err != nil {
			t.Errorf("stack daemon stop: %v\n%s", err, output)
		}
	})
	ready, err := os.ReadFile(filepath.Join(os.Getenv("ROSIE_DEV_STACK_DIR"), "daemon-ready.json"))
	if err != nil {
		t.Fatal(err)
	}
	var address struct {
		Host string
		Port int
	}
	if err := json.Unmarshal(ready, &address); err != nil || address.Port == 0 {
		t.Fatalf("daemon ready: %s %v", ready, err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), nativeJogStartup)
	defer cancel()
	ticker := time.NewTicker(nativeJogPoll)
	defer ticker.Stop()
	wait := func(predicate func() bool) {
		for !predicate() {
			select {
			case <-ticker.C:
			case <-ctx.Done():
				t.Fatal("dense observation deadline")
			}
		}
	}
	subject := "robot/v4/robot.dev-cell.command"
	wait(func() bool { broker.mu.Lock(); defer broker.mu.Unlock(); return len(broker.subscribers[subject]) > 0 })
	var sequence atomic.Uint64
	command := func(kind string, fields map[string]any) map[string]any {
		id := strconv.FormatUint(sequence.Add(1), 10)
		request := map[string]any{"schema": "robot.v4.robot-command.v1", "robot": "dev-cell", "sender_id": "offline-programming:smoke", "controller_boot_id": "smoke-boot", "command": kind, "command_id": id}
		for k, v := range fields {
			request[k] = v
		}
		body, err := json.Marshal(request)
		if err != nil {
			t.Fatal(err)
		}
		reply := "_INBOX.smoke." + id
		messages := broker.SubscribeLocal(reply)
		broker.PublishLocal(subject, reply, body)
		select {
		case message := <-messages:
			var result map[string]any
			if err := json.Unmarshal(message.Payload, &result); err != nil {
				t.Fatal(err)
			}
			if result["ok"] != true {
				var last []byte
				for len(statuses) > 0 {
					last = (<-statuses).Payload
				}
				t.Fatalf("%s: %s; status: %s", kind, message.Payload, last)
			}
			return result
		case <-ctx.Done():
			t.Fatalf("%s reply unavailable", kind)
			return nil
		}
	}
	lease := command("leader_acquire", map[string]any{"ttl_ms": 2000})
	stopRenew, renewed := make(chan struct{}), make(chan struct{})
	defer func() { close(stopRenew); <-renewed }()
	go func() {
		defer close(renewed)
		// Unverified source renewal cadence; completion still requires native status.
		tick := time.NewTicker(100 * time.Millisecond)
		defer tick.Stop()
		for {
			select {
			case <-stopRenew:
				return
			case <-tick.C:
			}
			body, _ := json.Marshal(map[string]any{"schema": "robot.v4.robot-command.v1", "robot": "dev-cell", "sender_id": "offline-programming:smoke", "controller_boot_id": "smoke-boot", "command": "leader_renew", "command_id": strconv.FormatUint(sequence.Add(1), 10), "lease_id": lease["lease_id"], "leader_fence_epoch": lease["leader_fence_epoch"], "ttl_ms": 2000})
			broker.PublishLocal(subject, "", body)
		}
	}()
	ingest := func(request map[string]any, blob []byte) map[string]any {
		raw, err := json.Marshal(request)
		if err != nil {
			t.Fatal(err)
		}
		payload := make([]byte, 4+len(raw)+len(blob))
		binary.BigEndian.PutUint32(payload, uint32(len(raw)))
		copy(payload[4:], raw)
		copy(payload[4+len(raw):], blob)
		conn, err := net.DialTimeout("tcp", net.JoinHostPort(address.Host, strconv.Itoa(address.Port)), nativeJogStartup)
		if err != nil {
			t.Fatal(err)
		}
		defer conn.Close()
		if err := conn.SetDeadline(time.Now().Add(nativeJogStartup)); err != nil {
			t.Fatal(err)
		}
		if err := binary.Write(conn, binary.BigEndian, uint64(len(payload))); err != nil {
			t.Fatal(err)
		}
		if _, err := conn.Write(payload); err != nil {
			t.Fatal(err)
		}
		var size uint64
		if err := binary.Read(conn, binary.BigEndian, &size); err != nil {
			t.Fatal(err)
		}
		if size < 4 || size > 1048576 {
			t.Fatalf("invalid ingest bytes: %d", size)
		}
		response := make([]byte, size)
		if _, err := io.ReadFull(conn, response); err != nil {
			t.Fatal(err)
		}
		var result map[string]any
		if err := json.Unmarshal(response[4:], &result); err != nil {
			t.Fatal(err)
		}
		if result["ok"] != true {
			t.Fatalf("ingest %v: %s", request, response[4:])
		}
		return result
	}
	blob, err := os.ReadFile(filepath.Join(root, "rt-core/clients/cpp/examples/testdata/valid.rdt"))
	if err != nil {
		t.Fatal(err)
	}
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	// The local stack's explicit pair revision is 1; encode it at the producer
	// boundary, then upload those immutable bytes without daemon-side edits.
	plan.Header.ManifestRevision = 1
	blob, err = densejointtraj.Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	var header map[string]any
	if err := json.Unmarshal(blob[4:4+binary.BigEndian.Uint32(blob)], &header); err != nil {
		t.Fatal(err)
	}
	ingest(map[string]any{"kind": "upload"}, blob)
	identity := map[string]any{"kind": "preload"}
	for _, key := range []string{"plan_id", "program_id", "program_digest", "trajectory_digest", "manifest_revision", "plan_revision"} {
		identity[key] = header[key]
	}
	ingest(identity, nil)
	delete(identity, "kind")
	identity["leader_lease_id"] = lease["lease_id"]
	identity["leader_fence_epoch"] = lease["leader_fence_epoch"]
	identity["motion_intent_seq"] = 1
	command("play", identity)
	client, err := control.Dial(control.UnixPath(os.Getenv("ROSIE_RT_CONTROL_SOCKET")))
	if err != nil {
		t.Fatal(err)
	}
	defer client.Close()
	var state control.Status
	wait(func() bool {
		state, err = client.Status(ctx)
		return err == nil && state.Execution.State == "completed"
	})
	if state.Execution.NativeSequence == 0 || state.Execution.Generation != 1 || state.Core.Axes[0].PositionCounts < 19 || state.Core.Axes[0].PositionCounts > 23 {
		t.Fatalf("dense completion: %s", fmt.Sprint(state.Execution, state.Core.Axes[0]))
	}
	command("stop", nil)
	wait(func() bool {
		s, err := client.Status(ctx)
		return err == nil && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0
	})
}
