//go:build linux

package oracle

import (
	"context"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	. "rosieos/rt-core/sdk/control"
	"strings"
	"testing"
	"time"
)

type service struct {
	client                         *Client
	core                           *exec.Cmd
	root, dir, build, socket, hash string
}

var must = simfixture.Must

func process(t *testing.T, dir, name, binary string, args ...string) *exec.Cmd {
	t.Helper()
	return simfixture.SpawnLogged(t, exec.Command(binary, args...), filepath.Join(dir, name+".log"), 5*time.Second, 1800, simfixture.CheckExit(t), func(raw []byte) {
		// Origin: the frozen control oracle rejects gateway race diagnostics.
		if strings.Contains(string(raw), "DATA RACE") {
			t.Errorf("%s reported a data race", name)
		}
	})
}

func startService(t *testing.T, remoteArgs ...string) *service {
	t.Helper()
	return startServiceMachine(t, nil, remoteArgs...)
}

func startServiceMachine(t *testing.T, machineFixture func(root, build string) (machine, profiles string), remoteArgs ...string) *service {
	t.Helper()
	root, err := filepath.Abs("../../../..")
	must(t, err)
	dir, err := os.MkdirTemp("", "s-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "sdk-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	machine := filepath.Join(root, "config/machines/simulation/simulation-program.json")
	profiles := filepath.Join(root, "config/drives")
	if machineFixture != nil {
		machine, profiles = machineFixture(root, build)
	}
	compiled, args, err := simfixture.CompileProfiles(root, machine, profiles, filepath.Join(build, "config"))
	simfixture.Must(t, err)
	coreSocket := filepath.Join(dir, "ipc.sock")
	coreBinary := simfixture.Locate(root, "rosie-rt-core-sim")
	core := process(t, build, "core", coreBinary, append(args, "--socket-path", coreSocket)...)
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 3*time.Second, time.Millisecond, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	socket := filepath.Join(dir, "control.sock")
	args = simfixture.ControlArgs(coreSocket, socket, filepath.Base(compiled), "bench", 7)
	controlBinary := simfixture.Locate(root, "rt-control")
	process(t, build, "control", controlBinary, append(args, remoteArgs...)...)
	client, err := Dial(UnixPath(socket))
	must(t, err)
	t.Cleanup(func() { client.Close() })
	poll(t, 3*time.Second, func() bool {
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		defer cancel()
		state, err := client.Status(ctx)
		return err == nil && state.Motion.TimeNS != 0
	})
	return &service{client: client, core: core, root: root, dir: dir, build: build, socket: socket, hash: filepath.Base(compiled)}
}

var poll = func(t *testing.T, budget time.Duration, predicate func() bool) {
	t.Helper()
	simfixture.PollTest(t, budget, time.Millisecond, predicate)
}

func (s *service) acquire(t *testing.T, renewal bool) {
	t.Helper()
	_, err := s.client.Acquire(context.Background(), "motion-server", Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: s.hash})
	must(t, err)
	if renewal {
		_, err = s.client.StartRenewal(context.Background(), 50*time.Millisecond)
		must(t, err)
	}
}

func (s *service) enable(t *testing.T) {
	t.Helper()
	_, err := s.client.Enable(context.Background(), 511)
	must(t, err)
	_, err = s.client.Arm(context.Background())
	must(t, err)
	s.waitStatus(t, func(v Status) bool {
		if v.Core.Armed != 1 || v.Core.AxisEnableMask != 511 {
			return false
		}
		return ipcclient.AllOperationEnabled(v.Core)
	})
}

func (s *service) waitStatus(t *testing.T, predicate func(Status) bool) Status {
	t.Helper()
	var state Status
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	defer cancel()
	poll(t, 4*time.Second, func() bool { var err error; state, err = s.client.Status(ctx); return err == nil && predicate(state) })
	return state
}

// oracleProgram applies the frozen oracle fixture transformation using the
// existing dense codec, preserving independent sample-count and count assertions.
func oracleProgram(t *testing.T, root string, torch bool) []byte {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join(root, "adapters/rosie/densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt"))
	must(t, err)
	plan, err := densejointtraj.Decode(raw)
	must(t, err)
	initial := plan.Segments[0].Samples[0].QRad
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			s := &plan.Segments[i].Samples[j]
			for axis := range s.QRad {
				s.QRad[axis] = (s.QRad[axis] - initial[axis]) * .05
				s.QdRadS[axis] *= .05
			}
			if !torch {
				s.TorchOn = false
			}
		}
	}
	result, err := densejointtraj.Encode(plan.Header, plan.Segments)
	must(t, err)
	return result
}

func rejected(t *testing.T, err error, reason string) {
	t.Helper()
	var r *Rejected
	if !errors.As(err, &r) || r.Reason != reason {
		t.Fatalf("want rejection %q, got %v", reason, err)
	}
}
