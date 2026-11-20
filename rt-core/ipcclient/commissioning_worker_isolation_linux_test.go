//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/internal/unixhttp"
	"strconv"
	"testing"
	"time"
)

// All timing assertions use independent absolute budgets and CLOCK_MONOTONIC
// timestamps from native command/result envelopes or simulator mailbox events.
func commissioningFixture(t *testing.T, axes int, blockMS string) fixture {
	t.Helper()
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-home.json"))
	if err != nil {
		t.Fatal(err)
	}
	var config map[string]any
	if err = json.Unmarshal(raw, &config); err != nil {
		t.Fatal(err)
	}
	first := config["axes"].([]any)[0].(map[string]any)
	var all []any
	for i := 0; i < axes; i++ {
		axis := make(map[string]any)
		for k, v := range first {
			axis[k] = v
		}
		axis["name"], axis["slave_position"] = fmt.Sprintf("axis_%d", i), i
		all = append(all, axis)
	}
	config["axes"] = all
	raw, err = json.Marshal(config)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "commissioning.json")
	if err = os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	relative, err := filepath.Rel(filepath.Join(root, "config"), path)
	if err != nil {
		t.Fatal(err)
	}
	return startCoreWithArgs(t, relative, "--sim-sdo-block-ms", blockMS)
}

func mailboxStarted(t *testing.T, f fixture) uint64 {
	t.Helper()
	pattern := regexp.MustCompile(`SIM_SDO_BLOCK begin_ns=(\d+) duration_ms=`)
	deadline := time.Now().Add(time.Second)
	var pollResult uint64
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "core.log"))
		if err != nil {
			t.Fatal(err)
		}
		if match := pattern.FindSubmatch(raw); len(match) == 2 {
			ns, err := strconv.ParseUint(string(match[1]), 10, 64)
			if err != nil {
				t.Fatal(err)
			}
			pollResult = ns
			return true
		}
		if time.Now().After(deadline) {
			t.Fatal("no native blocked-mailbox event")
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func commissioningResult(t *testing.T, c *Client, sequence uint64) CommandResult {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	result, err := c.WaitResult(ctx, sequence)
	var rejected *RejectedError
	if err != nil && !errors.As(err, &rejected) {
		t.Fatalf("result %d: %+v %v", sequence, result, err)
	}
	if result.Sequence != sequence {
		t.Fatalf("wrong job receipt: %+v", result)
	}
	return result
}

func nativeEnvelopeTime(t *testing.T, r ring, sequence uint64, result bool) uint64 {
	t.Helper()
	written, _ := r.indices()
	count := written
	if count > r.capacity {
		count = r.capacity
	}
	for i := uint32(0); i < count; i++ {
		offset := r.offset + 32 + ((written-1-i)%r.capacity)*GRADIENT_RING_MSG_BYTES
		slot := r.data[offset : offset+GRADIENT_RING_MSG_BYTES]
		if result {
			if wire.Uint16(slot) != MsgCommandResult || wire.Uint64(slot[24:]) != sequence {
				continue
			}
		} else if wire.Uint64(slot[8:]) != sequence {
			continue
		}
		return wire.Uint64(slot[16:])
	}
	t.Fatalf("missing native envelope for sequence %d", sequence)
	return 0
}

func boundedCommissioningCommand(t *testing.T, c *Client, kind uint16, payload []byte) CommandResult {
	t.Helper()
	seq, err := c.command(context.Background(), kind, payload)
	if err != nil {
		t.Fatal(err)
	}
	sent := nativeEnvelopeTime(t, c.commands, seq, false)
	result := commissioningResult(t, c, seq)
	returned := nativeEnvelopeTime(t, c.statuses, seq, true)
	if returned < sent || returned-sent > 50_000_000 {
		t.Fatalf("command %#x blocked: native dispatch-to-result=%d ns", kind, returned-sent)
	}
	t.Logf("command %#x: %d ns, result=%d reason=%d", kind, returned-sent, result.Result, result.Reason)
	return result
}

func maskPayload(mask uint32) []byte {
	p := make([]byte, 8)
	wire.PutUint32(p, mask)
	return p
}

func serviceWritePayload(mask uint32, value uint16) []byte {
	p := make([]byte, 16)
	wire.PutUint32(p, mask)
	wire.PutUint16(p[4:], 0x3000)
	p[6], p[7] = 1, 1
	wire.PutUint32(p[8:], uint32(value))
	return p
}

func TestBlockedSdoDoesNotDelayControlCommands(t *testing.T) {
	f := commissioningFixture(t, 1, "2000")
	c := f.client
	home, err := c.Home(context.Background(), 1)
	if err != nil {
		t.Fatal(err)
	}
	blocked := mailboxStarted(t, f)
	for _, kind := range []uint16{0x0102, 0x0101} {
		r := boundedCommissioningCommand(t, c, kind, maskPayload(1))
		if r.Result != ResultRejected || r.Reason != ReasonNotReady {
			t.Fatalf("Home permit bypass: %+v", r)
		}
	}
	begin := make([]byte, 24)
	wire.PutUint64(begin, 811)
	wire.PutUint32(begin[8:], 1)
	wire.PutUint32(begin[16:], 2)
	if r := boundedCommissioningCommand(t, c, 0x0120, begin); r.Result != ResultAccepted {
		t.Fatalf("inert Begin: %+v", r)
	}
	for i := uint32(0); i < 2; i++ {
		point := make([]byte, 288)
		wire.PutUint64(point, 811)
		wire.PutUint32(point[8:], i)
		wire.PutUint64(point[16:], uint64(i)*1_000_000)
		wire.PutUint32(point[280:], 1)
		if _, err := c.command(context.Background(), 0x0121, point); err != nil {
			t.Fatal(err)
		}
	}
	commit := make([]byte, 16)
	wire.PutUint64(commit, 811)
	if r := boundedCommissioningCommand(t, c, 0x0122, commit); r.Result != ResultPrepared {
		t.Fatalf("inert Commit: %+v", r)
	}
	handle := EncodeTrajectoryHandle(TrajectoryHandle{Handle: 811})
	if r := boundedCommissioningCommand(t, c, MsgTrajectoryStart, handle[:]); r.Reason != ReasonModeConflict {
		t.Fatalf("Home Start conflict: %+v", r)
	}
	if r := boundedCommissioningCommand(t, c, MsgTrajectoryDiscard, handle[:]); r.Result != ResultAccepted {
		t.Fatalf("Discard: %+v", r)
	}
	before, err := c.Status()
	if err != nil {
		t.Fatal(err)
	}
	motion, err := c.Motion()
	if err != nil {
		t.Fatal(err)
	}
	observing := monotonicNS()
	deadline := time.Now().Add(150 * time.Millisecond)
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		s, se := c.Status()
		m, me := c.Motion()
		if se == nil && me == nil && s.TimeNS > before.TimeNS && m.TimeNS > motion.TimeNS && s.TimeNS >= observing && m.TimeNS >= observing {
			if s.TimeNS-observing > 100_000_000 || m.TimeNS-observing > 100_000_000 {
				t.Fatal("observation missed one publisher period")
			}
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("mailbox stalled observation: %+v %+v %v %v", s, m, se, me)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	if monotonicNS()-blocked >= 2_000_000_000 {
		t.Fatal("positive control was no longer inside blocked SDO")
	}
	if _, err := c.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if c.stopAck.Load() == 0 {
		t.Fatal("Stop was not acknowledged")
	}
	if r := commissioningResult(t, c, home); r.Result != ResultRejected {
		t.Fatalf("late Home success: %+v", r)
	}
}

func TestHomeAndBulkUploadDoNotSerialize(t *testing.T) {
	f := commissioningFixture(t, 2, "2000")
	c := f.client
	points := make([]Point, 250_000)
	for i := range points {
		points[i] = Point{Time: time.Duration(i) * time.Millisecond, Position: []float64{0, 0}}
	}
	home, err := c.Home(context.Background(), 3)
	if err != nil {
		t.Fatal(err)
	}
	blocked := mailboxStarted(t, f)
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	handle, err := c.PrepareTrajectory(ctx, 812, 3, points)
	if err != nil || handle != 812 {
		t.Fatalf("bulk alongside Home: %d %v", handle, err)
	}
	c.commandMu.Lock()
	commitSequence := c.sequence
	c.commandMu.Unlock()
	submitted := nativeEnvelopeTime(t, c.commands, commitSequence, false)
	prepared := nativeEnvelopeTime(t, c.statuses, commitSequence, true)
	// Encoding all 250k points may outlast the first injected SDO delay on a
	// shared runner. Require Commit submission and completion inside one SDO
	// block: a delayed receipt alone cannot establish concurrent processing.
	log, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "core.log"))
	if err != nil {
		t.Fatal(err)
	}
	overlapped := false
	for _, event := range regexp.MustCompile(`SIM_SDO_BLOCK begin_ns=(\d+) duration_ms=(\d+)`).FindAllSubmatch(log, -1) {
		begin, beginErr := strconv.ParseUint(string(event[1]), 10, 64)
		duration, durationErr := strconv.ParseUint(string(event[2]), 10, 64)
		if beginErr != nil || durationErr != nil {
			t.Fatalf("invalid mailbox event: %q", event[0])
		}
		if submitted >= begin && prepared >= submitted && prepared-begin < duration*1_000_000 {
			overlapped = true
		}
	}
	if !overlapped {
		t.Fatalf("native prepared receipt %d did not overlap a blocked mailbox", prepared)
	}
	if r := commissioningResult(t, c, home); r.Result != ResultAccepted {
		t.Fatalf("multi-axis Home: %+v", r)
	}
	state := wait(t, c, func(s Status) bool { return s.HomeValidMask == 3 && s.CommissioningPhase == 0 })
	if state.Armed != 0 || state.AxisEnableMask != 0 {
		t.Fatal("Home enabled motion")
	}
	t.Logf("250000 points prepared %d ns after Home began blocking; native receipt overlaps a blocked mailbox", prepared-blocked)
}

func TestConcurrentCommissioningJobsRejectBusy(t *testing.T) {
	f := commissioningFixture(t, 1, "2000")
	c := f.client
	seq, err := c.Home(context.Background(), 1)
	if err != nil {
		t.Fatal(err)
	}
	mailboxStarted(t, f)
	// Native v1 has no separate busy code: its existing exclusive-job reason
	// is mode_conflict. Retain the frozen wire vocabulary.
	for _, command := range []struct {
		kind    uint16
		payload []byte
	}{
		{0x0108, maskPayload(1)}, {0x0109, serviceWritePayload(1, 7)},
	} {
		r := boundedCommissioningCommand(t, c, command.kind, command.payload)
		if r.Result != ResultRejected || r.Reason != ReasonModeConflict {
			t.Fatalf("exclusive worker did not reject busy: %+v", r)
		}
	}
	if r := commissioningResult(t, c, seq); r.Result != ResultAccepted {
		t.Fatalf("first Home: %+v", r)
	}
	seq, err = c.Home(context.Background(), 1)
	if err != nil {
		t.Fatal(err)
	}
	if r := commissioningResult(t, c, seq); r.Result != ResultAccepted {
		t.Fatalf("new Home after Finish: %+v", r)
	}
}

func dirtyEpochs(t *testing.T, f fixture) (uint64, uint64) {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
	if err != nil {
		t.Fatal(err)
	}
	var s struct {
		Configuration uint64 `json:"configuration_epoch"`
		Dirty         uint64 `json:"dirty_configuration_epoch"`
	}
	if err = json.Unmarshal(raw, &s); err != nil {
		t.Fatal(err)
	}
	return s.Configuration, s.Dirty
}

func TestConfigurationWriteBumpsDirtyEpochAndInvalidatesHomeReservation(t *testing.T) {
	f := commissioningFixture(t, 1, "2000")
	c := f.client
	configuration, dirty := dirtyEpochs(t, f)
	home, err := c.Home(context.Background(), 1)
	if err != nil {
		t.Fatal(err)
	}
	mailboxStarted(t, f)
	if r := boundedCommissioningCommand(t, c, 0x0109, serviceWritePayload(1, 7)); r.Reason != ReasonModeConflict {
		t.Fatalf("write reused Home reservation: %+v", r)
	}
	if r := commissioningResult(t, c, home); r.Result != ResultAccepted {
		t.Fatalf("Home: %+v", r)
	}
	handle, err := c.PrepareTrajectory(context.Background(), 813, 1, []Point{
		{Position: []float64{0}}, {Time: time.Millisecond, Position: []float64{0}},
	})
	if err != nil {
		t.Fatal(err)
	}
	write, err := c.command(context.Background(), 0x0109, serviceWritePayload(1, 7))
	if err != nil {
		t.Fatal(err)
	}
	state := wait(t, c, func(s Status) bool { return s.ConfigVerifiedMask == 0 && s.CommissioningPhase == 2 })
	if state.HomeValidMask != 0 {
		t.Fatal("write preserved old Home evidence")
	}
	current, changed := dirtyEpochs(t, f)
	if current != configuration+1 || changed != dirty+1 {
		t.Fatalf("write epoch: configuration=%d dirty=%d; before=%d/%d", current, changed, configuration, dirty)
	}
	if r := boundedCommissioningCommand(t, c, 0x0102, maskPayload(1)); r.Reason != ReasonNotReady {
		t.Fatalf("write bypassed verification: %+v", r)
	}
	start := EncodeTrajectoryHandle(TrajectoryHandle{Handle: handle})
	if r := boundedCommissioningCommand(t, c, MsgTrajectoryStart, start[:]); r.Reason != ReasonNotReady {
		t.Fatalf("Start bypassed dirty configuration: %+v", r)
	}
	if r := commissioningResult(t, c, write); r.Result != ResultAccepted {
		t.Fatalf("write failed: %+v", r)
	}
	wait(t, c, func(s Status) bool { return s.ConfigVerifiedMask == 1 && s.CommissioningPhase == 0 })
	current, changed = dirtyEpochs(t, f)
	if current != configuration+2 || changed != dirty+1 {
		t.Fatalf("readback epoch: %d/%d", current, changed)
	}
}

func TestStopDuringBlockedMailboxDrainsThenFinishes(t *testing.T) {
	f := commissioningFixture(t, 2, "2000")
	c := f.client
	home, err := c.Home(context.Background(), 3)
	if err != nil {
		t.Fatal(err)
	}
	blocked := mailboxStarted(t, f)
	if _, err = c.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if c.stopAck.Load() == 0 {
		t.Fatal("no inhibited submission acknowledgement")
	}
	draining := wait(t, c, func(s Status) bool { return s.CommissioningPhase == 2 })
	if draining.TimeNS-blocked >= 2_000_000_000 || draining.NativeHomeActiveMask != 0 || draining.Armed != 0 || draining.AxisEnableMask != 0 {
		t.Fatalf("Stop waited for blocked mailbox: %+v", draining)
	}
	if r := commissioningResult(t, c, home); r.Result != ResultRejected || r.Reason != ReasonNotReady {
		t.Fatalf("cancelled Home completion: %+v", r)
	}
	finished := wait(t, c, func(s Status) bool { return s.CommissioningPhase == 0 })
	if finished.HomeValidMask != 0 || finished.Armed != 0 || finished.AxisEnableMask != 0 {
		t.Fatalf("late completion restored permission: %+v", finished)
	}
	// The terminal failed job must release mailbox ownership without reconnect.
	next, err := c.Home(context.Background(), 3)
	if err != nil {
		t.Fatal(err)
	}
	if result := commissioningResult(t, c, next); result.Result != ResultAccepted {
		t.Fatalf("Home after Stop retained busy claim: %+v", result)
	}
	wait(t, c, func(s Status) bool { return s.HomeEpoch == 3 && s.HomeValidMask == 3 && s.CommissioningPhase == 0 })
}

// Run the public HTTP status path against the real adapter while its native
// Home call waits for the commissioning receipt. Renewal has its own request.
func TestBlockedSdoPublicStatusKeepsAdvancing(t *testing.T) {
	f := commissioningFixture(t, 1, "2000")
	if err := f.client.Close(); err != nil {
		t.Fatal(err)
	}
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	dir := filepath.Dir(f.options.SocketPath)
	socket := filepath.Join(dir, "control.sock")

	process := simfixture.StartControl(root, f.options.SocketPath, socket, f.options.ConfigurationSHA256, "commissioning-test", 1)
	simfixture.SpawnLogged(t, process, filepath.Join(dir, "control.log"), 3*time.Second, 0, nil)

	transport := unixhttp.NewClient(socket).Transport.(*http.Transport)
	t.Cleanup(transport.CloseIdleConnections)
	s := &jogService{http: &http.Client{Transport: transport, Timeout: 15 * time.Second}, t: t}
	for deadline := time.Now().Add(3 * time.Second); ; {
		var clock LocalJogClock
		if err = s.get("/v1/jog/clock", &clock); err == nil {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal(err)
		}
		time.Sleep(time.Millisecond)
	}
	r := s.must("acquire", map[string]any{"controller": "commissioning-test", "binding": map[string]any{
		"pair_id": "commissioning-test", "revision": 1, "configuration_sha256": f.options.ConfigurationSHA256}})
	var grant struct {
		Session    string
		Generation uint64
	}
	if err = json.Unmarshal(r.Data, &grant); err != nil {
		t.Fatal(err)
	}
	s.session, s.grant = grant.Session, grant.Generation
	stop := simfixture.Renew(context.Background(), 50*time.Millisecond, func() bool {
		reply, err := s.request("renew", nil)
		if err != nil || reply.Error != "" {
			t.Errorf("independent renewal: %+v %v", reply, err)
			return false
		}
		return true
	})
	t.Cleanup(stop)
	completed := make(chan error, 1)
	go func() {
		r, err := s.request("home", map[string]any{"axis_mask": 1})
		if err == nil && r.Error != "" {
			err = errors.New(r.Error)
		}
		completed <- err
	}()
	blocked := mailboxStarted(t, f)
	for count := 0; count < 3; count++ {
		started := monotonicNS()
		state := s.status(func(s Status) bool { return s.TimeNS >= started })
		if state.TimeNS-started > 100_000_000 || state.CommissioningPhase != 1 || state.NativeHomeActiveMask != 1 {
			t.Fatalf("public status missed one publisher period during Home: %+v, requested=%d", state, started)
		}
	}
	if monotonicNS()-blocked >= 2_000_000_000 {
		t.Fatal("public observation outlived the blocked SDO positive control")
	}
	select {
	case err := <-completed:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(12 * time.Second):
		t.Fatal("public Home did not finish")
	}
}
