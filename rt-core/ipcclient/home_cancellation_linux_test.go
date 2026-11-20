//go:build linux

package ipcclient

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"testing"
	"time"
)

type homeObservation struct {
	TimeNS        uint64 `json:"time_ns"`
	HomeValidMask uint32 `json:"home_valid_mask"`
	ActiveMask    uint32 `json:"native_home_active_axis_mask"`
	Armed         uint32 `json:"armed"`
	EnabledMask   uint32 `json:"axis_enable_mask"`
	Axes          []struct {
		State      uint32 `json:"native_home_state"`
		Statusword uint16 `json:"statusword"`
	} `json:"axes"`
}

func twoAxisHomeFixture(t *testing.T) (fixture, string) {
	t.Helper()
	// The simulator has no stored home-offset object until the transaction
	// writes it. Avoid unrelated startup truth-read retries blocking the metrics
	// stream used to observe the first axis entering Home; the Home transaction
	// still performs its own configured truth write and refresh.
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-home.json"))
	if err != nil {
		t.Fatal(err)
	}
	var machine map[string]any
	if err := json.Unmarshal(raw, &machine); err != nil {
		t.Fatal(err)
	}
	first := machine["axes"].([]any)[0].(map[string]any)
	second := make(map[string]any, len(first))
	for key, value := range first {
		second[key] = value
	}
	second["name"], second["slave_position"] = "axis_1", 2
	machine["axes"] = []any{first, second}
	config, err := json.MarshalIndent(machine, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	// Keep the concurrently written trace on runtime TMPDIR. Go 1.26 TempDir
	// prefers GOTMPDIR; concurrent reads on its /mnt/c build path can fail ENODATA.
	directory, err := os.MkdirTemp("", "home-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(directory) })
	path := filepath.Join(directory, "two-axis-home.json")
	if err := os.WriteFile(path, config, 0600); err != nil {
		t.Fatal(err)
	}
	// The shared fixture joins its machine argument under config/. Keep its
	// existing code unchanged while targeting this independent temporary file.
	relative, err := filepath.Rel(filepath.Join(root, "config"), path)
	if err != nil {
		t.Fatal(err)
	}
	f := startCoreWithArgs(t, relative, "--capture-cycles", "10000")
	trace := filepath.Join(filepath.Dir(f.options.SocketPath), "cycle-capture.jsonl")
	wait(t, f.client, func(s Status) bool { return s.ConfigVerifiedMask == 3 && s.HomeValidMask == 0 && s.Armed == 0 })
	return f, trace
}

func readHomeObservation(t *testing.T, f fixture) homeObservation {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
	if err != nil {
		t.Fatal(err)
	}
	var observation homeObservation
	if err := json.Unmarshal(raw, &observation); err != nil {
		t.Fatal(err)
	}
	now := monotonicNS()
	if len(observation.Axes) != 2 || observation.TimeNS == 0 || observation.TimeNS > now || now-observation.TimeNS > uint64(time.Second) {
		t.Fatalf("invalid/stale home observation: %+v", observation)
	}
	return observation
}

func waitHomeObservation(t *testing.T, f fixture, predicate func(homeObservation) bool) homeObservation {
	t.Helper()
	deadline := time.Now().Add(4 * time.Second)
	var pollResult homeObservation
	if !simfixture.PollUntil(deadline, 5*time.Millisecond, func() bool {
		observation := readHomeObservation(t, f)
		if predicate(observation) {
			pollResult = observation
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("home observation did not converge: %+v", observation)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

// The retained high-rate diagnostic stream observes the simulated CiA402
// response. It supplements 5 Hz metrics, which can miss a service-state pulse.
// It is feedback evidence, not a direct controlword or physical standstill claim.
func laterAxisServiceSamples(t *testing.T, trace string, after uint64) (samples, active int) {
	t.Helper()
	raw, err := os.ReadFile(trace)
	if err != nil {
		t.Fatal(err)
	}
	lines := bytes.Split(raw, []byte{'\n'})
	for _, line := range lines[:len(lines)-1] { // Last line may be concurrently incomplete.
		if len(line) == 0 {
			continue
		}
		var sample struct {
			TimeNS uint64 `json:"t_ns"`
			Axes   []struct {
				Index      uint32 `json:"i"`
				Statusword uint16 `json:"sw"`
			} `json:"ax"`
		}
		if err := json.Unmarshal(line, &sample); err != nil {
			t.Fatal(err)
		}
		if sample.TimeNS < after {
			continue
		}
		for _, axis := range sample.Axes {
			if axis.Index != 1 {
				continue
			}
			samples++
			if axis.Statusword&0x006f != 0x0040 {
				active++
			}
		}
	}
	return samples, active
}

func TestMultiAxisHomeCompletesWithoutCancellation(t *testing.T) {
	f, trace := twoAxisHomeFixture(t)
	sequence, err := f.client.Home(context.Background(), 3)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	defer cancel()
	result, err := f.client.WaitResult(ctx, sequence)
	if err != nil || result.Result != ResultAccepted {
		t.Fatalf("multi-axis Home: %+v %v", result, err)
	}
	completed := monotonicNS()
	observation := waitHomeObservation(t, f, func(s homeObservation) bool { return s.TimeNS >= completed })
	if observation.HomeValidMask != 3 || observation.ActiveMask != 0 || observation.Armed != 0 || observation.EnabledMask != 0 ||
		observation.Axes[0].State != 2 || observation.Axes[1].State != 2 {
		t.Fatalf("successful Home did not establish both disabled coordinates: %+v", observation)
	}
	samples, active := laterAxisServiceSamples(t, trace, 0)
	if samples == 0 || active == 0 {
		t.Fatalf("positive control did not observe axis1 service feedback: samples=%d active=%d", samples, active)
	}
	t.Logf("positive control: home_valid=%#x, states=%d/%d, axis1 service samples=%d", observation.HomeValidMask, observation.Axes[0].State, observation.Axes[1].State, active)
}

func TestStopDuringMultiAxisHomeCannotStartLaterAxis(t *testing.T) {
	f, trace := twoAxisHomeFixture(t)
	sequence, err := f.client.Home(context.Background(), 3)
	if err != nil {
		t.Fatal(err)
	}
	requested := waitHomeObservation(t, f, func(s homeObservation) bool {
		return s.Axes[0].State == 1 && s.Axes[1].State == 0
	})
	if _, err := f.client.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	stopAcknowledged := monotonicNS()
	type completion struct {
		result CommandResult
		err    error
	}
	completed := make(chan completion, 1)
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	defer cancel()
	go func() {
		result, err := f.client.WaitResult(ctx, sequence)
		completed <- completion{result, err}
	}()
	var helperResult completion
	laterAxisObserved := false
	var resurrectedMask uint32
	var final homeObservation
	for waiting := true; waiting; {
		observation := readHomeObservation(t, f)
		if observation.TimeNS >= stopAcknowledged {
			laterAxisObserved = laterAxisObserved || observation.Axes[1].State == 1 || observation.Axes[1].State == 2
			resurrectedMask |= observation.HomeValidMask
		}
		select {
		case helperResult = <-completed:
			waiting = false
		case <-ctx.Done():
			t.Fatal("Home helper did not return after Stop")
		case <-time.After(5 * time.Millisecond):
		}
	}
	var rejected *RejectedError
	if helperResult.err != nil && !errors.As(helperResult.err, &rejected) {
		t.Fatalf("no terminal Home result after cancellation: %+v %v", helperResult.result, helperResult.err)
	}
	// Stop clears ActiveMask immediately; that alone does not prove the worker
	// finished. Wait for its correlated result, then a newer metrics snapshot.
	helperFinished := monotonicNS()
	final = waitHomeObservation(t, f, func(s homeObservation) bool { return s.TimeNS >= helperFinished })
	laterAxisObserved = laterAxisObserved || final.Axes[1].State == 1 || final.Axes[1].State == 2
	resurrectedMask |= final.HomeValidMask
	samples, active := laterAxisServiceSamples(t, trace, stopAcknowledged)
	if samples == 0 {
		t.Fatal("no post-Stop high-rate feedback evidence")
	}
	t.Logf("requested=%+v; completed=%+v; native_result=%+v; axis1 post-Stop samples=%d service=%d", requested, final, helperResult.result, samples, active)
	if laterAxisObserved || resurrectedMask != 0 || final.ActiveMask != 0 || final.Armed != 0 || final.EnabledMask != 0 || active != 0 || helperResult.err == nil {
		t.Fatalf("cancelled Home resumed a later axis or published stale success: later_axis=%v home_valid=%#x active_samples=%d final=%+v result=%+v", laterAxisObserved, resurrectedMask, active, final, helperResult.result)
	}
}
