//go:build linux

package control

import (
	"encoding/json"
	"errors"
	"fmt"
	"net/http/httptest"
	"reflect"
	"testing"

	"rosieos/rt-core/ipcclient"
)

type statusViewCore struct {
	nativeClient
	state  ipcclient.Status
	motion ipcclient.MotionState
	err    error
}

func (n *statusViewCore) Status() (ipcclient.Status, error)      { return n.state, n.err }
func (n *statusViewCore) Motion() (ipcclient.MotionState, error) { return n.motion, n.err }
func (n *statusViewCore) Err() error                             { return nil }

func statusViewFixture(n int) *Controller {
	core := &statusViewCore{state: ipcclient.Status{TimeNS: 9007199254740993, HomeEpoch: 71, ConfigurationEpoch: 81}, motion: ipcclient.MotionState{TimeNS: 9007199254740995, CommandSequence: 101, TrajectoryID: 103}}
	core.state.Axes = make([]ipcclient.AxisStatus, 6)
	for i := range core.state.Axes {
		core.state.Axes[i].TimeNS = core.state.TimeNS
		core.state.Axes[i].HomeValid = true
		core.state.Axes[i].LogicalPosition = float64(i) + .125
	}
	c := &Controller{core: core, daemonIncarnation: "native-fixture", adapterIncarnation: "adapter-fixture", handles: map[uint64]*preparation{}, execution: Execution{State: "faulted", Error: "drive_readiness", Identity: Identity{PlanID: "plan", ProgramID: "program"}}, executionGeneration: 77}
	for i := 0; i < n; i++ {
		id := uint64(i + 1)
		c.handles[id] = &preparation{HandleRecord: HandleRecord{Handle: id, State: HandleConsumed, Generation: 7, NativeSequence: id, Identity: c.execution.Identity}}
	}
	return c
}

func readStatusView(t *testing.T, c *Controller, path string) (ProcessStatus, []byte) {
	t.Helper()
	rec := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(rec, httptest.NewRequest("GET", path, nil))
	if rec.Code != 200 {
		t.Fatalf("status %d: %s", rec.Code, rec.Body.String())
	}
	var response struct {
		Data ProcessStatus `json:"data"`
	}
	if err := json.Unmarshal(rec.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	return response.Data, append([]byte(nil), rec.Body.Bytes()...)
}

func TestStatusControlViewPreservesCurrentEvidenceAndFullHistory(t *testing.T) {
	c := statusViewFixture(4000)
	full, _ := readStatusView(t, c, "/v1/status")
	compact, raw := readStatusView(t, c, "/v1/status?view=control")
	var wire struct {
		Data struct {
			Execution struct {
				Handles json.RawMessage `json:"handles"`
			} `json:"execution"`
		} `json:"data"`
	}
	if err := json.Unmarshal(raw, &wire); err != nil || string(wire.Data.Execution.Handles) != "[]" {
		t.Fatalf("required handle array changed: %s (%v)", wire.Data.Execution.Handles, err)
	}
	again, _ := readStatusView(t, c, "/v1/status")
	unknown, _ := readStatusView(t, c, "/v1/status?view=unknown")
	if len(full.Execution.Handles) != 4000 || !reflect.DeepEqual(full.Execution.Handles, again.Execution.Handles) || !reflect.DeepEqual(full.Execution.Handles, unknown.Execution.Handles) {
		t.Fatal("compact observation changed retained/public history")
	}
	full.Execution.Handles = []HandleRecord{}
	// Process diagnostics are measured separately on each read; all evidence is exact.
	full.Adapter.HeapInuse, full.Adapter.Sys, full.Adapter.Goroutines = compact.Adapter.HeapInuse, compact.Adapter.Sys, compact.Adapter.Goroutines
	if !reflect.DeepEqual(full, compact) {
		t.Fatal("control view altered a field other than handle history or live process diagnostics")
	}
	if compact.TimeNS != 9007199254740993 || compact.Motion.TimeNS != 9007199254740995 || compact.Core.HomeEpoch != 71 || compact.Execution.Error != "drive_readiness" {
		t.Fatal("lost native clock, Home or fault evidence")
	}
}

func TestStatusControlViewPreservesNativeFailure(t *testing.T) {
	c := statusViewFixture(4000)
	c.core.(*statusViewCore).err = errors.New("native observation failed")
	var first []byte
	var code int
	for _, path := range []string{"/v1/status", "/v1/status?view=control"} {
		rec := httptest.NewRecorder()
		NewHandler(c).ServeHTTP(rec, httptest.NewRequest("GET", path, nil))
		if first == nil {
			first = append([]byte(nil), rec.Body.Bytes()...)
			code = rec.Code
		} else if code != rec.Code || string(first) != rec.Body.String() {
			t.Fatal("view changed native failure semantics")
		}
	}
}

func BenchmarkStatusHistoryView(b *testing.B) {
	for _, n := range []int{0, 400, 4000, 16000} {
		for _, history := range []bool{true, false} {
			name := fmt.Sprintf("handles=%d/full=%v", n, history)
			b.Run(name+"/snapshot", func(b *testing.B) {
				c := statusViewFixture(n)
				b.ReportAllocs()
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					if _, err := c.processSnapshot(history); err != nil {
						b.Fatal(err)
					}
				}
			})
			b.Run(name+"/http", func(b *testing.B) {
				c := statusViewFixture(n)
				h := NewHandler(c)
				path := "/v1/status"
				if !history {
					path += "?view=control"
				}
				req := httptest.NewRequest("GET", path, nil)
				b.ReportAllocs()
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					rec := httptest.NewRecorder()
					h.ServeHTTP(rec, req)
					if rec.Code != 200 {
						b.Fatal(rec.Body.String())
					}
					b.ReportMetric(float64(rec.Body.Len()), "response-bytes")
				}
			})
		}
	}
}
