//go:build linux

package faultmatrix

import (
	"bytes"
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	ipc "rosieos/rt-core/ipcclient"
	"strings"
	"testing"
)

func blockedSDO(t *testing.T, r row) {
	f := compileFixture(t, "simulation-home.json", func(c map[string]any) {
		axes := c["axes"].([]any)
		second := map[string]any{}
		for k, v := range axes[0].(map[string]any) {
			second[k] = v
		}
		second["name"] = "axis_1"
		second["slave_position"] = 2
		c["axes"] = append(axes, second)
	})
	trace := filepath.Join(f.socketDir, "cycle-capture.jsonl")
	f.launch(true, "--sim-sdo-block-ns", "2000000000", "--capture-cycles", "10000")
	seq, err := f.c.Home(context.Background(), 3)
	must(t, err)
	blocked, _ := f.event("sim_sdo_block")
	active := f.until(blocked, r.boundNS, func(s ipc.Status) bool { return s.CommissioningPhase == 1 })
	if active.HomeValidMask != 0 {
		t.Fatal("Home completed during blocked SDO")
	}
	at := ipc.HostMonotonicNS()
	_, err = f.c.Stop(context.Background())
	must(t, err)
	raw, err := os.ReadFile(f.logPath)
	must(t, err)
	if strings.Contains(string(raw), "sim_sdo_return") {
		t.Fatal("Stop ACK waited for mailbox return")
	}
	draining := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.CommissioningPhase == 2 })
	inhibited(t, draining, 0)
	if draining.NativeHomeActiveMask != 0 {
		t.Fatalf("Draining retained service permission: %+v", draining)
	}
	_, err = f.c.WaitResult(context.Background(), seq)
	reject(t, err, ipc.ReasonNotReady)
	returned, _ := f.event("sim_sdo_return")
	if returned < blocked+2_000_000_000 {
		t.Fatal("mailbox did not exercise configured block")
	}
	final := f.until(returned, r.boundNS, func(s ipc.Status) bool { return s.CommissioningPhase == 0 })
	inhibited(t, final, 0)
	if final.HomeValidMask != 0 || final.NativeHomeActiveMask != 0 || final.Axes[1].PositionCounts != 0 || final.Axes[1].Statusword&0x6f != 0x40 {
		t.Fatalf("cancelled Home reached later axis: %+v", final)
	}
	// Scan complete daemon-stamped trace records through the worker's return;
	// final masks alone would miss a later axis briefly acquiring service.
	f.until(returned, r.boundNS, func(s ipc.Status) bool { return s.TimeNS >= returned+f.cycle })
	raw, err = os.ReadFile(trace)
	must(t, err)
	lines := bytes.Split(raw, []byte{'\n'})
	samples := 0
	var last uint64
	for _, line := range lines[:len(lines)-1] {
		var sample struct {
			Time uint64 `json:"t_ns"`
			Axes []struct {
				Index    uint32 `json:"i"`
				Position int32  `json:"p"`
				Status   uint16 `json:"sw"`
			} `json:"ax"`
		}
		must(t, json.Unmarshal(line, &sample))
		if sample.Time < blocked {
			continue
		}
		for _, axis := range sample.Axes {
			if axis.Index == 1 {
				samples++
				last = sample.Time
				if axis.Status&0x6f != 0x40 || axis.Position != 0 {
					t.Fatalf("later axis acquired service: time_ns=%d axis=%+v", sample.Time, axis)
				}
			}
		}
	}
	if samples < 2 || last < returned {
		t.Fatalf("missing mailbox interval trace: samples=%d last_ns=%d returned_ns=%d", samples, last, returned)
	}
}
