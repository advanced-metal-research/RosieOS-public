//go:build linux

package control

import (
	"encoding/json"
	"fmt"
	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
	"testing"
)

// Isolate decode from HTTP/TLS/scheduling. The envelope and projection are the
// production types; these measurements are host-local, not robot latency.
func BenchmarkStatusObservationDecode(b *testing.B) {
	for _, n := range []int{0, 400, 4000, 16000} {
		status := Status{}
		status.Core.Axes = make([]ipcclient.AxisStatus, 6)
		status.Axes = make([]adapter.LogicalAxisStatus, 6)
		status.TimeNS = 9007199254740993
		status.Core.TimeNS = status.TimeNS
		status.Motion.TimeNS = status.TimeNS + 2
		status.Execution.Identity = Identity{PlanID: "plan", ProgramID: "program"}
		for i := 0; i < n; i++ {
			status.Execution.Handles = append(status.Execution.Handles, HandleRecord{Handle: uint64(i + 1), State: "consumed", Generation: 7, NativeSequence: uint64(i + 1), Identity: status.Execution.Identity})
		}
		payload, err := json.Marshal(status)
		if err != nil {
			b.Fatal(err)
		}
		full, err := json.Marshal(Response{Schema: "rosie.rt-control.response.v1", Data: payload})
		if err != nil {
			b.Fatal(err)
		}
		status.Execution.Handles = []HandleRecord{}
		payload, _ = json.Marshal(status)
		compact, _ := json.Marshal(Response{Schema: "rosie.rt-control.response.v1", Data: payload})
		b.Run(fmt.Sprintf("handles=%d/full-two-pass", n), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				var r Response
				if err := json.Unmarshal(full, &r); err != nil {
					b.Fatal(err)
				}
				if _, err := data[Status](r, nil); err != nil {
					b.Fatal(err)
				}
			}
		})
		for _, item := range []struct {
			name string
			raw  []byte
		}{{"full-one-pass", full}, {"compact-one-pass", compact}} {
			b.Run(fmt.Sprintf("handles=%d/%s", n, item.name), func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					var r statusObservationResponse
					if err := json.Unmarshal(item.raw, &r); err != nil {
						b.Fatal(err)
					}
				}
			})
		}
	}
}
