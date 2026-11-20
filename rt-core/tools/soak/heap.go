//go:build linux

package main

import "fmt"

type heapMinimum struct {
	Iteration int     `json:"iteration"`
	Bytes     *uint64 `json:"heap_inuse_bytes"`
}

// assessHeap records the minimum from each released iteration endpoint up to
// the next upload (or final sample). These snapshots are a quiescent proxy, not
// a retained-memory bound: the public API cannot confirm a completed GC cycle.
// Only a declared absolute byte allowance can qualify the proxy measurements.
func (r *record) assessHeap(add func(string, bool, string)) {
	available := r.Summary.AdapterHeapStart != nil && r.Summary.AdapterHeapEnd != nil && r.Summary.AdapterPeakHeap != nil
	var maximum uint64
	if r.Summary.AdapterHeapEnd != nil {
		maximum = *r.Summary.AdapterHeapEnd
	}
	for i, it := range r.Iterations {
		if it.Phase != "run" {
			continue
		}
		end := r.Final.HostNS
		if i+1 < len(r.Iterations) {
			end = r.Iterations[i+1].UploadStartedNS
		}
		minimum := it.End.Adapter.HeapInuse
		for _, s := range r.Samples {
			if s.HostNS >= it.End.HostNS && s.HostNS <= end && emptySlots(s) && s.Adapter.HeapInuse != nil {
				if minimum == nil || *s.Adapter.HeapInuse < *minimum {
					minimum = s.Adapter.HeapInuse
				}
			}
		}
		r.Summary.AdapterHeapMinima = append(r.Summary.AdapterHeapMinima, heapMinimum{it.Number, minimum})
		available = available && minimum != nil
		if minimum != nil {
			maximum = max(maximum, *minimum)
		}
	}
	add("adapter_heap_measurements", available, "start, peak, end and minimum after each iteration in bytes; GC completion unobserved")
	if r.Configuration.AdapterHeapAllowance == nil {
		r.Assertions = append(r.Assertions, assertion{"adapter_heap", "unqualified", "no declared allowance"})
		return
	}
	allowance := *r.Configuration.AdapterHeapAllowance
	add("adapter_heap", available && maximum <= allowance, fmt.Sprintf("maximum post-iteration minimum/final snapshot %d <= %d bytes declared allowance; GC completion unobserved", maximum, allowance))
}
