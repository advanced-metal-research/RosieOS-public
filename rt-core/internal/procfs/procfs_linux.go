//go:build linux

// Package procfs samples named Linux counters from the requested process/thread.
package procfs

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

type Memory struct {
	RSS       uint64  `json:"vm_rss_bytes"`
	HWM       uint64  `json:"vm_hwm_bytes"`
	HeapInuse *uint64 `json:"heap_inuse_bytes"`
}
type Faults struct {
	Minor uint64 `json:"minor_faults"`
	Major uint64 `json:"major_faults"`
}

// ProcessMemory requires both named Linux counters; missing data cannot become
// zero RSS. HeapInuse is unavailable through the current rt-control public API.
func ProcessMemory(pid int) (Memory, error) {
	raw, err := os.ReadFile(fmt.Sprintf("/proc/%d/status", pid))
	if err != nil {
		return Memory{}, err
	}
	return ParseMemory(string(raw))
}

func ParseMemory(raw string) (Memory, error) {
	values := map[string]uint64{}
	for _, line := range strings.Split(raw, "\n") {
		f := strings.Fields(line)
		if len(f) == 0 || (f[0] != "VmRSS:" && f[0] != "VmHWM:") {
			continue
		}
		if len(f) != 3 || f[2] != "kB" {
			return Memory{}, fmt.Errorf("invalid process memory row %q", line)
		}
		n, err := strconv.ParseUint(f[1], 10, 64)
		if err != nil || n > ^uint64(0)/1024 {
			return Memory{}, fmt.Errorf("invalid process memory value %q", line)
		}
		values[f[0]] = n * 1024
	}
	rss, rssOK := values["VmRSS:"]
	hwm, hwmOK := values["VmHWM:"]
	if !rssOK || !hwmOK || rss == 0 || hwm < rss {
		return Memory{}, errors.New("missing or inconsistent VmRSS/VmHWM")
	}
	return Memory{RSS: rss, HWM: hwm}, nil
}

func CycleTID(pid int) (int, error) {
	root := fmt.Sprintf("/proc/%d/task", pid)
	entries, err := os.ReadDir(root)
	if err != nil {
		return 0, err
	}
	var tid int
	for _, entry := range entries {
		raw, err := os.ReadFile(filepath.Join(root, entry.Name(), "comm"))
		if err != nil {
			return 0, err
		}
		if strings.TrimSpace(string(raw)) != "rt-cycle" {
			continue
		}
		if tid != 0 {
			return 0, errors.New("multiple rt-cycle threads")
		}
		tid, err = strconv.Atoi(entry.Name())
		if err != nil {
			return 0, err
		}
	}
	if tid == 0 {
		return 0, errors.New("rt-cycle thread absent")
	}
	return tid, nil
}

func ThreadFaults(pid, tid int) (Faults, error) {
	raw, err := os.ReadFile(fmt.Sprintf("/proc/%d/task/%d/stat", pid, tid))
	if err != nil {
		return Faults{}, err
	}
	return ParseFaults(string(raw))
}

// ParseFaults reads Linux stat fields 10/12 after the final comm parenthesis,
// preventing spaces in thread names from shifting the minor/major counters.
func ParseFaults(raw string) (Faults, error) {
	end := strings.LastIndexByte(raw, ')')
	if end < 0 {
		return Faults{}, errors.New("stat comm absent")
	}
	f := strings.Fields(raw[end+1:])
	if len(f) < 10 {
		return Faults{}, errors.New("stat fault counters absent")
	}
	minor, e1 := strconv.ParseUint(f[7], 10, 64)
	major, e2 := strconv.ParseUint(f[9], 10, 64)
	return Faults{minor, major}, errors.Join(e1, e2)
}

// MemoryCounters requires each requested kernel counter and returns byte units;
// an absent field cannot silently pass a residency bound as zero.
func MemoryCounters(pid int, names ...string) (map[string]uint64, error) {
	raw, err := os.ReadFile(fmt.Sprintf("/proc/%d/status", pid))
	if err != nil {
		return nil, err
	}
	values := make(map[string]uint64)
	for _, line := range strings.Split(string(raw), "\n") {
		fields := strings.Fields(line)
		if len(fields) != 3 {
			continue
		}
		for _, name := range names {
			if fields[0] != name {
				continue
			}
			value, err := strconv.ParseUint(fields[1], 10, 64)
			if err != nil || fields[2] != "kB" {
				return nil, fmt.Errorf("invalid memory counter: %q", line)
			}
			values[name] = value * 1024
		}
	}
	if len(values) != len(names) {
		return nil, fmt.Errorf("missing process memory counters: %s", raw)
	}
	return values, nil
}

// FirstCycleTID retains first-match sampling while threads may disappear.
func FirstCycleTID(pid int) (string, error) {
	entries, err := os.ReadDir(fmt.Sprintf("/proc/%d/task", pid))
	if err != nil {
		return "", err
	}
	for _, entry := range entries {
		comm, err := os.ReadFile(fmt.Sprintf("/proc/%d/task/%s/comm", pid, entry.Name()))
		if err == nil && strings.TrimSpace(string(comm)) == "rt-cycle" {
			return entry.Name(), nil
		}
	}
	return "", errors.New("rt-cycle thread absent")
}
