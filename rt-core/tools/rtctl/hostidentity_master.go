package rtctl

import (
	"fmt"
	"os"
	"reflect"
	"sort"
	"strconv"
	"strings"
	"syscall"
)

// identitySameDevice compares character device numbers, so an alias for the
// master cannot hide a foreign holder. Regular-file identity supports fixtures.
func identitySameDevice(device, fd os.FileInfo) bool {
	if device.Mode()&os.ModeCharDevice != 0 && fd.Mode()&os.ModeCharDevice != 0 {
		a, aOK := device.Sys().(*syscall.Stat_t)
		b, bOK := fd.Sys().(*syscall.Stat_t)
		return aOK && bOK && a.Rdev == b.Rdev
	}
	return os.SameFile(device, fd)
}

// identityMasterOwner enumerates open descriptors read-only. A hidden or
// unreadable process prevents qualification; any observed foreign holder is a
// conflict even when other process evidence is unavailable.
func (h *hostInspector) identityMasterOwner(core hostServiceIdentity) hostIdentityCheck {
	expected := "only the release-qualified rosie-rt-core MainPID holds /dev/EtherCAT0"
	devicePath, err := h.identityPath("/dev/EtherCAT0")
	if err != nil {
		return identityUnknown("master device unreadable", expected)
	}
	device, err := os.Stat(devicePath)
	if err != nil {
		return identityUnknown("master device unreadable", expected)
	}
	entries, err := os.ReadDir(h.path("/proc"))
	if err != nil {
		return identityUnknown("process enumeration unavailable", expected)
	}
	uncertain := false
	// hidepid can omit processes without returning a permission error. Inspection
	// with that mount policy is deliberately unqualified even for a privileged caller.
	mounts, mountErr := h.identityRead("/proc/mounts")
	if mountErr != nil || strings.Contains(string(mounts), "hidepid=") {
		uncertain = true
	}
	qualifiedCore := identityAllMatch(core.Layers)
	coreSeen := false
	foreign := []int{}
	for _, entry := range entries {
		pid, err := strconv.Atoi(entry.Name())
		if err != nil || pid <= 0 || strconv.Itoa(pid) != entry.Name() {
			continue
		}
		base := "/proc/" + entry.Name()
		start, startErr := h.processStart(pid)
		tasks, err := os.ReadDir(h.path(base + "/task"))
		if err != nil {
			uncertain = true
			continue
		}
		uncertain = uncertain || startErr != nil || len(tasks) == 0
		holder := false
		for _, task := range tasks {
			tid, err := strconv.Atoi(task.Name())
			if err != nil || tid <= 0 || strconv.Itoa(tid) != task.Name() || !task.IsDir() {
				uncertain = true
				continue
			}
			taskPath := base + "/task/" + task.Name()
			before, beforeErr := task.Info()
			fds, err := os.ReadDir(h.path(taskPath + "/fd"))
			if err != nil || beforeErr != nil {
				uncertain = true
				continue
			}
			for _, fd := range fds {
				path, err := h.identityPath(taskPath + "/fd/" + fd.Name())
				if err != nil {
					if !os.IsNotExist(err) {
						uncertain = true
					}
					continue
				}
				info, err := os.Stat(path)
				if err != nil {
					if !os.IsNotExist(err) {
						uncertain = true
					}
					continue
				}
				holder = holder || identitySameDevice(device, info)
			}
			after, err := os.Stat(h.path(taskPath))
			if err != nil || !os.SameFile(before, after) {
				uncertain = true
			}
		}
		closingTasks, err := os.ReadDir(h.path(base + "/task"))
		if err != nil || !reflect.DeepEqual(identityTaskNames(tasks), identityTaskNames(closingTasks)) {
			uncertain = true
		}
		after, err := h.processStart(pid)
		stable := startErr == nil && err == nil && after == start
		uncertain = uncertain || !stable
		if !holder {
			continue
		}
		if pid != core.pid || !qualifiedCore {
			foreign = append(foreign, pid)
		} else if stable && start == core.start {
			// Recheck the loaded bytes after enumeration; re-exec keeps start ticks.
			digest, _, err := h.identityDigest(base + "/exe")
			if err == nil && digest == core.Layers["loaded_image"].Observed {
				coreSeen = true
			} else {
				uncertain = true
			}
		} else {
			uncertain = true
		}
	}
	if len(foreign) != 0 {
		sort.Ints(foreign)
		return hostIdentityCheck{"conflict", fmt.Sprintf("foreign or unqualified master holder PIDs=%v", foreign), expected}
	}
	if uncertain || !coreSeen {
		return identityUnknown(fmt.Sprintf("complete_process_visibility=%t expected_holder_seen=%t", !uncertain, coreSeen), expected)
	}
	return hostIdentityCheck{"match", fmt.Sprintf("sole holder pid=%d start_ticks=%s", core.pid, core.start), expected}
}

// identityTaskNames preserves the enumerated task set so thread churn cannot
// qualify a scan that missed a descriptor table.
func identityTaskNames(tasks []os.DirEntry) []string {
	names := make([]string, len(tasks))
	for i, task := range tasks {
		names[i] = task.Name()
	}
	return names
}
