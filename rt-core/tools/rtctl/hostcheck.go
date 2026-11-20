package rtctl

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"net"
	"os"
	"path/filepath"
	"regexp"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"syscall"
)

// Measured repository dependency: host/ethercat-foundation.sh pins IgH 1.6.9.
const hostcheckIgHVersion = "1.6.9"

// Unverified inspection ceilings, in ns; passing these is not timing qualification.
const hostcheckTimerSlackNS = 50000

// Unverified inspection ceiling: 1000 ns CLOCK_MONOTONIC resolution.
const hostcheckTimerResolutionNS = 1000

type hostCheck struct {
	ID       string `json:"id"`
	Status   string `json:"status"`
	Observed string `json:"observed"`
	Expected string `json:"expected"`
	Error    string `json:"error,omitempty"`
}

type hostcheckReport struct {
	Schema   string              `json:"schema"`
	Checks   []hostCheck         `json:"checks"`
	Identity *hostIdentityReport `json:"release_identity,omitempty"`
	ExitCode int                 `json:"exit_code"`
}

type hostInspector struct {
	root   string
	report hostcheckReport
}

func (h *hostInspector) path(path string) string {
	return filepath.Join(h.root, strings.TrimPrefix(path, "/"))
}
func (h *hostInspector) add(id, status, observed, expected string) {
	h.report.Checks = append(h.report.Checks, hostCheck{ID: id, Status: status, Observed: observed, Expected: expected})
	if status == "fail" && h.report.ExitCode == 0 {
		h.report.ExitCode = 2
	}
}
func (h *hostInspector) check(id string, pass bool, observed, expected string) {
	status := "fail"
	if pass {
		status = "pass"
	}
	h.add(id, status, observed, expected)
}

// Missing optional facilities are observations; unreadable required evidence
// gets exit 3 so an I/O error cannot masquerade as a qualification failure.
func (h *hostInspector) readError(id, path, expected, missing string, err error) {
	if os.IsNotExist(err) && missing != "" {
		h.add(id, missing, path+": absent", expected)
		return
	}
	h.add(id, "fail", path+": unreadable", expected)
	// Keep fixture and real-host errors independent of the root prefix.
	if e, ok := err.(*os.PathError); ok {
		err = e.Err
	}
	h.report.Checks[len(h.report.Checks)-1].Error = err.Error()
	h.report.ExitCode = 3
}

func (h *hostInspector) read(id, path, expected, missing string) (string, bool) {
	raw, err := os.ReadFile(h.path(path))
	if err != nil {
		h.readError(id, path, expected, missing, err)
		return "", false
	}
	return strings.TrimSpace(string(raw)), true
}

// CPU membership accepts only complete decimal lists and bounded Linux CPU IDs;
// malformed IRQ masks must never be interpreted as excluding the RT CPU.
func hostCPUList(list string) (map[int]bool, error) {
	cpus := map[int]bool{}
	if list == "" {
		return cpus, nil
	}
	for _, span := range strings.Split(list, ",") {
		parts := strings.Split(span, "-")
		if len(parts) > 2 {
			return nil, fmt.Errorf("invalid CPU list %q", list)
		}
		values := []int{}
		for _, part := range parts {
			if part == "" || strings.Trim(part, "0123456789") != "" {
				return nil, fmt.Errorf("invalid CPU list %q", list)
			}
			value, err := strconv.Atoi(part)
			// Unverified parser bound: reject IDs above 1048575 rather than allocate from hostile input.
			if err != nil || value > 1048575 {
				return nil, fmt.Errorf("invalid CPU list %q", list)
			}
			values = append(values, value)
		}
		end := values[0]
		if len(values) == 2 {
			end = values[1]
		}
		if end < values[0] {
			return nil, fmt.Errorf("invalid CPU list %q", list)
		}
		for cpu := values[0]; cpu <= end; cpu++ {
			cpus[cpu] = true
		}
	}
	return cpus, nil
}

// Parse literal assignments without executing or sourcing privileged host files.
// Shell expansions and duplicate keys remain invalid instead of being evaluated.
func hostAssignments(raw string) map[string]string {
	result := map[string]string{}
	for _, pair := range hostAssignmentPairs(raw) {
		if _, duplicate := result[pair[0]]; duplicate {
			result[pair[0]] = ""
			continue
		}
		result[pair[0]] = pair[1]
	}
	return result
}

// hostAssignmentPairs keeps every literal assignment in file order, duplicates
// included, so a caller can tell an unset key from a repeated one.
func hostAssignmentPairs(raw string) [][2]string {
	pairs := [][2]string{}
	for _, line := range strings.Split(raw, "\n") {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		key, value, ok := strings.Cut(line, "=")
		if !ok {
			continue
		}
		key, value = strings.TrimSpace(key), strings.TrimSpace(value)
		if len(value) >= 2 && (value[0] == '"' && value[len(value)-1] == '"' || value[0] == '\'' && value[len(value)-1] == '\'') {
			value = value[1 : len(value)-1]
		}
		pairs = append(pairs, [2]string{key, value})
	}
	return pairs
}

type hostMachine struct {
	Runtime struct {
		CPU *int `json:"cpu"`
	} `json:"runtime"`
	EtherCAT struct {
		Interface string `json:"interface"`
		MAC       string `json:"mac"`
	} `json:"ethercat"`
}

// RunHostcheckCLI only reads host state and repository unit templates. The fake
// root replaces every host filesystem read, preventing tests from probing live
// hardware. Passing prerequisites never claims measured cycle timing.
func RunHostcheckCLI(args []string, componentRoot string, stdout, stderr io.Writer) int {
	flags := flag.NewFlagSet("hostcheck", flag.ContinueOnError)
	flags.SetOutput(stderr)
	config := flags.String("config", "", "machine JSON; otherwise read deployed control.env RT CPU")
	jsonOutput := flags.Bool("json", false, "print checks as JSON")
	expectedRelease := flags.String("expected-release", "", "explicit host identity manifest; compare installed services and loaded executables read-only")
	if err := flags.Parse(args); err != nil {
		return 2
	}
	if flags.NArg() != 0 {
		fmt.Fprintln(stderr, "hostcheck accepts --config FILE, --expected-release FILE and --json only")
		return 2
	}
	hostRoot := os.Getenv("ROSIE_RT_HOSTCHECK_ROOT")
	if hostRoot == "" {
		hostRoot = "/"
	}
	h := &hostInspector{root: hostRoot, report: hostcheckReport{Schema: "rosie-rt-core.hostcheck.v1", Checks: []hostCheck{}}}
	var machine hostMachine
	if *config != "" {
		raw, err := os.ReadFile(*config)
		if err != nil {
			h.readError("configuration", *config, "readable machine JSON", "", err)
		} else if err := json.Unmarshal(raw, &machine); err != nil {
			h.add("configuration", "fail", "invalid machine JSON: "+err.Error(), "runtime.cpu is a nonnegative integer")
		}
	} else if raw, ok := h.read("deployment", "/etc/rosie-rt-core/control.env", "deployment with --rt-cpu or explicit --config", "not_applicable"); ok {
		args := hostAssignments(raw)["ROSIE_RT_CORE_ARGS"]
		matches := regexp.MustCompile(`(?:^|\s)--rt-cpu\s+([0-9]+)(?:\s|$)`).FindAllStringSubmatch(args, -1)
		if len(matches) == 1 {
			if cpu, err := strconv.Atoi(matches[0][1]); err == nil {
				machine.Runtime.CPU = &cpu
			}
		}
	}
	cpu := -1
	if machine.Runtime.CPU != nil && *machine.Runtime.CPU >= 0 {
		cpu = *machine.Runtime.CPU
	}
	h.check("rt_cpu", cpu >= 0, strconv.Itoa(cpu), "configured nonnegative runtime.cpu (or deployed --rt-cpu)")

	release, releaseOK := h.read("kernel_version", "/proc/sys/kernel/osrelease", "nonempty kernel release", "")
	if releaseOK {
		h.check("kernel_version", release != "", release, "nonempty kernel release")
	}
	version, versionOK := h.read("kernel_build", "/proc/sys/kernel/version", "uname -v kernel build string", "")
	if versionOK {
		h.check("kernel_build", version != "", version, "uname -v kernel build string")
	}
	realtimeRaw, realtimeErr := os.ReadFile(h.path("/sys/kernel/realtime"))
	if realtimeErr != nil && !os.IsNotExist(realtimeErr) {
		h.readError("kernel_realtime_file", "/sys/kernel/realtime", "readable when present", "", realtimeErr)
	}
	realtime := strings.TrimSpace(string(realtimeRaw)) == "1" || strings.Contains(version, "PREEMPT_RT")
	flagValue := strings.TrimSpace(string(realtimeRaw))
	if os.IsNotExist(realtimeErr) {
		flagValue = "absent"
	}
	h.check("preempt_rt", realtime, "realtime="+flagValue+"; version="+version, "/sys/kernel/realtime=1 or uname -v contains PREEMPT_RT")

	groups, users := h.identities()
	deviceGroup := os.Getenv("ROSIE_RT_ETHERCAT_GROUP")
	if deviceGroup == "" {
		deviceGroup = "ethercat"
	}
	if _, err := os.Stat(h.path("/sys/module/ec_master")); err != nil {
		h.readError("igh_master", "/sys/module/ec_master", "loaded ec_master version "+hostcheckIgHVersion, "fail", err)
	} else if version, ok := h.read("igh_master", "/sys/module/ec_master/version", hostcheckIgHVersion, ""); ok {
		// The module reports "<version> <git describe>"; a release build of the pinned
		// version prints "1.6.9 unknown" on the RevPi bench, so only the first token is the version.
		fields := strings.Fields(version)
		h.check("igh_master", len(fields) > 0 && fields[0] == hostcheckIgHVersion, version, hostcheckIgHVersion)
	}
	h.permissions("ethercat_device", "/dev/EtherCAT0", "", deviceGroup, 0660, false, users, groups)
	nic := h.nic(machine)
	missingTiming := ""
	if !realtime {
		missingTiming = "not_applicable"
	}
	for _, name := range []string{"isolated", "nohz_full"} {
		path := "/sys/devices/system/cpu/" + name
		expected := fmt.Sprintf("includes RT CPU %d", cpu)
		if list, ok := h.read(name, path, expected, missingTiming); ok {
			cpus, err := hostCPUList(list)
			if cpu < 0 {
				h.add(name, "not_applicable", list+"; RT CPU unavailable", expected)
			} else {
				h.check(name, err == nil && cpus[cpu], list, expected)
			}
		}
	}
	if cpu >= 0 {
		path := fmt.Sprintf("/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor", cpu)
		if governor, ok := h.read("cpufreq_governor", path, "performance", missingTiming); ok {
			h.check("cpufreq_governor", governor == "performance", governor, "performance")
		}
	} else {
		h.add("cpufreq_governor", "not_applicable", "RT CPU unavailable", "performance on configured RT CPU")
	}
	h.irqs(nic, cpu)
	h.timers(realtime)
	h.services(componentRoot, deviceGroup, users, groups)
	if *expectedRelease != "" {
		h.releaseIdentity(*expectedRelease, componentRoot)
	}
	h.add("timing_qualification", "not_applicable", "read-only prerequisite inspection; no cycle latency measurement", "separate PREEMPT_RT hardware timing evidence before G1")
	if *jsonOutput {
		encoder := json.NewEncoder(stdout)
		encoder.SetIndent("", "  ")
		if err := encoder.Encode(h.report); err != nil {
			fmt.Fprintln(stderr, err)
			return 3
		}
	} else {
		for _, check := range h.report.Checks {
			fmt.Fprintf(stdout, "%s %s: observed=%q expected=%q", check.Status, check.ID, check.Observed, check.Expected)
			if check.Error != "" {
				fmt.Fprintf(stdout, " error=%q", check.Error)
			}
			fmt.Fprintln(stdout)
		}
		if h.report.Identity != nil {
			h.report.Identity.print(stdout)
		}
	}
	return h.report.ExitCode
}

type hostUser struct {
	uid, gid    string
	home, shell string
}
type hostGroup struct {
	gid     string
	members []string
}

func (h *hostInspector) identities() (map[string]hostGroup, map[string]hostUser) {
	groups := map[string]hostGroup{}
	users := map[string]hostUser{}
	if raw, ok := h.read("groups_file", "/etc/group", "readable local service groups", ""); ok {
		for _, line := range strings.Split(raw, "\n") {
			fields := strings.Split(line, ":")
			if len(fields) == 4 {
				groups[fields[0]] = hostGroup{fields[2], strings.Split(fields[3], ",")}
			}
		}
	}
	if raw, ok := h.read("passwd_file", "/etc/passwd", "readable local service users", ""); ok {
		for _, line := range strings.Split(raw, "\n") {
			fields := strings.Split(line, ":")
			if len(fields) == 7 {
				users[fields[0]] = hostUser{fields[2], fields[3], fields[5], fields[6]}
			}
		}
	}
	return groups, users
}

func (h *hostInspector) permissions(id, path, owner, group string, mode uint32, directory bool, users map[string]hostUser, groups map[string]hostGroup) {
	expected := fmt.Sprintf("owner=%s group=%s mode=%04o", owner, group, mode)
	if owner == "" {
		expected = fmt.Sprintf("group=%s mode=%04o", group, mode)
	}
	info, err := os.Lstat(h.path(path))
	if err != nil {
		h.readError(id, path, expected, "fail", err)
		return
	}
	stat, ok := info.Sys().(*syscall.Stat_t)
	if !ok {
		h.add(id, "fail", "ownership unavailable", expected)
		return
	}
	actualMode := uint32(info.Mode().Perm())
	if info.Mode()&os.ModeSetgid != 0 {
		actualMode |= 02000
	}
	if info.Mode()&os.ModeSetuid != 0 {
		actualMode |= 04000
	}
	if info.Mode()&os.ModeSticky != 0 {
		actualMode |= 01000
	}
	actualOwner, actualGroup := strconv.FormatUint(uint64(stat.Uid), 10), strconv.FormatUint(uint64(stat.Gid), 10)
	ownerOK := owner == "" || users[owner].uid != "" && actualOwner == users[owner].uid
	groupOK := groups[group].gid != "" && actualGroup == groups[group].gid
	if ownerOK {
		actualOwner = owner
	}
	if groupOK {
		actualGroup = group
	}
	observed := fmt.Sprintf("owner=%s group=%s mode=%04o", actualOwner, actualGroup, actualMode)
	if owner == "" {
		observed = fmt.Sprintf("group=%s mode=%04o", actualGroup, actualMode)
	}
	kindOK := info.Mode()&os.ModeSymlink == 0 && info.IsDir() == directory
	h.check(id, ownerOK && groupOK && actualMode == mode && kindOK, observed, expected)
}

// nic fails unless exactly one master NIC is named and it carries that MAC.
// Two files naming different NICs, or one file naming it twice, is ambiguous
// and fails. A file that leaves MASTER0_DEVICE unset (absent, or one empty
// assignment) names no NIC and is not a conflict: IgH 1.6.9 installs
// /etc/sysconfig/ethercat with MASTER0_DEVICE="" for its legacy init script,
// and a Rosie 1430 arm's servo host runs its master from /etc/ethercat.conf through
// `ethercatctl -c /etc/ethercat.conf` with that template left in place (read
// 2026-10-11; Master0 attached to eth0, 6 slaves). A master started from an
// unset file has no device, which ethercat_device (/dev/EtherCAT0) fails.
func (h *hostInspector) nic(machine hostMachine) string {
	expected := "MASTER0_DEVICE MAC matches the configured NIC address"
	configs := []string{"/etc/ethercat.conf", "/etc/sysconfig/ethercat"}
	masterMAC, named := "", false
	for _, path := range configs {
		raw, err := os.ReadFile(h.path(path))
		if os.IsNotExist(err) {
			continue
		}
		if err != nil {
			h.readError("master_nic", path, expected, "", err)
			return ""
		}
		assignments := 0
		for _, pair := range hostAssignmentPairs(string(raw)) {
			if pair[0] == "MASTER0_DEVICE" {
				assignments++
			}
		}
		value := strings.ToLower(hostAssignments(string(raw))["MASTER0_DEVICE"])
		if value == "" && assignments < 2 {
			continue
		}
		if named && value != masterMAC {
			h.add("master_nic", "fail", "conflicting EtherCAT configuration files", expected)
			return ""
		}
		masterMAC, named = value, true
	}
	mac, err := net.ParseMAC(masterMAC)
	if err != nil || len(mac) != 6 {
		h.add("master_nic", "fail", "MASTER0_DEVICE="+masterMAC, expected)
		return ""
	}
	nic := machine.EtherCAT.Interface
	if nic == "" {
		nic = os.Getenv("ROSIE_RT_FOUNDATION_ETHERCAT_INTERFACE")
	}
	if nic == "" {
		entries, err := os.ReadDir(h.path("/sys/class/net"))
		if err != nil {
			h.readError("master_nic", "/sys/class/net", expected, "", err)
			return ""
		}
		matches := []string{}
		for _, entry := range entries {
			raw, err := os.ReadFile(h.path("/sys/class/net/" + entry.Name() + "/address"))
			if err != nil {
				h.readError("nic_address_"+entry.Name(), "/sys/class/net/"+entry.Name()+"/address", "readable NIC address", "", err)
				continue
			}
			if strings.EqualFold(strings.TrimSpace(string(raw)), masterMAC) {
				matches = append(matches, entry.Name())
			}
		}
		if len(matches) != 1 {
			h.add("master_nic", "fail", fmt.Sprintf("MASTER0_DEVICE=%s; matching NICs=%v", masterMAC, matches), expected)
			return ""
		}
		nic = matches[0]
	}
	if filepath.Base(nic) != nic || nic == "." || nic == ".." {
		h.add("master_nic", "fail", "invalid interface="+nic, expected)
		return ""
	}
	address, ok := h.read("master_nic", "/sys/class/net/"+nic+"/address", expected, "")
	if !ok {
		return ""
	}
	matches := strings.EqualFold(address, masterMAC) && (machine.EtherCAT.MAC == "" || strings.EqualFold(address, machine.EtherCAT.MAC))
	h.check("master_nic", matches, "interface="+nic+" address="+address+" MASTER0_DEVICE="+masterMAC, expected)
	return nic
}

func (h *hostInspector) irqs(nic string, cpu int) {
	expected := fmt.Sprintf("nonempty NIC IRQ masks exclude RT CPU %d", cpu)
	if nic == "" || cpu < 0 {
		h.add("nic_irq_affinity", "not_applicable", "NIC or RT CPU unavailable", expected)
		return
	}
	interrupts, ok := h.read("nic_irq_affinity", "/proc/interrupts", expected, "")
	if !ok {
		return
	}
	irqs := map[int]bool{}
	irqLines := map[int]bool{}
	label := regexp.MustCompile(`(?:^|\s)` + regexp.QuoteMeta(nic) + `(?:[-:@\s]|$)`)
	for _, line := range strings.Split(interrupts, "\n") {
		prefix, suffix, found := strings.Cut(line, ":")
		if !found {
			continue
		}
		irq, err := strconv.Atoi(strings.TrimSpace(prefix))
		if err != nil || irq < 0 {
			continue
		}
		irqLines[irq] = true
		if label.MatchString(suffix) {
			irqs[irq] = true
		}
	}
	msiPath := "/sys/class/net/" + nic + "/device/msi_irqs"
	if entries, err := os.ReadDir(h.path(msiPath)); err == nil {
		for _, entry := range entries {
			if irq, err := strconv.Atoi(entry.Name()); err == nil && irq >= 0 {
				irqs[irq] = true
			}
		}
	} else if !os.IsNotExist(err) {
		h.readError("nic_msi_irqs", msiPath, "readable NIC IRQ identities", "", err)
	}
	legacyPath := "/sys/class/net/" + nic + "/device/irq"
	if raw, err := os.ReadFile(h.path(legacyPath)); err == nil {
		if irq, err := strconv.Atoi(strings.TrimSpace(string(raw))); err == nil && irq > 0 {
			irqs[irq] = true
		}
	} else if !os.IsNotExist(err) {
		h.readError("nic_legacy_irq", legacyPath, "readable NIC IRQ identity", "", err)
	}
	ids := []int{}
	for irq := range irqs {
		ids = append(ids, irq)
	}
	sort.Ints(ids)
	if len(ids) == 0 {
		h.add("nic_irq_affinity", "fail", "no NIC IRQs identified for "+nic, expected)
		return
	}
	for _, irq := range ids {
		id := fmt.Sprintf("nic_irq_%d", irq)
		path := fmt.Sprintf("/proc/irq/%d/smp_affinity_list", irq)
		if mask, ok := h.read(id, path, expected, ""); ok {
			cpus, err := hostCPUList(mask)
			h.check(id, irqLines[irq] && err == nil && len(cpus) > 0 && !cpus[cpu], fmt.Sprintf("IRQ %d mask=%s in_interrupts=%t", irq, mask, irqLines[irq]), expected)
		}
	}
}

func (h *hostInspector) timers(realtime bool) {
	// Observe the calling thread without cross-thread procfs permission checks.
	// PR_GET_TIMERSLACK is read-only; keep both timing reads on this OS thread.
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	missing := ""
	if !realtime {
		missing = "not_applicable"
	}
	var raw string
	var slackOK bool
	if h.root == "/" {
		value, err := hostTimerSlack()
		if err != nil {
			h.readError("timer_slack", "prctl(PR_GET_TIMERSLACK)", "readable calling-thread timer slack", "", err)
		} else {
			raw, slackOK = strconv.Itoa(value), true
		}
	} else {
		raw, slackOK = h.read("timer_slack", "/hostcheck/timer_slack_ns", fmt.Sprintf("0..%d ns (unverified inspection ceiling)", hostcheckTimerSlackNS), "")
	}
	if slackOK {
		slack, err := strconv.ParseUint(raw, 10, 64)
		if !realtime {
			h.add("timer_slack", "not_applicable", raw+" ns (hostcheck thread; non-RT kernel)", "PREEMPT_RT required for timing checks")
		} else {
			h.check("timer_slack", err == nil && slack <= hostcheckTimerSlackNS, raw+" ns (hostcheck thread)", fmt.Sprintf("0..%d ns (unverified inspection ceiling)", hostcheckTimerSlackNS))
		}
	}
	var resolution int64
	resolutionOK := true
	if h.root == "/" {
		value, err := hostMonotonicResolution()
		if err != nil {
			h.readError("timer_resolution", "clock_getres(CLOCK_MONOTONIC)", "readable resolution", "", err)
			resolutionOK = false
		} else {
			resolution = value
		}
	} else {
		// The fixture value replaces the read-only syscall, never the real host clock.
		raw, ok := h.read("timer_resolution", "/hostcheck/clock_monotonic_resolution_ns", "CLOCK_MONOTONIC resolution in ns", "")
		resolutionOK = ok
		if ok {
			var err error
			resolution, err = strconv.ParseInt(raw, 10, 64)
			if err != nil {
				resolution = -1
			}
		}
	}
	if resolutionOK {
		if !realtime {
			h.add("timer_resolution", "not_applicable", fmt.Sprintf("%d ns (non-RT kernel)", resolution), "PREEMPT_RT required for timing checks")
		} else {
			h.check("timer_resolution", resolution > 0 && resolution <= hostcheckTimerResolutionNS, fmt.Sprintf("%d ns", resolution), fmt.Sprintf("1..%d ns (unverified inspection ceiling)", hostcheckTimerResolutionNS))
		}
	}
	current, ok := h.read("clocksource", "/sys/devices/system/clocksource/clocksource0/current_clocksource", "current source is listed as available; stability needs hardware evidence", missing)
	if ok {
		available, availableOK := h.read("clocksource_available", "/sys/devices/system/clocksource/clocksource0/available_clocksource", "readable available clocksources", "")
		if availableOK {
			found := false
			for _, source := range strings.Fields(available) {
				found = found || source == current
			}
			if !realtime {
				h.add("clocksource", "not_applicable", "current="+current+" available="+available+" (non-RT kernel)", "PREEMPT_RT required for timing checks")
			} else {
				h.check("clocksource", found && current != "", "current="+current+" available="+available, "current source is listed as available; stability needs hardware evidence")
			}
		}
	}
}

func (h *hostInspector) services(componentRoot, deviceGroup string, users map[string]hostUser, groups map[string]hostGroup) {
	for _, group := range []string{"rosie-rt", "rosie-ctl", "rosie-rt-clients", deviceGroup} {
		entry, ok := groups[group]
		_, err := strconv.ParseUint(entry.gid, 10, 32)
		h.check("group_"+group, ok && err == nil, group+" present="+strconv.FormatBool(ok), "group exists with numeric GID")
	}
	for _, user := range []string{"rosie-rt", "rosie-ctl"} {
		entry, ok := users[user]
		uid, err := strconv.ParseUint(entry.uid, 10, 32)
		expected := "non-root service user; primary=" + user + " home=/nonexistent shell=/usr/sbin/nologin; required supplementary groups"
		required := []string{"rosie-ctl", deviceGroup}
		if user == "rosie-ctl" {
			required = []string{"rosie-rt-clients"}
		}
		memberships := true
		for _, group := range required {
			found := false
			for _, member := range groups[group].members {
				found = found || member == user
			}
			memberships = memberships && (found || entry.gid != "" && entry.gid == groups[group].gid)
		}
		safe := ok && err == nil && uid != 0 && entry.gid != "" && entry.gid == groups[user].gid && entry.home == "/nonexistent" && entry.shell == "/usr/sbin/nologin" && memberships
		observed := fmt.Sprintf("%s present=%t non_root=%t primary_matches=%t home=%s shell=%s memberships=%t", user, ok, err == nil && uid != 0, entry.gid != "" && entry.gid == groups[user].gid, entry.home, entry.shell, memberships)
		h.check("user_"+user, safe, observed, expected)
	}
	for _, service := range []string{"rosie-rt-core", "rosie-rt-control"} {
		path := filepath.Join(componentRoot, "host", service+".service")
		if h.root != "/" {
			path = h.path("/host/" + service + ".service")
		}
		raw, err := os.ReadFile(path)
		id := "memlock_" + service
		if err != nil {
			h.readError(id, "host/"+service+".service", "readable unit template", "", err)
			continue
		}
		value := hostAssignments(string(raw))["LimitMEMLOCK"]
		if service == "rosie-rt-control" && value == "" {
			h.add(id, "not_applicable", "LimitMEMLOCK absent in control unit template", "control adapter does not lock RT memory")
		} else {
			h.check(id, value == "infinity", "template LimitMEMLOCK="+value, "infinity in core unit template; effective service overrides require separate inspection")
		}
	}
	h.permissions("runtime_directory", "/run/rosie-rt-core", "rosie-rt", "rosie-rt-clients", 02770, true, users, groups)
	h.permissions("native_directory", "/run/rosie-rt-core/native", "rosie-rt", "rosie-ctl", 02750, true, users, groups)
	h.permissions("public_directory", "/run/rosie-rt-core/public", "rosie-ctl", "rosie-rt-clients", 02750, true, users, groups)
}
