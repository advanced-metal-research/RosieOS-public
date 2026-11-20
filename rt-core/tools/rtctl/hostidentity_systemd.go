package rtctl

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"regexp"
	"strconv"
	"strings"
	"time"
)

type hostBusValue struct {
	Type string          `json:"type"`
	Data json.RawMessage `json:"data"`
}

type hostManager struct {
	Fragment, LoadState, ActiveState  string
	DropIns, Environment              []string
	EnvironmentFiles                  []string
	Argv, ExecFlags                   []string
	Executable, ControlGroup          string
	MainPID                           int
	StartUS                           uint64
	NeedReload                        bool
	PassEnvironment, UnsetEnvironment []string
	RootDirectory, RootImage          string
}

// managerProperties uses only Properties.GetAll, never a lifecycle method.
// JSON preserves systemd's exact argv boundaries; fixture reads never invoke a
// host command. Manager failures are unknown without exposing environment data.
func (h *hostInspector) managerProperties(service, iface string) (map[string]hostBusValue, error) {
	var raw []byte
	var err error
	if h.root != "/" {
		raw, err = h.identityRead("/hostcheck/systemd/" + service + "." + iface + ".json")
	} else {
		// Unverified inspection ceiling: a D-Bus read is bounded to 5000 ms.
		ctx, cancel := context.WithTimeout(context.Background(), 5000*time.Millisecond)
		defer cancel()
		object := strings.NewReplacer("-", "_2d", ".", "_2e").Replace(service + ".service")
		cmd := exec.CommandContext(ctx, "busctl", "--system", "--json=short", "call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1/unit/"+object, "org.freedesktop.DBus.Properties", "GetAll", "s", "org.freedesktop.systemd1."+iface)
		cmd.Env = append(os.Environ(), "LC_ALL=C", "SYSTEMD_PAGER=cat")
		raw, err = cmd.Output()
	}
	if err != nil || identityNoDuplicateKeys(raw) != nil {
		return nil, fmt.Errorf("systemd %s properties unavailable", iface)
	}
	var wrapper hostBusValue
	var values []map[string]hostBusValue
	if json.Unmarshal(raw, &wrapper) != nil || wrapper.Type != "a{sv}" || json.Unmarshal(wrapper.Data, &values) != nil || len(values) != 1 {
		return nil, fmt.Errorf("systemd %s properties malformed", iface)
	}
	return values[0], nil
}

func (h *hostInspector) managerSnapshot(service string) (hostManager, error) {
	var m hostManager
	unit, err := h.managerProperties(service, "Unit")
	if err != nil {
		return m, err
	}
	process, err := h.managerProperties(service, "Service")
	if err != nil {
		return m, err
	}
	for _, p := range []struct {
		values    map[string]hostBusValue
		key, kind string
		out       any
	}{
		{unit, "FragmentPath", "s", &m.Fragment}, {unit, "DropInPaths", "as", &m.DropIns},
		{unit, "LoadState", "s", &m.LoadState}, {unit, "ActiveState", "s", &m.ActiveState},
		{unit, "NeedDaemonReload", "b", &m.NeedReload},
		{process, "MainPID", "u", &m.MainPID}, {process, "ExecMainStartTimestampMonotonic", "t", &m.StartUS},
		{process, "Environment", "as", &m.Environment}, {process, "ControlGroup", "s", &m.ControlGroup},
		{process, "PassEnvironment", "as", &m.PassEnvironment}, {process, "UnsetEnvironment", "as", &m.UnsetEnvironment},
		{process, "RootDirectory", "s", &m.RootDirectory}, {process, "RootImage", "s", &m.RootImage},
	} {
		v, ok := p.values[p.key]
		if !ok || v.Type != p.kind || string(v.Data) == "null" || json.Unmarshal(v.Data, p.out) != nil {
			return m, fmt.Errorf("missing or malformed systemd property: %s", p.key)
		}
	}
	var commands [][]json.RawMessage
	c := process["ExecStartEx"]
	if c.Type != "a(sasasttttuii)" || json.Unmarshal(c.Data, &commands) != nil || len(commands) != 1 || len(commands[0]) != 10 || json.Unmarshal(commands[0][0], &m.Executable) != nil || json.Unmarshal(commands[0][1], &m.Argv) != nil || string(commands[0][2]) == "null" || json.Unmarshal(commands[0][2], &m.ExecFlags) != nil || len(m.Argv) == 0 {
		return m, fmt.Errorf("single unambiguous ExecStartEx unavailable")
	}
	var files [][]json.RawMessage
	f := process["EnvironmentFiles"]
	if f.Type != "a(sb)" || string(f.Data) == "null" || json.Unmarshal(f.Data, &files) != nil {
		return m, fmt.Errorf("environment file list unavailable")
	}
	m.EnvironmentFiles = []string{}
	for _, row := range files {
		var path string
		var optional bool
		if len(row) != 2 || json.Unmarshal(row[0], &path) != nil || json.Unmarshal(row[1], &optional) != nil || !identityAbsolute(path) || strings.ContainsAny(path, "*?[%") {
			return m, fmt.Errorf("unsupported environment file path")
		}
		// Even optional files must be readable to establish an explicit identity.
		m.EnvironmentFiles = append(m.EnvironmentFiles, path)
	}
	return m, nil
}

// identityWords accepts the literal and quoted forms emitted by control-env.
// Escapes, specifiers and multiline constructs fail closed instead of attempting
// shell evaluation or guessing systemd's parsing semantics.
func identityWords(raw string) ([]string, error) {
	words := []string{}
	var word strings.Builder
	var quote rune
	started := false
	closedQuote := false
	for _, c := range raw {
		if c == '\\' || c == '%' || c == 0 || c == '\n' || c == '\r' {
			return nil, fmt.Errorf("unsupported literal syntax")
		}
		if quote != 0 {
			if c == quote {
				quote = 0
				closedQuote = true
			} else {
				word.WriteRune(c)
			}
		} else if c == '\'' || c == '"' {
			if started {
				return nil, fmt.Errorf("embedded quote is ambiguous")
			}
			quote, started = c, true
		} else if c == ' ' || c == '\t' {
			if started {
				words = append(words, word.String())
				word.Reset()
				started = false
				closedQuote = false
			}
		} else {
			if closedQuote {
				return nil, fmt.Errorf("text after a quoted word is ambiguous")
			}
			word.WriteRune(c)
			started = true
		}
	}
	if quote != 0 {
		return nil, fmt.Errorf("unclosed quote")
	}
	if started {
		words = append(words, word.String())
	}
	return words, nil
}

func identityEnvAssignment(raw string) (string, string, error) {
	key, value, ok := strings.Cut(raw, "=")
	if !ok || !regexp.MustCompile(`^[A-Za-z_][A-Za-z0-9_]*$`).MatchString(key) || strings.ContainsAny(value, "\x00\n\r") {
		return "", "", fmt.Errorf("invalid environment assignment")
	}
	return key, value, nil
}

func (h *hostInspector) managerEnvironment(m hostManager) (map[string]string, error) {
	env := map[string]string{}
	if len(m.PassEnvironment) != 0 || len(m.UnsetEnvironment) != 0 || m.RootDirectory != "" || m.RootImage != "" {
		return nil, fmt.Errorf("inherited, removed or namespaced environment is unsupported")
	}
	for _, assignment := range m.Environment {
		key, value, err := identityEnvAssignment(assignment)
		if _, duplicate := env[key]; err != nil || duplicate {
			return nil, fmt.Errorf("ambiguous manager environment")
		}
		env[key] = value
	}
	for _, path := range m.EnvironmentFiles {
		raw, err := h.identityRead(path)
		if err != nil {
			return nil, fmt.Errorf("environment file is unreadable")
		}
		seen := map[string]bool{}
		for _, line := range strings.Split(string(raw), "\n") {
			line = strings.TrimSpace(line)
			if line == "" || strings.HasPrefix(line, "#") || strings.HasPrefix(line, ";") {
				continue
			}
			key, rawValue, err := identityEnvAssignment(line)
			if err != nil || seen[key] {
				return nil, fmt.Errorf("ambiguous environment file")
			}
			seen[key] = true
			words, err := identityWords(rawValue)
			if err != nil || len(words) > 1 {
				return nil, fmt.Errorf("unsupported environment file value")
			}
			value := ""
			if len(words) == 1 {
				value = words[0]
			}
			env[key] = value
		}
	}
	if env["LD_PRELOAD"] != "" || env["LD_LIBRARY_PATH"] != "" || env["LD_AUDIT"] != "" {
		return nil, fmt.Errorf("unverified loader environment")
	}
	return env, nil
}

func identityExpand(argv []string, env map[string]string) ([]string, error) {
	result := []string{}
	variable := regexp.MustCompile(`^[A-Za-z_][A-Za-z0-9_]*$`)
	braced := regexp.MustCompile(`\$\{([A-Za-z_][A-Za-z0-9_]*)\}`)
	for _, arg := range argv {
		if strings.HasPrefix(arg, "$") && variable.MatchString(arg[1:]) {
			// Missing variables expand to zero arguments, as in systemd; expected
			// argv and the environment binding are still compared independently.
			words, err := identityWords(env[arg[1:]])
			if err != nil {
				return nil, err
			}
			result = append(result, words...)
			continue
		}
		remaining := braced.ReplaceAllString(arg, "")
		if strings.ContainsAny(remaining, "$%") {
			return nil, fmt.Errorf("unsupported ExecStart expansion")
		}
		result = append(result, braced.ReplaceAllStringFunc(arg, func(match string) string { return env[match[2:len(match)-1]] }))
	}
	return result, nil
}

func (h *hostInspector) inspectService(s *hostServiceIdentity, expected hostReleaseService, component string) {
	h.inspectServiceSnapshots(s, expected, component, h.managerSnapshot)
}

// inspectServiceSnapshots brackets procfs and file reads with manager snapshots;
// a restart or reload during inspection cannot qualify a mixed observation.
func (h *hostInspector) inspectServiceSnapshots(s *hostServiceIdentity, expected hostReleaseService, component string, snapshot func(string) (hostManager, error)) {
	layers := s.Layers
	layers["expected_release"] = identityComparison(expected.ExecutableSHA256, expected.ExecutableSHA256)
	template := filepath.Join(component, "host", s.Service+".service")
	if h.root != "/" {
		template = h.path("/host/" + s.Service + ".service")
	}
	if raw, err := os.ReadFile(template); err == nil {
		layers["template"] = identityComparison(identitySHA(raw), expected.TemplateSHA256)
	} else {
		layers["template"] = identityUnknown("unit template is unreadable", expected.TemplateSHA256)
	}
	for _, file := range expected.UnitFiles {
		layers["installed_file:"+file.Path] = h.identityFile(file.Path, file.SHA256)
	}
	layers["installed_executable"] = h.identityFile(expected.Argv[0], expected.ExecutableSHA256)
	for _, key := range []string{"effective_unit", "effective_command", "effective_environment", "process_identity", "process_command", "process_environment", "loaded_image", "stable_observation"} {
		layers[key] = identityUnknown("required observation unavailable", "matches explicit release and stable running service")
	}
	m, err := snapshot(s.Service)
	if err != nil {
		layers["effective_unit"] = identityUnknown(err.Error(), "loaded systemd unit and complete properties")
		return
	}
	paths := append([]string{m.Fragment}, m.DropIns...)
	wantPaths := []string{}
	for _, file := range expected.UnitFiles {
		wantPaths = append(wantPaths, file.Path)
	}
	layers["effective_unit"] = identityComparison(identityJSON(paths), identityJSON(wantPaths))
	if m.LoadState != "loaded" || m.ActiveState != "active" || m.NeedReload {
		layers["effective_unit"] = hostIdentityCheck{"conflict", fmt.Sprintf("load=%s active=%s need_daemon_reload=%t", m.LoadState, m.ActiveState, m.NeedReload), "loaded active unit; no pending daemon reload"}
	}
	// Read every effective fragment, including unexpected drop-ins, without
	// treating an unapproved file as release evidence.
	for _, path := range paths {
		if _, known := layers["installed_file:"+path]; !known {
			digest, _, err := h.identityDigest(path)
			check := hostIdentityCheck{"conflict", digest, "file must be listed in expected release"}
			if err != nil {
				check = identityUnknown("effective file unreadable", "file must be listed in expected release")
			}
			layers["installed_file:"+path] = check
		}
	}
	layers["environment_files"] = identityComparison(identityJSON(m.EnvironmentFiles), identityJSON(expected.EnvironmentFiles))
	env, envErr := h.managerEnvironment(m)
	if envErr == nil {
		layers["effective_environment"] = identityEnvironmentComparison(identityEnvironment(s.Service, env), expected.Environment)
		argv, err := identityExpand(m.Argv, env)
		if err == nil && len(m.ExecFlags) == 0 && identityAbsolute(m.Executable) && len(argv) > 0 {
			layers["effective_command"] = identityComparison(m.Executable+" "+identityArgv(argv), expected.Argv[0]+" "+identityArgv(expected.Argv))
		} else {
			layers["effective_command"] = identityUnknown("unsupported ExecStart syntax or flags", identityArgv(expected.Argv))
		}
	} else {
		layers["effective_environment"] = identityUnknown(envErr.Error(), "readable literal release environment")
	}
	h.inspectProcess(s, m, expected)
	after, err := snapshot(s.Service)
	stable := err == nil && reflect.DeepEqual(m, after)
	if s.pid != 0 {
		base := fmt.Sprintf("/proc/%d/", s.pid)
		cmdline, cmdErr := h.identityRead(base + "cmdline")
		environ, envErr := h.identityRead(base + "environ")
		stable = stable && cmdErr == nil && envErr == nil && identitySHA(cmdline) == s.commandSHA && identitySHA(environ) == s.environmentSHA
		if layers["loaded_image"].Status != "unknown" {
			image, info, imageErr := h.identityDigest(base + "exe")
			stable = stable && imageErr == nil && image == layers["loaded_image"].Observed && os.SameFile(s.imageInfo, info)
		}
	}
	stable = stable && h.identityFile(expected.Argv[0], expected.ExecutableSHA256) == layers["installed_executable"]
	if envErr == nil {
		afterEnv, err := h.managerEnvironment(m)
		stable = stable && err == nil && reflect.DeepEqual(env, afterEnv)
	}
	for _, file := range expected.UnitFiles {
		stable = stable && h.identityFile(file.Path, file.SHA256) == layers["installed_file:"+file.Path]
	}
	// Close the process incarnation after the last command, environment and image
	// read; a reused PID with identical release bytes must remain unqualified.
	if s.pid != 0 {
		start, startErr := h.processStart(s.pid)
		info, infoErr := os.Stat(h.path(fmt.Sprintf("/proc/%d", s.pid)))
		stable = stable && startErr == nil && infoErr == nil && start == s.start && os.SameFile(s.processInfo, info)
	}
	closing, closingErr := snapshot(s.Service)
	stable = stable && closingErr == nil && reflect.DeepEqual(m, closing)
	if stable {
		layers["stable_observation"] = identityComparison("unchanged manager, unit files and environment", "unchanged manager, unit files and environment")
	} else {
		layers["stable_observation"] = identityUnknown("service or files changed during inspection", "unchanged manager, unit files and environment")
	}
}

func (h *hostInspector) processStart(pid int) (string, error) {
	raw, err := h.identityRead(fmt.Sprintf("/proc/%d/stat", pid))
	if err != nil {
		return "", err
	}
	// /proc stat field 2 may contain spaces and parentheses. Field 22 is the
	// process start count in clock ticks, never a wall-clock timestamp.
	close := strings.LastIndex(string(raw), ") ")
	if close < 0 || !strings.HasPrefix(string(raw), strconv.Itoa(pid)+" (") {
		return "", fmt.Errorf("invalid process stat")
	}
	fields := strings.Fields(string(raw[close+2:]))
	if len(fields) < 20 || fields[0] == "Z" || fields[0] == "X" {
		return "", fmt.Errorf("no live process identity")
	}
	ticks, err := strconv.ParseUint(fields[19], 10, 64)
	if err != nil || ticks == 0 {
		return "", fmt.Errorf("invalid process start ticks")
	}
	return strconv.FormatUint(ticks, 10), nil
}

func identityNULList(raw []byte) ([]string, error) {
	if len(raw) == 0 || raw[len(raw)-1] != 0 {
		return nil, fmt.Errorf("incomplete NUL-delimited process data")
	}
	return strings.Split(string(raw[:len(raw)-1]), "\x00"), nil
}

func (h *hostInspector) inspectProcess(s *hostServiceIdentity, m hostManager, expected hostReleaseService) {
	if m.MainPID <= 0 || m.StartUS == 0 {
		s.Layers["process_identity"] = identityUnknown("MainPID or systemd start identity absent", "positive MainPID and start identity")
		return
	}
	base := fmt.Sprintf("/proc/%d/", m.MainPID)
	processInfo, infoErr := os.Stat(h.path(base))
	start, err := h.processStart(m.MainPID)
	boot, bootErr := h.identityRead("/proc/sys/kernel/random/boot_id")
	bootID := strings.TrimSpace(string(boot))
	if infoErr != nil || err != nil || bootErr != nil || !regexp.MustCompile(`^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$`).MatchString(bootID) {
		return
	}
	beforeImage, beforeInfo, beforeErr := h.identityDigest(base + "exe")
	cmdline, cmdErr := h.identityRead(base + "cmdline")
	if argv, err := identityNULList(cmdline); cmdErr == nil && err == nil {
		s.Layers["process_command"] = identityComparison(identityArgv(argv), identityArgv(expected.Argv))
	}
	environ, envErr := h.identityRead(base + "environ")
	if values, err := identityNULList(environ); envErr == nil && err == nil {
		env := map[string]string{}
		valid := true
		for _, value := range values {
			key, binding, err := identityEnvAssignment(value)
			_, duplicate := env[key]
			valid = valid && err == nil && !duplicate
			env[key] = binding
		}
		valid = valid && env["LD_PRELOAD"] == "" && env["LD_LIBRARY_PATH"] == "" && env["LD_AUDIT"] == ""
		if valid {
			s.Layers["process_environment"] = identityEnvironmentComparison(identityEnvironment(s.Service, env), expected.Environment)
		}
	}
	cgroup, groupErr := h.identityRead(base + "cgroup")
	member := false
	for _, line := range strings.Split(string(cgroup), "\n") {
		parts := strings.SplitN(line, ":", 3)
		if len(parts) == 3 && (parts[0] == "0" || parts[1] == "name=systemd") && parts[2] == m.ControlGroup && m.ControlGroup != "" {
			member = true
		}
	}
	afterCmd, afterCmdErr := h.identityRead(base + "cmdline")
	afterEnv, afterEnvErr := h.identityRead(base + "environ")
	image, imageInfo, imageErr := h.identityDigest(base + "exe")
	after, afterErr := h.processStart(m.MainPID)
	afterInfo, afterInfoErr := os.Stat(h.path(base))
	if beforeErr == nil && imageErr == nil && image == beforeImage && os.SameFile(beforeInfo, imageInfo) {
		s.Layers["loaded_image"] = identityComparison(image, expected.ExecutableSHA256)
	} else {
		s.Layers["loaded_image"] = identityUnknown("loaded executable is unreadable or changed", expected.ExecutableSHA256)
	}
	if afterErr == nil && afterInfoErr == nil && os.SameFile(processInfo, afterInfo) && start == after && cmdErr == nil && envErr == nil && afterCmdErr == nil && afterEnvErr == nil && string(cmdline) == string(afterCmd) && string(environ) == string(afterEnv) && member && groupErr == nil {
		identity := fmt.Sprintf("pid=%d boot_id=%s start_ticks=%s systemd_start_us=%d cgroup=%s", m.MainPID, bootID, start, m.StartUS, m.ControlGroup)
		s.Layers["process_identity"] = hostIdentityCheck{"match", identity, "stable MainPID, boot ID, start ticks and systemd cgroup"}
		s.pid, s.start = m.MainPID, start
		s.commandSHA, s.environmentSHA = identitySHA(cmdline), identitySHA(environ)
		s.processInfo, s.imageInfo = processInfo, imageInfo
	}
}
