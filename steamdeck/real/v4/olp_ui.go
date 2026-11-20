package main

import (
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"time"
)

// olp ui runs the operator Mac's OLP browser UI against a cell: an ssh tunnel
// from the Mac to the cell's OLP API (cell_deploy.olp.listen on the relay) and
// Vite serving offline-programming/v1/ui with its API proxy pointed at the
// tunnel. Until 2026-10-08 this was a hand setup (/tmp/olp-local/detach.py,
// stop.sh) whose tunnel died twice in ~12 h on "Timeout, server <relay> not
// responding" and stayed dead until someone restarted it, leaving the UI with
// "OLP API unreachable".
//
// launchd (KeepAlive, gui/<uid> domain) owns both jobs, so they survive the
// shell or agent session that started them and come back if they exit. The
// tunnel job runs the shared ssh supervisor (tunnel_supervisor.go), which the
// Steam Deck's `pendant tunnel` runs under systemd --user.

const olpUIUsage = `usage: olp ui <start|stop|restart|status|logs> --manifest cell.json [flags]

The operator Mac's OLP browser UI against a cell: a self-healing ssh tunnel to
the cell's OLP API (cell_deploy.olp.listen on the relay) and Vite serving
offline-programming/v1/ui with its API proxy on that tunnel. Both run as
launchd agents (macOS) or systemd --user units (Linux; WSL with systemd=true,
whose localhost the Windows browser reaches), independent of the shell that
started them. Jobs,
state and logs: ~/.rosie/olp-ui/<cell>/.

  start    Start what is not running and leave healthy processes alone; a dead
           tunnel is restarted at once. npm ci only when node_modules does not
           match package-lock.json. [--port 5189] [--tunnel-port 18894]
           [--wait 60s]; a re-rendered job (other ports, transport or tool
           version) is replaced, refused while the robot is in use.
  stop     Stop both. Refused while the OLP reports an armed robot or a held
           lease unless --force: the browser's heartbeat would lapse.
  restart  stop, then start (npm ci when needed).
  status   Read-only: tunnel up/down with its last restart reason, Vite, OLP
           health through the tunnel and through Vite, build id, cell listed.
           Exit 1 when anything is unhealthy.
  logs     Print the tail of tunnel.log and vite.log [--lines 40] [--follow].
start and restart take --transport tailscale|ssh; all take --dry-run.
Open http://127.0.0.1:<port>/offline-programming/v1/ui/.
`

const (
	olpUIDefaultPort       = 5189
	olpUIDefaultTunnelPort = 18894
	olpUIAPIPrefix         = "/api/offline-programming/v1"
	olpUIPagePath          = "/offline-programming/v1/ui/"
)

var olpUIActions = []string{"start", "stop", "restart", "status", "logs"}

// olpUIConfig is what start renders the jobs from; it is saved as ui.json so
// stop, status and logs act on what start launched.
type olpUIConfig struct {
	Cell       string `json:"cell"`
	Manifest   string `json:"manifest"`
	Transport  string `json:"transport"`
	Target     string `json:"target"` // user@host as the transport reaches the relay
	OLPListen  string `json:"olp_listen"`
	Port       int    `json:"port"`
	TunnelPort int    `json:"tunnel_port"`
	UIDir      string `json:"ui_dir"`
	StateDir   string `json:"state_dir"`
	TunnelBin  string `json:"tunnel_bin"`
	NPM        string `json:"npm"`
	PathEnv    string `json:"path_env"`

	backoffMinMS, backoffMaxMS, healthyAfterS int
}

func (c olpUIConfig) TunnelLabel() string { return "com.rosie.olp-ui." + c.Cell + ".tunnel" }
func (c olpUIConfig) ViteLabel() string   { return "com.rosie.olp-ui." + c.Cell + ".vite" }
func (c olpUIConfig) file(name string) string {
	return filepath.Join(c.StateDir, name)
}
func (c olpUIConfig) TunnelBase() string { return fmt.Sprintf("http://127.0.0.1:%d", c.TunnelPort) }
func (c olpUIConfig) ViteBase() string   { return fmt.Sprintf("http://127.0.0.1:%d", c.Port) }
func (c olpUIConfig) PageURL() string    { return c.ViteBase() + olpUIPagePath }

// olpUITunnelSpec is the Mac's forward: 127.0.0.1:<tunnel port> to the cell's
// OLP listen address on the relay, through tailscale ssh or ssh.
func olpUITunnelSpec(c olpUIConfig) tunnelSpec {
	ssh := []string{c.TunnelBin}
	if c.Transport == "tailscale" {
		ssh = append(ssh, "ssh")
	}
	return tunnelSpec{Title: "OLP UI tunnel supervisor for " + c.Cell, RenderedBy: "go run . olp ui start",
		SSH: append(ssh, c.Target), Forwards: []tunnelForward{{LocalPort: c.TunnelPort, Remote: c.OLPListen}},
		LogPath: c.file("tunnel.log"), StatePath: c.file("tunnel.state"),
		backoffMinMS: c.backoffMinMS, backoffMaxMS: c.backoffMaxMS, healthyAfterS: c.healthyAfterS}
}

func olpUITunnelJob(c olpUIConfig) serviceJob {
	return serviceJob{Label: c.TunnelLabel(), Argv: []string{"/bin/bash", c.file("tunnel.sh")}, Dir: c.StateDir,
		Env: map[string]string{"PATH": c.PathEnv}, Log: c.file("tunnel.supervisor.log")}
}

// olpUIViteJob runs `npm run dev` (stage:assets, then vite) on the chosen port;
// --strictPort makes a taken port an exit instead of Vite moving elsewhere.
func olpUIViteJob(c olpUIConfig) serviceJob {
	home, _ := os.UserHomeDir()
	return serviceJob{Label: c.ViteLabel(), Argv: []string{c.NPM, "run", "dev", "--", "--port", strconv.Itoa(c.Port), "--strictPort"},
		Dir: c.UIDir, Log: c.file("vite.log"),
		Env: map[string]string{"PATH": c.PathEnv, "HOME": home, "OFFLINE_PROGRAMMING_API_URL": c.TunnelBase()}}
}

// --- node dependencies ----------------------------------------------------

type olpUILockPackage struct {
	Version  string   `json:"version"`
	Optional bool     `json:"optional"`
	OS       []string `json:"os"`
	CPU      []string `json:"cpu"`
}

type olpUILockfile struct {
	Packages map[string]olpUILockPackage `json:"packages"`
}

// olpUINodeDepsCurrent compares package-lock.json with the hidden lockfile npm
// writes into node_modules. A missing optional or platform-specific package
// (another OS's esbuild binary) is expected; any other missing package or
// version difference means npm ci is needed.
func olpUINodeDepsCurrent(uiDir string) (bool, string) {
	read := func(p string) (olpUILockfile, error) {
		var l olpUILockfile
		raw, err := os.ReadFile(p)
		if err == nil {
			err = json.Unmarshal(raw, &l)
		}
		return l, err
	}
	lock, err := read(filepath.Join(uiDir, "package-lock.json"))
	if err != nil {
		return false, "package-lock.json unreadable: " + err.Error()
	}
	installed, err := read(filepath.Join(uiDir, "node_modules", ".package-lock.json"))
	if err != nil {
		return false, "node_modules has no readable .package-lock.json (never installed or a partial install)"
	}
	keys := make([]string, 0, len(lock.Packages))
	for k := range lock.Packages {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	n := 0
	for _, k := range keys {
		want := lock.Packages[k]
		if k == "" {
			continue
		}
		got, ok := installed.Packages[k]
		switch {
		case !ok && (want.Optional || len(want.OS) > 0 || len(want.CPU) > 0):
			continue
		case !ok:
			return false, k + " is in package-lock.json but not installed"
		case got.Version != want.Version:
			return false, fmt.Sprintf("%s is %s, package-lock.json wants %s", k, got.Version, want.Version)
		}
		n++
	}
	return true, fmt.Sprintf("%d installed packages match package-lock.json", n)
}

// --- launcher -------------------------------------------------------------------

type olpUIJobStatus struct {
	Loaded bool
	PID    int
	State  string
}

type olpUILauncher interface {
	Load(label, plistPath string) error
	Unload(label string) error
	Restart(label string) error
	Status(label string) (olpUIJobStatus, error)
}

type olpUILaunchd struct{ domain string }

func newOLPUILaunchd() olpUILauncher { return olpUILaunchd{domain: fmt.Sprintf("gui/%d", os.Getuid())} }

func (l olpUILaunchd) run(args ...string) (string, int, error) {
	out, err := exec.Command("launchctl", args...).CombinedOutput()
	var exit *exec.ExitError
	if errors.As(err, &exit) {
		return string(out), exit.ExitCode(), nil
	}
	return string(out), 0, err
}

func (l olpUILaunchd) Load(label, plistPath string) error {
	// A job just booted out can take a moment to leave the domain; bootstrap
	// fails with "5: Input/output error" until it has.
	var last string
	for i := 0; i < 20; i++ {
		out, code, err := l.run("bootstrap", l.domain, plistPath)
		if err != nil {
			return err
		}
		if code == 0 {
			return nil
		}
		last = fmt.Sprintf("exit %d: %s", code, strings.TrimSpace(out))
		time.Sleep(250 * time.Millisecond)
	}
	return fmt.Errorf("launchctl bootstrap %s %s: %s", l.domain, plistPath, last)
}

func (l olpUILaunchd) Unload(label string) error {
	out, code, err := l.run("bootout", l.domain+"/"+label)
	if err != nil {
		return err
	}
	if code != 0 && code != 3 && code != 113 && !strings.Contains(out, "No such process") && !strings.Contains(out, "Could not find") {
		return fmt.Errorf("launchctl bootout %s/%s: exit %d: %s", l.domain, label, code, strings.TrimSpace(out))
	}
	return nil
}

func (l olpUILaunchd) Restart(label string) error {
	out, code, err := l.run("kickstart", "-k", l.domain+"/"+label)
	if err == nil && code != 0 {
		err = fmt.Errorf("launchctl kickstart -k %s/%s: exit %d: %s", l.domain, label, code, strings.TrimSpace(out))
	}
	return err
}

var (
	olpUILaunchdPID   = regexp.MustCompile(`(?m)^\s*pid = (\d+)$`)
	olpUILaunchdState = regexp.MustCompile(`(?m)^\s*state = (\S+)$`)
)

func (l olpUILaunchd) Status(label string) (olpUIJobStatus, error) {
	out, code, err := l.run("print", l.domain+"/"+label)
	if err != nil || code != 0 {
		return olpUIJobStatus{}, err
	}
	s := olpUIJobStatus{Loaded: true}
	if m := olpUILaunchdPID.FindStringSubmatch(out); m != nil {
		s.PID, _ = strconv.Atoi(m[1])
	}
	if m := olpUILaunchdState.FindStringSubmatch(out); m != nil {
		s.State = m[1]
	}
	return s, nil
}

// olpUISystemd keeps the jobs alive as systemd --user units on Linux (a WSL
// distribution with systemd=true included). Load installs the rendered unit
// under ~/.config/systemd/user and enables it, so it starts with the user's
// manager like a launchd agent at login; Unload disables and removes it.
type olpUISystemd struct{ unitDir string }

func newOLPUISystemd() olpUILauncher {
	home, _ := os.UserHomeDir()
	return olpUISystemd{unitDir: filepath.Join(home, ".config", "systemd", "user")}
}

func (l olpUISystemd) run(args ...string) (string, int, error) {
	out, err := exec.Command("systemctl", append([]string{"--user"}, args...)...).CombinedOutput()
	var exit *exec.ExitError
	if errors.As(err, &exit) {
		return string(out), exit.ExitCode(), nil
	}
	return string(out), 0, err
}

func (l olpUISystemd) unit(label string) string { return filepath.Join(l.unitDir, label+".service") }

func (l olpUISystemd) Load(label, unitPath string) error {
	raw, err := os.ReadFile(unitPath)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(l.unitDir, 0o700); err != nil {
		return err
	}
	if err := os.WriteFile(l.unit(label), raw, 0o600); err != nil {
		return err
	}
	for _, args := range [][]string{{"daemon-reload"}, {"enable", "--now", label + ".service"}} {
		if out, code, err := l.run(args...); err != nil || code != 0 {
			return fmt.Errorf("systemctl --user %s: exit %d: %v %s", strings.Join(args, " "), code, err, strings.TrimSpace(out))
		}
	}
	return nil
}

func (l olpUISystemd) Unload(label string) error {
	if _, err := os.Stat(l.unit(label)); err != nil {
		return nil
	}
	if out, code, err := l.run("disable", "--now", label+".service"); err != nil || code != 0 {
		return fmt.Errorf("systemctl --user disable --now %s: exit %d: %v %s", label, code, err, strings.TrimSpace(out))
	}
	if err := os.Remove(l.unit(label)); err != nil {
		return err
	}
	_, _, err := l.run("daemon-reload")
	return err
}

func (l olpUISystemd) Restart(label string) error {
	out, code, err := l.run("restart", label+".service")
	if err == nil && code != 0 {
		err = fmt.Errorf("systemctl --user restart %s: exit %d: %s", label, code, strings.TrimSpace(out))
	}
	return err
}

func (l olpUISystemd) Status(label string) (olpUIJobStatus, error) {
	if _, err := os.Stat(l.unit(label)); err != nil {
		return olpUIJobStatus{}, nil
	}
	out, code, err := l.run("show", label+".service", "-p", "ActiveState", "-p", "MainPID")
	if err != nil || code != 0 {
		return olpUIJobStatus{}, err
	}
	s := olpUIJobStatus{Loaded: true}
	for _, line := range strings.Split(out, "\n") {
		k, v, _ := strings.Cut(strings.TrimSpace(line), "=")
		switch k {
		case "ActiveState":
			s.State = v
		case "MainPID":
			s.PID, _ = strconv.Atoi(v)
		}
	}
	return s, nil
}

// olpUIJobFile renders a job for the host's service manager. The systemd unit
// appends to the same log files as the launchd agent, so olp ui logs reads
// one place on both.
func olpUIJobFile(env olpUIEnv, j serviceJob) string {
	if env.goos != "linux" {
		return launchdPlist(j)
	}
	j.Description = j.Label
	logs := fmt.Sprintf("StandardOutput=append:%[1]s\nStandardError=append:%[1]s\n", j.Log)
	return strings.Replace(systemdUserUnit(j), "Restart=always\n", logs+"Restart=always\n", 1)
}

func olpUIJobExt(env olpUIEnv) string {
	if env.goos == "linux" {
		return ".service"
	}
	return ".plist"
}

// --- environment ------------------------------------------------------------------

type olpUIEnv struct {
	launcher olpUILauncher
	client   *http.Client
	lookPath func(string) (string, error)
	npmCI    func(c olpUIConfig) error
	goos     string
	poll     time.Duration
}

var olpUINewEnv = func() olpUIEnv {
	var launcher olpUILauncher = newOLPUILaunchd()
	if runtime.GOOS == "linux" {
		launcher = newOLPUISystemd()
	}
	return olpUIEnv{launcher: launcher, client: &http.Client{Timeout: 5 * time.Second}, lookPath: exec.LookPath,
		npmCI: olpUIRunNPMCI, goos: runtime.GOOS, poll: 250 * time.Millisecond}
}

func olpUIRunNPMCI(c olpUIConfig) error {
	f, err := os.OpenFile(c.file("npm-ci.log"), os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0o600)
	if err != nil {
		return err
	}
	defer f.Close()
	cmd := exec.Command(c.NPM, "ci", "--no-audit", "--no-fund")
	cmd.Dir, cmd.Stdout, cmd.Stderr = c.UIDir, f, f
	cmd.Env = append(os.Environ(), "PATH="+c.PathEnv)
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("npm ci in %s failed: %v (log %s)", c.UIDir, err, c.file("npm-ci.log"))
	}
	return nil
}

// --- probes -------------------------------------------------------------------------

type olpUIHealth struct {
	OK          bool   `json:"ok"`
	BuildID     string `json:"build_id"`
	StartedAt   string `json:"started_at"`
	Maintenance struct {
		Active bool `json:"active"`
	} `json:"maintenance"`
}

func olpUIGetJSON(client *http.Client, url string, v any) (time.Duration, error) {
	start := time.Now()
	resp, err := client.Get(url)
	if err != nil {
		return 0, err
	}
	defer resp.Body.Close()
	body, err := io.ReadAll(io.LimitReader(resp.Body, 4<<20))
	if err != nil {
		return 0, err
	}
	if resp.StatusCode != http.StatusOK {
		snippet := strings.TrimSpace(string(body))
		if len(snippet) > 160 {
			snippet = snippet[:160]
		}
		if snippet == "" {
			snippet = "no body (a dev proxy whose upstream refused the connection)"
		}
		return 0, fmt.Errorf("HTTP %d: %s", resp.StatusCode, snippet)
	}
	if err := json.Unmarshal(body, v); err != nil {
		return 0, fmt.Errorf("not JSON: %v", err)
	}
	return time.Since(start), nil
}

func olpUIHealthAt(client *http.Client, base string) (olpUIHealth, time.Duration, error) {
	var h olpUIHealth
	d, err := olpUIGetJSON(client, base+olpUIAPIPrefix+"/health", &h)
	if err == nil && !h.OK {
		err = fmt.Errorf("health answered ok=false")
	}
	return h, d, err
}

func olpUIPortOpen(port int) bool {
	conn, err := net.DialTimeout("tcp", fmt.Sprintf("127.0.0.1:%d", port), 500*time.Millisecond)
	if err == nil {
		conn.Close()
	}
	return err == nil
}

// olpUIPortHolder names the process listening on port, for a refusal message.
func olpUIPortHolder(port int) string {
	out, err := exec.Command("lsof", "-nP", "-t", fmt.Sprintf("-iTCP:%d", port), "-sTCP:LISTEN").Output()
	pid := strings.Fields(string(out))
	if err != nil || len(pid) == 0 {
		return "an unknown process"
	}
	cmd, _ := exec.Command("ps", "-o", "command=", "-p", pid[0]).Output()
	return fmt.Sprintf("pid %s (%s)", pid[0], strings.TrimSpace(string(cmd)))
}

// olpUIRobotInUse reads the OLP's execution status through the tunnel. Taking
// the tunnel or Vite down while a browser holds an armed session lapses its
// heartbeat and stops the robot, so stop and job replacement refuse then.
// An unreachable OLP is not in use through this UI: nothing is lost.
func olpUIRobotInUse(client *http.Client, base string) (bool, string) {
	var s struct {
		Armed   bool `json:"armed"`
		Session *struct {
			State string `json:"state"`
			Lease struct {
				Held bool `json:"held"`
			} `json:"lease"`
		} `json:"session"`
	}
	if _, err := olpUIGetJSON(client, base+olpUIAPIPrefix+"/dense-execution/status", &s); err != nil {
		return false, "OLP execution status unreadable through the tunnel (" + err.Error() + "); nothing to interrupt"
	}
	if s.Armed {
		return true, "the OLP reports the robot armed"
	}
	if s.Session != nil && s.Session.Lease.Held {
		return true, fmt.Sprintf("the OLP holds the session lease (session %s)", s.Session.State)
	}
	state := "no session"
	if s.Session != nil {
		state = "session " + s.Session.State + ", no lease"
	}
	return false, "robot disarmed, " + state
}

// olpUIReport prints the health of every part and returns what is unhealthy.
func olpUIReport(env olpUIEnv, c olpUIConfig, p *rtCoreProgress) []string {
	var problems []string
	bad := func(phase, msg string) {
		problems = append(problems, msg)
		p.Step(phase, "DOWN: "+msg)
	}
	job, _ := env.launcher.Status(c.TunnelLabel())
	st, stErr := readTunnelState(c.file("tunnel.state"))
	switch {
	case !job.Loaded:
		bad("tunnel", "launchd job "+c.TunnelLabel()+" not loaded (run olp ui start)")
	case stErr != nil:
		bad("tunnel", "supervisor has written no tunnel.state yet")
	case st.Phase != "up" || !olpUIPortOpen(c.TunnelPort):
		bad("tunnel", fmt.Sprintf("supervisor pid %d phase %s, 127.0.0.1:%d not listening; %s", st.SupervisorPID, st.Phase, c.TunnelPort, st.lastRestart()))
	default:
		p.Step("tunnel", fmt.Sprintf("up: 127.0.0.1:%d -> %s %s; supervisor pid %d, ssh pid %d, attempt %d up since %s; %s",
			c.TunnelPort, c.Target, c.OLPListen, st.SupervisorPID, st.SSHPID, st.Attempt, st.UpSince, st.lastRestart()))
	}
	tunnelHealth, d, err := olpUIHealthAt(env.client, c.TunnelBase())
	if err != nil {
		bad("health", fmt.Sprintf("OLP health through the tunnel %s: %v", c.TunnelBase(), err))
	} else {
		p.Step("health", fmt.Sprintf("through the tunnel: ok, OLP build %s started %s, maintenance notice %v (%d ms)",
			tunnelHealth.BuildID, tunnelHealth.StartedAt, tunnelHealth.Maintenance.Active, d.Milliseconds()))
		var targets struct {
			Targets []struct {
				CellID string `json:"cell_id"`
			} `json:"targets"`
			Selected *struct {
				CellID string `json:"cell_id"`
			} `json:"selected"`
		}
		if _, err := olpUIGetJSON(env.client, c.TunnelBase()+olpUIAPIPrefix+"/targets", &targets); err != nil {
			bad("cell", "OLP /targets: "+err.Error())
		} else {
			listed := false
			for _, t := range targets.Targets {
				listed = listed || t.CellID == c.Cell
			}
			selected := "not selected"
			if targets.Selected != nil && targets.Selected.CellID == c.Cell {
				selected = "selected"
			}
			if listed {
				p.Step("cell", fmt.Sprintf("%s listed by the OLP (%s; %d targets)", c.Cell, selected, len(targets.Targets)))
			} else {
				bad("cell", fmt.Sprintf("%s is not among the OLP's %d targets", c.Cell, len(targets.Targets)))
			}
		}
	}
	vjob, _ := env.launcher.Status(c.ViteLabel())
	if !vjob.Loaded {
		bad("vite", "launchd job "+c.ViteLabel()+" not loaded (run olp ui start)")
	} else if resp, err := env.client.Get(c.PageURL()); err != nil {
		bad("vite", fmt.Sprintf("job pid %d state %s; %s: %v", vjob.PID, vjob.State, c.PageURL(), err))
	} else {
		resp.Body.Close()
		if resp.StatusCode != http.StatusOK {
			bad("vite", fmt.Sprintf("%s answered HTTP %d", c.PageURL(), resp.StatusCode))
		} else {
			p.Step("vite", fmt.Sprintf("up: pid %d serves %s", vjob.PID, c.PageURL()))
			viteHealth, d, err := olpUIHealthAt(env.client, c.ViteBase())
			switch {
			case err != nil:
				bad("health", fmt.Sprintf("OLP health through Vite %s: %v", c.ViteBase(), err))
			case tunnelHealth.BuildID != "" && viteHealth.BuildID != tunnelHealth.BuildID:
				bad("health", fmt.Sprintf("Vite proxies to OLP build %s, the tunnel reaches %s: Vite is not on this tunnel", viteHealth.BuildID, tunnelHealth.BuildID))
			default:
				p.Step("health", fmt.Sprintf("through Vite: ok, OLP build %s (%d ms)", viteHealth.BuildID, d.Milliseconds()))
			}
		}
	}
	return problems
}

// --- command ------------------------------------------------------------------------

type olpUIFlags struct {
	common     cellDeployCommonFlags
	port       int
	tunnelPort int
	wait       time.Duration
	force      bool
	lines      int
	follow     bool
}

func runOLPUI(args []string, stdout, stderr io.Writer) error {
	action, rest := splitAction(olpUIActions, args)
	if action == "" {
		if len(args) == 0 || isHelp(args[0]) {
			fmt.Fprint(stdout, olpUIUsage)
			return nil
		}
		return fmt.Errorf("olp ui needs one of %s\n\n%s", strings.Join(olpUIActions, "|"), olpUIUsage)
	}
	var f olpUIFlags
	ctx, restore, err := cellDeploySetup("olp ui "+action, rest, stderr, func(fs *flag.FlagSet) {
		fs.IntVar(&f.port, "port", 0, fmt.Sprintf("Vite port on 127.0.0.1 (start/restart; default %d, else the last start's)", olpUIDefaultPort))
		fs.IntVar(&f.tunnelPort, "tunnel-port", 0, fmt.Sprintf("local tunnel port on 127.0.0.1 (start/restart; default %d, else the last start's)", olpUIDefaultTunnelPort))
		fs.DurationVar(&f.wait, "wait", 60*time.Second, "start/restart: how long to wait for the tunnel and Vite to answer")
		fs.BoolVar(&f.force, "force", false, "stop/restart/start: replace or stop jobs even while the OLP reports the robot armed or leased")
		fs.IntVar(&f.lines, "lines", 40, "logs: lines of each log")
		fs.BoolVar(&f.follow, "follow", false, "logs: keep following (tail -F)")
	}, &f.common, true, "--port", "--tunnel-port", "--wait", "--force", "--lines", "--follow")
	if err != nil {
		return err
	}
	defer restore()
	env := olpUINewEnv()
	c, err := olpUIResolveConfig(ctx, f, action, env)
	if err != nil {
		return err
	}
	switch action {
	case "start":
		return olpUIStart(env, c, f, stdout, false)
	case "restart":
		return olpUIStart(env, c, f, stdout, true)
	case "stop":
		return olpUIStop(env, c, f, stdout)
	case "status":
		return olpUIStatus(env, c, f, stdout)
	default:
		return olpUILogs(c, f, stdout, stderr)
	}
}

// olpUIResolveConfig builds the configuration: from the manifest and flags for
// start/restart, from the last start's ui.json for the others.
func olpUIResolveConfig(ctx cellDeployContext, f olpUIFlags, action string, env olpUIEnv) (olpUIConfig, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return olpUIConfig{}, err
	}
	stateDir := filepath.Join(home, ".rosie", "olp-ui", ctx.CellID)
	var saved olpUIConfig
	haveSaved := false
	if raw, err := os.ReadFile(filepath.Join(stateDir, "ui.json")); err == nil && json.Unmarshal(raw, &saved) == nil {
		haveSaved = true
	}
	if action != "start" && action != "restart" {
		if !haveSaved {
			return olpUIConfig{}, cellDeployExit(cellDeployExitFailed, "olp ui %s: %s was never started on this Mac (no %s); run: go run . olp ui start --manifest %s",
				action, ctx.CellID, filepath.Join(stateDir, "ui.json"), ctx.Manifest)
		}
		return saved, nil
	}
	if env.goos != "darwin" && env.goos != "linux" {
		return olpUIConfig{}, fmt.Errorf("olp ui keeps its processes alive with launchd (macOS) or systemd --user (Linux, WSL with systemd=true); this is %s", env.goos)
	}
	pick := func(flagValue, savedValue, def int) int {
		switch {
		case flagValue != 0:
			return flagValue
		case haveSaved && savedValue != 0:
			return savedValue
		}
		return def
	}
	c := olpUIConfig{Cell: ctx.CellID, Manifest: ctx.Manifest, Transport: f.common.Transport, OLPListen: ctx.Deploy.OLP.Listen,
		Port: pick(f.port, saved.Port, olpUIDefaultPort), TunnelPort: pick(f.tunnelPort, saved.TunnelPort, olpUIDefaultTunnelPort), StateDir: stateDir}
	for name, port := range map[string]int{"--port": c.Port, "--tunnel-port": c.TunnelPort} {
		if port < 1024 || port > 65535 {
			return olpUIConfig{}, fmt.Errorf("%s must be 1024..65535, got %d", name, port)
		}
	}
	if c.Port == c.TunnelPort {
		return olpUIConfig{}, fmt.Errorf("--port and --tunnel-port must differ")
	}
	if c.Transport == "tailscale" {
		c.Target = remoteTarget(ctx.RelayUser, ctx.RelayPeer)
	} else {
		c.Target = remoteTarget(ctx.RelayUser, ctx.RelayHost)
	}
	repo, err := repoRoot()
	if err != nil {
		return olpUIConfig{}, err
	}
	c.UIDir = filepath.Join(repo, "offline-programming", "v1", "ui")
	if _, err := os.Stat(filepath.Join(c.UIDir, "package.json")); err != nil {
		return olpUIConfig{}, fmt.Errorf("no UI at %s: %v", c.UIDir, err)
	}
	bin := "tailscale"
	if c.Transport == "ssh" {
		bin = "ssh"
	}
	if c.TunnelBin, err = env.lookPath(bin); err != nil {
		return olpUIConfig{}, fmt.Errorf("--transport %s needs %s on PATH: %v", c.Transport, bin, err)
	}
	if c.NPM, err = env.lookPath("npm"); err != nil {
		return olpUIConfig{}, fmt.Errorf("npm is not on PATH (export PATH=\"$HOME/.rosie/node-22.13.1/bin:$PATH\" or install Node): %v", err)
	}
	var dirs []string
	for _, d := range []string{filepath.Dir(c.NPM), filepath.Dir(c.TunnelBin), "/usr/bin", "/bin", "/usr/sbin", "/sbin"} {
		if !containsString(dirs, d) {
			dirs = append(dirs, d)
		}
	}
	c.PathEnv = strings.Join(dirs, ":")
	return c, nil
}

// olpUIWriteIfChanged writes content and reports whether the file changed.
func olpUIWriteIfChanged(p, content string, mode os.FileMode) (bool, error) {
	if old, err := os.ReadFile(p); err == nil && string(old) == content {
		return false, nil
	}
	tmp := p + ".tmp"
	if err := os.WriteFile(tmp, []byte(content), mode); err != nil {
		return false, err
	}
	return true, os.Rename(tmp, p)
}

func olpUIRotate(p string) {
	if info, err := os.Stat(p); err == nil && info.Size() > tunnelLogMaxBytes {
		_ = os.Rename(p, p+".1")
	}
}

// olpUIWait polls ok until it answers nil or the deadline passes.
func olpUIWait(env olpUIEnv, limit time.Duration, ok func() error) error {
	deadline := time.Now().Add(limit)
	for {
		err := ok()
		if err == nil || time.Now().After(deadline) {
			return err
		}
		time.Sleep(env.poll)
	}
}

func olpUIStart(env olpUIEnv, c olpUIConfig, f olpUIFlags, stdout io.Writer, restart bool) error {
	tunnelScript := tunnelSupervisorScript(olpUITunnelSpec(c))
	tunnelPlist, vitePlist := olpUIJobFile(env, olpUITunnelJob(c)), olpUIJobFile(env, olpUIViteJob(c))
	if f.common.DryRun {
		fmt.Fprintf(stdout, "olp ui start cell=%s (dry run: nothing started or contacted)\nui %s via tunnel 127.0.0.1:%d -> %s %s (%s)\n# %s\n%s# %s\n%s# %s\n%s",
			c.Cell, c.PageURL(), c.TunnelPort, c.Target, c.OLPListen, c.Transport,
			c.file("tunnel.sh"), tunnelScript, c.file(c.TunnelLabel()+olpUIJobExt(env)), tunnelPlist, c.file(c.ViteLabel()+olpUIJobExt(env)), vitePlist)
		return nil
	}
	command := "olp ui start"
	if restart {
		command = "olp ui restart"
	}
	run, err := newDeployJournalRun(c.Cell, command, stdout)
	if err != nil {
		return err
	}
	p := run.p
	p.Step("ui", fmt.Sprintf("cell=%s ui %s; Vite -> tunnel 127.0.0.1:%d -> %s %s (%s); state %s",
		c.Cell, c.PageURL(), c.TunnelPort, c.Target, c.OLPListen, c.Transport, c.StateDir))
	fail := func(code int, format string, args ...any) error {
		e := cellDeployExit(code, format, args...)
		result := "failed"
		if code == cellDeployExitRefused {
			result = "refused"
		}
		run.finish(result, code, e.Error())
		return e
	}
	if restart {
		if err := olpUIStopJobs(env, c, f, run); err != nil {
			var exit *rtCoreConfigExit
			if errors.As(err, &exit) {
				return fail(exit.Code, "%s", exit.Message)
			}
			return fail(cellDeployExitFailed, "%v", err)
		}
	}
	if err := os.MkdirAll(c.StateDir, 0o700); err != nil {
		return fail(cellDeployExitFailed, "%v", err)
	}
	saved, _ := json.MarshalIndent(c, "", "  ")
	if _, err := olpUIWriteIfChanged(c.file("ui.json"), string(saved)+"\n", 0o600); err != nil {
		return fail(cellDeployExitFailed, "%v", err)
	}
	inUseChecked := false
	replaceAllowed := func(what string) error {
		if f.force || inUseChecked {
			return nil
		}
		inUse, why := olpUIRobotInUse(env.client, c.TunnelBase())
		p.Step("check", why)
		if inUse {
			return cellDeployExit(cellDeployExitRefused, "%s needs replacing but %s; the browser's heartbeat would lapse. Finish or disarm first, or pass --force", what, why)
		}
		inUseChecked = true
		return nil
	}

	// Tunnel.
	run.begin("tunnel")
	scriptChanged, err := olpUIWriteIfChanged(c.file("tunnel.sh"), tunnelScript, 0o700)
	if err != nil {
		return fail(cellDeployExitFailed, "%v", err)
	}
	plistPath := c.file(c.TunnelLabel() + olpUIJobExt(env))
	plistChanged, err := olpUIWriteIfChanged(plistPath, tunnelPlist, 0o600)
	if err != nil {
		return fail(cellDeployExitFailed, "%v", err)
	}
	job, _ := env.launcher.Status(c.TunnelLabel())
	switch {
	case job.Loaded && (scriptChanged || plistChanged):
		if err := replaceAllowed("the tunnel job"); err != nil {
			return fail(cellDeployExitRefused, "%v", err)
		}
		if err := env.launcher.Unload(c.TunnelLabel()); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		if err := env.launcher.Load(c.TunnelLabel(), plistPath); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		p.Step("tunnel", "job re-rendered (ports, transport or tool version changed): replaced")
	case job.Loaded && olpUIPortOpen(c.TunnelPort):
		st, _ := readTunnelState(c.file("tunnel.state"))
		p.Step("tunnel", fmt.Sprintf("already up (supervisor pid %d, ssh pid %d); left alone", st.SupervisorPID, st.SSHPID))
	case job.Loaded:
		st, _ := readTunnelState(c.file("tunnel.state"))
		if err := env.launcher.Restart(c.TunnelLabel()); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		p.Step("tunnel", fmt.Sprintf("job loaded but 127.0.0.1:%d not listening (%s): restarted now instead of waiting out the backoff", c.TunnelPort, st.lastRestart()))
	default:
		if olpUIPortOpen(c.TunnelPort) {
			return fail(cellDeployExitRefused, "127.0.0.1:%d is held by %s, which this command did not start; stop it (the hand setup: /tmp/olp-local/stop.sh) or pass --tunnel-port", c.TunnelPort, olpUIPortHolder(c.TunnelPort))
		}
		if err := env.launcher.Load(c.TunnelLabel(), plistPath); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		p.Step("tunnel", "job "+c.TunnelLabel()+" started")
	}
	err = olpUIWait(env, f.wait, func() error {
		_, _, err := olpUIHealthAt(env.client, c.TunnelBase())
		return err
	})
	run.end("tunnel", err)
	if err != nil {
		st, _ := readTunnelState(c.file("tunnel.state"))
		p.Step("tunnel", fmt.Sprintf("no OLP health through the tunnel after %s: %v; %s (the supervisor keeps retrying)", f.wait, err, st.lastRestart()))
	} else {
		p.Step("tunnel", "OLP health ok through the tunnel")
	}

	// Vite.
	run.begin("vite")
	vitePath := c.file(c.ViteLabel() + olpUIJobExt(env))
	viteChanged, err := olpUIWriteIfChanged(vitePath, vitePlist, 0o600)
	if err != nil {
		return fail(cellDeployExitFailed, "%v", err)
	}
	vjob, _ := env.launcher.Status(c.ViteLabel())
	needDeps := func() error {
		ok, why := olpUINodeDepsCurrent(c.UIDir)
		if ok {
			p.Step("deps", why)
			return nil
		}
		p.Step("deps", why+": npm ci")
		run.begin("npm ci")
		err := env.npmCI(c)
		run.end("npm ci", err)
		if err == nil {
			p.Step("deps", "npm ci done")
		}
		return err
	}
	switch {
	case vjob.Loaded && viteChanged:
		if err := replaceAllowed("the Vite job"); err != nil {
			return fail(cellDeployExitRefused, "%v", err)
		}
		if err := env.launcher.Unload(c.ViteLabel()); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		if err := needDeps(); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		olpUIRotate(c.file("vite.log"))
		if err := env.launcher.Load(c.ViteLabel(), vitePath); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		p.Step("vite", "job re-rendered: replaced")
	case vjob.Loaded && olpUIPortOpen(c.Port):
		if ok, why := olpUINodeDepsCurrent(c.UIDir); !ok {
			p.Step("deps", why+"; Vite left running: `olp ui restart` reinstalls")
		}
		p.Step("vite", fmt.Sprintf("already up (pid %d); left alone", vjob.PID))
	case vjob.Loaded:
		if err := needDeps(); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		if err := env.launcher.Restart(c.ViteLabel()); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		p.Step("vite", fmt.Sprintf("job loaded but 127.0.0.1:%d not serving: restarted", c.Port))
	default:
		if olpUIPortOpen(c.Port) {
			return fail(cellDeployExitRefused, "127.0.0.1:%d is held by %s, which this command did not start; stop it (the hand setup: /tmp/olp-local/stop.sh) or pass --port", c.Port, olpUIPortHolder(c.Port))
		}
		if err := needDeps(); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		olpUIRotate(c.file("vite.log"))
		if err := env.launcher.Load(c.ViteLabel(), vitePath); err != nil {
			return fail(cellDeployExitFailed, "%v", err)
		}
		p.Step("vite", "job "+c.ViteLabel()+" started")
	}
	err = olpUIWait(env, f.wait, func() error {
		resp, err := env.client.Get(c.PageURL())
		if err != nil {
			return err
		}
		resp.Body.Close()
		if resp.StatusCode != http.StatusOK {
			return fmt.Errorf("HTTP %d", resp.StatusCode)
		}
		return nil
	})
	run.end("vite", err)

	problems := olpUIReport(env, c, p)
	if len(problems) > 0 {
		return fail(cellDeployExitFailed, "olp ui %s: unhealthy (processes left running; olp ui status / olp ui logs):\n  - %s", c.Cell, strings.Join(problems, "\n  - "))
	}
	p.Step("done", "open "+c.PageURL())
	run.finish("ok", 0, c.PageURL())
	return nil
}

// olpUIStopJobs unloads both jobs and waits for their ports to close.
func olpUIStopJobs(env olpUIEnv, c olpUIConfig, f olpUIFlags, run *cellDeployRun) error {
	p := run.p
	if f.force {
		p.Step("check", "--force: not reading the robot's state")
	} else {
		inUse, why := olpUIRobotInUse(env.client, c.TunnelBase())
		p.Step("check", why)
		if inUse {
			return cellDeployExit(cellDeployExitRefused, "olp ui stop refused (nothing was changed): %s; stopping the UI would lapse the browser's heartbeat. Finish or disarm first, or pass --force", why)
		}
	}
	run.begin("stop")
	for _, label := range []string{c.ViteLabel(), c.TunnelLabel()} {
		if err := env.launcher.Unload(label); err != nil {
			run.end("stop", err)
			return err
		}
	}
	err := olpUIWait(env, 10*time.Second, func() error {
		for _, port := range []int{c.Port, c.TunnelPort} {
			if olpUIPortOpen(port) {
				return fmt.Errorf("127.0.0.1:%d still held by %s", port, olpUIPortHolder(port))
			}
		}
		return nil
	})
	run.end("stop", err)
	if err != nil {
		return err
	}
	p.Step("stop", fmt.Sprintf("Vite and tunnel stopped; 127.0.0.1:%d and :%d free (logs kept in %s)", c.Port, c.TunnelPort, c.StateDir))
	return nil
}

func olpUIStop(env olpUIEnv, c olpUIConfig, f olpUIFlags, stdout io.Writer) error {
	if f.common.DryRun {
		fmt.Fprintf(stdout, "olp ui stop cell=%s (dry run: nothing contacted)\nwould read %s%s/dense-execution/status, then launchctl bootout %s and %s\n",
			c.Cell, c.TunnelBase(), olpUIAPIPrefix, c.ViteLabel(), c.TunnelLabel())
		return nil
	}
	run, err := newDeployJournalRun(c.Cell, "olp ui stop", stdout)
	if err != nil {
		return err
	}
	if err := olpUIStopJobs(env, c, f, run); err != nil {
		code := cellDeployExitFailed
		var exit *rtCoreConfigExit
		if errors.As(err, &exit) {
			code = exit.Code
		}
		result := "failed"
		if code == cellDeployExitRefused {
			result = "refused"
		}
		run.finish(result, code, err.Error())
		if exit != nil {
			return err
		}
		return cellDeployExit(code, "%v", err)
	}
	run.finish("ok", 0, "stopped")
	return nil
}

func olpUIStatus(env olpUIEnv, c olpUIConfig, f olpUIFlags, stdout io.Writer) error {
	if f.common.DryRun {
		fmt.Fprintf(stdout, "olp ui status cell=%s (read-only; dry run: nothing contacted)\nwould read launchd %s and %s, %s, %s%s/health and /targets, %s and %s%s/health\n",
			c.Cell, c.TunnelLabel(), c.ViteLabel(), c.file("tunnel.state"), c.TunnelBase(), olpUIAPIPrefix, c.PageURL(), c.ViteBase(), olpUIAPIPrefix)
		return nil
	}
	p := newRTCoreProgress(stdout, nil)
	p.Step("ui", fmt.Sprintf("cell=%s ui %s; tunnel 127.0.0.1:%d -> %s %s (%s)", c.Cell, c.PageURL(), c.TunnelPort, c.Target, c.OLPListen, c.Transport))
	if problems := olpUIReport(env, c, p); len(problems) > 0 {
		return cellDeployExit(cellDeployExitFailed, "olp ui %s unhealthy:\n  - %s", c.Cell, strings.Join(problems, "\n  - "))
	}
	p.Step("status", "healthy")
	return nil
}

func olpUILogs(c olpUIConfig, f olpUIFlags, stdout, stderr io.Writer) error {
	files := []string{c.file("tunnel.log"), c.file("vite.log")}
	if f.common.DryRun {
		fmt.Fprintf(stdout, "olp ui logs cell=%s (dry run)\nwould tail %s\n", c.Cell, strings.Join(files, " "))
		return nil
	}
	if f.lines < 1 {
		return fmt.Errorf("--lines must be at least 1")
	}
	args := []string{"-n", strconv.Itoa(f.lines)}
	if f.follow {
		args = append(args, "-F")
	}
	cmd := exec.Command("tail", append(args, files...)...)
	cmd.Stdout, cmd.Stderr = stdout, stderr
	return cmd.Run()
}
