// Maintenance notice: an update is replacing part of the cell (the controller,
// its configuration or this server). The deploy tooling writes one JSON file on
// the OLP host before it waits for the cell to go idle and removes it when it
// finishes, including on failure, and the same notice on the servo host, which
// the selected cell's rt-control serves (cell_maintenance.go). This server only
// reads them: GET .../maintenance merges both and tells every client (browser,
// Steam Deck) to show the blocking overlay, and while either is active every
// request that could start motion or take authority is refused with 503
// "maintenance". Protective requests (Stop, Pause, Disarm,
// release, heartbeat, ending or continuing a hold that is already running)
// always pass: a notice never disarms, stops or drops a session on its own,
// because that would be motion policy decided by a file.
package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"runtime/debug"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	maintenanceAPIPath = "/api/offline-programming/v1/maintenance"
	// The file the deploy tooling writes (steamdeck/real/v4 rt_core_maintenance.go
	// uses the same path). Outside any OLP deployment directory, so a notice
	// survives the OLP binary and tree being replaced underneath it, and outside
	// /run, so a host reboot mid-deploy does not clear it silently.
	defaultMaintenanceFile   = "/var/lib/rosie-olp/maintenance.json"
	maintenanceFileSchema    = "rosie.olp.maintenance.v1"
	maintenanceViewSchema    = "offline-programming.maintenance.v1"
	maintenanceFileMaxBytes  = 64 << 10
	maintenanceTextMaxLength = 400
	// A notice whose expected end passed this long ago is still active but
	// flagged stale: the deploy that wrote it probably died without clearing
	// it. Nothing clears it automatically; an operator checks and clears it.
	// Owner's choice in the request (2026-10-07), not measured.
	defaultMaintenanceStaleAfter = 2 * time.Hour
)

// maintenanceNoticeFile is the on-disk notice. Every field but started_at is
// optional; a present file with no "active" key is active.
type maintenanceNoticeFile struct {
	Schema    string `json:"schema,omitempty"`
	Active    *bool  `json:"active,omitempty"`
	Reason    string `json:"reason,omitempty"`
	Phase     string `json:"phase,omitempty"`
	StartedAt string `json:"started_at"`
	ETA       string `json:"eta,omitempty"`
	SetBy     string `json:"set_by,omitempty"`
	Release   string `json:"release,omitempty"`
	RunID     string `json:"run_id,omitempty"`
	UpdatedAt string `json:"updated_at,omitempty"`
	// Progress is the run's steps, gate suites and fraction (steamdeck/real/v4
	// notice_progress.go), passed to clients as written.
	Progress json.RawMessage `json:"progress,omitempty"`
}

// maintenanceProgressMaxBytes bounds the progress passed through; a larger
// or non-object one is dropped (the notice itself still counts).
const maintenanceProgressMaxBytes = 16 << 10

// MaintenanceView is what clients read. Times are RFC 3339 UTC; `now` is this
// server's clock so a client with a skewed clock (a Deck) can still count down.
type MaintenanceView struct {
	Schema    string `json:"schema"`
	Active    bool   `json:"active"`
	Reason    string `json:"reason,omitempty"`
	Phase     string `json:"phase,omitempty"`
	StartedAt string `json:"started_at,omitempty"`
	ETA       string `json:"eta,omitempty"`
	SetBy     string `json:"set_by,omitempty"`
	Release   string `json:"release,omitempty"`
	RunID     string `json:"run_id,omitempty"`
	UpdatedAt string `json:"updated_at,omitempty"`
	// Progress: this host's notice's progress, when it has one. Old clients
	// ignore it and read phase and eta.
	Progress json.RawMessage `json:"progress,omitempty"`
	Now      string          `json:"now"`
	// Overrun: the expected end has passed. Stale: it passed more than the
	// stale threshold ago (or, with no ETA, the notice is that old).
	Overrun bool `json:"overrun"`
	Stale   bool `json:"stale"`
	// Malformed: the file exists but could not be read as a notice. It is
	// still active: an unreadable notice fails closed.
	Malformed bool              `json:"malformed"`
	Error     string            `json:"error,omitempty"`
	Server    MaintenanceServer `json:"server"`
	// Who is connected (maintenance_clients.go), whether or not a notice is
	// active: the deploy tool reads it while it waits for the cell to go idle.
	Clients []MaintenanceClient `json:"clients"`
	Machine *MaintenanceMachine `json:"machine,omitempty"`
	// Cell: the selected cell's own notice from its rt-control, whether or not
	// it is active. The notice fields above already include it.
	Cell *MaintenanceCell `json:"cell,omitempty"`
	// Control: who is using the robot, as the requester sees it (control.go),
	// so clients learn of another controller from the poll they already run.
	Control *ControlView `json:"control,omitempty"`
}

// MaintenanceServer identifies this server process, so a client can tell
// after an update whether the server it talks to was replaced.
type MaintenanceServer struct {
	BuildID   string `json:"build_id"`
	StartedAt string `json:"started_at"`
}

// MaintenanceSource reads the notice file. One os.Stat per call; the file is
// parsed again only when its size or modification time changes.
type MaintenanceSource struct {
	path       string
	staleAfter time.Duration
	now        func() time.Time

	mu      sync.Mutex
	known   bool
	size    int64
	modTime time.Time
	notice  *maintenanceNoticeFile
	err     error
}

func NewMaintenanceSource(path string, staleAfter time.Duration) *MaintenanceSource {
	if staleAfter <= 0 {
		staleAfter = defaultMaintenanceStaleAfter
	}
	return &MaintenanceSource{path: path, staleAfter: staleAfter, now: time.Now}
}

// maintenanceSourceFromEnv: OFFLINE_PROGRAMMING_MAINTENANCE_FILE (default
// /var/lib/rosie-olp/maintenance.json) and
// OFFLINE_PROGRAMMING_MAINTENANCE_STALE_AFTER (a Go duration, default 2h).
func maintenanceSourceFromEnv() (*MaintenanceSource, error) {
	path := maintenanceFileFromEnv()
	staleAfter := defaultMaintenanceStaleAfter
	if raw := strings.TrimSpace(os.Getenv("OFFLINE_PROGRAMMING_MAINTENANCE_STALE_AFTER")); raw != "" {
		d, err := time.ParseDuration(raw)
		if err != nil || d <= 0 {
			return nil, fmt.Errorf("OFFLINE_PROGRAMMING_MAINTENANCE_STALE_AFTER must be a positive duration, got %q", raw)
		}
		staleAfter = d
	}
	return NewMaintenanceSource(path, staleAfter), nil
}

func maintenanceFileFromEnv() string {
	return envOrDefault("OFFLINE_PROGRAMMING_MAINTENANCE_FILE", defaultMaintenanceFile)
}

func (s *MaintenanceSource) Path() string {
	if s == nil {
		return ""
	}
	return s.path
}

// Current is the notice as of now. A nil source is never active.
func (s *MaintenanceSource) Current() MaintenanceView {
	if s == nil {
		return MaintenanceView{Schema: maintenanceViewSchema, Now: time.Now().UTC().Format(time.RFC3339), Server: maintenanceServerIdentity()}
	}
	now := s.now()
	notice, err := s.load()
	return maintenanceViewOf(notice, err, now, s.staleAfter)
}

func (s *MaintenanceSource) load() (*maintenanceNoticeFile, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	info, err := os.Stat(s.path)
	if errors.Is(err, os.ErrNotExist) {
		s.known, s.notice, s.err = false, nil, nil
		return nil, nil
	}
	if err != nil {
		s.known = false
		return nil, fmt.Errorf("notice file %s unreadable: %w", s.path, err)
	}
	if s.known && info.Size() == s.size && info.ModTime().Equal(s.modTime) {
		return s.notice, s.err
	}
	s.known, s.size, s.modTime = true, info.Size(), info.ModTime()
	s.notice, s.err = readMaintenanceNotice(s.path)
	return s.notice, s.err
}

func readMaintenanceNotice(path string) (*maintenanceNoticeFile, error) {
	file, err := os.Open(path)
	if errors.Is(err, os.ErrNotExist) {
		return nil, nil
	}
	if err != nil {
		return nil, fmt.Errorf("notice file %s unreadable: %w", path, err)
	}
	defer file.Close()
	raw, err := io.ReadAll(io.LimitReader(file, maintenanceFileMaxBytes+1))
	if err != nil {
		return nil, fmt.Errorf("notice file %s unreadable: %w", path, err)
	}
	if len(raw) > maintenanceFileMaxBytes {
		return nil, fmt.Errorf("notice file %s is larger than %d bytes", path, maintenanceFileMaxBytes)
	}
	var notice maintenanceNoticeFile
	if err := json.Unmarshal(raw, &notice); err != nil {
		return nil, fmt.Errorf("notice file %s is not a JSON notice: %v", path, err)
	}
	return &notice, nil
}

// maintenanceViewOf applies the contract: no file is inactive; active=false
// is inactive; anything unreadable is active and malformed; an overrun is
// reported and a long overrun is stale, but neither ends the notice.
func maintenanceViewOf(notice *maintenanceNoticeFile, readErr error, now time.Time, staleAfter time.Duration) MaintenanceView {
	view := MaintenanceView{Schema: maintenanceViewSchema, Now: now.UTC().Format(time.RFC3339), Server: maintenanceServerIdentity()}
	if readErr != nil {
		view.Active, view.Malformed, view.Error = true, true, readErr.Error()
		view.Reason = "Update in progress (the maintenance notice could not be read)"
		return view
	}
	if notice == nil {
		return view
	}
	if notice.Active != nil && !*notice.Active {
		return view
	}
	view.Active = true
	view.Reason = clipMaintenanceText(firstNonEmpty(strings.TrimSpace(notice.Reason), "Update in progress"))
	view.Phase = clipMaintenanceText(strings.TrimSpace(notice.Phase))
	view.SetBy = clipMaintenanceText(strings.TrimSpace(notice.SetBy))
	view.Release = clipMaintenanceText(strings.TrimSpace(notice.Release))
	view.RunID = clipMaintenanceText(strings.TrimSpace(notice.RunID))
	if raw := bytes.TrimSpace(notice.Progress); len(raw) > 0 && len(raw) <= maintenanceProgressMaxBytes && raw[0] == '{' {
		view.Progress = raw
	}
	var problems []string
	if notice.Schema != "" && notice.Schema != maintenanceFileSchema {
		problems = append(problems, fmt.Sprintf("schema %q is not %s", notice.Schema, maintenanceFileSchema))
	}
	started, err := time.Parse(time.RFC3339, strings.TrimSpace(notice.StartedAt))
	if err != nil {
		problems = append(problems, fmt.Sprintf("started_at %q is not RFC 3339", notice.StartedAt))
	} else {
		view.StartedAt = started.UTC().Format(time.RFC3339)
	}
	if raw := strings.TrimSpace(notice.UpdatedAt); raw != "" {
		if updated, err := time.Parse(time.RFC3339, raw); err == nil {
			view.UpdatedAt = updated.UTC().Format(time.RFC3339)
		}
	}
	if raw := strings.TrimSpace(notice.ETA); raw != "" {
		eta, err := time.Parse(time.RFC3339, raw)
		if err != nil {
			problems = append(problems, fmt.Sprintf("eta %q is not RFC 3339", notice.ETA))
		} else {
			view.ETA = eta.UTC().Format(time.RFC3339)
			view.Overrun = now.After(eta)
			view.Stale = now.Sub(eta) > staleAfter
		}
	} else if view.StartedAt != "" {
		view.Stale = now.Sub(started) > staleAfter
	}
	if len(problems) > 0 {
		view.Malformed = true
		view.Error = strings.Join(problems, "; ")
	}
	return view
}

func firstNonEmpty(values ...string) string {
	for _, value := range values {
		if value != "" {
			return value
		}
	}
	return ""
}

func clipMaintenanceText(text string) string {
	if len(text) <= maintenanceTextMaxLength {
		return text
	}
	return text[:maintenanceTextMaxLength-1] + "…"
}

var (
	maintenanceProcessStarted = time.Now().UTC()
	maintenanceBuildID        = sync.OnceValue(computeMaintenanceBuildID)
)

func maintenanceServerIdentity() MaintenanceServer {
	return MaintenanceServer{BuildID: maintenanceBuildID(), StartedAt: maintenanceProcessStarted.Format(time.RFC3339)}
}

// computeMaintenanceBuildID names the build this process runs: the VCS
// revision when the binary embeds one, else a hash of the executable (a
// deployed tree is a git archive, so it usually has no VCS stamp).
func computeMaintenanceBuildID() string {
	if info, ok := debug.ReadBuildInfo(); ok {
		revision, modified := "", false
		for _, setting := range info.Settings {
			switch setting.Key {
			case "vcs.revision":
				revision = setting.Value
			case "vcs.modified":
				modified = setting.Value == "true"
			}
		}
		if revision != "" {
			if len(revision) > 12 {
				revision = revision[:12]
			}
			if modified {
				revision += "-modified"
			}
			return "git:" + revision
		}
	}
	executable, err := os.Executable()
	if err != nil {
		return "unknown"
	}
	file, err := os.Open(executable)
	if err != nil {
		return "unknown"
	}
	defer file.Close()
	sum := sha256.New()
	if _, err := io.Copy(sum, file); err != nil {
		return "unknown"
	}
	return "sha256:" + hex.EncodeToString(sum.Sum(nil))[:12]
}

// maintenanceView is this host's notice merged with the selected cell's.
func (server *Server) maintenanceView(ctx context.Context) MaintenanceView {
	view := server.maintenance.Current()
	if server.cellMaintenance == nil {
		return view
	}
	now, staleAfter := time.Now(), defaultMaintenanceStaleAfter
	if server.maintenance != nil {
		now, staleAfter = server.maintenance.now(), server.maintenance.staleAfter
	}
	return mergeMaintenance(view, server.cellMaintenance.Current(ctx), now, staleAfter)
}

func (server *Server) maintenanceNotice(writer http.ResponseWriter, request *http.Request) {
	writer.Header().Set("Cache-Control", "no-store")
	view := server.maintenanceView(request.Context())
	status := server.denseStatus(request.Context())
	view.Machine = maintenanceMachine(status)
	you := controlClientOf(request)
	if server.control != nil {
		control := server.controlView(request.Context(), you, controlPostureOf(status))
		view.Control = &control
	}
	controller := ""
	if holder, ok := server.control.snapshot(); ok && view.Control != nil && view.Control.Owner != nil {
		controller = holder.client.key
	}
	view.Clients = server.clients.list(controller, you.key)
	writeJSON(writer, http.StatusOK, view)
}

// maintenanceGate refuses robot actions while a notice is active. Reads,
// planning, authoring and the local simulator are not robot actions and pass.
func (server *Server) maintenanceGate(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if (server.maintenance == nil && server.cellMaintenance == nil) || r.Method == http.MethodGet || r.Method == http.MethodHead || r.Method == http.MethodOptions {
			next.ServeHTTP(w, r)
			return
		}
		action, robot := maintenanceRobotAction(r.URL.Path)
		if !robot {
			next.ServeHTTP(w, r)
			return
		}
		// The cell's notice belongs to the selected cell, not to the one a
		// select names; only this host's notice refuses a select.
		view := server.maintenance.Current()
		if action != "select" {
			view = server.maintenanceView(r.Context())
		}
		if !view.Active {
			next.ServeHTTP(w, r)
			return
		}
		if action == "arm" || action == "target" || action == "io/test" {
			raw, err := io.ReadAll(http.MaxBytesReader(w, r.Body, maxDenseRequestBytes))
			if err != nil {
				writeDenseError(w, http.StatusBadRequest, "dense_request_invalid", "request body is invalid")
				return
			}
			r.Body = io.NopCloser(bytes.NewReader(raw))
			if maintenanceProtectiveBody(action, raw) {
				next.ServeHTTP(w, r)
				return
			}
		} else if action == "" {
			next.ServeHTTP(w, r)
			return
		}
		now := time.Now()
		if server.maintenance != nil {
			now = server.maintenance.now()
		}
		detail := maintenanceRefusalText(view, now)
		log.Printf("maintenance: refused %s %s from %s: %s", r.Method, r.URL.Path, r.RemoteAddr, detail)
		if retry := maintenanceRetryAfter(view, now); retry > 0 {
			w.Header().Set("Retry-After", strconv.Itoa(retry))
		}
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"error": "maintenance", "detail": detail, "maintenance": view})
	})
}

// maintenanceRobotAction classifies a mutating request. robot=false: not a
// robot route, never refused. action "": a protective robot route that always
// passes. "arm" and "target" pass only for their protective bodies (disarm,
// disconnect). Any other action is refused while a notice is active,
// including dense-execution routes added later (deny by default).
func maintenanceRobotAction(path string) (action string, robot bool) {
	switch {
	case path == "/api/offline-programming/v1/targets/select":
		return "select", true
	case path == executionPath+"/start", path == executionPath+"/review", path == executionGatewayPath+"/review":
		return "execution", true
	case strings.HasPrefix(path, executionPath+"/") && strings.HasSuffix(path, "/stop"):
		return "", true
	case strings.HasPrefix(path, denseExecutionPath+"/"):
	default:
		return "", false
	}
	switch rest := strings.TrimPrefix(path, denseExecutionPath); rest {
	// Stop also disarms and releases the grant; Pause holds a playing program.
	case "/stop", "/pause", "/heartbeat":
		return "", true
	// Finish cycle only ends a running loop at the program start (the
	// controller admits only the client that started it).
	case "/run-program/finish":
		return "", true
	// Ending a hold, or continuing one that is already running: "update" and
	// "intent" are refused by the controller unless that hold is live, so they
	// cannot start motion; refusing them would cut a running jog off through
	// its input deadman instead of the operator's release.
	case "/jog/end", "/jog/stop", "/jog/update", "/cartesian/stop", "/cartesian/halt", "/cartesian/intent", "/trace/update", "/trace/end":
		return "", true
	case "/arm":
		return "arm", true
	case "/target":
		return "target", true
	default:
		return strings.TrimPrefix(rest, "/"), true
	}
}

// maintenanceProtectiveBody: {"armed": false} disarms and {"cell": ""}
// disconnects (Stop, then release); both make the cell safer to update.
func maintenanceProtectiveBody(action string, raw []byte) bool {
	var input struct {
		End   bool    `json:"end"`
		Armed *bool   `json:"armed"`
		Cell  *string `json:"cell"`
	}
	if json.Unmarshal(raw, &input) != nil {
		return false
	}
	switch action {
	case "io/test":
		return input.End
	case "arm":
		return input.Armed != nil && !*input.Armed
	case "target":
		return input.Cell != nil && strings.TrimSpace(*input.Cell) == ""
	}
	return false
}

func maintenanceRefusalText(view MaintenanceView, now time.Time) string {
	text := "Update in progress: " + view.Reason
	if view.Phase != "" {
		text += " (" + view.Phase + ")"
	}
	if eta, err := time.Parse(time.RFC3339, view.ETA); err == nil {
		if now.After(eta) {
			text += fmt.Sprintf("; expected to finish at %s, overrunning by %s", view.ETA, maintenanceDuration(now.Sub(eta)))
		} else {
			text += fmt.Sprintf("; expected to finish at %s (in %s)", view.ETA, maintenanceDuration(eta.Sub(now)))
		}
	} else {
		text += "; no expected finish time"
	}
	return text + ". Robot actions are refused until it ends; Stop, Disarm and Disconnect still work."
}

func maintenanceRetryAfter(view MaintenanceView, now time.Time) int {
	eta, err := time.Parse(time.RFC3339, view.ETA)
	if err != nil || !eta.After(now) {
		return 30
	}
	return max(5, int(eta.Sub(now).Seconds()))
}

func maintenanceDuration(d time.Duration) string {
	d = d.Round(time.Second)
	if d < time.Minute {
		return fmt.Sprintf("%ds", int(d.Seconds()))
	}
	if d < time.Hour {
		return fmt.Sprintf("%dm%02ds", int(d.Minutes()), int(d.Seconds())%60)
	}
	return fmt.Sprintf("%dh%02dm", int(d.Hours()), int(d.Minutes())%60)
}

// runMaintenance is `offline-programming maintenance set|clear|show`, for an
// operator on the OLP host. It writes the same file the deploy tooling writes,
// so it works while no server runs. Writing under /var/lib needs root.
func runMaintenance(args []string, stdout, stderr io.Writer) error {
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, maintenanceUsage)
		return nil
	}
	flags := flag.NewFlagSet("maintenance "+args[0], flag.ContinueOnError)
	flags.SetOutput(stderr)
	path := flags.String("file", maintenanceFileFromEnv(), "notice file")
	switch args[0] {
	case "set":
		eta := flags.String("eta", "", "expected end: a duration from now (20m) or an RFC 3339 time")
		reason := flags.String("reason", "", "what is being updated (shown to operators)")
		phase := flags.String("phase", "", "the current step (shown to operators)")
		setBy := flags.String("set-by", "", "who or what set the notice (default user@host)")
		release := flags.String("release", "", "the release or commit being installed")
		runID := flags.String("run-id", "", "the deploy run that owns the notice")
		if err := flags.Parse(args[1:]); err != nil {
			return err
		}
		if flags.NArg() != 0 || strings.TrimSpace(*reason) == "" {
			return errors.New("maintenance set requires --reason and accepts no positional arguments")
		}
		now := time.Now().UTC()
		notice := maintenanceNoticeFile{Schema: maintenanceFileSchema, Reason: *reason, Phase: *phase, StartedAt: now.Format(time.RFC3339),
			SetBy: firstNonEmpty(*setBy, maintenanceDefaultSetBy()), Release: *release, RunID: *runID, UpdatedAt: now.Format(time.RFC3339)}
		active := true
		notice.Active = &active
		if previous, err := readMaintenanceNotice(*path); err == nil && previous != nil && (previous.Active == nil || *previous.Active) {
			// Updating a live notice keeps when it began, so elapsed time stays true.
			if _, err := time.Parse(time.RFC3339, previous.StartedAt); err == nil {
				notice.StartedAt = previous.StartedAt
			}
		}
		if strings.TrimSpace(*eta) != "" {
			resolved, err := maintenanceParseETA(*eta, now)
			if err != nil {
				return err
			}
			notice.ETA = resolved.Format(time.RFC3339)
		}
		if err := writeMaintenanceNotice(*path, notice); err != nil {
			return err
		}
		fmt.Fprintf(stdout, "maintenance notice set: %s\n", *path)
		return printMaintenanceView(stdout, *path)
	case "clear":
		runID := flags.String("run-id", "", "clear only if the notice belongs to this deploy run")
		if err := flags.Parse(args[1:]); err != nil {
			return err
		}
		if flags.NArg() != 0 {
			return errors.New("maintenance clear accepts no positional arguments")
		}
		if *runID != "" {
			if notice, err := readMaintenanceNotice(*path); err == nil && notice != nil && notice.RunID != *runID {
				fmt.Fprintf(stdout, "maintenance notice left in place: it belongs to run %q, not %q\n", notice.RunID, *runID)
				return nil
			}
		}
		if err := os.Remove(*path); err != nil && !errors.Is(err, os.ErrNotExist) {
			return err
		}
		fmt.Fprintf(stdout, "maintenance notice cleared: %s\n", *path)
		return nil
	case "show":
		if err := flags.Parse(args[1:]); err != nil {
			return err
		}
		return printMaintenanceView(stdout, *path)
	default:
		return fmt.Errorf("unknown maintenance command %q\n\n%s", args[0], maintenanceUsage)
	}
}

const maintenanceUsage = `usage: offline-programming maintenance <set|clear|show> [flags]

  set    --reason TEXT [--eta 20m|RFC3339] [--phase TEXT] [--release REF] [--set-by WHO] [--run-id ID]
  clear  [--run-id ID]   (with --run-id: only that run's notice)
  show

All take --file (default $OFFLINE_PROGRAMMING_MAINTENANCE_FILE or ` + defaultMaintenanceFile + `).
While a notice is active the OLP server refuses robot actions with 503
"maintenance" and every client shows a blocking overlay. Nothing clears a
notice on its own: a stale one stays until it is cleared.
`

func maintenanceParseETA(raw string, now time.Time) (time.Time, error) {
	raw = strings.TrimSpace(raw)
	if d, err := time.ParseDuration(raw); err == nil {
		if d <= 0 {
			return time.Time{}, fmt.Errorf("--eta duration must be positive, got %q", raw)
		}
		return now.Add(d).Truncate(time.Second), nil
	}
	at, err := time.Parse(time.RFC3339, raw)
	if err != nil {
		return time.Time{}, fmt.Errorf("--eta must be a duration (20m) or an RFC 3339 time, got %q", raw)
	}
	return at.UTC(), nil
}

func maintenanceDefaultSetBy() string {
	user := firstNonEmpty(os.Getenv("SUDO_USER"), os.Getenv("USER"), "unknown")
	host, _ := os.Hostname()
	return "offline-programming maintenance (" + user + "@" + firstNonEmpty(host, "unknown") + ")"
}

// writeMaintenanceNotice replaces the file atomically, world-readable, so a
// server running as another user reads it and never sees half a notice.
func writeMaintenanceNotice(path string, notice maintenanceNoticeFile) error {
	raw, err := json.MarshalIndent(notice, "", "  ")
	if err != nil {
		return err
	}
	dir := filepath.Dir(path)
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return err
	}
	tmp, err := os.CreateTemp(dir, ".maintenance-*.json")
	if err != nil {
		return err
	}
	defer os.Remove(tmp.Name())
	if _, err := tmp.Write(append(raw, '\n')); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Chmod(0o644); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	return os.Rename(tmp.Name(), path)
}

func printMaintenanceView(stdout io.Writer, path string) error {
	notice, err := readMaintenanceNotice(path)
	view := maintenanceViewOf(notice, err, time.Now(), defaultMaintenanceStaleAfter)
	raw, marshalErr := json.MarshalIndent(view, "", "  ")
	if marshalErr != nil {
		return marshalErr
	}
	_, err = fmt.Fprintln(stdout, string(raw))
	return err
}
