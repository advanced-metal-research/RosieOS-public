package seam

import (
	"context"
	"io"
	"io/fs"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"
)

// A spare is a worker process started BEFORE its request: launched, its imports
// done (`--preload`), blocked reading stdin. It still answers exactly one
// request and exits, so nothing a request touches can reach the next one --
// only the start-up moves out of the click.
//
// Measured on bracket_b_2x, seam_0010, median of 5, answers byte-identical to a
// fresh process:
//
//	                    fresh process   spare     fork server
//	plan_torch_path     1.044 s         0.392 s   0.427 s
//	check_torch_fits    0.989 s         0.341 s   0.383 s
//
// A fresh process spends 0.40 s importing OCP and the reach modules, and 0.23 s
// in interpreter teardown after it has answered (the worker now leaves with
// os._exit, spare or not). What stays per request is what depends on the
// request: 0.17 s parsing the STEP it carries and 0.03 s indexing it. The fork
// server -- one warm parent forking a child per request -- was slower and would
// have put a long-lived OCCT process in the design, so it is not used.

// SpareStarter is a CommandRunner that can start a worker ahead of its request.
// A runner without it -- every test double of CommandRunner -- runs cold.
type SpareStarter interface {
	Start(command Command) (Spare, error)
}

// Spare is one started worker waiting for its one request.
type Spare interface {
	// Serve hands the spare its request and waits for the process to finish.
	Serve(ctx context.Context, stdin []byte) (stdout []byte, stderr []byte, err error)
	// Exited reports a spare that died before it was asked anything.
	Exited() bool
	// StartedAt is when the process launched, so a spare that predates an edit
	// to the worker's code is retired instead of answering with the old code.
	StartedAt() time.Time
	// Discard ends a spare that will not be used.
	Discard()
}

// Warm starts the spares, so the first request after start-up is warm too.
// Until it is called none exist and every request starts its own process.
func (worker *Worker) Warm() {
	if worker.starter() == nil || worker.spareCount == 0 {
		return
	}
	worker.spareMu.Lock()
	if worker.closed {
		worker.spareMu.Unlock()
		return
	}
	worker.warmed = true
	worker.spareMu.Unlock()
	for waiting := len(worker.spares); waiting < worker.spareCount; waiting++ {
		go worker.refill()
	}
}

// Close ends the waiting spares and stops starting new ones. Requests keep
// working, cold.
func (worker *Worker) Close() {
	worker.spareMu.Lock()
	worker.closed = true
	worker.spareMu.Unlock()
	for {
		select {
		case spare := <-worker.spares:
			spare.Discard()
		default:
			return
		}
	}
}

// execute runs one request on a spare when one is waiting, cold otherwise.
func (worker *Worker) execute(ctx context.Context, payload []byte) ([]byte, []byte, error) {
	if spare := worker.takeSpare(); spare != nil {
		return spare.Serve(ctx, payload)
	}
	return worker.runner.Run(ctx, worker.command(payload))
}

func (worker *Worker) takeSpare() Spare {
	for {
		select {
		case spare := <-worker.spares:
			// Replaced whether or not it is usable, so the pool stays full.
			go worker.refill()
			if spare.Exited() || worker.codeChangedSince(spare.StartedAt()) {
				spare.Discard()
				continue
			}
			return spare
		default:
			return nil
		}
	}
}

func (worker *Worker) refill() {
	starter := worker.starter()
	if starter == nil {
		return
	}
	worker.spareMu.Lock()
	stop := worker.closed || !worker.warmed
	worker.spareMu.Unlock()
	if stop {
		return
	}
	spare, err := starter.Start(worker.spareCommand())
	if err != nil {
		// Logged, not returned: the request that finds no spare runs cold and a
		// launcher that cannot start says so there, with a code. But a spare
		// that never starts is otherwise invisible -- every click just quietly
		// pays the full start-up again.
		log.Printf("seam: a spare worker did not start, requests run cold: %v", err)
		return
	}
	worker.spareMu.Lock()
	defer worker.spareMu.Unlock()
	if worker.closed {
		spare.Discard()
		return
	}
	select {
	case worker.spares <- spare:
	default:
		spare.Discard()
	}
}

func (worker *Worker) starter() SpareStarter {
	starter, _ := worker.runner.(SpareStarter)
	return starter
}

func (worker *Worker) spareCommand() Command {
	command := worker.command(nil)
	command.Args = append(append([]string(nil), command.Args...), "--preload")
	command.Stdin = nil
	return command
}

// codeChangedSince reports a worker source file newer than a spare. A spare
// has imported the code as it was when it started, and in a checkout being
// edited that is the wrong code for exactly one request per spare -- the kind
// of stale answer that reads as the edit not working.
func (worker *Worker) codeChangedSince(started time.Time) bool {
	changed := false
	_ = filepath.WalkDir(filepath.Join(worker.root, "python"), func(path string, entry fs.DirEntry, err error) error {
		if err != nil {
			return nil
		}
		if entry.IsDir() {
			if entry.Name() == "__pycache__" {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(entry.Name(), ".py") {
			return nil
		}
		if info, err := entry.Info(); err == nil && info.ModTime().After(started) {
			changed = true
			return fs.SkipAll
		}
		return nil
	})
	return changed
}

// Start launches a worker that waits for its request on stdin.
func (ExecRunner) Start(command Command) (Spare, error) {
	process := exec.Command(command.Name, command.Args...)
	containProcessTree(process)
	process.Dir = command.Dir
	process.Env = append(os.Environ(), "PYTHONPATH="+filepath.Join(command.Dir, "python"))
	stdin, err := process.StdinPipe()
	if err != nil {
		return nil, err
	}
	spare := &execSpare{
		process: process,
		stdin:   stdin,
		stdout:  &limitedBuffer{limit: command.MaxOutputBytes},
		stderr:  &limitedBuffer{limit: 64 << 10},
		done:    make(chan struct{}),
		started: time.Now(),
	}
	process.Stdout = spare.stdout
	process.Stderr = spare.stderr
	if err := process.Start(); err != nil {
		return nil, err
	}
	// Wait owns the output copies, so the buffers are complete and no longer
	// written to by the time `done` closes.
	go func() {
		spare.waitErr = process.Wait()
		close(spare.done)
	}()
	return spare, nil
}

type execSpare struct {
	process *exec.Cmd
	stdin   io.WriteCloser
	stdout  *limitedBuffer
	stderr  *limitedBuffer
	done    chan struct{}
	waitErr error
	started time.Time
}

func (spare *execSpare) Serve(ctx context.Context, stdin []byte) ([]byte, []byte, error) {
	go func() {
		// A request with a STEP in it is bigger than a pipe holds, so this
		// blocks until the worker reads -- which a spare still importing has
		// not started to do. Closed either way, so the worker sees the end.
		_, _ = spare.stdin.Write(stdin)
		_ = spare.stdin.Close()
	}()
	select {
	case <-spare.done:
	case <-ctx.Done():
		_ = killProcessTree(spare.process)
		<-spare.done
		return spare.stdout.Bytes(), spare.stderr.Bytes(), ctx.Err()
	}
	if spare.stdout.exceeded {
		return spare.stdout.Bytes(), spare.stderr.Bytes(), errOutputLimit
	}
	return spare.stdout.Bytes(), spare.stderr.Bytes(), spare.waitErr
}

func (spare *execSpare) Exited() bool {
	select {
	case <-spare.done:
		return true
	default:
		return false
	}
}

func (spare *execSpare) StartedAt() time.Time { return spare.started }

func (spare *execSpare) Discard() {
	_ = spare.stdin.Close()
	_ = killProcessTree(spare.process)
}
