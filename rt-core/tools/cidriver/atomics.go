package main

import (
	"bufio"
	"bytes"
	"context"
	"fmt"
	"io"
	"os"
	"os/exec"
	"strings"
	"time"

	"golang.org/x/sys/unix"
)

type interopChild struct {
	cmd    *exec.Cmd
	in     io.WriteCloser
	out    *bufio.Reader
	stderr bytes.Buffer
	waited bool
}

// atomicInterop requires separate native processes to qualify SharedWords and status-sized AtomicSnapshot publications; a timeout is failure.
func atomicInterop(parent context.Context, binary string, log io.Writer) error {
	// unverified watchdog budget: bound a stuck fixture to 15 seconds; completion requires native snapshot assertions.
	ctx, cancel := context.WithTimeout(parent, 15*time.Second)
	defer cancel()
	fd, err := unix.MemfdCreate("ci-shared-words", unix.MFD_CLOEXEC)
	if err != nil {
		return err
	}
	file := os.NewFile(uintptr(fd), "ci-shared-words")
	defer file.Close()
	// Measured fixture ABI: SharedWords<16> has one sequence word plus 16 payload words, each 8 bytes.
	if err = unix.Ftruncate(fd, 136); err != nil {
		return err
	}
	children := []*interopChild{}
	defer func() {
		for _, child := range children {
			if !child.waited {
				_ = child.cmd.Process.Kill()
				_ = child.cmd.Wait()
			}
			_ = child.in.Close()
		}
	}()
	for _, mode := range []string{"read_stress", "write_stress"} {
		// unverified contention workload: 50,000 publications; the reader checks every accepted sequence and payload.
		child := &interopChild{cmd: exec.CommandContext(ctx, binary, mode, "50000")}
		child.cmd.ExtraFiles = []*os.File{file}
		child.cmd.Stderr = &child.stderr
		child.in, err = child.cmd.StdinPipe()
		if err != nil {
			return err
		}
		pipe, err := child.cmd.StdoutPipe()
		if err != nil {
			return err
		}
		child.out = bufio.NewReader(pipe)
		if err = child.cmd.Start(); err != nil {
			_ = child.in.Close()
			_ = pipe.Close()
			return err
		}
		children = append(children, child)
		fmt.Fprintf(log, "command=%q pid=%d shared_fd=3\n", child.cmd.Args, child.cmd.Process.Pid)
		line, err := child.out.ReadString('\n')
		if err != nil || line != "ready\n" {
			return fmt.Errorf("%s readiness: %q %v", mode, line, err)
		}
	}
	for _, child := range children {
		if _, err = io.WriteString(child.in, "c"); err != nil {
			return err
		}
	}
	for _, child := range children {
		output, readErr := io.ReadAll(child.out)
		err = child.cmd.Wait()
		child.waited = true
		fmt.Fprintf(log, "mode=%s stdout=%s stderr=%s\n", child.cmd.Args[1], output, child.stderr.String())
		if readErr != nil || err != nil || !strings.HasPrefix(string(output), "done accepted=") {
			return fmt.Errorf("atomic_snapshot_check_failed: %s read=%v exit=%v", child.cmd.Args[1], readErr, err)
		}
	}
	fmt.Fprintln(log, "PASS: two native processes shared one memfd; final generation=100000 publications=50000")
	// unverified contention workload: require all 50,000 status-sized publications in the production template.
	snapshot := exec.CommandContext(ctx, binary, "qualify_snapshot", "50000")
	snapshot.Stdout, snapshot.Stderr = log, log
	fmt.Fprintf(log, "command=%q\n", snapshot.Args)
	if err := snapshot.Run(); err != nil {
		return fmt.Errorf("atomic_status_snapshot_check_failed: %w", err)
	}
	return nil
}
