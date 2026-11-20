//go:build linux

package ipcclient

import (
	"errors"
	"net"
	"os"
	"path/filepath"
	"syscall"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

// Occupied lock files, unverified path types and redirected lock inodes must
// survive failed startup byte-for-byte; a refused connect alone grants nothing.
func TestRestartPreservesUnverifiedSocketPaths(t *testing.T) {
	f := newRestartFixture(t)
	for _, process := range []string{"daemon", "adapter"} {
		for _, obstacle := range []string{"regular", "symlink", "lock_symlink", "missing_lock", "held_stale", "held_missing"} {
			t.Run(process+"/"+obstacle, func(t *testing.T) {
				dir, err := os.MkdirTemp(f.dir, "o-")
				if err != nil {
					t.Fatal(err)
				}
				path := filepath.Join(dir, "s")
				target := filepath.Join(dir, "t")
				if err := os.WriteFile(target, []byte("preserve owner data\n"), 0600); err != nil {
					t.Fatal(err)
				}
				switch obstacle {
				case "regular":
					if err := os.WriteFile(path, []byte("not a socket\n"), 0600); err != nil {
						t.Fatal(err)
					}
				case "symlink":
					if err := os.Symlink(target, path); err != nil {
						t.Fatal(err)
					}
				case "lock_symlink":
					if err := os.Symlink(target, path+".lock"); err != nil {
						t.Fatal(err)
					}
				case "held_stale", "missing_lock":
					kind := unix.SOCK_STREAM
					if process == "daemon" {
						kind = unix.SOCK_SEQPACKET
					}
					fd, err := unix.Socket(unix.AF_UNIX, kind|unix.SOCK_CLOEXEC, 0)
					if err != nil {
						t.Fatal(err)
					}
					if err := unix.Bind(fd, &unix.SockaddrUnix{Name: path}); err != nil {
						unix.Close(fd)
						t.Fatal(err)
					}
					unix.Close(fd)
				}
				if obstacle == "held_stale" || obstacle == "held_missing" {
					fd, err := unix.Open(path+".lock", unix.O_RDWR|unix.O_CREAT|unix.O_CLOEXEC, 0600)
					if err != nil {
						t.Fatal(err)
					}
					defer unix.Close(fd)
					if err := unix.Flock(fd, unix.LOCK_EX|unix.LOCK_NB); err != nil {
						t.Fatal(err)
					}
				}
				before, beforeErr := os.Lstat(path)
				binary, args := f.binary, append(append([]string(nil), f.args...), "--socket-path", path)
				if process == "adapter" {
					binary, args = f.adapterBinary, append(append([]string(nil), f.adapterArgs...), "--socket", path)
				}
				child := restartSpawn(t, dir, binary, args)
				select {
				case <-child.done:
				case <-time.After(restartWait):
					t.Fatal("unsafe startup did not refuse")
				}
				if child.err == nil {
					t.Fatal("unsafe startup succeeded")
				}
				if obstacle == "held_stale" || obstacle == "held_missing" {
					restartRefused(t, child)
				}
				after, afterErr := os.Lstat(path)
				if beforeErr == nil {
					if afterErr != nil || !os.SameFile(before, after) {
						t.Fatal("startup changed an unverified path")
					}
				} else if !errors.Is(afterErr, os.ErrNotExist) {
					t.Fatal("startup bound despite a held lock")
				}
				data, err := os.ReadFile(target)
				if err != nil || string(data) != "preserve owner data\n" {
					t.Fatalf("lock redirect overwrote data: %q %v", data, err)
				}
			})
		}
	}
}

func TestAdapterImmediateCrashRestart(t *testing.T) {
	f := newRestartFixture(t)
	f.startAdapter()
	f.acquire()
	before := f.status(func(s restartStatus) bool { return s.Grant.GrantActive == 1 })
	f.adapter.signal(t, syscall.SIGKILL)
	// Restart immediately while native may still be draining the dead connection.
	f.startAdapter()
	after := f.assertInhibited(before.Core.TimeNS+1, true)
	if after.AdapterIncarnation == before.AdapterIncarnation || after.DaemonIncarnation != before.DaemonIncarnation {
		t.Fatal("immediate restart did not replace only the adapter")
	}
	f.stale("renew", "fence", nil)
	// A datagram connect proves the recovered jog socket is bound and live.
	conn, err := net.Dial("unixgram", filepath.Join(f.dir, "jog.sock"))
	if err != nil {
		t.Fatal(err)
	}
	conn.Close()
	f.acquire()
}
