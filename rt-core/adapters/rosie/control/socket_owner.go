//go:build linux

package control

import (
	"bufio"
	"errors"
	"fmt"
	"net"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	"golang.org/x/sys/unix"
)

var ErrSocketOwned = errors.New("socket owned by a live process")

// SocketOwner holds the persistent lock inode until its socket has been removed.
// Never unlink the lock file: doing so would allow two independent flock owners.
type SocketOwner struct {
	file  *os.File
	path  string
	bound os.FileInfo
}

// ClaimSocket verifies the UID, socket type, refused connection and exclusive
// lock before stale removal. Unknown failures preserve the existing path.
func ClaimSocket(path, network string) (_ *SocketOwner, err error) {
	if network != "unix" && network != "unixgram" {
		return nil, errors.New("unsupported socket ownership transport")
	}
	stale, err := probeSocket(path, network)
	if err != nil {
		return nil, err
	}
	// A refused socket without its predecessor's lock cannot prove ownership.
	flags := unix.O_RDWR | unix.O_CLOEXEC | unix.O_NOFOLLOW | unix.O_NONBLOCK
	if !stale {
		flags |= unix.O_CREAT
	}
	fd, err := unix.Open(path+".lock", flags, 0600)
	if err != nil {
		return nil, err
	}
	owner := &SocketOwner{file: os.NewFile(uintptr(fd), path+".lock"), path: path}
	defer func() {
		if err != nil {
			_ = owner.Close()
		}
	}()
	var stat unix.Stat_t
	if err = unix.Fstat(fd, &stat); err != nil {
		return nil, err
	}
	if stat.Mode&unix.S_IFMT != unix.S_IFREG || stat.Uid != uint32(os.Geteuid()) || stat.Nlink != 1 || stat.Mode&0022 != 0 {
		return nil, errors.New("socket lock ownership cannot be verified")
	}
	if err = unix.Flock(fd, unix.LOCK_EX|unix.LOCK_NB); err != nil {
		if errors.Is(err, unix.EWOULDBLOCK) {
			return nil, ErrSocketOwned
		}
		return nil, err
	}
	var current unix.Stat_t
	if err = unix.Lstat(path+".lock", &current); err != nil {
		return nil, err
	}
	if current.Dev != stat.Dev || current.Ino != stat.Ino {
		return nil, errors.New("socket lock path changed")
	}
	stale, err = probeSocket(path, network)
	if err != nil {
		return nil, err
	}
	proc, err := os.ReadFile("/proc/self/stat")
	if err != nil {
		return nil, err
	}
	end := strings.LastIndexByte(string(proc), ')')
	if end < 0 {
		return nil, errors.New("process start time unavailable")
	}
	fields := strings.Fields(string(proc[end+1:]))
	if len(fields) < 20 {
		return nil, errors.New("process start time unavailable")
	}
	if _, err = strconv.ParseUint(fields[19], 10, 64); err != nil {
		return nil, err
	}
	if err = owner.file.Truncate(0); err != nil {
		return nil, err
	}
	if _, err = fmt.Fprintf(owner.file, "pid=%d start_time_ticks=%s\n", os.Getpid(), fields[19]); err != nil {
		return nil, err
	}
	if stale {
		if err = os.Remove(path); err != nil {
			return nil, err
		}
	}
	return owner, nil
}

// A successful connect is enough to refuse takeover even if the owner cannot
// answer HTTP promptly. Only ECONNREFUSED permits recovery under the owner lock.
func probeSocket(path, network string) (bool, error) {
	var st unix.Stat_t
	if err := unix.Lstat(path, &st); err != nil {
		if errors.Is(err, unix.ENOENT) {
			return false, nil
		}
		return false, err
	}
	if st.Mode&unix.S_IFMT != unix.S_IFSOCK || st.Uid != uint32(os.Geteuid()) {
		return false, errors.New("socket ownership cannot be verified")
	}
	// Unverified probe budget: 200 ms bounds a silent or overloaded live owner.
	const probeBudget = 200 * time.Millisecond
	conn, err := net.DialTimeout(network, path, probeBudget)
	if err != nil {
		if errors.Is(err, unix.ECONNREFUSED) {
			return true, nil
		}
		return false, err
	}
	defer conn.Close()
	if network == "unix" {
		_ = conn.SetDeadline(time.Now().Add(probeBudget))
		if _, err := fmt.Fprint(conn, "GET /v1/describe HTTP/1.1\r\nHost: local\r\nConnection: close\r\n\r\n"); err == nil {
			if response, err := http.ReadResponse(bufio.NewReader(conn), nil); err == nil {
				_ = response.Body.Close()
			}
		}
	}
	return false, ErrSocketOwned
}

// Bound records the socket inode created while holding the claim, preventing
// shutdown from removing a replacement path it did not bind.
func (o *SocketOwner) Bound() error {
	info, err := os.Lstat(o.path)
	if err != nil {
		return err
	}
	if info.Mode()&os.ModeSocket == 0 {
		return errors.New("bound path is not a socket")
	}
	o.bound = info
	return nil
}

func (o *SocketOwner) Close() error {
	if o.file == nil {
		return nil
	}
	var removeErr error
	if o.bound != nil {
		if info, err := os.Lstat(o.path); err == nil && os.SameFile(o.bound, info) {
			removeErr = os.Remove(o.path)
		}
	}
	err := o.file.Close()
	o.file = nil
	return errors.Join(removeErr, err)
}
