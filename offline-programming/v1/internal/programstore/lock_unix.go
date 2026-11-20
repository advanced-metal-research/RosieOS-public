//go:build unix

package programstore

import (
	"errors"
	"os"
	"syscall"
)

// lockDir holds an exclusive flock on path for the store's lifetime. The
// kernel drops it when the process exits, so a crashed server never leaves
// the store locked.
func lockDir(path string) (*os.File, error) {
	f, err := os.OpenFile(path, os.O_CREATE|os.O_RDWR, 0o600)
	if err != nil {
		return nil, err
	}
	if err := syscall.Flock(int(f.Fd()), syscall.LOCK_EX|syscall.LOCK_NB); err != nil {
		f.Close()
		if errors.Is(err, syscall.EWOULDBLOCK) {
			return nil, errors.New("another OLP server holds this program store")
		}
		return nil, err
	}
	return f, nil
}

func unlockDir(f *os.File) error {
	_ = syscall.Flock(int(f.Fd()), syscall.LOCK_UN)
	return f.Close()
}
