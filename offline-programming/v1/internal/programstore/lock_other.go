//go:build !unix

package programstore

import "os"

// lockDir on a platform without flock (the Windows workstation build) opens
// the lock file only; a cell's OLP host is Linux and takes the real lock.
func lockDir(path string) (*os.File, error) {
	return os.OpenFile(path, os.O_CREATE|os.O_RDWR, 0o600)
}

func unlockDir(f *os.File) error { return f.Close() }
