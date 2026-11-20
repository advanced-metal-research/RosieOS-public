//go:build linux || darwin

package discovery

import (
	"context"
	"errors"
	"fmt"
	"net"
	"syscall"

	"golang.org/x/sys/unix"
)

// ListenMulticast opens the shared mDNS socket (0.0.0.0:5353, address and
// port reuse so avahi or mDNSResponder keep working) and joins the group on
// each interface only. Replies use multicast TTL 255 (RFC 6762 section 11).
func ListenMulticast(ifaces []Interface) (net.PacketConn, error) {
	if len(ifaces) == 0 {
		return nil, errors.New("discovery: no interfaces to advertise on")
	}
	config := net.ListenConfig{Control: func(_, _ string, raw syscall.RawConn) error {
		var serr error
		err := raw.Control(func(fd uintptr) {
			if serr = unix.SetsockoptInt(int(fd), unix.SOL_SOCKET, unix.SO_REUSEADDR, 1); serr != nil {
				return
			}
			serr = unix.SetsockoptInt(int(fd), unix.SOL_SOCKET, unix.SO_REUSEPORT, 1)
		})
		if err != nil {
			return err
		}
		return serr
	}}
	conn, err := config.ListenPacket(context.Background(), "udp4", fmt.Sprintf("0.0.0.0:%d", mdnsGroup.Port))
	if err != nil {
		return nil, fmt.Errorf("discovery: mdns socket: %w", err)
	}
	if err := withFD(conn, func(fd int) error {
		if err := unix.SetsockoptByte(fd, unix.IPPROTO_IP, unix.IP_MULTICAST_TTL, 255); err != nil {
			if err := unix.SetsockoptInt(fd, unix.IPPROTO_IP, unix.IP_MULTICAST_TTL, 255); err != nil {
				return fmt.Errorf("multicast ttl: %w", err)
			}
		}
		for _, iface := range ifaces {
			mreq := &unix.IPMreq{}
			copy(mreq.Multiaddr[:], mdnsGroup.IP.To4())
			copy(mreq.Interface[:], iface.IPv4.To4())
			if err := unix.SetsockoptIPMreq(fd, unix.IPPROTO_IP, unix.IP_ADD_MEMBERSHIP, mreq); err != nil {
				return fmt.Errorf("join %s on %s: %w", mdnsGroup.IP, iface.Name, err)
			}
		}
		return nil
	}); err != nil {
		conn.Close()
		return nil, fmt.Errorf("discovery: %w", err)
	}
	return conn, nil
}

// setMulticastInterface picks the interface the next multicast send leaves on.
func setMulticastInterface(conn net.PacketConn, ip net.IP) error {
	var addr [4]byte
	copy(addr[:], ip.To4())
	return withFD(conn, func(fd int) error {
		return unix.SetsockoptInet4Addr(fd, unix.IPPROTO_IP, unix.IP_MULTICAST_IF, addr)
	})
}

func withFD(conn net.PacketConn, f func(int) error) error {
	sc, ok := conn.(syscall.Conn)
	if !ok {
		return errors.New("socket has no file descriptor")
	}
	raw, err := sc.SyscallConn()
	if err != nil {
		return err
	}
	var ferr error
	if err := raw.Control(func(fd uintptr) { ferr = f(int(fd)) }); err != nil {
		return err
	}
	return ferr
}
