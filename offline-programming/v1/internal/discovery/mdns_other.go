//go:build !linux && !darwin

package discovery

import (
	"errors"
	"net"
)

func ListenMulticast([]Interface) (net.PacketConn, error) {
	return nil, errors.New("discovery: mdns is supported on linux and darwin only")
}

func setMulticastInterface(net.PacketConn, net.IP) error {
	return errors.New("discovery: mdns is supported on linux and darwin only")
}
