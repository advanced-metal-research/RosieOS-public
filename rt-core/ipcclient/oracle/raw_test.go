//go:build linux

package oracle

import (
	"encoding/binary"
	"errors"
	"math"
	"net"
	"path/filepath"
	"runtime"
	"strings"
	"sync/atomic"
	"testing"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
	ipc "rosieos/rt-core/ipcclient"
)

// Measured native ring slot alignment: the 28-byte header occupies 32 bytes.
const ringSlotOffset = 32

type proxy struct {
	data          []byte
	header        ipc.ShmHeaderV1
	fd            int
	seq           uint64
	dropHeartbeat atomic.Bool
}

// intercept preserves the actual daemon descriptors except for the explicitly
// full writer fixture, allowing invalid generated frames to reach the daemon.
func intercept(f *fixture, blocked bool) *proxy {
	t := f.t
	must(t, f.c.Close())
	poll(t, time.Second, func() bool { return f.rawStatus()["lease_valid"] == float64(0) })
	path := filepath.Join(f.dir, "proxy.sock")
	listener, err := net.ListenUnix("unixpacket", &net.UnixAddr{Name: path, Net: "unixpacket"})
	must(t, err)
	p := &proxy{seq: 1000}
	ready := make(chan error, 1)
	done := make(chan struct{})
	go func() {
		defer close(done)
		local, err := listener.AcceptUnix()
		if err != nil {
			ready <- err
			return
		}
		defer local.Close()
		remote, err := net.DialUnix("unixpacket", nil, &net.UnixAddr{Name: f.socket, Net: "unixpacket"})
		if err != nil {
			ready <- err
			return
		}
		defer remote.Close()
		hello := make([]byte, ipc.HelloV1Bytes)
		n, err := local.Read(hello)
		if err != nil {
			ready <- err
			return
		}
		if _, err = remote.Write(hello[:n]); err != nil {
			ready <- err
			return
		}
		welcome, oob := make([]byte, ipc.WelcomeV1Bytes), make([]byte, unix.CmsgSpace(20))
		n, on, _, _, err := remote.ReadMsgUnix(welcome, oob)
		if err != nil {
			ready <- err
			return
		}
		messages, err := unix.ParseSocketControlMessage(oob[:on])
		if err != nil {
			ready <- err
			return
		}
		var fds []int
		for _, m := range messages {
			rights, e := unix.ParseUnixRights(&m)
			if e != nil {
				ready <- e
				return
			}
			fds = append(fds, rights...)
		}
		defer func() {
			for _, fd := range fds {
				unix.Close(fd)
			}
		}()
		if len(fds) != 4 {
			ready <- errors.New("expected four oracle descriptors")
			return
		}
		var stat unix.Stat_t
		if err = unix.Fstat(fds[0], &stat); err != nil {
			ready <- err
			return
		}
		p.data, err = unix.Mmap(fds[0], 0, int(stat.Size), unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
		if err != nil {
			ready <- err
			return
		}
		defer unix.Munmap(p.data)
		p.header, err = ipc.DecodeShmHeaderV1(p.data[:ipc.ShmHeaderV1Bytes])
		if err != nil {
			ready <- err
			return
		}
		p.fd = fds[2]
		sent := append([]int{}, fds...)
		if blocked {
			fd, e := unix.MemfdCreate("full-oracle-ring", unix.MFD_CLOEXEC)
			if e != nil {
				ready <- e
				return
			}
			defer unix.Close(fd)
			copyData := append([]byte{}, p.data...)
			binary.LittleEndian.PutUint32(copyData[int(p.header.RingOffset)+ipc.RingHeaderV1WriteIdxOffset:], p.header.RingCapacity)
			if _, e = unix.Write(fd, copyData); e != nil {
				ready <- e
				return
			}
			sent[0] = fd
		}
		if _, _, err = local.WriteMsgUnix(welcome[:n], unix.UnixRights(sent...), nil); err != nil {
			ready <- err
			return
		}
		ready <- nil
		replies := make(chan struct{})
		go func() {
			defer close(replies)
			b := make([]byte, 512)
			for {
				n, e := remote.Read(b)
				if e != nil {
					return
				}
				if _, e = local.Write(b[:n]); e != nil {
					return
				}
			}
		}()
		b := make([]byte, 512)
		for {
			n, e := local.Read(b)
			if e != nil {
				break
			}
			if p.dropHeartbeat.Load() && string(b[:n]) == "RTHEART1" {
				continue
			}
			if _, e = remote.Write(b[:n]); e != nil {
				break
			}
		}
		remote.Close()
		local.Close()
		<-replies
	}()
	f.options.SocketPath = path
	f.c, err = ipc.Dial(ctx, f.options)
	must(t, err)
	must(t, <-ready)
	f.wait(func(s ipc.Status) bool { return s.LeaseValid == 1 })
	t.Cleanup(func() { f.c.Close(); listener.Close(); <-done })
	return p
}
func (p *proxy) counter(offset int) *uint32 {
	return (*uint32)(unsafe.Pointer(&p.data[int(p.header.RingOffset)+offset]))
}
func (p *proxy) drained() bool {
	return atomic.LoadUint32(p.counter(ipc.RingHeaderV1WriteIdxOffset)) == atomic.LoadUint32(p.counter(ipc.RingHeaderV1ReadIdxOffset))
}
func (p *proxy) send(t *testing.T, kind uint16, payload []byte, stamp uint64) {
	t.Helper()
	poll(t, time.Second, p.drained)
	if stamp == 0 {
		var ts unix.Timespec
		must(t, unix.ClockGettime(unix.CLOCK_MONOTONIC, &ts))
		stamp = uint64(ts.Nano())
	}
	written := atomic.LoadUint32(p.counter(ipc.RingHeaderV1WriteIdxOffset))
	offset := int(p.header.RingOffset) + ringSlotOffset + int(written%p.header.RingCapacity)*ipc.GRADIENT_RING_MSG_BYTES
	p.seq++
	header := ipc.EncodeMsgHeader(ipc.MsgHeader{Type: kind, Bytes: uint32(ipc.MsgHeaderBytes + len(payload)), Seq: p.seq, TimeNs: stamp})
	copy(p.data[offset:], header[:])
	copy(p.data[offset+ipc.MsgHeaderBytes:], payload)
	atomic.StoreUint32(p.counter(ipc.RingHeaderV1WriteIdxOffset), written+1)
	var wake [8]byte
	binary.LittleEndian.PutUint64(wake[:], 1)
	_, err := unix.Write(p.fd, wake[:])
	must(t, err)
}

// Origin: test_runtime.RuntimeTests.test_stop_interrupts_a_backpressured_writer.
func TestStopInterruptsABackpressuredWriter(t *testing.T) {
	f := start(t, "simulation.json")
	p := intercept(f, true)
	enable := ipc.EncodeCmdAxisMaskV1(ipc.CmdAxisMaskV1{AxisMask: 3})
	p.send(t, 0x0102, enable[:], 0)
	arm := ipc.EncodeCmdArmV1(ipc.CmdArmV1{Arm: 1})
	p.send(t, 0x0101, arm[:], 0)
	f.wait(func(s ipc.Status) bool { return s.Armed == 1 })
	result := make(chan error, 1)
	go func() { _, err := f.c.Arm(ctx); result <- err }()
	poll(t, time.Second, func() bool {
		b := make([]byte, 1<<20)
		n := runtime.Stack(b, true)
		return strings.Contains(string(b[:n]), "ipcclient.(*Client).sendLocked")
	})
	f.stop()
	select {
	case err := <-result:
		if !errors.Is(err, ipc.ErrRevoked) {
			t.Fatalf("blocked writer: %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("writer remained blocked")
	}
}

// Origin: test_runtime.RuntimeTests.test_stop_invalidates_an_uncommitted_upload.
func TestStopInvalidatesAnUncommittedUpload(t *testing.T) {
	f := start(t, "simulation.json")
	p := intercept(f, false)
	f.arm(3)
	begin := ipc.EncodeCmdTrajectoryBeginV1(ipc.CmdTrajectoryBeginV1{TrajId: 504, AxisMask: 3, ExpectedPoints: 2})
	p.send(t, 0x0120, begin[:], 0)
	_, err := f.c.WaitResult(ctx, p.seq)
	must(t, err)
	first := ipc.EncodeTrajectoryPointV1(ipc.TrajectoryPointV1{TrajId: 504, AxisMask: 3})
	p.send(t, 0x0121, first[:], 0)
	last := ipc.EncodeTrajectoryPointV1(ipc.TrajectoryPointV1{TrajId: 504, PointIndex: 1, Flags: 2, TFromStartNs: 1000000000, Q: [16]float64{.02}, AxisMask: 3})
	p.send(t, 0x0121, last[:], 0)
	poll(t, time.Second, p.drained)
	f.stop()
	enable := ipc.EncodeCmdAxisMaskV1(ipc.CmdAxisMaskV1{AxisMask: 3})
	p.send(t, 0x0102, enable[:], 0)
	arm := ipc.EncodeCmdArmV1(ipc.CmdArmV1{Arm: 1})
	p.send(t, 0x0101, arm[:], 0)
	f.wait(func(s ipc.Status) bool { return s.Armed == 1 })
	commit := ipc.EncodeCmdTrajectoryControlV1(ipc.CmdTrajectoryControlV1{TrajId: 504})
	p.send(t, 0x0122, commit[:], 0)
	f.stable(400*time.Millisecond, func(s ipc.Status) bool { return s.Axes[0].PositionCounts == 0 })
}

// Origin: test_runtime.RuntimeTests.test_lease_expires_even_with_socket_open.
func TestLeaseExpiresEvenWithSocketOpen(t *testing.T) {
	f := start(t, "simulation.json")
	p := intercept(f, false)
	f.arm(3)
	p.dropHeartbeat.Store(true)
	s := f.wait(func(s ipc.Status) bool { return s.Armed == 0 && s.LeaseValid == 0 })
	if s.AxisEnableMask != 0 {
		t.Fatal("lease expiry retained enable")
	}
}

// Origin: test_runtime.RuntimeTests.test_nan_velocity_revokes_outputs.
func TestNaNVelocityRevokesOutputs(t *testing.T) {
	f := start(t, "simulation.json")
	p := intercept(f, false)
	f.arm(3)
	frame := ipc.EncodeCmdJogV1(ipc.CmdJogV1{AxisMask: 1, Flags: 1, TimeoutNs: 150000000, Qd: [16]float64{math.NaN()}})
	p.send(t, 0x0130, frame[:], 0)
	f.wait(func(s ipc.Status) bool { return s.Armed == 0 })
}

// Origin: test_runtime.RuntimeTests.test_stale_command_revokes_outputs.
func TestStaleCommandRevokesOutputs(t *testing.T) {
	f := start(t, "simulation.json")
	p := intercept(f, false)
	f.arm(3)
	frame := ipc.EncodeCmdArmV1(ipc.CmdArmV1{Arm: 1})
	p.send(t, 0x0101, frame[:], 1)
	f.wait(func(s ipc.Status) bool { return s.Armed == 0 })
}

// Origin: test_runtime.HomingTests.test_arbitrary_setting_change_revokes_readback_evidence.
func TestArbitrarySettingChangeRevokesReadbackEvidence(t *testing.T) {
	f := start(t, "simulation-home.json")
	p := intercept(f, false)
	frame := ipc.EncodeCmdServiceSdoWriteV1(ipc.CmdServiceSdoWriteV1{AxisMask: 1, Index: 0x3000, Subindex: 1, ValueType: 1, ValueU32: 9})
	p.send(t, 0x0109, frame[:], 0)
	s := f.waitWithin(800*time.Millisecond, func(s ipc.Status) bool { return s.ConfigVerifiedMask == 0 })
	if s.HomeValidMask != 0 || s.Armed != 0 {
		t.Fatalf("changed setting evidence: %+v", s)
	}
	// The origin sampled once at 800 ms; revocation must persist, not flicker.
	f.stable(800*time.Millisecond, func(s ipc.Status) bool {
		return s.ConfigVerifiedMask == 0 && s.HomeValidMask == 0 && s.Armed == 0
	})
}
