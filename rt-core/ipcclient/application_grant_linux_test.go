//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"fmt"
	"net"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

// Unverified LAN fixture lease: 500,000,000 ns, independent of client preflight bounds.
const applicationGrantMaxLease = uint64(500000000)

func grantTestHeader() FastHeader {
	return FastHeader{Magic: applicationGrantMagic, VersionMajor: VersionMajor, VersionMinor: VersionMinor,
		RegionBytes: applicationGrantRegionBytes, DaemonIncarnation: [16]byte{19}, ConnectionId: 17,
		ConfigurationDigestLo: [16]byte{23}, ConfigurationDigestHi: [16]byte{29}, CycleNs: uint64(time.Millisecond), ConfiguredMask: 3}
}

func grantTestDigest() []byte {
	header := grantTestHeader()
	return append(append([]byte{}, header.ConfigurationDigestLo[:]...), header.ConfigurationDigestHi[:]...)
}

func grantTestFD(t *testing.T, header FastHeader, size, seals, dirtyOffset int) int {
	t.Helper()
	fd, err := unix.MemfdCreate("rtcore-grant-test", unix.MFD_CLOEXEC|unix.MFD_ALLOW_SEALING)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = unix.Close(fd) })
	if err = unix.Ftruncate(fd, int64(size)); err != nil {
		t.Fatal(err)
	}
	encoded := EncodeFastHeader(header)
	if _, err = unix.Pwrite(fd, encoded[:], 0); err != nil {
		t.Fatal(err)
	}
	if dirtyOffset >= 0 {
		if _, err = unix.Pwrite(fd, []byte{1}, int64(dirtyOffset)); err != nil {
			t.Fatal(err)
		}
	}
	if seals != 0 {
		if _, err = unix.FcntlInt(uintptr(fd), unix.F_ADD_SEALS, seals); err != nil {
			t.Fatal(err)
		}
	}
	return fd
}

func TestApplicationGrantRegionValidation(t *testing.T) {
	const allSeals = unix.F_SEAL_GROW | unix.F_SEAL_SHRINK | unix.F_SEAL_SEAL
	for _, test := range []struct {
		name                     string
		change                   func(*FastHeader)
		size, seals, dirtyOffset int
	}{
		{"short", nil, 4095, allSeals, -1}, {"long", nil, 8192, allSeals, -1},
		{"unsealed", nil, 4096, 0, -1}, {"can-grow", nil, 4096, unix.F_SEAL_SHRINK | unix.F_SEAL_SEAL, -1},
		{"can-shrink", nil, 4096, unix.F_SEAL_GROW | unix.F_SEAL_SEAL, -1}, {"can-reseal", nil, 4096, unix.F_SEAL_GROW | unix.F_SEAL_SHRINK, -1},
		{"magic", func(h *FastHeader) { h.Magic++ }, 4096, allSeals, -1},
		{"major", func(h *FastHeader) { h.VersionMajor++ }, 4096, allSeals, -1},
		{"minor", func(h *FastHeader) { h.VersionMinor-- }, 4096, allSeals, -1},
		{"declared-size", func(h *FastHeader) { h.RegionBytes-- }, 4096, allSeals, -1},
		{"incarnation", func(h *FastHeader) { h.DaemonIncarnation = [16]byte{} }, 4096, allSeals, -1},
		{"connection", func(h *FastHeader) { h.ConnectionId = 0 }, 4096, allSeals, -1},
		{"digest-low", func(h *FastHeader) { h.ConfigurationDigestLo[0]++ }, 4096, allSeals, -1},
		{"digest-high", func(h *FastHeader) { h.ConfigurationDigestHi[15]++ }, 4096, allSeals, -1},
		{"cycle", func(h *FastHeader) { h.CycleNs++ }, 4096, allSeals, -1},
		{"mask", func(h *FastHeader) { h.ConfiguredMask = 1 }, 4096, allSeals, -1},
		{"header-reserved", func(h *FastHeader) { h.Reserved = 1 }, 4096, allSeals, -1},
		{"header-gap", nil, 4096, allSeals, 96}, {"request-gap", nil, 4096, allSeals, 216},
		{"observation-tail", nil, 4096, allSeals, 392}, {"region-tail", nil, 4096, allSeals, 4095},
	} {
		t.Run(test.name, func(t *testing.T) {
			header := grantTestHeader()
			if test.change != nil {
				test.change(&header)
			}
			fd := grantTestFD(t, header, test.size, test.seals, test.dirtyOffset)
			region, err := mapApplicationGrantRegion(fd, grantTestDigest(), 2, time.Millisecond)
			if err == nil {
				_ = unix.Munmap(region.data)
				t.Fatal("invalid region accepted")
			}
		})
	}
	fd := grantTestFD(t, grantTestHeader(), 4096, allSeals, -1)
	region, err := mapApplicationGrantRegion(fd, grantTestDigest(), 2, time.Millisecond)
	if err != nil {
		t.Fatal(err)
	}
	defer unix.Munmap(region.data)
	if region.request.count != 10 || len(region.request.data) != 88 || region.observation.count != 16 || len(region.observation.data) != 136 {
		t.Fatalf("wrong atomic slot geometry: %+v", region)
	}
}

func TestApplicationGrantObservationValidation(t *testing.T) {
	fd := grantTestFD(t, grantTestHeader(), 4096, unix.F_SEAL_GROW|unix.F_SEAL_SHRINK|unix.F_SEAL_SEAL, -1)
	region, err := mapApplicationGrantRegion(fd, grantTestDigest(), 2, time.Millisecond)
	if err != nil {
		t.Fatal(err)
	}
	c := &Client{fast: region, done: make(chan struct{}), axes: 2}
	t.Cleanup(c.closeApplicationGrantRegion)
	if _, err := c.FastObservation(); !errors.Is(err, ErrFastObservationUnavailable) {
		t.Fatalf("unpublished: %v", err)
	}
	for _, test := range []struct {
		name   string
		change func(*GrantObservation)
	}{
		{"connection", func(s *GrantObservation) { s.ConnectionId++ }},
		{"native-generation", func(s *GrantObservation) { s.NativeGeneration = 0 }},
		{"config-epoch", func(s *GrantObservation) { s.ConfigurationEpoch = 0 }},
		{"home-epoch", func(s *GrantObservation) { s.HomeEpoch = 0 }},
		{"active-encoding", func(s *GrantObservation) { s.GrantActive = 2 }},
		{"reason-encoding", func(s *GrantObservation) { s.GrantReason = 13 }},
		{"outside-mask", func(s *GrantObservation) { s.AxisMask = 4 }},
		{"reserved", func(s *GrantObservation) { s.Reserved = 1 }},
		{"reserved1", func(s *GrantObservation) { s.Reserved1 = 1 }},
		{"reserved2", func(s *GrantObservation) { s.Reserved2 = 1 }},
		{"reserved3", func(s *GrantObservation) { s.Reserved3 = 1 }},
		{"zero-clock", func(s *GrantObservation) { s.NowHostNs = 0 }},
		{"future-clock", func(s *GrantObservation) { s.NowHostNs += uint64(time.Second) }},
		{"stale-clock", func(s *GrantObservation) { s.NowHostNs -= uint64(time.Second) }},
		{"active-without-identity", func(s *GrantObservation) { s.GrantActive = 1 }},
	} {
		t.Run(test.name, func(t *testing.T) {
			observation := GrantObservation{ConnectionId: 17, NativeGeneration: 11, ConfigurationEpoch: 5, HomeEpoch: 7, NowHostNs: monotonicNS()}
			test.change(&observation)
			encoded := EncodeGrantObservation(observation)
			var words [GrantObservationBytes / 8]uint64
			if sharedWordsFromLE(words[:], encoded[:]) != sharedWordsOK {
				t.Fatal("encode")
			}
			if _, status := region.observation.publish(words[:]); status != sharedWordsOK {
				t.Fatal(status)
			}
			if _, err := c.FastObservation(); err == nil {
				t.Fatal("malformed observation accepted")
			}
		})
	}
}

func TestApplicationGrantRoleNegotiationRejectsDescriptorFallback(t *testing.T) {
	for _, legacy := range []bool{false, true} {
		t.Run(fmt.Sprintf("legacy=%t", legacy), func(t *testing.T) {
			directory, err := os.MkdirTemp("", "grant-")
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() { _ = os.RemoveAll(directory) })
			socket := filepath.Join(directory, "ipc.sock")
			listener, err := net.ListenUnix("unixpacket", &net.UnixAddr{Name: socket, Net: "unixpacket"})
			if err != nil {
				t.Fatal(err)
			}
			defer listener.Close()
			server := make(chan error, 1)
			go func() {
				connection, err := listener.AcceptUnix()
				if err != nil {
					server <- err
					return
				}
				defer connection.Close()
				_ = connection.SetDeadline(time.Now().Add(time.Second))
				var hello [56]byte
				n, err := connection.Read(hello[:])
				role, wrongCount := uint32(3), 4
				if legacy {
					role, wrongCount = 1, 5
				}
				if err != nil || n != len(hello) || wire.Uint32(hello[12:]) != role {
					server <- fmt.Errorf("wrong hello role %d: %v", wire.Uint32(hello[12:]), err)
					return
				}
				var welcome [96]byte
				copy(welcome[:], "GIPC")
				wire.PutUint16(welcome[4:], 1)
				wire.PutUint16(welcome[6:], VersionMinor)
				wire.PutUint32(welcome[8:], 96)
				fds := make([]int, wrongCount)
				for i := range fds {
					fds[i], err = unix.Eventfd(0, unix.EFD_CLOEXEC)
					if err != nil {
						server <- err
						return
					}
					defer unix.Close(fds[i])
				}
				_, _, err = connection.WriteMsgUnix(welcome[:], unix.UnixRights(fds...), nil)
				server <- err
			}()
			client, err := Dial(context.Background(), Options{SocketPath: socket, Backend: "simulation", ConfigurationSHA256: strings.Repeat("0", 64), SimulationTest: legacy})
			if err == nil {
				client.Close()
				t.Fatal("wrong descriptor count accepted")
			}
			if err := <-server; err != nil {
				t.Fatal(err)
			}
		})
	}
	_, err := Dial(context.Background(), Options{Backend: "ethercat", ConfigurationSHA256: strings.Repeat("0", 64), SimulationTest: true})
	if err == nil || !strings.Contains(err.Error(), "requires the simulation backend") {
		t.Fatalf("legacy live role: %v", err)
	}
}

func applicationGrantNativeClient(t *testing.T) *Client {
	t.Helper()
	return startCoreWithRole(t, "simulation.json", false).client
}

func waitGrantObservation(t *testing.T, c *Client, predicate func(GrantObservation) bool) GrantObservation {
	t.Helper()
	deadline := time.Now().Add(time.Second)
	var pollResult GrantObservation
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		observation, err := c.FastObservation()
		if err == nil && predicate(observation) {
			pollResult = observation
			return true
		}
		if err != nil && !errors.Is(err, ErrFastObservationUnavailable) {
			t.Fatal(err)
		}
		if time.Now().After(deadline) {
			t.Fatalf("grant observation did not converge: %+v, %v", observation, err)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func TestApplicationGrantNativeAuthorityAndIndependentRenewal(t *testing.T) {
	c := applicationGrantNativeClient(t)
	initial := waitGrantObservation(t, c, func(s GrantObservation) bool { return s.GrantActive == 0 })
	sequence, err := c.Enable(context.Background(), 3)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = c.WaitResult(context.Background(), sequence); err == nil {
		t.Fatal("adapter enabled without an application grant")
	}
	identity := GrantIdentity{CapabilityID: [16]byte{1, 2, 3}, Generation: 1}
	receipt, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil {
		t.Fatal(err)
	}
	if receipt.NativeGeneration != initial.NativeGeneration || receipt.Identity != identity {
		t.Fatalf("wrong acquire receipt: %+v", receipt)
	}
	arm(t, c, 3)
	// A blocked bulk writer must not delay the fast renewal lane.
	c.commandMu.Lock()
	renewed, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	c.commandMu.Unlock()
	if err != nil || renewed.NativeGeneration != receipt.NativeGeneration {
		t.Fatalf("renew behind upload: %+v %v", renewed, err)
	}
	expired := waitGrantObservation(t, c, func(s GrantObservation) bool {
		return s.GrantActive == 0 && s.NativeGeneration != receipt.NativeGeneration
	})
	state := wait(t, c, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	if c.Err() != nil {
		t.Fatalf("native heartbeat connection failed: %v", c.Err())
	}
	_, err = c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	var rejected *GrantRejectedError
	if !errors.As(err, &rejected) || rejected.Reason != GrantReasonNativeGenerationMismatch {
		t.Fatalf("expired renewal adopted new native generation: %v", err)
	}
	_, err = c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	if !errors.As(err, &rejected) || rejected.Reason != GrantReasonStaleGeneration {
		t.Fatalf("stale acquire generation accepted: %v", err)
	}
	identity.Generation++
	fresh, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil || fresh.NativeGeneration != expired.NativeGeneration {
		t.Fatalf("fresh acquire: %+v %v", fresh, err)
	}
	t.Logf("native heartbeat stayed connected; application expiry advanced native generation %d -> %d, armed=%d mask=%#x", receipt.NativeGeneration, expired.NativeGeneration, state.Armed, state.AxisEnableMask)
}

func TestApplicationGrantOriginAndWaitBudget(t *testing.T) {
	c := applicationGrantNativeClient(t)
	identity := GrantIdentity{CapabilityID: [16]byte{3}, Generation: 1}
	deadline := HostMonotonicNS() + applicationGrantMaxLease
	c.grantMu.Lock()
	started, completed := make(chan struct{}), make(chan error, 1)
	go func() {
		close(started)
		_, err := c.MirrorAcquire(context.Background(), identity, deadline)
		completed <- err
	}()
	<-started
	time.Sleep(20 * time.Millisecond)
	unlocked := HostMonotonicNS()
	c.grantMu.Unlock()
	if err := <-completed; err != nil {
		t.Fatal(err)
	}
	c.fastMu.RLock()
	var words, scratch [GrantRequestBytes / 8]uint64
	_, status := c.fast.request.read(words[:], scratch[:])
	c.fastMu.RUnlock()
	if status != sharedWordsOK {
		t.Fatal(status)
	}
	var encoded [GrantRequestBytes]byte
	if sharedWordsToLE(encoded[:], words[:]) != sharedWordsOK {
		t.Fatal("decode")
	}
	request, ok := DecodeGrantRequest(encoded[:])
	if !ok || request.OriginHostNs >= unlocked || request.DeadlineHostNs != deadline {
		t.Fatalf("grant input was restamped after lock wait: %+v", request)
	}
	c.grantMu.Lock()
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Millisecond)
	_, err := c.MirrorRenew(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease)
	c.grantMu.Unlock()
	cancel()
	if !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("grant lock wait ignored context: %v", err)
	}
}

func TestApplicationGrantCloseInvalidatesFastMappingBeforeUploadWait(t *testing.T) {
	c := applicationGrantNativeClient(t)
	c.commandMu.Lock()
	completed := make(chan error, 1)
	go func() { completed <- c.Close() }()
	select {
	case <-c.done:
	case <-time.After(time.Second):
		c.commandMu.Unlock()
		t.Fatal("Close did not revoke")
	}
	deadline := time.Now().Add(time.Second)
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		c.fastMu.RLock()
		unmapped := c.fast == nil
		c.fastMu.RUnlock()
		if unmapped {
			return true
		}
		if time.Now().After(deadline) {
			c.commandMu.Unlock()
			t.Fatal("fast mapping waited behind upload")
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	_, readErr := c.FastObservation()
	_, publishErr := c.MirrorAcquire(context.Background(), GrantIdentity{CapabilityID: [16]byte{7}, Generation: 1}, HostMonotonicNS()+applicationGrantMaxLease)
	c.commandMu.Unlock()
	if err := <-completed; err != nil {
		t.Fatal(err)
	}
	if !errors.Is(readErr, ErrClosed) || !errors.Is(publishErr, ErrClosed) {
		t.Fatalf("closed mapping reused: read=%v publish=%v", readErr, publishErr)
	}
	for _, fd := range c.fds {
		if _, err := unix.FcntlInt(uintptr(fd), unix.F_GETFD, 0); !errors.Is(err, unix.EBADF) {
			t.Fatalf("descriptor %d retained after close: %v", fd, err)
		}
	}
}

func grantLocalClient(t *testing.T) *Client {
	t.Helper()
	fd := grantTestFD(t, grantTestHeader(), 4096, unix.F_SEAL_GROW|unix.F_SEAL_SHRINK|unix.F_SEAL_SEAL, -1)
	region, err := mapApplicationGrantRegion(fd, grantTestDigest(), 2, time.Millisecond)
	if err != nil {
		t.Fatal(err)
	}
	pair, err := unix.Socketpair(unix.AF_UNIX, unix.SOCK_STREAM|unix.SOCK_CLOEXEC, 0)
	if err != nil {
		_ = unix.Munmap(region.data)
		t.Fatal(err)
	}
	file := os.NewFile(uintptr(pair[0]), "grant-local-client")
	connection, err := net.FileConn(file)
	_ = file.Close()
	if err != nil {
		_ = unix.Close(pair[1])
		_ = unix.Munmap(region.data)
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = unix.Close(pair[1]) })
	c := &Client{fast: region, done: make(chan struct{}), axes: 2, conn: connection.(*net.UnixConn)}
	t.Cleanup(func() { _ = c.Close() })
	grantLocalObservation(t, c, GrantObservation{ConnectionId: 17, NativeGeneration: 11, ConfigurationEpoch: 5, HomeEpoch: 7})
	return c
}

func grantLocalObservation(t *testing.T, c *Client, observation GrantObservation) {
	t.Helper()
	observation.NowHostNs = HostMonotonicNS()
	encoded := EncodeGrantObservation(observation)
	var words [GrantObservationBytes / 8]uint64
	if sharedWordsFromLE(words[:], encoded[:]) != sharedWordsOK {
		t.Fatal("observation codec")
	}
	c.fastMu.RLock()
	_, status := c.fast.observation.publish(words[:])
	c.fastMu.RUnlock()
	if status != sharedWordsOK {
		t.Fatal(status)
	}
}

func grantLocalRequest(t *testing.T, c *Client, ticket uint64) GrantRequest {
	t.Helper()
	deadline := time.Now().Add(time.Second)
	var pollResult GrantRequest
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		var words, scratch [GrantRequestBytes / 8]uint64
		c.fastMu.RLock()
		_, status := c.fast.request.read(words[:], scratch[:])
		c.fastMu.RUnlock()
		if status == sharedWordsOK {
			var encoded [GrantRequestBytes]byte
			if sharedWordsToLE(encoded[:], words[:]) != sharedWordsOK {
				t.Fatal("request codec")
			}
			request, ok := DecodeGrantRequest(encoded[:])
			if ok && request.Ticket == ticket {
				pollResult = request
				return true
			}
		}
		if time.Now().After(deadline) {
			t.Fatal("grant request was not published")
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func grantAcceptedObservation(request GrantRequest) GrantObservation {
	return GrantObservation{Ticket: request.Ticket, ConnectionId: request.ConnectionId, NativeGeneration: request.NativeGeneration,
		AppGrantGeneration: request.AppGrantGeneration, CapabilityId: request.CapabilityId, DeadlineHostNs: request.DeadlineHostNs,
		ConfigurationEpoch: 5, HomeEpoch: 7, GrantActive: 1, AxisMask: request.AxisMask}
}

func TestApplicationGrantUncertainPublicationRequiresAcknowledgedStop(t *testing.T) {
	c := grantLocalClient(t)
	identity := GrantIdentity{CapabilityID: [16]byte{4}, Generation: 1}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	completed := make(chan error, 1)
	go func() {
		_, err := c.MirrorAcquire(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease)
		completed <- err
	}()
	first := grantLocalRequest(t, c, 1)
	cancel() // Publication is already visible; there is deliberately no receipt.
	err := <-completed
	var uncertain *GrantUncertainError
	if !errors.As(err, &uncertain) || !errors.Is(err, context.Canceled) {
		t.Fatalf("lost receipt reported as rejection: %v", err)
	}
	identity.Generation++
	_, err = c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	if !errors.As(err, &uncertain) {
		t.Fatalf("uncertain publication reused without Stop: %v", err)
	}
	if current := grantLocalRequest(t, c, 1); current != first {
		t.Fatal("a later grant overwrote the unresolved request")
	}
	// Exercise the local acknowledged-Stop fence. Native Stop delivery itself is
	// covered by the daemon cancellation tests; this isolated peer provides it.
	c.stopAck.Store(71)
	grantLocalObservation(t, c, GrantObservation{ConnectionId: 17, NativeGeneration: 12, ConfigurationEpoch: 5, HomeEpoch: 7, StopAckId: 71})
	go func() {
		_, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
		completed <- err
	}()
	second := grantLocalRequest(t, c, 2)
	if second.NativeGeneration != 12 {
		t.Fatal("recovery used the prior native generation")
	}
	grantLocalObservation(t, c, grantAcceptedObservation(second))
	if err := <-completed; err != nil {
		t.Fatalf("explicit reacquire after Stop failed: %v", err)
	}
}

func TestApplicationGrantReceiptMustStillDescribeUsableAuthority(t *testing.T) {
	for _, test := range []struct {
		name     string
		change   func(*GrantObservation)
		rejected bool
	}{
		{"explicit-rejection-retains-other-identity", func(s *GrantObservation) {
			s.GrantReason = GrantReasonBusy
			s.CapabilityId = [16]byte{9}
			s.AppGrantGeneration = 9
		}, true},
		{"accepted-then-revoked", func(s *GrantObservation) { s.GrantActive = 0; s.NativeGeneration++ }, false},
		{"wrong-capability", func(s *GrantObservation) { s.CapabilityId = [16]byte{9} }, false},
		{"wrong-deadline", func(s *GrantObservation) { s.DeadlineHostNs++ }, false},
		{"advanced-ticket", func(s *GrantObservation) { s.Ticket++ }, false},
		{"malformed-receipt", func(s *GrantObservation) { s.Reserved = 1 }, false},
	} {
		t.Run(test.name, func(t *testing.T) {
			c := grantLocalClient(t)
			completed := make(chan error, 1)
			go func() {
				_, err := c.MirrorAcquire(context.Background(), GrantIdentity{CapabilityID: [16]byte{4}, Generation: 1}, HostMonotonicNS()+applicationGrantMaxLease)
				completed <- err
			}()
			request := grantLocalRequest(t, c, 1)
			observation := grantAcceptedObservation(request)
			test.change(&observation)
			grantLocalObservation(t, c, observation)
			err := <-completed
			var rejected *GrantRejectedError
			var uncertain *GrantUncertainError
			if test.rejected {
				if !errors.As(err, &rejected) || errors.As(err, &uncertain) {
					t.Fatalf("explicit rejection not distinguished: %v", err)
				}
			} else if !errors.As(err, &uncertain) {
				t.Fatalf("unusable authority accepted/rejected definitively: %v", err)
			}
			if c.admittedGrant != (GrantReceipt{}) {
				t.Fatal("failed request retained renewal authority")
			}
		})
	}
}

func TestApplicationGrantCloseInterruptsPublishedRequest(t *testing.T) {
	c := grantLocalClient(t)
	completed := make(chan error, 1)
	go func() {
		_, err := c.MirrorAcquire(context.Background(), GrantIdentity{CapabilityID: [16]byte{4}, Generation: 1}, HostMonotonicNS()+applicationGrantMaxLease)
		completed <- err
	}()
	_ = grantLocalRequest(t, c, 1)
	if err := c.Close(); err != nil {
		t.Fatal(err)
	}
	select {
	case err := <-completed:
		var uncertain *GrantUncertainError
		if !errors.As(err, &uncertain) || !errors.Is(err, ErrClosed) {
			t.Fatalf("closed pending request: %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("pending grant operation survived Close")
	}
}

func TestApplicationGrantStopFencesRequestWaitingBeforeObservation(t *testing.T) {
	c := applicationGrantNativeClient(t)
	initial := waitGrantObservation(t, c, func(s GrantObservation) bool { return s.GrantActive == 0 })
	// Capture the public-entry inputs explicitly, then hold its serialization
	// boundary. This fixes the interleaving without a production sleep/hook.
	generation, origin, stopAck := c.revocation.Load(), HostMonotonicNS(), c.stopAck.Load()
	c.grantMu.Lock()
	completed := make(chan error, 1)
	go func() {
		_, err := c.mirrorGrant(context.Background(), GrantOperationAcquire, GrantIdentity{CapabilityID: [16]byte{4}, Generation: 1}, origin, origin+applicationGrantMaxLease, generation, stopAck)
		completed <- err
	}()
	_, stopErr := c.Stop(context.Background())
	c.grantMu.Unlock()
	if stopErr != nil {
		t.Fatal(stopErr)
	}
	if err := <-completed; !errors.Is(err, ErrRevoked) {
		t.Fatalf("pre-Stop request adopted post-Stop native identity: %v", err)
	}
	final := waitGrantObservation(t, c, func(s GrantObservation) bool { return s.NativeGeneration != initial.NativeGeneration })
	c.fastMu.RLock()
	var words, scratch [GrantRequestBytes / 8]uint64
	_, status := c.fast.request.read(words[:], scratch[:])
	c.fastMu.RUnlock()
	if status != sharedWordsUnpublished || final.GrantActive != 0 || c.grantTicket != 0 {
		t.Fatalf("pre-Stop grant was published after Stop: status=%d snapshot=%+v ticket=%d", status, final, c.grantTicket)
	}
}

func TestApplicationGrantStopPreventsCachingLateReceipt(t *testing.T) {
	c := grantLocalClient(t)
	completed := make(chan error, 1)
	go func() {
		_, err := c.MirrorAcquire(context.Background(), GrantIdentity{CapabilityID: [16]byte{4}, Generation: 1}, HostMonotonicNS()+applicationGrantMaxLease)
		completed <- err
	}()
	request := grantLocalRequest(t, c, 1)
	if err := c.disarmUrgent(); err != nil {
		t.Fatal(err)
	}
	grantLocalObservation(t, c, grantAcceptedObservation(request))
	err := <-completed
	var uncertain *GrantUncertainError
	if !errors.As(err, &uncertain) || !errors.Is(err, ErrRevoked) || c.admittedGrant != (GrantReceipt{}) {
		t.Fatalf("pre-Stop receipt became renewable authority: %v, cached=%+v", err, c.admittedGrant)
	}
}

func TestApplicationGrantAcquireWaitsForMatchingStopObservation(t *testing.T) {
	c := grantLocalClient(t)
	c.stopAck.Store(71)
	// The timestamp is current, but this observation predates the acknowledged
	// Stop. A new Acquire must not borrow its older native generation.
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Millisecond)
	defer cancel()
	_, err := c.MirrorAcquire(ctx, GrantIdentity{CapabilityID: [16]byte{4}, Generation: 1}, HostMonotonicNS()+applicationGrantMaxLease)
	var uncertain *GrantUncertainError
	if !errors.Is(err, context.DeadlineExceeded) || errors.As(err, &uncertain) {
		t.Fatalf("stale Stop observation used for publication: %v", err)
	}
	var words, scratch [GrantRequestBytes / 8]uint64
	_, status := c.fast.request.read(words[:], scratch[:])
	if status != sharedWordsUnpublished || c.grantTicket != 0 {
		t.Fatal("request published before matching Stop observation")
	}
}

func TestApplicationGrantStopBeforeUncertainOutcomeAllowsFreshAcquire(t *testing.T) {
	c := grantLocalClient(t)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	completed := make(chan error, 1)
	go func() {
		_, err := c.MirrorAcquire(ctx, GrantIdentity{CapabilityID: [16]byte{4}, Generation: 1}, HostMonotonicNS()+applicationGrantMaxLease)
		completed <- err
	}()
	_ = grantLocalRequest(t, c, 1)
	c.stopAck.Store(71) // The request was published before this acknowledgement.
	cancel()
	if err := <-completed; !errors.Is(err, context.Canceled) {
		t.Fatal(err)
	}
	grantLocalObservation(t, c, GrantObservation{ConnectionId: 17, NativeGeneration: 12, ConfigurationEpoch: 5, HomeEpoch: 7, StopAckId: 71})
	go func() {
		_, err := c.MirrorAcquire(context.Background(), GrantIdentity{CapabilityID: [16]byte{4}, Generation: 2}, HostMonotonicNS()+applicationGrantMaxLease)
		completed <- err
	}()
	request := grantLocalRequest(t, c, 2)
	observation := grantAcceptedObservation(request)
	observation.StopAckId = 71
	grantLocalObservation(t, c, observation)
	if err := <-completed; err != nil {
		t.Fatalf("already acknowledged Stop required a redundant Stop: %v", err)
	}
}

func TestApplicationGrantRenewAndExpiryDuringNativeHome(t *testing.T) {
	// Retain the configured Home transaction, including its native helper waits;
	// skip unrelated startup reads of the not-yet-written simulated home object.
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	c := startCoreWithRole(t, "simulation-home.json", false).client
	identity := GrantIdentity{CapabilityID: [16]byte{8}, Generation: 1}
	acquired, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil {
		t.Fatal(err)
	}
	sequence, err := c.Home(context.Background(), 1)
	if err != nil {
		t.Fatal(err)
	}
	active := wait(t, c, func(s Status) bool {
		return s.CommissioningPhase == 1 && s.NativeHomeActiveMask == 1 && s.HomeValidMask == 0
	})
	before := waitGrantObservation(t, c, func(s GrantObservation) bool { return s.GrantActive == 1 && s.HomeEpoch == active.HomeEpoch })
	// A short explicit renewal makes expiry occur while this Home job is still
	// pending. It is a test input, not a remote-jog or hardware timing budget.
	renewed, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+uint64(20*time.Millisecond))
	if err != nil {
		t.Fatal(err)
	}
	after := waitGrantObservation(t, c, func(s GrantObservation) bool {
		return s.Ticket == renewed.RequestSequence && s.NowHostNs > before.NowHostNs
	})
	c.stateMu.Lock()
	_, homeAlreadyReturned := c.results[sequence]
	c.stateMu.Unlock()
	if homeAlreadyReturned || after.GrantActive == 0 {
		t.Fatal("Home was no longer pending when the independent renewal completed")
	}
	expired := waitGrantObservation(t, c, func(s GrantObservation) bool {
		return s.GrantActive == 0 && s.NativeGeneration != acquired.NativeGeneration
	})
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	result, err := c.WaitResult(ctx, sequence)
	var rejected *RejectedError
	if !errors.As(err, &rejected) {
		t.Fatalf("expired native Home later completed: result=%+v error=%v", result, err)
	}
	helperFinished := HostMonotonicNS()
	final := wait(t, c, func(s Status) bool {
		return s.TimeNS >= helperFinished && s.CommissioningPhase == 0 && s.NativeHomeActiveMask == 0
	})
	if final.HomeValidMask != 0 || final.Armed != 0 || final.AxisEnableMask != 0 ||
		final.Axes[0].Statusword&0x006f != 0x0040 || c.Err() != nil {
		t.Fatalf("expired Home retained service/home authority or lost heartbeat connection: %+v error=%v", final, c.Err())
	}
	t.Logf("renewal ticket=%d and fast observation advanced during pending Home; expiry advanced native generation %d -> %d; final home_valid=%#x statusword=%#x",
		renewed.RequestSequence, acquired.NativeGeneration, expired.NativeGeneration, final.HomeValidMask, final.Axes[0].Statusword)
	identity.Generation = 2
	if _, err = c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
		t.Fatal(err)
	}
	// Unverified renewal cadence: 50000000 ns; the successor must complete Home.
	stopRenew := simfixture.Renew(context.Background(), 50*time.Millisecond, func() bool {
		_, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease)
		return err == nil
	})
	defer stopRenew()
	next, err := c.Home(context.Background(), 1)
	if err != nil {
		t.Fatal(err)
	}
	if result := commissioningResult(t, c, next); result.Result != ResultAccepted {
		t.Fatalf("Home after lease expiry retained busy claim: %+v", result)
	}
	wait(t, c, func(s Status) bool { return s.HomeEpoch == 3 && s.HomeValidMask == 1 && s.CommissioningPhase == 0 })
}
