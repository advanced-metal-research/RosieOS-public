//go:build linux

package main

import (
	"context"
	"crypto/tls"
	"errors"
	"flag"
	"fmt"
	"io"
	"net"
	"net/http"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
)

func run() error {
	coreSocket := flag.String("core-socket", "/run/rosie-rt-core/ipc.sock", "native core socket")
	socket := flag.String("socket", "/run/rosie-rt-core/control.sock", "protected application socket")
	hash := flag.String("configuration-sha256", "", "independently compiled configuration identity")
	compiled := flag.String("compiled-config", os.Getenv("ROSIE_RT_COMPILED_CONFIG"), "compiled digest directory containing resources.json")
	backend := flag.String("backend", "ethercat", "ethercat or simulation")
	pair := flag.String("pair-id", "", "deployment pair identity")
	revision := flag.Uint64("pair-revision", 0, "deployment pair revision")
	remoteListen := flag.String("remote-listen", "", "owned mutual-TLS remote host:port; empty disables remote access")
	remoteCA := flag.String("remote-ca", "", "component-owned remote client CA PEM; required for remote listener")
	remoteCert := flag.String("remote-cert", "", "remote server certificate PEM; required for remote listener")
	remoteKey := flag.String("remote-key", "", "remote server private key PEM; required for remote listener")
	remoteCRL := flag.String("remote-crl", "", "component-owned CRL PEM, checked every handshake; required for remote listener")
	uncertainty := flag.Uint64("remote-jog-max-uncertainty-ns", 0, "unverified maximum offset interval width in ns; zero disables remote jog qualification")
	drift := flag.Uint64("remote-jog-drift-ppb", 0, "unverified remote clock drift bound in ppb; zero disables qualification")
	calibrationAge := flag.Uint64("remote-jog-calibration-max-age-ns", 0, "unverified maximum calibration age in ns; zero disables qualification")
	flag.Parse()
	var remoteTLS *tls.Config
	if *remoteListen != "" {
		var err error
		remoteTLS, err = control.NewRemoteTLSConfig(control.RemoteTLSFiles{CA: *remoteCA, Certificate: *remoteCert, Key: *remoteKey, CRL: *remoteCRL, PairID: *pair})
		if err != nil {
			return &remoteStartupError{err}
		}
	}
	// Claim the public endpoints before native bootstrap so a competing adapter
	// is distinguished from the daemon draining a crashed predecessor.
	owner, err := control.ClaimSocket(*socket, "unix")
	if err != nil {
		return err
	}
	defer owner.Close()
	jogSocket := filepath.Join(filepath.Dir(*socket), "jog.sock")
	jogOwner, err := control.ClaimSocket(jogSocket, "unixgram")
	if err != nil {
		return err
	}
	defer jogOwner.Close()
	options := ipcclient.Options{SocketPath: *coreSocket, ConfigurationSHA256: *hash, Backend: *backend}
	core, err := dialCoreWithRetry(options)
	if err != nil {
		return err
	}
	defer core.Close()
	controller, err := control.New(core, control.Binding{PairID: *pair, Revision: *revision, ConfigurationSHA256: *hash}, *compiled)
	if err != nil {
		return err
	}
	defer controller.Close()
	// A fixed path: rt-control reads no environment outside its host-identity
	// bindings (tools/rtctl TestHostIdentityRuntimeEnvironmentConsumers).
	controller.SetMaintenanceFile(control.DefaultMaintenanceFile)
	listener, err := net.ListenUnix("unix", &net.UnixAddr{Name: *socket, Net: "unix"})
	if err != nil {
		return err
	}
	defer listener.Close()
	listener.SetUnlinkOnClose(false)
	if err := owner.Bound(); err != nil {
		return err
	}
	if err := os.Chmod(*socket, 0660); err != nil {
		return err
	}
	jogListener, err := control.ListenJog(jogSocket)
	if err != nil {
		return err
	}
	defer jogListener.Close()
	if err := jogOwner.Bound(); err != nil {
		return err
	}
	handler := control.NewHandler(controller)
	server := &http.Server{Handler: handler, ReadHeaderTimeout: 2 * time.Second, ReadTimeout: 30 * time.Second, WriteTimeout: 65 * time.Second, IdleTimeout: controlIdleTimeout(controller.Describe())}
	stopping, stop := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer stop()
	controller.ReconnectNative(stopping, options)
	if remoteTLS != nil {
		remote, err := net.Listen("tcp", *remoteListen)
		if err != nil {
			return &remoteStartupError{err}
		}
		defer remote.Close()
		limits := control.JogClockLimits{Qualified: *uncertainty > 0 && *drift > 0 && *calibrationAge > 0,
			MaxInputAgeNS: controller.Describe().MaxJogInputAgeNS, MaxInputDurationNS: controller.Describe().MaxJogInputAgeNS,
			MaxUncertaintyNS: *uncertainty, DriftPPB: *drift, MaxCalibrationAgeNS: *calibrationAge}
		remoteHandler := control.NewRemoteHandler(controller, handler, limits)
		defer remoteHandler.Close()
		remoteServer := newRemoteServer(remoteHandler, controller.Describe())
		defer remoteServer.Close()
		go func() {
			if err := remoteServer.Serve(control.NewRemoteListener(remote, remoteTLS)); err != nil && !errors.Is(err, http.ErrServerClosed) && !errors.Is(err, net.ErrClosed) {
				fmt.Fprintln(os.Stderr, "remote listener:", err)
				stop()
			}
		}()
	}
	go func() {
		if err := controller.ServeJog(jogListener); err != nil && !errors.Is(err, net.ErrClosed) {
			fmt.Fprintln(os.Stderr, "jog listener:", err)
			stop()
		}
	}()
	go func() { <-stopping.Done(); _ = controller.Close(); _ = server.Close() }()
	fmt.Fprintln(os.Stderr, "rt-control listening on", *socket)
	err = server.Serve(listener)
	if err == http.ErrServerClosed {
		return nil
	}
	return err
}

type remoteStartupError struct{ error }

// Measured Sydney-to-LA TLS setup through two SSH hops: 1100–2900 ms;
// the previous 2000 ms bound rejected valid handshakes. Internet uses 10000 ms.
const internetHeaderTimeout = 10 * time.Second

// Unverified existing LAN header/handshake bound: 2000 ms.
const lanHeaderTimeout = 2 * time.Second

// newRemoteServer follows the compiled link timing contract. The compiler
// normalizes profile labels to effective ceilings; retain LAN's bound only for
// the exact LAN defaults (or legacy omitted defaults). Any custom timing uses
// the internet bound so an override cannot accidentally restore the short one.
func newRemoteServer(handler http.Handler, description control.Description) *http.Server {
	headerTimeout := internetHeaderTimeout
	if (description.MaxGrantLeaseNS == 0 || description.MaxGrantLeaseNS == 500000000) &&
		(description.MaxJogInputAgeNS == 0 || description.MaxJogInputAgeNS == 250000000) {
		headerTimeout = lanHeaderTimeout
	}
	// Unverified existing whole-request bounds: 30000 ms read, 65000 ms write.
	return &http.Server{Handler: handler, ReadHeaderTimeout: headerTimeout, ReadTimeout: 30 * time.Second, WriteTimeout: 65 * time.Second, IdleTimeout: controlIdleTimeout(description)}
}

// controlIdleTimeout uses the same compiled value advertised to clients,
// preventing listener policy from drifting away from reservation maintenance.
func controlIdleTimeout(description control.Description) time.Duration {
	if description.ControlIdleTimeoutNS == 0 {
		return time.Duration(control.CompiledControlIdleTimeoutNS)
	}
	return time.Duration(description.ControlIdleTimeoutNS)
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		if errors.Is(err, control.ErrSocketOwned) {
			os.Exit(3)
		}
		var startup *remoteStartupError
		if errors.As(err, &startup) {
			os.Exit(2)
		}
		os.Exit(1)
	}
}

// coreBootstrapWindow bounds how long the adapter waits for the native
// bootstrap. The daemon publishes its socket only after startup readbacks: a
// nine-axis A6-EC daemon took ~25 s (LA bench, 2026-11-01), past the old 5 s
// window, so the adapter exited and systemd restarted it. Owner decision
// 2026-11-01: wait, bounded at 60 s (unverified margin, not a measurement).
const coreBootstrapWindow = 60 * time.Second

// coreBootstrapRetry is the pause between bootstrap attempts. Unverified.
const coreBootstrapRetry = 20 * time.Millisecond

// dialCoreWithRetry retries unpublished diagnostics and transport-level failures (socket not yet
// bound, connection refused, or the daemon closing the socket before WELCOME
// because another controller still occupies its single slot). Validation and
// handshake rejections carrying a reason return immediately so a misconfigured
// adapter fails closed instead of looping.
func dialCoreWithRetry(options ipcclient.Options) (*ipcclient.Client, error) {
	deadline := time.Now().Add(coreBootstrapWindow)
	for attempt := 0; ; attempt++ {
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		core, err := ipcclient.Dial(ctx, options)
		cancel()
		if err == nil {
			if attempt > 0 {
				fmt.Fprintf(os.Stderr, "rt-control: native bootstrap succeeded after %d retries\n", attempt)
			}
			return core, nil
		}
		if !transientBootstrapError(err) || !time.Now().Before(deadline) {
			return nil, err
		}
		if attempt == 0 {
			fmt.Fprintf(os.Stderr, "rt-control: native bootstrap not ready, retrying: %v\n", err)
		}
		time.Sleep(coreBootstrapRetry)
	}
}

func transientBootstrapError(err error) bool {
	if errors.Is(err, ipcclient.ErrDiagnosticsNotPublished) {
		return true
	}
	if errors.Is(err, io.EOF) || errors.Is(err, io.ErrUnexpectedEOF) {
		return true
	}
	var errno syscall.Errno
	if errors.As(err, &errno) {
		switch errno {
		case syscall.ECONNREFUSED, syscall.ENOENT, syscall.ECONNRESET, syscall.EPIPE, syscall.EAGAIN:
			return true
		}
	}
	var opErr *net.OpError
	return errors.As(err, &opErr) && opErr.Op != "dial" && opErr.Timeout()
}
