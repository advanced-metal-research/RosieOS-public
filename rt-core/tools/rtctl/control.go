//go:build linux

package rtctl

import (
	"context"
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/signal"
	"strings"
	"time"

	"rosieos/rt-core/sdk/control"
)

func IsControlCommand(command string) bool {
	switch command {
	case "resources", "events", "describe", "status", "acquire", "release", "stop", "halt", "enable", "arm", "home", "reset-fault", "prepare", "start", "discard", "program-prepare", "program-start":
		return true
	}
	return false
}

// Long fenced commands own a lease keeper while their HTTP request is in
// flight. Stop and Release remain unrenewed terminal fences.
func controlCommandNeedsRenewal(command string) bool {
	switch command {
	case "home", "reset-fault", "halt", "prepare", "start", "program-prepare", "program-start":
		return true
	}
	return false
}

// RunControlCLI executes one public operation and emits its result. It retains
// shell authority on exit; only release relinquishes the explicitly supplied fence.
func RunControlCLI(args []string, stdout, stderr io.Writer) int {
	if len(args) == 0 || !IsControlCommand(args[0]) {
		fmt.Fprintln(stderr, "unknown control command")
		return 2
	}
	flags := flag.NewFlagSet(args[0], flag.ContinueOnError)
	flags.SetOutput(stderr)
	resourceSHA := ""
	if args[0] == "resources" {
		if len(args) < 3 || args[1] != "fetch" {
			fmt.Fprintln(stderr, "usage: rtctl resources fetch <definition-sha256> --out DIR [--socket PATH | --remote URL --ca FILE --cert FILE --key FILE]")
			return 2
		}
		resourceSHA = args[2]
		args = append([]string{args[0]}, args[3:]...)
	}
	output := flags.String("out", "", "directory for verified content-addressed robot resources")
	socket := flags.String("socket", "", "Unix control socket")
	remote := flags.String("remote", "", "mutual-TLS HTTPS origin")
	cert := flags.String("cert", "", "client certificate PEM")
	key := flags.String("key", "", "client private key PEM")
	ca := flags.String("ca", "", "server CA PEM")
	session := flags.String("session", "", "acquired session token")
	generation := flags.Uint64("generation", 0, "acquired control generation")
	asJSON := flags.Bool("json", false, "compact JSON output")
	after := flags.Uint64("after", 0, "last handled event sequence")
	follow := flags.Bool("follow", false, "stream events until interrupted")
	controller := flags.String("controller", "rtctl", "controller label")
	pair := flags.String("pair-id", "", "deployment pair identity")
	revision := flags.Uint64("pair-revision", 0, "deployment pair revision")
	hash := flags.String("configuration-sha256", "", "compiled configuration identity")
	mask := flags.Uint("axis-mask", 0, "axis bit mask")
	handle := flags.Uint64("handle", 0, "prepared trajectory handle")
	file := flags.String("file", "", "points JSON, identity JSON or binary .rdt file")
	requestID := flags.String("request-id", "", "reuse only for an identical retained request retry")
	// Unverified operator request budget: 10 seconds; SDK callers own their contexts.
	timeout := flags.Duration("timeout", 10*time.Second, "HTTP operation deadline")
	if err := flags.Parse(args[1:]); err != nil {
		return 2
	}
	fail := func(err error) int {
		fmt.Fprintln(stderr, err)
		var rejected *control.Rejected
		if errors.As(err, &rejected) {
			return 2
		}
		if errors.Is(err, control.ErrTransport) || errors.Is(err, control.ErrConnectionUnsent) || errors.Is(err, control.ErrProtocol) || errors.Is(err, control.ErrClosed) {
			return 3
		}
		return 2
	}
	if flags.NArg() != 0 || (*socket != "" && *remote != "") || *timeout <= 0 || uint64(*mask) > 0xffffffff {
		return fail(errors.New("invalid control options"))
	}
	if *remote == "" && (*cert != "" || *key != "" || *ca != "") {
		return fail(errors.New("TLS credentials require --remote"))
	}
	var client *control.Client
	var err error
	if *remote != "" {
		certificate, e := tls.LoadX509KeyPair(*cert, *key)
		if e != nil {
			return fail(e)
		}
		pem, e := os.ReadFile(*ca)
		if e != nil {
			return fail(e)
		}
		roots := x509.NewCertPool()
		if !roots.AppendCertsFromPEM(pem) {
			return fail(errors.New("invalid CA PEM"))
		}
		client, err = control.DialTLS(*remote, &tls.Config{Certificates: []tls.Certificate{certificate}, RootCAs: roots})
	} else {
		if *socket == "" {
			*socket = "/run/rosie-rt-core/control.sock"
		}
		client, err = control.Dial(control.UnixPath(*socket))
	}
	if err != nil {
		return fail(err)
	}
	defer client.Close()
	client.SetFence(control.Fence{Session: *session, Generation: *generation})
	ctx, cancel := context.WithTimeout(context.Background(), *timeout)
	defer cancel()
	if *session != "" && controlCommandNeedsRenewal(args[0]) {
		if _, err = client.StartRenewal(ctx, 0); err != nil {
			return fail(err)
		}
		defer client.StopRenewal()
	}
	var result any
	switch args[0] {
	case "resources":
		result, err = fetchResources(ctx, client, resourceSHA, *output)
	case "describe":
		result, err = client.Describe(ctx)
	case "status":
		result, err = client.Status(ctx)
	case "events":
		if *follow {
			streamContext, stop := signal.NotifyContext(context.Background(), os.Interrupt)
			defer stop()
			err = client.StreamEvents(streamContext, *after, func(event control.Event) error { return json.NewEncoder(stdout).Encode(event) })
			if errors.Is(err, context.Canceled) {
				return 0
			}
			if err != nil {
				return fail(err)
			}
			return 0
		}
		result, err = client.EventsContext(ctx, *after)
	case "program-prepare":
		var blob []byte
		blob, err = os.ReadFile(*file)
		if err == nil {
			result, err = client.PrepareProgram(ctx, blob)
		}
	default:
		operation := strings.ReplaceAll(args[0], "-", "_")
		switch operation {
		case "prepare":
			operation = "prepare_trajectory"
		case "start":
			operation = "start_trajectory"
		case "discard":
			operation = "discard_trajectory"
		case "program_start":
			operation = "start_program"
		}
		request := control.Request{Operation: operation, AxisMask: uint32(*mask), Handle: *handle}
		if *requestID != "" {
			request.RequestID = requestID
			request.Fence = client.Fence()
		}
		if operation == "acquire" {
			request.Controller = *controller
			request.Binding = control.Binding{PairID: *pair, Revision: *revision, ConfigurationSHA256: *hash}
			request.Fence = control.Fence{}
		}
		if operation == "prepare_trajectory" || operation == "start_program" {
			var raw []byte
			raw, err = os.ReadFile(*file)
			if err == nil {
				if operation == "prepare_trajectory" {
					err = json.Unmarshal(raw, &request.Points)
				} else {
					err = json.Unmarshal(raw, &request.Identity)
				}
			}
		}
		if err == nil {
			result, err = client.Command(ctx, &request)
		}
	}
	if err != nil {
		return fail(err)
	}
	encoder := json.NewEncoder(stdout)
	if !*asJSON {
		encoder.SetIndent("", "  ")
	}
	if err = encoder.Encode(result); err != nil {
		return fail(err)
	}
	return 0
}
