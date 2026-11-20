//go:build linux

package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/tools/natspublisher/publisher"
)

var errConfiguration = errors.New("invalid publisher configuration")

// Unverified process exit contract: distinct permanent failures prevent service
// restart loops while ordinary process failures retain on-failure recovery.
func exitCode(err error) int {
	for _, entry := range []struct {
		err  error
		code int
	}{
		{errConfiguration, 2}, {publisher.ErrContract, 10}, {publisher.ErrAuthRequired, 11},
		{publisher.ErrTLSRequired, 12}, {publisher.ErrJetStreamUnavailable, 13},
		{publisher.ErrStreamConfig, 14}, {publisher.ErrPublisherActive, 15},
	} {
		if errors.Is(err, entry.err) {
			return entry.code
		}
	}
	if err == nil {
		return 0
	}
	return 1
}

func run(ctx context.Context, args []string) error {
	flags := flag.NewFlagSet("rt-natspublisher", flag.ContinueOnError)
	flags.SetOutput(io.Discard)
	// Unverified deployment default: installed public socket; no native IPC access.
	socket := flags.String("socket", "/run/rosie-rt-core/public/control.sock", "public rt-control Unix socket")
	rawURL := flags.String("nats-url", os.Getenv("ROSIE_RT_NATS_URL"), "deployment NATS URL")
	host := flags.String("host", os.Getenv("ROSIE_RT_NATS_HOST"), "deployment's concrete v4 peer identity")
	defaultCadence := os.Getenv("ROSIE_RT_NATS_CADENCE")
	if defaultCadence == "" {
		defaultCadence = publisher.DefaultCadence.String()
	}
	cadenceText := flags.String("cadence", defaultCadence, "status and batch interval, in whole milliseconds")
	telemetryText := flags.String("telemetry-cadence", os.Getenv("ROSIE_RT_NATS_TELEMETRY_CADENCE"), "telemetry interval; defaults to status cadence to amortize batch overhead")
	retention := publisher.DefaultTelemetryMaxBytes
	if value := os.Getenv("ROSIE_RT_NATS_TELEMETRY_MAX_BYTES"); value != "" {
		var err error
		retention, err = strconv.ParseInt(value, 10, 64)
		if err != nil {
			return fmt.Errorf("%w: invalid telemetry byte retention", errConfiguration)
		}
	}
	telemetryMaxBytes := flags.Int64("telemetry-max-bytes", retention, "rt-core telemetry JetStream byte retention")
	printEnv := flags.Bool("print-env", false, "validate and emit the systemd deployment environment")
	if err := flags.Parse(args); err != nil {
		return fmt.Errorf("%w: %v", errConfiguration, err)
	}
	if flags.NArg() != 0 {
		return fmt.Errorf("%w: unexpected positional arguments", errConfiguration)
	}
	cadence, err := time.ParseDuration(*cadenceText)
	if err != nil || cadence <= 0 {
		return fmt.Errorf("%w: cadence must be a positive duration", errConfiguration)
	}
	config := publisher.Config{URL: *rawURL, Host: *host, Cadence: cadence}
	config.TelemetryMaxBytes = *telemetryMaxBytes
	if *telemetryText != "" {
		config.TelemetryCadence, err = time.ParseDuration(*telemetryText)
		if err != nil || config.TelemetryCadence <= 0 {
			return fmt.Errorf("%w: invalid telemetry cadence", errConfiguration)
		}
	}
	if err := config.Validate(); err != nil {
		return fmt.Errorf("%w: %v", errConfiguration, err)
	}
	if *printEnv {
		fmt.Printf("ROSIE_RT_NATS_URL=%s\nROSIE_RT_NATS_HOST=%s\nROSIE_RT_NATS_CADENCE=%s\n", strconv.Quote(config.URL), strconv.Quote(config.Host), strconv.Quote(config.Cadence.String()))
		telemetryCadence := config.TelemetryCadence
		if telemetryCadence == 0 {
			telemetryCadence = config.Cadence
		}
		fmt.Printf("ROSIE_RT_NATS_TELEMETRY_CADENCE=%s\nROSIE_RT_NATS_TELEMETRY_MAX_BYTES=%d\n", strconv.Quote(telemetryCadence.String()), config.TelemetryMaxBytes)
		return nil
	}
	client, err := control.Dial(control.UnixPath(*socket))
	if err != nil {
		return err
	}
	defer client.Close()
	p, err := publisher.New(client, config)
	if err != nil {
		return err
	}
	return p.Run(ctx)
}
func main() {
	ctx, cancel := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer cancel()
	if err := run(ctx, os.Args[1:]); err != nil {
		code := exitCode(err)
		fmt.Fprintf(os.Stderr, "component=rt-natspublisher event=exit publisher_state=refused exit_code=%d last_error=%q\n", code, err.Error())
		os.Exit(code)
	}
}
