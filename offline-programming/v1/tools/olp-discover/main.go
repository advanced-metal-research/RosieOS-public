// olp-discover lists the OLP servers advertising on this machine's network,
// the search the OLP server runs for the Cells menu, from a workstation (the
// OLP server itself builds on Linux only; this builds anywhere).
//
//	go run ./tools/olp-discover --interface en0 [--tailscale] [--json]
//
// --tailscale asks every online tailnet peer's advertisement-only listener
// (port --tailnet-port, 8798 on the fleet), as an OLP server does.
//
// --advertise-sample answers as a sample server with one made-up cell
// ("sample-cell", which no manifest knows) until interrupted, by mDNS on
// --interface and/or, with --tailnet, on this machine's tailnet address: a
// bench check that a search reaches it, never a stand-in for a cell.
package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"os"
	"os/signal"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/discovery"
)

func main() {
	if err := run(os.Args[1:], os.Stdout, os.Stderr); err != nil {
		fmt.Fprintln(os.Stderr, "olp-discover:", err)
		os.Exit(1)
	}
}

func run(args []string, stdout, stderr io.Writer) error {
	flags := flag.NewFlagSet("olp-discover", flag.ContinueOnError)
	flags.SetOutput(stderr)
	ifaces := flags.String("interface", "", "comma-separated interfaces to search (required for mDNS), e.g. en0")
	tailscale := flags.Bool("tailscale", false, "also ask online Tailscale peers' advertisement listeners")
	port := flags.Int("tailnet-port", discovery.DefaultTailnetPort, "advertisement listener port on tailnet peers (and for --advertise-sample --tailnet)")
	tailnet := flags.Bool("tailnet", false, "with --advertise-sample: also serve it on this machine's tailnet address at --tailnet-port")
	window := flags.Duration("window", discovery.DefaultBrowseWindow, "how long to listen for answers")
	asJSON := flags.Bool("json", false, "print the discovered-cells JSON")
	sample := flags.Bool("advertise-sample", false, "advertise a sample server until interrupted (bench check)")
	if err := flags.Parse(args); err != nil {
		return err
	}
	names := discovery.ParseInterfaceList(*ifaces)
	resolved, err := discovery.ResolveInterfaces(names)
	if err != nil {
		return err
	}
	if *sample {
		return advertiseSample(resolved, *tailnet, *port, stdout)
	}
	if len(resolved) == 0 && !*tailscale {
		return fmt.Errorf("name an --interface (e.g. en0) or pass --tailscale")
	}
	opts := discovery.Options{Interfaces: resolved, Window: *window}
	if *tailscale {
		opts.Tailscale = &discovery.TailscaleOptions{Port: *port}
	}
	ctx, cancel := context.WithTimeout(context.Background(), *window+10*time.Second)
	defer cancel()
	result := discovery.NewService(opts).Search(ctx, true)
	if *asJSON {
		encoder := json.NewEncoder(stdout)
		encoder.SetIndent("", "  ")
		return encoder.Encode(result)
	}
	for _, source := range result.Sources {
		line := fmt.Sprintf("%s: %d found", source.Name, source.Found)
		if len(source.Interfaces) > 0 {
			line += " on " + strings.Join(source.Interfaces, ",")
		}
		if source.Error != "" {
			line += " (" + source.Error + ")"
		}
		fmt.Fprintln(stdout, line)
	}
	for _, s := range result.Servers {
		fmt.Fprintf(stdout, "%s  %s:%d  %s  build %s  reach %s  health %s\n", s.Hostname, s.Address, s.Port, s.Instance, s.BuildID, s.Reach, s.Health)
		for _, c := range s.Cells {
			selected := ""
			if c.Selected {
				selected = "  (selected)"
			}
			fmt.Fprintf(stdout, "  %-28s %-22s %-10s pin %s  caps %s%s\n", c.CellID, c.Model, c.Mode, short(c.ConfigurationSHA256), short(c.CapabilitiesDigest), selected)
		}
	}
	return nil
}

func short(digest string) string {
	if len(digest) < 12 {
		return "—"
	}
	return digest[:12]
}

func advertiseSample(ifaces []discovery.Interface, tailnet bool, port int, stdout io.Writer) error {
	if len(ifaces) == 0 && !tailnet {
		return fmt.Errorf("--advertise-sample needs --interface and/or --tailnet")
	}
	host, _ := os.Hostname()
	record := discovery.Advertisement{
		Schema: discovery.Schema, Instance: discovery.NewInstance(host), Hostname: strings.Split(host, ".")[0],
		Port: 8794, Reach: "loopback", BuildID: "sample", StartedAt: time.Now().UTC().Format(time.RFC3339),
		Cells: []discovery.AdvertisedCell{{CellID: "sample-cell", Label: "Sample (not a cell)", Model: "bench_one_motor_1to1", Mode: "simulation"}},
	}
	logf := func(format string, args ...any) { fmt.Fprintf(stdout, format+"\n", args...) }
	if len(ifaces) > 0 {
		conn, err := discovery.ListenMulticast(ifaces)
		if err != nil {
			return err
		}
		responder := discovery.NewResponder(discovery.ResponderOptions{
			Interfaces: ifaces,
			Record:     func() discovery.Advertisement { return record },
			Logf:       logf,
		})
		go responder.Serve(conn)
		defer responder.Close()
		responder.Announce()
		fmt.Fprintf(stdout, "advertising %s as %s on %d interface(s)\n", discovery.ServiceType, record.Instance, len(ifaces))
	}
	if tailnet {
		listener := &discovery.TailnetListener{Port: port, Record: func() discovery.Advertisement { return record }, Logf: logf, RetryEvery: 2 * time.Second}
		listener.Start()
		defer listener.Close()
	}
	fmt.Fprintln(stdout, "Ctrl-C to stop")
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()
	<-ctx.Done()
	return nil
}
