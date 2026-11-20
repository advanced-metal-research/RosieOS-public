package discovery

import (
	"context"
	"crypto/rand"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"net"
	"strings"
	"sync"
	"time"
)

// mdnsGroup is the IPv4 mDNS group (RFC 6762). IPv6 is not used: every
// operator LAN this serves has IPv4, and one family keeps the socket code small.
var mdnsGroup = &net.UDPAddr{IP: net.IPv4(224, 0, 0, 251), Port: 5353}

const (
	serviceName  = ServiceType + ".local."
	servicesEnum = "_services._dns-sd._udp.local."
	// recordTTL is the TTL of every record this responder sends (RFC 6762
	// section 10 recommends 120 s for records naming a host).
	recordTTL = 120
	// legacyTTL caps TTLs in a reply to a one-shot query (section 6.7).
	legacyTTL = 10
	// DefaultBrowseWindow is how long a search listens for answers. Responders
	// delay multicast answers by 20-120 ms (section 6); unicast answers to a
	// one-shot query come at once. 1.5 s also covers a Wi-Fi retry.
	DefaultBrowseWindow = 1500 * time.Millisecond
)

// Interface is a network interface discovery may use, with its IPv4 address.
type Interface struct {
	Name  string
	Index int
	IPv4  net.IP
	Net   *net.IPNet
}

// ResolveInterfaces looks up the named interfaces. Each must be up, carry
// multicast and an IPv4 address, and not be loopback. There is no automatic
// choice: a cell host's default route can be the link it shares with the
// servo host (a relay: enP8p1s0, 2026-10-08), and nothing may be multicast
// onto that segment by accident.
func ResolveInterfaces(names []string) ([]Interface, error) {
	var out []Interface
	for _, name := range names {
		name = strings.TrimSpace(name)
		if name == "" {
			continue
		}
		iface, err := net.InterfaceByName(name)
		if err != nil {
			return nil, fmt.Errorf("discovery: interface %s: %w", name, err)
		}
		if iface.Flags&net.FlagUp == 0 || iface.Flags&net.FlagMulticast == 0 || iface.Flags&net.FlagLoopback != 0 {
			return nil, fmt.Errorf("discovery: interface %s is not an up, multicast, non-loopback interface", name)
		}
		addrs, err := iface.Addrs()
		if err != nil {
			return nil, fmt.Errorf("discovery: interface %s addresses: %w", name, err)
		}
		found := false
		for _, addr := range addrs {
			if ipnet, ok := addr.(*net.IPNet); ok && ipnet.IP.To4() != nil {
				out = append(out, Interface{Name: iface.Name, Index: iface.Index, IPv4: ipnet.IP.To4(), Net: ipnet})
				found = true
				break
			}
		}
		if !found {
			return nil, fmt.Errorf("discovery: interface %s has no IPv4 address", name)
		}
	}
	return out, nil
}

// NewInstance names this process: host label plus six random hex digits.
func NewInstance(hostname string) string {
	var b [3]byte
	_, _ = rand.Read(b[:])
	label := sanitizeLabel(hostname)
	if len(label) > 48 {
		label = label[:48]
	}
	if label == "" {
		label = "olp"
	}
	return label + "-olp-" + hex.EncodeToString(b[:])
}

// sanitizeLabel keeps a DNS label's safe characters, so names never need escaping.
func sanitizeLabel(s string) string {
	var b strings.Builder
	for _, r := range strings.ToLower(s) {
		if (r >= 'a' && r <= 'z') || (r >= '0' && r <= '9') || r == '-' {
			b.WriteRune(r)
		} else if r == '.' || r == '_' || r == ' ' {
			b.WriteRune('-')
		}
	}
	return strings.Trim(b.String(), "-")
}

// ResponderOptions configures an advertiser.
type ResponderOptions struct {
	Interfaces []Interface
	// Record returns the advertisement now; it is read for every answer, so
	// a selection or a maintenance notice shows in the next reply.
	Record func() Advertisement
	Logf   func(string, ...any)
	// Group overrides the mDNS group; tests point it at a loopback socket.
	Group *net.UDPAddr
}

// Responder answers DNS-SD queries for this server's _rosie-olp._tcp
// instance and announces it. It never answers for any other name.
type Responder struct {
	opts  ResponderOptions
	group *net.UDPAddr
	// sendMu guards conn: Serve sets it on its own goroutine while the caller
	// announces, so an unguarded read races and can skip the first announce.
	sendMu sync.Mutex
	conn   net.PacketConn
	done   chan struct{}
	closed sync.Once
}

func NewResponder(opts ResponderOptions) *Responder {
	group := opts.Group
	if group == nil {
		group = mdnsGroup
	}
	if opts.Logf == nil {
		opts.Logf = func(string, ...any) {}
	}
	return &Responder{opts: opts, group: group, done: make(chan struct{})}
}

func (r *Responder) names(a Advertisement) (instance, host string) {
	return canonical(a.Instance + "." + serviceName), canonical(a.Instance + ".local.")
}

// Serve answers queries read from conn until Close. conn is the shared mDNS
// socket (ListenMulticast) or, in a test, any UDP socket.
func (r *Responder) Serve(conn net.PacketConn) {
	r.sendMu.Lock()
	r.conn = conn
	r.sendMu.Unlock()
	select {
	case <-r.done:
		// Closed before Serve stored conn: Close could not close it.
		_ = conn.Close()
		return
	default:
	}
	buf := make([]byte, maxPacket)
	for {
		n, src, err := conn.ReadFrom(buf)
		if err != nil {
			select {
			case <-r.done:
			default:
				r.opts.Logf("discovery: mdns read: %v", err)
			}
			return
		}
		udp, ok := src.(*net.UDPAddr)
		if !ok {
			continue
		}
		r.handle(buf[:n], udp)
	}
}

func (r *Responder) handle(packet []byte, src *net.UDPAddr) {
	query, err := unpack(packet)
	if err != nil || query.Flags&flagResponse != 0 || len(query.Questions) == 0 {
		return
	}
	a := r.opts.Record()
	if a.Instance == "" {
		return
	}
	instance, host := r.names(a)
	iface := r.interfaceFor(src.IP)
	// The shared socket also hears queries from other links (Linux delivers
	// a group to every socket on the port once any socket joined it). Only
	// a querier on an advertised interface's subnet gets an answer.
	if len(r.opts.Interfaces) > 0 && iface == nil {
		return
	}
	var answers, extra []dnsRecord
	unicast := false
	for _, q := range query.Questions {
		name := canonical(q.Name)
		qtype := q.Type
		if q.Class&classTopBit != 0 {
			unicast = true
		}
		switch {
		case name == serviceName && (qtype == typePTR || qtype == typeANY):
			answers = append(answers, dnsRecord{Name: serviceName, Type: typePTR, Class: classIN, TTL: recordTTL, Target: instance})
			extra = append(extra, r.instanceRecords(a, instance, host, iface)...)
		case name == servicesEnum && (qtype == typePTR || qtype == typeANY):
			answers = append(answers, dnsRecord{Name: servicesEnum, Type: typePTR, Class: classIN, TTL: recordTTL, Target: serviceName})
		case name == instance && (qtype == typeSRV || qtype == typeTXT || qtype == typeANY):
			answers = append(answers, r.instanceRecords(a, instance, host, iface)...)
		case name == host && (qtype == typeA || qtype == typeANY):
			answers = append(answers, r.addressRecords(host, iface)...)
		}
	}
	if len(answers) == 0 {
		return
	}
	reply := dnsMessage{Flags: flagResponse | flagAuth, Answers: answers, Extra: extra}
	legacy := src.Port != mdnsGroup.Port
	if legacy {
		// A one-shot querier (RFC 6762 section 6.7) gets the ID and question
		// back, short TTLs and no cache-flush bits, by unicast.
		reply.ID, reply.Questions = query.ID, query.Questions
		for _, records := range [][]dnsRecord{reply.Answers, reply.Extra} {
			for i := range records {
				records[i].TTL = min(records[i].TTL, legacyTTL)
				records[i].Class &^= classTopBit
			}
		}
	}
	raw, err := reply.pack()
	if err != nil {
		r.opts.Logf("discovery: mdns reply: %v", err)
		return
	}
	if legacy || unicast {
		r.send(raw, src, nil)
		return
	}
	r.multicast(raw, iface)
}

// instanceRecords are the SRV, TXT and A records of the advertised instance.
func (r *Responder) instanceRecords(a Advertisement, instance, host string, iface *Interface) []dnsRecord {
	port := uint16(0)
	if a.Port > 0 && a.Port < 65536 {
		port = uint16(a.Port)
	}
	records := []dnsRecord{
		{Name: instance, Type: typeSRV, Class: classIN | classTopBit, TTL: recordTTL, Port: port, Target: host},
		{Name: instance, Type: typeTXT, Class: classIN | classTopBit, TTL: recordTTL, TXT: a.TXT()},
	}
	return append(records, r.addressRecords(host, iface)...)
}

// addressRecords answer with the address of the interface the query came in
// on, or every advertised interface when that is unknown.
func (r *Responder) addressRecords(host string, iface *Interface) []dnsRecord {
	ifaces := r.opts.Interfaces
	if iface != nil {
		ifaces = []Interface{*iface}
	}
	var records []dnsRecord
	for _, i := range ifaces {
		rec := dnsRecord{Name: host, Type: typeA, Class: classIN | classTopBit, TTL: recordTTL}
		copy(rec.A[:], i.IPv4.To4())
		records = append(records, rec)
	}
	return records
}

func (r *Responder) interfaceFor(ip net.IP) *Interface {
	for i := range r.opts.Interfaces {
		if n := r.opts.Interfaces[i].Net; n != nil && n.Contains(ip) {
			return &r.opts.Interfaces[i]
		}
	}
	return nil
}

func (r *Responder) multicast(raw []byte, iface *Interface) {
	if len(r.opts.Interfaces) == 0 {
		r.send(raw, r.group, nil)
		return
	}
	targets := r.opts.Interfaces
	if iface != nil {
		targets = []Interface{*iface}
	}
	for i := range targets {
		r.send(raw, r.group, &targets[i])
	}
}

func (r *Responder) send(raw []byte, to *net.UDPAddr, iface *Interface) {
	r.sendMu.Lock()
	defer r.sendMu.Unlock()
	if r.conn == nil {
		return
	}
	if iface != nil {
		if err := setMulticastInterface(r.conn, iface.IPv4); err != nil {
			r.opts.Logf("discovery: mdns interface %s: %v", iface.Name, err)
			return
		}
	}
	if _, err := r.conn.WriteTo(raw, to); err != nil {
		r.opts.Logf("discovery: mdns send to %s: %v", to, err)
	}
}

// Announce multicasts the instance unsolicited (RFC 6762 section 8.3), so
// listeners that cache see it without asking.
func (r *Responder) Announce() { r.announce(recordTTL) }

func (r *Responder) announce(ttl uint32) {
	a := r.opts.Record()
	if a.Instance == "" {
		return
	}
	instance, host := r.names(a)
	answers := append([]dnsRecord{{Name: serviceName, Type: typePTR, Class: classIN, TTL: ttl, Target: instance}},
		r.instanceRecords(a, instance, host, nil)...)
	for i := range answers {
		answers[i].TTL = ttl
	}
	raw, err := dnsMessage{Flags: flagResponse | flagAuth, Answers: answers}.pack()
	if err != nil {
		r.opts.Logf("discovery: mdns announce: %v", err)
		return
	}
	r.multicast(raw, nil)
}

// Close sends a goodbye (TTL 0, section 10.1) and stops Serve.
func (r *Responder) Close() {
	r.closed.Do(func() {
		r.announce(0)
		close(r.done)
		r.sendMu.Lock()
		conn := r.conn
		r.sendMu.Unlock()
		if conn != nil {
			_ = conn.Close()
		}
	})
}

// Found is one advertisement heard by a browser.
type Found struct {
	Advertisement
	// Address is the IPv4 source of the answer: where the packet came from,
	// not anything the advertisement claims.
	Address string
	// Dropped counts cells refused as malformed.
	Dropped int
}

// BrowseOptions configures one search.
type BrowseOptions struct {
	Interfaces []Interface
	Window     time.Duration
	// Self is this process's own instance, dropped from the results.
	Self  string
	Group *net.UDPAddr
}

// Browse sends a one-shot DNS-SD query for _rosie-olp._tcp on each interface
// from an ephemeral port and collects the unicast answers for the window. It
// never binds 5353, so it coexists with avahi or mDNSResponder on the host.
func Browse(ctx context.Context, opts BrowseOptions) ([]Found, error) {
	group := opts.Group
	if group == nil {
		group = mdnsGroup
	}
	window := opts.Window
	if window <= 0 {
		window = DefaultBrowseWindow
	}
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{})
	if err != nil {
		return nil, fmt.Errorf("discovery: browse socket: %w", err)
	}
	defer conn.Close()
	var id [2]byte
	_, _ = rand.Read(id[:])
	query, err := dnsMessage{
		ID:        binary.BigEndian.Uint16(id[:]),
		Questions: []dnsQuestion{{Name: serviceName, Type: typePTR, Class: classIN | classTopBit}},
	}.pack()
	if err != nil {
		return nil, err
	}
	sendAll := func() error {
		if len(opts.Interfaces) == 0 {
			_, err := conn.WriteTo(query, group)
			return err
		}
		var errs []error
		for _, iface := range opts.Interfaces {
			if err := setMulticastInterface(conn, iface.IPv4); err != nil {
				errs = append(errs, fmt.Errorf("%s: %w", iface.Name, err))
				continue
			}
			if _, err := conn.WriteTo(query, group); err != nil {
				errs = append(errs, fmt.Errorf("%s: %w", iface.Name, err))
			}
		}
		if len(errs) == len(opts.Interfaces) {
			return errors.Join(errs...)
		}
		return nil
	}
	if err := sendAll(); err != nil {
		return nil, fmt.Errorf("discovery: browse query: %w", err)
	}
	deadline := time.Now().Add(window)
	if d, ok := ctx.Deadline(); ok && d.Before(deadline) {
		deadline = d
	}
	// One repeat a third of the way in covers a query lost on Wi-Fi.
	resend := time.Now().Add(window / 3)
	resent := false
	found := map[string]*Found{}
	order := []string{}
	buf := make([]byte, maxPacket)
	for {
		if ctx.Err() != nil {
			break
		}
		readUntil := deadline
		if !resent && resend.Before(readUntil) {
			readUntil = resend
		}
		_ = conn.SetReadDeadline(readUntil)
		n, src, err := conn.ReadFrom(buf)
		if err != nil {
			var ne net.Error
			if errors.As(err, &ne) && ne.Timeout() {
				if !resent && time.Now().Before(deadline) {
					resent = true
					_ = sendAll()
					continue
				}
				break
			}
			return nil, fmt.Errorf("discovery: browse read: %w", err)
		}
		udp, ok := src.(*net.UDPAddr)
		if !ok || udp.IP.To4() == nil {
			continue
		}
		for _, f := range parseAnswer(buf[:n], udp.IP.To4().String()) {
			if f.Instance == opts.Self {
				continue
			}
			if _, dup := found[f.Instance]; dup {
				continue
			}
			entry := f
			found[f.Instance] = &entry
			order = append(order, f.Instance)
		}
	}
	out := make([]Found, 0, len(order))
	for _, k := range order {
		out = append(out, *found[k])
	}
	return out, nil
}

// parseAnswer reads the instances a response packet names for our service.
func parseAnswer(packet []byte, source string) []Found {
	msg, err := unpack(packet)
	if err != nil || msg.Flags&flagResponse == 0 {
		return nil
	}
	records := append(append([]dnsRecord{}, msg.Answers...), msg.Extra...)
	var instances []string
	srv := map[string]dnsRecord{}
	txt := map[string]dnsRecord{}
	for _, rec := range records {
		name := canonical(rec.Name)
		switch {
		case rec.Type == typePTR && name == serviceName && rec.TTL > 0:
			instances = append(instances, canonical(rec.Target))
		case rec.Type == typeSRV:
			srv[name] = rec
		case rec.Type == typeTXT:
			txt[name] = rec
		}
	}
	var out []Found
	for _, inst := range instances {
		t, ok := txt[inst]
		if !ok {
			continue
		}
		port := 0
		if s, ok := srv[inst]; ok {
			port = int(s.Port)
		}
		a, dropped, err := ParseTXT(t.TXT, port)
		if err != nil {
			continue
		}
		// The TXT id must be the instance the PTR names: one advertisement
		// cannot speak for another.
		if canonical(a.Instance+"."+serviceName) != inst {
			continue
		}
		out = append(out, Found{Advertisement: a, Address: source, Dropped: dropped})
	}
	return out
}
