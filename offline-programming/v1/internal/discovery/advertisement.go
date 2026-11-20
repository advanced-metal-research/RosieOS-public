// Package discovery lets OLP servers find each other on an operator LAN.
//
// An OLP server advertises itself as a DNS-SD service (_rosie-olp._tcp) on
// the interfaces it is told to use, and a browsing OLP server lists what it
// hears. An advertisement is information, never authority: it names the
// cells a server presents and the pins it holds, and nothing in it is used
// as an endpoint, a credential or a pin. Connecting to a cell still needs
// the browsing server's own commissioned catalogue entry, the repository
// manifest pin and the cell's mutual-TLS Describe. Every field read from the
// network is validated here and anything malformed is dropped.
package discovery

import (
	"fmt"
	"regexp"
	"strconv"
	"strings"
	"unicode"
	"unicode/utf8"
)

const (
	// ServiceType is the DNS-SD service an OLP server advertises.
	ServiceType = "_rosie-olp._tcp"
	// Schema names the advertisement record, in TXT ("v=1") and in JSON.
	Schema = "rosie.olp-advertisement.v1"
	// AdvertisementPath serves this server's own record as JSON: the same
	// facts as its TXT record, for a browser that probes over HTTP.
	AdvertisementPath = "/api/offline-programming/v1/advertisement"
	// DiscoveredCellsPath lists what this server found.
	DiscoveredCellsPath = "/api/offline-programming/v1/discovered-cells"
	// HealthPath is the OLP health route a browser may probe.
	HealthPath = "/api/offline-programming/v1/health"

	// maxCells bounds the cells in one advertisement. A TXT record is sent in
	// one packet; eight full entries stay under ~1.6 KB.
	maxCells = 8
	// maxTXTBytes keeps the whole TXT rdata inside one Ethernet frame with
	// the PTR/SRV/A records beside it (RFC 6763 section 6.2 advises < 1300).
	maxTXTBytes = 1300
)

// Advertisement is what one OLP server says about itself.
type Advertisement struct {
	Schema string `json:"schema"`
	// Instance is random per process, so a browser can drop its own
	// advertisement and tell two servers on one host apart.
	Instance string `json:"instance"`
	Hostname string `json:"hostname"`
	Port     int    `json:"port"`
	// Reach is "loopback" when the server's HTTP listener is bound to
	// loopback (reached through a tunnel only), else "network".
	Reach       string           `json:"reach"`
	BuildID     string           `json:"build_id"`
	StartedAt   string           `json:"started_at"`
	Maintenance bool             `json:"maintenance"`
	Cells       []AdvertisedCell `json:"cells"`
	// Truncated says cells were left out to fit one packet.
	Truncated bool `json:"truncated,omitempty"`
}

// AdvertisedCell is one catalogue cell as its server presents it.
type AdvertisedCell struct {
	CellID string `json:"cell_id"`
	Label  string `json:"label"`
	Model  string `json:"model"`
	// Mode is the catalogue's requested_mode: "simulation" or "real".
	Mode string `json:"mode"`
	// ConfigurationSHA256 is the server's catalogue pin for the cell.
	ConfigurationSHA256 string `json:"configuration_sha256,omitempty"`
	// CapabilitiesDigest is what the cell's rt-control reported in its last
	// authenticated Describe to that server; empty until it was described.
	CapabilitiesDigest string `json:"capabilities_digest,omitempty"`
	// Selected: that server holds this cell selected now.
	Selected bool `json:"selected,omitempty"`
}

var (
	idPattern       = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]{0,62}$`)
	modelPattern    = regexp.MustCompile(`^[a-z0-9][a-z0-9_]{0,39}$`)
	digestPattern   = regexp.MustCompile(`^[0-9a-f]{64}$`)
	instancePattern = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9-]{0,62}$`)
	hostPattern     = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9.-]{0,62}$`)
	startedPattern  = regexp.MustCompile(`^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9:.+Z-]{8,30}$`)
)

// cleanText keeps printable text, without the TXT field separator, bounded.
func cleanText(s string, limit int) string {
	var b strings.Builder
	for _, r := range s {
		if r == '|' || r == utf8.RuneError || !unicode.IsPrint(r) {
			continue
		}
		if b.Len()+utf8.RuneLen(r) > limit {
			break
		}
		b.WriteRune(r)
	}
	return strings.TrimSpace(b.String())
}

// Validate normalises a record read from the network (TXT or JSON). It
// refuses a record without a usable instance and drops each malformed cell,
// returning how many it dropped.
func (a *Advertisement) Validate() (dropped int, err error) {
	if a.Schema != Schema {
		return 0, fmt.Errorf("discovery: advertisement schema %q", a.Schema)
	}
	if !instancePattern.MatchString(a.Instance) {
		return 0, fmt.Errorf("discovery: advertisement instance invalid")
	}
	if a.Hostname != "" && !hostPattern.MatchString(a.Hostname) {
		a.Hostname = ""
	}
	if a.Port < 1 || a.Port > 65535 {
		a.Port = 0
	}
	if a.Reach != "loopback" && a.Reach != "network" {
		a.Reach = "unknown"
	}
	a.BuildID = cleanText(a.BuildID, 64)
	if !startedPattern.MatchString(a.StartedAt) {
		a.StartedAt = ""
	}
	cells := a.Cells[:0:0]
	seen := map[string]bool{}
	for _, c := range a.Cells {
		c.Label = cleanText(c.Label, 64)
		ok := idPattern.MatchString(c.CellID) && !seen[c.CellID] && modelPattern.MatchString(c.Model) &&
			(c.Mode == "simulation" || c.Mode == "real") &&
			(c.ConfigurationSHA256 == "" || digestPattern.MatchString(c.ConfigurationSHA256)) &&
			(c.CapabilitiesDigest == "" || digestPattern.MatchString(c.CapabilitiesDigest))
		if !ok || len(cells) == maxCells {
			dropped++
			continue
		}
		seen[c.CellID] = true
		cells = append(cells, c)
	}
	a.Cells = cells
	return dropped, nil
}

// TXT encodes the record as DNS-SD key=value strings. A cell is one string,
// "c<i>=id|model|mode|pin|caps|selected|label"; cells that do not fit
// maxTXTBytes are left out and "more=1" says so.
func (a Advertisement) TXT() []string {
	maint := "0"
	if a.Maintenance {
		maint = "1"
	}
	txt := []string{
		"v=1",
		"id=" + a.Instance,
		"host=" + a.Hostname,
		"reach=" + a.Reach,
		"build=" + cleanText(a.BuildID, 64),
		"started=" + a.StartedAt,
		"maint=" + maint,
	}
	size := 0
	for _, s := range txt {
		size += len(s) + 1
	}
	truncated := a.Truncated
	written := 0
	for _, c := range a.Cells {
		selected := "0"
		if c.Selected {
			selected = "1"
		}
		entry := fmt.Sprintf("c%d=%s|%s|%s|%s|%s|%s|", written, c.CellID, c.Model, c.Mode, c.ConfigurationSHA256, c.CapabilitiesDigest, selected)
		label := cleanText(c.Label, 255-len(entry))
		entry += label
		if written == maxCells || size+len(entry)+1+len("more=1")+1 > maxTXTBytes {
			truncated = true
			break
		}
		txt = append(txt, entry)
		size += len(entry) + 1
		written++
	}
	if truncated {
		txt = append(txt, "more=1")
	}
	return txt
}

// ParseTXT is the inverse of TXT, followed by Validate. The port comes from
// the SRV record, not from TXT.
func ParseTXT(txt []string, port int) (Advertisement, int, error) {
	fields := map[string]string{}
	for _, s := range txt {
		key, value, ok := strings.Cut(s, "=")
		if !ok {
			continue
		}
		key = strings.ToLower(key)
		if _, dup := fields[key]; !dup {
			fields[key] = value
		}
	}
	if fields["v"] != "1" {
		return Advertisement{}, 0, fmt.Errorf("discovery: TXT version %q", fields["v"])
	}
	a := Advertisement{
		Schema:      Schema,
		Instance:    fields["id"],
		Hostname:    fields["host"],
		Port:        port,
		Reach:       fields["reach"],
		BuildID:     fields["build"],
		StartedAt:   fields["started"],
		Maintenance: fields["maint"] == "1",
		Truncated:   fields["more"] == "1",
		Cells:       []AdvertisedCell{},
	}
	for i := 0; i < maxCells; i++ {
		raw, ok := fields["c"+strconv.Itoa(i)]
		if !ok {
			break
		}
		parts := strings.SplitN(raw, "|", 7)
		if len(parts) != 7 {
			a.Cells = append(a.Cells, AdvertisedCell{}) // dropped by Validate
			continue
		}
		a.Cells = append(a.Cells, AdvertisedCell{
			CellID: parts[0], Model: parts[1], Mode: parts[2],
			ConfigurationSHA256: parts[3], CapabilitiesDigest: parts[4],
			Selected: parts[5] == "1", Label: parts[6],
		})
	}
	dropped, err := a.Validate()
	return a, dropped, err
}
