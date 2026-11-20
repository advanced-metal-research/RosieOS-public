package main

import (
	"bufio"
	"compress/gzip"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Series is one Stop-dump window in raw drive counts, indexed by cycle. Every
// column is int64 so the fixture writer and reader can treat them uniformly;
// unit conversion happens once, in the analysis, from the cell configuration.
type Series struct {
	Sources  []SourceInfo
	PeriodNs uint64
	Axes     int
	// ClockOffsetNs maps record t_ns (the servo host's monotonic clock) to
	// UTC: wall = t_ns + ClockOffsetNs. Set by the clock step, not the reader.
	ClockOffsetNs int64

	Seq, TNs, WorkNs, JitterNs, Armed, Enable, Mode, Traj []int64
	JogOpen, JogRamping, JogGen, JogSrcSeq, JogAxisMask   []int64
	Safety, Exec, Wkc, WkcExp                             []int64
	Pos, Target, TV, VA, FE, HomeOff, VAValid, FEValid    [][]int64
	// Torque is 0x6077 (actual torque, per mille of rated) and TorqueValid
	// the drive's validity flag. Fixtures written before 2026-10-08 carry
	// neither; they read back as zero torque, never valid.
	Torque, TorqueValid [][]int64
}

// SourceInfo identifies one input file in the report so an iteration can be
// traced back to the exact dump bytes it was computed from.
type SourceInfo struct {
	File          string `json:"file"`
	Kind          string `json:"kind"`
	SHA256        string `json:"sha256"`
	ProtocolMajor uint32 `json:"protocol_major"`
	ProtocolMinor uint32 `json:"protocol_minor"`
	LayoutDigest  string `json:"layout_digest"`
	Trigger       string `json:"trigger"`
	TriggerSeq    uint64 `json:"trigger_sequence"`
	TriggerTNs    int64  `json:"trigger_t_ns"`
	FirstSeq      uint64 `json:"first_sequence"`
	LastSeq       uint64 `json:"last_sequence"`
	Records       int    `json:"records"`
	Missing       uint64 `json:"missing_records"`
	// FileTimeUTC is the timestamp the servo host put in the dump's name
	// (written ~53-63 ms after the trigger record on the 2026-10-07 15:33
	// dumps); the coarse clock when no journal pins it.
	FileTimeUTC string `json:"file_time_utc,omitempty"`
	Note        string `json:"note,omitempty"`
}

func (s *Series) Len() int { return len(s.Seq) }

type column struct {
	name string
	data *[]int64
}

// columns lists every stored column in a fixed order; the fixture format and
// the merge both walk it, so adding a field here carries it everywhere.
func (s *Series) columns() []column {
	cols := []column{{"seq", &s.Seq}, {"t_ns", &s.TNs}, {"work_ns", &s.WorkNs}, {"jitter_ns", &s.JitterNs},
		{"armed", &s.Armed}, {"enable", &s.Enable}, {"mode", &s.Mode}, {"traj", &s.Traj},
		{"jog_open", &s.JogOpen}, {"jog_ramping", &s.JogRamping}, {"jog_gen", &s.JogGen}, {"jog_src_seq", &s.JogSrcSeq},
		{"jog_axis_mask", &s.JogAxisMask}, {"safety", &s.Safety}, {"exec", &s.Exec}, {"wkc", &s.Wkc}, {"wkc_exp", &s.WkcExp}}
	for i := 0; i < s.Axes; i++ {
		for _, a := range []struct {
			n string
			p *[][]int64
		}{{"pos", &s.Pos}, {"target", &s.Target}, {"tv", &s.TV}, {"va", &s.VA}, {"fe", &s.FE}, {"home_off", &s.HomeOff}, {"va_valid", &s.VAValid}, {"fe_valid", &s.FEValid}} {
			cols = append(cols, column{fmt.Sprintf("j%d_%s", i+1, a.n), &(*a.p)[i]})
		}
	}
	// Appended after every legacy column so a fixture without them is a
	// strict prefix of this layout (readFixture relies on that).
	for i := 0; i < s.Axes; i++ {
		cols = append(cols, column{fmt.Sprintf("j%d_torque", i+1), &s.Torque[i]}, column{fmt.Sprintf("j%d_torque_valid", i+1), &s.TorqueValid[i]})
	}
	return cols
}

// legacyColumns is the column count of fixtures written before torque was
// carried (17 record columns and 8 per axis).
func legacyColumns(axes int) int { return 17 + 8*axes }

func newSeries(axes int, period uint64) *Series {
	s := &Series{Axes: axes, PeriodNs: period}
	for _, p := range []*[][]int64{&s.Pos, &s.Target, &s.TV, &s.VA, &s.FE, &s.HomeOff, &s.VAValid, &s.FEValid, &s.Torque, &s.TorqueValid} {
		*p = make([][]int64, axes)
	}
	return s
}

func b2i(v uint32) int64 {
	if v != 0 {
		return 1
	}
	return 0
}

func (s *Series) appendRecord(r ipcclient.CycleCaptureRecordV2) {
	s.Seq = append(s.Seq, int64(r.Seq))
	s.TNs = append(s.TNs, int64(r.TNs))
	s.WorkNs = append(s.WorkNs, int64(r.WorkNs))
	s.JitterNs = append(s.JitterNs, r.CycleJitterNs)
	s.Armed = append(s.Armed, b2i(r.Armed))
	s.Enable = append(s.Enable, int64(r.AxisEnableMask))
	s.Mode = append(s.Mode, int64(r.ActiveMode))
	s.Traj = append(s.Traj, int64(r.ActiveTrajId))
	s.JogOpen = append(s.JogOpen, b2i(r.JogSessionOpen))
	s.JogRamping = append(s.JogRamping, b2i(r.JogRamping))
	s.JogGen = append(s.JogGen, int64(r.JogGeneration))
	s.JogSrcSeq = append(s.JogSrcSeq, int64(r.JogSourceSequence))
	s.JogAxisMask = append(s.JogAxisMask, int64(r.JogAxisMask))
	s.Safety = append(s.Safety, int64(r.SafetyFaultMask))
	s.Exec = append(s.Exec, int64(r.ExecutionFaultReasons))
	s.Wkc = append(s.Wkc, int64(r.WkcActual))
	s.WkcExp = append(s.WkcExp, int64(r.WkcExpected))
	for i := 0; i < s.Axes; i++ {
		a := r.Ax[i]
		s.Pos[i] = append(s.Pos[i], int64(a.Position))
		s.Target[i] = append(s.Target[i], int64(a.Target))
		s.TV[i] = append(s.TV[i], int64(a.TargetVelocityCountsPerS))
		s.VA[i] = append(s.VA[i], int64(a.VelocityActualCountsPerS))
		s.FE[i] = append(s.FE[i], int64(a.FollowingErrorCounts))
		s.HomeOff[i] = append(s.HomeOff[i], int64(a.NativeHomePositionOffset))
		s.VAValid[i] = append(s.VAValid[i], b2i(a.VelocityActualValid))
		s.FEValid[i] = append(s.FEValid[i], b2i(a.FollowingErrorValid))
		s.Torque[i] = append(s.Torque[i], int64(a.TorqueRaw))
		s.TorqueValid[i] = append(s.TorqueValid[i], b2i(uint32(a.TorqueValid)))
	}
}

// slice returns rows [a, b) sharing no storage with s.
func (s *Series) slice(a, b int) *Series {
	out := newSeries(s.Axes, s.PeriodNs)
	out.Sources = append([]SourceInfo(nil), s.Sources...)
	out.ClockOffsetNs = s.ClockOffsetNs
	src, dst := s.columns(), out.columns()
	for c := range src {
		*dst[c].data = append([]int64(nil), (*src[c].data)[a:b]...)
	}
	return out
}

var dumpNameTime = regexp.MustCompile(`^(\d{8}T\d{6}\.\d+Z)`)

func parseDumpFileTime(name string) (time.Time, bool) {
	m := dumpNameTime.FindStringSubmatch(filepath.Base(name))
	if m == nil {
		return time.Time{}, false
	}
	t, err := time.Parse("20060102T150405.999999999Z", m[1])
	return t, err == nil
}

func fileSHA256(path string) (string, error) {
	f, err := os.Open(path)
	if err != nil {
		return "", err
	}
	defer f.Close()
	h := sha256.New()
	if _, err := io.Copy(h, f); err != nil {
		return "", err
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}

// readDump decodes a native Stop dump on the analysis host (never the servo
// host) with ipcclient.ReadTelemetryDump, the decoder rt-core's own tools use,
// under its one compatibility rule (TelemetryDumpIndex.CheckCompatible): a
// dump of another protocol minor is read when its record layout digest is this
// build's (1.13 and 1.14 share it, so a Rosie 1430 arm's whole dump history reads), and a
// foreign digest is refused because its fields would be misread.
func readDump(path string) (*Series, error) {
	var s *Series
	var triggerTNs int64
	index, err := ipcclient.ReadTelemetryDump(path, func(rec ipcclient.CycleCaptureRecordV2, index ipcclient.TelemetryDumpIndex) error {
		if s == nil {
			s = newSeries(int(index.AxisCount), index.CyclePeriodNS)
		}
		if rec.Seq == index.TriggerSequence {
			triggerTNs = int64(rec.TNs)
		}
		s.appendRecord(rec)
		return nil
	})
	if err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	if s == nil {
		s = newSeries(int(index.AxisCount), index.CyclePeriodNS)
	}
	sum, err := fileSHA256(path)
	if err != nil {
		return nil, err
	}
	info := SourceInfo{File: filepath.Base(path), Kind: "dump", SHA256: sum, ProtocolMajor: index.ProtocolMajor, ProtocolMinor: index.ProtocolMinor,
		LayoutDigest: index.LayoutDigest, Trigger: index.Trigger, TriggerSeq: index.TriggerSequence, TriggerTNs: triggerTNs,
		FirstSeq: index.FirstSequence, LastSeq: index.LastSequence, Records: int(index.RecordCount), Missing: index.MissingRecords,
		Note: index.CompatibilityNote()}
	if t, ok := parseDumpFileTime(path); ok {
		info.FileTimeUTC = t.Format(time.RFC3339Nano)
	}
	s.Sources = []SourceInfo{info}
	return s, nil
}

// Fixture format: gzip("TRFX1\n" + header JSON + "\n" + columns), each column
// zigzag-varint coded in columns() order as first or second differences,
// whichever is shorter (positions and targets move smoothly, so their second
// difference is small; work and jitter are noise, so their first is). Ten
// seconds of a 220 MB dump fit in about 100-200 kB, small enough to commit.
const fixtureMagic = "TRFX1\n"

type fixtureHeader struct {
	Axes     int          `json:"axes"`
	PeriodNs uint64       `json:"period_ns"`
	Rows     int          `json:"rows"`
	Columns  []string     `json:"columns"`
	Orders   []int        `json:"orders"`
	Sources  []SourceInfo `json:"sources"`
	Note     string       `json:"note,omitempty"`
}

func diffs(x []int64, order int) []int64 {
	d := append([]int64(nil), x...)
	for o := 0; o < order; o++ {
		for i := len(d) - 1; i > 0; i-- {
			d[i] -= d[i-1]
		}
	}
	return d
}

func undiff(d []int64, order int) {
	for o := 0; o < order; o++ {
		for i := 1; i < len(d); i++ {
			d[i] += d[i-1]
		}
	}
}

func varintLen(d []int64) int {
	var buf [binary.MaxVarintLen64]byte
	n := 0
	for _, v := range d {
		n += binary.PutVarint(buf[:], v)
	}
	return n
}

func writeFixture(path string, s *Series, note string) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	gz, _ := gzip.NewWriterLevel(f, gzip.BestCompression)
	w := bufio.NewWriter(gz)
	cols := s.columns()
	h := fixtureHeader{Axes: s.Axes, PeriodNs: s.PeriodNs, Rows: s.Len(), Sources: s.Sources, Note: note}
	coded := make([][]int64, len(cols))
	for i, c := range cols {
		h.Columns = append(h.Columns, c.name)
		d1, d2 := diffs(*c.data, 1), diffs(*c.data, 2)
		if varintLen(d2) < varintLen(d1) {
			coded[i] = d2
			h.Orders = append(h.Orders, 2)
		} else {
			coded[i] = d1
			h.Orders = append(h.Orders, 1)
		}
	}
	hb, _ := json.Marshal(h)
	w.WriteString(fixtureMagic)
	w.Write(hb)
	w.WriteByte('\n')
	var buf [binary.MaxVarintLen64]byte
	for _, d := range coded {
		for _, v := range d {
			n := binary.PutVarint(buf[:], v)
			w.Write(buf[:n])
		}
	}
	if err = w.Flush(); err != nil {
		return err
	}
	return gz.Close()
}

func readFixture(path string) (*Series, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	gz, err := gzip.NewReader(f)
	if err != nil {
		return nil, err
	}
	r := bufio.NewReader(gz)
	magic := make([]byte, len(fixtureMagic))
	if _, err = io.ReadFull(r, magic); err != nil || string(magic) != fixtureMagic {
		return nil, fmt.Errorf("%s: not a tuning-report fixture", path)
	}
	line, err := r.ReadBytes('\n')
	if err != nil {
		return nil, err
	}
	var h fixtureHeader
	if err = json.Unmarshal(line, &h); err != nil {
		return nil, err
	}
	s := newSeries(h.Axes, h.PeriodNs)
	cols := s.columns()
	stored := len(cols)
	if len(h.Columns) == legacyColumns(h.Axes) {
		stored = len(h.Columns)
	}
	if stored != len(h.Columns) || len(h.Orders) != stored {
		return nil, fmt.Errorf("%s: %d columns, this build expects %d (or %d without torque)", path, len(h.Columns), len(cols), legacyColumns(h.Axes))
	}
	for c := stored; c < len(cols); c++ {
		*cols[c].data = make([]int64, h.Rows)
	}
	for c := 0; c < stored; c++ {
		if cols[c].name != h.Columns[c] {
			return nil, fmt.Errorf("%s: column %d is %s, expected %s", path, c, h.Columns[c], cols[c].name)
		}
		data := make([]int64, h.Rows)
		for k := range data {
			if data[k], err = binary.ReadVarint(r); err != nil {
				return nil, fmt.Errorf("%s: column %s: %w", path, cols[c].name, err)
			}
		}
		undiff(data, h.Orders[c])
		*cols[c].data = data
	}
	s.Sources = h.Sources
	for i := range s.Sources {
		s.Sources[i].Kind = "fixture"
		if h.Note != "" {
			s.Sources[i].Note = strings.TrimSpace(s.Sources[i].Note + " " + h.Note)
		}
	}
	return s, nil
}

func loadInput(path string) (*Series, error) {
	if strings.HasSuffix(path, ".trfx.gz") {
		return readFixture(path)
	}
	return readDump(path)
}

// mergeSources orders inputs and joins dumps of the same run (overlapping
// sequences with identical t_ns) into one series, then splits every series at
// sequence gaps. A different run (rt-core restarted: sequence or clock reset)
// stays a separate window, so no metric is computed across a discontinuity.
func mergeSources(in []*Series) ([]*Series, error) {
	if len(in) == 0 {
		return nil, errors.New("no Stop dumps found")
	}
	sort.SliceStable(in, func(a, b int) bool {
		ta, tb := in[a].Sources[0].FileTimeUTC, in[b].Sources[0].FileTimeUTC
		if ta != tb {
			return ta < tb
		}
		return in[a].Seq[0] < in[b].Seq[0]
	})
	var groups []*Series
	for _, s := range in {
		if s.Len() == 0 {
			continue
		}
		var g *Series
		if n := len(groups); n > 0 {
			g = groups[n-1]
		}
		if g != nil && g.Axes == s.Axes && g.PeriodNs == s.PeriodNs && sameRun(g, s) {
			last := g.Seq[g.Len()-1]
			start := sort.Search(s.Len(), func(k int) bool { return s.Seq[k] > last })
			src, dst := s.columns(), g.columns()
			for c := range src {
				*dst[c].data = append(*dst[c].data, (*src[c].data)[start:]...)
			}
			g.Sources = append(g.Sources, s.Sources...)
			continue
		}
		groups = append(groups, s)
	}
	var out []*Series
	for _, g := range groups {
		a := 0
		for k := 1; k <= g.Len(); k++ {
			if k == g.Len() || g.Seq[k] != g.Seq[k-1]+1 {
				if k-a >= 2 {
					out = append(out, g.slice(a, k))
				}
				a = k
			}
		}
	}
	return out, nil
}

// sameRun reports whether s continues g's run: a shared sequence number must
// carry the same t_ns, or s must start right after g with a plausible clock.
func sameRun(g, s *Series) bool {
	first := s.Seq[0]
	k := sort.Search(g.Len(), func(k int) bool { return g.Seq[k] >= first })
	if k < g.Len() && g.Seq[k] == first {
		return g.TNs[k] == s.TNs[0]
	}
	last := g.Len() - 1
	if first > g.Seq[last] {
		dt := s.TNs[0] - g.TNs[last]
		expect := (first - g.Seq[last]) * int64(g.PeriodNs)
		return dt > 0 && absI(dt-expect) < expect/10+int64(g.PeriodNs)
	}
	return false
}

func absI(v int64) int64 {
	if v < 0 {
		return -v
	}
	return v
}
