package main

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"time"
)

// JournalLine is one journal line with its printed wall time. Short journal
// formats carry no zone or year: Local is the naive reading (zone unknown),
// fixed later against the dump clock.
type JournalLine struct {
	Local  time.Time // naive reading in UTC; true UTC = Local - zoneShift
	HasTZ  bool
	Text   string
	Source string // "revpi", "olp", "other"
	File   string
}

var (
	shortPrefix = regexp.MustCompile(`^([A-Z][a-z]{2}) +(\d{1,2}) (\d{2}):(\d{2}):(\d{2}(?:\.\d+)?) `)
	isoPrefix   = regexp.MustCompile(`^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:?\d{2}))`)
	kvPattern   = regexp.MustCompile(`([A-Za-z_][A-Za-z0-9_]*)=("[^"]*"|\{[^}]*\}|\[[^\]]*\]|\S*)`)
)

func parseJournalFile(path string, year int) ([]JournalLine, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	var out []JournalLine
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 1<<20), 1<<24)
	for sc.Scan() {
		line := sc.Text()
		var jl JournalLine
		if m := isoPrefix.FindStringSubmatch(line); m != nil {
			s := m[1]
			if len(s) > 5 && (s[len(s)-5] == '+' || s[len(s)-5] == '-') {
				s = s[:len(s)-2] + ":" + s[len(s)-2:]
			}
			t, err := time.Parse(time.RFC3339Nano, s)
			if err != nil {
				continue
			}
			jl = JournalLine{Local: t.UTC(), HasTZ: true}
		} else if m := shortPrefix.FindStringSubmatch(line); m != nil {
			mon, err := time.Parse("Jan", m[1])
			if err != nil {
				continue
			}
			day, _ := strconv.Atoi(m[2])
			h, _ := strconv.Atoi(m[3])
			mi, _ := strconv.Atoi(m[4])
			sec, _ := strconv.ParseFloat(m[5], 64)
			jl.Local = time.Date(year, mon.Month(), day, h, mi, 0, 0, time.UTC).Add(time.Duration(sec * 1e9))
		} else {
			continue
		}
		jl.Text, jl.File = line, path
		switch {
		case strings.Contains(line, "rosie-rt-core") || strings.Contains(line, "rosie-rt-control") || strings.Contains(line, "observed_ns="):
			jl.Source = "revpi"
		case strings.Contains(line, "offline-programming") || strings.Contains(line, "path hold:") || strings.Contains(line, "jog hold:"):
			jl.Source = "olp"
		default:
			jl.Source = "other"
		}
		out = append(out, jl)
	}
	return out, sc.Err()
}

func kv(text string) map[string]string {
	out := map[string]string{}
	for _, m := range kvPattern.FindAllStringSubmatch(text, -1) {
		out[m[1]] = strings.Trim(m[2], `"`)
	}
	return out
}

func kvFloat(m map[string]string, k string) float64 {
	v, err := strconv.ParseFloat(strings.TrimSuffix(m[k], ","), 64)
	if err != nil {
		return math.NaN()
	}
	return v
}

// Clock maps a window's record t_ns to UTC.
type Clock struct {
	Method      string  `json:"method"`
	OffsetNs    int64   `json:"offset_ns"`
	Uncertainty string  `json:"uncertainty"`
	RevPiShiftH float64 `json:"revpi_journal_utc_shift_h,omitempty"`
	OLPShiftH   float64 `json:"olp_journal_utc_shift_h,omitempty"`
	Matches     int     `json:"matches,omitempty"`
}

// coarseOffset is the dump file-name clock: the servo host names a dump when
// it writes it, 53 and 63 ms after the trigger record on the two 2026-10-07
// 15:33 dumps (checked against the RevPi journal there). 58 ms is their mean.
const fileNameAfterTriggerNs = 58e6

func coarseOffset(w *Series) (int64, bool) {
	for _, src := range w.Sources {
		if src.FileTimeUTC == "" || src.TriggerTNs == 0 {
			continue
		}
		t, err := time.Parse(time.RFC3339Nano, src.FileTimeUTC)
		if err != nil {
			continue
		}
		return t.UnixNano() - int64(fileNameAfterTriggerNs) - src.TriggerTNs, true
	}
	return 0, false
}

// zoneShift picks the whole-quarter-hour shift that makes a journal agree
// with the reference offset; a journal in the wrong zone otherwise lands
// hours away and silently matches nothing.
func zoneShift(deltas []float64, tolS float64) (float64, int) {
	best, bestN := 0.0, 0
	for q := -14 * 4; q <= 14*4; q++ {
		h := float64(q) / 4
		n := 0
		for _, d := range deltas {
			if math.Abs(d-h*3600) < tolS {
				n++
			}
		}
		if n > bestN || (n == bestN && math.Abs(h) < math.Abs(best)) {
			best, bestN = h, n
		}
	}
	return best, bestN
}

// settleClock fixes each window's ClockOffsetNs. Preference: RevPi journal
// lines carrying observed_ns (same monotonic clock as t_ns; the journal stamps
// a line after the event, so the smallest wall - observed is the tightest,
// within the journal's write latency of ~1 ms), then OLP path-hold begin lines
// matched to lane openings (median of the matched deltas), then the file name.
func settleClock(windows []*Series, lines []JournalLine, override *int64) []Clock {
	var clocks []Clock
	for _, w := range windows {
		c := Clock{Method: "none", Uncertainty: "unknown: no file-name time, journal or --clock-offset-ns"}
		coarse, ok := coarseOffset(w)
		if ok {
			c = Clock{Method: "dump file name", OffsetNs: coarse, Uncertainty: "+-30 ms (file written 53-63 ms after the trigger)"}
		}
		if override != nil {
			c = Clock{Method: "--clock-offset-ns", OffsetNs: *override, Uncertainty: "as given"}
			w.ClockOffsetNs = c.OffsetNs
			clocks = append(clocks, c)
			continue
		}
		t0, t1 := w.TNs[0], w.TNs[w.Len()-1]
		type obs struct {
			local time.Time
			hasTZ bool
			mono  int64
		}
		var rp []obs
		for _, l := range lines {
			if l.Source != "revpi" {
				continue
			}
			v, err := strconv.ParseInt(kv(l.Text)["observed_ns"], 10, 64)
			if err != nil || v < t0-600e9 || v > t1+600e9 {
				continue
			}
			rp = append(rp, obs{l.Local, l.HasTZ, v})
		}
		if len(rp) > 0 {
			// The zone: a line that carries it (journalctl -o short-iso)
			// needs none; otherwise the shift that agrees with the file-name
			// clock; with neither, the servo host's journal is read as UTC.
			shift, n, note := 0.0, len(rp), ""
			if !rp[0].hasTZ {
				if ok {
					deltas := make([]float64, len(rp))
					for i, o := range rp {
						deltas[i] = float64(o.local.UnixNano()-o.mono-coarse) / 1e9
					}
					shift, n = zoneShift(deltas, 2)
				} else {
					note = "; journal zone assumed UTC (no zone in the lines, no dump file-name time to check it)"
				}
			}
			if n > 0 {
				best := int64(math.MaxInt64)
				for _, o := range rp {
					d := o.local.UnixNano() - int64(shift*3600e9) - o.mono
					if (!ok || math.Abs(float64(d-coarse)) < 2e9) && d < best {
						best = d
					}
				}
				// Spread of the agreeing lines' offsets bounds the error.
				worst := best
				for _, o := range rp {
					d := o.local.UnixNano() - int64(shift*3600e9) - o.mono
					if (!ok || math.Abs(float64(d-coarse)) < 2e9) && d > worst {
						worst = d
					}
				}
				c = Clock{Method: "RevPi journal observed_ns", OffsetNs: best,
					Uncertainty: fmt.Sprintf("times may read late by the journal write latency (%d lines disagree by up to %.0f ms)%s", n, float64(worst-best)/1e6, note), RevPiShiftH: shift, Matches: n}
			}
		}
		if c.Method != "RevPi journal observed_ns" && ok {
			opens := laneOpenings(w)
			var begins []time.Time
			for _, l := range lines {
				if l.Source == "olp" && strings.Contains(l.Text, "path hold: begin") {
					begins = append(begins, l.Local)
				}
			}
			if len(opens) > 0 && len(begins) > 0 {
				var deltas []float64
				for _, b := range begins {
					for _, o := range opens {
						deltas = append(deltas, float64(b.UnixNano()-(o+coarse))/1e9)
					}
				}
				shift, _ := zoneShift(deltas, 1)
				var resid []float64
				for _, b := range begins {
					bt := b.UnixNano() - int64(shift*3600e9) - coarse
					bestD := math.Inf(1)
					for _, o := range opens {
						if d := float64(bt - o); math.Abs(d) < math.Abs(bestD) {
							bestD = d
						}
					}
					if math.Abs(bestD) < 1e9 {
						resid = append(resid, bestD)
					}
				}
				if len(resid) >= 2 {
					sort.Float64s(resid)
					med := resid[len(resid)/2]
					// The begin line is logged after native opened the lane;
					// the 2026-10-07 15:33 RevPi-pinned session measured
					// 20-150 ms. Shift the median by its 60 ms middle.
					c = Clock{Method: "OLP path-hold begin vs lane opening", OffsetNs: coarse - int64(med) + 60e6,
						Uncertainty: "+-60 ms (begin-line latency)", OLPShiftH: shift, Matches: len(resid)}
				}
			}
		}
		w.ClockOffsetNs = c.OffsetNs
		clocks = append(clocks, c)
	}
	return clocks
}

// olpShift returns the zone shift (hours) that maps the OLP journal's printed
// times onto the window's UTC clock, 0 when the lines carry their zone.
func olpShift(w *Series, lines []JournalLine) float64 {
	opens := laneOpenings(w)
	var deltas []float64
	for _, l := range lines {
		if l.Source != "olp" || !strings.Contains(l.Text, "path hold: begin") {
			continue
		}
		if l.HasTZ {
			return 0
		}
		for _, o := range opens {
			deltas = append(deltas, float64(l.Local.UnixNano()-(o+w.ClockOffsetNs))/1e9)
		}
	}
	if len(deltas) == 0 {
		// No hold to match: assume the analysis host's zone, as journalctl
		// prints in the zone of the host that wrote it (a cell's relay: cell local).
		_, off := time.Now().Zone()
		return float64(off) / 3600
	}
	s, _ := zoneShift(deltas, 1)
	return s
}

// laneOpenings are the t_ns of every jog lane opening (jog_open 0 -> 1).
func laneOpenings(w *Series) []int64 {
	var out []int64
	for k := 1; k < w.Len(); k++ {
		if w.JogOpen[k] == 1 && w.JogOpen[k-1] == 0 {
			out = append(out, w.TNs[k])
		}
	}
	return out
}

// RevPi journal facts that belong in a tuning report.
type TargetLeadLine struct {
	UTC      string  `json:"utc"`
	Axis     int     `json:"axis"`
	Cleared  bool    `json:"cleared"`
	LeadDeg  float64 `json:"lead_deg"`
	LimitDeg float64 `json:"limit_deg,omitempty"`
	VelDegS  float64 `json:"velocity_deg_s,omitempty"`
	DurMs    float64 `json:"duration_ms,omitempty"`
}

type ObserveLine struct {
	Axis  int    `json:"axis"`
	Key   string `json:"key"`
	Value string `json:"value"`
}

func revpiFacts(lines []JournalLine, shiftH float64) (leads []TargetLeadLine, observed []ObserveLine) {
	seen := map[string]bool{}
	for _, l := range lines {
		if l.Source != "revpi" {
			continue
		}
		utc := l.Local.Add(-time.Duration(shiftH * 3600e9)).Format("2006-01-02T15:04:05.000Z")
		m := kv(l.Text)
		switch {
		case strings.Contains(l.Text, "target_lead_cleared "):
			ax, _ := strconv.Atoi(m["axis"])
			leads = append(leads, TargetLeadLine{UTC: utc, Axis: ax, Cleared: true, LeadDeg: kvFloat(m, "peak_deg"), DurMs: kvFloat(m, "duration_ns") / 1e6})
		case strings.Contains(l.Text, "target_lead severity="):
			ax, _ := strconv.Atoi(m["axis"])
			leads = append(leads, TargetLeadLine{UTC: utc, Axis: ax, LeadDeg: kvFloat(m, "lead_deg"), LimitDeg: kvFloat(m, "limit_deg"), VelDegS: kvFloat(m, "velocity_deg_s")})
		case strings.Contains(l.Text, "startup observe"):
			ax, _ := strconv.Atoi(m["axis"])
			key := fmt.Sprintf("%d/%s", ax, m["key"])
			if !seen[key] {
				seen[key] = true
				observed = append(observed, ObserveLine{Axis: ax, Key: m["key"], Value: m["value"]})
			}
		}
	}
	return
}
