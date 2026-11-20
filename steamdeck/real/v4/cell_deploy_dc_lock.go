package main

import (
	"fmt"
	"sort"
	"strconv"
	"strings"
)

// The distributed-clock lock check of `realtime-core verify` (and so of start,
// restart, resume-after-reboot, rollback and release, which all end in it).
//
// Why it exists: until 9bac73cd rt-core wrote the DC reference clock once per
// 1000 cycles. The reference ESC steers its local time only on a received
// 0x0910 write, so it hunted and never locked. On the mini bench Pi 5 (one
// A6-EC, 1 ms CSP), 2026-11-03, ESC register 0x092C (system time difference)
// read 25.4 ms (0x8183d2e1), growing ~23 us/s, IgH logged "Slave did not sync
// after 5000 ms" at every start since September, and the drive's TxPDO update
// drifted through the host cycle: stale-then-doubled feedback bursts every
// 6-43 s, one of which latched coordinate_reference. Every deploy step passed.
// After the fix the same register reads 1.6-2.4 us. A clock that is not locked
// is now a verify failure, not something an operator finds in a Stop dump.
//
// What runs on the servo host is a handful of `ethercat reg_read` calls (an
// ESC register read through the master; no mailbox, no SDO) and one kernel
// journal count. The raw register text comes back to this machine and is
// interpreted here; nothing is computed on the servo host.

// cellDeployDCLock is the cell's bound, cell_deploy.dc_lock.
type cellDeployDCLock struct {
	// MaxSystemTimeDifferenceNs bounds |0x092C| on every DC slave, ns.
	MaxSystemTimeDifferenceNs int64 `json:"max_system_time_difference_ns"`
	// SettleS is how long rosie-rt-core must have been active before the
	// samples count: the clock may still be converging just after a start.
	SettleS int `json:"settle_s"`
}

func (l cellDeployDCLock) validate() error {
	// 0x092C carries a 31-bit magnitude; the bound must be one it can show.
	if l.MaxSystemTimeDifferenceNs < 1 || l.MaxSystemTimeDifferenceNs > 0x7fffffff {
		return fmt.Errorf("cell_deploy dc_lock.max_system_time_difference_ns must be 1..2147483647 ns (ESC 0x092C has a 31-bit magnitude), got %d", l.MaxSystemTimeDifferenceNs)
	}
	if l.SettleS < 0 || l.SettleS > 120 {
		return fmt.Errorf("cell_deploy dc_lock.settle_s must be 0..120 s, got %d", l.SettleS)
	}
	return nil
}

// cellDeployDCRounds samples each DC slave this many times, about a second
// apart (the servo script sleeps 1 s after each round). Three reads over ~2 s
// show a drifting clock as a trend: the unlocked mini bench clock moved
// ~23 us/s (2026-11-03), far past a 10 us bound within one round anyway.
const cellDeployDCRounds = 3

// cellDeployDCFacts is what the servo host's verify script printed.
type cellDeployDCFacts struct {
	// Skipped is why nothing was read (rosie-rt-core not active).
	Skipped string `json:"skipped"`
	// SettleWaitS is how long the script waited for settle_s; CoreUpS is how
	// long rosie-rt-core had been active when the first round began.
	SettleWaitS int                  `json:"settle_wait_s"`
	CoreUpS     int                  `json:"core_up_s"`
	Samples     []cellDeployDCSample `json:"samples"`
	// NotSynced counts IgH's "Slave did not sync" kernel lines since
	// rosie-rt-core started (-1: not read); KernelLines are the first few.
	NotSynced   int      `json:"not_synced"`
	KernelLines []string `json:"kernel_lines"`
}

type cellDeployDCSample struct {
	Round    int    `json:"round"`
	Position int    `json:"position"`
	Raw      string `json:"raw"`
}

// cellDeployDCSystemTimeDifference reads `ethercat reg_read -tuint32 0x092C`
// output ("0x8000095c 2147485020"; a bare decimal or hex word is accepted
// too) and returns the register word and its magnitude in ns. ESC 0x092C:
// bit 31 is the sign (set: the local copy of system time is smaller than the
// received one), bits 0-30 the magnitude in ns; 0x8000095c is 2396 ns, not
// 2.1 s. Anything else is an error: an unread register must never pass.
func cellDeployDCSystemTimeDifference(raw string) (word uint32, magnitudeNs uint32, err error) {
	fields := strings.Fields(raw)
	if len(fields) == 0 {
		return 0, 0, fmt.Errorf("no output")
	}
	text, base := fields[0], 10
	if rest, ok := strings.CutPrefix(strings.ToLower(text), "0x"); ok {
		text, base = rest, 16
	}
	v, perr := strconv.ParseUint(text, base, 32)
	if perr != nil || text == "" {
		return 0, 0, fmt.Errorf("not a 32-bit register value: %q", strings.Join(fields, " "))
	}
	if len(fields) >= 2 && base == 16 {
		// IgH prints the word twice, hex then decimal; they must agree.
		if d, derr := strconv.ParseUint(fields[1], 10, 32); derr != nil || d != v {
			return 0, 0, fmt.Errorf("hex and decimal disagree: %q", strings.Join(fields, " "))
		}
	}
	return uint32(v), uint32(v) & 0x7fffffff, nil
}

// cellDeployDCTime renders ns in the unit an operator reads it in.
func cellDeployDCTime(ns int64) string {
	switch {
	case ns < 1000:
		return fmt.Sprintf("%d ns", ns)
	case ns < 1000000:
		return fmt.Sprintf("%.1f us", float64(ns)/1e3)
	default:
		return fmt.Sprintf("%.1f ms", float64(ns)/1e6)
	}
}

// cellDeployEvaluateDCLock judges the samples against cell_deploy.dc_lock.
// It refuses a missing bound, an unread or unparsable register, fewer than
// cellDeployDCRounds readable samples on any DC slave, and any magnitude
// above the bound; IgH's "did not sync" lines are reported, never judged
// alone (the clock can converge after IgH's 5 s start check).
func cellDeployEvaluateDCLock(ctx cellDeployContext, dc *cellDeployDCFacts) (refuse, notes []string) {
	if len(ctx.DCSlaves) == 0 {
		return nil, []string{"DC lock not checked: the machine config sets bus_bringup.no_dc, so no slave runs a distributed clock"}
	}
	lock := ctx.Deploy.DCLock
	if lock == nil {
		return []string{fmt.Sprintf("cell_deploy.dc_lock is missing from %s: the bound on |ESC 0x092C| is cell configuration and verify will not pass a cell without it; add {\"max_system_time_difference_ns\", \"settle_s\"} (steamdeck/real/v4/README.md, \"Adding a new robot cell\")", ctx.Manifest)}, nil
	}
	if dc == nil {
		return []string{"DC lock not read: the servo host printed no distributed-clock samples"}, nil
	}
	if dc.Skipped != "" {
		return []string{"DC lock not read: " + dc.Skipped}, nil
	}
	bound := lock.MaxSystemTimeDifferenceNs
	byPosition := map[int][]string{}
	worst := map[int]int64{}
	readable := map[int]int{}
	samples := append([]cellDeployDCSample(nil), dc.Samples...)
	sort.SliceStable(samples, func(i, j int) bool { return samples[i].Round < samples[j].Round })
	for _, s := range samples {
		word, mag, err := cellDeployDCSystemTimeDifference(s.Raw)
		if err != nil {
			refuse = append(refuse, fmt.Sprintf("DC lock unreadable: slave %d ESC 0x092C round %d: %v", s.Position, s.Round, err))
			byPosition[s.Position] = append(byPosition[s.Position], "unread")
			continue
		}
		readable[s.Position]++
		byPosition[s.Position] = append(byPosition[s.Position], fmt.Sprintf("%s (0x%08x)", cellDeployDCTime(int64(mag)), word))
		if int64(mag) > worst[s.Position] {
			worst[s.Position] = int64(mag)
		}
	}
	for _, p := range ctx.DCSlaves {
		if readable[p] < cellDeployDCRounds {
			refuse = append(refuse, fmt.Sprintf("DC lock not shown: slave %d gave %d readable 0x092C sample(s) of %d", p, readable[p], cellDeployDCRounds))
		}
		if w := worst[p]; w > bound {
			refuse = append(refuse, fmt.Sprintf("EtherCAT distributed clock NOT LOCKED: slave %d system time difference (ESC 0x092C) reads %s, over the %s bound (cell_deploy.dc_lock), %d s after rosie-rt-core started. The drive's sync drifts through the cycle and returns stale then doubled feedback, which can latch coordinate_reference. Do not run the robot; report it (rt-core must write the reference clock every cycle, 9bac73cd)",
				p, cellDeployDCTime(w), cellDeployDCTime(bound), dc.CoreUpS))
		}
		notes = append(notes, fmt.Sprintf("DC slave %d |0x092C| %s (bound %s)", p, strings.Join(byPosition[p], ", "), cellDeployDCTime(bound)))
	}
	wait := ""
	if dc.SettleWaitS > 0 {
		wait = fmt.Sprintf(", waited %d s for it", dc.SettleWaitS)
	}
	notes = append(notes, fmt.Sprintf("DC lock sampled %d round(s) ~1 s apart from %d s after rosie-rt-core started (settle %d s%s)", cellDeployDCRounds, dc.CoreUpS, lock.SettleS, wait))
	switch {
	case dc.NotSynced < 0:
		notes = append(notes, "IgH \"Slave did not sync\" kernel lines since the start: not read")
	case dc.NotSynced == 0:
		notes = append(notes, "IgH \"Slave did not sync\" kernel lines since the start: none")
	default:
		notes = append(notes, fmt.Sprintf("IgH \"Slave did not sync\" kernel lines since the start: %d (IgH's 5 s start check; judged by the samples above): %s", dc.NotSynced, strings.Join(dc.KernelLines, " | ")))
	}
	return refuse, notes
}

// cellDeployDCPlan is the dry-run line for the check.
func cellDeployDCPlan(ctx cellDeployContext) string {
	switch {
	case len(ctx.DCSlaves) == 0:
		return "dc lock: not checked (machine config bus_bringup.no_dc)"
	case ctx.Deploy.DCLock == nil:
		return "dc lock: cell_deploy.dc_lock MISSING; verify will refuse"
	}
	l := ctx.Deploy.DCLock
	return fmt.Sprintf("dc lock: ESC 0x092C on slave(s) %s, %d samples ~1 s apart once rosie-rt-core has been active %d s; refuse any |difference| over %s (cell_deploy.dc_lock)",
		cellDeployDCPositions(ctx), cellDeployDCRounds, l.SettleS, cellDeployDCTime(l.MaxSystemTimeDifferenceNs))
}

// cellDeployDCPositions is the space-separated slave position list the servo
// script reads ("" when nothing is to be read).
func cellDeployDCPositions(ctx cellDeployContext) string {
	if ctx.Deploy.DCLock == nil {
		return ""
	}
	out := make([]string, len(ctx.DCSlaves))
	for i, p := range ctx.DCSlaves {
		out[i] = strconv.Itoa(p)
	}
	return strings.Join(out, " ")
}
