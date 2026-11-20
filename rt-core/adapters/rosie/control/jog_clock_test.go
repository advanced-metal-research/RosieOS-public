package control

import (
	"errors"
	"math"
	"testing"
)

func jogTestBinding() JogClockBinding {
	return JogClockBinding{Host: JogClockIncarnation{1}, Source: JogClockIncarnation{2}}
}

func jogTestLimits() JogClockLimits {
	return JogClockLimits{Qualified: true, MaxInputAgeNS: 100, MaxInputDurationNS: 200,
		MaxCalibrationAgeNS: 1000, MaxUncertaintyNS: 100, DriftPPB: 1_000_000}
}

func jogTestExchange() JogClockExchange {
	return JogClockExchange{HostSendNS: 1000, SourceReceiveNS: 900, SourceSendNS: 920, HostReceiveNS: 1050}
}

func jogTestMapping(t *testing.T) JogClockMapping {
	t.Helper()
	mapping, err := NewRemoteJogClock(jogTestBinding(), 7, jogTestExchange(), jogTestLimits())
	if err != nil {
		t.Fatal(err)
	}
	return mapping
}

func jogTestInput() JogClockInput {
	return JogClockInput{Binding: jogTestBinding(), MappingGeneration: 7,
		SourceOriginNS: 920, HostNowNS: 1060, RequestedDurationNS: 100, GrantExpiresHostNS: 2000}
}

func TestJogClockConservativeExchangeAndDeadline(t *testing.T) {
	mapping := jogTestMapping(t)
	admission, err := mapping.Admit(jogTestInput(), nil)
	if err != nil {
		t.Fatal(err)
	}
	// There are 50 host ticks of RTT and 20 source ticks of processing.
	// At source send the offset is [100 - ceil(20*.001), 130] = [99,130].
	// Deliberately asymmetric delay must not collapse this to a midpoint.
	if admission.EarliestHostOriginNS() != 1019 || admission.LatestHostOriginNS() != 1050 ||
		admission.WorstCaseAgeNS() != 41 || admission.DeadlineNS() != 1119 {
		t.Fatalf("unexpected conservative admission: %+v", admission)
	}
	input := jogTestInput()
	input.GrantExpiresHostNS = 1100
	admission, err = mapping.Admit(input, nil)
	if err != nil || admission.DeadlineNS() != 1100 {
		t.Fatalf("grant did not cap input lifetime: %+v, %v", admission, err)
	}
	input.HostNowNS = 1070
	input.GrantExpiresHostNS = 2000
	input.RequestedDurationNS = 200
	repeated, err := mapping.Admit(input, &admission)
	if err != nil || repeated.DeadlineNS() != 1100 {
		t.Fatalf("same captured input gained lifetime after renewal: %+v, %v", repeated, err)
	}
	if got := admission.Shorten(1090).Shorten(2000).DeadlineNS(); got != 1090 {
		t.Fatalf("deadline could be extended: %d", got)
	}
	if admission.DeadlineNS() != 1100 {
		t.Fatal("shortening mutated the original receipt")
	}
}

func TestJogClockDriftAppliesBeforeAndAfterCalibration(t *testing.T) {
	mapping := jogTestMapping(t)
	for _, tc := range []struct {
		name           string
		source, lo, hi uint64
	}{
		{"before calibration", 900, 998, 1031},
		{"after calibration", 930, 1028, 1061},
	} {
		t.Run(tc.name, func(t *testing.T) {
			input := jogTestInput()
			input.SourceOriginNS = tc.source
			admission, err := mapping.Admit(input, nil)
			if err != nil || admission.EarliestHostOriginNS() != tc.lo || admission.LatestHostOriginNS() != tc.hi {
				t.Fatalf("source-relative drift lost: %+v, %v", admission, err)
			}
		})
	}
}

func TestJogClockAdmissionBoundaries(t *testing.T) {
	mapping := jogTestMapping(t)
	for _, tc := range []struct {
		name   string
		change func(*JogClockInput)
		want   error
	}{
		{"age one tick below limit", func(i *JogClockInput) { i.HostNowNS = 1118; i.RequestedDurationNS = 200 }, nil},
		{"age at limit", func(i *JogClockInput) { i.HostNowNS = 1119; i.RequestedDurationNS = 200 }, ErrJogClockInputExpired},
		{"age past limit", func(i *JogClockInput) { i.HostNowNS = 1120; i.RequestedDurationNS = 200 }, ErrJogClockInputStale},
		{"whole interval future", func(i *JogClockInput) { i.SourceOriginNS = 1000 }, ErrJogClockInputFuture},
		{"interval straddles now", func(i *JogClockInput) { i.SourceOriginNS = 930 }, nil},
		{"deadline equals now", func(i *JogClockInput) { i.RequestedDurationNS = 41 }, ErrJogClockInputExpired},
		{"deadline next tick", func(i *JogClockInput) { i.RequestedDurationNS = 42 }, nil},
		{"grant equals now", func(i *JogClockInput) { i.GrantExpiresHostNS = i.HostNowNS }, ErrJogClockInputExpired},
		{"zero grant", func(i *JogClockInput) { i.GrantExpiresHostNS = 0 }, ErrJogClockInputExpired},
		{"zero duration", func(i *JogClockInput) { i.RequestedDurationNS = 0 }, ErrJogClockInvalid},
		{"duration above budget", func(i *JogClockInput) { i.RequestedDurationNS = 201 }, ErrJogClockInvalid},
		{"old mapping generation", func(i *JogClockInput) { i.MappingGeneration-- }, ErrJogClockGeneration},
		{"source reincarnation", func(i *JogClockInput) { i.Binding.Source[0]++ }, ErrJogClockIdentity},
		{"host reincarnation", func(i *JogClockInput) { i.Binding.Host[0]++ }, ErrJogClockIdentity},
		{"host before calibration", func(i *JogClockInput) { i.HostNowNS = 1049 }, ErrJogClockBackward},
		{"expired calibration", func(i *JogClockInput) { i.HostNowNS = 2051 }, ErrJogClockCalibrationExpired},
		{"drift exceeds uncertainty", func(i *JogClockInput) { i.SourceOriginNS = 100000 }, ErrJogClockUncertainty},
	} {
		t.Run(tc.name, func(t *testing.T) {
			input := jogTestInput()
			tc.change(&input)
			_, err := mapping.Admit(input, nil)
			if !errors.Is(err, tc.want) {
				t.Fatalf("got %v, want %v", err, tc.want)
			}
		})
	}
	if _, err := (JogClockMapping{}).Admit(jogTestInput(), nil); !errors.Is(err, ErrJogClockUnqualified) {
		t.Fatalf("zero mapping admitted input: %v", err)
	}
}

func TestJogClockLifetimeCannotExceedNativeAgeBudget(t *testing.T) {
	mapping := jogTestMapping(t) // Maximum age 100; maximum requested duration 200.
	for _, tc := range []struct {
		name                                string
		duration, grantExpiry, wantDeadline uint64
	}{
		{"requested duration is shorter", 50, 2000, 1069},
		{"requested duration equals age ceiling", 100, 2000, 1119},
		{"requested duration exceeds age ceiling", 200, 2000, 1119},
		{"grant is shorter than both", 200, 1080, 1080},
	} {
		t.Run(tc.name, func(t *testing.T) {
			input := jogTestInput()
			input.RequestedDurationNS, input.GrantExpiresHostNS = tc.duration, tc.grantExpiry
			admission, err := mapping.Admit(input, nil)
			if err != nil || admission.DeadlineNS() != tc.wantDeadline {
				t.Fatalf("native lifetime ceiling mismatch: %+v, %v", admission, err)
			}
			if admission.DeadlineNS()-admission.EarliestHostOriginNS() > 100 {
				t.Fatal("admitted deadline would be refused by native max_input_age_ns")
			}
		})
	}
}

func TestJogClockObservationCannotMoveBackwards(t *testing.T) {
	mapping := jogTestMapping(t)
	previous, err := mapping.Admit(jogTestInput(), nil)
	if err != nil {
		t.Fatal(err)
	}
	input := jogTestInput()
	input.HostNowNS--
	if _, err := mapping.Admit(input, &previous); !errors.Is(err, ErrJogClockBackward) {
		t.Fatalf("backwards host observation accepted: %v", err)
	}
	input = jogTestInput()
	input.SourceOriginNS--
	if _, err := mapping.Admit(input, &previous); !errors.Is(err, ErrJogClockBackward) {
		t.Fatalf("backwards source observation accepted: %v", err)
	}
	if _, err := mapping.Admit(jogTestInput(), &JogClockAdmission{}); !errors.Is(err, ErrJogClockUnqualified) {
		t.Fatalf("unqualified prior observation accepted: %v", err)
	}
}

func TestJogClockRejectsUnqualifiedAndInvalidCalibration(t *testing.T) {
	for _, tc := range []struct {
		name   string
		change func(*JogClockBinding, *uint64, *JogClockExchange, *JogClockLimits)
		want   error
	}{
		{"unqualified", func(_ *JogClockBinding, _ *uint64, _ *JogClockExchange, l *JogClockLimits) { l.Qualified = false }, ErrJogClockUnqualified},
		{"zero host", func(b *JogClockBinding, _ *uint64, _ *JogClockExchange, _ *JogClockLimits) {
			b.Host = JogClockIncarnation{}
		}, ErrJogClockIdentity},
		{"zero source", func(b *JogClockBinding, _ *uint64, _ *JogClockExchange, _ *JogClockLimits) {
			b.Source = JogClockIncarnation{}
		}, ErrJogClockIdentity},
		{"zero generation", func(_ *JogClockBinding, g *uint64, _ *JogClockExchange, _ *JogClockLimits) { *g = 0 }, ErrJogClockGeneration},
		{"zero input age", func(_ *JogClockBinding, _ *uint64, _ *JogClockExchange, l *JogClockLimits) { l.MaxInputAgeNS = 0 }, ErrJogClockInvalid},
		{"zero duration", func(_ *JogClockBinding, _ *uint64, _ *JogClockExchange, l *JogClockLimits) { l.MaxInputDurationNS = 0 }, ErrJogClockInvalid},
		{"zero calibration age", func(_ *JogClockBinding, _ *uint64, _ *JogClockExchange, l *JogClockLimits) { l.MaxCalibrationAgeNS = 0 }, ErrJogClockInvalid},
		{"zero uncertainty", func(_ *JogClockBinding, _ *uint64, _ *JogClockExchange, l *JogClockLimits) { l.MaxUncertaintyNS = 0 }, ErrJogClockInvalid},
		{"zero remote drift", func(_ *JogClockBinding, _ *uint64, _ *JogClockExchange, l *JogClockLimits) { l.DriftPPB = 0 }, ErrJogClockInvalid},
		{"invalid drift", func(_ *JogClockBinding, _ *uint64, _ *JogClockExchange, l *JogClockLimits) {
			l.DriftPPB = 1_000_000_000
		}, ErrJogClockInvalid},
		{"host backwards", func(_ *JogClockBinding, _ *uint64, e *JogClockExchange, _ *JogClockLimits) {
			e.HostReceiveNS = e.HostSendNS - 1
		}, ErrJogClockBackward},
		{"source backwards", func(_ *JogClockBinding, _ *uint64, e *JogClockExchange, _ *JogClockLimits) {
			e.SourceSendNS = e.SourceReceiveNS - 1
		}, ErrJogClockBackward},
		{"impossible processing", func(_ *JogClockBinding, _ *uint64, e *JogClockExchange, _ *JogClockLimits) {
			e.SourceSendNS = 1000
			e.HostReceiveNS = 1010
		}, ErrJogClockInconsistent},
		{"excessive RTT uncertainty", func(_ *JogClockBinding, _ *uint64, e *JogClockExchange, _ *JogClockLimits) { e.HostReceiveNS = 2000 }, ErrJogClockUncertainty},
	} {
		t.Run(tc.name, func(t *testing.T) {
			binding, generation, exchange, limits := jogTestBinding(), uint64(7), jogTestExchange(), jogTestLimits()
			tc.change(&binding, &generation, &exchange, &limits)
			_, err := NewRemoteJogClock(binding, generation, exchange, limits)
			if !errors.Is(err, tc.want) {
				t.Fatalf("got %v, want %v", err, tc.want)
			}
		})
	}
}

func TestJogClockRefreshIntersectsAndFencesFreshMappings(t *testing.T) {
	mapping := jogTestMapping(t)
	exchange := JogClockExchange{HostSendNS: 1100, SourceReceiveNS: 1000, SourceSendNS: 1020, HostReceiveNS: 1140}
	refreshed, err := mapping.Refresh(jogTestBinding(), exchange)
	if err != nil || refreshed.Generation() != 7 || refreshed.lo != 99 || refreshed.hi != 120 {
		t.Fatalf("consistent exchange did not intersect: %+v, %v", refreshed, err)
	}
	bad := exchange
	bad.SourceReceiveNS, bad.SourceSendNS = 950, 970
	if _, err := mapping.Refresh(jogTestBinding(), bad); !errors.Is(err, ErrJogClockInconsistent) {
		t.Fatalf("contradictory clocks were averaged or requalified: %v", err)
	}
	bad = exchange
	bad.HostSendNS = 1049
	if _, err := mapping.Refresh(jogTestBinding(), bad); !errors.Is(err, ErrJogClockBackward) {
		t.Fatalf("backwards host exchange accepted: %v", err)
	}
	bad = exchange
	bad.SourceReceiveNS = 919
	if _, err := mapping.Refresh(jogTestBinding(), bad); !errors.Is(err, ErrJogClockBackward) {
		t.Fatalf("backwards source exchange accepted: %v", err)
	}
	binding := jogTestBinding()
	binding.Source[0]++
	if _, err := mapping.Refresh(binding, exchange); !errors.Is(err, ErrJogClockIdentity) {
		t.Fatalf("refresh adopted another incarnation: %v", err)
	}
	for _, reason := range []string{"age", "uncertainty"} {
		t.Run(reason, func(t *testing.T) {
			limits := jogTestLimits()
			if reason == "age" {
				limits.MaxCalibrationAgeNS = 89
			} else {
				limits.MaxUncertaintyNS = 31
			}
			prior, err := NewRemoteJogClock(jogTestBinding(), 7, jogTestExchange(), limits)
			if err != nil {
				t.Fatal(err)
			}
			next, err := prior.Refresh(jogTestBinding(), exchange)
			if err != nil || next.Generation() != 8 {
				t.Fatalf("fresh mapping did not rotate generation: %+v, %v", next, err)
			}
			input := jogTestInput()
			input.HostNowNS, input.SourceOriginNS = 1141, 1020
			if _, err := next.Admit(input, nil); !errors.Is(err, ErrJogClockGeneration) {
				t.Fatalf("old mapping frame survived replacement: %v", err)
			}
			input.MappingGeneration = 8
			if _, err := next.Admit(input, nil); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestJogClockRecalibrationCannotExtendCapturedInput(t *testing.T) {
	mapping := jogTestMapping(t)
	input := jogTestInput()
	input.RequestedDurationNS = 200
	first, err := mapping.Admit(input, nil)
	if err != nil {
		t.Fatal(err)
	}
	refreshed, err := mapping.Refresh(jogTestBinding(), JogClockExchange{
		HostSendNS: 1061, SourceReceiveNS: 950, SourceSendNS: 960, HostReceiveNS: 1080})
	if err != nil {
		t.Fatal(err)
	}
	input.HostNowNS = 1081
	second, err := refreshed.Admit(input, &first)
	if err != nil || second.DeadlineNS() != first.DeadlineNS() || second.EarliestHostOriginNS() != 1029 {
		t.Fatalf("refined estimate extended existing input: first=%+v next=%+v err=%v", first, second, err)
	}
}

func TestJogClockSameHostRequiresExplicitNegotiatedIdentity(t *testing.T) {
	limits, binding := jogTestLimits(), jogTestBinding()
	limits.DriftPPB = 0
	if _, err := NewLocalJogClock(binding, 7, 1000, limits); !errors.Is(err, ErrJogClockIdentity) {
		t.Fatalf("different clocks accepted as same host: %v", err)
	}
	binding.Source = binding.Host
	mapping, err := NewLocalJogClock(binding, 7, 1000, limits)
	if err != nil {
		t.Fatal(err)
	}
	input := JogClockInput{Binding: binding, MappingGeneration: 7, SourceOriginNS: 1020,
		HostNowNS: 1050, RequestedDurationNS: 100, GrantExpiresHostNS: 2000}
	admission, err := mapping.Admit(input, nil)
	if err != nil || admission.EarliestHostOriginNS() != 1020 || admission.LatestHostOriginNS() != 1020 || admission.DeadlineNS() != 1120 {
		t.Fatalf("same-clock mapping changed time: %+v, %v", admission, err)
	}
	input.Binding.Source = jogTestBinding().Source
	if _, err := mapping.Admit(input, nil); !errors.Is(err, ErrJogClockIdentity) {
		t.Fatalf("frame forged clock domain: %v", err)
	}
	if _, err := mapping.RefreshLocal(binding, 999); !errors.Is(err, ErrJogClockBackward) {
		t.Fatalf("backwards local clock: %v", err)
	}
	current, err := mapping.RefreshLocal(binding, 2000) // Calibration age exactly at budget.
	if err != nil || current.Generation() != 7 {
		t.Fatalf("boundary refresh: %+v, %v", current, err)
	}
	fresh, err := mapping.RefreshLocal(binding, 2001)
	if err != nil || fresh.Generation() != 8 {
		t.Fatalf("expired local mapping not fenced: %+v, %v", fresh, err)
	}
}

func TestJogClockSignedOffsetsAndUint64Timestamps(t *testing.T) {
	for _, tc := range []struct {
		name                    string
		exchange                JogClockExchange
		origin, now, wantOrigin uint64
	}{
		{"negative offset", JogClockExchange{1000, 2000, 2020, 1050}, 2020, 1060, 1019},
		{"large unsigned clocks", JogClockExchange{math.MaxUint64 - 300, math.MaxUint64 - 400, math.MaxUint64 - 380, math.MaxUint64 - 250}, math.MaxUint64 - 380, math.MaxUint64 - 240, math.MaxUint64 - 281},
	} {
		t.Run(tc.name, func(t *testing.T) {
			mapping, err := NewRemoteJogClock(jogTestBinding(), 7, tc.exchange, jogTestLimits())
			if err != nil {
				t.Fatal(err)
			}
			input := jogTestInput()
			input.SourceOriginNS, input.HostNowNS, input.GrantExpiresHostNS = tc.origin, tc.now, math.MaxUint64
			admission, err := mapping.Admit(input, nil)
			if err != nil || admission.EarliestHostOriginNS() != tc.wantOrigin {
				t.Fatalf("unsigned timestamp truncated: %+v, %v", admission, err)
			}
		})
	}
	limits := jogTestLimits()
	limits.DriftPPB = 0
	binding := jogTestBinding()
	binding.Source = binding.Host
	mapping, err := NewLocalJogClock(binding, 7, math.MaxUint64, limits)
	if err != nil {
		t.Fatal(err)
	}
	input := JogClockInput{Binding: binding, MappingGeneration: 7, SourceOriginNS: math.MaxUint64,
		HostNowNS: math.MaxUint64, RequestedDurationNS: 1, GrantExpiresHostNS: math.MaxUint64}
	if _, err := mapping.Admit(input, nil); !errors.Is(err, ErrJogClockArithmetic) {
		t.Fatalf("deadline wrapped: %v", err)
	}
	mapping, err = NewLocalJogClock(binding, 7, math.MaxUint64-1, limits)
	if err != nil {
		t.Fatal(err)
	}
	input.SourceOriginNS, input.HostNowNS = math.MaxUint64-1, math.MaxUint64-1
	if _, err := mapping.Admit(input, nil); !errors.Is(err, ErrJogClockArithmetic) {
		t.Fatalf("age-ceiling addition wrapped while requested deadline fitted: %v", err)
	}
	remote, err := NewRemoteJogClock(jogTestBinding(), 7, JogClockExchange{1000, 2000, 2020, 1050}, jogTestLimits())
	if err != nil {
		t.Fatal(err)
	}
	input = jogTestInput()
	input.SourceOriginNS = 0
	if _, err := remote.Admit(input, nil); !errors.Is(err, ErrJogClockArithmetic) {
		t.Fatalf("negative origin wrapped: %v", err)
	}
	limits = jogTestLimits()
	limits.MaxCalibrationAgeNS = 89
	remote, err = NewRemoteJogClock(jogTestBinding(), math.MaxUint64, jogTestExchange(), limits)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := remote.Refresh(jogTestBinding(), JogClockExchange{1100, 1000, 1020, 1140}); !errors.Is(err, ErrJogClockArithmetic) {
		t.Fatalf("generation wrapped: %v", err)
	}
}

func TestJogClockArithmeticEdges(t *testing.T) {
	for _, tc := range []struct {
		a, b uint64
		want int64
		ok   bool
	}{
		{math.MaxInt64, 0, math.MaxInt64, true},
		{0, uint64(1) << 63, math.MinInt64, true},
		{math.MaxUint64, math.MaxUint64 - 12, 12, true},
		{uint64(1) << 63, 0, 0, false},
		{0, (uint64(1) << 63) + 1, 0, false},
	} {
		got, ok := jogClockDifference(tc.a, tc.b)
		if got != tc.want || ok != tc.ok {
			t.Fatalf("difference(%d,%d) = %d,%v", tc.a, tc.b, got, ok)
		}
	}
	if got, ok := jogClockOffsetDown(math.MaxInt64, math.MaxUint64); !ok || got != math.MinInt64 {
		t.Fatalf("full signed span down: %d,%v", got, ok)
	}
	if got, ok := jogClockOffsetUp(math.MinInt64, math.MaxUint64); !ok || got != math.MaxInt64 {
		t.Fatalf("full signed span up: %d,%v", got, ok)
	}
	if _, ok := jogClockOffsetDown(math.MinInt64, 1); ok {
		t.Fatal("signed lower bound wrapped")
	}
	if _, ok := jogClockOffsetUp(math.MaxInt64, 1); ok {
		t.Fatal("signed upper bound wrapped")
	}
	if got, ok := jogClockAddOffset(uint64(1)<<63, math.MinInt64); !ok || got != 0 {
		t.Fatalf("minimum offset application: %d,%v", got, ok)
	}
	if _, ok := jogClockAddOffset(math.MaxUint64, 1); ok {
		t.Fatal("unsigned upper bound wrapped")
	}
	if _, ok := jogClockAddOffset(0, -1); ok {
		t.Fatal("unsigned lower bound wrapped")
	}
	if jogClockWidth(math.MinInt64, math.MaxInt64) != math.MaxUint64 {
		t.Fatal("full interval width truncated")
	}
	for _, tc := range []struct{ distance, ppb, want uint64 }{
		{0, 999999999, 0}, {1, 1, 1}, {1000000000, 1, 1},
		{math.MaxUint64, 1, 18446744074}, {math.MaxUint64, 999999999, 18446744055262807542},
	} {
		if got := jogClockDrift(tc.distance, tc.ppb); got != tc.want {
			t.Fatalf("drift(%d,%d) = %d, want %d", tc.distance, tc.ppb, got, tc.want)
		}
	}
}
