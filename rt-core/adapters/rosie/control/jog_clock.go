package control

import (
	"errors"
	"math"
	"math/bits"
)

var (
	ErrJogClockUnqualified        = errors.New("jog_clock_unqualified")
	ErrJogClockInvalid            = errors.New("jog_clock_invalid_budget_or_exchange")
	ErrJogClockIdentity           = errors.New("jog_clock_incarnation_mismatch")
	ErrJogClockGeneration         = errors.New("jog_clock_mapping_generation_mismatch")
	ErrJogClockBackward           = errors.New("jog_clock_moved_backwards")
	ErrJogClockArithmetic         = errors.New("jog_clock_arithmetic_range")
	ErrJogClockUncertainty        = errors.New("jog_clock_uncertainty_exceeded")
	ErrJogClockCalibrationExpired = errors.New("jog_clock_calibration_expired")
	ErrJogClockInconsistent       = errors.New("jog_clock_exchange_inconsistent")
	ErrJogClockInputStale         = errors.New("jog_input_too_old")
	ErrJogClockInputFuture        = errors.New("jog_input_entirely_future")
	ErrJogClockInputExpired       = errors.New("jog_input_deadline_expired")
)

// JogClockIncarnation identifies a particular monotonic clock lifetime. Tokens
// and their relationship are established by authenticated negotiation, never by
// trusting a frame's claim that its timestamps use the host clock.
type JogClockIncarnation [16]byte

type JogClockBinding struct {
	Host   JogClockIncarnation
	Source JogClockIncarnation
}

// JogClockLimits comes from qualified deployment policy, not a jog producer.
// All quantities are nanoseconds except DriftPPB: the upper bound on absolute
// offset change per source-clock nanosecond, in parts per billion. Thus an
// interval expands by ceil(source_distance * DriftPPB / 1e9) at each end.
// Remote clocks require positive drift below 1e9; an explicitly negotiated
// identical host/source clock requires zero drift. Qualified records an external
// qualification decision; these pure functions do not measure or authenticate it.
type JogClockLimits struct {
	Qualified           bool
	MaxInputAgeNS       uint64 // Also caps input lifetime; the exact boundary is expired.
	MaxInputDurationNS  uint64 // Requested-duration budget; effective duration also respects MaxInputAgeNS.
	MaxCalibrationAgeNS uint64
	MaxUncertaintyNS    uint64 // Full interval width, not a half-width.
	DriftPPB            uint64
}

// JogClockExchange contains four supplied monotonic timestamps from one
// authenticated request/reply exchange, in causal order. Delays need not be
// symmetric. Host and source timestamps must use the negotiated incarnations.
type JogClockExchange struct {
	HostSendNS      uint64
	SourceReceiveNS uint64
	SourceSendNS    uint64
	HostReceiveNS   uint64
}

// JogClockMapping is immutable. Its offset interval is anchored at sourceAnchor,
// not at receipt: an input captured before calibration also needs drift expansion.
// The zero value is unqualified. It contains no clock, network or authority owner.
type JogClockMapping struct {
	binding        JogClockBinding
	generation     uint64
	limits         JogClockLimits
	lo, hi         int64
	sourceAnchor   uint64
	hostCalibrated uint64
	local          bool
	qualified      bool
}

func (m JogClockMapping) Generation() uint64 { return m.generation }

// NewRemoteJogClock accepts only timestamps supplied by the authenticated
// exchange owner. This policy cannot establish authentication from numeric data.
func NewRemoteJogClock(binding JogClockBinding, generation uint64,
	exchange JogClockExchange, limits JogClockLimits) (JogClockMapping, error) {
	if err := validateJogClock(binding, generation, limits, false); err != nil {
		return JogClockMapping{}, err
	}
	lo, hi, err := jogClockExchangeInterval(exchange, limits)
	if err != nil {
		return JogClockMapping{}, err
	}
	return JogClockMapping{binding: binding, generation: generation, limits: limits,
		lo: lo, hi: hi, sourceAnchor: exchange.SourceSendNS,
		hostCalibrated: exchange.HostReceiveNS, qualified: true}, nil
}

// NewLocalJogClock requires trusted negotiation establishing that both tokens
// name the exact same monotonic clock. Equality of untrusted frame fields alone
// is not this proof. The incarnation and mapping generation still fence inputs.
func NewLocalJogClock(binding JogClockBinding, generation, hostNowNS uint64,
	limits JogClockLimits) (JogClockMapping, error) {
	if err := validateJogClock(binding, generation, limits, true); err != nil {
		return JogClockMapping{}, err
	}
	return JogClockMapping{binding: binding, generation: generation, limits: limits,
		sourceAnchor: hostNowNS, hostCalibrated: hostNowNS, local: true, qualified: true}, nil
}

// Refresh intersects a new exchange with the previous interval expanded to the
// new source anchor. A stale or excessively uncertain prior estimate cannot be
// reused: a valid fresh exchange then allocates the next mapping generation.
// Contradictory qualified exchanges are rejected, not hidden by averaging or a
// fresh generation. A new clock incarnation requires separate negotiation.
func (m JogClockMapping) Refresh(binding JogClockBinding, exchange JogClockExchange) (JogClockMapping, error) {
	if !m.qualified {
		return JogClockMapping{}, ErrJogClockUnqualified
	}
	if m.local {
		return JogClockMapping{}, ErrJogClockInvalid
	}
	if binding != m.binding {
		return JogClockMapping{}, ErrJogClockIdentity
	}
	if exchange.HostSendNS < m.hostCalibrated || exchange.SourceReceiveNS < m.sourceAnchor {
		return JogClockMapping{}, ErrJogClockBackward
	}
	next, err := NewRemoteJogClock(binding, m.generation, exchange, m.limits)
	if err != nil {
		return JogClockMapping{}, err
	}
	lo, hi, err := m.intervalAt(next.sourceAnchor)
	if next.hostCalibrated-m.hostCalibrated > m.limits.MaxCalibrationAgeNS ||
		errors.Is(err, ErrJogClockUncertainty) {
		if m.generation == math.MaxUint64 {
			return JogClockMapping{}, ErrJogClockArithmetic
		}
		next.generation++
		return next, nil
	}
	if err != nil {
		return JogClockMapping{}, err
	}
	if lo > next.lo {
		next.lo = lo
	}
	if hi < next.hi {
		next.hi = hi
	}
	if next.lo > next.hi {
		return JogClockMapping{}, ErrJogClockInconsistent
	}
	return next, nil
}

// RefreshLocal renews explicit same-clock negotiation using supplied host time.
// An expired negotiation advances the generation just as a remote refresh does.
func (m JogClockMapping) RefreshLocal(binding JogClockBinding, hostNowNS uint64) (JogClockMapping, error) {
	if !m.qualified {
		return JogClockMapping{}, ErrJogClockUnqualified
	}
	if !m.local {
		return JogClockMapping{}, ErrJogClockInvalid
	}
	if binding != m.binding {
		return JogClockMapping{}, ErrJogClockIdentity
	}
	if hostNowNS < m.hostCalibrated {
		return JogClockMapping{}, ErrJogClockBackward
	}
	if hostNowNS-m.hostCalibrated > m.limits.MaxCalibrationAgeNS {
		if m.generation == math.MaxUint64 {
			return JogClockMapping{}, ErrJogClockArithmetic
		}
		m.generation++
	}
	m.hostCalibrated, m.sourceAnchor = hostNowNS, hostNowNS
	return m, nil
}

// JogClockInput includes trusted ingress observations alongside producer fields.
// HostNowNS and GrantExpiresHostNS are supplied by the adapter in the negotiated
// host monotonic domain; requested duration and source origin come from the input.
type JogClockInput struct {
	Binding             JogClockBinding
	MappingGeneration   uint64
	SourceOriginNS      uint64
	HostNowNS           uint64
	RequestedDurationNS uint64
	GrantExpiresHostNS  uint64
}

// JogClockAdmission is an immutable result to carry to native admission. A grant
// renewal or later publication must not recompute and extend this input's expiry.
// Callers still enforce sequence, session, mode and authority checks separately.
type JogClockAdmission struct {
	binding      JogClockBinding
	generation   uint64
	sourceOrigin uint64
	hostObserved uint64
	earliest     uint64
	latest       uint64
	deadline     uint64
	worstAge     uint64
	qualified    bool
}

func (a JogClockAdmission) EarliestHostOriginNS() uint64 { return a.earliest }
func (a JogClockAdmission) LatestHostOriginNS() uint64   { return a.latest }
func (a JogClockAdmission) DeadlineNS() uint64           { return a.deadline }
func (a JogClockAdmission) WorstCaseAgeNS() uint64       { return a.worstAge }

// Shorten returns a copy whose deadline cannot exceed this admission's deadline.
// A zero/past ceiling deliberately makes it expired; it cannot restore motion.
func (a JogClockAdmission) Shorten(deadlineCeilingNS uint64) JogClockAdmission {
	if deadlineCeilingNS < a.deadline {
		a.deadline = deadlineCeilingNS
	}
	return a
}

// Admit maps an input conservatively. previous is the last accepted observation
// for this mapping/session, or nil for its first input. Retaining it detects
// backwards host/source observations and prevents a repeated source timestamp
// from gaining lifetime after grant renewal or recalibration. Sequence replay
// rejection remains mandatory even when a timestamp is equal (coarse clocks).
func (m JogClockMapping) Admit(input JogClockInput, previous *JogClockAdmission) (JogClockAdmission, error) {
	if !m.qualified {
		return JogClockAdmission{}, ErrJogClockUnqualified
	}
	if input.Binding != m.binding {
		return JogClockAdmission{}, ErrJogClockIdentity
	}
	if input.MappingGeneration != m.generation {
		return JogClockAdmission{}, ErrJogClockGeneration
	}
	if input.HostNowNS < m.hostCalibrated {
		return JogClockAdmission{}, ErrJogClockBackward
	}
	if input.HostNowNS-m.hostCalibrated > m.limits.MaxCalibrationAgeNS {
		return JogClockAdmission{}, ErrJogClockCalibrationExpired
	}
	if input.RequestedDurationNS == 0 || input.RequestedDurationNS > m.limits.MaxInputDurationNS {
		return JogClockAdmission{}, ErrJogClockInvalid
	}
	if previous != nil {
		if !previous.qualified {
			return JogClockAdmission{}, ErrJogClockUnqualified
		}
		if previous.binding != m.binding {
			return JogClockAdmission{}, ErrJogClockIdentity
		}
		if previous.generation != m.generation {
			return JogClockAdmission{}, ErrJogClockGeneration
		}
		if input.HostNowNS < previous.hostObserved || input.SourceOriginNS < previous.sourceOrigin {
			return JogClockAdmission{}, ErrJogClockBackward
		}
	}
	lo, hi, err := m.intervalAt(input.SourceOriginNS)
	if err != nil {
		return JogClockAdmission{}, err
	}
	earliest, ok := jogClockAddOffset(input.SourceOriginNS, lo)
	if !ok {
		return JogClockAdmission{}, ErrJogClockArithmetic
	}
	latest, ok := jogClockAddOffset(input.SourceOriginNS, hi)
	if !ok {
		return JogClockAdmission{}, ErrJogClockArithmetic
	}
	if earliest > input.HostNowNS {
		return JogClockAdmission{}, ErrJogClockInputFuture
	}
	age := input.HostNowNS - earliest
	if age > m.limits.MaxInputAgeNS {
		return JogClockAdmission{}, ErrJogClockInputStale
	}
	if age == m.limits.MaxInputAgeNS {
		return JogClockAdmission{}, ErrJogClockInputExpired
	}
	if input.RequestedDurationNS > math.MaxUint64-earliest || m.limits.MaxInputAgeNS > math.MaxUint64-earliest {
		return JogClockAdmission{}, ErrJogClockArithmetic
	}
	deadline := earliest + input.RequestedDurationNS
	// Native uses max_input_age_ns as both freshness and lifetime ceiling.
	// A longer well-formed requested duration must never outlive that ceiling.
	if ageDeadline := earliest + m.limits.MaxInputAgeNS; ageDeadline < deadline {
		deadline = ageDeadline
	}
	if input.GrantExpiresHostNS < deadline {
		deadline = input.GrantExpiresHostNS
	}
	if previous != nil && previous.sourceOrigin == input.SourceOriginNS && previous.deadline < deadline {
		deadline = previous.deadline
	}
	if deadline <= input.HostNowNS {
		return JogClockAdmission{}, ErrJogClockInputExpired
	}
	return JogClockAdmission{binding: m.binding, generation: m.generation,
		sourceOrigin: input.SourceOriginNS, hostObserved: input.HostNowNS,
		earliest: earliest, latest: latest, deadline: deadline, worstAge: age, qualified: true}, nil
}

func validateJogClock(binding JogClockBinding, generation uint64, limits JogClockLimits, local bool) error {
	if !limits.Qualified {
		return ErrJogClockUnqualified
	}
	if binding.Host == (JogClockIncarnation{}) || binding.Source == (JogClockIncarnation{}) ||
		(local && binding.Host != binding.Source) {
		return ErrJogClockIdentity
	}
	if generation == 0 {
		return ErrJogClockGeneration
	}
	if limits.MaxInputAgeNS == 0 || limits.MaxInputDurationNS == 0 ||
		limits.MaxCalibrationAgeNS == 0 || limits.MaxUncertaintyNS == 0 ||
		limits.DriftPPB >= 1_000_000_000 || (local && limits.DriftPPB != 0) || (!local && limits.DriftPPB == 0) {
		return ErrJogClockInvalid
	}
	return nil
}

func jogClockExchangeInterval(e JogClockExchange, limits JogClockLimits) (int64, int64, error) {
	if e.HostReceiveNS < e.HostSendNS || e.SourceSendNS < e.SourceReceiveNS {
		return 0, 0, ErrJogClockBackward
	}
	lo, ok := jogClockDifference(e.HostSendNS, e.SourceReceiveNS)
	if !ok {
		return 0, 0, ErrJogClockArithmetic
	}
	hi, ok := jogClockDifference(e.HostReceiveNS, e.SourceSendNS)
	if !ok {
		return 0, 0, ErrJogClockArithmetic
	}
	// At source receive, offset >= host_send-source_receive. Between receive
	// and send it may fall by the drift bound. At source send, causality gives
	// offset <= host_receive-source_send. Subtracting source processing tightens
	// the uncertainty; no half-RTT/symmetric-delay approximation is involved.
	lo, ok = jogClockOffsetDown(lo, jogClockDrift(e.SourceSendNS-e.SourceReceiveNS, limits.DriftPPB))
	if !ok {
		return 0, 0, ErrJogClockArithmetic
	}
	if lo > hi {
		return 0, 0, ErrJogClockInconsistent
	}
	if jogClockWidth(lo, hi) > limits.MaxUncertaintyNS {
		return 0, 0, ErrJogClockUncertainty
	}
	return lo, hi, nil
}

func (m JogClockMapping) intervalAt(sourceNS uint64) (int64, int64, error) {
	var distance uint64
	if sourceNS < m.sourceAnchor {
		distance = m.sourceAnchor - sourceNS
	} else {
		distance = sourceNS - m.sourceAnchor
	}
	drift := jogClockDrift(distance, m.limits.DriftPPB)
	lo, ok := jogClockOffsetDown(m.lo, drift)
	if !ok {
		return 0, 0, ErrJogClockArithmetic
	}
	hi, ok := jogClockOffsetUp(m.hi, drift)
	if !ok {
		return 0, 0, ErrJogClockArithmetic
	}
	if jogClockWidth(lo, hi) > m.limits.MaxUncertaintyNS {
		return 0, 0, ErrJogClockUncertainty
	}
	return lo, hi, nil
}

func jogClockDrift(distance, ppb uint64) uint64 {
	// ppb < 1e9 makes the quotient fit uint64 even when the product does not.
	hi, lo := bits.Mul64(distance, ppb)
	quotient, remainder := bits.Div64(hi, lo, 1_000_000_000)
	if remainder != 0 {
		quotient++
	}
	return quotient
}

func jogClockDifference(a, b uint64) (int64, bool) {
	if a >= b {
		if a-b > math.MaxInt64 {
			return 0, false
		}
		return int64(a - b), true
	}
	return jogClockNegative(b - a)
}

func jogClockMagnitude(value int64) uint64 {
	if value == 0 {
		return 0
	}
	return uint64(-(value + 1)) + 1
}

func jogClockNegative(magnitude uint64) (int64, bool) {
	if magnitude > uint64(1)<<63 {
		return 0, false
	}
	if magnitude == uint64(1)<<63 {
		return math.MinInt64, true
	}
	return -int64(magnitude), true
}

func jogClockOffsetDown(value int64, amount uint64) (int64, bool) {
	if value <= 0 {
		magnitude := jogClockMagnitude(value)
		if amount > (uint64(1)<<63)-magnitude {
			return 0, false
		}
		return jogClockNegative(magnitude + amount)
	}
	if amount <= uint64(value) {
		return value - int64(amount), true
	}
	return jogClockNegative(amount - uint64(value))
}

func jogClockOffsetUp(value int64, amount uint64) (int64, bool) {
	if value >= 0 {
		if amount > math.MaxInt64-uint64(value) {
			return 0, false
		}
		return value + int64(amount), true
	}
	magnitude := jogClockMagnitude(value)
	if amount < magnitude {
		return jogClockNegative(magnitude - amount)
	}
	if amount-magnitude > math.MaxInt64 {
		return 0, false
	}
	return int64(amount - magnitude), true
}

func jogClockAddOffset(source uint64, offset int64) (uint64, bool) {
	if offset >= 0 {
		if uint64(offset) > math.MaxUint64-source {
			return 0, false
		}
		return source + uint64(offset), true
	}
	magnitude := jogClockMagnitude(offset)
	if magnitude > source {
		return 0, false
	}
	return source - magnitude, true
}

func jogClockWidth(lo, hi int64) uint64 {
	if lo >= 0 {
		return uint64(hi) - uint64(lo)
	}
	if hi < 0 {
		return jogClockMagnitude(lo) - jogClockMagnitude(hi)
	}
	return uint64(hi) + jogClockMagnitude(lo)
}
