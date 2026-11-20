package denseexec

import "time"

// ManualIOSetupLifetime bounds the operator's setup session, not the physical
// output lease. Deck4 empty-session measurements included 125–349 ms replies;
// the old 150 ms token expired even with every output OFF. The native per-output
// 200 ms deadline remains independent: a late renewal is refused there and
// cannot revive an expired output without a new explicit Begin.
// No daemon timer renews ON; every native intent comes from a client sample.
const ManualIOSetupLifetime = 500 * time.Millisecond

// Every sample belongs to an explicit operator-started test session. Begin is
// an ON tap, never a heartbeat; expiry cannot silently recreate an ON intent.
type ManualIOSample struct {
	Channel string `json:"channel"`
	Value   bool   `json:"value"`
	Begin   bool   `json:"begin,omitempty"`
}
type ManualIORequest struct {
	Session  string           `json:"session"`
	Sequence uint64           `json:"sequence"`
	Samples  []ManualIOSample `json:"samples"`
	End      bool             `json:"end,omitempty"`
}
