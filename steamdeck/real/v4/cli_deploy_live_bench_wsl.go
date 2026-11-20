package main

const (

// Motion Server accepts Steam Deck leader identities only in the
// steamdeck:steamdeck-<digits> namespace. Keep the WSL deployment target
// name separate from the controller identity used on the live command bus.

// The child may spend 2.5s readiness + 8s Arm + 2.5s admission +
// 95s completion + 2.5s Disarm before its first atomic proof write.
// Keep the parent comfortably above that 110.5s sequential bound.

)

const (

// AlarmGuard owns a bounded HM35 settle/rearm state machine with a
// 90-second terminal timeout. Leave a small reporting margin so the outer
// proof monitor cannot kill that client before it records the real result.

)
