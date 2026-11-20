# Executor ordering

Leader acquire/release serialize through a separate transition mutex. Mark the
leader inactive under its state mutex, release that mutex before executor revoke
or native I/O, then commit the new lease only after successful acquisition.
Renew, authorize and snapshot must never wait on socket I/O. Revocation completes
before the new leader epoch becomes active.
Snapshot leader state before taking the executor mutex; pass that snapshot
through ON payload construction. Keep tick output and abort output/state
transitions serialized so an old tick cannot launch an ON after stop/revoke
returns. The send-gated executor regressions cover both ordering and lock
inversion.

`torch_on` is local intent state. Detached NATS publication and an inactive
UDP send are not downstream acknowledgement or physical output evidence.

For ingest shutdown, join the accept thread before closing/resetting the
listener descriptor. Making the member atomic alone would still permit the
OS to reuse a descriptor that the accept thread is about to access.

The current `rt_core` backend uses the sibling rt-core public C++ client.
Never redirect the legacy UDP setpoint stream or use native IPC/raw HTTP.
Describe and bind machine/deployment identity before acquisition; preserve the
original uploaded bytes and verify the entire prepared identity before Start.
Persisted prepared metadata is diagnostic only, never authority after restart.
Pause is unsupported. Torch-bearing programs fail preload; native execution
does not own or publish torch output.

Native Start, Stop/release and preparation share the executor mutex. Acquisition,
commissioning and tick observations use a separate public client outside that mutex, with cancellation
and epoch/session validation after relocking and before Start. Stop cancels
pending observations before locking. Capture the leader snapshot first. The
atomic renewal deadline can extend only that captured epoch during commissioning,
without taking the leader mutex. Stop joins renewal, stops then releases the
authenticated native session, and invalidates
preparations. Cleanup stays serialized with acquisition. Reconcile timestamped,
inhibited standstill within the configured
observation window. Shutdown has one unverified 1000 ms I/O budget from entry
under the executor mutex, including the public client's bounded 250 ms renewal
join. Stop, Release and reconciliation share its absolute deadline; no operation
gets a fresh control timeout. This bounds shutdown's executor/leader-transition
lock occupancy on a hung adapter. Deadline exhaustion publishes
native_state=cleanup_uncertain, invalidates preparation and drops the client
before returning. It does not certify standstill. A fresh acquire reconciles
within the lesser of 1000 ms and the configured observation window.
Reconciliation accepts a changed incarnation only after distinct inhibited
samples from that same current incarnation and the approved configuration with
no active grant. Any active grant, including a foreign grant, reports
native_state=grant_active and consumer_action=reconcile_status_then_acquire;
re-check Status on the next acquire without stopping the foreign controller.
Unauthenticated preload precondition failures refuse only that request and must
not consume renewal results, cancel observations, or change session/leader state.
Persisted metadata remains diagnostic only. Event cursor loss reconciles Status
and resumes at the loss record sequence; publisher_overflow is an ordinary event.
NATS play and authority transitions run on separate owned workers so commissioning
cannot block Stop, revocation, renewal or status publication. Status reads a
separately locked published snapshot, including while acquisition performs I/O.
Publish commissioning_phase before each commissioning I/O phase so pending Home
or readiness remains visible to NATS consumers; clear it on completion or Stop.
Cancel work and join both workers and ingest before destroying the backend.

Use the composed Go tests and SDK transport-gated C++ regressions for native
changes. Keep the legacy follower and its ordering tests passing. Build outputs
stay under this component's build directory; WSL Unix sockets use TMPDIR on tmpfs.
