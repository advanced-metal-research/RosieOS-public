# Public simulation soak (P8-03)

Run from `rt-core` in Linux/WSL with the built simulation daemon, real adapter,
and configuration compiler. The driver starts its own processes and never
connects to a physical machine or an existing daemon.

```sh
export PATH=/home/operator/.rosie/go-1.26.2/bin:$PATH
export GOCACHE=$PWD/build/go-cache GOMODCACHE=$PWD/build/go-mod-cache
export GOTMPDIR=$PWD/build/go-tmp GOFLAGS=-mod=mod
export TMPDIR=/dev/shm/rt-core/soak2
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR" "$TMPDIR"
make soak
make soak-check SOAK_RECORD=build/soak-$(bash tools/packageinfo/git.sh rev-parse --short HEAD).json
```

`make soak` runs two warm-ups and 50 measured iterations. Each iteration
prepares a 250,000-point stationary J1 trajectory, starts it, observes the
matching handle/start sequence at completed point 249999, and acknowledges
Discard. Every fourth iteration also discards a full preparation before execution.
Between executions a local jog lane moves J1 in alternating directions while
another 250,000-point trajectory uploads. That overlap trajectory is discarded
without activation. Jog uses the SDK's prepared-session handshake, then sends
at least 200 fresh inputs, continuing until upload returns. Three simultaneous
small preparations and acknowledged discards additionally exercise all slots.
Every iteration ends inhibited, with zero live handles and native reservations.

The fixture is `config/machines/simulation/simulation-program.json`; the compiled machine and process
arguments, source commit, binary SHA-256 hashes, WSL kernel and CPU model/count
are recorded. Output and logs stay under `build/`; sockets use `TMPDIR`.
The driver leaves a record on failure, including any obtainable final status,
grant and jog observations. A workload error stops the run without retrying
motion or suppressing the failed iteration.

## Measurement sources

| Field | Source and meaning |
| --- | --- |
| `daemon.vm_rss_bytes`, `adapter.vm_rss_bytes`, HWM | Named VmRSS/VmHWM counters in `/proc/<pid>/status`, converted from kB to bytes. |
| `adapter.heap_inuse_bytes` | `/v1/status` `adapter.heap_inuse`, sampled by the adapter's `runtime.ReadMemStats`. Linux `/proc` does not identify Go-managed heap. The daemon's Go-heap field is not applicable; its memory measure is RSS. |
| `daemon_faults` | Minor/major faults from daemon `/proc/<pid>/stat`, fields 10 and 12. |
| `rt_thread` | Separate faults from `/proc/<pid>/task/<rt-tid>/stat`. |
| `native_metrics` | Daemon `metrics.json`: overrun count, maximum absolute wake jitter (the published lateness maximum), maximum cycle work and diagnostics drops. |
| `event_batches` | Actual `/v1/events` batches drained from cursor zero, preserving sequence, incarnation and explicit drop records. `events` separately retains sampled motion transitions. |
| `event_ring` | A separate cursor-zero probe supplies the oldest retained sequence and latest sequence. Occupancy is their inclusive span, including entries beyond the bounded response batch. Retention overwrite on this probe is distinct from loss on the advancing consumer cursor. |
| `prepared_handles`, `buffer_health`, `native_slots_in_use` | Status handle states and all six native slot states. Writing/prepared/ready/active reserve slots; free/retired are reusable, matching native admission. |
| `adapter_open_preparation_slots` | Public adapter status; both preparation lanes must be open at iteration boundaries. |
| `daemon_io` | `/proc/<pid>/io` read_bytes/write_bytes and rchar/wchar. Kernel-accounted disk bytes can remain zero for cached/tmpfs traffic; character counts record read/write syscall traffic. This is process I/O, not cell process-output qualification. |
| `jog_observations` | Unique native source receipts: first-observed host timestamp minus original input timestamp. Raw observations and overlap upload timestamps are retained. |

Process measurements are serialized, sampled periodically and at iteration
boundaries. Status/jog polls retain observed publication gaps and observer gaps.
Sampling cannot prove that every native publication or every input was observed.
Missing required fields remain null with an `unavailable` reason and fail
acceptance; absence is never interpreted as a measured zero.
Saved records are checked recursively for every required measurement member,
including zero-valued scalar counters in samples, iteration endpoints, warm-up
and final observations. Missing or null members are rejected by their JSON path.
Unavailable summary deltas print `unavailable (<reason>)`.

## Mechanical acceptance

`make soak-check` checks the committed record at
`docs/evidence/soak-simulation-2026-09-17/run.json.gz`. Override `SOAK_RECORD` to
check another JSON or gzip record. It starts no processes, recomputes the summary
and assertions from observations, rejects malformed/truncated data, and requires
exactly 50 complete measured iterations. It does not trust the saved `status`.

The finite-memory, deadline, event-loss and input-age contracts are described in
`docs/CONTRACTS.md`. That document supplies no numeric RSS-growth or Go-heap
reclamation allowance. The acceptance rules are:

- Daemon RSS growth after measured iteration 2 is at most one Linux page per
  subsequent iteration (the existing unverified driver policy).
- Adapter HeapInuse start, peak and end are measurements. For each measured
  iteration the checker also records the minimum of the released endpoint and
  subsequent empty-slot samples through the next upload start (through the final
  sample for the last iteration). This is the best available quiescent proxy.
  `--adapter-heap-allowance-bytes N` declares an absolute byte cap for every such
  minimum and the final snapshot. The allowance is stored in the configuration;
  the same flag can supply or override it when checking a record. Start and peak
  remain descriptive and never determine this cap. Declare and independently
  justify the allowance before collecting a qualification run.
- RT thread minor/major fault deltas and daemon overrun deltas are zero after
  warm-up. Host load is recorded and never exempts overruns.
- Maximum published absolute wake jitter is within the compiled cycle-lateness
  budget. Observed jog input age is within the ceiling published by Describe.
- The advancing event cursor is continuous, with no dropped/overflow records
  and a stable adapter incarnation. Retained occupancy is at most 256 events.
- Every iteration and final status have zero live handles/native reservations
  and two open adapter preparation lanes. All slot-state counts total three.
- Cumulative counters are monotonic across samples and iteration endpoints.
  Each endpoint, including warm-up baseline and final, must equal the counters
  at its sample timestamp or be bracketed by the nearest surrounding samples.
- Every measured upload overlap contains an accepted native jog observation;
  every measured trajectory reaches correlated completion and is discarded.

HeapInuse snapshots do not bound retained live memory: occupied spans can include
uncollected objects and fragmentation, and the public API does not confirm GC
completion. A proper retained-memory measurement needs a quiescent adapter sample
after a completed collection/sweep cycle, plus an independently justified byte
allowance. The adapter does not offer that measurement today. Even a passing
snapshot cap qualifies only this proxy, not retained-memory reclamation.

Without a declared allowance the checker prints
`adapter_heap: unqualified (no declared allowance)`; heap counts as neither pass
nor fail. Overall verdicts are `PASS` (exit 0: all bounds pass), `FAIL` (exit 1:
any failed/missing measurement, workload error, or incomplete qualification run),
and `UNQUALIFIED` (exit 3: no failures, but a bound has no allowance). Invalid
command-line flags use exit 2. FAIL takes precedence over UNQUALIFIED. The driver
uses the same verdicts for its requested iteration count; `--check` additionally
requires exactly 50 iterations. `go run` itself wraps nonzero program exits.

Focused tests include saved records with missing/null scalar and nested counters,
the iteration 25/26 regression, endpoint/sample disagreement, a leaked slot,
absolute heap-cap violations, event loss and unavailable deltas:

```sh
go test -race -count=1 ./tools/soak
```

The optional `-tags=soak` test remains a separate ten-iteration development run;
it is not the recorded-run checker and cannot qualify a 50-iteration record.
