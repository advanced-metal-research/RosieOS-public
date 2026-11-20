# Historical records

Consolidated at source `45cb6d313c3d641db8964476f867356a8660fc62` for P8-12;
the "Superseded claims" table below was re-read against `403063dc`.
The thirteen `*.json` evidence records indexed below were untracked on
2026-09-17 by owner decision (bloat review item 5) and remain readable at tag
`archive/standalone-rt-core-2026-09-17`; they are cited by name, not linked.

Use [ARCHITECTURE.md](../ARCHITECTURE.md) for current implementation,
[APPLICATION_API.md](../APPLICATION_API.md) for the implemented public contract,
and [IMPLEMENTATION_PLAN.md](../IMPLEMENTATION_PLAN.md) for the backlog.
The tracker and CI command records supersede `VALIDATION.md`
as the current acceptance record. Moving a record does not rerun its checks or
advance its review stage.

## File index

| File | Review/checkpoint and what it recorded |
|---|---|
| [ARCHITECTURE_REVIEW.md](ARCHITECTURE_REVIEW.md) | Pre-REV-01 review of `f68e871`, 2026-09-11: simulation findings and architectural risks; its stop, plan and monolithic-runtime descriptions predate P2/P4 corrections. |
| [CONTROL_SURFACE.md](CONTROL_SURFACE.md) | P0 target-contract checkpoint, revised 2026-09-12: desired operations, lanes and ownership; implementation is now specified by the two schemas and current API, not every target promise. |
| [P2_CORRECTIONS.md](P2_CORRECTIONS.md) | M1/P2 checkpoint `b615edc`: readiness, submission, cancellation, endpoint and configuration corrections; REV-01 subsequently reopened wrapped-coordinate recovery. |
| VALIDATION.md | Accumulated P2/P3 and REV-01 to REV-04 validation checkpoints: command results and limitations; open-worker/API wording predates later implementation. |
| [BLOAT-REVIEW-2026-09-17.md](BLOAT-REVIEW-2026-09-17.md) | Bloat review of the branch against PR 108 (2026-09-17): what was removed, untracked or compacted, the owner decisions, and the robot/v4 follow-on cards. |
| `m1-evidence-2026-09-12.json` | M1 `b615edc`: source hashes and canonical correction validation; later REV-01 disputes remain in the review record. |
| `p2-evidence-2026-09-12.json` | P2 baseline `895e249`: simulator injection provenance and failing-baseline/passing-candidate observations. |
| `p3-evidence-2026-09-12.json` | P3 foundation `1c77968`: Home cancellation, atomic publication and jog/clock foundations; not the later full ingress/worker implementation. |
| `p3-grant-evidence-2026-09-12.json` | P3-09 `214770e`, reviewed by REV-02: native mirrored application grant and mapping retirement; then-current jog/worker gaps are historical. |
| `rev01-followup-evidence-2026-09-12.json` | REV-01/REV-02 follow-up `1c79f6f`, accepted in part by REV-03: reproducible oracle replay, idle recovery and build safeguards. |
| `review-evidence-2026-09-11.json` | Pre-REV-01 `f68e871` simulation: Stop displacement, initial-point speed and deadline observations; no hardware evidence. |
| `review-evidence-895e249.json` | Pre-REV-01 `895e249` targeted simulation checkpoint: delayed readiness, failed sends, Abort, limits and bulk-delayed jog. |
| `review-response-2026-09-12.json` | REV-01 response through `fa26e36`: reopened wrapped-feedback gap, historical hashes and remaining validation work. |
| `review-response-rev02-2026-09-12.json` | REV-02 response for `214770e` and follow-ups: assertion guards, grant teardown and protocol-policy work. |
| `review-response-rev03-2026-09-12.json` | REV-03 response to source `1c79f6f`/docs `6b24185`: accepted P2-13 and retained CI, replay and public-reset follow-ups. |
| `review-response-rev04-2026-09-12.json` | REV-04 response to `6b24185..dd15e34`: accepted grant/freshness fixes, retained source qualification and required composition extraction. |
| `review-runtime-evidence-2026-09-12.json` | REV-01/REV-02 runtime correction `ccbde01`, reviewed by REV-04: current-domain evidence, terminal connection cleanup and one fence per event. |

REV-01 through REV-06 review blocks remain in tracker.html;
REV-05/REV-06 have no separate response JSON in this snapshot. The REV-07 gate
checkpoint is recorded in P8-01 evidence there, without a separate response file.
The existing evidence directory retains pinned raw logs, copied
sources and replay records in their original locations. Historical JSON source
hash keys and command strings identify the original snapshots; they are not a
claim that those old source paths or hashes describe today's tree. At consolidation
the cross-reference paths inside ten archived files (ARCHITECTURE_REVIEW.md,
CONTROL_SURFACE.md, P2_CORRECTIONS.md, VALIDATION.md and six JSON records:
m1-evidence, p3-grant-evidence, review-response-2026-09-12, review-response-rev02,
review-response-rev04, review-runtime-evidence) were rewritten to their `history/`
locations; no other content in any archived file changed, and the pinned copied-source
manifests under `../evidence/` were not edited.

## Superseded claims and current code

| Older claim or ambiguity | Current code and reading rule |
|---|---|
| KICKOFF/CONTROL_SURFACE name three schemas and planned public/jog generators. | Two schemas exist. `tools/protocolgen` owns native IPC and the jog frame in `protocol/control.json`; `tools/apigen` owns `protocol/application-v1.schema.json`. See [current schemas](../ARCHITECTURE.md#schemas-clients-and-generators). |
| Old APPLICATION_API, KICKOFF and VALIDATION say jog, Home, bulk and publisher separation remain unfinished. | Main composition starts separate helper, commissioning, bulk and publisher workers; capture drain is part of the publisher. [src/main.cpp:47](../../src/main.cpp#L47), [src/main.cpp:138](../../src/main.cpp#L138), [engine/status_publisher.hpp:590](../../engine/status_publisher.hpp#L590). |
| Old APPLICATION_API says EndJog always inhibits and qualified ramps are future work. | The EndJog/expiry/halt ramp is configured on live and simulation builds from each axis's jog acceleration; a failed bound inhibits. Public `halt` is implemented (P4-05 verified); powered stopping measurement follows BENCH_TESTING. [engine/jog_lane.hpp:240](../../engine/jog_lane.hpp#L240), [adapters/rosie/control/halt_linux.go:25](../../adapters/rosie/control/halt_linux.go#L25), [current API](../APPLICATION_API.md#jog-lanes-and-remote-listener). |
| Old APPLICATION_API says ResetFault clears all Home and reason-specific recovery is missing. | Native per-bit affected-axis/trust policy governs recovery, and the public API reports correlated outcomes. The bus-transport recovery table explicitly narrows KICKOFF R21 blanket invalidation for trusted disabled recovery. [include/fault_recovery.hpp:46](../../include/fault_recovery.hpp#L46), [adapters/rosie/control/reset_linux.go:20](../../adapters/rosie/control/reset_linux.go#L20). |
| Old APPLICATION_API/HANDOFF treat logical status/events and C++ application clients as missing. | Status/event routes, a C++ client and generated TypeScript contracts exist; P5-04 is verified in the tracker. [adapters/rosie/control/http.go:57](../../adapters/rosie/control/http.go#L57), [clients/cpp/include/rosie/rt_control_client.hpp:396](../../clients/cpp/include/rosie/rt_control_client.hpp#L396). |
| Grant checkpoint text says daemon/adapter must share a UID. | Peer admission allows the explicitly configured adapter UID and requires matching HELLO/kernel PID; installed services use separate users. [include/peer_admission.hpp:19](../../include/peer_admission.hpp#L19), [host/rosie-rt-control.service:12](../../host/rosie-rt-control.service#L12). |
| Earlier reviews estimate four plan buffers; plan P4-07 title asks for one active plus one staged. | Code retains three prefaulted resident slots to preserve oracle behavior and decodes into Writing storage. Its declared LP64 plan layout is 210,000,168 bytes (280-byte points), excluding process overhead. [engine/trajectory_store.hpp:50](../../engine/trajectory_store.hpp#L50), [engine/trajectory_store.hpp:216](../../engine/trajectory_store.hpp#L216). |
| BENCH_TESTING/MIGRATION describe Go tools, install permissions and restart behavior as unavailable. | Go tooling, service identity provisioning, private/public socket paths and restart fencing exist; physical qualification and real consumer migration remain separate. [tools/rtctl/command.go:1](../../tools/rtctl/command.go#L1), [host/install.sh:140](../../host/install.sh#L140), [adapters/rosie/control/controller.go:187](../../adapters/rosie/control/controller.go#L187). |
| Older API/schema wording implies all identity fields are equally verified or compared. | Native reconstructs machine semantics; externally supplied deployment digest gets syntax validation. Acquire selects machine digest when supplied, otherwise configuration digest. [include/daemon_options.hpp:1847](../../include/daemon_options.hpp#L1847), [adapters/rosie/control/controller.go:499](../../adapters/rosie/control/controller.go#L499). |
| Current schema names RecoveryStatus for restore_anchor and generally specifies HTTP 409 errors. | Both discrepancies were closed under P5-01 at `15f5b353`: restore_anchor returns RecoveryStatus with the receipt in `Response.sequence`, and event cursor errors return HTTP 409. See [the implemented API](../APPLICATION_API.md#transport-and-response-semantics). |

The six retained supporting documents have a status header; their bodies preserve
decision, contract, procedure or lineage context. The previous APPLICATION_API
and README narratives remain available in Git at the source commit named above.
No archived claim overrides current code or closes an outstanding release gate.
