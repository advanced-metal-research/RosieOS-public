# Motion control surface

Target contract, revised 2026-09-12. This describes the accepted architecture,
not an assertion that it is implemented. [APPLICATION_API.md](../APPLICATION_API.md)
records the current implemented API; [IMPLEMENTATION_PLAN.md](../IMPLEMENTATION_PLAN.md)
owns the complete backlog, review dispositions and release gates.

The [kickoff decisions](../KICKOFF.md) name the separate Go adapter and C++ daemon,
their exact target sockets and sole application grant owner. The C++ motion-server
client uses the public Go adapter, not private native IPC. Remote jog uses a
dedicated authenticated WSS connection; local jog uses a Unix datagram socket;
only the adapter publishes the native latest-value slot. Clock uncertainty and
stale-input rejection are specified there. All C++/Go wire codecs are generated
incrementally from one canonical schema per wire format, without a second
hand-packed implementation. KICKOFF names the three formats and generators.

## Ownership and language boundary

C++ owns the cyclic machine state, execution clock, coordinate transforms,
CiA402 transitions and final output permit. Go owns off-cycle configuration,
application authority, bulk ingestion, clients, tooling and integration tests.
Provide a C++ client for the existing C++ motion server. The completed component
and its new control path contain no Python; no consumer shells out per command.

The motion server owns planning, program identity/storage and application
sequencing. A thin Rosie adapter binds authenticated remote controller/pair
identity to one local expiring grant. Cell I/O retains its existing owner.
Normal clients use logical axes and units, never PDO counts or vendor SDOs.
Commissioning is a privileged exclusive lifecycle, outside cyclic execution.

## Independent traffic paths

| Lane | Contract from producer through adapter/client/IPC to native consumer |
|---|---|
| Latest-value jog | Dedicated bounded slot/stream. New input supersedes old input; preserve source age, sequence and expiry. RT reads the slot directly. No bulk/helper queue or whole-operation lock in this path. |
| Bulk preparation | Reliable bounded transfer, complete validation and immutable ownership handoff outside RT. Inert until explicit Start. Backpressure stays on this lane. |
| Reliable control | Bounded commands and correlated results for mode/lifecycle operations. Authority renewal has independent progress; slow commissioning runs in a worker, not the command dispatcher. |
| Status/events | Coherent, timestamped snapshots and bounded ordered events with sequence/overflow semantics. Slow readers cannot delay the cycle or command admission. |

Stop and authority revocation additionally have an urgent path independent of
all bulk/commissioning work. Separate queues alone are insufficient if they
share a blocked adapter mutex, connection writer, worker or unbounded allocation.
Admission and resource budgets apply across simultaneous traffic.

All paths converge at **one RT mode owner and one final output permit**. For the
same controlled axis group, a plan executing rejects BeginJog; active jog rejects
StartTrajectory. This is a native state decision, not an adapter string check.
Inert preparation may coexist with jog. New jog updates supersede only other
updates in that jog session; they never implicitly take over a running plan.

## Target operations

| Operation | Meaning and required observation |
|---|---|
| Describe | Version, capabilities, axis IDs/order/units, effective limits, capacities and verified configuration identity. |
| Acquire / Renew / Release | Authorized expiring grant and generation; release revokes all associated motion and preparations. |
| Enable / Arm | Request drive/output preparation and new-motion permission; observe native group readiness before motion. Receipt alone is insufficient. |
| PrepareTrajectory / PrepareProgram | Validate inert complete data; return immutable handle/digest/identities/resource reservation or structured rejection. |
| Discard | Invalidate an inert prepared handle and release its resources. |
| StartTrajectory / StartProgram | Activate a matching valid handle after authority, home/configuration, continuity and fresh group-readiness checks. Then observe execution ID/state. |
| Abort | Cancel identified execution or preparation with explicit scope; an acknowledged cancelled handle cannot later Start. |
| BeginJog / UpdateJog / EndJog | Explicit jog session, latest-value velocities and observed applied sequence. Define and qualify the end/expiry stop policy. |
| Stop | Urgently cancel active/pending motion, fence all lanes and inhibit output. New motion needs fresh Enable/Arm. |
| Halt | Controlled deceleration/hold only when implemented and qualified; a distinct capability from Stop. |
| Home | Exclusive declared procedure with completed/failed/cancelled result and a new home identity on success. |
| ResetFault | Request reason-specific recovery with output inhibited; publish remaining home/readiness requirements. Never resume automatically. |
| ReadStatus / SubscribeEvents | Coherent state and bounded ordered events, with time, validity and configuration/home/session/execution generations. |

Specify per-operation idempotence, timeout/unknown-outcome recovery, stale identity
rejection and handle retention/consumption. A lost Start reply must not cause
motion to be submitted twice. Authorized Stop remains possible after motion
permission expires, without accepting unauthenticated remote requests.

Distinguish received, accepted, executing, completed, cancelled, faulted and
unknown delivery. Successful Stop submission requires a checked successful host
API submission of inhibited output for every configured domain. IgH can still
drop queued datagrams on link failure; this return does not establish wire
transmission, drive receipt, physical standstill, brake engagement or STO.
Bus API failures and subsequent input-health failures must remain observable.
A completed trajectory also needs the defined endpoint/feedback settling policy.

## Identity, clocks and observation

The schema defines protocol version, request/session/jog IDs, command sequences,
configuration/home identity, daemon incarnation and execution generation wherever
applicable. Configuration identity is derived from the effective canonical
artifact, not a supplied hash label. Separate deployment identity from machine
semantics. Stop/revocation/restart fences every old writer and prepared handle.

Jog includes source time and a short immutable expiry. Same-host monotonic clocks
can share a domain; remote producers require a defined conversion/uncertainty
policy and sequence/session checks. Establish a conservative local deadline at
authenticated ingress; reject stale, implausibly future or unresolvable input.
Publication, RT dispatch and control-lease renewal never extend that deadline.
Define independent budgets for authority, input freshness and feedback freshness.

Status includes measured and commanded logical position/velocity, explicit field
validity, observed mode/readiness, home/absolute validity, execution cursor/time,
buffer health, effective limits and typed faults. A coherent snapshot identifies
its sample time and generations. Raw diagnostic counts remain supplemental.
Events identify drops/overflow and how consumers resynchronize; neither missing
feedback nor absent optional PDO fields may be represented as trustworthy zeros.

## Trajectories, storage and motion policy

Samples use segment-local `time_from_start_ns`, ordered configured axis IDs,
positions in radians/metres and optional velocity feedforward in rad/s or m/s.
Position determines the path; feedforward does not bypass derivative limits.
The first sample and explicit stationary segment boundaries must match the held
command under the declared continuity rule. RT owns the execution clock and
starts it only when the group is actually ready.

Keep linear position interpolation as an explicit initial capability. Declare
endpoint behavior, feedforward mapping and supported derivative/acceleration/jerk
checks. Later interpolation changes need version/capability semantics and extrema
validation. Invalid timing is rejected or replanned upstream; never silently
retime axes independently while process events continue on the original clock.

Use bounded preallocated execution storage with immutable ownership transfer.
Retain the 250,000-point capability, preferably one active plus one staged plan,
and measure native plus Go/body/encoding memory. Extra resident buffers need a
reason. Chunked upload is a bulk transport feature; paged execution is deferred
unless required and its underrun behavior is qualified. Short moves use the same
Prepare/Start lifecycle as long programs.

Jog uses the same coordinate model and final output checks, with configured
acceleration, target lead and following-error policy. Target lead bounds integrated
command position; it is not a velocity law defined by a fixed distance ahead of
feedback. Decide per-condition rejection, clamping, controlled halt or fault
according to the filled R21 table in KICKOFF and expose the reason to operators.
Valid velocity-cap overshoot is limited, with feedback, rather than causing a
disarm solely for slider overshoot. The limit governor/controlled halt remains
unimplemented and must be qualified before replacing existing inhibition.
Use time durations, not fixed
cycle counts. Do not expose Pause as a renamed Disarm; continuation needs its
own qualified semantics and newly validated path.

## Current motion-server and process-I/O handoff

The existing [dense trajectory format](../../../motion-server/v2/docs/DENSE-JOINT-TRAJECTORY-V1.md)
uses 100 Hz timed q/qd samples, nine named rotary axes, segment-local clocks,
digests and per-sample torch flags. Preserve the 250,000-sample ceiling and explicit
mapping; the generic RT surface also supports configured linear axes. The legacy
`robot.v4` schema identifier does not imply a source dependency on v4.

Resolve the format's 1e-3 boundary tolerance versus the adapter's 1e-12 continuity
rule using real planner outputs. Prefer canonical stationary shared endpoints
from the producer. Any normalization changes samples and requires revalidation
and new digests; otherwise reject with an actionable reason and replan. Do not
silently connect a gap. Preserve dwell and process-marker timing.

Replace the actual C++ motion server's position-to-velocity follower with native
prepared execution. Its UDP `CmdJointSetpointV2` stream cannot be redirected to
GIPC to achieve this. A SDK/CLI example is not deployed consumer integration.

Associate generic process markers with execution ID, segment/time and action ID.
The cell I/O owner follows observed native execution, invalidates pending ON after
Stop/revoke, enforces independent expiry and reports real output readback.
Publication or local torch intent is not output acknowledgement. Cycle-synchronous
I/O requires a capable backend and measured timing; ordinary NATS delivery alone
does not establish it. Torch-bearing execution stays refused until qualified.

## Current implementation gaps

| Area through native protocol 1.5 | Current state and remaining work |
|---|---|
| Native q/qd execution, Prepare/Start, mode conflicts | Fresh Operation Enabled/CSP group gate, bounded endpoint settling and prepared Abort/Discard/replacement lifecycle implemented and tested. Physical tracking qualification remains. |
| Jog | Acceleration/lead policies exist; adapter/client/helper still serialize it behind bulk and refresh freshness late. No complete Begin/Update/End lifecycle. |
| Urgent Stop/leases | Checked inhibited group submission is required for acknowledgement; failed/unattempted sends cannot acknowledge. The sole adapter grant is mirrored into direct native deadline/generation enforcement, with separate generated requests/receipts. Stop publication fences pending grants and does not wait behind renewal receipts. Remote identity and host qualification remain. |
| Ownership/profiles/coordinates | Pure Start/output policies and checked cyclic bus boundary; strict per-axis descriptors with dead global defaults removed. Complete cyclic machine state and qualified wrap/restart reconciliation remain. |
| Protocol/clients | GIPC 1.5; generated Start/Discard handles plus grant/header/observation records and operation/reason constants. Normal role 3 requires application authority; role 1 is simulation-only compatibility. Complete C++/Go application surface and Python removal remain. |
| Current review corrections | Terminal observation failures close one connection with one fence and safe mapping retirement; expiry and each received native RTDISARM/RTSTOP02 packet fence once. Public Stop still emits multiple native packets through transitional writer barriers. Fast-region geometry/shared lease cap are generated. Current PDO freshness consumes each domain exchange once; fixed-period execution exchanges the full group. Independent review and machine qualification remain required. |
| Status/recovery | Raw counts and separate motion snapshot; full logical coherent status, ordered events and public ResetFault remain. |
| `.rdt` | Local decode/prepare/execute works for supported stationary boundaries; compatibility policy and real motion-server integration remain. |
| Resources | Three resident 250,000-point slots plus staging; replacement reclamation implemented, broader memory/admission budgeting remains. |
| Deployment and I/O | Active runtime unchanged. Authenticated remote handoff, service recovery/access, qualified process I/O and physical tests remain. |

REV-01 reopens P2-05 for wrapped-feedback re-arm; the other review follow-ups and
all release gates remain open. See [the response](review-response-2026-09-12.json).

Current fault policy is conservative even while idle: transport errors, and loss
of valid WKC after a healthy bus, can latch FAULTED and invalidate verification
and home evidence. Explicit reset and fresh verification are required; re-home
is mandatory where configured. Initial startup incomplete WKC is distinct.

COMPLETED requires a whole-group fresh, enabled CSP/error-free observation.
Each axis must settle by its endpoint-relative deadline; before that deadline
it may leave and regain tolerance, but afterward it must stay settled until the
group finishes. There is no dwell or hysteresis. Group COMPLETED is terminal.
completion_timeout_ms is a required profile key whose bench values need machine
qualification. Nonzero mapped error_code remains an admission/execution blocker
without a qualified severity mapping; a warning status bit alone is different.
Follow P0-P8 and G0-G2 in the implementation plan for completion.
