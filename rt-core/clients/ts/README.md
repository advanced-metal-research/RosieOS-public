# Generated rt-core contracts

Run `make generate-protocol` and `make generate-api` from the rt-core root.
`make check-protocol` and `make check-api` compare every generated artifact,
including both TypeScript files; drift fails the checks.

The files have no imports or runtime dependencies. Compile them with your
application's TypeScript toolchain targeting ES2020 or later. No package install
is needed in rt-core. The protocol conformance test executes them with
`$HOME/.rosie/node-22.13.1/bin/node --experimental-strip-types` (or `NODE` when set) and fails
if that executable is absent.

`rt_control_api_generated.ts` declares public types, reasons, capabilities and
the schema capabilities digest. Its JSON number fields describe JSON values;
applications needing exact uint64 JSON observations must use a lossless parser.
Binary telemetry counters use `bigint`.

`rt_protocol_generated.ts` exports `decodeCycleCaptureAxisV2`,
`decodeCycleCaptureRecordV2`, `decodeTelemetryBatchHeaderV1`, their byte sizes,
and `CycleCaptureRecordV2LayoutDigest`. Each decoder takes an exact-size
`Uint8Array`; little-endian `DataView` reads preserve signed values and uint64
precision. Invalid lengths and reserved fields throw before returning a value.

For telemetry, accumulate 312 header bytes, validate magic `0x31425452`, version
1, header size 312, record size `CycleCaptureRecordV2Bytes`, the layout digest,
axis count 1..16 and record count at most 4096 before accumulating the body.
Decode each `CycleCaptureRecordV2Bytes`-byte record; verify its sequence and axis
count against the header. Preserve partial network reads: HTTP chunks need not
match application batches. Resume using
`LastSequence`, count `Dropped`, and reconcile adapter or daemon incarnation
changes. The complete transport contract is in `docs/APPLICATION_API.md`.

V2 adds signed `TorqueRaw` (0x6077 per-mille) and `TorqueValid`, coordinate
evidence/epochs, output velocity, watchdog counters/peaks/trip quantity, Home,
brake, mode/DS402/slave state and extended diagnostics. `ExtValidMask` bits 0..8
qualify position error, multi-turn low/high, bus voltage, load rate, IGBT/motor
temperatures, drive-not-ready and motor-not-rotating; `ExtValid` is any valid bit.
Check `PdoFresh` and each optional validity flag before using a value. The group
prefix adds motion sample, authority generations, fault masks, submission,
signed `CycleJitterNs`, and configured cell I/O words. All i64/u64 values are
`bigint`, including continuous coordinates. Original axis offsets are preserved.

The measured V2 sizes are 288 bytes/axis and 272 group bytes. The transitive digest
is `ec0762bded3f81ea3bef49917c29f441bd15b023795b316e3e111499926d4825`.
The header decoder throws `TelemetryLayoutMismatchError` with stored and expected
digests before accepting an incompatible body. V1 dumps require the matching old
`rtctl`; they are never interpreted as V2. The batch envelope remains version 1.
Full offsets, units and validity semantics are in
[the telemetry contract](../../docs/APPLICATION_API.md#telemetry-publication).
