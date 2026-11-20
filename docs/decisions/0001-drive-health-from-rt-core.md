# 0001: Drive health comes from rt-core and shows on an OLP page

Status: accepted 2026-10-22. Deploying it needs a full
rt-core release (owner approval).

## Goal

The operator sees, in the browser, each drive's heatsink temperature and each
joint's recent torque and following error, on any cell, without a process of
its own on the servo host.

## Problem

One cell has this today only through `amr-servo-temperature-feed` on its
Jetson, installed by hand (recorded in `servo-dashboard/v1` on branch
`sl/chore/servo-dashboard-import`). It works, but:

- Every 5 s it logs in to the servo host over ssh, starts Python under sudo and
  runs 12 `ethercat upload` processes beside the running master.
- For torque and following error it decodes rt-core's 1 kHz telemetry ring in
  Python on the servo host, for up to 30 minutes per page view. The owner rule
  says never decode telemetry there (a decode on a servo host stalled
  EtherCAT for about 1 s and invalidated Home).
- It hard-codes the ring's byte offsets, the protocol digest and that cell's
  configuration hash, so any rt-core or configuration change stops it until
  someone edits files on the host.
- No repo command installs, checks or rolls it back, and it serves one cell.

## Solution

rt-core reads and summarises; the OLP serves; a browser page shows.

1. **Drive profile `monitor` list.** A drive profile names objects to read
   while running, for example in `config/drives/a6ec.json`:

   ```json
   "monitor": [{"key": "heatsink_temperature", "label": "Heatsink temperature",
                "object": "U40.30 / 0x2040:31", "index": 8256, "subindex": 49,
                "type": "i16", "decimals": 1, "unit": "degC", "interval_ms": 5000}]
   ```

   The compiler renders it per axis (`monitor=key|type|index|sub|decimals|unit|interval_ms`),
   and the daemon's loader and configuration identity check it like every
   other axis key. Nothing about the A6-EC is in rt-core's code.
2. **The commissioning worker reads it.** When idle, beside the drive alarm
   read (0x203F), it reads at most one due monitor object per pass, through
   `read_sdo_scalar_axis` (the shared mailbox lock and admission checks). The
   value is `raw / 10^decimals`. A failed read keeps the last value with its
   age; a client sees the age grow.
3. **The metrics thread summarises the telemetry ring.** Each 100 ms pass
   reads the records written since its last pass (lock-free, as the stream
   API does) and keeps ten 100 ms bins per axis: torque sum of squares and
   peak, following-error peak, sample count. Published: the last 1 s.
   Nothing is added to the real-time cycle.
4. **Status carries both.** Per axis in `metrics.json` and so in
   `ipcclient.AxisStatus`: `monitor` (key, value, unit, age) and `load`
   (window, samples, torque RMS and peak in per-mille of rated torque,
   following-error peak in counts). The application schema changes, so the
   capabilities digest changes. The OLP server ships with rt-core in the
   release; the browser UI runs from the operator's checkout of the same
   commit; and the Deck pendant's bundled `olp-core.js` refuses any other
   digest, so every Deck on that cell needs `pendant build` and
   `pendant install` (separate approval) or it refuses the cell.
5. **OLP route and page.** `GET /api/offline-programming/v1/dense-execution/drive-health`
   reads the connected cell's status and description and answers per joint
   in display units (°C, % of rated torque, degrees). `ui/drives.html`, beside
   `cameras.html`, polls it once a second and keeps its own short history.
   Browser only; the Deck does not show it.
6. **The hand-installed unit retires** once this page runs on that cell. Not
   built yet: it needs a repo command step (the camera install's `REPLACES`
   pattern), not a hand removal.

## Why

- **Data stays where it is cheap.** rt-core already holds torque and
  following error every cycle and already reads drive objects over the
  mailbox off the real-time thread. A summary of a few hundred bytes a second
  replaces either a decode on the servo host or the raw ring over the link
  (about 4.9 MB/s, `internal/denseexec/rt_core_telemetry.go`).
- **One mailbox owner.** rt-core serialises every SDO through one lock and
  its admission checks; a second process on the master is unmeasured risk.
- **No new service.** The OLP already has the authenticated connection, the
  deploy command, rollback and the cell selection. A separate service would
  need its own client certificate, CRL entry, unit, port and tunnel.
- **Generic.** The profile names the objects, so another drive or another
  reading (motor temperature, DC bus voltage) is a profile edit.

## Not decided here

- Temperature warning and trip values: the vendor's figures are not
  confirmed (the hand-made page left them blank too). The page shows values
  and the drive's own alarms, no thresholds.
- `interval_ms` 5000 matches the hand-made page and is unverified for
  mailbox load; measure on a bench before raising the rate.
- A6-EC fixed PDO sets (0x1B02/0x1B04) carry no temperature
  (`include/axis_profiles.hpp` `validate_a6_fixed_preset`), so it stays a
  mailbox read.
