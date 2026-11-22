# Cell configs

A cell config binds a named installation to its hosts, runtime, machine
configuration, pair identity and application listener. This directory is the
single repository source for cell configs and the Steam Deck fleet inventory.
Paths supplied to consumers are repository-relative: `rt-core/config/cells/<name>.json`.
The former copies have been removed; there are no compatibility links.

The offline-programming target resolver reads these bindings to compile the
selected configuration and validate the execution identity. The Deck fleet
tooling selects cells and the wake inventory here. The fleet daemon consumes
cell-config-derived discovery and deployment records. The robot and Deck deploy
CLIs, including their CI qualification scripts, read the same files when building,
staging and installing a cell. NATS subjects and discovery KV keys retain their
existing wire names.

Each native node carries an `rt_core` block:

| Field | Contract |
|---|---|
| `machine_config` | Existing repository-relative machine JSON compiled with the current `rt-core/config/drives` and compiler. |
| `configuration_sha256` | Exact compiled configuration digest; every native cell config here now pins it. |
| `pair_id`, `pair_revision` | Nonempty token and positive revision identifying the execution pair. |
| `remote_listen` | Host and numeric port for the authenticated application listener; any tunnel or provisioning requirement is described by `remote_listen_note`. |
| `package_artifact` | Standalone runtime package selected for deployment. |
| `ethercat_device_group` | Host device-access group for the native runtime. |
| `activate` | Whether deployment requests activation; a digest pin does not activate anything. |
| `remote_pki` | Optional. `installed`: the host's listener PKI and `ROSIE_RT_REMOTE_ARGS` are already provisioned, and deploy keeps them verbatim (`generate-control-env.sh --keep-remote-args`); `remote_listen` then names the client endpoint. Absent: deploy publishes the listener disabled. |
| `nats` | Optional observation-publisher configuration; it grants no execution authority. |

The dense axis map is not a cell config field. The adapter maps the dense
wire's columns to the described rotary axes of the compiled configuration, in
order, so the robot definition and the machine config are the only sources of
the axis count (a bench maps nine columns as a Rosie 1400 and six as a
Rosie 1420, and a one-motor bench one). Configs written before 2026-09-20
carried a `dense_axis_ids` list that had to restate that count; the reader
ignores it now and the compatibility check no longer asks for it.
## Current compatibility

The Go validator compiles every `rt_core` block, checks the pin, pair, listener and
dense-axis mapping, and returns an error for compiler warnings. Both `make test-go`
and the offline-programming targets tests exercise it. The tests record each
config's exact existing watchdog findings and reject changed or additional findings. A
passing regression test does **not** mean all these configs compile without
warnings. Watchdog thresholds require separate qualification; this move does not
invent thresholds, disable diagnostics or alter machine configuration.

`linear-rail-1axis.json` (machine `config/machines/linear-rail-1axis.json`,
one axis), `example-arm-1420.json` (machine
`config/machines/example-arm-1420.json`, a Rosie 1420 V1 arm on six axes) and
`example-arm-1430.json` (machine `config/machines/example-arm-1430.json`, a
Rosie 1430 V1 arm on six axes with the values the 1430 arms in service run,
less their per-joint drive tuning) are the patterns to copy for a new cell.
All three use documentation addresses (192.0.2.x), the login `changeme` and
`activate: false`, so none can reach or drive real hardware; tests use them where they need a cell;
`config/templates/cell.json` is the template. The configs of real cells and
benches are site configuration: each one's pin, axis count and recorded
watchdog findings live with it, and the compatibility test checks every cell
config present. Where several configs address the same servo host, only one
may be active (`activate`), because only one configuration can own the bus.
`aliases/` holds deploy-only manifests for a host that already has a cell (a
build or tour alias of a bench): the deploy commands take them by path, the
compatibility check reads them, and the offline programming app never offers
them as cells, because it reads only this directory's own `<cell>.json` files.

Pinning the current compiled identity
prevents future compiler, drive or machine changes from silently selecting different
execution bindings. Deployment/readback qualification is separate from this check.

A configuration digest change requires reviewing the intended machine, re-pinning
the affected configs here, and re-homing the cell against the new identity.
Do not silently re-pin a mismatch or reuse Home evidence from another digest.
