# Weld Preflight package boundary

RosieOS produces planner-native candidate bytes. It does not import or vendor
Weld Preflight. The adapter invokes an isolated, versioned `weld-preflight`
tool only after an exact package and schema capability handshake.

`weld-preflight.lock.json` pins:

- distribution and exact semantic version;
- isolated Python runtime;
- wheel filename, GitHub Release URL, and SHA-256;
- capability schema and required candidate-ingest schema;
- `execution_authority: none`.

Install after the corresponding reviewed GitHub Release exists:

```sh
python tools/install_weld_preflight.py
```

The default download uses the authenticated GitHub CLI. Current release assets
live in the same private RosieOS repository, so CI uses its read-only repository
token; the token is not stored in this lock or passed to the installed tool.

For a release-candidate integration check, supply the exact local wheel:

```sh
python tools/install_weld_preflight.py \
  --wheel /path/to/amr_motion_bench-0.3.1-py3-none-any.whl \
  --target .run/weld-preflight
```

The tool runs under Python 3.12 in `.run/weld-preflight`; it is not installed
into the planner's Python 3.10/3.11 CUDA environment. Before every ingest,
RosieOS requires `capabilities` to match the exact version, accepted candidate
schema, proposal-only candidate authority, and absent execution authority.

Missing package, hash mismatch, unsupported schema, version drift, probe
timeout, or any execution authority is `capability_unavailable`. It does not
change the planner result and cannot authorize Motion Server or robot motion.

RosieOS pull-request CI validates the lock and handshake under the planner's
Python 3.11 boundary, then downloads the exact private RosieOS release wheel
through the same validated URL path as production, verifies its lock hash,
provisions isolated Python 3.12, and runs a real candidate ingest. Fork pull
requests do not receive or run the private-release job. A missing credential is
`capability_unavailable`, not permission to use an unpinned checkout.
