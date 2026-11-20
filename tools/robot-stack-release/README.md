# Robot-stack bundle (offline only)

From the repository root:

Before offline packaging, fetch the independently published component tags into
the repository supplied as `--repo`. Their source commits need not be ancestors
of main; branch-only/CI clones can omit them even without shallow history.
This explicit network setup reads the trusted specification (requires `jq`):

```sh
jq -r '.components[].release_tag' releases/robot-stack/v0.1.0/release.json |
  while IFS= read -r tag; do
    git fetch origin "refs/tags/$tag:refs/tags/$tag" || exit
  done
```

Packaging still reads source by the specification's exact commit, never by a
moving tag. It performs no automatic fetch and rejects missing source before
creating output. The published component tags retain these otherwise-unmerged
pins. Existing complete bundles can be verified without any Git source objects.

```sh
uv run --no-project python -m unittest discover -s tools/robot-stack-release -v
uv run --no-project python tools/robot-stack-release/bundle.py package \
  --spec releases/robot-stack/v0.1.0/release.json --repo . \
  --inputs /PATH/TO/RETAINED/deploy-c08f8ae6 --bundle /NEW/robot-stack-v0.1.0
uv run --no-project python tools/robot-stack-release/bundle.py verify \
  --spec releases/robot-stack/v0.1.0/release.json --bundle /PATH/robot-stack-v0.1.0
uv run --no-project python tools/robot-stack-release/bundle.py archives \
  --spec releases/robot-stack/v0.1.0/release.json --bundle /PATH/robot-stack-v0.1.0 \
  --output /NEW/release-assets
```

The generated `COMPONENTS.md` contains the manifest selection as a readable table.
Archives include one full stack and one independently consumable asset per
component tag, each with checksums. Component archives intentionally contain only
their selected binary/provenance, with the complete manifest as composition
context; use their local checksum inventory rather than full-stack verification.
The release-scoped `AGENT-HANDOFF.md` is included under `release-inputs/`.

The trusted release specification selects independent component versions,
proposed release tags, exact mixed source pins and artifact digests. Packaging
does not publish those tags. The output contains artifacts, source-specific
dependency locks/Makefiles read with `git show PIN:PATH`, whitelisted producer
receipts, the retained qualification statement, `manifest.json` with every file
hash and `SHA256SUMS`. Existing output is never overwritten; interrupted output
is retained for inspection. No secrets or installed cell/Home data are selected.

Verification compares against the supplied trusted specification, rejects
tampering, unsafe paths, symlinks, incomplete inventory and extra files.
Checksums establish integrity, not authenticity: obtain the specification and
published checksum through the trusted release channel. Do not trust a modified
specification merely because its own manifest and checksums agree.

This bundle is not an installer or compatibility-range declaration. Consumers
select components explicitly; cell configuration and Home references remain
separately versioned/hashed inputs. Daemon/CD remains an external prerequisite.
Do not infer automatic upgrade, target-loader acceptance, new-cell qualification
or PR97 compatibility. Retained Go binaries identify Go1.26.5, not the declared
Go1.26.2 or Nix1.26.7. Missing producer logs are explicit gaps; Makefile snapshots
must not be described as evidence that a particular invocation actually ran.

## Retained release preparation

Supply the exact retained process-intent fixture as `RETAINED_PROCESS_INTENTS`;
the old repository fixture has retired. This is offline packet preparation.
Current deployment code refuses legacy runtime selections, so installation of
published v4 assets uses the retained release procedure, not this checkout's CLI.

`prepare.py` validates the whole bundle before creating a local packet. Supply
an owner-reviewed selection JSON keyed by component name; each entry has
`target_role` (the existing manifest node) and `path` (a new versioned absolute
slot ending in the component name). Select only components being upgraded.

```sh
uv run --no-project python tools/robot-stack-release/prepare.py \
  --spec releases/robot-stack/v0.1.0/release.json --bundle "$STACK" \
  --manifest "$CELL_MANIFEST" --fixture "$RETAINED_PROCESS_INTENTS" \
  --selection "$SELECTION" --cell "$CELL_CONFIG" --home "$HOME_REFERENCE" \
  --output "$PREPARATION"
```

The private packet contains verified binaries, a derived manifest changing only
selected `runtime_paths`, a filtered exact-hash fixture, byte-identical previous
manifest/fixture, external reference hashes (not copied cell/Home contents), and
canonical CLI argument arrays in `plan.json`. The CLI resolves paths from the
manifest, not merely the fixture. Existing args/env and manifest configuration
are preserved in the packet; the canonical CLI may subsequently normalize them,
so review its resolved dry-run before publishing. Never treat preparation as
target staging, loader acceptance, activation or Arm.

Repeating the identical preparation reuses verified files, restores missing
files and restores executable/private modes after an interrupted chmod. A
conflicting or partial file rejects before any further output changes;
retain that attempt and choose a new directory. No subprocess/network operation
runs during preparation. The owner still stages named peer/slot binaries and
uses existing authenticated daemon publication, then checks loaded executable
hashes and references. Automatic transfer/apply/rollback integration remains
not_evaluated; the generated previous inputs are a recovery source, not a
verified runtime rollback receipt.
