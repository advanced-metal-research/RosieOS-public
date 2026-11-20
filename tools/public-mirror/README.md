# Public mirror

RosieOS is published as a separate public repository,
[`advanced-metal-research/RosieOS-public`](https://github.com/advanced-metal-research/RosieOS-public),
built from this one, not by making this repository public. That keeps this
repository's history (deleted files, author emails, commit trailers), its PRs,
issues, Actions logs and release tarballs private, and lets working records
stay here.

- `private-paths.txt`: paths that stay private (the team devlog and agent
  scratchpads, the rt-core tracker and raw bench records, a skill whose license
  forbids redistribution). Add a path here rather than deleting a record the
  team still uses.
- `export.mjs <empty-dir> [--rev <commit>] [--allowlisted]`: writes one commit's
  files minus the private paths, then runs the leak check over the result.
  Strict by default: any hit fails.
- [`quality/leak-check.mjs`](../../quality/leak-check.mjs): tailnet names,
  Tailscale addresses, MAC addresses, personal and company emails, home
  directories, Cloudflare IDs, credential formats, logins recorded as
  `(<login>@<machine>)`, and the site's cell, host and robot names (learned
  from the private manifests and the private `site-names.txt`, for fleet
  machines and logins no manifest names, in any spelling: `host-2`,
  `Host 2`, `host2`). It refuses a Markdown link from a published doc to a
  private path, this private repository or the team's Linear (a dead link in
  the mirror: keep the reference as plain text or move the passage to
  `docs/site/`). It opens ZIP and gzip files (a `.weldplan` is a ZIP) and
  checks each member as a file. It refuses CAD source files (STEP, IGES and
  the like), a mesh inside an archive that is not byte-identical to a public
  mesh, and any public copy of a private mesh or CAD file; the bundled Dog
  Box STEP and the shared pedestal mesh are the reviewed exceptions. CI (`leak-check.yml`)
  runs it on every PR against `quality/leak-allowlist.txt`, today's known hits
  as (pattern, path) pairs; a new pair fails. The list is empty: keep it so.
- `gitleaks.toml`: the secret scanner's default rules minus reviewed false
  positives (placeholder test tokens, content hashes, credential reference
  names). CI scans the exported tree with it on every PR.
- `publish.mjs [--rev <commit>] [--push] [--author "Name <email>"] [--message <text>]`:
  exports a commit on `origin/main`, scans it with gitleaks, and commits the
  tree on top of the mirror's previous commit under the publisher's GitHub
  noreply identity. Without `--push` it only reports what it would publish.

The mirror's history is one commit per publish; nothing from this
repository's history reaches it. Publish from a merged commit:

```sh
git fetch origin
node tools/public-mirror/publish.mjs --rev origin/main          # report only
node tools/public-mirror/publish.mjs --rev origin/main --push   # publish
```

## Outside contributions

No pull request is merged on the mirror, by anyone. Two repository rulesets
let only the `rosieos-public-maintainers` team (the two maintainers)
create, update, delete or force-push any branch or tag there, so no one else
can push or merge; and a commit merged on the mirror would be dropped by the
next publish anyway, because a publish replaces the mirror's tree with the
export. Actions is disabled on the mirror, so a fork's pull request runs
nothing. [CONTRIBUTING.md](../../CONTRIBUTING.md) tells contributors this.

To take a public pull request:

1. A maintainer reviews it on the mirror and approves it or asks for changes.
2. Apply it on a private branch with its author kept, then land it like any
   other change (qualification plan, review, CI). The mirror's paths are this
   repository's paths:

   ```sh
   gh pr diff <n> -R advanced-metal-research/RosieOS-public | git apply --index
   git commit --author "<name> <email from the PR's commits>"
   ```

3. After it is on `origin/main`, publish, then close the public pull request
   with a comment naming the mirror commit that carries it.
