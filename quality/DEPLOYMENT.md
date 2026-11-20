# Deployment feedback, not another deployment path

Requires Node 22+ and Git. No package installation or agent subscription.

## See what a branch adds to deployment

```sh
node quality/deployment-surface.mjs origin/main HEAD
```

Compares exact Git trees, never executes their code. Reports declared runtime
paths, allowed roots, build command names, service units and daemon endpoints in
manifest JSON, plus added/removed command entrypoints and service files.
This is advisory. It neither allows an artifact nor blocks a merge. Generated
shell and arbitrary config writers are not reliably counted; do not call this
an exhaustive architecture audit. Review whether each added dependency earns
its cost and which older route it replaces.

## Harness hooks

Every harness can call these same commands. Do not mandate an authoring tool.
For fast local checks: `node --test quality/deployment-tools.test.mjs`.
The optional pre-commit configuration runs only these tests on relevant edits:
`pre-commit install --config quality/deployment-hooks.yaml`.
No live host probe runs at commit time. The advisory Actions report uses the
same surface command; it has read-only repository permissions and no secrets.

Keep larger interrupted-install/retry tests in the owning deployment component
and web/native conformance tests in the client work; do not duplicate them here.
