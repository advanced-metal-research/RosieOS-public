# Contributing

RosieOS is developed in a private repository. The public repository,
[RosieOS-public](https://github.com/advanced-metal-research/RosieOS-public),
is a mirror of it: one commit per release, published from the private main
branch. Nothing is merged on the mirror, so a pull request there is a proposal
that a maintainer carries into the private repository.

1. Open a pull request against `main` with one change. Say what it fixes and
   how you tested it, with the commands you ran.
2. A maintainer reviews it. Only the RosieOS maintainers can approve a change;
   no one else can push to or merge on the mirror.
3. An approved change is applied in the private repository with you as its
   author and goes through the same checks as any change there. The next
   release publishes it, and the pull request is closed with a link to that
   release's commit.

The mirror runs no CI; the maintainers run the checks below on the private
branch. A change to motion, drive handling or anything that commands a robot
also needs evidence from real hardware that only the maintainers can collect,
and waits until that evidence exists. Contributions are licensed under the
Apache License 2.0 ([LICENSE](LICENSE)).

# Quality checks

For read-only host/config diagnostics and advisory deployment dependency diffs,
see [deployment feedback tools](quality/DEPLOYMENT.md). They run directly with
Node, without an agent harness. Live probes are never commit hooks.

Install the existing coordination development environment with
`npm ci --prefix coordination/v1`. From the repository root:

```sh
node quality/quality.mjs check
node quality/quality.mjs format
node quality/quality.mjs format-check
node quality/quality.mjs lint
node quality/quality.mjs typecheck
```

`check` is the CPU-only CI gate: Oxfmt, existing Oxlint (including the native
restricted-type rule), tested ast-grep imports, generated Worker types, both
existing tsc projects, domain tests, and schema tests. It does not run a server,
deploy, or replace the existing package `check`/runtime integration tests.
Only `format` rewrites hand-authored source, and only coordination `src`, `test`,
`scripts`, and `public/app.js`. `typecheck` (also called by `check`) generates the
ignored `coordination/v1/worker-configuration.d.ts`. Oxfmt import and package
sorting are disabled. No default
command formats evidence, CAD, generated files, vendor code, or historical trees.

Use one formatter per language; keep formatting-only commits separate from
behavior changes. Preserve explicit errors, units, clock domains and ownership.
Suppression comments must name the rule and reason; never hide an error by
dropping failure handling or weakening a test. Linters do not prove installation,
controller admission, collision freedom or physical execution.

## Incremental native reports (not CI gates)

Report-only means **not required by CI**. Missing tools and nonzero tool exits
propagate failure. Clippy is explicitly execution-only: cargo can exit zero with
warnings, so the runner prints `report_completed_not_qualified`, never `passed`
for that step. Read its diagnostics; exit zero does not mean no Clippy findings.
Other check commands return nonzero on findings. No command applies fixes.

| Command suffix | Scope and prerequisite |
| --- | --- |
| `python` | weld_planner/v1 python/tests/tools; `uvx` runs isolated Ruff 0.15.6 without the GPU/CAD environment, retaining existing project Ruff configuration |
| `go` | tracked Robot-v4 Go sources, then module-wide golangci-lint 2.13.2 (errcheck, govet, staticcheck); module's Go toolchain required |
| `workflows` | all workflows; actionlint 1.7.12 via the Go module registry; shell snippets are deliberately separate |
| `shell` | Robot-v4 local-sim shell; locally provisioned ShellCheck (baseline 0.11.0) |
| `cpp <compile-db-dir> <source>` | one motion-server/joint-trajectory/v1/src file, existing real compile_commands.json, locally provisioned clang-format/clang-tidy; no guessed compiler flags or build invocation |

For example: `node quality/quality.mjs python`. Pinned npm tools live in the
existing coordination lockfile; Go/Ruff registry versions are literal in the
runner. ShellCheck and LLVM are system prerequisites, not auto-installed.
The C++ configuration is opt-in; it is not an accepted repository-wide style.
No C++ checks can be claimed without the actual target compile database.

Promote native checks module by module after measuring and resolving their
baseline. Keep full-module reports alongside new-code checks: diff filtering
alone misses dependency-wide regressions. Do not mass-fix old debt. ShellCheck's
two arithmetic warnings on `expected-safety-rejection` are known false positives,
not evidence of runtime defects. Generated shell inside Go remains a separate
render-and-analyze gap.

The house rule rejects static child_process imports in the coordination Worker
source (which only hands work to owning runtimes). Three forbidden imports and
three legitimate nearby forms are tested. This syntax rule is not a sandbox:
dynamic imports, require aliases and dataflow are outside its coverage.
