# Managed release test coverage

The managed release gate has 12 source and behavior suites. The gate runs them before its legacy build and deploy path.

The `release-test-coverage.json` file maps each suite to one proof owner. It also records whether existing proof is complete.

Think of this map as a wiring diagram. It shows which station checks each part without moving the test itself.

## Proof owners

| Owner | Use |
|---|---|
| `pull_request` | Run fast, target-free source checks before merge. |
| `build1_candidate` | Run package checks and isolated timing checks for one immutable candidate. |
| `target_qualification` | Run checks that require a Jetson, RevPi, integrated cell, or physical motion. |

The map does not make partial proof equal to the managed gate. A `partial` entry names the overlap and the missing command.

`covered` means that the named workflow contains the check. It does not prove that GitHub ran or required that workflow for a commit.

For a source-owned suite, evidence names the fixed worker source and command.

A `gap` entry has no matching check. It must name the missing proof.

## Validation

Run the contract check:

```sh
node quality/release-test-coverage.mjs check
```

The check compares the map with `gateSuites` in `steamdeck/real/v4/release_tests.go`. It fails if either list changes alone.

The check also verifies each workflow, job, and command reference. A renamed GitHub Actions job cannot leave silent stale evidence.

The `PR admission / required` job runs this check. The job remains one stable branch-protection context.

## Current limits

This contract records coverage. It does not execute a missing suite.

All 12 suites have an implementation owner. Four suites run alone in the build host's fixed worker.

Each build-host suite has its own deadline, log, and source-bound receipt. A failed suite does not hide later suite results.

Hardware proof remains a separate target qualification step.

Every candidate and target result must later reference the source commit and candidate digest. This contract does not add that receipt field.
