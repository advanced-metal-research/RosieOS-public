#!/usr/bin/env bash
# Read build identities across native and Windows-created WSL worktrees without rewriting git metadata.
set -euo pipefail
export GIT_OPTIONAL_LOCKS=0
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${component}"
if git rev-parse --git-dir >/dev/null 2>&1; then exec git "$@"; fi
pointer="$(dirname "${component}")/.git"
[[ -f "${pointer}" ]] || { printf 'cannot resolve worktree git directory\n' >&2; exit 1; }
git_dir="$(sed -n 's/^gitdir: //p' "${pointer}" | tr -d '\r')"
[[ "${git_dir}" =~ ^[A-Za-z]:/ ]] || { printf 'unsupported worktree git pointer\n' >&2; exit 1; }
git_dir="$(wslpath -u "${git_dir}")"
exec git --git-dir="${git_dir}" --work-tree="$(dirname "${component}")" "$@"
