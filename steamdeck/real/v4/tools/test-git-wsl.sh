#!/usr/bin/env bash
# Translate Windows worktree pointers for local tests without changing Git
# configuration or teaching deployed commands about the developer host.
set -eu
args=("$@")
root="$PWD"
if [[ "${1:-}" == -C ]]; then
  root="$(realpath -- "$2")"
fi
while [[ ! -e "$root/.git" && "$root" != / ]]; do root="$(dirname "$root")"; done
if [[ -f "$root/.git" ]]; then
  pointer="$(cat "$root/.git")"
  if [[ "$pointer" =~ ^gitdir:\ [A-Za-z]: ]]; then
    gitdir="$(wslpath -u "${pointer#gitdir: }")"
    exec /usr/bin/git --git-dir="$gitdir" --work-tree="$root" "${args[@]}"
  fi
fi
exec /usr/bin/git "${args[@]}"
