#!/usr/bin/env bash
# Run deployment checks with WSL worktree identity, without changing Git config.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
repo="$(cd ../../.. && pwd)"
mkdir -p "$repo/rt-core/build/test-tools"
rm -f ../../../rt-core/build/test-tools/git
cat > ../../../rt-core/build/test-tools/git <<'GIT'
#!/usr/bin/env bash
exec bash "$(dirname "${BASH_SOURCE[0]}")/../../../steamdeck/real/v4/tools/test-git-wsl.sh" "$@"
GIT
chmod 0755 ../../../rt-core/build/test-tools/git
export PATH="$PWD/../../../rt-core/build/test-tools:$PATH"
if [[ "${1:-}" == go && "${2:-}" == test ]]; then
  # Go's testing.TempDir honors GOTMPDIR; keep test sockets on the supplied tmpfs.
  cat > ../../../rt-core/build/test-tools/run-test <<'TEST'
#!/usr/bin/env bash
export GOTMPDIR="${TMPDIR:?test tmpfs required}"
exec "$@"
TEST
  chmod 0755 ../../../rt-core/build/test-tools/run-test
  shift 2
  exec go test -exec "$PWD/../../../rt-core/build/test-tools/run-test" "$@"
fi
exec "$@"
