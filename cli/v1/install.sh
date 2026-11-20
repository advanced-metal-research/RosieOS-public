#!/usr/bin/env bash
# Install the `rosie` command as ~/.rosie/bin/rosie (ROSIE_HOME/bin): a
# wrapper that runs THIS checkout's build, so `git pull && cli/v1/install.sh`
# is the whole upgrade. ~/.rosie/bin is already on the documented toolchain
# PATH (CLAUDE.md "Toolchains"). Needs Node 22+ (the snapshot driver uses
# Node's built-in WebSocket) and, for `rosie view`, Chrome or Edge.
# The rosie agent skills (.claude/skills/rosie*) are linked into
# ~/.claude/skills so every Claude Code session on this machine finds them;
# --no-skills skips that.
set -euo pipefail

skills=1
for arg in "$@"; do
  case "$arg" in
    --no-skills) skills=0 ;;
    *) echo "usage: install.sh [--no-skills]" >&2; exit 2 ;;
  esac
done

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ui="$here/../../offline-programming/v1/ui"
home="${ROSIE_HOME:-$HOME/.rosie}"

major="$(node -p 'process.versions.node.split(".")[0]' 2>/dev/null || echo 0)"
if [ "$major" -lt 22 ]; then
  echo "rosie needs Node 22 or later on PATH (found: $(node --version 2>/dev/null || echo none))" >&2
  exit 1
fi

# The engine bundle and the snapshot page are built from OLP's own UI packages.
[ -d "$ui/node_modules/three" ] || (cd "$ui" && npm ci)
[ -d "$here/node_modules/typescript" ] || (cd "$here" && npm ci)
(cd "$here" && npm run build)

mkdir -p "$home/bin"
cat > "$home/bin/rosie" <<WRAPPER
#!/usr/bin/env bash
# Installed by $here/install.sh: runs that checkout's build.
exec node "$here/dist/src/main.js" "\$@"
WRAPPER
chmod +x "$home/bin/rosie"

echo "installed $home/bin/rosie -> $here/dist/src/main.js"
case ":$PATH:" in
  *":$home/bin:"*) ;;
  *) echo "add it to PATH: export PATH=\"$home/bin:\$PATH\"" ;;
esac
"$home/bin/rosie" --version
if [ "$skills" = 1 ]; then
  "$home/bin/rosie" skills install || echo "skills not installed (see above); rosie works without them" >&2
fi
