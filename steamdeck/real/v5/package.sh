#!/usr/bin/env bash
# Builds and stages the Deck app tree in build/package/RosiePendantV5 with a
# SHA256SUMS manifest. Run on Ubuntu 22.04 / WSL with the ~/.rosie toolchains.
# --with-tesseract-env also stages tesseract/v1's pixi environment (~1 GB),
# which only changes when its pixi.lock does. Site files (cell.json,
# olp-cells.json, olp-rt-core.json, planner.env, credentials, sim/) are never
# staged.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd "$here/../../.." && pwd)
export PATH="$HOME/.rosie/bin:$HOME/.rosie/go-1.26.2/bin:$HOME/.rosie/node-22.13.1/bin:$PATH"
out="$here/build/package/RosiePendantV5"
with_env=0
[[ "${1:-}" != --with-tesseract-env ]] || with_env=1

# The app, OLP's program logic bundle (olp-core.js) and the Cartesian solver.
# OLP_SERVER_BIN: a linux/amd64 server binary built from this same commit
# elsewhere (`pendant build` cross-compiles it natively instead of under the
# build container's emulation).
make -C "$here" -j4 all
if [[ -n "${OLP_SERVER_BIN:-}" ]]; then
  install -D -m 755 "$OLP_SERVER_BIN" "$here/build/package/offline-programming-linux"
else
  (cd "$repo/offline-programming/v1" && GOOS=linux GOARCH=amd64 CGO_ENABLED=0 \
    go build -o "$here/build/package/offline-programming-linux" .)
fi

# Empty rather than remove: a Windows handle on the directory blocks rmdir from WSL.
mkdir -p "$out" && find "$out" -mindepth 1 -delete
mkdir -p "$out/bin" "$out/lib"
install -m 755 "$here/run.sh" "$here/start-olp.sh" "$out/"
install -m 644 "$here/controller.vdf" "$out/"
install -m 755 "$here/build/rosie-pendant-v5" "$here/build/robot-v4-cartesiand" "$out/bin/"
install -m 644 "$here/build/olp-core.js" "$out/bin/"
mkdir -p "$out/share"
cp -a "$here/assets/robot-visuals" "$out/share/"

# SteamOS ships Qt 5.15 (qt5-base, qt5-declarative) but not Assimp or
# OpenH264 (the camera view's H.264 decoder). Assimp and the two it needs,
# and OpenH264, are the build host's own Ubuntu 22.04 copies, dereferenced,
# each with its distribution copyright notice.
for name in libassimp.so.5 libdraco.so.4 libminizip.so.1 libopenh264.so.6; do
  path=$(ldd "$here/build/rosie-pendant-v5" | awk -v n="$name" '$1 == n { print $3 }')
  [[ -f "$path" ]] || { echo "package: $name does not resolve on this build host" >&2; exit 1; }
  cp -L "$path" "$out/lib/$name"
  owner=$(dpkg -S "/usr/lib/x86_64-linux-gnu/$name" | head -1 | cut -d: -f1)
  stem=${name#lib}
  install -m 644 "/usr/share/doc/$owner/copyright" "$out/lib/${stem%%.so*}-copyright.txt"
done

# The OLP tree the server reads at runtime: tracked and untracked-but-not-ignored
# files, so the package is the working tree that was built and tested. A remote
# cell in olp-cells.json resolves through rt-core/config: its cell config, the
# machine config that names, and the drive profiles it compiles against
# (offline-programming/v1/internal/targets ResolveCellConfiguration).
mkdir -p "$out/olp"
git -C "$repo" ls-files -z -co --exclude-standard -- offline-programming/v1 motion-server/v1 \
    robot_description tesseract/v1 weld_planner/v1 cadquery/v1 \
    rt-core/config/cells rt-core/config/machines rt-core/config/drives |
  grep -zv '^offline-programming/v1/ui/dist/' |
  (cd "$repo" && xargs -0 cp --parents -t "$out/olp")
install -D -m 755 "$here/build/package/offline-programming-linux" "$out/olp/offline-programming/v1/bin/offline-programming-linux"
# `serve` refuses to start without ui/dist/index.html. The pendant never opens
# the server's UI, so this stub stands there. Never the checkout's ui/dist: it
# is the deliberate :5189 redirect stub.
mkdir -p "$out/olp/offline-programming/v1/ui/dist"
cat > "$out/olp/offline-programming/v1/ui/dist/index.html" <<'EOF'
<!doctype html>
<meta charset="utf-8">
<title>Rosie pendant v5</title>
<p>This OLP server is the backend of the native Rosie pendant (bin/rosie-pendant-v5). It serves no browser UI.</p>
EOF
# One solver: the local planner's canonical path names the pendant's resolver.
mkdir -p "$out/olp/motion-server/v1/bin"
ln -sfn ../../../../bin/robot-v4-cartesiand "$out/olp/motion-server/v1/bin/robot-v4-cartesiand"
if (( with_env )); then
  mkdir -p "$out/olp/tesseract/v1/.pixi/envs"
  cp -a "$repo/tesseract/v1/.pixi/envs/default" "$out/olp/tesseract/v1/.pixi/envs/"
fi

(cd "$out" && find . -path ./olp/tesseract/v1/.pixi -prune -o -type f -print0 | sort -z |
  xargs -0 sha256sum > ../SHA256SUMS && mv ../SHA256SUMS SHA256SUMS)
grep -E ' \./(bin/|lib/|olp/offline-programming/v1/bin/|olp/offline-programming/v1/ui/dist/index.html|run\.sh|start-olp\.sh)' "$out/SHA256SUMS"
