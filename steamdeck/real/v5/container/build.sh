#!/usr/bin/env bash
# Builds, checks and packages steamdeck/real/v5 from one commit, in a work tree
# that persists between builds (the container's /work volume, or a native
# linux/amd64 Ubuntu 22.04 builder's directory): fetch the commit from a git
# bundle, refuse a tree with changes of any kind (package.sh ships untracked
# files too, so nothing but the commit may be in the tree), check out exactly
# the commit, then build incrementally. npm ci and the tesseract pixi
# environment run only when their lock files changed. With
# <in>/offline-programming-linux present (cross-compiled natively by
# `pendant build`), package.sh takes that binary instead of building it.
# usage: build.sh <commit> <ref> <in> <out> <work> [--with-tesseract-env] [--clean]
set -euo pipefail
commit=${1:?commit}; ref=${2:?ref}; in=${3:?in}; out=${4:?out}; work=${5:?work}; shift 5
with_env=0; clean=0
for a in "$@"; do
  case $a in
    --with-tesseract-env) with_env=1 ;;
    --clean) clean=1 ;;
    *) echo "build: unknown option $a" >&2; exit 2 ;;
  esac
done
export PATH="$HOME/.rosie/bin:$HOME/.rosie/go-1.26.2/bin:$HOME/.rosie/node-22.13.1/bin:$PATH"
# LFS objects stay pointers for the whole build, checkout included: the bundle
# carries no LFS store and the package needs none of them. The container has no
# git-lfs; a native builder with it installed (the WSL builder, 2026-11-03)
# otherwise failed the checkout smudging weld_planner/v1/data/fixtures/*.step
# from the bundle path.
export GIT_LFS_SKIP_SMUDGE=1
T0=$(date +%s%N)
step(){ printf '@@step %d %s\n' $(( ($(date +%s%N) - T0) / 1000000 )) "$*"; }
refuse(){ printf '@@refused %s\n' "$*"; exit 2; }
repo=$work/RosieOS
git config --global --add safe.directory '*'
if [[ $clean == 1 && -e $repo ]]; then step "phase clean"; rm -rf -- "$repo"; fi
mkdir -p "$work" "$out"
step "phase source"
fresh=0
if [[ ! -d $repo/.git ]]; then
  git clone -q --no-checkout "$in/src.bundle" "$repo"
  fresh=1
fi
cd "$repo"
changes=""
[[ $fresh == 1 ]] || changes=$(git status --porcelain)
[[ -z $changes ]] || refuse "the work tree $repo has changes ($(printf '%s' "$changes" | head -n 5 | tr '\n' ' ')); inspect them, then pass --clean"
git fetch -q "$in/src.bundle" "$ref"
git -c advice.detachedHead=false checkout -q --detach "$commit"
[[ $(git rev-parse HEAD) == "$commit" ]] || refuse "the work tree is at $(git rev-parse HEAD), not $commit"
[[ -z $(git status --porcelain) ]] || refuse "the work tree has changes after checking out $commit"
step "build source $commit, clean"

stamp(){ sha256sum "$1" | cut -d' ' -f1; }
step "phase npm-ci"
ui=offline-programming/v1/ui
if [[ -d $ui/node_modules && $(cat "$ui/node_modules/.rosie-lock-sha256" 2> /dev/null) == "$(stamp "$ui/package-lock.json")" ]]; then
  step "build node_modules current for package-lock $(stamp "$ui/package-lock.json" | cut -c1-12)"
else
  (cd "$ui" && npm ci --no-audit --no-fund) && stamp "$ui/package-lock.json" > "$ui/node_modules/.rosie-lock-sha256"
fi
if [[ $with_env == 1 ]]; then
  step "phase tesseract-env"
  if [[ -d tesseract/v1/.pixi/envs/default && $(cat tesseract/v1/.pixi/.rosie-lock-sha256 2> /dev/null) == "$(stamp tesseract/v1/pixi.lock)" ]]; then
    step "build tesseract env current for pixi.lock $(stamp tesseract/v1/pixi.lock | cut -c1-12)"
  else
    (cd tesseract/v1 && pixi install --locked -e default) && stamp tesseract/v1/pixi.lock > tesseract/v1/.pixi/.rosie-lock-sha256
  fi
fi
step "phase make"
make -C steamdeck/real/v5 -j"$(nproc)" all
step "phase check"
make -C steamdeck/real/v5 check
step "phase package"
if [[ -f $in/offline-programming-linux ]]; then export OLP_SERVER_BIN=$in/offline-programming-linux; fi
if [[ $with_env == 1 ]]; then steamdeck/real/v5/package.sh --with-tesseract-env; else steamdeck/real/v5/package.sh; fi
changes=$(git status --porcelain)
[[ -z $changes ]] || refuse "the build left changes in the work tree ($(printf '%s' "$changes" | head -n 5 | tr '\n' ' ')): package.sh would have shipped them"
printf '%s\n' "$commit" > steamdeck/real/v5/build/package/RosiePendantV5/SOURCE_COMMIT
# A tarball, not a tree: a macOS host's case-insensitive filesystem collapses
# terminfo names in the tesseract environment (hp2621A vs hp2621a).
step "phase tar"
tarball=$out/RosiePendantV5-$commit.tar
tar -C steamdeck/real/v5/build/package/RosiePendantV5 -cf "$tarball.part" .
mv -f "$tarball.part" "$tarball"
sha=$(stamp "$tarball")
step "phase end"
printf '@@result {"status":"ok","commit":"%s","package":"%s","sha256":"%s","bytes":%d}\n' "$commit" "$(basename "$tarball")" "$sha" "$(stat -c %s "$tarball")"
