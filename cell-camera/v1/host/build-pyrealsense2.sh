#!/usr/bin/env bash
# Build Intel's pyrealsense2 with librealsense's RSUSB (libusb) backend for the
# cell camera's RealSense source (cell_camera/realsense.py), for the Python the
# camera unit runs: the host's /usr/bin/python3 (CPython 3.10 on the Jetsons'
# Ubuntu 22.04). host/install.sh --realsense-sdk takes the package this makes.
#
# Why not PyPI's wheel: it reads the camera through V4L2, which delivers each
# frame's metadata on a second device node that librealsense pairs back up with
# the frame. When colour frames arrive cut short, each costs librealsense a
# metadata buffer, and once all four are gone the colour stream stops for good.
# RSUSB reads frames over libusb with the metadata inside the frame's own
# header, so there is nothing to pair. PyPI also has no wheel for every Python
# on aarch64.
#
# On the cell host itself (native): needs build-essential, cmake, git,
# libusb-1.0-0-dev, pkg-config and python3-dev. A heavy job on a cell host runs
# under the cell lock (CLAUDE.md), from steamdeck/real/v4:
#   go run . realtime-core lock --manifest ../../../rt-core/config/cells/<cell>.json \
#     --reason "build pyrealsense2" -- bash <tree>/cell-camera/v1/host/build-pyrealsense2.sh
#
# --arch aarch64 on an x86_64 host builds the same package for an arm64 Ubuntu
# 22.04 host instead, inside an emulated arm64 Ubuntu 22.04: the pinned
# ubuntu-base root filesystem, entered with bwrap as an unprivileged user
# namespace and run by the host's qemu-aarch64 binfmt_misc handler, which must
# carry the F flag (`sudo apt install qemu-user-static bubblewrap` registers it
# so). The build tools and python3-dev come from the frozen jammy release
# pocket, so the extension is built against Ubuntu 22.04's CPython 3.10 headers
# and glibc. It is kept only once its ELF machine is AArch64, it needs no GLIBC
# newer than 2.35, and it has the cp310 aarch64 name.
#
# Default: acquire the pinned inputs and build. An existing build of the same
# pins is left alone.
# --download-only --inputs-dir DIR acquires bare Git mirrors, the SQLite ZIP
# and, for --arch aarch64, the root filesystem and the build packages.
# --offline-prepare --inputs-dir DIR checks those inputs, then builds with no
# network: the emulated build runs in a network namespace of its own.
# --out DIR puts the package elsewhere than
# ${ROSIE_HOME:-$HOME/.rosie}/realsense/pyrealsense2-<version>-<arch>.
# Work files go beside the output, never into the source tree.
set -euo pipefail

# A step this script runs inside the emulated root filesystem (see with_rootfs), not a usage.
INNER=()
if [[ "${1:-}" == --in-target ]]; then
    INNER=("${@:2}")
    set --
fi

LRS_TAG="v2.59.1"
LRS_COMMIT="625279aa1fe903528a818f48746ee47c1b5fefd3"   # what $LRS_TAG pointed at when pinned
LRS_URL="https://github.com/realsenseai/librealsense.git"
JSON_URL="https://github.com/nlohmann/json.git"
JSON_TAG="v3.12.0"
JSON_TAG_OBJECT="65ee68451d8eb2b5f3a30b410476ab83deb3289b"
JSON_COMMIT="55f93686c01528224f448c19128836e7df245f72"
PYBIND11_URL="https://github.com/pybind/pybind11.git"
PYBIND11_TAG="v2.13.6"
PYBIND11_COMMIT="a2e59f0e7065404b44dfe92a28aca47ba1378dc4"
PYBIND11_JSON_URL="https://github.com/pybind/pybind11_json.git"
PYBIND11_JSON_TAG="0.2.15"
PYBIND11_JSON_COMMIT="32043f433ed987b2c2ce99d689ec337bcbd4ba95"
FASTCDR_URL="https://github.com/eProsima/Fast-CDR.git"
FASTCDR_TAG="v1.0.25"
FASTCDR_COMMIT="d6069f345be2e5b34534815a82c3ecfaecb37406"
YAML_CPP_URL="https://github.com/jbeder/yaml-cpp.git"
YAML_CPP_TAG="yaml-cpp-0.7.0"
YAML_CPP_COMMIT="0579ae3d976091d7d664aa9d2527e0d0cff25763"
LIBDEFLATE_URL="https://github.com/ebiggers/libdeflate.git"
LIBDEFLATE_TAG="v1.25"
LIBDEFLATE_TAG_OBJECT="bf11fcce4d2e12c413ac92ba519917a92e9baac7"
LIBDEFLATE_COMMIT="c8c56a20f8f621e6a966b716b31f1dedab6a41e3"
LIBUSB_CMAKE_URL="https://github.com/libusb/libusb-cmake.git"
LIBUSB_CMAKE_TAG="v1.0.27-1"
LIBUSB_CMAKE_COMMIT="cec1e49eec481a6333c218726ecaba8e4f447a07"
# SQLite's own archive, which librealsense's file support downloads during its build.
SQLITE_URL="https://sqlite.org/2025/sqlite-amalgamation-3490100.zip"
SQLITE_SHA256="6cebd1d8403fc58c30e93939b246f3e6e58d0765a5cd50546f16c00fd805d2c3"
# The emulated arm64 build's root filesystem: Canonical's Ubuntu 22.04 base image.
ROOTFS_URL="https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/ubuntu-base-22.04-base-arm64.tar.gz"
ROOTFS_SHA256="6dd67ec02fdc64b5bba4125066462d01e66a2ae14c4c9e571541fba617d7e721"
TARGET_GLIBC="2.35"        # Ubuntu 22.04's: no newer symbol may be needed
TARGET_PYTHON_TAG="cp310"  # Ubuntu 22.04's python3, the one the camera unit runs
# The jammy release pocket never changes after release, so these resolve to the same packages every time.
APT_SOURCE="deb http://ports.ubuntu.com/ubuntu-ports jammy main universe"
APT_PACKAGES=(build-essential cmake git libusb-1.0-0-dev pkg-config python3-dev)

fail() {
    echo "build-pyrealsense2: $*" >&2
    exit 1
}

MODE=full
INPUTS_DIR=""
HOST_ARCH="$(uname -m)"
ARCH="$HOST_ARCH"
OUT_DIR=""
while (( $# )); do
    case "$1" in
        --download-only|--offline-prepare)
            [[ "$MODE" == full ]] || fail "select only one phase"
            MODE="${1#--}"
            shift
            ;;
        --inputs-dir)
            [[ $# -ge 2 && -n "$2" ]] || fail "--inputs-dir requires DIR"
            INPUTS_DIR="$2"
            shift 2
            ;;
        --arch)
            [[ $# -ge 2 && -n "$2" ]] || fail "--arch requires ARCH"
            ARCH="$2"
            shift 2
            ;;
        --out)
            [[ $# -ge 2 && -n "$2" ]] || fail "--out requires DIR"
            OUT_DIR="$2"
            shift 2
            ;;
        *) fail "usage: $0 [--arch aarch64] [--out DIR] [--download-only|--offline-prepare] [--inputs-dir DIR]" ;;
    esac
done
[[ "$MODE" == full || -n "$INPUTS_DIR" ]] || fail "explicit phases require --inputs-dir DIR"

# The host's own architecture builds natively; arm64 on x86_64 is emulated.
EMULATED=false
if [[ "$ARCH" != "$HOST_ARCH" ]]; then
    [[ "$ARCH" == aarch64 && "$HOST_ARCH" == x86_64 ]] || fail "can't build for $ARCH on $HOST_ARCH: only aarch64 on x86_64"
    EMULATED=true
fi
BUILT_MARK="librealsense $LRS_TAG $LRS_COMMIT RSUSB $ARCH $TARGET_PYTHON_TAG"
if (( ${#INNER[@]} == 0 )); then
    [[ -n "$OUT_DIR" ]] || OUT_DIR="${ROSIE_HOME:-$HOME/.rosie}/realsense/pyrealsense2-${LRS_TAG#v}-$ARCH"
    mkdir -p "$(dirname "$OUT_DIR")"
    OUT_DIR="$(cd "$(dirname "$OUT_DIR")" && pwd -P)/$(basename "$OUT_DIR")"
    WORK_ROOT="$(dirname "$OUT_DIR")/.build-pyrealsense2-$ARCH"
fi

if (( ${#INNER[@]} == 0 )) && [[ "$MODE" == full && -f "$OUT_DIR/BUILT_FROM" ]] &&
    [[ "$(cat "$OUT_DIR/BUILT_FROM")" == "$BUILT_MARK" ]]; then
    echo "pyrealsense2 $LRS_TAG (RSUSB, $ARCH, $TARGET_PYTHON_TAG): already in $OUT_DIR/"
    exit 0
fi

# A full run without --inputs-dir acquires into a temporary directory it removes.
if (( ${#INNER[@]} == 0 )) && [[ -z "$INPUTS_DIR" ]]; then
    mkdir -p "$WORK_ROOT"
    INPUTS_DIR="$(mktemp -d "$WORK_ROOT/inputs.XXXXXX")"
    trap 'rm -rf "$INPUTS_DIR"' EXIT
fi

# Do not inherit Git configuration, object stores, replacements, hooks or credentials. These
# settings also reach the Git commands librealsense's CMake runs.
unset GIT_CONFIG GIT_CONFIG_PARAMETERS GIT_DIR GIT_WORK_TREE GIT_COMMON_DIR GIT_INDEX_FILE
unset GIT_OBJECT_DIRECTORY GIT_ALTERNATE_OBJECT_DIRECTORIES GIT_SHALLOW_FILE GIT_REPLACE_REF_BASE
unset GIT_ASKPASS SSH_ASKPASS GIT_SSH GIT_SSH_COMMAND GIT_PROXY_COMMAND
export GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_SYSTEM=/dev/null GIT_CONFIG_GLOBAL=/dev/null
export GIT_TERMINAL_PROMPT=0 GIT_NO_REPLACE_OBJECTS=1 GIT_NO_LAZY_FETCH=1
export GIT_CONFIG_COUNT=4
export GIT_CONFIG_KEY_0=credential.helper GIT_CONFIG_VALUE_0=
export GIT_CONFIG_KEY_1=core.hooksPath GIT_CONFIG_VALUE_1=/dev/null
# No detached maintenance may change an input mirror after it was acquired.
export GIT_CONFIG_KEY_2=gc.auto GIT_CONFIG_VALUE_2=0
export GIT_CONFIG_KEY_3=maintenance.auto GIT_CONFIG_VALUE_3=false

# libusb-cmake is librealsense's fallback when the host has no libusb; it is
# staged either way so CMake's own choice is never short of an input.
GIT_NAMES=(librealsense json pybind11 pybind11_json fastcdr yaml-cpp libdeflate libusb-cmake)
GIT_URLS=("$LRS_URL" "$JSON_URL" "$PYBIND11_URL" "$PYBIND11_JSON_URL" "$FASTCDR_URL" "$YAML_CPP_URL" "$LIBDEFLATE_URL" "$LIBUSB_CMAKE_URL")
GIT_TAGS=("$LRS_TAG" "$JSON_TAG" "$PYBIND11_TAG" "$PYBIND11_JSON_TAG" "$FASTCDR_TAG" "$YAML_CPP_TAG" "$LIBDEFLATE_TAG" "$LIBUSB_CMAKE_TAG")
GIT_COMMITS=("$LRS_COMMIT" "$JSON_COMMIT" "$PYBIND11_COMMIT" "$PYBIND11_JSON_COMMIT" "$FASTCDR_COMMIT" "$YAML_CPP_COMMIT" "$LIBDEFLATE_COMMIT" "$LIBUSB_CMAKE_COMMIT")
GIT_TAG_OBJECTS=("" "$JSON_TAG_OBJECT" "" "" "" "" "$LIBDEFLATE_TAG_OBJECT" "")

verify_mirror() {
    local mirror="$1" index="$2" actual
    [[ -d "$mirror" && ! -L "$mirror" ]] || fail "missing or unsafe Git input: $mirror"
    [[ "$(git --git-dir="$mirror" rev-parse --is-bare-repository)" == true ]] || fail "not a bare mirror: $mirror"
    [[ "$(git --git-dir="$mirror" config --local --get remote.origin.url)" == "${GIT_URLS[$index]}" ]] || fail "foreign mirror: $mirror"
    [[ ! -e "$mirror/shallow" && ! -e "$mirror/objects/info/alternates" && ! -e "$mirror/objects/info/http-alternates" ]] || fail "incomplete or external object store: $mirror"
    if git --git-dir="$mirror" config --local --get-regexp '^(extensions\.partialclone|remote\..*\.(promisor|partialclonefilter))$'; then
        fail "partial Git input: $mirror"
    fi
    actual="$(git --git-dir="$mirror" rev-parse --verify "refs/tags/${GIT_TAGS[$index]}^{commit}")"
    [[ "$actual" == "${GIT_COMMITS[$index]}" ]] || fail "moved or foreign tag in $mirror"
    if [[ -n "${GIT_TAG_OBJECTS[$index]}" ]]; then
        actual="$(git --git-dir="$mirror" rev-parse --verify "refs/tags/${GIT_TAGS[$index]}")"
        [[ "$actual" == "${GIT_TAG_OBJECTS[$index]}" ]] || fail "foreign tag object in $mirror"
    fi
    # A full traversal checks every object, not only that the pinned commit's name exists.
    git --git-dir="$mirror" fsck --full --no-reflogs
}

verify_sha256() {
    local file="$1" sha="$2" what="$3"
    [[ -f "$file" && ! -L "$file" ]] || fail "missing or unsafe $what input: $file"
    printf '%s  %s\n' "$sha" "$file" | sha256sum --check --status || fail "corrupt or foreign $what input: $file"
}

acquire_mirror() (
    local index="$1" mirror="$INPUTS_DIR/git/${GIT_NAMES[$1]}.git" staging
    if [[ -e "$mirror" || -L "$mirror" ]]; then
        verify_mirror "$mirror" "$index"
        return
    fi
    staging="$(mktemp -d "$INPUTS_DIR/git/.${GIT_NAMES[$index]}.XXXXXX")"
    trap 'rm -rf "$staging"' EXIT
    # Only the pinned tag's complete object closure: no shallow or partial clone, no other branches.
    git init --quiet --bare --template= "$staging/repository"
    git --git-dir="$staging/repository" remote add origin "${GIT_URLS[$index]}"
    git --git-dir="$staging/repository" fetch --quiet --no-tags origin \
        "refs/tags/${GIT_TAGS[$index]}:refs/tags/${GIT_TAGS[$index]}"
    verify_mirror "$staging/repository" "$index"
    [[ ! -e "$mirror" && ! -L "$mirror" ]] || fail "Git input appeared during acquisition: $mirror"
    mv "$staging/repository" "$mirror"
)

acquire_download() (
    local url="$1" sha="$2" what="$3" file="$INPUTS_DIR/downloads/${1##*/}" staging
    if [[ -e "$file" || -L "$file" ]]; then
        verify_sha256 "$file" "$sha" "$what"
        return
    fi
    staging="$(mktemp -d "$INPUTS_DIR/downloads/.download.XXXXXX")"
    trap 'rm -rf "$staging"' EXIT
    curl -q --fail --location --proto '=https' --proto-redir '=https' --output "$staging/file" "$url"
    verify_sha256 "$staging/file" "$sha" "$what"
    [[ ! -e "$file" && ! -L "$file" ]] || fail "$what input appeared during acquisition: $file"
    mv "$staging/file" "$file"
)

acquire_inputs() {
    local index
    mkdir -p "$INPUTS_DIR/git" "$INPUTS_DIR/downloads"
    for index in "${!GIT_NAMES[@]}"; do
        acquire_mirror "$index"
    done
    acquire_download "$SQLITE_URL" "$SQLITE_SHA256" SQLite
    if [[ "$EMULATED" == true ]]; then
        acquire_download "$ROOTFS_URL" "$ROOTFS_SHA256" "arm64 root filesystem"
        acquire_packages
    fi
}

verify_inputs() {
    local index
    if [[ "$EMULATED" == true ]]; then
        verify_sha256 "$INPUTS_DIR/downloads/${ROOTFS_URL##*/}" "$ROOTFS_SHA256" "arm64 root filesystem"
        verify_packages
    fi
    for index in "${!GIT_NAMES[@]}"; do
        verify_mirror "$INPUTS_DIR/git/${GIT_NAMES[$index]}.git" "$index"
    done
    verify_sha256 "$INPUTS_DIR/downloads/${SQLITE_URL##*/}" "$SQLITE_SHA256" SQLite
}

use_offline_git() {
    local index config_index key value
    export GIT_ALLOW_PROTOCOL=file
    for index in "${!GIT_NAMES[@]}"; do
        config_index="$GIT_CONFIG_COUNT"
        key="GIT_CONFIG_KEY_$config_index"
        value="GIT_CONFIG_VALUE_$config_index"
        export "$key=url.file://$INPUTS_DIR/git/${GIT_NAMES[$index]}.git.insteadOf"
        export "$value=${GIT_URLS[$index]}"
        GIT_CONFIG_COUNT=$((GIT_CONFIG_COUNT + 1))
    done
}

# ---- emulated arm64 build ----

# What the emulation needs from the host, checked before anything is fetched or built.
check_emulation() {
    local handler=/proc/sys/fs/binfmt_misc/qemu-aarch64
    command -v bwrap > /dev/null || fail "bwrap is not installed (sudo apt install bubblewrap)"
    [[ -r "$handler" ]] ||
        fail "no qemu-aarch64 binfmt_misc handler on this host; register one once with: sudo apt install qemu-user-static"
    [[ "$(head -n 1 "$handler")" == enabled ]] || fail "the qemu-aarch64 binfmt_misc handler is disabled"
    grep -q '^flags: .*F' "$handler" ||
        fail "the qemu-aarch64 binfmt_misc handler lacks the F flag, so it can't run inside a root filesystem"
}

verify_packages() {
    [[ -f "$INPUTS_DIR/apt/PACKAGES" && "$(cat "$INPUTS_DIR/apt/PACKAGES")" == "${APT_PACKAGES[*]}" ]] ||
        fail "missing or different arm64 build packages: $INPUTS_DIR/apt (acquire them with --download-only)"
}

# The build tools' Debian packages, resolved and downloaded by apt inside the root filesystem (apt checks the
# archive's signature and every package's digest), kept with apt's package lists for the offline install.
acquire_packages() {
    if [[ -f "$INPUTS_DIR/apt/PACKAGES" ]]; then
        verify_packages
        return
    fi
    local work
    mkdir -p "$INPUTS_DIR/apt/lists/partial" "$INPUTS_DIR/apt/archives/partial" "$WORK_ROOT"
    work="$(mktemp -d "$WORK_ROOT/apt.XXXXXX")"
    with_rootfs "$work" online apt_download "$INPUTS_DIR" || { rm -rf "$work"; fail "apt couldn't fetch the build packages"; }
    rm -rf "$work"
    echo "${APT_PACKAGES[*]}" > "$INPUTS_DIR/apt/PACKAGES"
}

# Unpack a private root filesystem under $1 and run this script's step $3... inside it with bwrap as its
# root user (a user namespace: no host privilege), the inputs at their own path. $2 online lets it reach
# the package archive; anything else gives it no network.
with_rootfs() {
    local base="$1" network="$2" rootfs
    shift 2
    rootfs="$base/rootfs"
    mkdir -p "$rootfs"
    tar -xzf "$INPUTS_DIR/downloads/${ROOTFS_URL##*/}" -C "$rootfs" --exclude='./dev/*'
    echo "$APT_SOURCE" > "$rootfs/etc/apt/sources.list"
    rm -f "$rootfs/etc/resolv.conf"
    local args=(--unshare-user --uid 0 --gid 0 --unshare-pid --die-with-parent
        --bind "$rootfs" / --dev /dev --proc /proc --tmpfs /tmp
        --ro-bind "${BASH_SOURCE[0]}" /opt/build-pyrealsense2.sh
        --clearenv --setenv PATH /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
        --setenv HOME /tmp --setenv LC_ALL C --setenv DEBIAN_FRONTEND noninteractive)
    if [[ "$network" == online ]]; then
        cp -L /etc/resolv.conf "$rootfs/etc/resolv.conf"
        args+=(--bind "$INPUTS_DIR" "$INPUTS_DIR")
    else
        args+=(--unshare-net --ro-bind "$INPUTS_DIR" "$INPUTS_DIR")
    fi
    [[ ! -d "$base/tree" ]] || args+=(--bind "$base/tree" "$base/tree")
    bwrap "${args[@]}" /bin/bash /opt/build-pyrealsense2.sh --in-target "$@"
}

apt_options() {
    local inputs="$1"
    echo "-o Dir::State::Lists=$inputs/apt/lists -o Dir::Cache::Archives=$inputs/apt/archives"
    echo "-o APT::Sandbox::User=root -o APT::Install-Recommends=false -o Acquire::Retries=3"
}

# Inside the root filesystem: resolve and download the build packages into the inputs.
inner_apt_download() {
    local inputs="$1"
    # shellcheck disable=SC2046  # apt_options is a list of words
    apt-get $(apt_options "$inputs") update
    # shellcheck disable=SC2046
    apt-get $(apt_options "$inputs") install --yes --download-only "${APT_PACKAGES[@]}"
}

# Inside the root filesystem, without network: install the downloaded build packages, then compile
# against the root filesystem's own python3, Ubuntu 22.04's.
inner_build() {
    local inputs="$1" tree="$2" jobs="$3"
    # shellcheck disable=SC2046
    apt-get $(apt_options "$inputs") install --yes --no-download "${APT_PACKAGES[@]}"
    INPUTS_DIR="$inputs"
    use_offline_git
    compile_sdk "$tree" /usr/bin/python3 "$jobs"
    stage_package "$tree" /usr/bin/python3
}

# Is $1 an ELF library for the build's target, using no GLIBC past the target's?
check_elf() {
    local file="$1" header newest
    header="$(readelf -h "$file" 2> /dev/null)" || fail "$file is not an ELF library"
    grep -q 'Class: *ELF64' <<< "$header" || fail "$file is not a 64-bit ELF"
    grep -q 'Machine: *AArch64' <<< "$header" || fail "$file is not an AArch64 library: $(grep Machine <<< "$header")"
    newest="$(objdump -T "$file" | grep -o 'GLIBC_[0-9.]*' | sed 's/GLIBC_//' | sort -V | tail -n 1)"
    [[ -z "$newest" || "$(printf '%s\n%s\n' "$newest" "$TARGET_GLIBC" | sort -V | tail -n 1)" == "$TARGET_GLIBC" ]] ||
        fail "$file needs GLIBC $newest, newer than the target's $TARGET_GLIBC"
}

# ---- the build ----

# Check out librealsense in $1/librealsense and compile its Python binding into $1/build with Python $2, $3 jobs.
compile_sdk() {
    local WORK_DIR="$1" PYTHON="$2" JOBS="$3" SRC
    SRC="$WORK_DIR/librealsense"
    git clone --quiet --no-hardlinks --branch "$LRS_TAG" "file://$INPUTS_DIR/git/librealsense.git" "$SRC"
    [[ "$(git -C "$SRC" rev-parse HEAD)" == "$LRS_COMMIT" ]] || fail "incorrect librealsense checkout"

    # Declare librealsense's sqlite3 download target in advance, from the verified local archive. Every path
    # is upstream's default in third-party/realsense-file; its sqlite3_lib compile and link stay upstream's.
    # The local path is escaped for a CMake quoted argument; no upstream source is edited.
    local sqlite_archive="$INPUTS_DIR/downloads/${SQLITE_URL##*/}"
    sqlite_archive="${sqlite_archive//\\/\\\\}"
    sqlite_archive="${sqlite_archive//\"/\\\"}"
    sqlite_archive="${sqlite_archive//\$/\\\$}"
    cat > "$WORK_DIR/sqlite-input.cmake" <<EOF
include(ExternalProject)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
set(_sdk_sqlite_prefix "\${CMAKE_BINARY_DIR}/third-party/realsense-file/sqlite3-prefix")
ExternalProject_Add(sqlite3
  PREFIX "\${_sdk_sqlite_prefix}"
  SOURCE_DIR "\${_sdk_sqlite_prefix}/src/sqlite3"
  BINARY_DIR "\${_sdk_sqlite_prefix}/src/sqlite3-build"
  DOWNLOAD_DIR "\${_sdk_sqlite_prefix}/src"
  STAMP_DIR "\${_sdk_sqlite_prefix}/src/sqlite3-stamp"
  TMP_DIR "\${_sdk_sqlite_prefix}/tmp"
  INSTALL_DIR "\${_sdk_sqlite_prefix}"
  URL "file://$sqlite_archive"
  URL_HASH SHA256=$SQLITE_SHA256
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "")
EOF

    # BUILD_SHARED_LIBS=OFF links librealsense into the extension, as PyPI's wheel does, so the
    # package needs nothing on the host but its libusb-1.0.
    cmake -S "$SRC" -B "$WORK_DIR/build" \
        -DCMAKE_PROJECT_realsense2_INCLUDE="$WORK_DIR/sqlite-input.cmake" \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_PYTHON_BINDINGS=ON \
        -DPYTHON_EXECUTABLE="$PYTHON" \
        -DPython_EXECUTABLE="$PYTHON" \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=OFF \
        -DBUILD_WITH_DDS=OFF \
        -DCHECK_FOR_UPDATES=OFF \
        -DENABLE_STATS=OFF \
        -DENABLE_AI_ASSISTANT=OFF
    cmake --build "$WORK_DIR/build" --target pyrealsense2 -j "$JOBS"
}

# Lay out Intel's own Python package in $1/package: its __init__.py, its _version.py and the extension
# built for Python $2.
stage_package() {
    local WORK_DIR="$1" PYTHON="$2" SRC="$1/librealsense" suffix
    suffix="$("$PYTHON" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
    mkdir -p "$WORK_DIR/package/pyrealsense2"
    cp "$SRC/wrappers/python/pyrealsense2/__init__.py" "$WORK_DIR/package/pyrealsense2/"
    "$PYTHON" "$SRC/wrappers/python/find_librs_version.py" "$SRC" "$WORK_DIR/package/pyrealsense2" > /dev/null
    [[ -f "$WORK_DIR/build/Release/pyrealsense2$suffix" ]] || fail "the build made no pyrealsense2$suffix"
    cp "$WORK_DIR/build/Release/pyrealsense2$suffix" "$WORK_DIR/package/pyrealsense2/"
}

check_python() {
    local python=/usr/bin/python3 tag
    [[ -x "$python" ]] || fail "no /usr/bin/python3, the camera unit's interpreter"
    tag="$("$python" -c 'import sys; print("cp%d%d" % sys.version_info[:2])')"
    [[ "$tag" == "$TARGET_PYTHON_TAG" ]] ||
        fail "/usr/bin/python3 is $tag, not $TARGET_PYTHON_TAG (Ubuntu 22.04's, which the package is pinned to)"
    "$python" -c 'import sysconfig, os, sys; sys.exit(not os.path.exists(os.path.join(sysconfig.get_paths()["include"], "Python.h")))' ||
        fail "no Python.h for /usr/bin/python3 (sudo apt install python3-dev)"
}

prepare_inputs() (
    # Refuse a missing or corrupt input before any build, source or output exists.
    verify_inputs
    [[ "$EMULATED" == false ]] || check_emulation
    use_offline_git
    local WORK_DIR TREE STAGE SO
    mkdir -p "$WORK_ROOT"
    WORK_DIR="$(mktemp -d "$WORK_ROOT/work.XXXXXX")"
    trap 'rm -rf "$WORK_DIR"' EXIT
    TREE="$WORK_DIR/tree"
    mkdir -p "$TREE"
    if [[ "$EMULATED" == true ]]; then
        with_rootfs "$WORK_DIR" offline build "$INPUTS_DIR" "$TREE" "$(nproc)"
        SO="$TREE/package/pyrealsense2/pyrealsense2.cpython-${TARGET_PYTHON_TAG#cp}-$ARCH-linux-gnu.so"
        [[ -f "$SO" ]] || fail "the build made no $(basename "$SO")"
        check_elf "$SO"
    else
        compile_sdk "$TREE" /usr/bin/python3 "$(nproc)"
        stage_package "$TREE" /usr/bin/python3
        (cd "$TREE/package" && /usr/bin/python3 -B -c 'import pyrealsense2.pyrealsense2') ||
            fail "/usr/bin/python3 cannot load the package it was built for"
    fi

    # The output is assembled beside itself and replaces it only once complete and checked.
    STAGE="$(mktemp -d "$(dirname "$OUT_DIR")/.$(basename "$OUT_DIR").XXXXXX")"
    trap 'rm -rf "$WORK_DIR" "$STAGE"' EXIT
    cp -a "$TREE/package/pyrealsense2" "$STAGE/pyrealsense2"
    echo "$BUILT_MARK" > "$STAGE/BUILT_FROM"
    rm -rf "$OUT_DIR"
    mv "$STAGE" "$OUT_DIR"
    echo "pyrealsense2 $LRS_TAG (RSUSB, $ARCH, $TARGET_PYTHON_TAG): built into $OUT_DIR/"
    echo "Next: host/install.sh <name> <settings.env> --realsense-sdk $OUT_DIR (camera install: the camera block's realsense.sdk_dir)"
)

if (( ${#INNER[@]} )); then
    "inner_${INNER[0]}" "${INNER[@]:1}"
    exit
fi
if [[ "$EMULATED" == false && "$MODE" != download-only ]]; then
    check_python
fi
if [[ "$MODE" != offline-prepare ]]; then
    [[ "$EMULATED" == false ]] || check_emulation
    export GIT_ALLOW_PROTOCOL=https
    acquire_inputs
fi
if [[ "$MODE" != download-only ]]; then
    INPUTS_DIR="$(cd "$INPUTS_DIR" && pwd -P)"
    prepare_inputs
fi
