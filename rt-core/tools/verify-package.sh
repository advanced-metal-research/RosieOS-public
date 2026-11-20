#!/usr/bin/env bash
# Verify payload hashes and definition-bound factory resources with the standalone inspector.
set -euo pipefail
[[ $# == 1 ]] || { printf 'usage: verify-package.sh PACKAGE_DIR\n' >&2; exit 2; }
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
inspector="${component}/bin/rt-package"
if [[ ! -x "${inspector}" ]]; then inspector="${component}/build/rt-package"; fi
[[ -x "${inspector}" ]] || { printf 'package inspector missing\n' >&2; exit 3; }
exec "${inspector}" verify "$1"
