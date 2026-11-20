#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export PATH=/home/operator/.rosie/go-1.26.2/bin:$PATH
export GOCACHE=$PWD/build/go-cache GOMODCACHE=$PWD/build/go-mod-cache GOTMPDIR=$PWD/build/go-tmp GOFLAGS=-mod=mod
export TMPDIR=/dev/shm/rt-core/remotecell
mkdir -p "$GOCACHE" "$GOMODCACHE" "$GOTMPDIR" "$TMPDIR"
go test -race -count=1 ./sdk/control -run '^TestDelayedAdapterLeaseRenewal$|^TestReviewColdDescribeWarmLease$' -v
