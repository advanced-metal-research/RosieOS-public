package build

import "strings"

// SSHBuildPeerCacheScript serializes cold builds by immutable artifact
// identity. Concurrent requesters wait, recheck, and consume the first atomic
// publication instead of duplicating the build.
func SSHBuildPeerCacheScript(cacheRoot, identity, artifactName, buildCommand string) string {
	// The caller always streams the source archive on stdin. A cache-hit branch
	// must consume it before returning the binary: otherwise the SSH channel can
	// deadlock with the client writing stdin while the peer writes stdout.
	cacheHit := func(kind string) string {
		return `if test -x "$BINARY"; then cat >/dev/null; echo "robot-v4 ssh build cache: ` + kind + ` artifact=` + artifactName + ` identity=` + identity + `" >&2; printf 'cache_hit=1\n' >&2; base64 "$BINARY"; exit 0; fi`
	}
	return strings.Join([]string{
		"set -eu",
		`CACHE_ROOT="` + cacheRoot + `"`,
		`CACHE="$CACHE_ROOT/` + identity + `"`,
		`BINARY="$CACHE/` + artifactName + `"`,
		`mkdir -p "$CACHE_ROOT"`,
		cacheHit("hit"),
		`LOCK="$CACHE_ROOT/.lock-` + identity + `"`,
		`command -v flock >/dev/null 2>&1 || { echo "robot-v4 ssh build cache: flock is required on the build peer" >&2; exit 1; }`,
		`exec 9>"$LOCK"`,
		`flock 9`,
		cacheHit("hit-after-wait"),
		`echo "robot-v4 ssh build cache: miss artifact=` + artifactName + ` identity=` + identity + `" >&2`,
		`printf 'cache_hit=0\n' >&2`,
		`WORK=$(mktemp -d "$CACHE_ROOT/.build.` + identity + `.XXXXXX")`,
		`trap 'rm -rf "$WORK"' EXIT`,
		`base64 -d > "$WORK/source.tgz"`,
		`tar -xzf "$WORK/source.tgz" -C "$WORK"`,
		`export DAEMON_V1_BUILD_WORKSPACE="$WORK"`,
		`export DAEMON_V1_BUILD_OUTPUT="$WORK/` + artifactName + `"`,
		`cd "$WORK"`,
		`(` + buildCommand + `) >&2`,
		`test -x "$DAEMON_V1_BUILD_OUTPUT"`,
		`PUBLISH="$WORK/publish"`,
		`mkdir "$PUBLISH"`,
		`install -m 0755 "$DAEMON_V1_BUILD_OUTPUT" "$PUBLISH/` + artifactName + `"`,
		`if test -e "$CACHE"; then echo "robot-v4 ssh build cache: recovering incomplete publication artifact=` + artifactName + ` identity=` + identity + `" >&2; rm -rf "$CACHE"; fi`,
		`mv "$PUBLISH" "$CACHE"`,
		`base64 "$BINARY"`,
	}, "; ")
}
