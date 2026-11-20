#!/usr/bin/env bash
# The Deck half of `go run . pendant install|rollback|status` (steamdeck/real/v4
# pendant_deploy.go). That command holds the run journal, uploads the package to
# $APP.incoming/package.tar and sends the Deck this file on stdin (`bash -s`)
# after the variables below. Never in argv (`bash -c`): this text names the
# app's binary, so in a command line it would make the pgrep -f and pkill -f
# patterns here match the installer itself. Not run by hand.
#   ACTION    install | rollback | status | cell (write EXPECT_CELL into
#             $LAUNCHER of an app another cell's manifest installs, nothing
#             else)
#   APP       the app directory (the manifest's pendant_deploy.decks[].app_dir)
#   GAMEID    the Steam shortcut's 64-bit game id; 33554432 when the manifest
#             names no steam_app_id
#   WAIT      seconds each stop or start waits
#   RELAUNCH  1: start the shortcut afterwards if the app was running: the one
#             it was started from, else GAMEID's; none when two were running
#   TAR_SHA, COMMIT  install: package.tar's sha256 and its commit
#   LAUNCHER  the site script in $APP the Steam shortcut runs instead of run.sh
#             (the manifest's launcher); empty when the shortcut runs run.sh
#   EXPECT_CELL  the manifest's cell id: install writes it into $LAUNCHER (or
#             pendant.env without one) as ROSIE_PENDANT_EXPECT_CELL, which
#             run.sh passes to the app as --expect-cell, and refuses a file
#             that names another cell; empty leaves both alone
#   OLP_ORIGIN  the origin of the Deck's tunnel to the cell's OLP
#             (http://127.0.0.1:<pendant_deploy.decks[].tunnel.olp_port>);
#             install writes it beside the cell as ROSIE_PENDANT_OLP_ORIGIN
#             when neither pendant.env nor $LAUNCHER names one, and refuses
#             one that names another origin; empty: no tunnel, left alone
#   APP_OWNER the cell id whose manifest installs $APP when this cell's
#             shortcut runs another cell's install (decks[].app_owner); empty
#             when this manifest installs it
#   IDLE_PY   the cell deploy's OLP idle check (cell_deploy_scripts.go): exits
#             1 while the robot behind the OLP is armed, enabled, leased or
#             running a program
# The package owns the first path component of every SHA256SUMS line (built by
# package.sh), SHA256SUMS, SOURCE_COMMIT and SOURCE_REVISION; install moves only
# those, plus the retired web kiosk's run-qt.sh, so site files (pendant.env,
# olp-cells.json, private/, credentials, sim/) stay. SOURCE_REVISION is the
# commit's first 8 hex digits, the marker hand installs wrote (one Deck
# 60c98c11, another 53f8b256-profiling): kept as a site file it named the
# hand build after an install replaced it. Install stages and checks the package
# before the app is touched (refused: exit 2, nothing changed); after the swap a
# failed check restores the previous package and relaunches it (exit 5). Status
# is read-only. Progress: @@step, @@refused and @@result lines.
set -uo pipefail
export LC_ALL=C
T0=$(date +%s%N)
step(){ printf '@@step %d %s\n' $(( ($(date +%s%N) - T0) / 1000000 )) "$*"; }
refuse(){ printf '@@refused %s\n' "$*"; exit 2; }
[ -n "${APP:-}" ] || { echo "deck-install.sh: no APP; it is the Deck half of go run . pendant (steamdeck/real/v4)" >&2; exit 2; }
PREV="$APP.previous"; INCOMING="$APP.incoming"
APP_BIN=bin/rosie-pendant-v5
RETIRED=run-qt.sh
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
export DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}
# The pendant's OLP as run.sh resolves it: pendant.env's remote origin (sourced
# last, so it wins), else the one the shortcut's launcher exports before it
# execs run.sh, else the local server. With no pendant.env, reading only
# pendant.env made the idle check ask 127.0.0.1:8794, which answers nothing
# when the pendant uses a tunnel, and nothing reads as idle.
origin_in(){ sed -n 's/^\(export[[:space:]]\{1,\}\)\{0,1\}ROSIE_PENDANT_OLP_ORIGIN=//p' "$1" 2> /dev/null | tail -n 1 | tr -d "\"'"; }
o=$(origin_in "$APP/pendant.env"); ORIGIN_FROM=pendant.env
[ -n "$o" ] || [ -z "${LAUNCHER:-}" ] || { o=$(origin_in "$APP/$LAUNCHER"); ORIGIN_FROM=$LAUNCHER; }
SITE_ORIGIN=$o
ORIGIN=${o:-http://127.0.0.1:8794}
app_running(){ pgrep -f 'bin/rosie-pendant-v[5]' > /dev/null 2>&1; }
# The origins running apps were started with (--olp): a Deck with two
# shortcuts runs the same app against another cell's OLP.
app_origins(){ pgrep -af 'bin/rosie-pendant-v[5]' 2> /dev/null | sed -n 's/.* --olp \([^ ]*\).*/\1/p' | sort -u; }
# The game ids of the Steam shortcuts the running apps were started from
# (Steam's reaper: `SteamLaunch AppId=<id> -- $APP/<script>`). Relaunching the
# manifest's shortcut instead would move an app open on another cell's
# shortcut to this cell (a Steam Deck, 2026-10-11: one arm's launcher open,
# the manifest naming another arm's launcher).
shortcut_ids(){
  local a
  pgrep -af 'SteamLaunch AppId=[0-9]' 2> /dev/null | grep -F -- " -- $APP/" | sed -n 's/.*SteamLaunch AppId=\([0-9]\{1,\}\) .*/\1/p' | sort -u |
    while read -r a; do printf '%u\n' $(( (a << 32) | 0x02000000 )); done
}
# The app, run.sh still waiting for its OLP before starting it, or the local
# OLP unit.
running(){ app_running || pgrep -f "${APP##*/}/(\./)?run[.]sh" > /dev/null 2>&1 || systemctl --user -q is-active rosie-v5-olp 2> /dev/null; }
entries(){ sed -n 's|^[0-9a-f]\{64\}  \./\([^/]*\).*|\1|p' "$1/SHA256SUMS" | sort -u; }
# The cell the shortcut's launch is for (2026-10-11: a hidden pendant for
# one arm armed it with presses meant for another arm's): the launcher's
# ROSIE_PENDANT_EXPECT_CELL, or pendant.env's when the shortcut runs run.sh.
cell_file(){ if [ -n "${LAUNCHER:-}" ]; then echo "$APP/$LAUNCHER"; else echo "$APP/pendant.env"; fi; }
cell_in(){ sed -n 's/^\(export[[:space:]]\{1,\}\)\{0,1\}ROSIE_PENDANT_EXPECT_CELL=//p' "$1" 2> /dev/null | tail -n 1 | tr -d "\"'"; }
pendant_cell_check(){ # before anything changes
  if [ -n "${OLP_ORIGIN:-}" ] && [ -n "$SITE_ORIGIN" ] && [ "$SITE_ORIGIN" != "$OLP_ORIGIN" ]; then
    refuse "$ORIGIN_FROM points the pendant at $SITE_ORIGIN, but the manifest's tunnel for this Deck forwards $OLP_ORIGIN: correct pendant_deploy.decks[].tunnel.olp_port or that file (nothing changed)"
  fi
  [ -n "${EXPECT_CELL:-}" ] || return 0
  local f c
  f=$(cell_file); c=$(cell_in "$f")
  [ -z "$c" ] || [ "$c" = "$EXPECT_CELL" ] || refuse "${f##*/} starts the pendant for cell $c, but this install is for $EXPECT_CELL: correct the manifest's pendant_deploy.decks[].launcher or that file (nothing changed)"
}
# Sets $2=$3 in site file $1: an export after a launcher's shebang, before
# anything that runs run.sh (written in place, so the file keeps its mode), or
# a line appended to pendant.env. The file as it was is kept in $W for
# pendant_restore (SITE_SAVED).
site_set(){ # $1 file, $2 name, $3 value
  if [ "${SITE_SAVED:-0}" = 0 ]; then
    if [ -e "$1" ]; then cp -p -- "$1" "$W/site-before" || return 1; SITE_SAVED=1; else SITE_SAVED=absent; fi
  fi
  if [ -n "${LAUNCHER:-}" ]; then
    if head -n 1 "$1" | grep -q '^#!'; then
      { head -n 1 "$1"; echo "export $2=$3"; tail -n +2 "$1"; } > "$W/launcher" || return 1
    else
      { echo "export $2=$3"; cat "$1"; } > "$W/launcher" || return 1
    fi
    cat "$W/launcher" > "$1"
  else
    printf '%s=%s\n' "$2" "$3" >> "$1"
  fi
}
# Puts the site file back as it was before site_set (a restored package).
site_restore(){
  local f
  f=$(cell_file)
  ORIGIN=${SITE_ORIGIN:-http://127.0.0.1:8794}
  case ${SITE_SAVED:-0} in
    1) cat "$W/site-before" > "$f" && step "pendant ${f##*/} put back as it was before this install" ;;
    absent) rm -f -- "$f" && step "pendant ${f##*/} removed: this install wrote it" ;;
  esac
}
pendant_cell_write(){ # after the new package verified, before the relaunch; sets CELL_WRITTEN
  CELL_WRITTEN=0
  [ -n "${EXPECT_CELL:-}" ] || [ -n "${OLP_ORIGIN:-}" ] || return 0
  local f
  f=$(cell_file)
  if [ -n "${LAUNCHER:-}" ] && [ ! -f "$f" ]; then
    step "pendant WARNING: launcher $f is missing; its shortcut cannot start, and no cell or origin was written"; return 0
  fi
  # The origin first: the cell then lands right after a launcher's shebang.
  if [ -n "${OLP_ORIGIN:-}" ]; then
    if [ "$SITE_ORIGIN" = "$OLP_ORIGIN" ]; then
      step "pendant $ORIGIN_FROM points the pendant at $OLP_ORIGIN, the Deck's tunnel"
    else
      site_set "$f" ROSIE_PENDANT_OLP_ORIGIN "$OLP_ORIGIN" || return 1
      ORIGIN=$OLP_ORIGIN
      step "pendant ${f##*/} now points the pendant at $OLP_ORIGIN, the Deck's tunnel to the cell's OLP (pendant tunnel), instead of the Deck-local server"
    fi
  fi
  [ -n "${EXPECT_CELL:-}" ] || return 0
  if [ "$(cell_in "$f")" = "$EXPECT_CELL" ]; then step "pendant ${f##*/} names cell $EXPECT_CELL"; return 0; fi
  site_set "$f" ROSIE_PENDANT_EXPECT_CELL "$EXPECT_CELL" || return 1
  CELL_WRITTEN=1
  step "pendant ${f##*/} now names cell $EXPECT_CELL (run.sh passes it to the app as --expect-cell)"
}
# The cells running apps check (--expect-cell; none: started without one).
app_cells(){ pgrep -af 'bin/rosie-pendant-v[5]' 2> /dev/null | sed -n 's/.* --expect-cell \([^ ]*\).*/\1/p' | sort -u; }
# Running apps against this launcher's OLP that do not check this cell: they
# started before the cell was written (it applies from their next start).
stale_cell_apps(){ pgrep -af 'bin/rosie-pendant-v[5]' 2> /dev/null | grep -F -- " --olp $ORIGIN" | grep -vF -- " --expect-cell $EXPECT_CELL" | cut -d' ' -f1 | paste -sd' ' -; }
# Pendants running from another directory than $APP: a hand-made copy this
# command does not update (a hand install beside the managed one).
other_apps(){
  local p exe
  for p in $(pgrep -f 'bin/rosie-pendant-v[5]' 2> /dev/null); do
    exe=$(tr '\0' ' ' < "/proc/$p/cmdline" 2> /dev/null | cut -d' ' -f1)
    case $exe in "$APP"/*|"") ;; *) printf '%s(pid %s) ' "$exe" "$p" ;; esac
  done
}
site_files(){ (cd "$1" 2> /dev/null && ls -A | grep -vxF -f <(entries "$1" 2> /dev/null; echo SHA256SUMS; echo SOURCE_COMMIT; echo SOURCE_REVISION) | tr '\n' ' '); }

# Refuses while the robot behind the pendant's OLP is in use: closing the
# pendant sends that OLP Stop.
idle_origins(){
  local apps origins
  apps=$(pgrep -af 'bin/rosie-pendant-v[5]' 2> /dev/null | grep -F '/bin/rosie-pendant-v')
  origins=$(app_origins)
  set -- $(shortcut_ids)
  # A shared app may be open on the other cell's shortcut. When exactly one
  # managed app and its Steam shortcut are known, installation closes and
  # relaunches that app only. An inactive owner's shortcut is unaffected.
  # Keep the conservative fallback for ambiguous/multiple apps, a missing
  # shortcut identity, or a local OLP service that stopping may also affect.
  if [ "$(printf '%s\n' "$apps" | grep -c .)" = 1 ] &&
     [ "$(printf '%s\n' "$origins" | grep -c .)" = 1 ] && [ $# = 1 ] &&
     ! systemctl --user -q is-active rosie-v5-olp 2> /dev/null; then
    case ${apps#* } in "$APP/$APP_BIN "*) printf '%s\n' "$origins"; return ;; esac
  fi
  (echo "$ORIGIN"; app_origins) | sort -u
}
# Only a running pendant (or run.sh, or the Deck-local OLP) can send a robot
# Stop when install closes it. With none running nothing here reaches a robot,
# so install does not wait for one to be idle: one Deck on 2026-10-19 was
# refused because an arm ran a loop from another Deck while no pendant was
# open on the first. pendant_stop refuses if one starts after this check.
pendant_idle(){ # sets IDLE_CHECKED
  local o
  IDLE_CHECKED=0
  if ! running; then
    step "pendant not running on this Deck: install closes nothing, so no robot is checked or stopped"
    return 0
  fi
  IDLE_CHECKED=1
  for o in $(idle_origins); do
    curl -fsS -m 5 "$o/api/offline-programming/v1/dense-execution/status" > "$W/status.json" 2> /dev/null || : > "$W/status.json"
    python3 -c "$IDLE_PY" "$W/status.json" || refuse "the robot behind the pendant's OLP ($o) is in use: finish, Stop, disarm and release first"
  done
}
pendant_stop(){
  WAS_RUNNING=0; RELAUNCH_IDS=""
  running || return 0
  [ "${IDLE_CHECKED:-0}" = 1 ] || refuse "a pendant started on this Deck during the install, after the idle check: its robot was not checked (nothing changed); rerun"
  WAS_RUNNING=1
  RELAUNCH_IDS=$(shortcut_ids | tr '\n' ' ')
  step "pendant stopping the running app (checked OLPs $(idle_origins | tr '\n' ' '); the robot is idle; started from shortcut ${RELAUNCH_IDS:-unknown})"
  pkill -TERM -f 'bin/rosie-pendant-v[5]' 2> /dev/null
  for i in $(seq 1 "$WAIT"); do running || break; sleep 1; done
  systemctl --user stop rosie-v5-olp 2> /dev/null
  ! running || refuse "the pendant did not exit within $WAIT s; close it on the Deck"
}
pendant_verify(){ # $1 tree; sets HEALTH_FAIL
  (cd "$1" && sha256sum --quiet -c SHA256SUMS > "$W/sums.txt" 2>&1) || { HEALTH_FAIL="SHA256SUMS mismatch in $1: $(head -n 3 "$W/sums.txt" | tr '\n' ' ')"; return 1; }
  if LD_LIBRARY_PATH="$1/lib" ldd "$1/$APP_BIN" 2>&1 | grep -q 'not found'; then HEALTH_FAIL="unresolved libraries: $(LD_LIBRARY_PATH="$1/lib" ldd "$1/$APP_BIN" | grep 'not found' | tr '\n' ' ')"; return 1; fi
  step "pendant $1: $(wc -l < "$1/SHA256SUMS") hashes match, libraries resolve"
}
pendant_launch(){ # sets HEALTH_FAIL
  [ "$RELAUNCH" = 1 ] || { step "pendant relaunch skipped (--no-relaunch)"; return 0; }
  local g=$GAMEID
  set -- ${RELAUNCH_IDS:-}
  if [ $# -gt 1 ]; then step "pendant $# shortcuts were running ($*): not relaunched, start one on the Deck"; return 0; fi
  if [ $# = 1 ]; then
    g=$1
    [ "$g" = "$GAMEID" ] || step "pendant relaunching the shortcut the app was started from ($g), not the manifest's ($GAMEID)"
  fi
  [ "$g" != 33554432 ] || { step "pendant no steam_app_id in the manifest: start the shortcut on the Deck"; return 0; }
  steam "steam://rungameid/$g" > /dev/null 2>&1 &
  for i in $(seq 1 "$WAIT"); do app_running && break; sleep 1; done
  app_running || { HEALTH_FAIL="the app did not start within $WAIT s of steam://rungameid/$g"; return 1; }
  local o
  for i in $(seq 1 "$WAIT"); do
    o=$(app_origins | head -n 1); o=${o:-$ORIGIN}
    if curl -fsS -m 2 -o /dev/null "$o/api/offline-programming/v1/health" 2> /dev/null; then step "pendant running; its OLP $o health ok"; return 0; fi
    sleep 1
  done
  HEALTH_FAIL="the pendant's OLP $o health not ok within $WAIT s"; return 1
}
# Moves the package-owned entries from $1 to $2. The tesseract pixi
# environment (not hashed, ~1 GB) follows the app when the incoming tree has
# none.
pendant_move(){ # $1 from, $2 to
  local e
  [ -f "$1/SHA256SUMS" ] || return 0
  mkdir -p "$2" || return 1
  for e in $(entries "$1") SHA256SUMS SOURCE_COMMIT SOURCE_REVISION $RETIRED; do
    [ -e "$1/$e" ] || continue
    rm -rf -- "$2/$e" && mv -- "$1/$e" "$2/$e" || return 1
  done
}
pendant_carry_env(){ # $1 from, $2 to
  if [ -d "$1/olp/tesseract/v1/.pixi" ] && [ ! -e "$2/olp/tesseract/v1/.pixi" ] && [ -d "$2/olp" ]; then
    mkdir -p "$2/olp/tesseract/v1" && mv -- "$1/olp/tesseract/v1/.pixi" "$2/olp/tesseract/v1/.pixi" && step "pendant tesseract env carried from $1"
  fi
}
quote_reason(){ printf '%s' "$1" | tr '"\\' "''"; }

pendant_restore(){ # $1 why; exits 5
  step "pendant RESTORE: $1"
  rm -rf -- "$PREV.failed"; pendant_carry_env "$APP" "$PREV"; pendant_move "$APP" "$PREV.failed"
  pendant_move "$PREV" "$APP" && rm -rf -- "$PREV" && { [ ! -e "$PREV.old" ] || mv -- "$PREV.old" "$PREV"; }
  # The previous package relaunches as it ran: against the OLP its site file
  # named before this install wrote one.
  site_restore
  HEALTH_FAIL=""
  if pendant_verify "$APP"; then [ "$WAS_RUNNING" = 1 ] && pendant_launch; else step "pendant restored tree fails its checks: $HEALTH_FAIL; reinstall by hand"; fi
  printf '@@result {"status":"restored","reason":"%s"}\n' "$(quote_reason "$1")"
  exit 5
}

pendant_install(){
  [ -d "$APP" ] || refuse "no app directory $APP: a Deck's first install, its Steam shortcut and site files are provisioned by hand (steamdeck/real/v5/README.md)"
  [ "$(sha256sum < "$INCOMING/package.tar" | cut -d' ' -f1)" = "$TAR_SHA" ] || refuse "the uploaded package does not have sha256 $TAR_SHA"
  step "pendant install $COMMIT into $APP (now $(cat "$APP/SOURCE_COMMIT" 2> /dev/null || echo unknown)); OLP $ORIGIN"
  pendant_cell_check
  pendant_idle
  rm -rf -- "$INCOMING/tree" && mkdir -p "$INCOMING/tree" && tar -C "$INCOMING/tree" -xf "$INCOMING/package.tar" || refuse "cannot extract the package in $INCOMING"
  printf '%s\n' "$COMMIT" > "$INCOMING/tree/SOURCE_COMMIT"
  printf '%s\n' "${COMMIT:0:8}" > "$INCOMING/tree/SOURCE_REVISION"
  HEALTH_FAIL=""
  pendant_verify "$INCOMING/tree" || refuse "staged package: $HEALTH_FAIL (app untouched)"
  pendant_stop
  rm -rf -- "$PREV.old"; [ ! -e "$PREV" ] || mv -- "$PREV" "$PREV.old" || refuse "cannot keep $PREV"
  pendant_move "$APP" "$PREV" || pendant_restore "cannot move the current package aside"
  pendant_move "$INCOMING/tree" "$APP" || pendant_restore "cannot move the new package into $APP"
  pendant_carry_env "$PREV" "$APP"
  [ ! -e "$PREV/$RETIRED" ] || step "pendant the retired kiosk's $RETIRED moved to $PREV"
  step "pendant site files kept: $(site_files "$APP")"
  pendant_verify "$APP" || pendant_restore "$HEALTH_FAIL"
  pendant_cell_write || pendant_restore "cannot write the cell into $(cell_file)"
  o=$(other_apps); [ -z "$o" ] || step "pendant WARNING: pendants running from another directory, not updated by this install: $o(the next launch of this one closes them; retire that copy)"
  if [ "$WAS_RUNNING" = 1 ]; then pendant_launch || pendant_restore "$HEALTH_FAIL"; else step "pendant was not running; not started"; fi
  rm -rf -- "$PREV.old" "$INCOMING"
  printf '@@result {"status":"ok","commit":"%s","previous":"%s"}\n' "$COMMIT" "$(cat "$PREV/SOURCE_COMMIT" 2> /dev/null || echo unknown)"
}

# Swaps the current package with the one install kept in $APP.previous.
pendant_rollback(){
  [ -f "$PREV/SHA256SUMS" ] || refuse "no previous package in $PREV"
  HEALTH_FAIL=""
  pendant_verify "$PREV" || refuse "the previous package fails its checks: $HEALTH_FAIL"
  step "pendant rollback $(cat "$APP/SOURCE_COMMIT" 2> /dev/null || echo unknown) -> $(cat "$PREV/SOURCE_COMMIT" 2> /dev/null || echo unknown)"
  pendant_idle
  pendant_stop
  rm -rf -- "$PREV.swap"
  pendant_move "$APP" "$PREV.swap" && pendant_move "$PREV" "$APP" && pendant_carry_env "$PREV.swap" "$APP" && rm -rf -- "$PREV" && mv -- "$PREV.swap" "$PREV" \
    || { printf '@@result {"status":"failed","reason":"swap failed; %s and %s need a look by hand"}\n' "$APP" "$PREV.swap"; exit 1; }
  pendant_verify "$APP" && { [ "$WAS_RUNNING" = 0 ] || pendant_launch; } || { printf '@@result {"status":"failed","reason":"%s"}\n' "$(quote_reason "$HEALTH_FAIL")"; exit 1; }
  printf '@@result {"status":"ok","commit":"%s"}\n' "$(cat "$APP/SOURCE_COMMIT" 2> /dev/null || echo unknown)"
}

# This cell's launcher of an app $APP_OWNER's manifest installs: one app per
# Deck, one launcher per cell. Writes only the launcher's cell: the package,
# pendant.env and the running app are the owner install's, so the two
# manifests never take turns swapping one app directory or closing its pendant.
pendant_cell(){
  [ -f "$APP/SHA256SUMS" ] || refuse "no installed app in $APP: ${APP_OWNER:-the owning cell}'s manifest installs it (pendant install --manifest <that cell>)"
  [ -n "${LAUNCHER:-}" ] || refuse "no launcher: a shared app's pendant.env is every shortcut's, so the cell goes into this cell's launcher (pendant_deploy.decks[].launcher)"
  local have
  have=$(cat "$APP/SOURCE_COMMIT" 2> /dev/null || echo unknown)
  [ -z "${COMMIT:-}" ] || [ "$have" = "$COMMIT" ] || refuse "$APP runs $have, not $COMMIT: install $COMMIT with ${APP_OWNER:-the owning cell}'s manifest first (nothing changed)"
  step "pendant $APP at $have, installed by ${APP_OWNER:-another cell}'s manifest; this cell's launcher $LAUNCHER, OLP $ORIGIN"
  pendant_cell_check
  [ -f "$APP/$LAUNCHER" ] || refuse "launcher $APP/$LAUNCHER is missing: its Steam shortcut cannot start (site file, provisioned by hand)"
  pendant_cell_write || { printf '@@result {"status":"failed","reason":"cannot write the cell into %s"}\n' "$APP/$LAUNCHER"; exit 1; }
  local s
  s=$(stale_cell_apps)
  [ -z "$s" ] || step "pendant WARNING: pendant(s) pid $s against $ORIGIN started without --expect-cell $EXPECT_CELL: the cell check applies from their next start (close and reopen the shortcut when the robot is idle)"
  printf '@@result {"status":"ok","cell":"%s","launcher":"%s","commit":"%s","written":%s}\n' "$EXPECT_CELL" "$LAUNCHER" "$have" "$( [ "$CELL_WRITTEN" = 1 ] && echo true || echo false)"
}

pendant_status(){
  echo "app $APP commit $(cat "$APP/SOURCE_COMMIT" 2> /dev/null || echo unknown) files $(wc -l < "$APP/SHA256SUMS" 2> /dev/null || echo 0)"
  # A package installed before SOURCE_COMMIT existed is still a rollback target.
  if [ -f "$PREV/SHA256SUMS" ]; then echo "previous $(cat "$PREV/SOURCE_COMMIT" 2> /dev/null || echo unknown) files $(wc -l < "$PREV/SHA256SUMS" | tr -d ' ')"; else echo "previous none"; fi
  echo "site files $(site_files "$APP")"
  echo "cell $(cell_in "$(cell_file)" | grep . || echo none) in $(basename "$(cell_file)")${EXPECT_CELL:+ (manifest $EXPECT_CELL)}${APP_OWNER:+; app installed by the $APP_OWNER manifest}"
  if running; then echo "running yes $(app_origins | tr '\n' ' ')cells $(app_cells | tr '\n' ' ' | grep . || echo 'none ')shortcuts $(shortcut_ids | tr '\n' ' ')"; else echo "running no"; fi
  o=$(other_apps); [ -z "$o" ] || echo "other app directories running: $o"
  echo "olp $ORIGIN health $(curl -sS -m 3 -o /dev/null -w '%{http_code}' "$ORIGIN/api/offline-programming/v1/health" 2> /dev/null)${OLP_ORIGIN:+ (manifest tunnel $OLP_ORIGIN)}"
  echo "free_mib $(df -Pm "$APP" 2> /dev/null | awk 'NR == 2 { print $4 }')"
}

# One compound command, read whole before it runs, with stdin from /dev/null:
# nothing below can read the rest of the script as input.
case ${ACTION:-} in
  install|rollback|cell)
    trap '' HUP PIPE
    W=$(mktemp -d) || refuse "mktemp failed"
    trap 'rm -rf -- "$W"' EXIT
    RELAUNCH=${RELAUNCH:-1}
    "pendant_$ACTION" ;;
  status) pendant_status ;;
  *) echo "deck-install.sh: ACTION must be install, rollback, cell or status; it is the Deck half of go run . pendant (steamdeck/real/v4)" >&2; exit 2 ;;
esac < /dev/null
