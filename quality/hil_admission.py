"""Run one read-only, cell-local servo-bench admission rehearsal.

This command does not grant motion authority or qualify a candidate. It checks
one manual Fleet reservation and the installed, disarmed RTCore pair. Its only
physical scenario reads state twice. Host-write assurance stays cooperative.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener


ROOT = Path(__file__).resolve().parent.parent
SCENARIO = "installed-identity-observation"
RESOURCE_CAPABILITY = "motor-testbench"
MAX_OUTPUT = 1024 * 1024


def sha256(data: bytes) -> str:
    """Return the lowercase SHA-256 digest of bytes."""
    return hashlib.sha256(data).hexdigest()


def run(program: str, args: list[str], *, timeout: int = 20,
        environment: dict[str, str] | None = None) -> dict:
    """Run a fixed observation command and retain its result and digest."""
    started = datetime.now(timezone.utc).isoformat()
    try:
        result = subprocess.run(
            [program, *args], cwd=ROOT, capture_output=True, timeout=timeout,
            check=False, env=environment,
        )
        stdout, stderr, code = result.stdout, result.stderr, result.returncode
    except (OSError, subprocess.TimeoutExpired) as error:
        stdout, stderr, code = b"", str(error).encode(), -1
    if len(stdout) > MAX_OUTPUT or len(stderr) > MAX_OUTPUT:
        raise ValueError("observation output exceeds 1 MiB")
    return {
        "program": program,
        "arguments": args,
        "startedAt": started,
        "finishedAt": datetime.now(timezone.utc).isoformat(),
        "exitCode": code,
        "stdoutSha256": sha256(stdout),
        "stderrSha256": sha256(stderr),
        "stdout": stdout.decode("utf-8", errors="replace"),
        "stderr": stderr.decode("utf-8", errors="replace"),
    }


def checked_json(observation: dict, name: str) -> dict:
    """Decode a successful JSON observation or fail closed."""
    if observation["exitCode"] != 0:
        raise ValueError(f"{name} failed: {observation['stderr'][-300:]}")
    value = json.loads(observation["stdout"])
    if not isinstance(value, dict):
        raise ValueError(f"{name} is not a JSON object")
    return value


class NoRedirect(HTTPRedirectHandler):
    """Keep a bearer token on the configured Fleet origin."""

    def redirect_request(self, request, fp, code, msg, headers, newurl):
        """Reject every redirect, including an HTTPS redirect."""
        raise ValueError("Fleet snapshot redirected")


def fleet_snapshot(url: str, token: str) -> dict:
    """Read a fresh snapshot with a snapshot-only Fleet credential."""
    parsed = urlsplit(url)
    if (parsed.scheme != "https" or not parsed.hostname or parsed.username or
            parsed.password or parsed.path != "/api/snapshot" or
            parsed.query or parsed.fragment):
        raise ValueError("Fleet URL must be an exact HTTPS /api/snapshot URL")
    request = Request(url, headers={
        "Authorization": f"Bearer {token}",
        "User-Agent": "RosieOS-Qualification/1.0",
    })
    with build_opener(NoRedirect()).open(request, timeout=5) as response:
        raw = response.read(MAX_OUTPUT + 1)
    if len(raw) > MAX_OUTPUT:
        raise ValueError("Fleet snapshot exceeds 1 MiB")
    snapshot = json.loads(raw)
    if not isinstance(snapshot, dict):
        raise ValueError("Fleet snapshot is not a JSON object")
    return snapshot


def parse_instant(value: object) -> datetime:
    """Parse an offset-aware RFC 3339 timestamp."""
    if not isinstance(value, str):
        raise ValueError("timestamp is missing")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise ValueError("timestamp has no time zone")
    return parsed.astimezone(timezone.utc)


def reservation_errors(snapshot: dict, job_id: str, relay_peer: str,
                       now: datetime) -> list[str]:
    """Check that one current manual hold covers the whole declared bench."""
    errors = []
    if (snapshot.get("schema") != "rosie.fleet-coordination.snapshot.v1" or
            snapshot.get("authorityCeiling") != "coordination_only"):
        return ["fleet_snapshot_contract"]
    try:
        age = (now - parse_instant(snapshot.get("generatedAt"))).total_seconds()
    except ValueError:
        return ["fleet_snapshot_time_unknown"]
    if age < -5 or age > 30:
        errors.append("fleet_snapshot_stale")
    if any(not isinstance(snapshot.get(key), list) for key in
           ("resources", "jobs", "assignments", "holds")):
        return errors + ["fleet_snapshot_shape_unknown"]
    resources = [resource for resource in snapshot.get("resources", [])
                 if isinstance(resource, dict) and
                 resource.get("node_id") == relay_peer and
                 resource.get("capability_id") == RESOURCE_CAPABILITY]
    if len(resources) != 1:
        return errors + ["bench_resource_not_unique"]
    resource_id = resources[0].get("id")
    jobs = [job for job in snapshot.get("jobs", [])
            if isinstance(job, dict) and job.get("id") == job_id]
    if len(jobs) != 1 or jobs[0].get("status") != "reserved":
        errors.append("manual_job_not_reserved")
    assignments = [item for item in snapshot.get("assignments", [])
                   if isinstance(item, dict) and item.get("job_id") == job_id and
                   item.get("resource_instance_id") == resource_id]
    if len(assignments) != 1:
        errors.append("bench_not_assigned_to_job")
    holds = [hold for hold in snapshot.get("holds", [])
             if isinstance(hold, dict) and hold.get("resource_instance_id") == resource_id]
    if len(holds) != 1 or holds[0].get("job_id") != job_id:
        return errors + ["bench_hold_not_exclusive"]
    try:
        start = parse_instant(holds[0].get("starts_at"))
        end = parse_instant(holds[0].get("ends_at"))
        if start > now or end <= now:
            errors.append("bench_hold_not_current")
        if (end - now).total_seconds() < 60:
            errors.append("bench_hold_ends_too_soon")
    except ValueError:
        errors.append("bench_hold_time_unknown")
    return errors


def safe_state_errors(status: dict, description: dict, axes: int,
                      expected_config: str, expected_slaves: int) -> list[str]:
    """Require disarmed native outputs and matching live configuration."""
    core = status.get("core")
    if not isinstance(core, dict) or not isinstance(description, dict):
        return ["rt_status_unknown"]
    errors = []
    if (core.get("backend") != "ethercat" or
            not re.fullmatch(r"[0-9a-f]{64}", str(core.get("configuration_sha256"))) or
            core.get("configuration_sha256") != description.get("configuration_sha256") or
            core.get("configuration_sha256") != expected_config or
            description.get("backend") != "ethercat"):
        errors.append("configuration_identity_mismatch")
    if (not isinstance(core.get("axes"), list) or
            not isinstance(description.get("axes"), list) or
            len(core["axes"]) != axes or len(description["axes"]) != axes):
        errors.append("live_axis_count_mismatch")
    if core.get("config_verified_mask") != (1 << axes) - 1:
        errors.append("drive_configuration_unverified")
    if (type(core.get("time_ns")) is not int or core["time_ns"] <= 0 or
            type(status.get("time_ns")) is not int or status["time_ns"] <= 0):
        errors.append("controller_observation_time_unknown")
    if (not isinstance(status.get("daemon_incarnation"), str) or
            not status["daemon_incarnation"] or
            not isinstance(status.get("adapter_incarnation"), str) or
            not status["adapter_incarnation"]):
        errors.append("controller_incarnation_unknown")
    bus = core.get("bus")
    if (not isinstance(bus, dict) or bus.get("admitted") is not True or
            bus.get("reason") != "none"):
        errors.append("ethercat_health_unknown")
    else:
        held = bus.get("held_slaves")
        expected = bus.get("responding_expected")
        if (type(expected) is not int or expected != expected_slaves or
                not isinstance(held, list) or expected != axes + len(held)):
            errors.append("held_bus_state_unsafe_or_unknown")
        else:
            positions = set()
            for slave in held:
                if not isinstance(slave, dict):
                    errors.append("held_bus_state_unsafe_or_unknown")
                    break
                position = slave.get("slave_position")
                if (type(position) is not int or position < 0 or position in positions or
                        slave.get("disposition") not in {"held", "declared_unused"} or
                        slave.get("present") is not True or
                        type(slave.get("al_state")) is not int or slave["al_state"] not in {1, 2} or
                        any(type(slave.get(field)) is not int or slave[field] < 0
                            for field in ("vendor_id", "product_code", "revision"))):
                    errors.append("held_bus_state_unsafe_or_unknown")
                    break
                positions.add(position)
            if held and (bus.get("unknown_slaves") != "hold" or
                         bus.get("unused_change_policy") != "log_and_continue_axes"):
                errors.append("held_bus_state_unsafe_or_unknown")
    for field in ("armed", "axis_enable_mask", "lease_valid", "safety_fault_mask",
                  "execution_fault_reasons", "commissioning_phase"):
        if type(core.get(field)) is not int or core[field] != 0:
            errors.append("unsafe_or_unknown_" + field)
    grant = status.get("grant")
    if (not isinstance(grant, dict) or
            type(grant.get("GrantActive")) is not int or grant["GrantActive"] != 0):
        errors.append("control_grant_active_or_unknown")
    motion = status.get("motion")
    if (not isinstance(motion, dict) or
            type(motion.get("queue_depth")) is not int or motion["queue_depth"] != 0 or
            motion.get("done") is not True):
        errors.append("motion_pending_or_unknown")
    io = core.get("io")
    if not isinstance(io, dict) or io.get("armed") is not False or not isinstance(io.get("outputs"), list):
        errors.append("io_state_unknown")
    elif any(not isinstance(output, dict) or output.get("intent") is not False or
             output.get("commanded") is not False or output.get("valid") is not True or
             output.get("readback") is not False for output in io["outputs"]):
        errors.append("io_output_active_or_unknown")
    return errors


def pair_from_cell(path: Path) -> tuple[str, dict, int, str, str, str, int]:
    """Read paired hosts, axes, expected slaves, and identity from a cell."""
    if (path.is_absolute() or ".." in path.parts or
            len(path.parts) < 4 or path.parts[:3] != ("rt-core", "config", "cells") or
            path.suffix != ".json"):
        raise ValueError("cell must be a repository cell config")
    cell = json.loads((ROOT / path).read_text())
    relay = [node for node in cell.get("nodes", []) if node.get("role") == "motion-server"]
    servo = [node for node in cell.get("nodes", []) if node.get("role") == "realtime-core"]
    if len(relay) != 1 or len(servo) != 1:
        raise ValueError("cell needs one motion and one RTCore peer")
    peer = servo[0]
    axes = len(cell.get("robot_axes", {}).get("axes", []))
    if not 1 <= axes <= 16:
        raise ValueError("cell axis count is invalid")
    for value in (relay[0].get("peer"), peer.get("peer"), peer.get("ssh_host"),
                  peer.get("remote_user")):
        if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.:%_-]*", value):
            raise ValueError("cell peer address is invalid")
    target = f"{relay[0]['peer']}-{peer['peer']}".lower()
    rosie_home = relay[0].get("rosie_home")
    if (not isinstance(rosie_home, str) or
            not re.fullmatch(r"/home/[A-Za-z0-9_-]+/\.rosie", rosie_home)):
        raise ValueError("motion peer RosieOS home is invalid")
    config_pin = peer.get("rt_core", {}).get("configuration_sha256")
    if not isinstance(config_pin, str) or not re.fullmatch(r"[0-9a-f]{64}", config_pin):
        raise ValueError("cell configuration identity pin is missing")
    expected_slaves = peer.get("revpi_setup", {}).get("ethercat_foundation", {}).get("expected_slaves")
    if type(expected_slaves) is not int or expected_slaves < axes:
        raise ValueError("cell expected slave count is invalid")
    return (relay[0]["peer"], peer, axes, target,
            str(Path(rosie_home) / "bin/node"), config_pin, expected_slaves)


def relay_boot_id() -> str:
    """Read the local Linux boot identity for the rehearsal receipt."""
    value = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if not re.fullmatch(r"[0-9a-f-]{36}", value):
        raise ValueError("relay boot identity is invalid")
    return value


def remote(servo: dict, command: str) -> dict:
    """Run one fixed read-only command on the paired servo host."""
    return run("ssh", ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                       "-o", "ConnectTimeout=5", "-l", servo["remote_user"],
                       servo["ssh_host"], command])


def observe_servo(servo: dict, evidence: list[dict]) -> dict:
    """Read the installed package, live controller, and final-safe state."""
    commands = {
        "identity": "set -e; hostname -s; cat /proc/sys/kernel/random/boot_id; readlink -f /opt/rosie-rt-core/current; sudo -n sha256sum /opt/rosie-rt-core/current/rt_package.json",
        "package": "sudo -n /opt/rosie-rt-core/current/tools/verify-package.sh \"$(readlink -f /opt/rosie-rt-core/current)\"",
        "units": "for u in rosie-rt-core.service rosie-rt-control.service rosie-rt-natspublisher.service; do systemctl is-active \"$u\" || :; done",
        "status": "sudo -n /opt/rosie-rt-core/current/bin/rtctl status --json",
        "describe": "sudo -n /opt/rosie-rt-core/current/bin/rtctl describe --json",
    }
    data = {}
    for name, command in commands.items():
        observation = remote(servo, command)
        evidence.append({"name": name, **{key: value for key, value in observation.items()
                                           if key not in {"stdout", "stderr"}}})
        if observation["exitCode"] != 0:
            raise ValueError(f"servo {name} failed: {observation['stderr'][-300:]}")
        data[name] = checked_json(observation, name) if name in {"status", "describe"} else observation["stdout"]
    return data


def installed_errors(data: dict, manifest: dict, servo_peer: str) -> list[str]:
    """Match the installed package and service state to the candidate."""
    lines = data["identity"].strip().splitlines()
    expected = manifest["identity"]
    if len(lines) != 4:
        return ["installed_identity_unreadable"]
    errors = []
    if lines[0].lower() != servo_peer.lower():
        errors.append("servo_host_mismatch")
    if not re.fullmatch(r"[0-9a-f-]{36}", lines[1]):
        errors.append("servo_boot_identity_missing")
    if (Path(lines[2]).parent != Path("/opt/rosie-rt-core") or
            Path(lines[2]).name != expected["source"]["commit"]):
        errors.append("installed_commit_mismatch")
    if lines[3].split(" ", 1)[0] != expected["artifact"]["package_manifest_sha256"]:
        errors.append("installed_package_mismatch")
    units = data["units"].strip().splitlines()
    if len(units) != 3 or units[:2] != ["active", "active"] or units[2] not in {"active", "inactive"}:
        errors.append("servo_units_not_active")
    return errors


def source_checkout() -> tuple[str | None, list[str]]:
    """Read the checkout identity before executing its qualification code."""
    head = run("git", ["rev-parse", "HEAD"])
    if head["exitCode"] != 0 or not re.fullmatch(r"[0-9a-f]{40}", head["stdout"].strip()):
        return None, ["source_checkout_unavailable"]
    status = run("git", ["status", "--porcelain", "--untracked-files=all"])
    if status["exitCode"] != 0:
        return None, ["source_checkout_unavailable"]
    if status["stdout"]:
        return head["stdout"].strip(), ["source_checkout_dirty"]
    return head["stdout"].strip(), []


def write_receipt(directory: Path, receipt: dict) -> Path:
    """Write an immutable-name receipt without replacing earlier evidence."""
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = directory.stat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid() or info.st_mode & 0o077:
        raise ValueError("rehearsal receipt directory must be private")
    raw = (json.dumps(receipt, sort_keys=True, separators=(",", ":")) + "\n").encode()
    path = directory / f"{sha256(raw)}.receipt.json"
    with path.open("xb") as output:
        output.write(raw)
    return path


def main(argv: list[str]) -> int:
    """Authorize only a read-only, motion-free observation on the local cell."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cell", required=True, type=Path)
    parser.add_argument("--candidate-dir", required=True, type=Path)
    parser.add_argument("--receipts-dir", required=True, type=Path)
    parser.add_argument("--rehearsal-dir", required=True, type=Path)
    parser.add_argument("--install-only", action="store_true")
    parser.add_argument("--publication-receipt", type=Path)
    parser.add_argument("--fleet-url", required=True)
    parser.add_argument("--fleet-token-file", required=True, type=Path)
    parser.add_argument("--job-id", required=True)
    args = parser.parse_args(argv)
    args.candidate_dir = args.candidate_dir.resolve()
    args.receipts_dir = args.receipts_dir.resolve()
    args.rehearsal_dir = args.rehearsal_dir.resolve()
    if args.install_only != (args.publication_receipt is not None):
        raise ValueError("--install-only requires --publication-receipt, and vice versa")
    relay, servo, axes, target, node, config_pin, expected_slaves = pair_from_cell(args.cell)
    if socket.gethostname().split(".")[0].lower() != relay.lower():
        raise ValueError("admission must run on the cell's motion peer")
    local_boot = relay_boot_id()
    if not re.fullmatch(r"job-[A-Za-z0-9-]{1,124}", args.job_id):
        raise ValueError("invalid Fleet job ID")
    token_stat = args.fleet_token_file.stat()
    if not stat.S_ISREG(token_stat.st_mode) or token_stat.st_mode & 0o077:
        raise ValueError("Fleet snapshot token file must be private")
    token = args.fleet_token_file.read_text().strip()
    if not token or "\n" in token:
        raise ValueError("Fleet snapshot token is invalid")
    lock_path = Path("/run/lock") / f"rosie-hil-{relay}.lock"
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    lock_info = os.fstat(lock_fd)
    if not stat.S_ISREG(lock_info.st_mode) or lock_info.st_uid != os.geteuid():
        os.close(lock_fd)
        raise ValueError("local HIL lock owner or type differs")
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        os.close(lock_fd)
        raise ValueError("another local HIL check owns the bench") from error
    observations = []
    started = datetime.now(timezone.utc).isoformat()
    blockers = []
    candidate_id = None
    source_sha = None
    policy_sha = None
    scenario_identity = None
    publication_receipt_sha = None
    uncovered_source_claims = []
    snapshot_digests = []
    servo_boot_id = None
    final_safe = "not_evaluated"
    try:
        checkout_sha, checkout_blockers = source_checkout()
        blockers += checkout_blockers
        # The candidate verifier is the build host's worker, site infrastructure
        # left out of the public mirror: without it a candidate is blocked,
        # never admitted unverified.
        if not (ROOT / "tools/build1-release-worker").is_dir():
            blockers.append("candidate_verifier_missing")
        if not blockers:
            manifest = checked_json(run("python3", ["-m", "rosie_build1_candidate", "verify",
                                                    "--candidate", str(args.candidate_dir)],
                                        environment={**os.environ, "PYTHONPATH":
                                                     str(ROOT / "tools/build1-release-worker")},
                                        timeout=120), "candidate verification")
            candidate_id = manifest.get("candidate_id")
            source_sha = manifest.get("identity", {}).get("source", {}).get("commit")
            if not isinstance(source_sha, str) or not re.fullmatch(r"[0-9a-f]{40}", source_sha):
                blockers.append("source_commit_missing")
            elif checkout_sha != source_sha:
                blockers.append("source_checkout_mismatch")
        if not blockers:
            input_command = ["quality/qualification.mjs",
                             "install-inputs" if args.install_only else "hil-inputs",
                             "--candidate-dir", str(args.candidate_dir),
                             "--receipts-dir", str(args.receipts_dir),
                             "--cell", str(args.cell)]
            if args.install_only:
                input_command += ["--publication-receipt", str(args.publication_receipt)]
            hil = checked_json(run(node, input_command, timeout=120), "HIL inputs")
            policy_sha = hil.get("policySha256")
            expected_schema = ("rosie.qualification-install-inputs.v1" if args.install_only
                               else "rosie.qualification-hil-inputs.v1")
            if hil.get("schema") != expected_schema:
                blockers.append("hil_input_contract_invalid")
            source = hil.get("source")
            if not args.install_only and (not isinstance(source, dict) or
                                          source.get("state") != "source-qualified"):
                blockers.append("source_evidence_incomplete")
            expected_status = "install-inputs-checked" if args.install_only else "inputs_checked"
            if hil.get("inputStatus") != expected_status or hil.get("blockers") != []:
                blockers.append("qualification_inputs_blocked")
            if hil.get("candidateId") != candidate_id:
                blockers.append("candidate_identity_mismatch")
            if not isinstance(policy_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", policy_sha):
                blockers.append("policy_identity_missing")
            uncovered_source = hil.get("uncoveredSourceClaims")
            if not isinstance(uncovered_source, list) or any(
                    not isinstance(claim, str) for claim in uncovered_source):
                blockers.append("source_claims_invalid")
            elif not args.install_only and uncovered_source:
                blockers.append("source_claims_uncovered")
            else:
                uncovered_source_claims = uncovered_source
            if args.install_only:
                floor = hil.get("softwareFloor")
                publication_receipt_sha = (floor.get("publicationReceiptSha256")
                                           if isinstance(floor, dict) else None)
                if not isinstance(publication_receipt_sha, str) or not re.fullmatch(
                        r"[0-9a-f]{64}", publication_receipt_sha):
                    blockers.append("publication_evidence_missing")
                elif publication_receipt_sha != sha256(args.publication_receipt.read_bytes()):
                    blockers.append("publication_receipt_changed")
            if not hil.get("targetProfile", {}).get("matches") or target != hil["targetProfile"]["expected"]:
                blockers.append("candidate_target_mismatch")
            if hil.get("simulation", {}).get("state") != "simulation-qualified":
                blockers.append("simulation_evidence_incomplete")
            selected = [item for item in hil.get("scenarios", [])
                        if isinstance(item, dict) and item.get("id") == SCENARIO and
                        item.get("claim") == "installed-identity" and
                        item.get("motionAllowed") is False and
                        item.get("missingCapabilities") == [] and
                        isinstance(item.get("identity"), str) and
                        re.fullmatch(r"sha256:[0-9a-f]{64}", item["identity"])]
            if len(selected) != 1:
                blockers.append("motion_free_scenario_missing")
            else:
                scenario_identity = selected[0]["identity"]
        if not blockers:
            snapshot = fleet_snapshot(args.fleet_url, token)
            snapshot_digests.append(sha256(json.dumps(snapshot, sort_keys=True).encode()))
            blockers += reservation_errors(snapshot, args.job_id, relay,
                                           datetime.now(timezone.utc))
        if not blockers:
            first = observe_servo(servo, observations)
            identity_lines = first["identity"].strip().splitlines()
            servo_boot_id = identity_lines[1] if len(identity_lines) > 1 else None
            blockers += installed_errors(first, manifest, servo["peer"])
            blockers += safe_state_errors(first["status"], first["describe"], axes,
                                          config_pin, expected_slaves)
            if not blockers:
                time.sleep(1)
                second = observe_servo(servo, observations)
                final_errors = installed_errors(second, manifest, servo["peer"])
                final_errors += safe_state_errors(second["status"], second["describe"], axes,
                                                  config_pin, expected_slaves)
                if second["identity"].strip().splitlines()[:3] != first["identity"].strip().splitlines()[:3]:
                    final_errors.append("servo_changed_during_observation")
                first_core = first["status"]["core"]
                second_core = second["status"].get("core")
                if not isinstance(second_core, dict) or second_core.get("bus") != first_core.get("bus"):
                    final_errors.append("servo_bus_changed_during_observation")
                second_time = second_core.get("time_ns") if isinstance(second_core, dict) else None
                second_status_time = second["status"].get("time_ns")
                if (second["status"].get("daemon_incarnation") != first["status"].get("daemon_incarnation") or
                        second["status"].get("adapter_incarnation") != first["status"].get("adapter_incarnation") or
                        type(second_status_time) is not int or
                        second_status_time <= first["status"]["time_ns"] or
                        type(second_time) is not int or second_time <= first_core["time_ns"]):
                    final_errors.append("controller_restarted_or_stale")
                final_snapshot = fleet_snapshot(args.fleet_url, token)
                snapshot_digests.append(sha256(json.dumps(final_snapshot, sort_keys=True).encode()))
                final_errors += reservation_errors(final_snapshot, args.job_id, relay,
                                                    datetime.now(timezone.utc))
                blockers += final_errors
                final_safe = "pass" if not final_errors else "incomplete"
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        blockers.append("observation_unavailable: " + str(error)[:300])
    receipt = {
        "schema": "rosie.qualification-hil-rehearsal.v1",
        "scenarioId": SCENARIO,
        "candidateId": candidate_id,
        "sourceSha": source_sha,
        "policySha256": policy_sha,
        "inputScope": "install-only" if args.install_only else "full-hil",
        "publicationReceiptSha256": publication_receipt_sha,
        "uncoveredSourceClaims": uncovered_source_claims,
        "scenarioIdentity": scenario_identity,
        "cellConfigSha256": sha256((ROOT / args.cell).read_bytes()),
        "expectedRuntimeConfigurationSha256": config_pin,
        "cell": str(args.cell),
        "targetProfile": target,
        "relayPeer": relay,
        "relayBootId": local_boot,
        "servoPeer": servo["peer"],
        "servoBootId": servo_boot_id,
        "jobId": args.job_id,
        "motionAllowed": False,
        "hostWriteAssurance": "cooperative",
        "qualificationState": "not_qualified",
        "startedAt": started,
        "finishedAt": datetime.now(timezone.utc).isoformat(),
        "behaviorVerdict": "pass" if not blockers else "incomplete",
        "finalSafeVerdict": final_safe,
        "blockers": sorted(set(blockers)),
        "fleetSnapshotSha256": snapshot_digests,
        "observations": observations,
    }
    try:
        path = write_receipt(args.rehearsal_dir, receipt)
    finally:
        os.close(lock_fd)
    print(json.dumps({"admission": "motion_free_observation_only" if not blockers else "blocked",
                      "receipt": str(path), "blockers": receipt["blockers"]}, indent=2))
    return 0 if not blockers else 2


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except (OSError, ValueError) as error:
        print(f"hil_admission_unavailable: {error}", file=sys.stderr)
        sys.exit(2)
