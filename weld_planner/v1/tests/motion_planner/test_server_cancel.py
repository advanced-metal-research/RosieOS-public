"""Cancellation kills actual OS workers, not merely their awaiting HTTP task."""
from __future__ import annotations

import asyncio
import hashlib
import json
import os
import sys

import pytest

from weld_motion_planner.server import motion_planner_server as server


class Request:
    def __init__(self, body=b"wait"):
        self.data = body
        self.disconnected = False

    async def body(self):
        return self.data

    async def is_disconnected(self):
        return self.disconnected


@pytest.fixture
def worker(tmp_path, monkeypatch):
    script = tmp_path / "worker.py"
    script.write_text("""
import hashlib, json, os, signal, sys, time
from pathlib import Path
job = Path(sys.argv[1])
mode = (job / 'request.weldplan').read_bytes()
if mode in (b'descendant', b'leader-exit'):
    import subprocess
    ready = job.parent.parent / 'descendant.pid'
    code = ("import os,signal,time; from pathlib import Path; "
            "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
            "Path(" + repr(str(ready)) + ").write_text(str(os.getpid())); "
            "time.sleep(120)")
    subprocess.Popen([sys.executable, '-c', code])
    while not ready.exists():
        time.sleep(0.01)
if mode == b'stubborn':
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
blob = b'completed dense trajectory'
digest = 'sha256:' + hashlib.sha256(blob).hexdigest()
(job / 'dense').mkdir()
(job / 'dense' / (digest.replace(':','-') + '.rdt')).write_bytes(blob)
print('ROSIE_PROGRESS:' + json.dumps('worker:' + str(os.getpid())), flush=True)
if mode == b'leader-exit':
    raise SystemExit(3)
if mode != b'complete':
    while True:
        time.sleep(0.02)
(job / 'result.json').write_text(json.dumps({'out': {'seams': [], 'dense': {'trajectory_digest': digest}}, 'moves': 0, 'joining_error': None}))
""", encoding="utf-8")
    monkeypatch.setattr(server, "_worker_command", lambda job: [sys.executable, str(script), str(job)])
    return tmp_path / "store"


async def wait_for(predicate):
    async with asyncio.timeout(8):
        while not predicate():
            await asyncio.sleep(0.01)


def assert_dead(pid):
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


def endpoint(app, path):
    return next(route.endpoint for route in app.routes if route.path == path)


def plan(app, request):
    return endpoint(app, "/api/motion/plan")(
        request, k_best=5, screen_collisions=True, samples_per_seam=None,
        run_weld_trajopt=True, run_connecting_trajopt=True,
        free_space_backend="bspline", order_seams=False, verify=True, verify_margin_mm=2.0,
        dense=True, program_id="test", manifest_revision=1, plan_revision=1)


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
@pytest.mark.parametrize("mode", [b"wait", b"stubborn"])
def test_disconnect_stops_real_worker_releases_lock_and_discards_output(worker, mode):
    async def scenario():
        app = server.make_app(worker)
        progress = endpoint(app, "/api/motion/progress")
        active = Request(mode)
        running = asyncio.create_task(plan(app, active))
        await wait_for(lambda: progress()["stage"] is not None)
        pid = int(progress()["stage"].split(":")[1])
        # A cancelled queued request must neither launch nor clear active status.
        queued = Request()
        waiting = asyncio.create_task(plan(app, queued))
        await asyncio.sleep(0.03)
        queued.disconnected = True
        assert (await asyncio.wait_for(waiting, 2)).status_code == 499
        assert progress()["stage"] == f"worker:{pid}"
        assert len(list(worker.glob(".planning-*"))) == 1
        active.disconnected = True
        assert (await asyncio.wait_for(running, 5)).status_code == 499
        assert_dead(pid)
        assert progress()["stage"] is None
        assert list(worker.iterdir()) == []
        # A subsequent request acquires the released lock and publishes only
        # its successful output, while cancelled private exports stay absent.
        done = await asyncio.wait_for(plan(app, Request(b"complete")), 5)
        assert done.status_code == 200
        answer = json.loads(done.body)
        path = worker / (answer["dense"]["trajectory_digest"].replace(":", "-") + ".rdt")
        assert path.read_bytes() == b"completed dense trajectory"
        assert list(worker.glob(".planning-*")) == []
    asyncio.run(scenario())


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
def test_handler_task_cancellation_also_reaps_worker(worker):
    async def scenario():
        app = server.make_app(worker)
        progress = endpoint(app, "/api/motion/progress")
        task = asyncio.create_task(plan(app, Request()))
        await wait_for(lambda: progress()["stage"] is not None)
        pid = int(progress()["stage"].split(":")[1])
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert_dead(pid)
        assert progress()["stage"] is None
        assert list(worker.iterdir()) == []
        assert (await plan(app, Request(b"complete"))).status_code == 200
    asyncio.run(scenario())


def test_real_worker_entrypoint_imports_and_reports_invalid_request(tmp_path):
    """Exercise the production interpreter/module command without running GPU work."""
    async def scenario():
        with pytest.raises(RuntimeError, match="PlanRequestError: not a zip archive"):
            await asyncio.wait_for(server._run_plan_job(
                b"not a weldplan", {
                    "k_best": 5, "screen_collisions": True, "samples_per_seam": None,
                    "run_weld_trajopt": True, "run_connecting_trajopt": True,
                    "free_space_backend": "bspline", "order_seams": False, "verify": True,
                    "verify_margin_mm": 2.0, "dense": True, "program_id": "test",
                    "manifest_revision": 1, "plan_revision": 1,
                }, tmp_path, Request(), lambda stage: None), 30)
        assert list(tmp_path.iterdir()) == []
    asyncio.run(scenario())


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
def test_aborting_real_http_request_kills_planner_and_allows_next_plan(worker):
    import httpx
    import socket
    import uvicorn

    async def scenario():
        app = server.make_app(worker)
        progress = endpoint(app, "/api/motion/progress")
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
        instance = uvicorn.Server(uvicorn.Config(app, log_level="error", lifespan="off"))
        serving = asyncio.create_task(instance.serve(sockets=[listener]))
        try:
            await wait_for(lambda: instance.started)
            async with httpx.AsyncClient(base_url=f"http://127.0.0.1:{port}", timeout=10) as client:
                posting = asyncio.create_task(client.post("/api/motion/plan", content=b"wait"))
                await wait_for(lambda: progress()["stage"] is not None)
                pid = int(progress()["stage"].split(":")[1])
                posting.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await posting
                await wait_for(lambda: progress()["stage"] is None)
                assert_dead(pid)
                assert list(worker.iterdir()) == []
                reply = await client.post("/api/motion/plan", content=b"complete")
                assert reply.status_code == 200
        finally:
            instance.should_exit = True
            await asyncio.wait_for(serving, 10)
            listener.close()
    asyncio.run(scenario())


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
def test_task_cancel_during_spawn_does_not_orphan_worker(worker, monkeypatch):
    async def scenario():
        create_process = asyncio.create_subprocess_exec
        spawned = []
        release_handle = asyncio.Event()

        async def delayed_handle(*args, **kwargs):
            process = await create_process(*args, **kwargs)
            spawned.append(process)
            await release_handle.wait()
            return process

        monkeypatch.setattr(asyncio, "create_subprocess_exec", delayed_handle)
        task = asyncio.create_task(server._run_plan_job(
            b"wait", {}, worker, Request(), lambda stage: None))
        await wait_for(lambda: bool(spawned))
        task.cancel()
        await asyncio.sleep(0)
        release_handle.set()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 5)
        assert_dead(spawned[0].pid)
        assert list(worker.iterdir()) == []
    asyncio.run(scenario())


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
@pytest.mark.parametrize("mode", [b"descendant", b"leader-exit"])
def test_cancellation_or_failed_leader_kills_remaining_process_group(worker, mode):
    from pathlib import Path

    async def scenario():
        request = Request(mode)
        stages = []
        task = asyncio.create_task(server._run_plan_job(mode, {}, worker, request, stages.append))
        await wait_for(lambda: bool(stages))
        child_pid = int((worker.parent / "descendant.pid").read_text())
        if mode == b"descendant":
            request.disconnected = True
            with pytest.raises(server._ClientDisconnected):
                await asyncio.wait_for(task, 5)
        else:
            with pytest.raises(RuntimeError, match="status 3"):
                await asyncio.wait_for(task, 5)
        # Reparented descendants may briefly remain zombies until PID1 reaps
        # them; zombies have exited and cannot retain CUDA or file handles.
        def child_exited():
            status = Path(f"/proc/{child_pid}/stat")
            return not status.exists() or status.read_text().split()[2] == "Z"
        await wait_for(child_exited)
        assert list(worker.iterdir()) == []
    asyncio.run(scenario())


def test_completed_but_disconnected_result_cannot_replace_public_blob(worker, monkeypatch):
    from weld_motion_planner.io.dense_output import dense_blob_path
    import hashlib

    async def scenario():
        worker.mkdir()
        digest = "sha256:" + hashlib.sha256(b"completed dense trajectory").hexdigest()
        published = dense_blob_path(worker, digest)
        published.write_bytes(b"previous published bytes")
        request = Request(b"complete")
        stop_worker = server._stop_worker
        async def disconnect_after_completion(process):
            await stop_worker(process)
            request.disconnected = True
        monkeypatch.setattr(server, "_stop_worker", disconnect_after_completion)
        with pytest.raises(server._ClientDisconnected):
            await server._run_plan_job(b"complete", {}, worker, request, lambda stage: None)
        assert published.read_bytes() == b"previous published bytes"
        assert list(worker.glob(".planning-*")) == []
    asyncio.run(scenario())


def test_simultaneous_admission_keeps_queued_disconnect_cancellable(worker):
    async def scenario():
        app = server.make_app(worker)
        progress = endpoint(app, "/api/motion/progress")
        simultaneous = asyncio.Event()
        entered = []
        class SimultaneousRequest(Request):
            async def is_disconnected(self):
                if self not in entered:
                    entered.append(self)
                    await simultaneous.wait()
                return self.disconnected
        active, queued = SimultaneousRequest(), SimultaneousRequest()
        first = asyncio.create_task(plan(app, active))
        second = asyncio.create_task(plan(app, queued))
        await wait_for(lambda: len(entered) == 2)
        simultaneous.set()
        await wait_for(lambda: progress()["stage"] is not None)
        queued.disconnected = True
        assert (await asyncio.wait_for(second, 2)).status_code == 499
        assert not first.done()
        active.disconnected = True
        assert (await asyncio.wait_for(first, 5)).status_code == 499
        assert list(worker.iterdir()) == []
    asyncio.run(scenario())


def refused_result():
    return {"seams": [{"id": "seam_0011", "crossed": True,
                       "trajectory": {"verdict": {"verdict": "FAIL", "collisions": 1},
                                      "limit_violations": [{"axis": "J2", "kind": "position",
                                                            "value_rad": 2, "limit_rad": [-1, 1]}]}}],
            "dense_error": {"reason": "motion_not_verified", "detail": "verification failed",
                            "segment_index": None},
            "additional_evidence": {"pair": ["torch", "workpiece"], "value": 0.000001}}


def test_refused_request_retains_complete_result_and_prunes_pairs(tmp_path, monkeypatch):
    out = refused_result()
    async def refuse(*args):
        return {"out": out, "moves": 0, "joining_error": None}
    monkeypatch.setattr(server, "_run_plan_job", refuse)

    async def scenario():
        store = tmp_path / "store"
        app = server.make_app(store)
        refused = store / "refused"
        digests = []
        for index in range(server.REFUSED_KEPT + 2):
            data = f"request {index}".encode()
            digest = hashlib.sha256(data).hexdigest()
            digests.append(digest)
            response = await plan(app, Request(data))
            assert response.status_code == 200
            assert json.loads(response.body) == out, "retention must not alter the API response"
            path = refused / f"{digest}.weldplan"
            assert path.read_bytes() == data
            assert json.loads(path.with_suffix(".result.json").read_text()) == out
            # Deterministic ordering even on filesystems with coarse mtimes.
            os.utime(path, (index + 1, index + 1))
        expected = set(digests[-server.REFUSED_KEPT:])
        assert {p.stem for p in refused.glob("*.weldplan")} == expected
        assert {p.name.removesuffix(".result.json") for p in refused.glob("*.result.json")} == expected
        assert not list(refused.glob("*.tmp"))
    asyncio.run(scenario())


@pytest.fixture
def spare(worker, tmp_path, monkeypatch):
    """The fake worker started as a spare: it announces itself, then takes its
    job directory on stdin and runs the fake worker in the same process."""
    script = tmp_path / "spare.py"
    script.write_text(f"""
import os, runpy, sys
from pathlib import Path
here = Path({str(tmp_path)!r})
(here / f"started-{{os.getpid()}}").touch()
line = sys.stdin.readline().strip()
if not line:
    raise SystemExit(0)
(here / f"took-{{os.getpid()}}").touch()
sys.argv = [str(here / "worker.py"), line]
runpy.run_path(sys.argv[0], run_name="__main__")
""", encoding="utf-8")
    monkeypatch.setattr(server, "_spare_command", lambda: [sys.executable, str(script)])
    return worker


def pids(directory, prefix):
    return sorted(int(p.name.split("-")[1]) for p in directory.glob(prefix + "-*"))


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
def test_plan_runs_on_the_spare_started_before_it_and_the_next_spare_follows(spare):
    here = spare.parent

    async def scenario():
        app = server.make_app(spare, warm_worker=True)
        async with app.router.lifespan_context(app):
            await wait_for(lambda: len(pids(here, "started")) == 1)
            first = pids(here, "started")[0]
            done = await asyncio.wait_for(plan(app, Request(b"complete")), 5)
            assert done.status_code == 200
            assert pids(here, "took") == [first]
            # The next spare starts once this plan's process is gone.
            await wait_for(lambda: len(pids(here, "started")) == 2)
            assert_dead(first)
            second = [p for p in pids(here, "started") if p != first][0]
            assert (await asyncio.wait_for(plan(app, Request(b"complete")), 5)).status_code == 200
            assert second in pids(here, "took")
            await wait_for(lambda: len(pids(here, "started")) == 3)
            third = [p for p in pids(here, "started") if p not in (first, second)][0]
        # Shutdown takes the waiting spare with it.
        assert_dead(third)
        assert list(spare.glob(".planning-*")) == []
    asyncio.run(scenario())


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
def test_disconnect_kills_the_spare_running_the_plan_and_the_next_plan_still_runs(spare):
    here = spare.parent

    async def scenario():
        app = server.make_app(spare, warm_worker=True)
        progress = endpoint(app, "/api/motion/progress")
        async with app.router.lifespan_context(app):
            await wait_for(lambda: len(pids(here, "started")) == 1)
            first = pids(here, "started")[0]
            active = Request(b"wait")
            running = asyncio.create_task(plan(app, active))
            await wait_for(lambda: progress()["stage"] is not None)
            assert progress()["stage"] == f"worker:{first}"
            active.disconnected = True
            assert (await asyncio.wait_for(running, 5)).status_code == 499
            assert_dead(first)
            assert list(spare.iterdir()) == []
            assert (await asyncio.wait_for(plan(app, Request(b"complete")), 5)).status_code == 200
    asyncio.run(scenario())


@pytest.mark.skipif(os.name != "posix", reason="production process-group lifecycle is POSIX")
def test_a_dead_spare_means_a_fresh_worker_not_a_failed_plan(spare):
    import signal
    here = spare.parent

    async def scenario():
        app = server.make_app(spare, warm_worker=True)
        async with app.router.lifespan_context(app):
            await wait_for(lambda: len(pids(here, "started")) == 1)
            os.kill(pids(here, "started")[0], signal.SIGKILL)
            await asyncio.sleep(0.2)
            assert (await asyncio.wait_for(plan(app, Request(b"complete")), 5)).status_code == 200
            assert pids(here, "took") == []
            await wait_for(lambda: len(pids(here, "started")) == 2)
    asyncio.run(scenario())


def test_the_spare_preloads_only_modules_that_exist():
    import importlib.util
    missing = [name for name in server.WARM_MODULES if importlib.util.find_spec(name) is None]
    assert missing == []


def test_refused_diagnostics_write_failure_does_not_change_response(tmp_path, monkeypatch):
    out = refused_result()
    async def refuse(*args):
        return {"out": out, "moves": 0, "joining_error": None}
    monkeypatch.setattr(server, "_run_plan_job", refuse)
    store = tmp_path / "store"
    store.mkdir()
    (store / "refused").write_text("not a directory")
    async def scenario():
        response = await plan(server.make_app(store), Request(b"request"))
        assert response.status_code == 200
        assert json.loads(response.body) == out
    asyncio.run(scenario())
