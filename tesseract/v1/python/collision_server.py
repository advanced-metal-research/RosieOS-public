"""Persistent loopback adapter for the exact installed Tesseract collision worker.

Every request still checks every waypoint. Only immutable geometry setup is reused.
"""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import multiprocessing
import resource
import socket
import time
import sys

FACTORY_SHA256 = '6a2d0f850fe354b695bb9318d017c2660a9ec307c6c0b39e93ceae32361d2111'


def geometry_identity(request):
    urdf = Path(request['urdf_path']).resolve(strict=True)
    root = Path(request['mesh_package_root']).resolve(strict=True)
    meshes = root / 'meshes'
    if not meshes.is_dir():
        raise ValueError('Mesh package is unavailable')
    files = sorted(meshes.rglob('*'))
    if len(files) > 10000:
        raise ValueError('Mesh package exceeds file bound')
    identity = []
    for path in files:
        if path.is_file():
            resolved = path.resolve(strict=True)
            stat = resolved.stat()
            identity.append((str(path.relative_to(root)), str(resolved), stat.st_dev,
                             stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns))
    return (str(urdf), hashlib.sha256(urdf.read_bytes()).hexdigest(), str(root),
            tuple(identity), tuple(request['joint_names']), request['robot_model'])


class PersistentCollision:
    def __init__(self, factory):
        self.factory = factory
        self.world = None
        self.identity = None

    def check(self, request):
        before = geometry_identity(request)
        reused = self.world is not None and self.identity == before
        if not reused:
            self.world = None
            self.identity = None
            world = self.factory.CollisionWorld(request)
            if geometry_identity(request) != before:
                raise ValueError('Geometry changed while loading collision world')
            self.world = world
            self.identity = before
        result = self.world.check(request)
        if geometry_identity(request) != before:
            self.world = None
            self.identity = None
            raise ValueError('Geometry changed during collision checking')
        result['persistent_world_reused'] = reused
        return result


def load_factory(path, expected_hash=FACTORY_SHA256):
    path = path.resolve(strict=True)
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_hash:
        raise ValueError('Installed collision worker differs from qualified source')
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location('jslop_collision_factory', path)
    factory = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(factory)
    return factory


def collision_child(connection, factory_path, factory_hash):
    """One process owns one exact factory world; no state is shared with its peer."""
    try:
        service = PersistentCollision(load_factory(Path(factory_path), factory_hash))
        while True:
            request, expected = connection.recv()
            if geometry_identity(request) != expected:
                raise ValueError('Child geometry differs from parent snapshot')
            result = service.check(request)
            if geometry_identity(request) != expected:
                raise ValueError('Child geometry changed during checking')
            peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            connection.send({'result': result, 'identity': expected,
                             'peak_rss_bytes': int(peak if sys.platform == 'darwin' else peak * 1024)})
    except EOFError:
        pass
    except Exception as error:
        try:
            connection.send({'error': f'{type(error).__name__}: {error}'})
        except (BrokenPipeError, OSError):
            pass
    finally:
        connection.close()


class ParallelCollision:
    """Opt-in two-way contiguous batches. Every pose uses the original factory."""
    def __init__(self, factory, timeout=230):
        self.factory_path = Path(factory.__file__).resolve(strict=True)
        self.factory_hash = FACTORY_SHA256
        self.timeout = timeout
        self.children = []

    def close(self):
        for connection, process in self.children:
            connection.close()
            if process.is_alive():
                process.terminate()
            process.join(timeout=1)
            if process.is_alive():
                process.kill()
                process.join(timeout=1)
            process.close()
        self.children = []

    def start(self):
        if self.children:
            return
        context = multiprocessing.get_context('spawn')
        try:
            for _ in range(2):
                parent, child = context.Pipe()
                process = context.Process(target=collision_child,
                                          args=(child, str(self.factory_path), self.factory_hash), daemon=True)
                try:
                    process.start()
                except Exception:
                    parent.close()
                    child.close()
                    raise
                child.close()
                self.children.append((parent, process))
        except Exception:
            self.close()
            raise

    def check(self, request):
        try:
            if hashlib.sha256(self.factory_path.read_bytes()).hexdigest() != self.factory_hash:
                raise ValueError('Qualified collision factory changed')
            before = geometry_identity(request)
            waypoints = request.get('waypoints', [])
            middle = (len(waypoints) + 1) // 2
            batches = [waypoints[:middle], waypoints[middle:]] if len(waypoints) > 1 else [waypoints]
            self.start()
            started = time.monotonic()
            deadline = started + self.timeout
            for (connection, _), batch in zip(self.children, batches):
                connection.send(({**request, 'waypoints': batch}, before))
            replies = []
            for (connection, _), batch in zip(self.children, batches):
                if not connection.poll(max(0, deadline - time.monotonic())):
                    raise TimeoutError('Persistent collision child deadline')
                reply = connection.recv()
                if reply.get('error'):
                    raise RuntimeError(reply['error'])
                if reply.get('identity') != before:
                    raise ValueError('Collision child geometry identity mismatch')
                result = reply['result']
                if (result.get('ok') is not True
                        or result.get('checked_waypoint_count') != len(batch)
                        or result.get('contact_count') != len(result.get('contacts', []))):
                    raise ValueError('Collision child result is incomplete')
                replies.append(reply)
            if geometry_identity(request) != before:
                raise ValueError('Geometry changed during parallel collision checking')
            if hashlib.sha256(self.factory_path.read_bytes()).hexdigest() != self.factory_hash:
                raise ValueError('Qualified collision factory changed during checking')
            # Compare all immutable checker/geometry/ACM metadata. Never silently
            # combine results from different checker versions or policies.
            varying = {'contacts', 'contact_count', 'checked_waypoint_count',
                       'check_duration_s', 'persistent_world_reused'}
            results = [reply['result'] for reply in replies]
            metadata = {k: v for k, v in results[0].items() if k not in varying}
            if any({k: v for k, v in result.items() if k not in varying} != metadata for result in results):
                raise ValueError('Collision child checker/ACM metadata differs')
            merged = dict(results[0])
            # Contiguous shards and ordered concatenation preserve original pose
            # order, including duplicate waypoint indices and repeated contacts.
            merged['contacts'] = [contact for result in results for contact in result['contacts']]
            merged['contact_count'] = sum(result['contact_count'] for result in results)
            merged['checked_waypoint_count'] = sum(result['checked_waypoint_count'] for result in results)
            merged['check_duration_s'] = time.monotonic() - started
            merged['persistent_world_reused'] = all(result['persistent_world_reused'] for result in results)
            merged['parallel_worker_count'] = len(results)
            merged['parallel_worker_check_duration_s'] = [result['check_duration_s'] for result in results]
            merged['parallel_worker_peak_rss_bytes'] = [reply['peak_rss_bytes'] for reply in replies]
            return merged
        except Exception:
            # A failed or changed world never survives to serve the next request.
            self.close()
            raise


def serve(factory, port, ready_file, workers=1):
    service = ParallelCollision(factory) if workers == 2 else PersistentCollision(factory)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(('127.0.0.1', port))
        listener.listen(4)
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        ready_file.write_text(json.dumps({'schema': 'jslop.collision-server-ready.v1',
                                         'host': '127.0.0.1', 'port': listener.getsockname()[1], 'workers': workers}) + '\n')
        while True:
            connection, _ = listener.accept()
            with connection:
                connection.settimeout(240)
                try:
                    request = json.loads(factory.read_frame(connection))
                    result = service.check(request)
                except Exception as error:
                    result = factory.error_result(error)
                try:
                    factory.write_frame(connection, json.dumps(result, allow_nan=False).encode())
                except (OSError, ValueError):
                    # A disconnected caller cannot kill the persistent service.
                    continue


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--factory-worker', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--ready-file', type=Path, required=True)
    parser.add_argument('--workers', type=int, choices=(1, 2), default=1,
                        help='Opt-in two independent persistent collision worlds; default 1')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error('Invalid loopback port')
    serve(load_factory(args.factory_worker), args.port, args.ready_file, args.workers)
