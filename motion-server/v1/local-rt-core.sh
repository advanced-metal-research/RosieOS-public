#!/usr/bin/env bash
# Local simulation configuration and process entry points. Explicit bindings
# prevent a missing simulator from redirecting local tools to a deployed cell.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Unverified local defaults; Unix sockets require a Linux filesystem in WSL.
RT_DIR="${ROSIE_LOCAL_RT_DIR:-${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}/rosie-local-rt-$UID}"
RT_BUILD="${ROSIE_LOCAL_RT_BUILD:-$ROOT/rt-core/build/local-stack}"
umask 077
[[ ! -L "$RT_DIR" && ( ! -e "$RT_DIR" || -O "$RT_DIR" ) ]] || { echo "local-rt-core: refusing unowned runtime directory $RT_DIR" >&2; exit 1; }
mkdir -p "$RT_DIR" "$RT_BUILD"
chmod 0700 "$RT_DIR" "$RT_BUILD"
case "${1:-}" in
  prepare)
    python3 - "$ROOT" "$RT_BUILD" <<'PY'
import json, math, os, pathlib, sys, xml.etree.ElementTree as ET
root, build = map(pathlib.Path, sys.argv[1:])
# The local cell: flat simulated axes by default, which Home, arm and jog.
# DEV_STACK_MACHINE_CONFIG=rt-core/config/machines/simulation/simulation-rosie1400.json
# binds the Rosie 1400 instead, so the cell serves the description the OLP
# plans against and judges a plan's robot by it (docs/PLANNING-CONTEXT.md)
# The simulated A6-EC now supports its Home dictionary; robot-bound cells
# retain the real profile and Home requirement. They remain explicit opt-ins.
machine_config = os.environ.get('DEV_STACK_MACHINE_CONFIG') or 'rt-core/config/machines/simulation/simulation-program.json'
machine = json.loads((root / machine_config).read_text())
if machine.get('backend') != 'simulation':
    raise SystemExit('local simulation refuses a hardware machine configuration')
allowed = sorted(os.sched_getaffinity(0))
if len(allowed) < 2:
    raise SystemExit('local simulation requires two allowed CPUs')
if machine['runtime']['cpu'] not in allowed:
    machine['runtime']['cpu'] = allowed[-1]
if 'robot' not in machine:
    # Flat axes: the simulation Home profile supplies commissioning for dense
    # playback. A robot-bound machine takes its drive profile and Home
    # policy from the robot definition.
    # The synthetic commissioning profile remains necessary for simulator
    # Home, but its fixture travel numbers are not the robot's authority.
    # Use the same canonical URDF advertised by `bind` below on every start.
    urdf = root / 'robot_description/robots/rosie_1400_v3/robot.urdf'
    joints = {joint.attrib['name']: joint for joint in ET.parse(urdf).getroot().findall('joint')}
    for axis in machine['axes']:
        joint = joints.get(axis['name'])
        limit = joint.find('limit') if joint is not None else None
        if limit is None:
            raise SystemExit('simulation joint has no canonical URDF limits: ' + axis['name'])
        lower, upper, velocity = (float(limit.attrib[k]) for k in ('lower', 'upper', 'velocity'))
        if not all(math.isfinite(v) for v in (lower, upper, velocity)) or lower >= upper or velocity <= 0:
            raise SystemExit('invalid canonical URDF limits: ' + axis['name'])
        axis['limits'].update(min=lower, max=upper, max_velocity=velocity)
        axis['limits']['max_source'] = 'Derived from the selected local robot description robot.urdf at startup.'
        axis['limits']['max_velocity_source'] = axis['limits']['max_source']
        axis['profile'] = 'cia402_sim_home'
        axis['require_home'] = True
(build / 'machine.json').write_text(json.dumps(machine))
PY
    "$ROOT/rt-core/build/rtctl" compile --config "$RT_BUILD/machine.json" \
      --profiles "$ROOT/rt-core/config/drives" --out "$RT_BUILD/config" > "$RT_BUILD/compiled-path"
    ;;
  sim)
    exec python3 - "$ROOT" "$RT_BUILD" "$RT_DIR" <<'PY'
import json, os, pathlib, sys
root, build, runtime = map(pathlib.Path, sys.argv[1:])
compiled = pathlib.Path((build / 'compiled-path').read_text().strip())
args = json.loads((compiled / 'argv.json').read_text())
binary = str(root / 'rt-core/build/rosie-rt-core-sim')
os.chdir(build)
os.execv(binary, [binary, *args, '--socket-path', str(runtime / 'ipc.sock')])
PY
    ;;
  control)
    compiled="$(cat "$RT_BUILD/compiled-path")"
    cd "$RT_BUILD"
    # The compiled directory holds the description the machine pins and its
    # machine_planning_calibration.json; the adapter serves them (docs/PLANNING-CONTEXT.md).
    exec "$ROOT/rt-core/build/rt-control" --core-socket "$RT_DIR/ipc.sock" \
      --socket "$RT_DIR/control.sock" --compiled-config "${compiled}" \
      --configuration-sha256 "${compiled##*/}" --backend simulation \
      --pair-id local-dev --pair-revision 1
    ;;
  bind)
    curl --fail --silent --show-error --unix-socket "$RT_DIR/control.sock" \
      http://localhost/v1/describe > "$RT_BUILD/describe.json"
    python3 - "$ROOT" "$RT_BUILD" "$RT_DIR" <<'PY'
import hashlib, json, os, pathlib, shlex, sys, xml.etree.ElementTree as ET
root, build, runtime = map(pathlib.Path, sys.argv[1:])
d = json.loads((build / 'describe.json').read_text())['data']
compiled = pathlib.Path((build / 'compiled-path').read_text().strip())
if d['backend'] != 'simulation' or d['configuration_sha256'] != compiled.name:
    raise SystemExit('local adapter identity mismatch')
socket = str(runtime / 'control.sock')
machine = json.loads((build / 'machine.json').read_text())
reference = (machine.get('robot') or {}).get('robot_description')
if reference:
    model = (root / reference['path']).resolve(strict=True)
    model.relative_to((root / 'robot_description/robots').resolve(strict=True))
    robot = d.get('robot') or {}
    if robot.get('model_id') != model.name or robot.get('robot_description_sha256') != reference['sha256']:
        raise SystemExit('local adapter robot description mismatch')
    urdf = model / 'robot.urdf'
else:
    urdf = root / 'robot_description/robots/rosie_1400_v3/robot.urdf'
joint_names = [j.attrib['name'] for j in ET.parse(urdf).getroot().findall('joint')
               if j.attrib['type'] != 'fixed']
axis_names = [a['id'] for a in d['axes']]
if sorted(axis_names) != sorted(joint_names) or any(a['index'] != i for i, a in enumerate(d['axes'])):
    raise SystemExit('local adapter axes disagree with the selected URDF')
axis_mask = (1 << len(axis_names)) - 1
env = {
    'ROSIE_RT_CONTROL_SOCKET': socket, 'ROSIE_RT_JOG_SOCKET': str(runtime / 'jog.sock'),
    'ROSIE_RT_PAIR_ID': 'local-dev', 'ROSIE_RT_PAIR_REVISION': '1',
    'ROSIE_RT_CONFIGURATION_SHA256': d['configuration_sha256'],
    'ROSIE_RT_URDF': str(urdf), 'ROSIE_RT_URDF_SHA256': hashlib.sha256(urdf.read_bytes()).hexdigest(),
    'OFFLINE_PROGRAMMING_RT_CORE_CONFIG': str(build / 'olp-rt-core.json'),
}
(build / 'binding.env').write_text(''.join('export '+k+'='+shlex.quote(v)+'\n' for k,v in env.items()))
# Bind consumers to the model actually served by this simulator.
cells = [{'cell_id':'local-simulation', 'label':'Local simulation', 'role':'LOCAL SIMULATION',
          'models':[urdf.parent.name], 'execution':{'local':True}, 'requested_mode':'simulation'}]
if os.environ.get('DEV_STACK_OLP_CELLS'):
    source = pathlib.Path(os.environ['DEV_STACK_OLP_CELLS']).resolve(strict=True)
    extra = json.loads(source.read_text())
    if extra.get('schema') != 'offline-programming.cell-catalogue.v1':
        raise SystemExit('cell_catalogue_invalid')
    for cell in extra['cells']:
        for field in ('ca_file', 'certificate_file', 'key_file'):
            if cell.get('execution', {}).get(field):
                cell['execution'][field] = str((source.parent / cell['execution'][field]).resolve())
    cells.extend(extra['cells'])
(build / 'olp-cells.json').write_text(json.dumps({'schema':'offline-programming.cell-catalogue.v1', 'cells':cells})+'\n')
(build / 'olp-rt-core.json').write_text(json.dumps({
    'socket': socket, 'binding': {'pair_id':'local-dev', 'revision':1, 'machine_sha256':d['machine_sha256'],
                               'configuration_sha256':d['configuration_sha256']},
    'deployment_sha256':d['deployment_sha256'], 'capabilities_digest':d['capabilities_digest'], 'axis_mask':axis_mask,
}))
(build / 'daemon-rt-core.json').write_text(json.dumps({
    'schema':'robot.v4.joint-trajectory-daemon.config.v1', 'backend':'rt_core',
    'rt_core': {'socket':socket, 'controller':'local-dense', 'pair_id':'local-dev', 'pair_revision':1,
                'configuration_sha256':d['configuration_sha256'], 'machine_sha256':d['machine_sha256'],
                'deployment_sha256':d['deployment_sha256'], 'expected_backend':'simulation',
                'home_policy':'home'},
}))
PY
    ;;
  *) echo 'usage: local-rt-core.sh prepare|sim|control|bind' >&2; exit 2 ;;
esac
