# rosie_1430_v1

A six-axis Rosie 1430 arm on a pedestal, on six A6-EC drives, with its MIG
torch on `link_6`. No positioner. This is the model of the Rosie 1430 arms in
service: the same URDF geometry, limits, meshes, collision spheres and gearing
they run, under the model name `rosie_1430_v1` (the package its URDF has
always named its meshes under). A real arm's own description and its
commissioning data (signs, encoder batteries, per-joint drive tuning) are site
configuration and stay with that arm.

The public pattern for running one is the placeholder cell
`rt-core/config/cells/example-arm-1430.json` and its machine config
`rt-core/config/machines/example-arm-1430.json`: copy both, replace the hosts,
logins and paths with your own, and set every per-arm value below from your
arm before it moves.

## Check on your arm

- **Signs.** All six are -1, as on the arms in service. A sign is the arm's
  drive wiring, not the model: check each joint's direction with a slow joint
  jog before any Cartesian work.
- **Encoder batteries.** Recorded `present`, so the compiled Home behaviour
  is multi-turn. Check the cables and battery condition before relying on Home
  across a power cycle.
- **J5 range and the torch.** J5 is +/-2.2 rad. A fitted torch can hit the arm
  before that (one arm caps J5 at +0.5934 rad, +34 degrees): measure your
  torch's free range and lower the limit before jogging J5 near the top of its
  range.
- **Home.** Required. Every joint needs an operator HM35 at its mechanical
  Home pose; no anchors are included.

## Configuration evidence

- Six arm joints J1-J6; reductions 100/100/100/18/100/100 with J6 at 100:1,
  the current Rosie 1430 gearbox (owner, 2026-11-13). The reduction is a
  software ratio: no drive object reads it back. 131072 motor encoder counts
  per revolution (A6-EC, AS715N).
- Joint limits J1 +/-3.1, J2 +/-1.9, J3 +/-2.6, J4 +/-3.1, J5 +/-2.2, J6 +/-3.1
  rad; 80 degrees per second (1.3963 rad/s) on every joint, and the
  4*pi rad/s^2 acceleration.
- Geometry: the Rosie 1430 CAD geometry (J5 origin x 0.5147 m, J6 origin z
  -0.0993 m). Joint jog does not depend on it; confirm it on your arm before
  Cartesian work.
- The model permits no per-cell kinematic corrections. Reset is the URDF zero
  pose. Base/work are `world`; the tool is `tool0`, electrode axis +X.

## spheres.json

The collision spheres, and the sha256 of the meshes and URDF they were fitted
from (96 clusters per link, every triangle covered). Refit with
`provenance/fit_spheres.py`; `provenance/test_description.py` checks the
identity, the gearing and the sphere cover.

## provenance/

The sphere fit and its checks. Nothing here is registered in
`robot_description_manifest.json`, so editing it never changes the identity.
