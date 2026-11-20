# sphere tool

One page. It authors two things a cell needs and nothing else produces:

- **`robot_description/<cell>/spheres.json`**, the collision model
  `weld_motion_planner.robot_cell.collision.spheres` reads and the motion
  search collides against. It is part of the robot's description
  (`docs/ROBOT-DESCRIPTION-CONTRACT.md`) and carries the hashes of the URDF and
  meshes it was fitted from, so an export has to be re-stamped:
  `pixi run -e motion python tools/fit_spheres.py --cell <cell> --restamp`.
- **the joint limits in `robot_description/<cell>/robot.urdf`**, found by driving each
  joint to where the arm stops rather than typed at it.

```bash
python3 tools/sphere_tool/serve.py        # then open the URL it prints
```

`serve.py` reads cells out of `robot_description/<cell>/robot.urdf` and serves the built
page from `dist/`. To work on the page itself, `npm install` once and then
`npm run dev` **beside** `serve.py`, which Vite proxies `/api`, `/cell-assets`
and `/cell-urdf` to.

Pass a cell id to open it first: `python3 serve.py 8795 rosie_1420_v1`. The page
opens whichever cell leads the list and loads that cell's committed sphere
model if the repo has one.

## why it lives here

It used to be one of five pages under `weld_planner/v1/ui`, served by the weld
planner's authoring server. D24 deletes that server and the pages with it,
correctly for the other four: they were its front end. This one is not. It
produces committed assets, and the planner refuses to run without one of them:

    cell 'rosie_1420_v1' has no sphere model; author one with
    tools/sphere_tool and save it as robot_description/robots/rosie_1420_v1/spheres.json

It reads every bundle under `robot_description/robots`, and since it gained
limit editing it writes one back, but it sits in `tools/` rather than beside
them: the deploy planner treats every path under `robot_description/` as
runtime configuration, and an edit to this page is not a robot change. It
also reads `weld_planner/v1`'s URDF parser by file (`serve.py`).

`src/kinematics.ts` is a COPY of the module of the same name in
`weld_planner/v1/ui/src`, not an import of it: GOAL.md forbids reaching into
another component's code, and forward kinematics over a URDF-derived cell
descriptor is a `robot_description` concern anyway. The copy over there could not simply
be deleted, because `viewer.ts` and `graph-pose.ts` import it too -- but D24
deletes that whole tree, so the duplication lasts only until it lands. The two
are byte-identical today; while both exist, a fix to one belongs in the other.

## what it does

- **fit** a link's spheres automatically (cluster / boxes / hierarchy, or
  `auto` to try all three and keep the best), then **refine**
- **move and resize** one sphere at a time with a gizmo, or nudge with arrows
- **frame axes** on every frame the cell declares, in either view. A cell
  declares more frames than it draws: `tool0` and `end_effector` carry no mesh
  at all, and a link with geometry still has an origin its spheres are measured
  from. None of that is visible in a shaded render, which is how
  `rosie_1420_v1` shipped with a tool frame 56 mm off the gun it was supposed
  to be the tip of while every picture of the robot looked right.
- **single link** frames one link in its own coordinates, which is where
  spheres are authored. **full robot** poses the whole cell through forward
  kinematics with joint sliders, which is where a model gets checked against
  the arm it belongs to.
- **joint limits**, in full-robot view. Each joint is three lines: name and
  angle, the slider at full width, then its bounds under the ends of the track
  they clamp. The arrows take a bound from where the joint is posed. **Download
  URDF** rewrites the cell's own `robot.urdf`: the text comes back whole and
  exactly the `lower`/`upper` of the joints you changed move, so meshes,
  materials, transmissions and the comments that say why a number is what it is
  all survive.

  A URDF is not a local file. `robot.urdf` is copied byte-for-byte into
  `realtime-core/natsbridge/<cell>_factory.urdf`, embedded in the pair owner
  with `//go:embed`, and hashed: `pairConfigFactoryURDFSHA256` pins it, the
  pair's canonical snapshot carries the limits themselves, and the home profile
  key is derived from both. Change it and all three move, and a cell running
  that pair needs its revision re-applied. The tool says so when it exports.
- **Download JSON** writes the sphere asset. Only links that HAVE spheres are
  written, so hand-authored links merge with ones seeded from another cell.

## the fit over-covers on purpose, and that once blinded the screen

The radius step sets each sphere to cover the surface voxels assigned to it,
which on a link that is two regimes at once produces a sphere standing well off
the body. On `link_6` -- a bulky wrist casting plus a 150 mm, 15 mm-radius
torch tube -- that put 47 mm of radius around a 15 mm tube, and with the tool
origin on the weld point the screen rejected 100% of weld points.
`weld_planner/v1/docs/issues/0006`. `weld_planner/v1/tools/fit_spheres.py` is
the answer to that one: it fits by interior depth, so every sphere is
inscribed, and it splices the analytic torch from `tooling.json` over the tip
zone. Read it before re-fitting `link_6` by hand.

Coverage is not a safety argument either way. Under-cover and the search misses
a contact, but the CCD verifier re-checks the winner against exact meshes, so
what ships is still sound; what suffers is the search wasting itself in basins
that fail at the end. Over-cover and plans get conservative. The coverage report
travels with the asset so `spheres.problems()` can say which links are worth
another pass.
