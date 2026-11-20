"""A box as COLLADA, in metres, Z up -- the same shape `load_dae` reads for
every other link of this cell."""
import sys
import numpy as np
import collada

HALF_X, HALF_Y, HEIGHT = 0.25, 0.25, 1.0   # 0.5 x 0.5 in plan, 1.0 tall
name, out = sys.argv[1], sys.argv[2]
# Origin at the centre of the BOTTOM face, so the fixed joint that places it
# names the point it stands on and the top face is `HEIGHT` above that.
xs, ys, zs = (-HALF_X, HALF_X), (-HALF_Y, HALF_Y), (0.0, HEIGHT)
verts = np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=np.float32)
# (x, y, z) index bits: 4*i + 2*j + k
def v(i, j, k): return 4 * i + 2 * j + k
faces = []
for (a, b, c, d) in (
    (v(0,0,0), v(0,1,0), v(1,1,0), v(1,0,0)),   # bottom
    (v(0,0,1), v(1,0,1), v(1,1,1), v(0,1,1)),   # top
    (v(0,0,0), v(1,0,0), v(1,0,1), v(0,0,1)),   # -y
    (v(0,1,0), v(0,1,1), v(1,1,1), v(1,1,0)),   # +y
    (v(0,0,0), v(0,0,1), v(0,1,1), v(0,1,0)),   # -x
    (v(1,0,0), v(1,1,0), v(1,1,1), v(1,0,1)),   # +x
):
    faces += [[a, b, c], [a, c, d]]
faces = np.asarray(faces, dtype=np.int32)

mesh = collada.Collada()
mesh.assetInfo.upaxis = collada.asset.UP_AXIS.Z_UP
mesh.assetInfo.unitmeter, mesh.assetInfo.unitname = 1.0, "meter"
source = collada.source.FloatSource(f"{name}-pos", verts.reshape(-1), ("X", "Y", "Z"))
geom = collada.geometry.Geometry(mesh, f"geom-{name}", name, [source])
inputs = collada.source.InputList()
inputs.addInput(0, "VERTEX", f"#{name}-pos")
geom.primitives.append(geom.createTriangleSet(faces.reshape(-1), inputs, "ref"))
mesh.geometries.append(geom)
node = collada.scene.Node(name, children=[collada.scene.GeometryNode(geom, [])])
scene = collada.scene.Scene("scene", [node])
mesh.scenes.append(scene); mesh.scene = scene
mesh.write(out)
print(f"{out}: {len(verts)} vertices, {len(faces)} triangles, "
      f"x {xs} y {ys} z {zs}")
