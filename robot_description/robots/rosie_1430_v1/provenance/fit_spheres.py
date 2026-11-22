#!/usr/bin/env python3
"""Fit the Rosie 1430 meshes without borrowing another robot's torch or spheres.

Run from any directory with numpy, scipy and pycollada installed. Cluster small
triangles and enclose each cluster's vertices in a sphere. Convexity guarantees
that each assigned triangle is covered. Final motion still requires the
exact-mesh continuous collision verifier. This conservative search model is
not physical qualification or a replacement for exact collision geometry.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np

MODEL = Path(__file__).resolve().parents[1]
ROOT = MODEL.parents[2]
sys.path.insert(0, str(ROOT / "weld_planner/v1/python"))
sys.path.insert(0, str(ROOT / "weld_planner/v1/tools"))
from scipy.cluster.vq import kmeans2
from weld_motion_planner.verifier.verify import load_dae
from weldplan.robot_description import mesh_sha256

def subdivide(vertices, faces, edge_limit):
    # Subdivision changes sampling only, never the source geometry.
    triangles = vertices[faces]
    finished = []
    while len(triangles):
        edges = np.stack([triangles[:, 1] - triangles[:, 0],
                          triangles[:, 2] - triangles[:, 1],
                          triangles[:, 0] - triangles[:, 2]], axis=1)
        longest = np.max(np.linalg.norm(edges, axis=2), axis=1)
        done = longest <= edge_limit
        finished.append(triangles[done])
        t = triangles[~done]
        if not len(t):
            break
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        triangles = np.concatenate([np.stack(x, axis=1) for x in
                                    [(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)]])
    return np.concatenate(finished)


def main():
    raw = (MODEL / "robot.urdf").read_bytes()
    assert hashlib.sha256(raw).hexdigest() == "ae401a9385060c9595f415564b84b3ff9f6a9136b88c4025b7f434aefb4c1106"
    document = {
        "schema": "amr-weld-planner-v1.sphere-model.v1", "cell_id": "rosie_1430_v1",
        "note": "Conservative triangle-cluster cover of the Rosie 1430 collision meshes. No foreign spheres or analytic torch substitution. Exact mesh verification remains mandatory; conservative rejection rate is not yet qualified.",
        "links": {}, "coverage": {},
        "provenance": {
            "robot_urdf_sha256": hashlib.sha256(raw).hexdigest(),
            "meshes_sha256": mesh_sha256(MODEL),
            "method": "triangles subdivided to 20 mm maximum edge; deterministic kmeans++ (seed 1430), 96 clusters/link; radius encloses every assigned triangle vertex plus 2 micrometres for rounding; no excluded torch region",
        },
    }
    for i in range(1, 7):
        name = f"link_{i}"
        mesh = load_dae(MODEL / "meshes" / (name + ".dae"))
        triangles = subdivide(mesh.vertices, mesh.faces, 0.020)
        centres, labels = kmeans2(triangles.mean(axis=1), 96, iter=40,
                                 minit="++", seed=1430, missing="raise")
        fitted = []
        maximum_gap = -float("inf")
        for index, centre in enumerate(centres):
            points = triangles[labels == index].reshape(-1, 3)
            if not len(points):
                raise ValueError(f"{name}: empty sphere cluster")
            radius = float(np.max(np.linalg.norm(points - centre, axis=1))) + 0.000002
            sphere = {"center_m": [round(float(v), 6) for v in centre],
                      "radius_m": round(radius, 6)}
            maximum_gap = max(maximum_gap, float(np.max(np.linalg.norm(
                points - sphere["center_m"], axis=1) - sphere["radius_m"])))
            fitted.append(sphere)
        if maximum_gap > 0:
            raise ValueError(f"{name}: assigned triangle escapes sphere by {maximum_gap} m")
        document["links"][name] = fitted
        document["coverage"][name] = {
            "uncovered_fraction": 0.0, "worst_uncovered_m": 0.0,
            "assigned_triangle_vertex_max_gap_m": maximum_gap,
            "subdivided_triangles": len(triangles),
        }
        print(name, len(fitted), document["coverage"][name], flush=True)
    (MODEL / "spheres.json").write_text(json.dumps(document, indent=2) + "\n")


if __name__ == "__main__":
    main()
