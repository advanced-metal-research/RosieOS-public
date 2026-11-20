import assert from "node:assert/strict";
import test from "node:test";

import { adaptCadQueryTopology } from "../adapters/cadquery.ts";
import { isSelectableTopologyEdge } from "../domain/edge-catalog.ts";
import { DOG_BOX_FALLBACK_TOPOLOGY } from "../examples/dog-box.ts";
import { createProject } from "../domain/project.ts";
import { transformWorkpiecePoint } from "../domain/placement.ts";
import {
  EDGE_HOVER_RADIUS_M,
  EDGE_PICK_RADIUS_M,
  EDGE_SELECTED_RADIUS_M,
  EDGE_STATE_STYLE,
  WELD_ENDPOINT_RADIUS_M,
  WELD_START_COLOR,
  WELD_STOP_COLOR,
  edgeVisualState,
  pointerNDC,
  type EdgeSelectionState,
} from "./edge-picking.ts";
import { projectCadEdgeOverlays, projectFramePoints } from "./workstation.ts";

test("edge visual state priority is active > selected > hover > default", () => {
  const edge = { id: "edge-1" };
  const state = (overrides: Partial<EdgeSelectionState>): EdgeSelectionState => ({
    mode: true,
    selectedIds: new Set<string>(),
    activeId: null,
    ...overrides,
  });
  assert.equal(
    edgeVisualState(edge, state({ selectedIds: new Set(["edge-1"]), activeId: "edge-1" }), "edge-1"),
    "active",
  );
  assert.equal(
    edgeVisualState(edge, state({ selectedIds: new Set(["edge-1"]) }), null),
    "selected",
  );
  assert.equal(edgeVisualState(edge, state({}), "edge-1"), "hover");
  // Hover styling only exists inside edge-selection mode. No edge gets a
  // suggestion accent: STEP edges all present alike until the user acts.
  assert.equal(
    edgeVisualState(edge, state({ mode: false }), "edge-1"),
    "default",
  );
  assert.equal(edgeVisualState(edge, state({}), null), "default");
  // An active id that is no longer selected must not read as active.
  assert.equal(
    edgeVisualState(edge, state({ activeId: "edge-1" }), null),
    "default",
  );
  // Visible tubes match GradientOS; pick stays cell-scale hittable.
  assert.equal(EDGE_PICK_RADIUS_M, 0.004);
  assert.equal(EDGE_HOVER_RADIUS_M, 0.0005);
  assert.equal(EDGE_SELECTED_RADIUS_M, 0.0006);
  assert.equal(WELD_ENDPOINT_RADIUS_M, 0.0075);
  // Shop grey: start in the confirmed green, stop in the fault red.
  assert.equal(WELD_START_COLOR, 0x2a6238);
  assert.equal(WELD_STOP_COLOR, 0xa8231c);
  for (const style of Object.values(EDGE_STATE_STYLE)) {
    assert.ok(style.color >= 0 && style.opacity > 0 && style.renderOrder >= 11);
  }
});

test("face-wire composites are not selectable topology edges", () => {
  assert.equal(isSelectableTopologyEdge({ geometryType: "LINE", pathClass: "raw_fragment" }), true);
  assert.equal(isSelectableTopologyEdge({ geometryType: "CIRCLE", pathClass: "circle" }), true);
  assert.equal(isSelectableTopologyEdge({ geometryType: "WIRE", pathClass: "raw_fragment" }), false);
  assert.equal(
    isSelectableTopologyEdge({ geometryType: "LINE", pathClass: "wire_non_seam" }),
    false,
  );
  assert.equal(
    isSelectableTopologyEdge({ geometryType: "WIRE", pathClass: "dogbone_4arc_4line" }),
    false,
  );
});

test("pointer NDC maps the pane rectangle to [-1, 1] and rejects outside points", () => {
  const rect = { left: 100, top: 50, width: 200, height: 100 };
  assert.deepEqual(pointerNDC(200, 100, rect), { x: 0, y: 0 });
  assert.deepEqual(pointerNDC(100, 50, rect), { x: -1, y: 1 });
  assert.deepEqual(pointerNDC(300, 150, rect), { x: 1, y: -1 });
  assert.equal(pointerNDC(99, 100, rect), null);
  assert.equal(pointerNDC(200, 151, rect), null);
  assert.equal(pointerNDC(0, 0, { left: 0, top: 0, width: 0, height: 0 }), null);
});

test("project edge overlays place catalog samples through the fixture transform", () => {
  const project = adaptCadQueryTopology(createProject(), DOG_BOX_FALLBACK_TOPOLOGY);
  const overlays = projectCadEdgeOverlays(project);
  assert.equal(overlays.length, DOG_BOX_FALLBACK_TOPOLOGY.edges.length);
  const first = overlays[0]!;
  const catalogEdge = project.edgeCatalog[0]!;
  assert.equal(first.id, catalogEdge.id);
  assert.deepEqual(
    first.pointsM[0],
    transformWorkpiecePoint(project, catalogEdge.samplesM[0]!),
  );
  // Every edge surfaces as plain pickable geometry — no suggestion accents.
  const backRight = overlays.find((overlay) => overlay.id === "dog-box:back-right")!;
  assert.ok(backRight.pointsM.length >= 2);
});

test("empty program frames the CAD edges so Frame program zooms to the part", () => {
  const project = adaptCadQueryTopology(createProject(), DOG_BOX_FALLBACK_TOPOLOGY);
  const framed = projectFramePoints(project);
  assert.ok(framed.length >= 2);
  assert.equal(framed[0]?.motion, "travel");
});

test("project edge overlays omit face-wire composites so a click is one line", () => {
  const project = adaptCadQueryTopology(createProject(), DOG_BOX_FALLBACK_TOPOLOGY);
  const line = project.edgeCatalog[0]!;
  project.edgeCatalog = [
    ...project.edgeCatalog,
    {
      ...line,
      id: "part_0:wire_00000",
      geometryType: "WIRE",
      pathClass: "wire_non_seam",
    },
  ];
  const overlays = projectCadEdgeOverlays(project);
  assert.equal(overlays.some((overlay) => overlay.id === "part_0:wire_00000"), false);
  assert.equal(overlays.length, DOG_BOX_FALLBACK_TOPOLOGY.edges.length);
});
