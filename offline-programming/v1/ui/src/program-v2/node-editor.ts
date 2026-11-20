/**
 * The parameter dialog: one node, every field, Save or Cancel.
 *
 * A native `<dialog>` -- focus trapping, Escape and the backdrop come free.
 * The body is a section nav on the left and the parameter table on the
 * right, the way a controller's parameter window is laid out, so a weld's
 * thirty fields are reached by heading and not by scrolling. Everything
 * edits a DRAFT: nothing touches the project until Save, and Save is
 * refused with the reasons while the contract has any. Cancel and Escape
 * discard the draft.
 *
 * A classic weld's dialog also carries its source seam (trim, reverse).
 * Those are project edits, not node fields, so they travel beside the node
 * to `deps.commit` and the host writes both in one revision.
 */

import type { ProgramV2Node, ScalarFunction, WeldNode } from "../contracts/robot-v4-program-v2.ts";
import type { AuthoredSeam, OfflineProgrammingProject } from "../domain/project.ts";
import { polylineLengthM } from "../domain/seam.ts";
import { el, replace } from "../rails/dom.ts";
import { createSplineEditor } from "../rails/spline-editor.ts";
import {
  type FieldGroup,
  type FieldSpec,
  type SpecContext,
  applyEdit,
  beneathOf,
  groupsForNode,
  nodeProblems,
} from "./field-specs.ts";
import { fieldForm, type FormModel } from "./field-form.ts";
import { classicProvenance, classicWeldPoses } from "./from-project.ts";
import { isWeld, nodeById } from "./nodes.ts";
import { moveProfileChart, tcpProfileChart, type PlannedMoveProfile } from "./move-profile.ts";

const GEOMETRY_KIND_LABELS: Record<WeldNode["geometry"]["kind"], string> = {
  seam: "rails seam",
  posed_polyline: "posed polyline",
  manual_pose_wp: "hand-placed poses",
};

export interface SeamEdits {
  seamId: string;
  trimStartM: number;
  trimEndM: number;
  reversed: boolean;
}

export interface NodeEditorDeps {
  getProject(): OfflineProgrammingProject;
  /** Every problem the whole document would have with `node` in place of
   *  its current version. The line rule lives here, not in the node. */
  documentProblems(node: ProgramV2Node): string[];
  /** Write the node (and the seam edits, when the weld has a source seam)
   *  as one revision. Throws with the reason. */
  commit(node: ProgramV2Node, seam: SeamEdits | null): void;
  status(text: string): void;
  motionProfile(nodeId: string): Promise<PlannedMoveProfile | null>;
  plan(): Promise<void>;
}

export interface NodeEditor {
  open(nodeId: string): void;
  close(): void;
  isOpen(): boolean;
}

export function createNodeEditor(dialog: HTMLDialogElement, deps: NodeEditorDeps): NodeEditor {
  let draft: ProgramV2Node | null = null;
  let opened: ProgramV2Node | null = null;
  let seamDraft: SeamEdits | null = null;
  let seamOpened: SeamEdits | null = null;
  let context: SpecContext = { defaults: { clearance_min_mm: 20, speed_scale: 1, torch_policy: "free", positioner_policy: "free" }, classicSeam: false };
  let section = "";
  let refused: string | null = null;
  let profile: PlannedMoveProfile | null = null;
  let profileStatus = "";
  let profileGeneration = 0;

  dialog.addEventListener("click", (event) => {
    if (event.target === dialog) dialog.close();
  });
  dialog.addEventListener("close", () => {
    profileGeneration++;
    draft = null;
    opened = null;
    seamDraft = null;
    seamOpened = null;
    refused = null;
  });

  function classicSeam(): AuthoredSeam | null {
    if (!draft || !isWeld(draft)) return null;
    const provenance = classicProvenance(draft);
    if (!provenance) return null;
    return deps.getProject().seams.find((seam) => seam.id === provenance.seam_id) ?? null;
  }

  function open(nodeId: string): void {
    const project = deps.getProject();
    const node = nodeById(project.nodes, nodeId);
    if (!node) {
      deps.status(`node ${nodeId} is not in the program`);
      return;
    }
    opened = structuredClone(node);
    draft = structuredClone(node);
    const seam = classicSeam();
    seamOpened = seam
      ? { seamId: seam.id, trimStartM: seam.trim.startOffsetM, trimEndM: seam.trim.endOffsetM, reversed: seam.reversed }
      : null;
    seamDraft = seamOpened ? { ...seamOpened } : null;
    context = { defaults: { ...project.programV2.defaults }, classicSeam: seam !== null };
    section = draft.op === "move" ? "motion" : groupsForNode(draft, context)[0]?.id ?? "";
    profile = null;
    profileStatus = "Loading planned motion…";
    const generation = ++profileGeneration;
    refused = null;
    render();
    if (!dialog.open) dialog.showModal();
    if (node.op === "move") void deps.motionProfile(nodeId).then(result => {
      if (generation !== profileGeneration) return;
      profile = result;
      profileStatus = result ? "" : "Plan this move to see its joint velocity and acceleration profiles.";
      render();
    }).catch(error => {
      if (generation !== profileGeneration) return;
      profileStatus = `Profile unavailable: ${error instanceof Error ? error.message : String(error)}`;
      render();
    });
  }

  function problems(): string[] {
    if (!draft) return [];
    const own = nodeProblems(draft);
    const whole = own.length ? [] : deps.documentProblems(draft);
    const seam = seamDraft && (seamDraft.trimStartM < 0 || seamDraft.trimEndM < 0) ? ["trims must be non-negative"] : [];
    return [...own, ...whole, ...seam];
  }

  function dirty(): boolean {
    return JSON.stringify(draft) !== JSON.stringify(opened) || JSON.stringify(seamDraft) !== JSON.stringify(seamOpened);
  }

  /** Recompute a CAD-derived weld's points from its seam with the draft
   *  trims and direction applied, so the dialog shows what Save will do. */
  function regeneratePoints(): void {
    if (!draft || !isWeld(draft) || !seamDraft) return;
    const provenance = classicProvenance(draft);
    if (!provenance?.cad_derived) return;
    const project = structuredClone(deps.getProject());
    const seam = project.seams.find((candidate) => candidate.id === seamDraft!.seamId);
    const step = project.sequence.find((candidate) => candidate.seamId === seamDraft!.seamId);
    if (!seam || !step) return;
    seam.trim = { startOffsetM: seamDraft.trimStartM, endOffsetM: seamDraft.trimEndM };
    seam.reversed = seamDraft.reversed;
    const points = classicWeldPoses(project, seam, step);
    if (points.length >= 2 && draft.geometry.kind === "posed_polyline") draft.geometry.points = points;
  }

  const model: FormModel = {
    get resolved() { return resolved(draft!) as unknown as Record<string, unknown>; },
    get beneath() { return beneathOf(resolved(opened!), context); },
    readVirtual(path) {
      const source = path === "_seam.trimStartMm" || path === "_seam.trimEndMm" || path === "_seam.reversed" ? seamDraft : null;
      if (!source) return undefined;
      if (path === "_seam.trimStartMm") return source.trimStartM * 1000;
      if (path === "_seam.trimEndMm") return source.trimEndM * 1000;
      return source.reversed;
    },
    set(spec, value) {
      if (!draft) return;
      if (spec.path.startsWith("_seam.")) {
        if (!seamDraft) throw new Error("this weld has no source seam");
        const seam = classicSeam();
        const lengthM = seam ? polylineLengthM(seam.geometry.pointsM) : Number.POSITIVE_INFINITY;
        if (spec.path === "_seam.reversed") seamDraft.reversed = Boolean(value);
        else {
          const metres = Number(value) / 1000;
          if (!Number.isFinite(metres) || metres < 0) throw new Error(`${spec.label} must be a non-negative length`);
          const other = spec.path === "_seam.trimStartMm" ? seamDraft.trimEndM : seamDraft.trimStartM;
          if (metres + other >= lengthM) throw new Error(`${spec.label} leaves nothing of the seam to weld`);
          if (spec.path === "_seam.trimStartMm") seamDraft.trimStartM = metres;
          else seamDraft.trimEndM = metres;
        }
        regeneratePoints();
        return;
      }
      applyEdit(draft, spec, value);
    },
    profileEditor(spec, value, onChange) {
      const editor = createSplineEditor({ label: spec.label, unit: spec.unit ?? "", min: -90, max: 90, zero: 0, onChange });
      editor.set(value, "node");
      if (draft && isWeld(draft) && draft.geometry.kind === "seam") {
        const axis = spec.path.endsWith("work_angle_deg") ? "work_angle_deg" : "travel_angle_deg";
        const limits = (draft.geometry.orientation.limits as Record<string, { min?: ScalarFunction; max?: ScalarFunction }> | undefined)?.[axis];
        if (limits) editor.setCorridor(limits.min, limits.max);
      }
      return editor.root;
    },
    onEdited() {
      refused = null;
      render();
    },
    onRefused(spec, reason) {
      refused = `${spec.label}: ${reason}`;
      render();
    },
  };

  function save(): void {
    if (!draft || refused) return;
    const found = problems();
    if (found.length) {
      refused = found[0]!;
      render();
      return;
    }
    try {
      deps.commit(structuredClone(draft), seamDraft && JSON.stringify(seamDraft) !== JSON.stringify(seamOpened) ? { ...seamDraft } : null);
      dialog.close();
    } catch (error) {
      refused = error instanceof Error ? error.message : String(error);
      render();
    }
  }

  function resolved(node: ProgramV2Node): ProgramV2Node {
    return node.op === "move" ? { ...node, acceleration_scale: node.acceleration_scale ?? node.speed_scale, constant_tcp_speed: node.constant_tcp_speed ?? true } : node;
  }

  async function saveAndPlan(): Promise<void> {
    if (!draft || refused || problems().length) return;
    const id = draft.id;
    const projectId = deps.getProject().identity.id;
    try {
      if (dirty()) deps.commit(structuredClone(draft), null);
      dialog.close();
      await deps.plan();
      if (deps.getProject().identity.id === projectId && !dialog.open) open(id);
    } catch (error) {
      deps.status(error instanceof Error ? error.message : String(error));
    }
  }

  function render(): void {
    if (!draft) return;
    const groups = groupsForNode(draft, context);
    const active = groups.find((group) => group.id === section) ?? groups[0]!;
    section = active.id;
    const found = problems();
    const blocked = found[0] ?? null;
    const geometryKind = isWeld(draft) ? draft.geometry.kind : null;
    const form = fieldForm([active], model);
    // The table's own scroll host keeps its position across a re-render
    // when the same section is drawn again.
    const previous = dialog.querySelector<HTMLElement>(".fields");
    const scrollTop = previous?.scrollTop ?? 0;
    replace(
      dialog,
      el(
        "form",
        { method: "dialog", class: "modal node-editor" },
        el(
          "header",
          {},
          el("span", { class: "op-badge", data: { op: draft.op } }, draft.op),
          el("h2", {}, draft.label || draft.id),
          el("span", { class: "muted" }, geometryKind ? `${GEOMETRY_KIND_LABELS[geometryKind]} · ` : "", draft.id),
          el("button", { class: "plus", type: "button", value: "close", title: "cancel", on: { click: () => dialog.close() } }, "×"),
        ),
        el(
          "div",
          { class: "node-editor-body" },
          el("nav", { class: "sections", "aria-label": "Parameter sections" },
            groups.filter(group => group.id !== "joint-limits").map((group: FieldGroup) => el("button", {
              type: "button",
              class: group.id === active.id ? "is-active" : "",
              data: { section: group.id },
              on: { click: () => { section = group.id; render(); } },
            }, group.group))),
          el("div", { class: "fields" }, form,
            draft.op === "move" && section === "motion" ? el("div", { class: "move-profile-host" },
              profileStatus ? el("p", { class: "muted", role: "status" }, profileStatus) : null,
              profile && (dirty() || !profile.current) ? el("p", { class: "move-profile-stale", role: "status" }, "Previous plan — save and replan to update this profile.") : null,
              profile ? tcpProfileChart(profile) : null,
              fieldForm(groups.filter(group => group.id === "joint-limits"), model),
              profile ? moveProfileChart(profile) : null) : null),
        ),
        el(
          "footer",
          {},
          el("div", { class: `problems ${blocked || refused ? "has-problems" : ""}` },
            refused ? el("div", { class: "error" }, refused) : null,
            found.map((line) => el("div", { class: "error" }, line)),
            !blocked && !refused ? el("span", { class: "muted" }, dirty() ? "unsaved changes" : "no changes") : null),
          el("span", { class: "spacer" }),
          el("button", { type: "button", class: "button secondary", on: { click: () => dialog.close() } }, "Cancel"),
          draft.op === "move" ? el("button", { type: "button", class: "button secondary",
            disabled: Boolean(blocked || refused), on: { click: () => { void saveAndPlan(); } },
          }, dirty() ? "Save & plan" : "Plan profile") : null,
          el("button", {
            type: "button",
            class: "button primary",
            disabled: Boolean(blocked || refused) || !dirty(),
            title: blocked ?? (dirty() ? "apply to the node and recompile its rows" : "nothing changed"),
            on: { click: save },
          }, "Save"),
        ),
      ),
    );
    const next = dialog.querySelector<HTMLElement>(".fields");
    if (next) next.scrollTop = scrollTop;
  }

  return {
    open,
    close: () => { if (dialog.open) dialog.close(); },
    isOpen: () => dialog.open,
  };
}

export type { FieldSpec };
