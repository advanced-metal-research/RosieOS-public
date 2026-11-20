import { WELD_TYPES, type ProjectWeld } from "../adapters/weld-authoring.ts";
import { catalogEdgeById } from "../domain/edge-catalog.ts";
import type { OfflineProgrammingProject, WeldType } from "../domain/project.ts";
import { polylineLengthM } from "../domain/seam.ts";

/**
 * Weld-from-edges drawer (P2.12), following the GradientOS WeldPanel model:
 * an additive multi-edge draft with per-segment normalized start/end trims
 * (mm estimates alongside), weld type/name, torch angle defaults, an active
 * segment, and segment removal. Nothing exists in the program until
 * "Create weld"; existing welds are listed with their delete action.
 */

export interface WeldDraftSegmentUIState {
  edgeId: string;
  startS: number;
  endS: number;
  reversed: boolean;
}

export interface WeldDraftUIState {
  name: string;
  weldType: WeldType;
  segments: WeldDraftSegmentUIState[];
  activeEdgeId: string | null;
  workAngleDeg: number;
  travelAngleDeg: number;
  spinAngleDeg: number;
}

export type WeldAngleField = "workAngleDeg" | "travelAngleDeg" | "spinAngleDeg";

export interface WeldDrawerHandlers {
  onToggleMode: () => void;
  onNameChange: (name: string) => void;
  onTypeChange: (weldType: WeldType) => void;
  onAngleChange: (field: WeldAngleField, valueDeg: number) => void;
  onSegmentTrim: (edgeId: string, side: "startS" | "endS", value: number) => void;
  onSegmentReverse: (edgeId: string) => void;
  onSegmentActivate: (edgeId: string) => void;
  onSegmentRemove: (edgeId: string) => void;
  onCreateWeld: () => void;
  onClearDraft: () => void;
  onDeleteWeld: (weldId: string) => void;
}

export interface WeldDrawerInput {
  project: OfflineProgrammingProject;
  draft: WeldDraftUIState | null;
  edgeSelectionMode: boolean;
  welds: readonly ProjectWeld[];
}

export function isWeldType(value: unknown): value is WeldType {
  return typeof value === "string" && (WELD_TYPES as readonly string[]).includes(value);
}

/** Start a draft (or add to it) from a clicked edge — the additive reducer. */
export function draftWithEdge(
  draft: WeldDraftUIState | null,
  edgeId: string,
): WeldDraftUIState {
  const segments = draft?.segments ?? [];
  const alreadySelected = segments.some((segment) => segment.edgeId === edgeId);
  const weldType = draft?.weldType ?? "fillet";
  return {
    name: draft?.name ?? "",
    weldType,
    segments: alreadySelected
      ? segments
      : [...segments, { edgeId, startS: 0, endS: 1, reversed: false }],
    activeEdgeId: edgeId,
    workAngleDeg: draft?.workAngleDeg ?? 45,
    travelAngleDeg: draft?.travelAngleDeg ?? 10,
    spinAngleDeg: draft?.spinAngleDeg ?? 0,
  };
}

/** Remove a segment; the draft dissolves when its last segment goes. */
export function draftWithoutEdge(
  draft: WeldDraftUIState,
  edgeId: string,
): WeldDraftUIState | null {
  const segments = draft.segments.filter((segment) => segment.edgeId !== edgeId);
  if (segments.length === 0) return null;
  return {
    ...draft,
    segments,
    activeEdgeId: draft.activeEdgeId === edgeId
      ? segments[0]!.edgeId
      : draft.activeEdgeId,
  };
}

export function renderWeldDrawer(
  host: HTMLElement,
  input: WeldDrawerInput,
  handlers: WeldDrawerHandlers,
): void {
  const { project, draft, edgeSelectionMode, welds } = input;
  const hasCatalog = (project.edgeCatalog ?? []).length > 0;
  host.hidden = !hasCatalog && welds.length === 0 && !draft;
  if (host.hidden) {
    host.innerHTML = "";
    return;
  }

  host.innerHTML = `
    <div class="weld-drawer-heading">
      <div>
        <p class="eyebrow">WELDS</p>
        <h3>Define welds from edges</h3>
      </div>
      <button
        id="toggle-edge-select"
        class="mini-button ${edgeSelectionMode ? "is-active" : ""}"
        type="button"
        data-weld-action="toggle-mode"
        aria-pressed="${edgeSelectionMode}"
        ${hasCatalog ? "" : "disabled"}
        title="Click part edges in the viewport to add them to the weld draft"
      >${edgeSelectionMode ? "EDGE SELECT ON" : "SELECT EDGES"}</button>
    </div>
    ${draft ? draftSection(project, draft) : emptyDraftSection(edgeSelectionMode, hasCatalog, welds.length > 0)}
    ${weldListSection(welds)}
  `;

  host.onclick = (event) => {
    const target = event.target as HTMLElement;
    const button = target.closest<HTMLButtonElement>("[data-weld-action]");
    if (!button) return;
    const action = button.dataset.weldAction!;
    const edgeId = button.dataset.edgeId ?? "";
    if (action === "toggle-mode") handlers.onToggleMode();
    else if (action === "activate-segment") handlers.onSegmentActivate(edgeId);
    else if (action === "remove-segment") handlers.onSegmentRemove(edgeId);
    else if (action === "reverse-segment") handlers.onSegmentReverse(edgeId);
    else if (action === "create-weld") handlers.onCreateWeld();
    else if (action === "clear-draft") handlers.onClearDraft();
    else if (action === "delete-weld") handlers.onDeleteWeld(button.dataset.weldId ?? "");
  };
  host.onchange = (event) => applyWeldField(event.target as HTMLElement, handlers);
  host.oninput = (event) => {
    const target = event.target as HTMLElement;
    const field = (target as HTMLInputElement).dataset?.weldField;
    if (field === "startS" || field === "endS") applyWeldField(target, handlers);
  };
}

function applyWeldField(target: HTMLElement, handlers: WeldDrawerHandlers): void {
  const field = (target as HTMLInputElement | HTMLSelectElement).dataset.weldField;
  if (!field) return;
  if (!(target instanceof HTMLInputElement) && !(target instanceof HTMLSelectElement)) return;
  if (field === "name") handlers.onNameChange(target.value);
  else if (field === "weldType" && isWeldType(target.value)) {
    handlers.onTypeChange(target.value);
  } else if (
    field === "workAngleDeg" || field === "travelAngleDeg" || field === "spinAngleDeg"
  ) {
    const value = Number(target.value);
    if (Number.isFinite(value)) handlers.onAngleChange(field, value);
  } else if (field === "startS" || field === "endS") {
    const edgeId = target.dataset.edgeId ?? "";
    const value = Number(target.value);
    if (Number.isFinite(value)) handlers.onSegmentTrim(edgeId, field, value);
  }
}

export function weldSegmentTrimLabel(
  project: OfflineProgrammingProject,
  segment: WeldDraftSegmentUIState,
): string {
  const edge = catalogEdgeById(project, segment.edgeId);
  const lengthM = edge ? polylineLengthM(edge.samplesM) : 0;
  const startS = Math.min(segment.startS, segment.endS);
  const endS = Math.max(segment.startS, segment.endS);
  return `${(startS * lengthM * 1000).toFixed(1)} – ${(endS * lengthM * 1000).toFixed(1)} / ${(lengthM * 1000).toFixed(1)} mm`;
}

export function updateWeldTrimReadout(
  host: HTMLElement,
  project: OfflineProgrammingProject,
  draft: WeldDraftUIState,
  edgeId: string,
): void {
  const segment = draft.segments.find((entry) => entry.edgeId === edgeId);
  if (!segment) return;
  const readout = host.querySelector(`[data-weld-trim-readout="${cssAttr(edgeId)}"]`);
  if (readout) readout.textContent = weldSegmentTrimLabel(project, segment);
}

function cssAttr(value: string): string {
  return value.replace(/\\/g, "\\\\").replace(/"/g, '\\"');
}

function emptyDraftSection(
  edgeSelectionMode: boolean,
  hasCatalog: boolean,
  hasWelds: boolean,
): string {
  if (!hasCatalog) {
    return `<p class="weld-drawer-note">${
      hasWelds
        ? "This program's welds predate pickable edges. Re-import its STEP file to select edges."
        : "Import a STEP part to get pickable edges."
    }</p>`;
  }
  return `<p class="weld-drawer-note">${
    edgeSelectionMode
      ? "Click part edges in the viewport to add them to a new weld."
      : "Turn on SELECT EDGES, then click part edges to draft a weld. Nothing is added to the program until you create it."
  }</p>`;
}

function draftSection(
  project: OfflineProgrammingProject,
  draft: WeldDraftUIState,
): string {
  return `
    <div class="weld-draft">
      <div class="weld-draft-fields">
        <label class="field">
          <span>Name</span>
          <input data-weld-field="name" type="text" value="${escapeHTML(draft.name)}"
            placeholder="fillet weld" autocomplete="off" />
        </label>
        <label class="field">
          <span>Weld type</span>
          <select data-weld-field="weldType">
            ${WELD_TYPES.map((weldType) => `
              <option value="${weldType}" ${weldType === draft.weldType ? "selected" : ""}>
                ${weldType}
              </option>
            `).join("")}
          </select>
        </label>
        <label class="field">
          <span>Work · °</span>
          <input data-weld-field="workAngleDeg" type="number" min="-89" max="89" step="1"
            value="${numberText(draft.workAngleDeg)}" />
        </label>
        <label class="field">
          <span>Travel · °</span>
          <input data-weld-field="travelAngleDeg" type="number" min="-89" max="89" step="1"
            value="${numberText(draft.travelAngleDeg)}" />
        </label>
        <label class="field">
          <span>Spin · °</span>
          <input data-weld-field="spinAngleDeg" type="number" min="-180" max="180" step="1"
            value="${numberText(draft.spinAngleDeg)}" />
        </label>
      </div>
      <p class="weld-drawer-note">Selected edges (${draft.segments.length})</p>
      <ul class="weld-segment-list">
        ${draft.segments.map((segment) => segmentRow(project, draft, segment)).join("")}
      </ul>
      <div class="weld-draft-actions">
        <button class="button primary" type="button" data-weld-action="create-weld">
          Create weld
        </button>
        <button class="button secondary" type="button" data-weld-action="clear-draft">
          Discard
        </button>
      </div>
    </div>
  `;
}

function segmentRow(
  project: OfflineProgrammingProject,
  draft: WeldDraftUIState,
  segment: WeldDraftSegmentUIState,
): string {
  const edge = catalogEdgeById(project, segment.edgeId);
  const active = draft.activeEdgeId === segment.edgeId;
  const label = edge?.label ?? segment.edgeId;
  return `
    <li class="weld-segment ${active ? "is-active" : ""}">
      <button
        class="weld-segment-body"
        type="button"
        data-weld-action="activate-segment"
        data-edge-id="${escapeHTML(segment.edgeId)}"
        title="${escapeHTML(segment.edgeId)}"
      >
        <strong>${escapeHTML(label)}</strong>
        <small data-weld-trim-readout="${escapeHTML(segment.edgeId)}">${escapeHTML(weldSegmentTrimLabel(project, segment))}</small>
      </button>
      <div class="weld-segment-trims">
        <input data-weld-field="startS" data-edge-id="${escapeHTML(segment.edgeId)}"
          type="range" min="0" max="1" step="0.001" value="${numberText(segment.startS)}"
          aria-label="Start of ${escapeHTML(label)} (normalized)" />
        <input data-weld-field="endS" data-edge-id="${escapeHTML(segment.edgeId)}"
          type="range" min="0" max="1" step="0.001" value="${numberText(segment.endS)}"
          aria-label="End of ${escapeHTML(label)} (normalized)" />
      </div>
      <button class="mini-button ${segment.reversed ? "is-active" : ""}" type="button"
        data-weld-action="reverse-segment" data-edge-id="${escapeHTML(segment.edgeId)}"
        aria-pressed="${segment.reversed}"
      >${segment.reversed ? "REV" : "FWD"}</button>
      <button class="mini-button weld-segment-remove" type="button"
        data-weld-action="remove-segment" data-edge-id="${escapeHTML(segment.edgeId)}"
        aria-label="Remove ${escapeHTML(label)} from the draft"
      >×</button>
    </li>
  `;
}

function weldListSection(welds: readonly ProjectWeld[]): string {
  if (welds.length === 0) return "";
  return `
    <p class="weld-drawer-note">Welds in program (${welds.length})</p>
    <ul class="weld-list">
      ${welds.map((weld, index) => `
        <li class="weld-list-item">
          <span>
            <strong>Weld ${index + 1}${weld.label && !/^weld \d+$/i.test(weld.label) ? ` — ${escapeHTML(weld.label)}` : ""}</strong>
            <small>${escapeHTML(weld.weldType ?? "weld")} · ${weld.seamIds.length} edge${weld.seamIds.length === 1 ? "" : "s"}</small>
          </span>
          <button class="mini-button weld-segment-remove" type="button"
            data-weld-action="delete-weld" data-weld-id="${escapeHTML(weld.id)}"
            aria-label="Delete ${escapeHTML(weld.label)}"
          >DELETE</button>
        </li>
      `).join("")}
    </ul>
  `;
}

function numberText(value: number): string {
  return Number.isFinite(value) ? String(value) : "0";
}

function escapeHTML(value: string): string {
  return value
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}
