import { robotCatalogue } from "../domain/robots.ts";
/** The arms with chrome authored in this app: the public descriptions. */
const PUBLIC_ARM_MODELS = ["rosie_1400_v3", "rosie_1420_v1"] as const;

/** Every arm this app can draw: the public ones and every other model the
 * server's robot catalogue describes (its descriptions are the record of a
 * model, so no robot id is written here). With no machine connected the
 * program may be written for any of them, so the picker offers this list
 * rather than the last cell's (a bench left every arm disabled after None
 * (offline)). */
export function armModels(): readonly string[] {
  const models = new Set<string>(PUBLIC_ARM_MODELS);
  for (const entry of robotCatalogue()) models.add(entry.modelId);
  return [...models];
}
export type ArmModelID = string;
// Catalogue identities are external input; only classification grants capabilities.
export type RobotModelID = string;

/** Classify explicit catalogue identities; unknown IDs cannot acquire an arm
 * renderer or Cartesian capability from a label or a shared prefix. */
export function robotModelClass(model: string): "arm" | "bench" | "unknown" {
  if (armModels().includes(model)) return "arm";
  switch (model) {
    case "bench_one_motor_1to1": case "bench_nine_motors_1to1": return "bench";
    default: return "unknown";
  }
}

/** Reject every unrecognised identity, including disagreement between the
 * picker and status, so a known model cannot mask an unknown machine. */
export function unknownRobotModelReason(...models: (string | undefined)[]): string {
  const model = models.find(model => model !== undefined && robotModelClass(model) === "unknown");
  return model === undefined ? "" : `Unknown robot model ${model}: this app does not know how to drive it`;
}

/** A bench has no renderer model. Keeping this separate from the arm project
 * prevents catalogue selection from loading an unrelated URDF. */
export function cellModelPresentation(model: RobotModelID) {
  const modelClass = robotModelClass(model);
  const bench = modelClass === "bench";
  return {
    armModel: modelClass === "arm" ? model as ArmModelID : null,
    label: model === "bench_one_motor_1to1" ? "Mini bench (one motor, 1:1)"
      : model === "bench_nine_motors_1to1" ? "Bench (nine motors, 1:1)" : model,
    selectDisabled: modelClass !== "arm",
    viewportNote: unknownRobotModelReason(model) || (bench ? "Bench cell: no robot model. Live joint table below." : ""),
  };
}

/** Apply bench visibility without destroying the arm renderer or its options,
 * so returning to an arm restores the existing viewer and model selection. */
export function renderCellModel(select: HTMLSelectElement, note: HTMLElement, panes: readonly HTMLElement[], model: RobotModelID): void {
  const view = cellModelPresentation(model);
  select.querySelector('[data-bench-model]')?.remove();
  select.disabled = view.selectDisabled;
  if (!view.armModel) {
    const option = document.createElement("option");
    option.dataset.benchModel = ""; option.value = model; option.textContent = view.label;
    select.append(option);
  }
  select.value = model;
  note.textContent = view.viewportNote;
  note.hidden = !view.viewportNote;
  for (const pane of panes) pane.style.visibility = view.armModel ? "" : "hidden";
}
export interface CellTarget {
  cell_id: string;
  label: string;
  role: string;
  models: RobotModelID[];
  requested_mode: "simulation" | "real";
  backend: "simulation" | "ethercat" | "unreachable";
  simulation?: boolean;
  host: string;
  reason?: string;
}
export interface CellSelection { cell_id: string; model_id: RobotModelID }
export interface CellTargets { targets: CellTarget[]; selected: CellSelection | null }

/** Invalid catalogue data must disable the presentation instead of supplying
 * unchecked model or selection claims to machine controls. The message names
 * the rejected field so an operator can tell server skew from a bad reply. */
export class CellCatalogueError extends Error {
  constructor(reason: string, what = "Invalid machine catalogue") { super(`${what}: ${reason}`); }
}

/** A proxy or tunnel failure answers without a JSON body. It still discards
 * catalogue claims, but says unreachable with its HTTP status: "invalid"
 * sent the operator looking for version skew on a dead tunnel (2026-10-08). */
export class CellCatalogueUnreachableError extends CellCatalogueError {
  constructor(status: number) { super(`HTTP ${status} with no JSON body`, "OLP API unreachable"); }
}

function object(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}
function identity(value: unknown): value is string { return typeof value === "string" && value.trim().length > 0; }
function shown(value: unknown): string { return value === undefined ? "missing" : JSON.stringify(value); }
/** Returns why `value` is not a CellTarget, or "" when it is one. */
function targetProblem(value: unknown, at: string): string {
  if (!object(value)) return `${at} is not an object`;
  for (const key of ["cell_id", "label", "role", "host"] as const) {
    if (!identity(value[key])) return `${at}.${key} ${shown(value[key])} is not a non-empty string`;
  }
  if (!Array.isArray(value.models) || value.models.length === 0) return `${at}.models is not a non-empty list`;
  if (!value.models.every(identity)) return `${at}.models has an empty or non-string model`;
  if (new Set(value.models).size !== value.models.length) return `${at}.models repeats a model`;
  if (value.requested_mode !== "simulation" && value.requested_mode !== "real") {
    return `${at}.requested_mode ${shown(value.requested_mode)} is not simulation or real`;
  }
  if (value.backend !== "simulation" && value.backend !== "ethercat" && value.backend !== "unreachable") {
    return `${at}.backend ${shown(value.backend)} is not simulation, ethercat or unreachable`;
  }
  if (value.simulation !== undefined && typeof value.simulation !== "boolean") return `${at}.simulation ${shown(value.simulation)} is not a boolean`;
  if (value.reason !== undefined && typeof value.reason !== "string") return `${at}.reason is not a string`;
  return "";
}
/** Returns why `value` is not a selection of one of `targets`, or "". */
function selectionProblem(value: unknown, targets: CellTarget[], at: string): string {
  if (!object(value) || !identity(value.cell_id) || !identity(value.model_id)) return `${at} is not a cell_id/model_id pair`;
  return targets.some(target => target.cell_id === value.cell_id && target.models.includes(value.model_id as string))
    ? "" : `${at} ${value.cell_id}/${value.model_id} is not a catalogued cell and model`;
}

/** Show only public display identity. Credentials and configuration paths never
 * belong in the connected-target line. */
export function cellTargetText(cell: CellTarget, model: RobotModelID): string {
  return `${cell.label} · ${model} · ${cell.backend} · ${cell.host}`;
}

/** Catalogue reads and selection carry intent only; there is no Arm or lease
 * request in this client, and an HTTP refusal cannot become a local selection. */
export class CellTargetsClient {
  readonly #fetch: typeof fetch;
  constructor(fetcher: typeof fetch = globalThis.fetch.bind(globalThis)) { this.#fetch = fetcher; }

  async #request(path: string, selection?: CellSelection): Promise<unknown> {
    const response = await this.#fetch(`/api/offline-programming/v1/targets${path}`, {
      method: selection ? "POST" : "GET", credentials: "same-origin", cache: "no-store", redirect: "error",
      ...(selection ? { headers: { "Content-Type": "application/json" }, body: JSON.stringify(selection) } : {}),
    });
    let body: unknown;
    try { body = await response.json(); } catch {
      if (!response.ok) throw new CellCatalogueUnreachableError(response.status);
      throw new CellCatalogueError(`HTTP ${response.status} body is not JSON`);
    }
    if (!response.ok) throw new Error(object(body) && typeof body.error === "string" ? body.error : "cell_unreachable");
    return body;
  }
  async list(): Promise<CellTargets> {
    const body = await this.#request("");
    if (!object(body)) throw new CellCatalogueError("response is not an object");
    if (!Array.isArray(body.targets)) throw new CellCatalogueError("targets is not a list");
    const targets: unknown[] = body.targets;
    for (const [index, target] of targets.entries()) {
      const problem = targetProblem(target, `targets[${index}]`);
      if (problem) throw new CellCatalogueError(problem);
    }
    const cells = targets as CellTarget[];
    const seen = new Set<string>();
    for (const cell of cells) {
      if (seen.has(cell.cell_id)) throw new CellCatalogueError(`cell_id ${cell.cell_id} appears twice`);
      seen.add(cell.cell_id);
    }
    if (body.selected !== null) {
      const problem = selectionProblem(body.selected, cells, "selected");
      if (problem) throw new CellCatalogueError(problem);
    }
    return { targets: cells, selected: body.selected as CellSelection | null };
  }
  async select(selection: CellSelection): Promise<{ selected: CellSelection; target: CellTarget }> {
    const reason = unknownRobotModelReason(selection.model_id);
    if (reason) throw new Error(reason);
    const body = await this.#request("/select", selection);
    if (!object(body)) throw new CellCatalogueError("selection response is not an object");
    const problem = targetProblem(body.target, "target");
    if (problem) throw new CellCatalogueError(problem);
    const target = body.target as CellTarget;
    const selectedProblem = selectionProblem(body.selected, [target], "selected");
    if (selectedProblem) throw new CellCatalogueError(selectedProblem);
    const selected = body.selected as CellSelection;
    if (selected.cell_id !== selection.cell_id || selected.model_id !== selection.model_id) {
      throw new CellCatalogueError(`selected ${selected.cell_id}/${selected.model_id} differs from requested ${selection.cell_id}/${selection.model_id}`);
    }
    return { selected, target };
  }
}
