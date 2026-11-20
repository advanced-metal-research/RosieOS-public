/**
 * The browser's side of the cell program store: runs cell-sync.ts over
 * `fetch` and draws its state -- the header pill ("synced with <host> ·
 * rev 12", "local draft", "conflict") and the banner that asks the operator
 * the questions only they can answer (reload theirs or keep mine as a copy;
 * upload a program that is only in this browser).
 *
 * IndexedDB stays this browser's copy of every program (autosave.ts): the
 * offline cache and the draft while the cell cannot be reached. The cell's
 * revision is what every client shares.
 */
import type { OfflineProgrammingProject } from "../domain/project.ts";
import { assertProject } from "../domain/project.ts";
import { migrateProject } from "../domain/serialization.ts";
import {
  adoptRemoteProject, cellDocument, CellProgramSync, copyProject, LocalStorageLinks,
  type CellOpenCheck, type CellOpenChoice, type CellPicker, type CellProgramHead, type CellRequest, type CellSyncView,
} from "./cell-sync.ts";

export interface BrowserCellSyncHost {
  clientId: string;
  project(): OfflineProgrammingProject;
  /** Open a program the cell holds (already adopted for this client). */
  open(project: OfflineProgrammingProject, message: string): void;
  notice(message: string): void;
  /** New program (the picker's action); absent: not offered. */
  newProgram?(): void;
  /** Open this browser's own copy of a program (IndexedDB): a local-only
   *  program, or this browser's draft of a cell program. */
  openLocal?(programId: string): void;
  /** The view changed: redraw what reads it. */
  changed(view: CellSyncView): void;
}

const TICK_MS = 250;

export class BrowserCellSync {
  readonly sync: CellProgramSync;
  private readonly host: BrowserCellSyncHost;
  private readonly fetcher: typeof fetch;
  private timer: ReturnType<typeof setInterval> | null = null;
  private readonly controllers = new Set<AbortController>();
  private lastViewKey = "";
  private dismissedUpload = new Set<string>();
  private readonly clock: () => number;

  constructor(host: BrowserCellSyncHost, options: { fetch?: typeof fetch; storage?: Storage | null; clock?: () => number } = {}) {
    this.host = host;
    this.clock = options.clock ?? Date.now;
    this.fetcher = options.fetch ?? ((input, init) => fetch(input, init));
    this.sync = new CellProgramSync(new LocalStorageLinks(options.storage === undefined ? localStorageOrNull() : options.storage), {
      clientTag: host.clientId, keepsLocalCopies: true,
    });
  }

  start(): void {
    if (this.timer) return;
    this.tick();
    this.timer = setInterval(() => this.tick(), TICK_MS);
  }

  /** Stop ticking and abandon the requests in flight (a held change feed included). */
  stop(): void {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
    for (const controller of this.controllers) controller.abort();
    this.controllers.clear();
  }

  tick(now = this.clock()): void {
    this.sync.observe(cellDocument(this.host.project()), now);
    for (const request of this.sync.requests(now)) void this.run(request);
    const local = this.sync.takeLocalOpen();
    if (local) this.host.openLocal?.(local);
    for (const message of this.sync.takeNotices()) this.host.notice(message);
    this.render(now);
  }

  /** Open program: the cell's programs (cell-sync.ts picker) with this
   *  browser's run lock folded into each row's refusal. */
  picker(runLock = "", now = this.clock()): CellPicker {
    return this.sync.picker(now, runLock);
  }

  /** What opening `programId` would do to the open program (cell-sync.ts openCheck). */
  openCheck(programId: string, runLock = "", local = false): CellOpenCheck {
    return this.sync.openCheck(programId, this.clock(), runLock, local);
  }

  /** Open it with the operator's answer to the open program's draft; returns
   *  what to tell the operator, throws a refusal. */
  openAfter(programId: string, choice: CellOpenChoice, runLock = "", local = false): string {
    const now = this.clock();
    this.sync.observe(cellDocument(this.host.project()), now);
    const message = this.sync.openAfter(programId, choice, now, runLock, local);
    this.tick(now);
    return message;
  }

  view(now = this.clock()): CellSyncView {
    return this.sync.view(now);
  }

  /** Put the open program on the cell (a new program, or migration). */
  upload(makeCurrent = false): void {
    const now = this.clock();
    this.sync.observe(cellDocument(this.host.project()), now);
    this.sync.upload(makeCurrent, now);
    this.tick(now);
  }

  /** Open a cell program; the cell's current program becomes it, so the Deck
   *  follows. The banner's rows: refused, like Open program, when the open
   *  program holds an unsaved edit (Open program asks about it). */
  openCell(programId: string): void {
    try {
      this.host.notice(this.openAfter(programId, "open"));
    } catch (error) {
      this.host.notice(error instanceof Error ? error.message : String(error));
    }
  }

  makeCurrent(): void {
    const view = this.view();
    if (view.programId) this.sync.makeCurrent(view.programId);
    this.tick();
  }

  reloadTheirs(): void {
    this.sync.resolveTheirs();
    this.tick();
  }

  keepMineAsCopy(): void {
    const copy = copyProject(this.host.project());
    this.host.open(copy, `Your edits are now "${copy.identity.name}"; the original follows the cell`);
    this.sync.resolveCopy(cellDocument(copy), false, this.clock());
    this.tick();
  }

  get canCreate(): boolean {
    return Boolean(this.host.newProgram);
  }

  newProgram(): void {
    this.host.newProgram?.();
  }

  dismissUpload(): void {
    this.dismissedUpload.add(this.host.project().identity.id);
    this.lastViewKey = "";
    this.render(this.clock());
  }

  private async run(request: CellRequest): Promise<void> {
    const controller = new AbortController();
    this.controllers.add(controller);
    const timeout = setTimeout(() => controller.abort(), request.timeoutMs);
    let status = 0;
    let body: unknown = null;
    try {
      const response = await this.fetcher(request.path, {
        method: request.method,
        headers: request.body === undefined ? { Accept: "application/json" } : { Accept: "application/json", "Content-Type": "application/json" },
        body: request.body === undefined ? undefined : JSON.stringify(request.body),
        signal: controller.signal,
        cache: "no-store",
      });
      status = response.status;
      body = await response.json().catch(() => null);
    } catch {
      status = 0;
    } finally {
      clearTimeout(timeout);
      this.controllers.delete(controller);
    }
    if (controller.signal.aborted && !this.timer) return;
    const now = this.clock();
    this.sync.reply(request.id, status, body, now);
    this.takeRemote();
    for (const message of this.sync.takeNotices()) this.host.notice(message);
    this.tick(now);
  }

  private takeRemote(): void {
    const remote = this.sync.takeRemote();
    if (!remote) return;
    let opened: OfflineProgrammingProject;
    try {
      const migrated = migrateProject(remote.document);
      assertProject(migrated);
      opened = adoptRemoteProject(this.host.project(), migrated);
    } catch (error) {
      this.host.notice(`The cell's revision ${remote.head.revision} of ${remote.head.name} could not be opened: ${error instanceof Error ? error.message : String(error)}`);
      return;
    }
    this.host.open(opened, `Opened ${remote.head.name} · rev ${remote.head.revision} from ${this.view().host}${by(remote.head)}`);
    this.sync.applied(remote.head, cellDocument(opened));
  }

  private render(now: number): void {
    const view = this.sync.view(now);
    const key = JSON.stringify([view.state, view.label, view.editorsText, view.conflict, view.offerUpload, view.current, view.isCurrent,
      view.programs.map((p) => `${p.programId}:${p.revision}`), this.dismissedUpload.has(view.programId),
      view.offerUpload ? this.sync.picker(now) : null]);
    if (key === this.lastViewKey) return;
    this.lastViewKey = key;
    renderPill(view);
    renderBanner(view, this, this.dismissedUpload.has(view.programId), document, now);
    this.host.changed(view);
  }
}

function by(head: CellProgramHead): string {
  return head.updatedBy.label ? ` (saved by ${clientNameOf(head)})` : "";
}

function clientNameOf(head: CellProgramHead): string {
  const label = head.updatedBy.label;
  const pendant = /^rosie-pendant[^(]*\(([^)]+)\)/.exec(label);
  return pendant ? pendant[1] ?? label : label.replace(/ on https?:\/\/\S+$/, "");
}

function localStorageOrNull(): Storage | null {
  try {
    return typeof localStorage === "undefined" ? null : localStorage;
  } catch {
    return null;
  }
}

/** The header pill beside the program name. */
export function renderPill(view: CellSyncView, root: ParentNode = document): void {
  const pill = root.querySelector<HTMLElement>("#cell-sync");
  if (!pill) return;
  pill.textContent = view.editorsText ? `${view.label} · ${view.editorsText}` : view.label;
  pill.title = view.detail;
  pill.dataset.state = view.state;
  pill.hidden = view.state === "unknown";
}

/** The banner under the header: only for a question the operator must answer. */
export function renderBanner(view: CellSyncView, actions: BrowserCellSync, dismissedUpload: boolean, root: Document = document,
  now = Date.now()): void {
  let banner = root.querySelector<HTMLElement>("#cell-sync-banner");
  if (!banner) {
    banner = root.createElement("div");
    banner.id = "cell-sync-banner";
    banner.className = "cell-sync-banner";
    banner.setAttribute("role", "status");
    root.body.append(banner);
  }
  banner.replaceChildren();
  const button = (label: string, title: string, run: () => void, primary = false): HTMLButtonElement => {
    const b = root.createElement("button");
    b.type = "button";
    b.className = primary ? "mini-button primary" : "mini-button";
    b.textContent = label;
    b.title = title;
    b.addEventListener("click", run);
    return b;
  };
  const message = (text: string): HTMLSpanElement => {
    const span = root.createElement("span");
    span.className = "cell-sync-banner-text";
    span.textContent = text;
    return span;
  };
  if (view.conflict) {
    banner.dataset.kind = "conflict";
    banner.append(message(`Conflict · ${view.conflict.text}`));
    banner.append(button(view.conflict.kind === "deleted" ? "Keep it local" : `Reload theirs (rev ${view.conflict.revision})`,
      "Drop the edits made here and open the cell's revision", () => actions.reloadTheirs(), true));
    banner.append(button("Keep mine as a copy", "Save the edits made here to the cell as a new program", () => actions.keepMineAsCopy()));
    banner.hidden = false;
    return;
  }
  if (view.offerUpload && !dismissedUpload) {
    // The cell's saved programs (the Deck's first-start picker, cell-sync.ts
    // picker): open one -- it becomes the cell's current program -- or put
    // this one on the cell, or start a new one.
    const picker = actions.sync.picker(now);
    banner.dataset.kind = "picker";
    banner.append(message(`"${picker.local?.name ?? view.programId}" is only in this browser. Open one of ${view.host}'s programs, or upload this one so the Steam Deck and other browsers see it.`));
    const list = root.createElement("div");
    list.className = "cell-sync-picker";
    if (picker.rows.length === 0) list.append(message(`No saved programs on ${view.host} yet.`));
    const blocked = picker.rows.find((row) => row.blocked)?.blocked ?? "";
    for (const row of picker.rows.slice(0, 8)) {
      const marks = [row.current ? "CURRENT" : "", row.running ? "ON THE ROBOT" : ""].filter(Boolean).join(" · ");
      const b = button(`${row.current ? "★ " : ""}${row.name} · ${row.model} · rev ${row.revision} · ${row.saved}${marks ? ` · ${marks}` : ""}`,
        row.blocked || `Open it from ${view.host}; it becomes the cell's current program (the Steam Deck follows)`,
        () => actions.openCell(row.programId));
      b.dataset.programId = row.programId;
      b.disabled = Boolean(row.blocked);
      list.append(b);
    }
    if (blocked) list.append(message(blocked));
    banner.append(list);
    const row = root.createElement("div");
    row.className = "cell-sync-picker-actions";
    row.append(button(`Upload "${picker.local?.name ?? "this program"}" to ${view.host}`, "Save this program in the cell's program store", () => actions.upload(false), true));
    if (actions.canCreate) row.append(button("New program", "Start a new program on the cell", () => actions.newProgram()));
    row.append(button("Not now", "Keep this program in this browser only", () => actions.dismissUpload()));
    banner.append(row);
    banner.hidden = false;
    return;
  }
  banner.hidden = true;
}
