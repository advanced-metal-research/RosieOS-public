/**
 * One open program kept in step with the cell's program store
 * (offline-programming/v1 cell_programs.go): the programs every client of one
 * robot edits, kept on the robot's own OLP host.
 *
 * The browser and the Steam Deck pendant both run this class, so they cannot
 * disagree about when an edit is saved, when it conflicts or what an operator
 * is told. It does no I/O: the host asks it for requests (`requests`),
 * performs them with its own transport (fetch, Qt's OlpClient) and hands every
 * answer back (`reply`). A program fetched from the cell is handed to the
 * host (`takeRemote`) and the host says what it opened (`applied`).
 *
 * The rules it exists to keep:
 * - an edit is saved from the revision it was made on; the cell refuses it
 *   when another client saved since, and this never resolves that by itself:
 *   the operator reloads theirs or keeps theirs as a copy;
 * - a cell that cannot be reached leaves the edit as a local draft that says
 *   so, and the same rule decides it on reconnect;
 * - a program the robot has loaded is not saved over (423): the edit waits as
 *   a draft until Stop.
 *
 * Bundled for Qt 5.15's engine (ES2016): no padStart, no Object.entries, no
 * `#private` members (they become WeakMaps, which crash that engine).
 */
import type { OfflineProgrammingProject } from "../domain/project.ts";
import { projectId } from "../domain/project.ts";

export const CELL_PROGRAMS_API = "/api/offline-programming/v1/cell-programs";

export interface CellClientRef {
  id: string;
  label: string;
  kind: string;
  address: string;
}

export interface CellProgramHead {
  programId: string;
  revision: number;
  deleted: boolean;
  updatedAt: string;
  updatedBy: CellClientRef;
  name: string;
  robotModel: string;
  descriptionSha256: string;
  programRef: string;
  bytes: number;
}

export interface CellPresence {
  programId: string;
  id: string;
  label: string;
  kind: string;
  editing: boolean;
  you: boolean;
}

export interface CellStoreView {
  storeId: string;
  instance: string;
  label: string;
  version: number;
  current: string | null;
  currentSetBy: string;
  executing: { programId: string; programRef: string; reason: string } | null;
  programs: CellProgramHead[];
  presence: CellPresence[];
  /** The watched program's head, deleted ones included; undefined when the
   *  answer watched nothing. */
  watched?: CellProgramHead | null;
}

/** What this client knows of one program's place in one cell's store; kept
 *  across restarts so an offline draft is reconciled under the same rules. */
export interface CellLink {
  storeId: string;
  programId: string;
  /** The store revision the local program was last equal to (0: never saved). */
  baseRevision: number;
  /** Local edits the cell has not accepted. */
  dirty: boolean;
  /** projectSyncKey of what the cell holds at baseRevision. */
  syncedKey: string;
}

export interface CellLinkStorage {
  get(storeId: string, programId: string): CellLink | null;
  put(link: CellLink): void;
  remove(storeId: string, programId: string): void;
}

/** The open program as the host sees it. `document` is the OLP project. */
export interface CellDocument {
  programId: string;
  name: string;
  key: string;
  document: unknown;
}

export type CellRequestKind = "feed" | "presence" | "save" | "fetch" | "current";

export interface CellRequest {
  id: number;
  kind: CellRequestKind;
  method: "GET" | "POST";
  path: string;
  body?: unknown;
  timeoutMs: number;
}

export interface CellConflict {
  kind: "changed" | "deleted";
  programId: string;
  revision: number;
  by: string;
  at: string;
}

/** One of the cell's programs as a picker row (newest save first). */
export interface CellPickerRow {
  programId: string;
  name: string;
  model: string;
  revision: number;
  /** "saved 3 min ago by <deck>". */
  saved: string;
  /** The last save: its ISO time, "3 min ago" and who ("<deck>"). */
  updatedAt: string;
  when: string;
  by: string;
  /** The other clients that have it open: "<deck> editing, Browser (Chrome on macOS) viewing". */
  openBy: string;
  /** This client has it open now. */
  here: boolean;
  /** This client keeps edits of it the cell has not accepted (the browser's
   *  IndexedDB draft); opening it opens that draft, never the cell's
   *  revision over it. Only for a client that keeps local copies. */
  draft: boolean;
  current: boolean;
  /** The robot has this program loaded or playing. */
  running: boolean;
  /** Why opening it is refused now ("" when it may be opened). */
  blocked: string;
}

/** The cell's saved programs and what else the operator can do: what the
 *  Deck shows when it starts or connects with a program the cell does not
 *  have, both clients' Open program, and the browser's banner. */
export interface CellPicker {
  host: string;
  rows: CellPickerRow[];
  /** This client's own program, when the cell does not have it. */
  local: { programId: string; name: string; edited: string } | null;
  /** Why no cell program can be opened now (no store, unreachable, not
   *  answered yet); "" when the rows say for themselves. */
  unavailable: string;
}

/** What opening a program would do to the open one, asked before it is
 *  opened: an edit the cell has not accepted is never replaced silently. */
export interface CellOpenCheck {
  /** Refused now, and why ("" when it may be opened). */
  blocked: string;
  /** It is already the open program: nothing to open. */
  here: boolean;
  /** The browser opens its own draft of it (CellPickerRow.draft), not the cell's revision. */
  openLocalDraft: boolean;
  /** The open program's edits that opening would leave; null when nothing is lost. */
  draft: CellDraft | null;
}

export interface CellDraft {
  name: string;
  /** What the operator is told: what is unsaved and where it would go. */
  text: string;
  /** Why "save to the cell, then open" cannot run now ("" when it can). */
  saveRefused: string;
  /** Opened anyway, the edits stay on this client (the browser's IndexedDB
   *  copy and its draft link); false on the Deck, whose one program file
   *  the next program replaces. */
  keptHere: boolean;
}

/** The operator's answer to a draft: save it to the cell first, discard it,
 *  keep it as a draft on this client (CellDraft.keptHere), or plain open
 *  when there is none. */
export type CellOpenChoice = "open" | "save" | "discard" | "keep";

/**
 * Why this client must not switch programs now, in one wording for the
 * browser and the Deck: a session loaded, staged, playing or paused on the
 * robot, a loop between its cycles, or a press of this client's still
 * running. Switching then would leave the robot holding a plan of a program
 * no client shows (the cell refuses its current-program switch for the same
 * reason, cell_programs.go noteLoaded); STOP ends all of them.
 */
export function programSwitchLock(input: { sessionState?: string; looping?: boolean; busy?: string }): string {
  if (input.looping) return "A loop is running on the robot; Finish cycle or STOP it before opening another program";
  const state = input.sessionState ?? "";
  if (state === "loaded" || state === "staged" || state === "playing" || state === "paused") {
    return `The robot has a program ${state}; STOP it before opening another program`;
  }
  if (input.busy) return `Not while ${input.busy}; STOP it before opening another program`;
  return "";
}

export type CellSyncState =
  "unknown" | "disabled" | "local" | "synced" | "saving" | "offline" | "conflict" | "locked" | "updating" | "error";

export interface CellSyncView {
  state: CellSyncState;
  /** One line: "synced with <host> · rev 12". */
  label: string;
  /** The Deck header's: "<HOST> · R12". */
  short: string;
  /** What it means and what to do, for a tooltip or the status line. */
  detail: string;
  host: string;
  programId: string;
  revision: number;
  dirty: boolean;
  editors: { name: string; editing: boolean }[];
  editorsText: string;
  conflict: (CellConflict & { text: string }) | null;
  /** The open program is not on the cell: offer Upload. */
  offerUpload: boolean;
  current: { programId: string; name: string } | null;
  isCurrent: boolean;
  locked: string;
  programs: CellProgramHead[];
}

export interface CellSyncOptions {
  /** Unique per client, for save ids (the tab's or pendant's client id). */
  clientTag: string;
  /** The Deck shows the cell's current program: it opens it whenever its own
   *  program has nothing unsaved, and makes its program current when the
   *  cell has none. The browser opens what the operator opens. */
  followCurrent?: boolean;
  /** Quiet time after an edit before it is sent (autosave settles first). */
  debounceMs?: number;
  /** How long the cell holds one change-feed request open. */
  feedTimeoutMs?: number;
  /** "Editing" is shown for this long after the last edit. */
  editingMs?: number;
  /** Every program this client opened stays on it (the browser's IndexedDB):
   *  switching away from an unsaved edit leaves it as a draft here. The Deck
   *  keeps one program file, which the next program replaces. */
  keepsLocalCopies?: boolean;
}

const OFFLINE_RETRY_MS = [1000, 2000, 5000, 10000];
// The least time between two change-feed requests: a server that answers at
// once (another instance, a presence change) must not be asked in a loop.
const FEED_FLOOR_MS = 250;
const DISABLED_RETRY_MS = 60000;

function record(value: unknown): Record<string, unknown> {
  return value && typeof value === "object" && !Array.isArray(value) ? value as Record<string, unknown> : {};
}

function text(value: unknown): string {
  return typeof value === "string" ? value : "";
}

function count(value: unknown): number {
  return typeof value === "number" && isFinite(value) ? value : 0;
}

function parseClient(value: unknown): CellClientRef {
  const r = record(value);
  return { id: text(r.id), label: text(r.label), kind: text(r.kind), address: text(r.address) };
}

export function parseHead(value: unknown): CellProgramHead | null {
  const r = record(value);
  if (!text(r.program_id)) return null;
  return {
    programId: text(r.program_id), revision: count(r.revision), deleted: r.deleted === true,
    updatedAt: text(r.updated_at), updatedBy: parseClient(r.updated_by), name: text(r.name),
    robotModel: text(r.robot_model), descriptionSha256: text(r.description_sha256), programRef: text(r.program_ref),
    bytes: count(r.bytes),
  };
}

export function parseStoreView(value: unknown): CellStoreView | null {
  const r = record(value);
  if (r.schema !== "offline-programming.cell-programs.v1" || r.enabled !== true) return null;
  const store = record(r.store);
  const current = record(r.current);
  const executing = r.executing ? record(r.executing) : null;
  const view: CellStoreView = {
    storeId: text(store.id), instance: text(store.instance), label: text(store.label) || text(store.host) || "the cell",
    version: count(r.version),
    current: text(current.program_id) || null,
    currentSetBy: clientName(parseClient(current.set_by).label),
    executing: executing ? { programId: text(executing.program_id), programRef: text(executing.program_ref), reason: text(executing.reason) } : null,
    programs: (Array.isArray(r.programs) ? r.programs : []).map(parseHead).filter((h): h is CellProgramHead => h !== null),
    presence: (Array.isArray(r.presence) ? r.presence : []).map((p) => {
      const q = record(p);
      return { programId: text(q.program_id), id: text(q.id), label: text(q.label), kind: text(q.kind), editing: q.editing === true, you: q.you === true };
    }),
  };
  if ("watched" in r) view.watched = r.watched ? parseHead(r.watched) : null;
  return view;
}

/** A client as an operator names it: the Deck's host ("<deck>"), the
 *  browser without its origin, else the server's label. */
export function clientName(label: string): string {
  const pendant = /^rosie-pendant[^(]*\(([^)]+)\)/.exec(label);
  if (pendant) return pendant[1] ?? label;
  const browser = /^(Browser \([^)]*\))/.exec(label);
  if (browser) return browser[1] ?? label;
  return label || "another client";
}

/** What decides whether a program changed for the cell: its revision, its
 *  plan and its robot description pin. Selecting a row moves none of them. */
export function projectSyncKey(project: OfflineProgrammingProject): string {
  return [
    project.identity.id, project.identity.revision, project.plan?.state ?? "", project.plan?.id ?? "",
    project.denseTrajectory?.trajectoryDigest ?? "", project.robot.description?.robotDescriptionSha256 ?? "",
  ].join("|");
}

export function cellDocument(project: OfflineProgrammingProject): CellDocument {
  return { programId: project.identity.id, name: project.identity.name, key: projectSyncKey(project), document: project };
}

/**
 * A program from the cell, as this client opens it. A client keeps its own
 * revision counter, and the browser's project store refuses a revision number
 * it already holds with other content; a cell program whose identity.revision
 * is not ahead of the one open here is moved one past it.
 */
export function adoptRemoteProject(local: OfflineProgrammingProject | null, remote: OfflineProgrammingProject): OfflineProgrammingProject {
  if (local && local.identity.id === remote.identity.id && remote.identity.revision <= local.identity.revision) {
    return { ...remote, identity: { ...remote.identity, revision: local.identity.revision + 1 } };
  }
  return remote;
}

/** "Keep mine as a copy": the local program under a new identity. */
export function copyProject(project: OfflineProgrammingProject, now = new Date().toISOString()): OfflineProgrammingProject {
  const id = projectId();
  return {
    ...project,
    identity: { ...project.identity, id, name: `${project.identity.name} (copy)`, createdAt: now, updatedAt: now },
    program: { ...project.program, programId: id, baseProgramRevision: 0, catalogRevision: null },
  };
}

/** "3 min ago" for an ISO time; "" when it is not one. */
export function ago(at: string, now: number): string {
  const t = Date.parse(at);
  if (!isFinite(t)) return "";
  const s = Math.max(0, Math.round((now - t) / 1000));
  if (s < 60) return `${s} s ago`;
  if (s < 3600) return `${Math.round(s / 60)} min ago`;
  return `${Math.round(s / 3600)} h ago`;
}

export class CellProgramSync {
  readonly options: Required<CellSyncOptions>;
  readonly storage: CellLinkStorage;
  store: CellStoreView | null = null;
  private minViewVersion = 0;
  /** null until the server has answered once. */
  enabled: boolean | null = null;
  online: boolean | null = null;
  link: CellLink | null = null;
  local: CellDocument | null = null;
  /** The local program the cell has not accepted yet. */
  pending: CellDocument | null = null;
  conflict: CellConflict | null = null;
  lockReason = "";
  /** A save the cell refused for good (too large, invalid): not retried until the next edit. */
  error = "";
  notices: string[] = [];
  /** Set by `open` until the first answer tells which store this is. */
  private linkResolved = false;
  private remote: { head: CellProgramHead; document: unknown; force: boolean } | null = null;
  private wantFetch: { programId: string; revision: number; force: boolean } | null = null;
  private wantCurrent: string | null = null;
  private makeCurrentOnSave = false;
  private inflight: { [kind: string]: number } = {};
  private sending: { doc: CellDocument; saveId: string; base: number } | null = null;
  private lastSaveId: { key: string; base: number; saveId: string } | null = null;
  private nextId = 1;
  private saveSeq = 0;
  private failures = 0;
  private retryAt = 0;
  private feedAt = 0;
  private lastEditAt = -Infinity;
  private sentEditing = false;
  private blockedKey = "";
  private offered = "";
  /** upload() asked before the first answer named the store. */
  private uploadWhenKnown: { programId: string; makeCurrent: boolean } | null = null;
  /** "Save, then open": opened once the cell accepted the open program's edits. */
  private afterSave: { programId: string; name: string; local: boolean } | null = null;
  /** A program of this client's own (not the cell's) to open, handed to the host by takeLocalOpen. */
  private localOpen: string | null = null;

  constructor(storage: CellLinkStorage, options: CellSyncOptions) {
    this.storage = storage;
    this.options = {
      clientTag: options.clientTag, followCurrent: options.followCurrent ?? false,
      debounceMs: options.debounceMs ?? 800, feedTimeoutMs: options.feedTimeoutMs ?? 20000, editingMs: options.editingMs ?? 15000,
      keepsLocalCopies: options.keepsLocalCopies ?? false,
    };
  }

  /** The open program, every tick or after every change: a new identity is a
   *  program switch, a new key on the same one is an edit. */
  observe(doc: CellDocument, now: number): void {
    if (!this.local || this.local.programId !== doc.programId) {
      this.open(doc);
      return;
    }
    if (doc.key === this.local.key) return;
    this.local = doc;
    this.lastEditAt = now;
    if (this.link && doc.key !== this.link.syncedKey) {
      this.pending = doc;
      if (!this.link.dirty) {
        this.link.dirty = true;
        this.persist();
      }
    } else if (this.link && doc.key === this.link.syncedKey) {
      // Undone back to what the cell holds.
      this.pending = null;
      if (this.link.dirty) {
        this.link.dirty = false;
        this.persist();
      }
    }
  }

  private open(doc: CellDocument): void {
    this.local = doc;
    this.pending = null;
    this.conflict = null;
    this.error = "";
    this.lockReason = "";
    this.wantFetch = null;
    this.makeCurrentOnSave = false;
    this.afterSave = null;
    this.link = null;
    this.linkResolved = false;
    this.resolveLink();
  }

  private resolveLink(): void {
    if (this.linkResolved || !this.store || !this.local) return;
    this.linkResolved = true;
    this.link = this.storage.get(this.store.storeId, this.local.programId);
    if (this.link && this.link.dirty) this.pending = this.local;
    else if (this.link && this.local.key !== this.link.syncedKey && this.link.baseRevision > 0) {
      // Edited while this client was not running the sync (an older build, a
      // crash before the link was written): an edit like any other.
      this.link.dirty = true;
      this.pending = this.local;
      this.persist();
    }
  }

  private persist(): void {
    if (this.link) this.storage.put({ ...this.link });
  }

  /** Put the open program on the cell (migration, a new program): saved from
   *  revision 0, so a program of that identity already there is a conflict. */
  upload(makeCurrent: boolean, now: number): void {
    if (!this.local) return;
    if (!this.store) {
      this.uploadWhenKnown = { programId: this.local.programId, makeCurrent };
      return;
    }
    this.uploadWhenKnown = null;
    this.link = { storeId: this.store.storeId, programId: this.local.programId, baseRevision: 0, dirty: true, syncedKey: "" };
    this.linkResolved = true;
    this.pending = this.local;
    this.conflict = null;
    this.error = "";
    this.makeCurrentOnSave = makeCurrent;
    this.lastEditAt = now - this.options.debounceMs;
    this.persist();
  }

  /** Open a program from the cell; with makeCurrent the cell's current
   *  program becomes it too (the Deck follows). */
  openCell(programId: string, makeCurrent: boolean): void {
    if (makeCurrent) this.wantCurrent = programId;
    this.wantFetch = { programId, revision: 0, force: true };
  }

  makeCurrent(programId: string): void {
    this.wantCurrent = programId;
  }

  /** Conflict: drop the local edit and open the cell's revision. */
  resolveTheirs(): void {
    const conflict = this.conflict;
    if (!conflict || !this.link) return;
    this.pending = null;
    this.link.dirty = false;
    this.conflict = null;
    if (conflict.kind === "deleted") {
      if (this.store) this.storage.remove(this.store.storeId, conflict.programId);
      this.link = null;
      this.notice(`The program is no longer on ${this.host()}; it stays open here as a local program`);
      return;
    }
    this.persist();
    this.wantFetch = { programId: conflict.programId, revision: conflict.revision, force: true };
  }

  /** Conflict: the host opened `copy` (copyProject of the local program);
   *  it goes to the cell as a new program, and the original is left to the
   *  cell's revision. */
  resolveCopy(copy: CellDocument, makeCurrent: boolean, now: number): void {
    const conflict = this.conflict;
    if (conflict && this.store) this.storage.remove(this.store.storeId, conflict.programId);
    this.local = copy;
    this.upload(makeCurrent, now);
  }

  /** A program fetched from the cell, for the host to open. An edit made here
   *  while it was on its way turns it into a conflict instead. */
  takeRemote(): { head: CellProgramHead; document: unknown } | null {
    const remote = this.remote;
    this.remote = null;
    if (!remote) return null;
    if (!remote.force && this.link && this.link.dirty && this.link.programId === remote.head.programId) {
      this.conflict = this.conflictOf("changed", remote.head);
      return null;
    }
    return { head: remote.head, document: remote.document };
  }

  /** The host opened the cell's revision `head` as `doc`. */
  applied(head: CellProgramHead, doc: CellDocument): void {
    if (!this.store) return;
    this.local = doc;
    this.pending = null;
    this.conflict = null;
    this.error = "";
    this.link = { storeId: this.store.storeId, programId: head.programId, baseRevision: head.revision, dirty: false, syncedKey: doc.key };
    this.linkResolved = true;
    this.persist();
  }

  notice(message: string): void {
    this.notices.push(message);
  }

  /** One-shot messages for the host's status line. */
  takeNotices(): string[] {
    const out = this.notices;
    this.notices = [];
    return out;
  }

  /** The cell's saved programs, newest save first, with the current and
   *  in-use markers and why one cannot be opened now (the robot runs another:
   *  the cell refuses to switch its current program until Stop). */
  picker(now: number, runLock = ""): CellPicker {
    const store = this.store;
    const host = this.host();
    const executing = store?.executing ?? null;
    const runningName = executing
      ? store!.programs.find((p) => p.programId === executing.programId)?.name ?? executing.programRef
      : "";
    const unavailable = this.enabled === false
      ? "This OLP server keeps no cell program store: programs are kept on this client only"
      : this.online === false ? `${host} cannot be reached; its programs can be opened again when it answers`
      : this.enabled === null || !store ? `Asking ${host} for its saved programs…` : "";
    const hereId = this.link?.programId ?? this.local?.programId ?? "";
    const rows = (store?.programs ?? []).filter((p) => !p.deleted).slice()
      .sort((a, b) => (a.updatedAt < b.updatedAt ? 1 : a.updatedAt > b.updatedAt ? -1 : 0))
      .map((p): CellPickerRow => {
        const running = Boolean(executing && executing.programId === p.programId);
        const when = ago(p.updatedAt, now);
        const by = p.updatedBy.label ? clientName(p.updatedBy.label) : "";
        const here = p.programId === hereId;
        const openBy = store!.presence.filter((q) => !q.you && q.programId === p.programId)
          .map((q) => `${clientName(q.label)} ${q.editing ? "editing" : "viewing"}`).join(", ");
        const kept = this.options.keepsLocalCopies && this.store ? this.storage.get(this.store.storeId, p.programId) : null;
        return {
          programId: p.programId, name: p.name, model: p.robotModel, revision: p.revision,
          saved: `saved ${when || "at an unknown time"}${by ? ` by ${by}` : ""}`,
          updatedAt: p.updatedAt, when, by, openBy, here,
          draft: Boolean(this.options.keepsLocalCopies && (here ? this.link?.dirty : kept?.dirty)),
          current: Boolean(store && store.current === p.programId), running,
          blocked: unavailable || runLock ||
            (executing && !running ? `The robot is running ${runningName}; Stop it before opening another program` : ""),
        };
      });
    const localOnCell = Boolean(this.local && (this.link || rows.some((r) => r.programId === this.local!.programId)));
    const identity = record(record(this.local?.document).identity);
    const edited = ago(text(identity.updatedAt), now);
    const local = this.local && !localOnCell
      ? { programId: this.local.programId, name: this.local.name, edited: edited ? `edited ${edited}` : "" }
      : null;
    return { host, rows, local, unavailable };
  }

  /** The open program's edits the cell has not accepted, as a draft the
   *  operator must answer before another program replaces it; null when
   *  opening another loses nothing. A program only on this client counts
   *  where nothing else keeps it (the Deck). */
  private openDraft(): CellDraft | null {
    const local = this.local;
    if (!local) return null;
    const host = this.host();
    const keptHere = this.options.keepsLocalCopies;
    const unsaved = Boolean(this.link && (this.link.dirty || this.pending || this.sending)) || Boolean(this.conflict);
    if (!this.link && !keptHere && this.store && this.enabled) {
      return {
        name: local.name, keptHere,
        text: `"${local.name}" is only on this client, not on ${host}. Opening another program replaces it.`,
        saveRefused: this.online === false ? `${host} cannot be reached` : "",
      };
    }
    if (!unsaved) return null;
    const rev = this.link?.baseRevision ?? 0;
    const saveRefused = this.conflict ? `${host} has a newer revision (conflict): reload theirs or keep yours as a copy first`
      : this.lockReason ? `${host} refuses edits to it: ${this.lockReason}`
      : this.error ? this.error
      : this.online === false ? `${host} cannot be reached`
      : this.enabled !== true ? `${host} keeps no program store` : "";
    return {
      name: local.name, keptHere, saveRefused,
      text: `"${local.name}" has edits ${host} has not saved (rev ${rev} + local edits).` +
        (keptHere ? " Kept as a draft they stay in this browser and are saved when it is opened again." : " Opening another program replaces them."),
    };
  }

  /** Before opening `programId` (a cell row, or with `local` one of this
   *  client's own programs): refused and why, already open, or the draft
   *  the operator must answer first. */
  openCheck(programId: string, now: number, runLock = "", local = false): CellOpenCheck {
    const hereId = this.link?.programId ?? this.local?.programId ?? "";
    if (programId === hereId) return { blocked: "", here: true, openLocalDraft: false, draft: null };
    let openLocalDraft = local;
    if (local) {
      if (runLock) return { blocked: runLock, here: false, openLocalDraft, draft: null };
    } else {
      const row = this.picker(now, runLock).rows.find((r) => r.programId === programId);
      if (!row) return { blocked: `That program is no longer on ${this.host()}`, here: false, openLocalDraft, draft: null };
      if (row.blocked) return { blocked: row.blocked, here: false, openLocalDraft, draft: null };
      openLocalDraft = row.draft;
    }
    return { blocked: "", here: false, openLocalDraft, draft: this.openDraft() };
  }

  /**
   * Open `programId` with the operator's answer to the open program's draft
   * (openCheck). "save" sends the edits first and opens once the cell
   * accepted them; a refusal on the way opens nothing and keeps the edits.
   * A cell program becomes the cell's current one (the Deck follows); a
   * local one, or the browser's own draft of a cell program, is handed to
   * the host by takeLocalOpen. Returns what to tell the operator; throws
   * when the answer does not fit (a draft answered "open", a refused save).
   */
  openAfter(programId: string, choice: CellOpenChoice, now: number, runLock = "", local = false): string {
    const check = this.openCheck(programId, now, runLock, local);
    if (check.blocked) throw new Error(check.blocked);
    const name = local ? "the program" : this.store?.programs.find((p) => p.programId === programId)?.name ?? programId;
    if (check.here) return `${name} is already open`;
    const draft = check.draft;
    const fromHere = local || check.openLocalDraft;
    if (draft && choice === "open") throw new Error(`${draft.text} Save, discard or keep it first`);
    if (draft && choice === "keep" && !draft.keptHere) throw new Error(`${draft.name} cannot be kept on this client`);
    if (draft && choice === "save") {
      if (draft.saveRefused) throw new Error(`Cannot save ${draft.name}: ${draft.saveRefused}`);
      this.afterSave = { programId, name, local: fromHere };
      if (!this.link) this.upload(false, now);
      else this.lastEditAt = Math.min(this.lastEditAt, now - this.options.debounceMs);
      return `Saving ${draft.name} to ${this.host()}, then opening ${name}`;
    }
    if (draft && choice === "discard") this.discardDraft();
    this.afterSave = null;
    if (fromHere) this.localOpen = programId;
    else this.openCell(programId, true);
    return `Opening ${name}${fromHere ? "" : ` from ${this.host()}`}${draft && choice === "discard" ? `; ${draft.name}'s edits discarded` : ""}`;
  }

  /** Drop the open program's edits the cell has not accepted. */
  private discardDraft(): void {
    this.pending = null;
    this.conflict = null;
    this.error = "";
    this.sending = null;
    if (this.link && this.link.dirty) {
      this.link.dirty = false;
      this.persist();
    }
  }

  /** A program of this client's own to open now (openAfter): the host opens its local copy. */
  takeLocalOpen(): string | null {
    const id = this.localOpen;
    this.localOpen = null;
    return id;
  }

  private abandonAfterSave(why: string): void {
    const after = this.afterSave;
    if (!after) return;
    this.afterSave = null;
    this.notice(`${after.name} was not opened: ${why}. The edits are kept here.`);
  }

  private saved(): void {
    const after = this.afterSave;
    if (!after || this.pending || this.sending || !this.link || this.link.dirty) return;
    this.afterSave = null;
    if (after.local) this.localOpen = after.programId;
    else this.openCell(after.programId, true);
  }

  /** The Deck's question when it starts or connects with a program the cell
   *  does not have: the cell's saved programs to open, or upload this one, or
   *  start a new one. Asked once per cell and program. */
  takeOffer(now: number): CellPicker | null {
    if (!this.options.followCurrent || !this.store || !this.local || this.link || this.uploadWhenKnown) return null;
    const key = `${this.store.storeId}>${this.local.programId}`;
    if (this.offered === key) return null;
    this.offered = key;
    return this.picker(now);
  }

  private host(): string {
    return this.store?.label ?? "the cell";
  }

  private editing(now: number): boolean {
    return Boolean(this.link && this.link.dirty) || now - this.lastEditAt < this.options.editingMs;
  }

  private watchId(): string {
    return this.link?.programId ?? this.local?.programId ?? "";
  }

  private request(kind: CellRequestKind, method: "GET" | "POST", path: string, timeoutMs: number, body?: unknown): CellRequest {
    const id = this.nextId++;
    this.inflight[kind] = id;
    return body === undefined ? { id, kind, method, path, timeoutMs } : { id, kind, method, path, body, timeoutMs };
  }

  private feedPath(waitMs: number, editing: boolean): string {
    const parts = [`since=${this.store ? this.store.version : 0}`, `instance=${encodeURIComponent(this.store ? this.store.instance : "")}`,
      `watch=${encodeURIComponent(this.watchId())}`, `editing=${editing ? 1 : 0}`, `timeout_ms=${waitMs}`];
    return `${CELL_PROGRAMS_API}/changes?${parts.join("&")}`;
  }

  /** The requests to start now. Each is answered through `reply`. */
  requests(now: number): CellRequest[] {
    const out: CellRequest[] = [];
    if (this.afterSave && this.conflict) this.abandonAfterSave(`${this.host()} has a newer revision (conflict)`);
    if (this.afterSave && this.lockReason) this.abandonAfterSave(`${this.host()} refuses edits to it: ${this.lockReason}`);
    this.saved();
    const editing = this.editing(now);
    if (!this.inflight.feed && now >= this.retryAt && now >= this.feedAt) {
      const wait = this.enabled === false || this.online !== true ? 0 : this.options.feedTimeoutMs;
      out.push(this.request("feed", "GET", this.feedPath(wait, editing), wait + 10000));
    }
    if (this.enabled !== true || this.online !== true) return out;
    if (editing !== this.sentEditing && !this.inflight.presence) {
      this.sentEditing = editing;
      out.push(this.request("presence", "GET", this.feedPath(0, editing), 10000));
    }
    if (this.wantCurrent && !this.inflight.current) {
      out.push(this.request("current", "POST", `${CELL_PROGRAMS_API}/current`, 10000, { program_id: this.wantCurrent }));
    }
    if (this.wantFetch && !this.inflight.fetch && !this.inflight.current) {
      out.push(this.request("fetch", "GET", `${CELL_PROGRAMS_API}/programs/${encodeURIComponent(this.wantFetch.programId)}`, 30000));
    }
    const pending = this.pending;
    if (this.link && pending && !this.inflight.save && !this.conflict && !this.lockReason && !this.wantFetch &&
        pending.key !== this.blockedKey && now - this.lastEditAt >= this.options.debounceMs && now >= this.retryAt) {
      const base = this.link.baseRevision;
      // A retry of the same document from the same revision reuses its id:
      // a save whose answer was lost is then a replay, not a conflict with itself.
      const reuse = this.lastSaveId && this.lastSaveId.key === pending.key && this.lastSaveId.base === base;
      const saveId = reuse ? this.lastSaveId!.saveId : `${this.options.clientTag}-${base}-${++this.saveSeq}`;
      this.lastSaveId = { key: pending.key, base, saveId };
      this.sending = { doc: pending, saveId, base };
      out.push(this.request("save", "POST", `${CELL_PROGRAMS_API}/programs/${encodeURIComponent(this.link.programId)}/save`, 60000,
        { base_revision: base, save_id: saveId, make_current: this.makeCurrentOnSave, document: pending.document }));
    }
    return out;
  }

  private wentOffline(now: number): void {
    this.abandonAfterSave(`${this.host()} stopped answering`);
    // Every request in flight fails together in one outage: back off once.
    if (this.online === false && now < this.retryAt) return;
    this.online = false;
    this.retryAt = now + (OFFLINE_RETRY_MS[Math.min(this.failures, OFFLINE_RETRY_MS.length - 1)] ?? 10000);
    this.failures++;
  }

  private disabled(now: number): void {
    this.enabled = false;
    this.online = true;
    this.retryAt = now + DISABLED_RETRY_MS;
  }

  private conflictOf(kind: CellConflict["kind"], head: CellProgramHead | null): CellConflict {
    return {
      kind, programId: head?.programId ?? this.link?.programId ?? "", revision: head?.revision ?? 0,
      by: head ? clientName(head.updatedBy.label) : "", at: head?.updatedAt ?? "",
    };
  }

  /** The answer to a request from `requests`: HTTP status (0 when the cell
   *  could not be reached) and the JSON body. */
  reply(id: number, status: number, body: unknown, now: number): void {
    let kind = "";
    for (const k in this.inflight) if (this.inflight[k] === id) kind = k;
    if (!kind) return;
    delete this.inflight[kind];
    const r = record(body);
    const storeDisabled = status === 503 && r.error === "program_store_disabled";
    if (status === 0 || (status >= 502 && status <= 504 && !storeDisabled)) {
      if (kind === "save") this.sending = null;
      if (kind === "presence") this.sentEditing = !this.sentEditing;
      this.wentOffline(now);
      return;
    }
    if (storeDisabled || status === 404 && kind === "feed") {
      this.sending = null;
      this.disabled(now);
      return;
    }
    if (kind === "feed") this.feedAt = now + FEED_FLOOR_MS;
    switch (kind) {
      case "feed":
      case "presence":
        this.onView(body, now);
        break;
      case "save":
        this.onSave(status, r);
        break;
      case "fetch":
        this.onFetch(status, r);
        break;
      case "current":
        this.onCurrent(status, r);
        break;
    }
  }

  private onView(body: unknown, now: number): void {
    const view = parseStoreView(body);
    if (!view) {
      this.disabled(now);
      return;
    }
    if (this.store && view.storeId === this.store.storeId && view.instance === this.store.instance &&
        view.version < Math.max(this.store.version, this.minViewVersion)) return;
    if (this.store && (view.storeId !== this.store.storeId || view.instance !== this.store.instance)) {
      this.minViewVersion = 0;
    }
    this.store = view;
    this.enabled = true;
    this.online = true;
    this.failures = 0;
    this.retryAt = 0;
    this.resolveLink();
    const deferred = this.uploadWhenKnown;
    if (deferred && this.local && deferred.programId === this.local.programId && !this.link &&
        !view.programs.some((p) => p.programId === deferred.programId)) {
      this.upload(deferred.makeCurrent, now);
    }
    this.uploadWhenKnown = null;
    if (!this.link && this.local) {
      // The cell has this program but this client never synced it (another
      // browser origin, cleared storage): which copy wins is the operator's.
      const existing = view.programs.find((p) => p.programId === this.local!.programId);
      if (existing) {
        this.link = { storeId: view.storeId, programId: existing.programId, baseRevision: 0, dirty: true, syncedKey: "" };
        this.pending = this.local;
        this.persist();
        this.conflict = this.conflictOf("changed", existing);
      }
    }
    const link = this.link;
    const executing = view.executing;
    if (link) {
      this.lockReason = executing && executing.programId === link.programId ? executing.reason : "";
      // `watched` answers the program the request watched, which is not this
      // one when the open program changed while the request was out.
      const listed = view.programs.find((p) => p.programId === link.programId) ?? null;
      const head = view.watched && view.watched.programId === link.programId ? view.watched : listed;
      if (!this.sending && !this.conflict && !this.wantFetch) {
        if ((!head || head.deleted) && link.baseRevision > 0) {
          this.conflict = this.conflictOf("deleted", head);
        } else if (head && !head.deleted && head.revision > link.baseRevision) {
          if (link.dirty) this.conflict = this.conflictOf("changed", head);
          else this.wantFetch = { programId: head.programId, revision: head.revision, force: false };
        }
      }
    } else {
      this.lockReason = "";
    }
    if (this.options.followCurrent && this.local && !this.conflict && !this.wantFetch && !this.sending) {
      if (view.current && link && link.programId !== view.current && !link.dirty && !this.pending) {
        const head = view.programs.find((p) => p.programId === view.current);
        this.wantFetch = { programId: view.current, revision: head?.revision ?? 0, force: true };
        this.notice(`Opening ${head?.name ?? view.current}: ${view.label}'s current program (set by ${view.currentSetBy})`);
      } else if (!view.current && !executing) {
        // An unlinked program is the operator's to place (takeOffer).
        if (link && link.baseRevision > 0 && !this.wantCurrent) this.wantCurrent = link.programId;
      }
    }
  }

  private onSave(status: number, r: Record<string, unknown>): void {
    const sending = this.sending;
    this.sending = null;
    if (!sending || !this.link) return;
    if (status === 200) {
      const head = parseHead(r.program);
      if (!head) return;
      this.link.baseRevision = head.revision;
      this.link.syncedKey = sending.doc.key;
      this.makeCurrentOnSave = false;
      if (this.pending && this.pending.key === sending.doc.key) {
        this.pending = null;
        this.link.dirty = false;
      }
      this.persist();
      if (this.store) {
        this.minViewVersion = Math.max(this.minViewVersion, count(r.version));
        this.store.programs = [head].concat(this.store.programs.filter((p) => p.programId !== head.programId));
        if (this.store.watched !== undefined && head.programId === this.watchId()) this.store.watched = head;
      }
      if (text(r.current_refused)) this.notice(`Saved, but ${this.host()}'s current program did not change: ${text(r.current_refused)}`);
      this.saved();
      return;
    }
    if (status === 409) {
      const head = parseHead(r.current);
      this.conflict = this.conflictOf(head && !head.deleted ? "changed" : "deleted", head);
      this.abandonAfterSave(`${this.host()} has a newer revision (conflict)`);
      return;
    }
    if (status === 423) {
      this.lockReason = text(r.detail) || "the robot has this program loaded";
      this.abandonAfterSave(`${this.host()} refused the save: ${this.lockReason}`);
      return;
    }
    this.error = text(r.detail) || `the cell refused the save (HTTP ${status})`;
    this.blockedKey = sending.doc.key;
    this.abandonAfterSave(this.error);
  }

  private onFetch(status: number, r: Record<string, unknown>): void {
    const want = this.wantFetch;
    this.wantFetch = null;
    if (!want) return;
    const head = parseHead(r.program);
    if (status === 200 && head) {
      this.remote = { head, document: r.document, force: want.force };
      return;
    }
    if (status === 404) this.notice(`${want.programId} is no longer on ${this.host()}`);
    else this.notice(`Could not open the program from ${this.host()}: ${text(r.detail) || `HTTP ${status}`}`);
  }

  private onCurrent(status: number, r: Record<string, unknown>): void {
    const wanted = this.wantCurrent;
    this.wantCurrent = null;
    if (status === 200) {
      if (this.store && wanted) this.store.current = wanted;
      return;
    }
    this.notice(`${this.host()}'s current program did not change: ${text(r.detail) || `HTTP ${status}`}`);
    // A refused switch must not open the program as if it were current.
    if (this.wantFetch && this.wantFetch.programId === wanted && this.options.followCurrent) this.wantFetch = null;
  }

  view(now: number): CellSyncView {
    const host = this.host();
    const store = this.store;
    const link = this.link;
    const current = store && store.current ? store.programs.find((p) => p.programId === store.current) ?? null : null;
    const others = store && link
      ? store.presence.filter((p) => !p.you && p.programId === link.programId)
      : [];
    const editors = others.map((p) => ({ name: clientName(p.label), editing: p.editing }));
    const editorsText = editors.map((e) => `${e.name} ${e.editing ? "editing" : "viewing"}`).join(", ");
    const base: CellSyncView = {
      state: "unknown", label: "", short: "", detail: "", host, programId: link?.programId ?? this.local?.programId ?? "",
      revision: link?.baseRevision ?? 0, dirty: Boolean(link?.dirty), editors, editorsText, conflict: null, offerUpload: false,
      current: current ? { programId: current.programId, name: current.name } : null,
      isCurrent: Boolean(link && store && store.current === link.programId), locked: this.lockReason, programs: store?.programs ?? [],
    };
    const upper = host.toUpperCase();
    const rev = link ? link.baseRevision : 0;
    if (this.enabled === null && this.online !== false) {
      return { ...base, state: "unknown", label: "connecting to the cell…", short: "…", detail: "Asking the OLP server for its program store" };
    }
    if (this.enabled === false) {
      return { ...base, state: "disabled", label: "local only", short: "LOCAL",
        detail: "This OLP server keeps no cell program store: the program is saved on this client only" };
    }
    if (this.conflict) {
      const c = this.conflict;
      const what = c.kind === "deleted"
        ? `${host} no longer has this program${c.by ? ` (deleted by ${c.by})` : ""}`
        : `${host} has rev ${c.revision}, saved by ${c.by || "another client"}${c.at ? ` ${ago(c.at, now)}` : ""}`;
      const textOf = `${what}; your edits from rev ${rev} are kept here. Reload theirs, or keep yours as a copy.`;
      return { ...base, state: "conflict", label: `conflict · ${what}`, short: "CONFLICT", detail: textOf, conflict: { ...c, text: textOf } };
    }
    if (!link) {
      if (this.online === false) {
        return { ...base, state: "offline", label: `local only · ${host} unreachable`, short: "LOCAL", detail: `${host} cannot be reached; the program is saved here` };
      }
      return { ...base, state: "local", label: `local only · not on ${host}`, short: "LOCAL", offerUpload: Boolean(store),
        detail: `This program is saved on this client only. Upload it to ${host} to share it with every client of the robot.` };
    }
    if (this.online === false) {
      const edits = link.dirty ? " + local edits" : "";
      return { ...base, state: "offline", label: `local draft · ${host} unreachable · rev ${rev}${edits}`, short: "LOCAL DRAFT",
        detail: `${host} cannot be reached. Edits are kept here and are saved to ${host} when it answers again, unless another client saved first.` };
    }
    if (this.lockReason && link.dirty) {
      return { ...base, state: "locked", label: `local draft · ${this.lockReason}`, short: "DRAFT · RUNNING",
        detail: `The robot has this program; ${host} refuses edits to it until Stop. Your edits are kept here and saved after Stop.` };
    }
    if (this.error && link.dirty) {
      return { ...base, state: "error", label: `not saved to ${host} · ${this.error}`, short: "NOT SAVED", detail: this.error };
    }
    if (this.wantFetch || this.remote) {
      return { ...base, state: "updating", label: `updating from ${host}…`, short: `${upper} · …`, detail: `Opening the cell's latest revision` };
    }
    if (link.dirty || this.sending) {
      return { ...base, state: "saving", label: `saving to ${host} · rev ${rev} + edits`, short: `${upper} · SAVING`,
        detail: `Edits are being saved to ${host}` };
    }
    return { ...base, state: "synced", label: `synced with ${host} · rev ${rev}`, short: `${upper} · R${rev}`,
      detail: `This program is ${host}'s revision ${rev}${base.isCurrent ? ", the cell's current program" : ""}` };
  }
}

/** Links in the browser's localStorage, one key per store and program. */
export class LocalStorageLinks implements CellLinkStorage {
  private readonly storage: Pick<Storage, "getItem" | "setItem" | "removeItem"> | null;
  constructor(storage: Pick<Storage, "getItem" | "setItem" | "removeItem"> | null) {
    this.storage = storage;
  }
  private key(storeId: string, programId: string): string {
    return `olp.cell-sync.v1:${storeId}:${programId}`;
  }
  get(storeId: string, programId: string): CellLink | null {
    try {
      const raw = this.storage?.getItem(this.key(storeId, programId));
      const r = raw ? record(JSON.parse(raw)) : null;
      if (!r || r.storeId !== storeId || r.programId !== programId) return null;
      return { storeId, programId, baseRevision: count(r.baseRevision), dirty: r.dirty === true, syncedKey: text(r.syncedKey) };
    } catch {
      return null;
    }
  }
  put(link: CellLink): void {
    try { this.storage?.setItem(this.key(link.storeId, link.programId), JSON.stringify(link)); } catch { /* kept for the session */ }
  }
  remove(storeId: string, programId: string): void {
    try { this.storage?.removeItem(this.key(storeId, programId)); } catch { /* nothing kept */ }
  }
}

/** Links in memory, exported whole for the Deck to keep in a file. */
export class MemoryLinks implements CellLinkStorage {
  links: { [key: string]: CellLink } = {};
  changed = false;
  get(storeId: string, programId: string): CellLink | null {
    const link = this.links[`${storeId}:${programId}`];
    return link ? { ...link } : null;
  }
  put(link: CellLink): void {
    this.links[`${link.storeId}:${link.programId}`] = { ...link };
    this.changed = true;
  }
  remove(storeId: string, programId: string): void {
    delete this.links[`${storeId}:${programId}`];
    this.changed = true;
  }
}
