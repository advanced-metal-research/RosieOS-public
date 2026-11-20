// The pendant's program is the cell's current program: the same sync state
// machine the browser runs (OLP's storage/cell-sync.ts), with followCurrent,
// applied to this session's project. The Qt side (src/program_sync.cpp) only
// performs the requests this returns, hands every answer back, keeps the links
// in a file beside current.olp.json and draws the view.
//
// Bundled for Qt 5.15's engine (ES2016): no padStart, no Object.entries, no
// `#private` members.
import {
  cellDocument, CellProgramSync, MemoryLinks, programSwitchLock,
  type CellLink, type CellOpenCheck, type CellOpenChoice, type CellPicker, type CellRequest, type CellSyncView,
} from "../../../../offline-programming/v1/ui/src/storage/cell-sync.ts";
import { copyOpenProject, currentProject, openCellProject } from "./session.ts";

let links = new MemoryLinks();
let sync: CellProgramSync | null = null;

function engine(): CellProgramSync {
  if (!sync) throw new Error("cell sync is not started");
  return sync;
}

function hasProject(): boolean {
  try {
    currentProject();
    return true;
  } catch {
    return false;
  }
}

/** Start with this pendant's client id and the links kept from its last run
 *  (the file's `links` object, or null). */
export function cellSyncStart(clientTag: string, kept: unknown): void {
  links = new MemoryLinks();
  const r = kept && typeof kept === "object" ? kept as { [key: string]: unknown } : {};
  for (const key in r) {
    const link = r[key] as Partial<CellLink> | null;
    if (link && typeof link.storeId === "string" && typeof link.programId === "string") {
      links.links[key] = {
        storeId: link.storeId, programId: link.programId, baseRevision: Number(link.baseRevision) || 0,
        dirty: link.dirty === true, syncedKey: typeof link.syncedKey === "string" ? link.syncedKey : "",
      };
    }
  }
  sync = new CellProgramSync(links, { clientTag, followCurrent: true });
}

export interface CellSyncTick {
  requests: CellRequest[];
  view: CellSyncView;
  /** The links to keep, when they changed since the last tick; else null. */
  links: { [key: string]: CellLink } | null;
  /** Ask the operator (first start, a new cell): the cell's saved programs
   *  to open, upload this Deck's program, or start a new one. */
  offer: CellPicker | null;
  notices: string[];
}

/** Every timer tick: what to send, what to draw, what to keep, what to ask. */
export function cellSyncTick(now: number): CellSyncTick {
  const s = engine();
  if (hasProject()) s.observe(cellDocument(currentProject()), now);
  const requests = hasProject() ? s.requests(now) : [];
  const changed = links.changed;
  links.changed = false;
  return { requests, view: s.view(now), links: changed ? links.links : null, offer: s.takeOffer(now), notices: s.takeNotices() };
}

/** The answer to a request; `opened` is the message when a program from the
 *  cell replaced the open one (the Qt side refreshes and autosaves). */
export function cellSyncReply(id: number, status: number, body: unknown, now: number): { opened: string; error: string } {
  const s = engine();
  s.reply(id, status, body, now);
  const remote = s.takeRemote();
  if (!remote) return { opened: "", error: "" };
  try {
    const project = openCellProject(remote.document);
    s.applied(remote.head, cellDocument(project));
    return { opened: `Opened ${remote.head.name} · rev ${remote.head.revision} from ${s.view(now).host}`, error: "" };
  } catch (error) {
    return { opened: "", error: `The cell's revision ${remote.head.revision} of ${remote.head.name} could not be opened: ${error instanceof Error ? error.message : String(error)}` };
  }
}

export function cellSyncView(now: number): CellSyncView {
  return engine().view(now);
}

/** Conflict: "theirs" drops the edits made here and opens the cell's
 *  revision; "copy" keeps them as a new program, which becomes the cell's
 *  current one (the pendant shows the current program). */
export function cellSyncResolve(choice: string, now: number): string {
  const s = engine();
  if (choice === "theirs") {
    s.resolveTheirs();
    return "Reloading the cell's revision";
  }
  if (choice === "copy") {
    const copy = copyOpenProject();
    s.resolveCopy(cellDocument(copy), true, now);
    return `Your edits are now "${copy.identity.name}" on the cell`;
  }
  throw new Error(`unknown conflict choice ${choice}`);
}

/** Put the open program on the cell and make it current (a new program, an
 *  opened file, the startup question). */
export function cellSyncUpload(now: number): void {
  const s = engine();
  s.observe(cellDocument(currentProject()), now);
  s.upload(true, now);
}

/** The cell's saved programs for the picker, newest save first; `runLock`
 *  (cellSyncRunLock) refuses every row while the robot runs a program. */
export function cellSyncPicker(now: number, runLock = ""): CellPicker {
  return engine().picker(now, runLock);
}

/** OPEN: refused and why, already open, or the open program's draft the
 *  operator answers first (save, discard, cancel: the Deck's one program
 *  file keeps nothing else). */
export function cellSyncOpenCheck(programId: string, now: number, runLock = ""): CellOpenCheck {
  return engine().openCheck(programId, now, runLock);
}

/** Open a cell program with the operator's answer ("open", "save" or
 *  "discard"); it becomes the cell's current program. Returns what to tell
 *  the operator; throws the refusal. */
export function cellSyncOpenAfter(programId: string, choice: string, now: number, runLock = ""): string {
  if (choice !== "open" && choice !== "save" && choice !== "discard") throw new Error(`unknown open choice ${choice}`);
  const s = engine();
  if (hasProject()) s.observe(cellDocument(currentProject()), now);
  return s.openAfter(programId, choice as CellOpenChoice, now, runLock);
}

/** Why the Deck must not switch programs now, from its runView: a session
 *  loaded, staged, playing or paused, a loop, or its Run press in flight
 *  (cell-sync.ts programSwitchLock, the browser's words). */
export function cellSyncRunLock(run: { controls?: { state?: string }; looping?: boolean; session?: unknown } | null, pressing: boolean): string {
  return programSwitchLock({
    sessionState: run && run.session ? run.controls?.state ?? "" : "",
    looping: Boolean(run && run.looping),
    busy: pressing ? "the Run program press is still running" : "",
  });
}

/**
 * The picker's choice: "open" a cell program (it becomes the cell's current
 * program; refused, with the reason, while the robot runs another), "upload"
 * this Deck's program as current, or "new" (the Qt side asks for a name and
 * uploads it). Returns what to tell the operator.
 */
export function cellSyncPick(action: string, programId: string, now: number): string {
  const s = engine();
  const picker = s.picker(now);
  if (action === "open") {
    const row = picker.rows.find((r) => r.programId === programId);
    if (!row) throw new Error(`${programId} is no longer on ${picker.host}`);
    if (row.blocked) throw new Error(row.blocked);
    // The question itself offered uploading this Deck's program: opening
    // one of the cell's instead is the operator's answer to that draft.
    const check = s.openCheck(programId, now);
    s.openAfter(programId, check.draft ? "discard" : "open", now);
    return `Opening ${row.name} from ${picker.host}`;
  }
  if (action === "upload") {
    cellSyncUpload(now);
    return `Putting ${picker.local?.name ?? "this program"} on ${picker.host} as its current program`;
  }
  if (action === "new") return "";
  throw new Error(`unknown picker action ${action}`);
}
