import type { OfflineProgrammingProject } from "../domain/project.ts";
import { assertProject, cloneProject } from "../domain/project.ts";
import { migrateProject } from "../domain/serialization.ts";

// Stored records can predate whole blocks of the current schema — projects
// saved before the Robot V4 `program` object exists have no `program` at all,
// and reading one back raw crashes boot on the first `project.program` access.
// Hydrating at the storage boundary keeps every read path on the current shape;
// a record too broken to hydrate is dropped so callers fall back to a new one.
function hydrateStored(value: OfflineProgrammingProject): OfflineProgrammingProject | null {
  try {
    return migrateProject(value);
  } catch {
    return null;
  }
}

export interface ProjectSummary {
  id: string;
  name: string;
  revision: number;
  updatedAt: string;
  robotModel: string;
  planState: string;
}

/** A STEP this browser has imported, kept so a project reopened later can be
 *  worked on without the file again.
 *
 *  It is here because nothing else has it. The weld planner used to keep a
 *  copy in its workspace and rails asked for it by hash on reopen; D24 deleted
 *  that workspace, and the seam worker resolves nothing from a hash by design
 *  -- the bytes travel in every request. So the OLP owns the file, which is
 *  what "the OLP owns the state" meant in the first place.
 *
 *  Keyed by fingerprint, which for an imported STEP is its sha256, so two
 *  projects cut from one part share one copy. */
export interface StoredSource {
  fingerprint: string;
  filename: string;
  bytes: Uint8Array;
  storedAt: string;
}

/** The dense joint trajectory a plan produced -- the bytes the robot plays.
 *
 *  Kept here rather than left on the motion server, whose dense store is a
 *  cache under `~/.cache` and owes nobody a copy later. Keyed by the
 *  trajectory digest, which is the only name a blob has: the daemon fences on
 *  it and the motion server serves it by it, so there is no second id to keep
 *  in step. */
export interface StoredTrajectory {
  digest: string;
  bytes: Uint8Array;
  storedAt: string;
  /** Which program this came out of, and when it was planned. A trajectory
   *  read back without them can only be checked against whatever is open,
   *  which is not the same question as "is this still the plan for this
   *  program?" -- and after an edit the answer differs. */
  programDigest?: string;
  programId?: string;
  plannedAt?: string;
}

export interface PutTrajectory {
  digest: string;
  bytes: Uint8Array;
  programDigest?: string;
  programId?: string;
  plannedAt?: string;
}

/** One saved parameter preset.
 *
 *  Scoped by window -- `seam.search`, `weld.process` -- because two schemas
 *  with a field of the same name are still two different parameters, so the
 *  scope is part of the key rather than a filter applied after reading.
 *
 *  Values, never a reference: what a preset holds is copied out when it is
 *  loaded and copied in when it is saved, so editing one cannot reach back
 *  and change a weld that was already authored against it. */
export interface StoredPreset {
  scope: string;
  id: string;
  label: string;
  values: Record<string, number | string | boolean>;
  updatedAt: string;
}

export interface ProjectRepository {
  listPresets(scope: string): Promise<StoredPreset[]>;
  putPreset(preset: StoredPreset): Promise<void>;
  putTrajectory(trajectory: PutTrajectory): Promise<void>;
  getTrajectory(digest: string): Promise<StoredTrajectory | null>;
  putSource(source: { fingerprint: string; filename: string; bytes: Uint8Array }): Promise<void>;
  getSource(fingerprint: string): Promise<StoredSource | null>;
  /** Every kept STEP, newest first, WITHOUT bytes: the Import menu's recent
   *  parts list. `getSource` fetches the one chosen. */
  listSources(): Promise<Array<Omit<StoredSource, "bytes">>>;
  init(): Promise<void>;
  save(project: OfflineProgrammingProject): Promise<void>;
  get(projectId: string): Promise<OfflineProgrammingProject | null>;
  list(): Promise<ProjectSummary[]>;
  delete(projectId: string): Promise<void>;
  getRevision(projectId: string, revision: number): Promise<OfflineProgrammingProject | null>;
  listRevisions(projectId: string): Promise<number[]>;
}

export class MemoryProjectRepository implements ProjectRepository {
  #presets = new Map<string, StoredPreset>();
  #sources = new Map<string, StoredSource>();
  #trajectories = new Map<string, StoredTrajectory>();
  #projects = new Map<string, OfflineProgrammingProject>();
  #revisions = new Map<string, Map<number, OfflineProgrammingProject>>();

  async init(): Promise<void> {}

  async listPresets(scope: string): Promise<StoredPreset[]> {
    return [...this.#presets.values()].filter((preset) => preset.scope === scope);
  }

  async putPreset(preset: StoredPreset): Promise<void> {
    this.#presets.set(`${preset.scope}\u0000${preset.id}`, { ...preset });
  }

  async putTrajectory(trajectory: PutTrajectory): Promise<void> {
    this.#trajectories.set(trajectory.digest, {
      ...trajectory,
      bytes: trajectory.bytes.slice(),
      storedAt: new Date().toISOString(),
    });
  }

  async getTrajectory(digest: string): Promise<StoredTrajectory | null> {
    const stored = this.#trajectories.get(digest);
    return stored ? { ...stored, bytes: stored.bytes.slice() } : null;
  }

  async putSource(source: { fingerprint: string; filename: string; bytes: Uint8Array }): Promise<void> {
    this.#sources.set(source.fingerprint, {
      ...source,
      bytes: source.bytes.slice(),
      storedAt: new Date().toISOString(),
    });
  }

  async getSource(fingerprint: string): Promise<StoredSource | null> {
    const stored = this.#sources.get(fingerprint);
    return stored ? { ...stored, bytes: stored.bytes.slice() } : null;
  }

  async listSources(): Promise<Array<Omit<StoredSource, "bytes">>> {
    return [...this.#sources.values()]
      .map(({ fingerprint, filename, storedAt }) => ({ fingerprint, filename, storedAt }))
      .sort((a, b) => b.storedAt.localeCompare(a.storedAt));
  }

  async save(project: OfflineProgrammingProject): Promise<void> {
    assertProject(project);
    const copy = cloneProject(project);
    const revisions = this.#revisions.get(copy.identity.id) ?? new Map<number, OfflineProgrammingProject>();
    const existing = revisions.get(copy.identity.revision);
    if (
      existing &&
      JSON.stringify(existing) !== JSON.stringify(copy) &&
      !sameAuthoredRevision(existing, copy)
    ) {
      throw new Error(`revision ${copy.identity.revision} for ${copy.identity.id} is immutable`);
    }
    revisions.set(copy.identity.revision, copy);
    this.#revisions.set(copy.identity.id, revisions);
    const current = this.#projects.get(copy.identity.id);
    if (!current || copy.identity.revision >= current.identity.revision) {
      this.#projects.set(copy.identity.id, copy);
    }
  }

  async get(projectId: string): Promise<OfflineProgrammingProject | null> {
    const value = this.#projects.get(projectId);
    return value ? cloneProject(value) : null;
  }

  async list(): Promise<ProjectSummary[]> {
    return [...this.#projects.values()]
      .sort((left, right) => right.identity.updatedAt.localeCompare(left.identity.updatedAt))
      .map(summary);
  }

  async delete(projectId: string): Promise<void> {
    this.#projects.delete(projectId);
    this.#revisions.delete(projectId);
  }

  async getRevision(projectId: string, revision: number): Promise<OfflineProgrammingProject | null> {
    const value = this.#revisions.get(projectId)?.get(revision);
    return value ? cloneProject(value) : null;
  }

  async listRevisions(projectId: string): Promise<number[]> {
    return [...(this.#revisions.get(projectId)?.keys() ?? [])].sort((left, right) => left - right);
  }
}

type RevisionRecord = {
  projectId: string;
  revision: number;
  project: OfflineProgrammingProject;
};

export class IndexedDbProjectRepository implements ProjectRepository {
  readonly databaseName: string;
  readonly factory: IDBFactory | null;
  #db: IDBDatabase | null = null;
  #fallback = new MemoryProjectRepository();
  #initialized = false;

  constructor(options: { databaseName?: string; factory?: IDBFactory | null } = {}) {
    this.databaseName = options.databaseName ?? "offline-programming-v1";
    this.factory = options.factory === undefined
      ? (typeof indexedDB === "undefined" ? null : indexedDB)
      : options.factory;
  }

  get usingMemoryFallback(): boolean {
    return this.#initialized && this.#db === null;
  }

  async init(): Promise<void> {
    if (this.#initialized) return;
    await this.#fallback.init();
    if (this.factory) {
      try {
        this.#db = await withTimeout(
          openDatabase(this.factory, this.databaseName),
          2_500,
          "IndexedDB open timed out",
        );
      } catch {
        this.#db = null;
      }
    }
    this.#initialized = true;
  }

  async putTrajectory(trajectory: PutTrajectory): Promise<void> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.putTrajectory(trajectory);
    try {
      const transaction = this.#db.transaction("trajectories", "readwrite");
      transaction.objectStore("trajectories").put({
        ...trajectory,
        bytes: trajectory.bytes.slice(),
        storedAt: new Date().toISOString(),
      } satisfies StoredTrajectory);
      await transactionDone(transaction);
    } catch {
      this.#disableDatabase();
      await this.#fallback.putTrajectory(trajectory);
    }
  }

  async getTrajectory(digest: string): Promise<StoredTrajectory | null> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.getTrajectory(digest);
    try {
      const value = await requestResult<StoredTrajectory | undefined>(
        this.#db.transaction("trajectories", "readonly").objectStore("trajectories").get(digest),
      );
      return value ?? null;
    } catch {
      this.#disableDatabase();
      return this.#fallback.getTrajectory(digest);
    }
  }

  async listPresets(scope: string): Promise<StoredPreset[]> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.listPresets(scope);
    try {
      const store = this.#db.transaction("presets", "readonly").objectStore("presets");
      return await requestResult<StoredPreset[]>(store.index("by-scope").getAll(scope));
    } catch {
      this.#disableDatabase();
      return this.#fallback.listPresets(scope);
    }
  }

  async putPreset(preset: StoredPreset): Promise<void> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.putPreset(preset);
    try {
      const transaction = this.#db.transaction("presets", "readwrite");
      transaction.objectStore("presets").put({ ...preset } satisfies StoredPreset);
      await transactionDone(transaction);
    } catch (error) {
      this.#disableDatabase();
      await this.#fallback.putPreset(preset);
      // Rethrown after the fallback took it: the preset survives this
      // session, and the caller must be able to say it did not survive the
      // browser. A Save that reports success and kept nothing is how an
      // operator loses a procedure they believe is written down.
      throw error;
    }
  }

  async putSource(source: { fingerprint: string; filename: string; bytes: Uint8Array }): Promise<void> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.putSource(source);
    try {
      const transaction = this.#db.transaction("sources", "readwrite");
      // A copy, because the caller keeps using the array it handed over and a
      // structured clone of a live view would race with whatever writes it.
      transaction.objectStore("sources").put({
        fingerprint: source.fingerprint,
        filename: source.filename,
        bytes: source.bytes.slice(),
        storedAt: new Date().toISOString(),
      } satisfies StoredSource);
      await transactionDone(transaction);
    } catch {
      this.#disableDatabase();
      await this.#fallback.putSource(source);
    }
  }

  async getSource(fingerprint: string): Promise<StoredSource | null> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.getSource(fingerprint);
    try {
      const value = await requestResult<StoredSource | undefined>(
        this.#db.transaction("sources", "readonly").objectStore("sources").get(fingerprint),
      );
      return value ?? null;
    } catch {
      this.#disableDatabase();
      return this.#fallback.getSource(fingerprint);
    }
  }

  async listSources(): Promise<Array<Omit<StoredSource, "bytes">>> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.listSources();
    try {
      const values = await requestResult<StoredSource[]>(
        this.#db.transaction("sources", "readonly").objectStore("sources").getAll(),
      );
      return values
        .map(({ fingerprint, filename, storedAt }) => ({ fingerprint, filename, storedAt }))
        .sort((a, b) => b.storedAt.localeCompare(a.storedAt));
    } catch {
      this.#disableDatabase();
      return this.#fallback.listSources();
    }
  }

  async save(project: OfflineProgrammingProject): Promise<void> {
    assertProject(project);
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.save(project);
    const existing = await this.getRevision(project.identity.id, project.identity.revision);
    if (
      existing &&
      JSON.stringify(existing) !== JSON.stringify(project) &&
      !sameAuthoredRevision(existing, project)
    ) {
      throw new Error(`revision ${project.identity.revision} for ${project.identity.id} is immutable`);
    }
    if (!this.#db) return this.#fallback.save(project);
    try {
      const transaction = this.#db.transaction(["projects", "revisions"], "readwrite");
      transaction.objectStore("projects").put(cloneProject(project));
      const record: RevisionRecord = {
        projectId: project.identity.id,
        revision: project.identity.revision,
        project: cloneProject(project),
      };
      transaction.objectStore("revisions").put(record);
      await transactionDone(transaction);
    } catch {
      this.#disableDatabase();
      await this.#fallback.save(project);
    }
  }

  async get(projectId: string): Promise<OfflineProgrammingProject | null> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.get(projectId);
    try {
      const value = await requestResult<OfflineProgrammingProject | undefined>(
        this.#db.transaction("projects", "readonly").objectStore("projects").get(projectId),
      );
      return value ? hydrateStored(value) : null;
    } catch {
      this.#disableDatabase();
      return this.#fallback.get(projectId);
    }
  }

  async list(): Promise<ProjectSummary[]> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.list();
    try {
      const values = await requestResult<OfflineProgrammingProject[]>(
        this.#db.transaction("projects", "readonly").objectStore("projects").getAll(),
      );
      return values
        .map(hydrateStored)
        .filter((value): value is OfflineProgrammingProject => value !== null)
        .sort((left, right) => right.identity.updatedAt.localeCompare(left.identity.updatedAt))
        .map(summary);
    } catch {
      this.#disableDatabase();
      return this.#fallback.list();
    }
  }

  async delete(projectId: string): Promise<void> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.delete(projectId);
    try {
      const transaction = this.#db.transaction(["projects", "revisions"], "readwrite");
      transaction.objectStore("projects").delete(projectId);
      const index = transaction.objectStore("revisions").index("by-project");
      const cursor = index.openCursor(IDBKeyRange.only(projectId));
      cursor.onsuccess = () => {
        const value = cursor.result;
        if (!value) return;
        value.delete();
        value.continue();
      };
      await transactionDone(transaction);
    } catch {
      this.#disableDatabase();
      await this.#fallback.delete(projectId);
    }
  }

  async getRevision(projectId: string, revision: number): Promise<OfflineProgrammingProject | null> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.getRevision(projectId, revision);
    try {
      const value = await requestResult<RevisionRecord | undefined>(
        this.#db.transaction("revisions", "readonly").objectStore("revisions").get([projectId, revision]),
      );
      return value ? hydrateStored(value.project) : null;
    } catch {
      this.#disableDatabase();
      return this.#fallback.getRevision(projectId, revision);
    }
  }

  async listRevisions(projectId: string): Promise<number[]> {
    await this.ensureInitialized();
    if (!this.#db) return this.#fallback.listRevisions(projectId);
    try {
      const values = await requestResult<RevisionRecord[]>(
        this.#db.transaction("revisions", "readonly").objectStore("revisions").index("by-project").getAll(projectId),
      );
      return values.map((value) => value.revision).sort((left, right) => left - right);
    } catch {
      this.#disableDatabase();
      return this.#fallback.listRevisions(projectId);
    }
  }

  private async ensureInitialized(): Promise<void> {
    if (!this.#initialized) await this.init();
  }

  #disableDatabase(): void {
    if (this.#db) {
      try {
        this.#db.close();
      } catch {
        // Ignore close races while abandoning a wedged connection.
      }
    }
    this.#db = null;
  }
}

function openDatabase(factory: IDBFactory, name: string): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const request = factory.open(name, 3);
    request.onupgradeneeded = () => {
      const database = request.result;
      if (!database.objectStoreNames.contains("projects")) {
        database.createObjectStore("projects", { keyPath: "identity.id" });
      }
      // Version 2. A project reopened in a new session had no STEP: the weld
      // planner's workspace held the only other copy and that went with the
      // authoring server.
      if (!database.objectStoreNames.contains("sources")) {
        database.createObjectStore("sources", { keyPath: "fingerprint" });
      }
      if (!database.objectStoreNames.contains("trajectories")) {
        database.createObjectStore("trajectories", { keyPath: "digest" });
      }
      if (!database.objectStoreNames.contains("revisions")) {
        const revisions = database.createObjectStore("revisions", { keyPath: ["projectId", "revision"] });
        revisions.createIndex("by-project", "projectId", { unique: false });
      }
      // Version 3. Parameter presets the operator saved: not project state,
      // because a procedure worked out on one part is meant to be reachable
      // from the next one.
      if (!database.objectStoreNames.contains("presets")) {
        const presets = database.createObjectStore("presets", { keyPath: ["scope", "id"] });
        presets.createIndex("by-scope", "scope", { unique: false });
      }
    };
    request.onerror = () => reject(request.error ?? new Error("IndexedDB open failed"));
    request.onblocked = () => reject(new Error("IndexedDB upgrade blocked"));
    request.onsuccess = () => resolve(request.result);
  });
}

function withTimeout<T>(promise: Promise<T>, ms: number, message: string): Promise<T> {
  return new Promise<T>((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error(message)), ms);
    promise.then(
      (value) => {
        clearTimeout(timer);
        resolve(value);
      },
      (error) => {
        clearTimeout(timer);
        reject(error);
      },
    );
  });
}

function requestResult<T>(request: IDBRequest<T>): Promise<T> {
  return withTimeout(
    new Promise<T>((resolve, reject) => {
      request.onsuccess = () => resolve(request.result);
      request.onerror = () => reject(request.error ?? new Error("IndexedDB request failed"));
    }),
    2_500,
    "IndexedDB request timed out",
  );
}

function transactionDone(transaction: IDBTransaction): Promise<void> {
  return withTimeout(
    new Promise<void>((resolve, reject) => {
      transaction.oncomplete = () => resolve();
      transaction.onerror = () => reject(transaction.error ?? new Error("IndexedDB transaction failed"));
      transaction.onabort = () => reject(transaction.error ?? new Error("IndexedDB transaction aborted"));
    }),
    2_500,
    "IndexedDB transaction timed out",
  );
}

function summary(project: OfflineProgrammingProject): ProjectSummary {
  return {
    id: project.identity.id,
    name: project.identity.name,
    revision: project.identity.revision,
    updatedAt: project.identity.updatedAt,
    robotModel: project.robot.model,
    planState: project.plan.state,
  };
}

/**
 * Planning enriches an immutable authored revision with a server-owned plan.
 * The plan may be replaced by a later correlated planning attempt, but no
 * authored or program input at that revision may change.
 */
function sameAuthoredRevision(
  left: OfflineProgrammingProject,
  right: OfflineProgrammingProject,
): boolean {
  const leftAuthored = cloneProject(left);
  const rightAuthored = cloneProject(right);
  leftAuthored.plan = rightAuthored.plan;
  leftAuthored.denseTrajectory = rightAuthored.denseTrajectory;
  leftAuthored.planHistory = rightAuthored.planHistory;
  // Selecting a row or moving the preview cursor does not edit the program.
  leftAuthored.simulation = rightAuthored.simulation;
  // The description pin is provenance, not authorship: a program written
  // before the contract acquires one the first time it is opened, and that
  // is not an edit anybody made. Without this, opening such a program threw
  // "revision N is immutable" and the app never finished loading.
  leftAuthored.robot.description = rightAuthored.robot.description;
  return JSON.stringify(leftAuthored) === JSON.stringify(rightAuthored);
}
