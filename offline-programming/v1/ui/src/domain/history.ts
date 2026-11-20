import {
  type OfflineProgrammingProject,
  cloneProject,
  restoreAsRevision,
} from "./project.ts";

export class ProjectHistory {
  readonly limit: number;
  #past: OfflineProgrammingProject[] = [];
  #future: OfflineProgrammingProject[] = [];

  constructor(limit = 80) {
    this.limit = Math.max(1, Math.floor(limit));
  }

  get canUndo(): boolean {
    return this.#past.length > 0;
  }

  get canRedo(): boolean {
    return this.#future.length > 0;
  }

  push(previous: OfflineProgrammingProject): void {
    this.#past.push(cloneProject(previous));
    if (this.#past.length > this.limit) this.#past.shift();
    this.#future = [];
  }

  undo(current: OfflineProgrammingProject, timestamp?: string): OfflineProgrammingProject {
    const snapshot = this.#past.pop();
    if (!snapshot) return current;
    this.#future.push(cloneProject(current));
    return restoreAsRevision(current, snapshot, "undo", timestamp);
  }

  redo(current: OfflineProgrammingProject, timestamp?: string): OfflineProgrammingProject {
    const snapshot = this.#future.pop();
    if (!snapshot) return current;
    this.#past.push(cloneProject(current));
    return restoreAsRevision(current, snapshot, "redo", timestamp);
  }

  clear(): void {
    this.#past = [];
    this.#future = [];
  }
}

