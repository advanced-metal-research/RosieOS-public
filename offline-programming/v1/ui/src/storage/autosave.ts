import type { OfflineProgrammingProject } from "../domain/project.ts";
import type { ProjectRepository } from "./repository.ts";

export class AutosaveController {
  readonly delayMS: number;
  #repository: ProjectRepository;
  #timer: ReturnType<typeof setTimeout> | null = null;
  #pending: OfflineProgrammingProject | null = null;
  #onError: (error: unknown) => void;
  #onSaved: () => void;

  constructor(
    repository: ProjectRepository,
    delayMS = 450,
    onError: (error: unknown) => void = () => {},
    onSaved: () => void = () => {},
  ) {
    this.#repository = repository;
    this.delayMS = Math.max(0, delayMS);
    this.#onError = onError;
    this.#onSaved = onSaved;
  }

  schedule(project: OfflineProgrammingProject): void {
    this.#pending = structuredClone(project);
    if (this.#timer) clearTimeout(this.#timer);
    this.#timer = setTimeout(() => {
      this.#timer = null;
      void this.flush().catch(() => {}); // flush reports scheduled and explicit failures.
    }, this.delayMS);
  }

  async flush(): Promise<void> {
    if (this.#timer) {
      clearTimeout(this.#timer);
      this.#timer = null;
    }
    const project = this.#pending;
    this.#pending = null;
    if (project) {
      try {
        await this.#repository.save(project);
        if (!this.#pending) this.#onSaved();
      } catch (error) {
        this.#onError(error);
        throw error;
      }
    }
  }

  cancel(): void {
    if (this.#timer) clearTimeout(this.#timer);
    this.#timer = null;
    this.#pending = null;
  }
}
