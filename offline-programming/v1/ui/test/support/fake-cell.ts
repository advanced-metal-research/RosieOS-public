import type { OfflineProgrammingProject } from "../../src/domain/project.ts";

export const DECK = { id: "deck-1", label: "rosie-pendant-v5 (steamdeck-operator)", kind: "pendant", address: "127.0.0.1" };
export const TAB = { id: "tab-1", label: "Browser (Chrome on macOS) on http://127.0.0.1:5189", kind: "browser", address: "127.0.0.1" };

type Client = typeof TAB;

/** The cell's store as cell_programs.go answers, in memory: enough to drive
 *  the browser through its real fetch path. `offline` makes every request
 *  fail as an unreachable host does; `disabled` answers as a server started
 *  without --program-store. */
export class FakeCell {
  version = 1;
  programs = new Map<string, { revision: number; document: unknown; by: Client; name: string }>();
  current: string | null = null;
  offline = false;
  disabled = false;
  /** Other clients' open programs, beside the watching client's own entry. */
  presence: { program_id: string; id: string; label: string; kind: string; address: string; editing: boolean }[] = [];
  private waiters: (() => void)[] = [];

  bump(): void {
    this.version++;
    for (const wake of this.waiters.splice(0)) wake();
  }

  head(id: string) {
    const p = this.programs.get(id)!;
    return { program_id: id, revision: p.revision, name: p.name, robot_model: "rosie_1400_v3", updated_at: "2026-10-11T18:00:00Z", updated_by: p.by };
  }

  write(id: string, base: number, document: unknown, by: Client): [number, unknown] {
    const existing = this.programs.get(id);
    if ((existing?.revision ?? 0) !== base) {
      return [409, { ok: false, error: "program_conflict", detail: "stale", current: existing ? this.head(id) : null }];
    }
    const name = (document as OfflineProgrammingProject).identity.name;
    this.programs.set(id, { revision: base + 1, document, by, name });
    this.bump();
    return [200, { ok: true, program: this.head(id) }];
  }

  view(watch: string) {
    const body: Record<string, unknown> = {
      schema: "offline-programming.cell-programs.v1", enabled: true,
      store: { id: "store-rosie_1420_v1", instance: "i1", label: "example-relay" }, version: this.version,
      current: this.current ? { program_id: this.current, set_by: DECK } : null, executing: null,
      programs: [...this.programs.keys()].map((id) => this.head(id)),
      presence: [{ program_id: watch, ...DECK, editing: true }, ...this.presence],
    };
    if (watch) body.watched = this.programs.has(watch) ? this.head(watch) : null;
    return body;
  }

  fetch: typeof fetch = async (input, init) => {
    if (this.offline) throw new TypeError("Failed to fetch");
    if (this.disabled) {
      return new Response(JSON.stringify({ error: "program_store_disabled", detail: "no store" }), { status: 503, headers: { "Content-Type": "application/json" } });
    }
    const url = new URL(String(input), "http://olp");
    const path = url.pathname.replace("/api/offline-programming/v1/cell-programs", "");
    let status = 200;
    let body: unknown;
    if (path === "/changes") {
      // Held like the server holds it: until something changes.
      if (Number(url.searchParams.get("since")) === this.version && Number(url.searchParams.get("timeout_ms")) > 0) {
        await new Promise<void>((wake, fail) => {
          this.waiters.push(wake);
          init?.signal?.addEventListener("abort", () => fail(new Error("aborted")));
        });
      }
      if (this.offline) throw new TypeError("Failed to fetch");
      body = this.view(url.searchParams.get("watch") ?? "");
    }
    else if (path.endsWith("/save")) {
      const id = decodeURIComponent(path.split("/")[2]!);
      const request = JSON.parse(String(init?.body));
      [status, body] = this.write(id, request.base_revision, request.document, TAB);
      if (status === 200 && request.make_current) this.current = id;
    } else if (path === "/current") {
      this.current = JSON.parse(String(init?.body)).program_id;
      this.bump();
      body = { ok: true };
    } else if (path.startsWith("/programs/")) {
      const id = decodeURIComponent(path.split("/")[2]!);
      body = { program: this.head(id), document: this.programs.get(id)!.document };
    }
    return new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } });
  };
}

/** localStorage in memory: the browser's sync links survive a program switch. */
export class MemoryStorage {
  private values = new Map<string, string>();
  get length(): number { return this.values.size; }
  clear(): void { this.values.clear(); }
  key(index: number): string | null { return [...this.values.keys()][index] ?? null; }
  getItem(key: string): string | null { return this.values.get(key) ?? null; }
  setItem(key: string, value: string): void { this.values.set(key, value); }
  removeItem(key: string): void { this.values.delete(key); }
}
