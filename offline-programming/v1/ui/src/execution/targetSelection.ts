// Unverified presentation constant: operator-required selection-change wording.
export const TARGET_SELECTION_CHANGED = "Machine selection changed in another window; re-check before arming";

/** Retire a rejected browser identity immediately and refresh status once.
 * Requests carrying that old identity stay local; energising requests are never
 * replayed after refresh, preventing stale tabs from flooding the controller. */
export class TargetSelectionSync {
  readonly #fetch: typeof fetch;
  readonly #base: string;
  readonly #changed: (payload?: unknown) => void;
  readonly #stale = new Set<string>();
  #refresh: Promise<void> | null = null;
  constructor(fetcher: typeof fetch, base: string, changed: (payload?: unknown) => void) {
    this.#fetch = fetcher; this.#base = base; this.#changed = changed;
  }
  readonly fetch: typeof fetch = async (url, init) => {
    const headers = new Headers(init?.headers);
    const generation = headers.get("X-RT-Target-Generation");
    const cell = headers.get("X-RT-Target-Cell");
    // A request is worth refusing locally when it names a target identity (any
    // method: a stale identity's state reads stay local too) or when it is an
    // identity-less mutation (a preview tab's jog start). Identity-less reads
    // (the machine catalogue, capabilities, state) must always reach the
    // server: blocking them under the shared "[null,null]" key stranded a tab on
    // "catalogue could not be read" with no machine and no joint rows
    // (2026-09-16, LA bench).
    const identified = generation !== null && cell !== null;
    const mutation = (init?.method ?? "GET").toUpperCase() !== "GET";
    const blockable = identified || mutation;
    const key = JSON.stringify([generation, cell]);
    const status = String(url) === `${this.#base}/status`;
    const inhibition = String(url) === `${this.#base}/stop` || String(url) === `${this.#base}/pause` ||
      String(url).endsWith("/local-simulator/jog/halt") || String(url).endsWith("/local-simulator/jog/stop") ||
      (String(url) === `${this.#base}/arm` && typeof init?.body === "string" && JSON.parse(init.body).armed === false);
    if (blockable && !status && !inhibition && this.#stale.has(key)) return Response.json({ error: "target_changed", detail: TARGET_SELECTION_CHANGED }, { status: 409 });
    const response = await this.#fetch(url, init);
    if (blockable && !status && response.status === 409) {
      const body = await response.clone().json().catch(() => null);
      if (body?.error === "target_changed" && !this.#stale.has(key)) {
        this.#stale.add(key);
        this.#changed();
        this.#refresh ??= this.#resync().finally(() => { this.#refresh = null; });
        await this.#refresh;
      }
    }
    return response;
  };
  async #resync(): Promise<void> {
    try {
      const response = await this.#fetch(`${this.#base}/status`, { method: "GET", credentials: "same-origin", cache: "no-store", redirect: "error" });
      if (response.ok) this.#changed(await response.json());
    } catch { /* Normal status polling can recover; the rejected identity stays blocked. */ }
  }
}
